'''swingplanner.py

   Two-handed sword grip planner for the G1 (23 DOF) + sword URDF.

   The sword is rigidly attached to the right hand in the URDF.  The left
   hand must grip the handle just below the right hand.  As each wrist only
   has a roll joint, the handle must stay perpendicular to both forearms,
   which makes the grip a non-trivial inverse kinematics problem.

   We parameterize the vertical swing by the blade angle 'th' (degrees,
   measured from vertical, positive = tilted forward, rotation about the
   world y axis).  For each blade angle we solve (bounded least squares)
   for the handle position (x,z in the sagittal plane) and the 10 arm
   joints so that:
     - the right hand carries the handle origin to the handle point with
       the handle axis pointing along the blade direction,
     - the left hand's grip point sits on the handle 6.5cm below the
       right hand's, with its z axis along the handle,
     - the elbows stay out of the chest,
     - the handle follows a nominal path (softly).
   We solve with continuation (each angle warm-started from the previous),
   so the resulting joint table is continuous.

   plan = SwingPlanner(urdf_xml)
   table = plan.solve(thetas)       -> (thetas, Q) with Q[i] the full joint vector
'''

import numpy as np
from scipy.optimize import least_squares

from karate.g1kin import Kinematics, Rotn


# Arm joint names (5 per arm).
def armjoints(side):
    return [f'{side}_shoulder_pitch_joint', f'{side}_shoulder_roll_joint',
            f'{side}_shoulder_yaw_joint',   f'{side}_elbow_joint',
            f'{side}_wrist_roll_joint']


# Grip geometry (meters), measured from the rubber hand meshes.
GRIP = {
    # Handle origin expressed in the right-hand frame (matches the URDF
    # joint 'right_hand_to_sword').
    'right': np.array([0.20,  0.005, -0.04]),
    # Left fist center in the left-hand frame (mirror of the right).
    'left':  np.array([0.20, -0.005,  0.00]),
}
# Where the left fist sits along the handle (handle frame z).
LEFT_ON_HANDLE = -0.065

# Ready stance: blade 30 deg forward, handle at (x,z) = (0.32, 0.95).
READY_THETA  = 30.0
READY_HANDLE = np.array([0.32, 0.95])

# Seed joint values (pitch, roll, yaw, elbow, wrist) for the ready stance.
READY_SEED = {
    'right': np.array([-1.31, -0.81,  1.51, 0.05,  1.93]),
    'left':  np.array([-0.74,  0.70, -1.22, 0.02, -1.41]),
}


def Roty(a):
    return Rotn([0, 1, 0], a)


def nominal_handle(th):
    '''Nominal handle path (x,z) and how strongly to follow it.'''
    if th <= READY_THETA:
        # Raising: move the hands up and slightly back, above the forehead.
        s = (READY_THETA - th) / 70.0
        return np.array([0.32 - 0.08*s, 0.95 + 0.32*s]), 1.0
    else:
        # Striking: the roll-only wrists force the hands to come back
        # toward the body as the blade tips forward, so follow loosely.
        return READY_HANDLE.copy(), 0.05


class SwingPlanner:
    def __init__(self, urdf_xml):
        self.kin = Kinematics(urdf_xml)
        K = self.kin
        self.idx  = {s: [K.index[n] for n in armjoints(s)] for s in ('right', 'left')}
        self.AI   = self.idx['right'] + self.idx['left']
        # Variables: handle (x,z), then right arm (5), then left arm (5).
        self.lo = np.r_[0.05, 0.70, K.qmin[self.AI]]
        self.hi = np.r_[0.60, 1.60, K.qmax[self.AI]]

    def full(self, v):
        q = np.zeros(len(self.kin.jointnames))
        q[self.AI] = v[2:]
        return q

    def armerr(self, q, side, pg, a):
        K = self.kin
        T, _ = K.fkin(q, f'{side}_wrist_roll_rubber_hand')
        pgrip = T[:3, :3] @ GRIP[side] + T[:3, 3]
        # Keep the elbow out of the chest (in torso frame).
        Te, _ = K.fkin(q, f'{side}_elbow_link')
        Tt, _ = K.fkin(q, 'torso_link')
        d = Tt[:3, :3].T @ (Te[:3, 3] - Tt[:3, 3])
        pen = max(0.0, 0.16 - abs(d[1])) * (d[0] < 0.15)
        return np.r_[pg - pgrip, 0.3*(a - T[:3, 2]), 2*pen]

    def residual(self, v, th, vprev):
        q = self.full(v)
        p = np.array([v[0], 0.0, v[1]])
        a = Roty(np.radians(th))[:, 2]                  # blade direction
        pnom, wp = nominal_handle(th)
        return np.concatenate([
            self.armerr(q, 'right', p, a),
            self.armerr(q, 'left',  p + a*LEFT_ON_HANDLE, a),
            wp*(v[:2] - pnom),
            0.02*(v - vprev)])

    def griperror(self, v, th):
        '''Max position error (m) of either hand on the handle.'''
        e = self.residual(v, th, v)
        return max(np.linalg.norm(e[0:3]), np.linalg.norm(e[7:10]))

    def solve_from(self, v, thetas):
        out = []
        for th in thetas:
            r = least_squares(self.residual,
                              np.clip(v, self.lo + 1e-7, self.hi - 1e-7),
                              bounds=(self.lo, self.hi), args=(th, v))
            v = r.x
            out.append(v.copy())
        return out

    def solve(self, thmin, thmax, step=1.0):
        '''Joint table for blade angles thmin..thmax (deg), via continuation
           outward from the ready stance.  Returns (thetas, Q, maxgriperr).'''
        v0 = np.r_[READY_HANDLE, READY_SEED['right'], READY_SEED['left']]
        v0 = self.solve_from(v0, [READY_THETA])[0]
        up   = np.arange(READY_THETA - step, thmin - 1e-9, -step)
        down = np.arange(READY_THETA + step, thmax + 1e-9,  step)
        vup   = self.solve_from(v0, up)
        vdown = self.solve_from(v0, down)
        thetas = np.r_[up[::-1], READY_THETA, down]
        V = vup[::-1] + [v0] + vdown
        err = max(self.griperror(v, th) for v, th in zip(V, thetas))
        return thetas, np.array([self.full(v) for v in V]), err
