'''swing.py

   Make the G1 hold the sword with both hands and repeat a simple
   vertical swing.

   Node:        /swing
   Publish:     /joint_states           sensor_msgs/JointState

   At startup this solves the two-handed grip for every blade angle of the
   swing (see swingplanner.py), then plays back:
     - move from the zero pose to the ready stance (blade 30 deg forward)
     - repeat:  raise overhead -> fast strike -> hold -> back to ready
'''

import os
import rclpy
import numpy as np

from ament_index_python.packages import get_package_share_directory as pkgdir
from rclpy.executors             import ExternalShutdownException
from rclpy.node                  import Node
from sensor_msgs.msg             import JointState

from karate.swingplanner         import SwingPlanner, READY_THETA


# Blade angles (deg from vertical, + = forward).
TH_RAISED = -40.0
TH_STRIKE =  55.0

# Swing timing (seconds): (duration, start angle, end angle).
STARTUP = 2.5
CYCLE = [
    (0.6, READY_THETA, READY_THETA),    # hold ready
    (1.2, READY_THETA, TH_RAISED),      # raise overhead
    (0.3, TH_RAISED,   TH_RAISED),      # pause at the top
    (0.4, TH_RAISED,   TH_STRIKE),      # strike!
    (0.5, TH_STRIKE,   TH_STRIKE),      # hold the strike
    (0.9, TH_STRIKE,   READY_THETA),    # return to ready
]


def quintic(t, T):
    '''Smooth 0->1 over [0,T] with zero velocity/acceleration at the ends.'''
    s = min(max(t / T, 0.0), 1.0)
    return 10*s**3 - 15*s**4 + 6*s**5


class SwingNode(Node):
    def __init__(self):
        super().__init__('swing')

        urdf = os.path.join(pkgdir('karate'), 'urdf', 'g1_23dof_sword.urdf')
        with open(urdf, 'r') as f:
            planner = SwingPlanner(f.read())

        self.get_logger().info('Solving two-handed grip for the swing...')
        (self.thetas, self.Q, err) = planner.solve(TH_RAISED, TH_STRIKE, 1.0)
        self.get_logger().info(
            f'Done: {len(self.thetas)} blade angles, max grip error {err*1000:.2f} mm')

        self.jointnames = planner.kin.jointnames
        self.qready = self.qof(READY_THETA)
        self.period = sum(seg[0] for seg in CYCLE)

        self.pub = self.create_publisher(JointState, '/joint_states', 10)
        self.dt = 0.01
        self.t0 = self.get_clock().now()
        self.timer = self.create_timer(self.dt, self.update)

    def qof(self, th):
        '''Joint vector for a blade angle (interpolating the solved table).'''
        return np.array([np.interp(th, self.thetas, self.Q[:, j])
                         for j in range(self.Q.shape[1])])

    def theta_at(self, tc):
        '''Blade angle at time tc into the swing cycle.'''
        tc = tc % self.period
        for (T, th0, th1) in CYCLE:
            if tc < T:
                return th0 + (th1 - th0) * quintic(tc, T)
            tc -= T
        return READY_THETA

    def update(self):
        if not self.context.ok():               # shutting down
            return
        t =(self.get_clock().now() - self.t0).nanoseconds * 1e-9
        if t < STARTUP:
            q = self.qready * quintic(t, STARTUP)       # zero pose -> ready
        else:
            q = self.qof(self.theta_at(t - STARTUP))

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.jointnames
        msg.position = q.tolist()
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SwingNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
