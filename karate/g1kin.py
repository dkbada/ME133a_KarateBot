'''g1kin.py

   Minimal kinematics for the G1 + sword URDF.

   Kinematics(urdf_xml)                 Parse the URDF (fixed + revolute joints)
   kin.fkin(q, link)       -> (T, J)    4x4 pose of 'link' in the world and the
                                        6xN geometric Jacobian [Jv; Jw] w.r.t.
                                        all N revolute joints (kin.jointnames).
'''

import numpy as np
import xml.etree.ElementTree as ET


def Rrpy(r, p, y):
    cr, sr, cp, sp, cy, sy = (np.cos(r), np.sin(r), np.cos(p), np.sin(p),
                              np.cos(y), np.sin(y))
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def Rotn(n, a):
    n = np.asarray(n, dtype=float)
    K = np.array([[0, -n[2], n[1]], [n[2], 0, -n[0]], [-n[1], n[0], 0]])
    return np.eye(3) + np.sin(a) * K + (1 - np.cos(a)) * K @ K


class Kinematics:
    def __init__(self, urdf_xml):
        root = ET.fromstring(urdf_xml)
        self.parent = {}        # child link -> (joint name, parent link, T, axis, type)
        self.jointnames = []
        self.lower = {}
        self.upper = {}
        for j in root.findall('joint'):
            o = j.find('origin')
            xyz = [float(v) for v in (o.get('xyz', '0 0 0') if o is not None else '0 0 0').split()]
            rpy = [float(v) for v in (o.get('rpy', '0 0 0') if o is not None else '0 0 0').split()]
            T = np.eye(4)
            T[:3, :3] = Rrpy(*rpy)
            T[:3, 3] = xyz
            a = j.find('axis')
            axis = np.array([float(v) for v in a.get('xyz').split()]) if a is not None else np.array([1.0, 0, 0])
            jtype = j.get('type')
            self.parent[j.find('child').get('link')] = (
                j.get('name'), j.find('parent').get('link'), T, axis, jtype)
            if jtype in ('revolute', 'continuous'):
                self.jointnames.append(j.get('name'))
                lim = j.find('limit')
                self.lower[j.get('name')] = float(lim.get('lower', -np.pi)) if lim is not None else -np.pi
                self.upper[j.get('name')] = float(lim.get('upper',  np.pi)) if lim is not None else np.pi
        self.index = {n: i for i, n in enumerate(self.jointnames)}
        self.qmin = np.array([self.lower[n] for n in self.jointnames])
        self.qmax = np.array([self.upper[n] for n in self.jointnames])

    def chain(self, link):
        steps = []
        while link in self.parent:
            steps.insert(0, self.parent[link])
            link = self.parent[link][1]
        return steps

    def fkin(self, q, link):
        T = np.eye(4)
        cols = []               # (joint index, axis_world, position_world)
        for (name, _, Tshift, axis, jtype) in self.chain(link):
            T = T @ Tshift
            if jtype in ('revolute', 'continuous'):
                i = self.index[name]
                n = T[:3, :3] @ axis
                cols.append((i, n, T[:3, 3].copy()))
                R = np.eye(4)
                R[:3, :3] = Rotn(axis, q[i])
                T = T @ R
        J = np.zeros((6, len(self.jointnames)))
        for (i, n, p) in cols:
            J[:3, i] = np.cross(n, T[:3, 3] - p)
            J[3:, i] = n
        return T, J
