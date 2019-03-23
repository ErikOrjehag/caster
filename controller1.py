import numpy as np
from math import sqrt

class Controller():

    def __init__(self, dt, wheel_offset, arm_length, radius, n_casters, mass, inertia, beta):
        self.dt = dt
        self.N = n_casters
        self.b = wheel_offset
        self.h = arm_length
        self.r = radius
        self.beta = beta
        self.x = np.zeros(3).astype(np.float)

    def J_inv(self, beta, b, h, r, phi):
        sp = np.sin(phi)
        cp = np.cos(phi)
        sb = np.sin(beta)
        cb = np.cos(beta)
        cbcp = cb * cp
        cbsp = cb * sp
        sbsp = sb * sp
        sbcp = sb * cp
        row = [-sp/b, cp/b, h*(cbcp+sbsp)/b]
        J_inv = np.array([
            row,
            [cp/r, sp/r, h*(cbsp-sbcp)/r],
            row
        ])
        J_inv[0,2] -= 1
        return J_inv

    def compensator(self, x, xdot, x_desired, xdot_desired, xdotdot_desired):
        kp = 10
        kv = sqrt(kp)
        return -kp*(x-x_desired) - kv*(xdot-xdot_desired) + xdotdot_desired

    def control(self, q, xdot_desired):
        assert len(q) == self.N * 2

        q = [q[2*i:2*i+2] for i in range(self.N)]

        J_inv = [
            self.J_inv(
                beta=self.beta[i],
                b=self.b,
                h=self.h,
                r=self.r,
                phi=q[i][0])
            for i in range(self.N)]

        qdot = [J_inv[i][0:2,:].dot(xdot_desired) for i in range(self.N)]

        for i in range(len(qdot)):
            qdot[i][0] *= -1

        u = np.concatenate(qdot)

        return u
