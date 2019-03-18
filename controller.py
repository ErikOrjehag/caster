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
        self.A = np.diag([mass, mass, inertia])
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

    def control(self, q, qdot, x_desired, xdot_desired, xdotdot_desired):
        assert len(q) == len(qdot) == self.N * 2

        q = [q[2*i:2*i+2] for i in range(self.N)]
        qdot = [qdot[2*i:2*i+2] for i in range(self.N)]

        J_inv = [
            self.J_inv(
                beta=self.beta[i],
                b=self.b,
                h=self.h,
                r=self.r,
                phi=q[i][0])
            for i in range(self.N)]

        lam = np.sum([
            J_inv[i].T @ self.A @ J_inv[i]
            for i in range(self.N)
        ], axis=0)

        Csharp = [
            np.linalg.pinv(J_inv[i][0:2,:], rcond=0.005)
            for i in range(self.N)]

        xdot = [
            Csharp[i].dot(qdot[i])
            for i in range(self.N)]

        my = np.sum([
            J_inv[i].T.dot((self.A @ J_inv[i]).dot(xdot[i]))
            for i in range(self.N)
        ], axis=0)

        xdot = np.sum(xdot, axis=0)
        self.x += xdot * self.dt

        Fstar = self.compensator(
            self.x,
            xdot,
            x_desired,
            xdot_desired,
            xdotdot_desired
        )

        F = lam.dot(Fstar) + my

        print("F", F)

        u = np.concatenate([
            Csharp[i].T.dot(F)
            for i in range(self.N)])

        return u
