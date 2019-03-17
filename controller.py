import numpy as np

class Controller():

    def __init__(self, wheel_offset, arm_length, radius, n_casters, mass, inertia):
        self.N = n_casters
        self.b = wheel_offset
        self.h = arm_length
        self.r = radius
        self.beta = np.linspace(np.pi/4, 7*np.pi/4, n_casters)
        self.A = np.diag([mass, mass, inertia])

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
            [ cp/r, sp/r, h*(cbsp-sbcp)/r],
            row
        ])
        J_inv[0,2] -= 1
        return J_inv

    def control(self, q, qdot, Fstar):
        assert len(q) == len(qdot) == self.N * 2

        q = [q[i:i+2] for i in range(self.N)]
        qdot = [qdot[i:i+2] for i in range(self.N)]

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
            J_inv[i].T.dot(self.A.dot(J_inv[i].dot(xdot[i])))
            for i in range(self.N)
        ], axis=0)

        F = lam.dot(Fstar) + my

        u = np.concatenate([
            Csharp[i].T.dot(F) 
            for i in range(self.N)])

        return u