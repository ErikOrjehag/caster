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
        self.m1 = 1.0
        self.m2 = mass
        self.inertia = inertia
        self.x = np.zeros(3).astype(np.float)

    def calc_J(self, i, phi):
        beta = self.beta[i]
        b = self.b
        h = self.h
        r = self.r
        sb = np.sin(beta)
        cb = np.cos(beta)
        sp = np.sin(phi)
        cp = np.cos(phi)
        sbp = np.sin(beta + phi)
        cbp = np.cos(beta + phi)
        J = np.array([
            [h*sb+b*sbp,  r*cbp, h*sb],
            [h*cb+b*cbp, -r*sbp, h*cb],
            [    1     ,    0  ,   1 ]
        ])
        Jinv = np.array([
            [ r*sbp,  r*cbp, -r*h*cp    ],
            [ b*cbp, -b*sbp,  b*h*sp    ],
            [-r*sbp, -r*cbp,  r*(b+h*cp)]
        ]) / (r*b)
        return J, Jinv

    def calc_Jinvaug(self, q):
        Jinvaug = []
        for i in range(self.N):
            Jinvaug.append(self.calc_J(i, q[i][1,0])[1][0:2,:])
        return np.array(Jinvaug)

    def compensator(self, x, xdot, x_desired, xdot_desired, xdotdot_desired):
        kp = 10
        kv = sqrt(kp)
        return -kp*(x-x_desired) - kv*(xdot-xdot_desired) + xdotdot_desired

    def control(self, q, qdot, x_desired, xdot_desired, xdotdot_desired):
        assert len(q) == len(qdot) == self.N * 2

        q = [q[2*i:2*i+2] for i in range(self.N)]
        qdot = [qdot[2*i:2*i+2] for i in range(self.N)]

        J = []
        Jinv = []
        A = []
        lam = np.zeros((3, 3))
        for i in range(self.N):
            j, ji = self.calc_J(i, q[i][0])
            J += [j]
            Jinv += [ji]
            jv = j[0:2,:]
            jw = j[2:3,:]
            a = self.m1 * jv.T @ jv + \
                self.m2 * jv.T @ jv + \
                self.inertia * jw.T @ jw
            A += [a]
            lam += np.linalg.inv(j @ np.linalg.inv(a) @ j.T)

        #Csharp = [
        #    np.linalg.pinv(Jinv[i][0:2,:], rcond=0.005)
        #    for i in range(self.N)]
        Jlpi = np.linalg.pinv([ji[0:2,:] for ji in Jinv], rcond=0.005)

        print(Jlpi)
        print(qdot)

        xdot = [Jlpi[i].dot(qdot[i]) for i in range(self.N)]

        my = np.sum([
            Jinv[i].T.dot((A[i] @ Jinv[i]).dot(xdot))
            for i in range(self.N)
        ], axis=0)

        #xdot = np.sum(xdot, axis=0)
        self.x += xdot * self.dt

        Fstar = self.compensator(
            self.x,
            xdot,
            x_desired,
            xdot_desired,
            xdotdot_desired
        )

        F = lam.dot(Fstar) + my

        u = np.concatenate([
            Jlpi[i].T.dot(F)
            for i in range(self.N)])

        return u
