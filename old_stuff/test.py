
import numpy as np
from controller0 import Controller

dt=0.01

c = Controller(
    dt=dt,
    wheel_offset=0.04,
    arm_length=0.2,
    radius=0.02,
    n_casters=3,
    mass=0.8,
    inertia=0.025,
    beta=[0, 2*np.pi/3, -2*np.pi/3]
)

J, Jinv = c.calc_J(0, 0)

print(J)
print(Jinv)
print(J @ Jinv)

print("---")

print(J.dot(np.array([100.0, 0.0, 0.0]))) # xdot = J * qdot, (sigma, rho, phi), maybe (phi, rho, sigma) ???
print(J.dot(np.array([0.0, 100.0, 0.0])))
print(J.dot(np.array([0.0, 0.0, 100.0])))

print("---")

print(J @ np.array([[1.0], [2.0], [3.0]]))
print(J.dot(np.array([1.0, 2.0, 3.0])))
print(np.array([[1, 2, 3]]).T)

print("---")

qdot = np.array([1.0, 2.0, 3.0])
xdot = J.dot(qdot)
qdot_ = Jinv.dot(xdot)

print(qdot)
print(qdot_)

print("---")

Jinv_ = Jinv[0:2,:]
print(Jinv_)
qdot_ = Jinv_.dot(xdot)
print(qdot_)

print("---")

xdot = np.array([[0, 0, 0.1]]).T

q = [
    np.array([[0, 0.0]]).T, # phi, rho
    np.array([[0, 0.0]]).T,
    np.array([[0, 0.0]]).T,
]

print(q)
Jinvaug = c.calc_Jinvaug(q)
print(Jinvaug)

qdotaug_ = Jinvaug @ xdot

print(qdotaug_)

JLPI = np.linalg.pinv(Jinvaug)

xdot_ = JLPI @ qdotaug_

print(xdot_)
