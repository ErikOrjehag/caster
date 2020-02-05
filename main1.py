
import numpy as np
from controller1 import Controller
from interfaces.vrep import VREP
from time import time
from math import sqrt

dt = 0.01

c = Controller(
    dt=dt,
    wheel_offset=0.04,
    arm_length=0.2,
    radius=0.075/2,
    n_casters=4,
    mass=0.8,
    inertia=0.025,
    beta=[0., np.pi/2, np.pi, 3*np.pi/2]
)

sim = VREP(
    scene='scenes/caster2.ttt',
    joint_names=[
        'Wheel0_SteerJoint',
        'Wheel0_DriveJoint',
        'Wheel1_SteerJoint',
        'Wheel1_DriveJoint',
        'Wheel2_SteerJoint',
        'Wheel2_DriveJoint',
        'Wheel3_SteerJoint',
        'Wheel3_DriveJoint',
    ],
    dt=dt
)
sim.connect()

traj_N = 2000
traj_acc = np.zeros((traj_N, 3)).astype(np.float)
traj_vel = np.zeros((traj_N, 3)).astype(np.float)
traj_pos = np.zeros((traj_N, 3)).astype(np.float)

traj_acc[0:100,:]   = np.array([ 0.0,  0.8,  0.0])
traj_acc[100:200,:] = np.array([ 0.0, -0.8,  0.0])
traj_acc[200:300,:] = np.array([ 0.8,  0.0,  0.0])
traj_acc[300:500,:] = np.array([-0.8,  0.0,  0.0])
traj_acc[500:600,:] = np.array([ 0.0,  0.0,  0.0])
traj_acc[600:700,:] = np.array([ 0.8,  0.0,  0.0])
traj_acc[700:,:] = np.array([ 0.0,  -1.6,  0.0])
for i in range(1, traj_N):
    traj_vel[i] = traj_vel[i-1] + traj_acc[i-1] * dt
for i in range(1, traj_N):
    traj_pos[i] = traj_pos[i-1] + traj_vel[i-1] * dt

vel = [sqrt(v[0]**2 + v[1]**2) for v in traj_vel]
print("MAX", np.max(vel))

for ti in range(traj_N):
    q, qdot = sim.get_feedback()

    sim.send_trajectory(list(traj_pos[ti]))

    uu = c.control(
        q=q,
        xdot_desired=traj_vel[ti]
    )

    #print(uu)

    #uu = np.array([2.0, 0.0, 2.0, 0.0, 2.0, 0.0, 2.0, 0.0])

    sim.send_velocities(uu)

sim.disconnect()
