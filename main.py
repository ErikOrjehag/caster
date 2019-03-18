
import numpy as np
from controller import Controller
from interfaces.vrep import VREP
from time import time

dt = 0.01

c = Controller(
    dt=dt,
    wheel_offset=0.04,
    arm_length=0.2,
    radius=0.02,
    n_casters=1,
    mass=0.00008,
    inertia=0.00008,
    beta=[0.]
    #beta=[0., np.pi/2, np.pi, 3*np.pi/2]
)

sim = VREP(
    scene='scenes/caster2.ttt',
    joint_names=[
        'Wheel0_SteerJoint',
        'Wheel0_DriveJoint',
        #'Wheel1_SteerJoint',
        #'Wheel1_DriveJoint',
        #'Wheel2_SteerJoint',
        #'Wheel2_DriveJoint',
        #'Wheel3_SteerJoint',
        #'Wheel3_DriveJoint',
    ],
    dt=dt
)
sim.connect()

traj_N = 500
traj_acc = np.zeros((traj_N, 3)).astype(np.float)
traj_vel = np.zeros((traj_N, 3)).astype(np.float)
traj_pos = np.zeros((traj_N, 3)).astype(np.float)

traj_acc[0:100,:] = np.array([0.2, 0, 0.0])
for i in range(1, traj_N):
    traj_vel[i] = traj_vel[i-1] + traj_acc[i-1] * dt
for i in range(1, traj_N):
    traj_pos[i] = traj_pos[i-1] + traj_vel[i-1] * dt

for ti in range(traj_N):
    q, qdot = sim.get_feedback()

    print(traj_pos[ti])
    print(traj_vel[ti])
    print(traj_acc[ti])

    uu = c.control(
        q=q,
        qdot=qdot,
        x_desired=traj_pos[ti],
        xdot_desired=traj_vel[ti],
        xdotdot_desired=traj_acc[ti]
    )

    #uu = np.array([0, 0.001] * 4)

    #print("uu", uu)

    sim.send_forces(uu)

sim.disconnect()
