
import numpy as np
from controller import Controller
from interfaces.vrep import VREP
from time import time

c = Controller(
    wheel_offset=0.04,
    arm_length=0.233,
    radius=0.02,
    n_casters=4,
    mass=10.0,
    inertia=0.015
)

sim = VREP(joint_names=[
    'fl_caster_steer_joint',
    'fl_caster_wheel_joint',
    'rl_caster_steer_joint',
    'rl_caster_wheel_joint',
    'rr_caster_steer_joint',
    'rr_caster_wheel_joint',
    'fr_caster_steer_joint',
    'fr_caster_wheel_joint',
])
sim.connect()

forces = np.full((100, 3), np.array([0., 0., 1.]))

for f in forces:
    t = time()
    q, qdot = sim.get_feedback()
    print("time feedback: %.2fms" % ((time() - t) * 1000))

    print(q)
    print(qdot)

    t = time()
    uu = c.control(
        q=q,
        qdot=qdot,
        Fstar=f
    )
    print("time u: %.2fms" % ((time() - t) * 1000))

    print(uu)

    t = time()
    sim.send_forces(np.full(8, 1.))
    print("time forces: %.2fms" % ((time() - t) * 1000))

sim.disconnect()