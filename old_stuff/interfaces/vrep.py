
import numpy as np

from .interface import Interface
from .vrep_files import vrep
from .vrep_files.vrepConst import \
    simx_opmode_blocking, \
    simx_opmode_oneshot, \
    sim_floatparam_simulation_time_step, \
    sim_boolparam_display_enabled, \
    sim_handle_all, \
    sim_scripttype_customizationscript


class VREP(Interface):

    def __init__(self, scene, joint_names, dt=0.001):
        super(VREP, self).__init__()

        self.scene = scene
        self.N = len(joint_names)
        self.joint_names = joint_names

        self.q = np.zeros(self.N)
        self.qdot = np.zeros(self.N)

        self.joint_target_vel = np.ones(self.N) * 1.e4

        self.dt = dt
        self.t = 0

    def connect(self):
        vrep.simxFinish(-1)
        self.clientID = vrep.simxStart(
            connectionAddress='127.0.0.1',
            connectionPort=19997,
            waitUntilConnected=True,
            doNotReconnectOnceDisconnected=True,
            timeOutInMs=500,
            commThreadCycleInMs=0
        )

        if self.clientID == -1:
            raise Exception('Failed connecting to VREP remote API server')

        vrep.simxCloseScene(self.clientID, simx_opmode_blocking)
        vrep.simxLoadScene(self.clientID, self.scene, options=1, operationMode=simx_opmode_blocking)

        vrep.simxSynchronous(self.clientID, True)

        self.joint_handles = [
            vrep.simxGetObjectHandle(
                self.clientID, name, simx_opmode_blocking
            )[1] for name in self.joint_names]

        vrep.simxSetFloatingParameter(
            self.clientID,
            sim_floatparam_simulation_time_step,
            self.dt,
            simx_opmode_oneshot
        )

        vrep.simxSetBooleanParameter(
            self.clientID,
            sim_boolparam_display_enabled,
            True,
            simx_opmode_oneshot
        )

        vrep.simxStartSimulation(
            self.clientID,
            simx_opmode_blocking
        )

        print('Connected to VREP remote API server')

    def disconnect(self):
        vrep.simxStopSimulation(
            self.clientID,
            simx_opmode_blocking
        )

        vrep.simxGetPingTime(self.clientID)

        vrep.simxFinish(self.clientID)

        print('VREP connection closed')

    def send_forces(self, u):
        u *= -1

        for i, joint_handle in enumerate(self.joint_handles):
            _, torque = vrep.simxGetJointForce(
                self.clientID,
                joint_handle,
                simx_opmode_blocking
            )

            if _ != 0:
                raise Exception('Error retrieving joint torque, return code:', _)

            if np.sign(torque) * np.sign(u[i]) <= 0:
                self.joint_target_vel[i] *= -1
                _ = vrep.simxSetJointTargetVelocity(
                    self.clientID,
                    joint_handle,
                    self.joint_target_vel[i],
                    simx_opmode_blocking
                )
                if _ != 0:
                    raise Exception('Error setting joint target velocity, return code:', _)

            _ = vrep.simxSetJointForce(
                self.clientID,
                joint_handle,
                abs(u[i]),
                simx_opmode_blocking
            )
            if _ != 0:
                raise Exception('Error setting joint force, return code:', _)

            vrep.simxSynchronousTrigger(self.clientID)
            self.t += self.dt

    def send_velocities(self, u):
        for i, joint_handle in enumerate(self.joint_handles):
            _ = vrep.simxSetJointForce(
                self.clientID,
                joint_handle,
                10000,
                simx_opmode_blocking
            )
            if _ != 0:
                raise Exception('Error setting joint force, return code:', _)
            _ = vrep.simxSetJointTargetVelocity(
                self.clientID,
                joint_handle,
                u[i],
                simx_opmode_blocking
            )
            if _ != 0:
                raise Exception('Error setting joint target velocity, return code:', _)
        vrep.simxSynchronousTrigger(self.clientID)
        self.t += self.dt

    def get_feedback(self):
        for i, joint_handle in enumerate(self.joint_handles):
            _, self.q[i] = vrep.simxGetJointPosition(
                self.clientID,
                joint_handle,
                simx_opmode_blocking
            )
            if _ != 0:
                raise Exception('Error retrieving joint angle, return code:', _)

            _, self.qdot[i] = vrep.simxGetObjectFloatParameter(
                self.clientID,
                joint_handle,
                2012, # Joint anglular velocity
                simx_opmode_blocking
            )
            if _ != 0:
                raise Exception('Error retrieving joint velocity, return code:', _)

        return self.q, self.qdot

    def send_trajectory(self, data):
        _ = vrep.simxCallScriptFunction(
            self.clientID,
            'Graph',
            sim_scripttype_customizationscript,
            'trajectory',
            [],
            data,
            [],
            bytearray(),
            simx_opmode_blocking
        )[0]
        if _ != 0:
            raise Exception('Error sending user data, return code:', _)
