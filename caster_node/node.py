# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from caster_node.controller1 import Controller
from caster_node.caster import *
import numpy as np

class CasterNode(Node):

    def __init__(self):
        super().__init__('caster_node')
        self.subscription = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_cb, 10)
        self.subscription  # prevent unused variable warning
        
        self.casters = [
            Caster("208C3373304B", 0),
            Caster("206A337F304B", 1),
            Caster("206A339F304B", 0),
            Caster("20773398304B", 1),
        ]

        self.controller = Controller(
            dt=0.01,
            wheel_offset=0.08,
            arm_length=0.415/2,
            radius=0.075/2,
            n_casters=4,
            mass=0.8,
            inertia=0.025,
            #beta=[0., np.pi/2, np.pi, 3*np.pi/2]
            beta=[0., np.pi/2, np.pi, -np.pi/2]
        )
        self.cmd_vel = np.array([0.0,0.0,0.0])

        # Move to other topic, andra parametern ställer om ett visst steg i kalibreringen ska köras eller ej
        run_setup_sequence(self.casters, True)


        print("Setup done")
        self.timer = self.create_timer(0.2, self.timer_cb)

        
        
    def timer_cb(self):
        self.destroy_timer(self.timer)
        
        
          
        q = []
        for caster in self.casters:
            wheel_pos, arm_pos = caster.get_position()
            q += [arm_pos, wheel_pos]

        u = self.controller.control(np.copy(q), np.copy(self.cmd_vel))
    
        for i, caster in enumerate(self.casters):
            arm_vel, wheel_vel = u[i*2:i*2+2]
            caster.set_velocity(wheel_vel, arm_vel)
        
        
        self.timer = self.create_timer(0.2, self.timer_cb)
        
        
    def cmd_vel_cb(self, msg):
        self.cmd_vel += 0.1*(np.array([msg.linear.x, msg.linear.y, msg.angular.z]) - self.cmd_vel)
        print("Callback from joystick")

    def stop_motors(self):
        run_shutdown_sequence(self.casters)

def main(args=None):
    rclpy.init(args=args)

    node = CasterNode()
    print("Spin")
    rclpy.spin(node)

    node.stop_motors()
    
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

