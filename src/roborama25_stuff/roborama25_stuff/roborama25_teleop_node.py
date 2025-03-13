#!/usr/bin/env python3

import rclpy
#import sys
#import serial
#import math
#import time
#import numpy as np

from rclpy.node import Node
from geometry_msgs.msg import Twist
#from std_msgs.msg import String
#from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Joy

class Roborama25TeleopNode(Node):
    # parameters?
    name = "teleop"

    spin_last = 0
    fwdRev_last = 0
    speed_last = 0
    fwdRevMpsMax = 1.0 # maximum fwd/rev meters per sec
    spinRotPsMax = 2.0/25 # maximum spin rotations per second

    pi = 3.14159265

    def __init__(self):
        super().__init__('roborama_teleop_node')

        self.joy_subscription = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.get_logger().info("Teleop Started")
       
    # Get button commands from Joy message
    # TODO: Process steering joystick controls, replace teleop_twist_joy 
    def joy_callback(self, msg):
        # get buttons
        speed = msg.axes[2] # Continuous 1.0 when not pulled, becomes -1.0 when pulled all the way
        #resetAxes = msg.buttons[6] # 1 when pushed
        fwdRev = msg.axes[7] # 1,0,-1 1.0 when up arrow pressed, -1.0 when down arrow pushed
        spin = msg.axes[6] # 1,0,-1  1.0 when left arrow pressed, -1.0 when right arrow pushed

        # convert speed range +1 to -1 into 0 to 1
        speed = 2.0 - (1.0 + speed)
        if speed > 1.0 : 
            speed = 1.0

        # TODO: move here and send an action command to wheel node
        # if resetAxes :
        #     # reset encoders and transform
        #     # TODO: ??reset coders on wheel Pico (needs new pico code cmd)??
        #     self.odMesssageCount = 0 # resets tf 

        # this worked until I added code in PICO to kill motors when no velocity is detected
        # after 1 second!
        #if spin != self.spin_last or fwdRev != self.fwdRev_last or speed != self.speed_last :
        if speed!=0 or (speed==0 and self.speed_last!=0) :
            self.fwdRev_last = fwdRev
            self.speed_last = speed
            self.spin_last = spin

            cmd_vel = Twist()
            linear = 0.0
            angular = 0.0

            if (fwdRev != 0) :
                #velR = speed * fwdRev * self.fwdRevMpsMax
                #velL = speed * fwdRev * self.fwdRevMpsMax
                linear = speed * fwdRev * self.fwdRevMpsMax

            elif(spin != 0) :
                #velR = speed * +spin * self.fwdRevMpsMax
                #velL = speed * -spin * self.fwdRevMpsMax
                # TODO: calc max rot/sec based on max wheel velocity???
                angular = speed * spin * self.spinRotPsMax * 2*self.pi
            

            # Send velocities over USB serial
            #self.wheel_serial_port.write(f"RL {velR} {velL}\n".encode())
                
            # Publish /cmd_vel
            cmd_vel.linear.x = linear
            cmd_vel.linear.y = linear
            cmd_vel.angular.z = angular
            self.cmd_vel_publisher.publish(cmd_vel)

def main(args=None):
    rclpy.init(args=args)

    node = Roborama25TeleopNode()
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()

# This code is needed to run .py file directly
if __name__ == '__main__':
    main()
