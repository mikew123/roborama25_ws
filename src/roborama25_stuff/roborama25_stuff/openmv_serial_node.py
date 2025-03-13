#!/usr/bin/env python3

import rclpy
import sys
import serial
import math
import time
import numpy as np

from rclpy.node import Node
from std_msgs.msg import String

class OpenmvSerialNode(Node):
    # parameters?

    timerRateHz = 50.0; # Rate to check serial port for messages

    #serial_port = "/dev/ttyACM0"
    serial_port = "/dev/serial/by-id/usb-MicroPython_OpenMV_IMXRT1060_9D7B4061D7210432-if00"

    def __init__(self):
        super().__init__('openmv_serial_node')

        self.openmv_serial_port = serial.Serial(self.serial_port, 115200)

        self.openmv_msg_publisher = self.create_publisher(String, 'openmv_msg', 10)

        self.timer = self.create_timer((1.0/self.timerRateHz), self.timer_callback)

        if (not self.openmv_serial_port.is_open) : self.get_logger().error(f"openmv port {self.serial_port} not open")
        self.get_logger().info(f"OpenmvSerialNode Started")

    # check serial port at timerRateHz and parse out messages to publish
    def timer_callback(self):
        
        # Check if a line has been received on the serial port
        if self.openmv_serial_port.in_waiting > 0:
            received_data = self.openmv_serial_port.readline().decode().strip()
            #self.get_logger().info(f"Received: {received_data}")
            
            # Publish the received serial line as a String message
            emsg = String()
            emsg.data = received_data

            self.openmv_msg_publisher.publish(emsg)


def main(args=None):
    rclpy.init(args=args)

    node = OpenmvSerialNode()
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()

# This code is needed to run .py file directly
if __name__ == '__main__':
    main()

