import rclpy
import sys
import serial
import math
import time
import numpy as np

from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import String

class Robo24ImuSerialNode(Node):

    # IMU rate can be up to 100 Hz
    timerRateHz = 110
    imu_serial_port_name = "/dev/serial/by-id/usb-Adafruit_QT_Py_RP2040_DF625857C7633033-if00"

    def __init__(self):
        super().__init__('robo24_imu_serial_node')

        self.imu_serial_port = serial.Serial(self.imu_serial_port_name, 115200)

        self.imu_msg_publisher = self.create_publisher(Imu, 'IMU', 10)
        self.cal_msg_publisher = self.create_publisher(String, 'imu_cal_msg', 10)


        self.timer = self.create_timer((1.0/self.timerRateHz), self.timer_callback)

        self.get_logger().info(f"Robo24ImuSerialNode Started")

    def timer_callback(self):

        # Check if a line has been received on the serial port
        try :
            if self.imu_serial_port.in_waiting > 0:
                received_data = self.imu_serial_port.readline().decode().strip()
                #self.get_logger().info(f"Received: {received_data}")
                
                # parse the IMU serial string
                strArray = received_data.split(" ")
                if strArray[0]!="IMU" and strArray[0]!="CAL":
                    self.get_logger().error(f"Not IMU or CAL message error: {received_data}")
                    return

                if strArray[0]=="CAL" :
                    if len(strArray)!=5:
                        self.get_logger().error(f"CAL message format error: {received_data}")
                        return

                    else :
                        msg = String()
                        msg.data = received_data
                        self.cal_msg_publisher.publish(msg)

                if strArray[0]=="IMU" :
                    if len(strArray)!=12:
                        self.get_logger().error(f"IMU message format error: {received_data}")
                        return

                    else :
                        # Create and publish the received serial line as a Imu message
                        msg = Imu()
                        
                        imu_timestamp = int(strArray[1])

                        # NOTE: should we use the imu timestamp to get better differential accuracy?
                        msg.header.stamp = self.get_clock().now().to_msg()
                        # The IMU is located at the centroid of the robot and aligned XY
                        # so no offest is needed
                        msg.header.frame_id = "base_link"

                        msg.orientation.x = float(strArray[9])
                        msg.orientation.y = float(strArray[10])
                        msg.orientation.z = float(strArray[11])
                        msg.orientation.w = float(strArray[8])

                        msg.angular_velocity.x = float(strArray[2])
                        msg.angular_velocity.y = float(strArray[3])
                        msg.angular_velocity.z = float(strArray[4])

                        msg.linear_acceleration.x = float(strArray[5])
                        msg.linear_acceleration.y = float(strArray[6])
                        msg.linear_acceleration.z = float(strArray[7])

                        self.imu_msg_publisher.publish(msg)

        except Exception as ex:
            self.get_logger().error(f"imu serial read exception {ex}")
            self.imu_serial_port.close()
            self.imu_serial_port.open()
            return

def main(args=None):
    rclpy.init(args=args)

    node = Robo24ImuSerialNode()
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()

# This code is needed to run .py file directly
if __name__ == '__main__':
    main()
