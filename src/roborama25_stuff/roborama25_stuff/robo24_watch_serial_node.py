#!/usr/bin/env python3

import rclpy
import sys
import serial
import math
import time
import numpy as np
import json
import os

from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import BatteryState

class WatchSerialNode(Node):
    # parameters?

    timerRateHz = 50.0; # Rate to check serial port for messages

    #serial_port = "/dev/ttyUSB0"
    #ESP32 C6 DEV M-1 UART port
    #serial_port = "/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_c2454e516757ed1183862df21c62bc44-if00-port0"
    #ESP32 S2 MINI 2U
    serial_port = "/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_4cf2988ca91ced11a618bcb4bbdd3192-if00-port0"

    # Status info to send to watch
    batteryVolt: float = 0
    batteryCurr: float = 0
    batteryReady: bool = False
    nav_modeReady:bool = False

    def __init__(self):
        super().__init__('watch_serial_node')

        self.watch_serial_port = serial.Serial(self.serial_port, 115200)

        self.watch_json_publisher = self.create_publisher(String, 'watch_json', 10)
        self.robo24_json_publisher = self.create_publisher(String, 'robo24_json',10)

        self.battery_subscription = self.create_subscription(BatteryState, 'battery_status', self.battery_callback, 10)
        self.robo24_json_subscription = self.create_subscription(String, 'robo24_json', self.robo24_json_callback, 10)
        self.watch_json_subscription = self.create_subscription(String, 'watch_json', self.watch_json_callback, 10)

        self.timer = self.create_timer((1.0/self.timerRateHz), self.timer_callback)

        if (not self.watch_serial_port.is_open) : self.get_logger().error(f"watch port {self.serial_port} not open")
            
        self.get_logger().info(f"{self.watch_serial_port.rts=} {self.watch_serial_port.dtr=} {self.watch_serial_port.cts=} {self.watch_serial_port.dsr=} {self.watch_serial_port.ri=} {self.watch_serial_port.cd=}")

        self.get_logger().info(f"WatchSerialNode Started")

    def robo24_json_publish(self, data:str) -> None :
        msg = String()
        msg.data = data
        self.robo24_json_publisher.publish(msg)

    def watch_json_publish(self, data:str) -> None :
        msg = String()
        msg.data = data
        self.watch_json_publisher.publish(msg)

    # check serial port at timerRateHz and parse out messages to publish and send statuses
    # the strings to/from watch are JSON formated
    # All serial reads and writes are handled in timer 
    def timer_callback(self):
        
        # Check if a line has been received on the serial port
        # TODO: check for valid JSON???
        if self.watch_serial_port.in_waiting > 0 :
            data = self.watch_serial_port.readline().decode().strip()
            self.get_logger().info(f"watch msg rx json {data=}")
            self.watch_json_publish(data)
            
        # Send battery voltage and current to watch via serial and ESp32 board esp-now
        if self.batteryReady == True :
            #send status information in JSON
            jsonObj:dict = {"rv": self.batteryVolt, "ra": self.batteryCurr}
            #self.get_logger().info(f"watch msg tx {jsonObj=}")
            jsonStr:str = json.dumps(jsonObj)
            self.watch_serial_port.write((jsonStr+"\n").encode())
            self.batteryReady = False

        # Navigator state info to watch
        if self.nav_modeReady == True :
            self.watch_serial_port.write((self.nav_mode+"\n").encode())
            self.nav_modeReady = False

    # modes encoded as JSON strings
    def robo24_json_callback(self, msg:String) -> None :
        # send nav modes to watch serial
        packet_bytes = msg.data

        try :
            packet = json.loads(packet_bytes)
            if "run_state" in packet :
                if self.nav_modeReady == False :
                    self.nav_mode = packet_bytes
                    self.nav_modeReady = True
                
        except Exception as ex:
            self.get_logger().error(f"watch serial robo24_json_callback exception {ex}")        

    def watch_json_callback(self, msg:String) -> None :
        
        data = msg.data

        try :
            packet = json.loads(data)
            self.get_logger().error(f"{packet=}")
            # send nav run state to nav publish
            if "nav_cmd" in packet :
                self.robo24_json_publish(data)
            if "claw" in packet :
                self.robo24_json_publish(data)

        except Exception as ex:
            self.get_logger().error(f"watch serial watch_json_callback exception {ex}")        

    def battery_callback(self, msg:BatteryState) -> None :
        if self.batteryReady == False :
            self.batteryVolt = msg.voltage
            self.batteryCurr = msg.current
            self.batteryReady = True

def main(args=None):
    rclpy.init(args=args)

    node = WatchSerialNode()
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()

# This code is needed to run .py file directly
if __name__ == '__main__':
    main()

