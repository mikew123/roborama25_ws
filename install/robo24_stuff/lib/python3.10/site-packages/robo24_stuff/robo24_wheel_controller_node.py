#!/usr/bin/env python3

import rclpy
import sys
import serial
import math
import time
import numpy as np

from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Joy
from nav_msgs.msg import Odometry

from geometry_msgs.msg import Twist
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Point, Quaternion, Vector3

from tf2_ros.transform_broadcaster import TransformBroadcaster

from robo24_interfaces.srv import Claw
import json

class Robo24WheelControllerNode(Node):
    # parameters?
    name = "robo24"

    # Enable TF odom->base_link
    #tf_enable = False # false when ekf is used
    tf_enable = False

    # values sent to wheel Pico over serial interface
    odometryRateHz = 30; # Rate that the wheel and odom encoders send data on the serial port
    # fwdPullOffset = 1.0025 # cal 5M forward compensate for pull offset
    fwdPullOffset = 1.0020 # cal Quick Trip with laser
    revPullOffset = 1.0300  # reverse compesate for pull offset
    wheelVelocityAccLimit = 1.5 # acceleration max in meters per sec per sec
    
    timerRateHz = 1.5*odometryRateHz; # Rate to check serial port for messages


    pi = 3.14159265


    wheelEncoders = True

    wheelDiameter = 0.080
    wheelEncoderCounts = 48*20.408666666
    #wheelDistance = 0.31675 # cal CCW
    #wheelDistance = 0.3172 # cal slam acc angular error running waypoints
    #wheelDistance = 0.3152 # cal slam angular error with plot juggler
#    wheelDistance = 0.3175 # cal quicktrip with laser
    wheelDistance = 0.3180 # cal quicktrip using dprg distances

#    odomDiameter = 0.048
#    odomDiameter = 0.048 * 4.950/4.970 # adjust for 5M-5cm travel error of 2cm
    odomDiameter = 0.048 * 65651/65910 # adjust for 5M-5cm travel, re-glued
    odomEncoderCounts = 2000.0 #per rotation
    #odomDistance = 0.224 #around center
    odomDistance = 0.224 * (pi/(3.024297+0.0075))#Calibrated using wheel odom

    odMesssageCount = 0

    od_last = np.empty(5, dtype=int)
    xy_last = np.empty(3, dtype=float)

    xyo_last = np.empty(3, dtype=float)

    spin_last = 0
    fwdRev_last = 0
    speed_last = 0
    fwdRevMpsMax = 0.2

    vel_cmd_timeout_sec:float = 1.0

    #serial_port = "/dev/ttyACM2"
    serial_port = "/dev/serial/by-id/usb-Waveshare_RP2040_Zero_E6617C93E33D6927-if00"

    def __init__(self):
        super().__init__('robo24_wheel_controller_node')

        #self.wheel_serial_port = serial.Serial('/dev/ttyACM0', 115200)  # doublecheck find way to autodetect
        self.wheel_serial_port = serial.Serial(self.serial_port, 115200)
        # Do I need to check/wait for serial port to be ready?

        self.cmd_vel_subscription = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.encoders_msg_subscription = self.create_subscription(String, 'encoders_msg', self.encoders_msg_callback, 10)
        self.joy_subscription = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        
        #self.watch_json_subscription = self.create_subscription(String, 'watch_json', self.watch_json_callback, 10)
        self.robo24_json_subscription = self.create_subscription(String, 'robo24_json', self.robo24_json_callback, 10)

        self.encoders_msg_publisher = self.create_publisher(String, 'encoders_msg', 10)
        self.odometry_publisher = self.create_publisher(Odometry, 'wheel_odom', 10)
        self.wheel_debug_msg_publisher = self.create_publisher(String, 'wheel_debug_msg', 10)

        self.tf_broadcaster = TransformBroadcaster(self)


        self.timer = self.create_timer((1.0/self.timerRateHz), self.timer_callback)

        # Configure wheel controller 
        self.wheel_serial_port.write("CP 0 1000\n".encode()) # Extra write to wait a tiny bit
        self.wheel_serial_port.write(f"OR {self.odometryRateHz}\n".encode())
        self.wheel_serial_port.write(f"WO {self.fwdPullOffset} {self.revPullOffset}\n".encode())
        self.wheel_serial_port.write(f"AR {self.wheelVelocityAccLimit}\n".encode())
        self.wheel_serial_port.write("CP 0 1000\n".encode())

        self.get_logger().info(f"{self.wheel_serial_port.rts=} {self.wheel_serial_port.dtr=} {self.wheel_serial_port.cts=} {self.wheel_serial_port.dsr=} {self.wheel_serial_port.ri=} {self.wheel_serial_port.cd=}")

        self.get_logger().info(f"Robo24WheelNode Started: Odometry rate = {self.odometryRateHz} Hz")

    # Get button commands from Joy message
    def joy_callback(self, msg):
        resetAxes = msg.buttons[6] # 1 when pushed

        if resetAxes :
            # reset encoders and transform
            # TODO: ??reset coders on wheel Pico (needs new pico code cmd)??
            self.odMesssageCount = 0 # resets tf 


    # Convert /cmd_vel messages to physical 2 wheel diff drive velocities in meters per second
    def cmd_vel_callback(self, msg):
        linear_velocity = msg.linear.x
        angular_velocity = msg.angular.z
        
        #self.get_logger().info("/vel_cmd " + str(linear_velocity) +", "+str(angular_velocity))
       
        # A simple differential drive model
        # TODO wheel_base should come from a parameter
        
        angular_scale = 2*self.pi # Increase angular velocity to rotate once per second?
        
        right_wheel_velocity = linear_velocity + (angular_velocity*angular_scale * self.wheelDistance / 2)
        left_wheel_velocity = linear_velocity - (angular_velocity*angular_scale * self.wheelDistance / 2)

        # Send velocities over USB serial
        self.wheel_serial_port.write(f"RL {right_wheel_velocity} {left_wheel_velocity} {self.vel_cmd_timeout_sec}\n".encode())
    
    # Convert encoder messages from physical wheels into tf2 position/velocity messages
    # and publish the /wheel_odom topic for NAV2 stack
    # Message string format OD timeUs wheelR wheelL odomF odomB
    # all data integer unsigned 32 bit
    def encoders_msg_callback(self, msg):
        # Parse OD message from wheel encoders
        od = np.empty(5, dtype=int)

        odStrArray = msg.data.split(" ")

        if odStrArray[0]!="OD" or len(odStrArray)!=6:
            self.get_logger().error(f"OD message error: {msg.data}")
            return
    
        try:
            od[0] = int(odStrArray[1]) #timestamp
            od[1] = int(odStrArray[2]) #wheelR
            od[2] = int(odStrArray[3]) #wheelL
            od[3] = int(odStrArray[4]) #odomF-X(translation fwd-rev)
            od[4] = int(odStrArray[5]) #odomB-Y(rotation CW-CC)
        except:
            self.get_logger().error(f"OD message error: {msg.data}")
            return

        # init to zero offset
        if self.odMesssageCount == 0:
            self.od_last = od
            self.xy_last = [0.0, 0.0, 0.0]
            self.odMesssageCount = 1

            self.xyo_last = [0.0, 0.0, 0.0]

            return
        
        xy = np.empty(3, dtype=float) #x,y,theta

        # Generate Pose
        # encoder difference
        if self.wheelEncoders==True:
            #using wheel encoders
            dEncR = od[1] - self.od_last[1]
            dEncL = od[2] - self.od_last[2]

            #convert to meters wheel moved (assumes both wheels are the same diameter)
            dR = (dEncR / self.wheelEncoderCounts) * (self.pi*self.wheelDiameter)
            dL = (dEncL / self.wheelEncoderCounts) * (self.pi*self.wheelDiameter)
            dRL = (dR+dL)/2.0
            dTheta = (dR-dL)/self.wheelDistance

            #-------------------------------------------------------------------------
            #This section of code is where I believe the odometry errors creep in
            # I am starting to doubt this assumption, sytemic errors are in the wheel spacing and radiuses
            # The navigation stack and fusion seem to use velocities and time for distance, sys delta time error?
            #intergrate(accumulate delta offsets) for the current position

            xy[2] = self.xy_last[2] - dTheta  # Theta sign corrected, reversed
            #wrap Theta to range of +-pi
            if xy[2] > self.pi:
                xy[2] = xy[2] - (2*self.pi)
            if xy[2] < -self.pi:
                xy[2] = xy[2] + (2*self.pi)

            xy[0] = self.xy_last[0] + (dRL * math.cos(-xy[2])) #X
            xy[1] = self.xy_last[1] + (dRL * math.sin(-xy[2])) #Y
            #-------------------------------------------------------------------------

#        else :
            xyo = np.empty(3, dtype=float) #x,y,theta
            #using odom_encoders
            #using wheel encoders
            dEncX:int = od[3] - self.od_last[3]
            dEncY:int = od[4] - self.od_last[4]

            odomWheelCircum  = (self.pi*self.odomDiameter)
            odomCircleCircum = (self.pi*self.odomDistance)
            dT:float = 2*self.pi*(((dEncY/self.odomEncoderCounts)*odomWheelCircum)/odomCircleCircum)
            xyo[2] = self.xyo_last[2] - dT
            #wrap Theta to range of +-pi
            if xyo[2] > self.pi:
                xyo[2] = xyo[2] - (2*self.pi)
            if xyo[2] < -self.pi:
                xyo[2] = xyo[2] + (2*self.pi)
            To = xyo[2] # current rotation

            dXenc:float = (dEncX/self.odomEncoderCounts) * odomWheelCircum
            #X offset error caused by rotation
            #dXoff:float = self.odomDistance/2.0 * (1 - math.cos(dT))
            #dX:float = dXenc - dXoff
            #dY:float = dX * math.tan(dT)
            dXo:float = dXenc * math.cos(To)
            dYo:float = dXenc * -math.sin(To)


            xyo[0] = self.xyo_last[0] + dXo
            xyo[1] = self.xyo_last[1] + dYo


        #wdmsg = String()
        #wdmsg.data = f"\n{xy  = }\n{xyo = }\n{xy - xyo = }"
        #self.wheel_debug_msg_publisher.publish(wdmsg)

        if self.tf_enable == True :
            self.broadcast_tf(xy[0], xy[1], -xy[2]) # x,y,theta
            #self.broadcast_tf(xyo[0], xyo[1], -xyo[2]) # x,y,theta


        # Publish odometry
        # Calculate velocities using wheel encoder time
        dt = 1e-6*(od[0] - self.od_last[0]) # Seconds

        dx = xy[0] - self.xy_last[0] # Meters
        dy = xy[1] - self.xy_last[1] # Meters
        dtheta = self.xy_last[2] - xy[2] # Radians

        #dx = xyo[0] - self.xyo_last[0] # Meters
        #dy = xyo[1] - self.xyo_last[1] # Meters
        #dtheta = self.xyo_last[2] - xyo[2] # Radians

        # handle the transition .999<>-.999 rollover
        if dtheta > +self.pi : dtheta -= 2*self.pi
        if dtheta < -self.pi : dtheta += 2*self.pi
        vl = Vector3()
        va = Vector3()
        if dt > 0 :
            #vl.x = dx/dt
            #vl.y = dy/dt
            vl.x = dRL/dt
            vl.y = 0.0
            va.z = dtheta/dt


        # publish the Odometry topic /wheel_odom for NAV2
        omsg = Odometry()
        # TODO: Should sys time or vel be tweaked to match time from encoders to reduce error? 
        omsg.header.stamp = self.get_clock().now().to_msg()
        omsg.header.frame_id = 'odom'
        omsg.child_frame_id = 'base_link'
    
        # Position of child_frame_id relative to header.frame_id 
        omsg.pose.pose.position.x = xy[0] #X
        omsg.pose.pose.position.y = xy[1] #Y
        qa = self.quaternion_from_euler(0.0, 0.0, -xy[2]) #0,0,theta

        #omsg.pose.pose.position.x = xyo[0] #X
        #omsg.pose.pose.position.y = xyo[1] #Y
        #qa = self.quaternion_from_euler(0.0, 0.0, -xyo[2]) #0,0,theta

        omsg.pose.pose.orientation.x = qa[0]
        omsg.pose.pose.orientation.y = qa[1]
        omsg.pose.pose.orientation.z = qa[2]
        omsg.pose.pose.orientation.w = qa[3]

        # TODO: make a valid matrix, this can be calc once in init
        # used variance set to 0.001, unused set to 1 (set ignore in config)
        omsg.pose.covariance = self.set_covariance(1e-3, 1e-3, 1, 1, 1, 1e-3)

        # Velocities of child_frame_id relative to self

        omsg.twist.twist.linear.x = vl.x
        omsg.twist.twist.linear.y = vl.y
        omsg.twist.twist.angular.z = va.z

        # TODO: make a valid matrix, this can be calc once in init
        # used variance set to 0.001, unused set to 1 (set ignore in config)
        omsg.twist.covariance = self.set_covariance(1e-3, 1e-3, 1, 1, 1, 1e-3)

        self.odometry_publisher.publish(omsg)


        # Update variables to use as the last (previous) encoder and pose info 
        if self.odMesssageCount != 0 :
            self.od_last = od
            self.xy_last = xy
            self.odMesssageCount += 1

            self.xyo_last = xyo


    #set diagnal of 6x6 covariance matrix 
    # returns array float64[36]
    def set_covariance(self,x,y,z,rx,ry,rz) :
        cv = np.zeros(36, dtype=np.float64)
        cv[0] = x
        cv[6+1] = y
        cv[2*(6+1)] = z
        cv[3*(6+1)] = rx
        cv[4*(6+1)] = ry
        cv[5*(6+1)] = rz
        return cv

    # def watch_json_callback(self, msg) ->None :
    #     #self.get_logger().info(f"wheel watch_json_callback {msg=}")
    #     packet_bytes = msg.data
    #     try :
    #         packet = json.loads(packet_bytes)
    #         # if (("motor" in packet) and ("action" in packet)):
    #             # motor = packet['motor']

    #     except Exception as ex:
    #         self.get_logger().error(f"wheel controller watch_json_callback exception {ex}")        


    CPmsg = ""

    def robo24_json_callback(self, msg) :
        #self.get_logger().info(f"{msg}")

        try :
            packet = json.loads(msg.data)
            #self.get_logger().info(f"{packet}")
            if "claw" in packet :
                claw_cmd = packet["claw"]
                if "open" in claw_cmd :
                    pct = claw_cmd["open"]
                if "time" in claw_cmd :
                    msec = claw_cmd["time"]
                self.wheel_serial_port.write(f"CP {pct} {msec}\n".encode())
                #self.get_logger().info(f"{packet}")

        except Exception as ex:
            self.get_logger().error(f"wheel controller robo24_json_callback exception {ex}")        

    # check serial port at timerRateHz and parse out messages to publish
    # TODO: actually parse the messages (currently only OD encoder messages)
    def timer_callback(self):
        # Check if a line has been received on the serial port
        try :
            if self.wheel_serial_port.in_waiting > 0:
                received_data = self.wheel_serial_port.readline().decode().strip()
                #self.get_logger().info(f"Received: {received_data}")
                
                # Publish the received serial line as a String message
                emsg = String()
                emsg.data = received_data
                # assume it is an OD encoder message
                self.encoders_msg_publisher.publish(emsg)

        except Exception as ex:
            self.get_logger().error(f"wheel controller timer serial read exception {ex}")
            self.wheel_serial_port.close()
            self.wheel_serial_port.open()
            return

        # Check to see if a serial message needs to be transmitted
        try:
            if self.CPmsg!="" :
                self.get_logger().info(f"Claw serial write in timer \"{self.CPmsg}\"\n".encode())
                self.wheel_serial_port.write(self.CPmsg)
                self.CPmsg=""

        except Exception as ex:
            self.get_logger().error(f"wheel controller timer serial write exception {ex}")

    def broadcast_tf(self, x, y, theta ):
        # Create and broadcast the transform message 
        tfs = TransformStamped()
        tfs.header.stamp = self.get_clock().now().to_msg()
        tfs.header.frame_id = "odom"
        tfs._child_frame_id = "base_link"
        tfs.transform.translation.x = x
        tfs.transform.translation.y = y
        tfs.transform.translation.z = 0.0 #theta # for debug should be 0.0  

        q = self.quaternion_from_euler(0.0, 0.0, theta) #0,0,theta

        tfs.transform.rotation.x = q[0]
        tfs.transform.rotation.y = q[1]
        tfs.transform.rotation.z = q[2]
        tfs.transform.rotation.w = q[3]

#        self.get_logger().info(f"Broadcast odom {tfs = }".encode())

        self.tf_broadcaster.sendTransform(tfs)    

    # simplified code for 2D robot
    def quaternion_from_euler(self, ai, aj, ak):
        ai /= 2.0
        aj /= 2.0
        ak /= 2.0
        ci = math.cos(ai)
        si = math.sin(ai)
        cj = math.cos(aj)
        sj = math.sin(aj)
        ck = math.cos(ak)
        sk = math.sin(ak)
        cc = ci*ck
        cs = ci*sk
        sc = si*ck
        ss = si*sk

        q = np.empty((4, ))
        q[0] = cj*sc - sj*cs
        q[1] = cj*ss + sj*cc
        q[2] = cj*cs - sj*sc
        q[3] = cj*cc + sj*ss

        return q

def main(args=None):
    rclpy.init(args=args)

    node = Robo24WheelControllerNode()
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()

# This code is needed to run .py file directly
if __name__ == '__main__':
    main()

