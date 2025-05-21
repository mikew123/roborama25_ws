#!/usr/bin/env python3

import rclpy
#import sys
import serial
import math
import time
import numpy as np

from rclpy.node import Node
from std_msgs.msg import String, Header
from sensor_msgs.msg import BatteryState
from sensor_msgs.msg import Temperature
from sensor_msgs.msg import Imu
from sensor_msgs.msg import PointCloud2, PointField, Range
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped, Transform

# create quanterion and inserts into transform (if r_rot is sent) also returns
def quaternion_from_euler(roll:float, pitch:float, yaw:float, t_rot:Transform.rotation=0):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = [0] * 4
    q[0] = cy * cp * cr + sy * sp * sr #w
    q[1] = cy * cp * sr - sy * sp * cr #x
    q[2] = sy * cp * sr + cy * sp * cr #y
    q[3] = sy * cp * cr - cy * sp * sr #z

    if t_rot != 0:
        t_rot.x = q[1]
        t_rot.y = q[2]
        t_rot.z = q[3]
        t_rot.w = q[0]
    
    return q   

class Roborama25SensorSerialNode(Node):
    # parameters?

    timerRateHz = 30.0; # Rate to check serial port for messages

    reflVal:int = 25 # TOF8x8 reflectance default = 25
    sigmVal:int = 40 # TOF8x8 sigma value default = 10

    #serial_port = "/dev/ttyACM2"
    serial_port:str = "/dev/serial/by-id/usb-Waveshare_RP2040_Zero_E6625C05E790A423-if00"

    def __init__(self):
        super().__init__('roborama25_sensor_serial_node')

        self.sensor_serial_port = serial.Serial(self.serial_port, 2000000)
        # configure interface
        self.sensor_serial_port.write(f"MODE ROS2\n".encode()) # extra write for startup
        self.sensor_serial_port.write(f"MODE ROS2\n".encode())
        self.sensor_serial_port.write(f"REFL {self.reflVal}\n".encode())
        self.sensor_serial_port.write(f"SIGM {self.sigmVal}\n".encode())


        self.tofL5L_pcd_publisher = self.create_publisher(PointCloud2, 'tofL5L_pcd', 10)
        self.tofL5R_pcd_publisher = self.create_publisher(PointCloud2, 'tofL5R_pcd', 10)
        self.tofL4_rng_publisher = self.create_publisher(Range, 'tofL4_rng', 10)
        self.tofL5_msg_publisher = self.create_publisher(String, 'tofL5_msg', 10)
        self.tofL4_msg_publisher = self.create_publisher(String, 'tofL4_msg', 10)
        self.IMU_msg_publisher = self.create_publisher(Imu, 'IMU', 10)
        self.CAL_msg_publisher = self.create_publisher(String, 'IMUCAL_msg', 10)
        self.battery_status_msg_publisher = self.create_publisher(BatteryState, 'battery_status', 10)
        self.temperature_msg_publisher = self.create_publisher(Temperature, 'temperature', 10)

        self.timer = self.create_timer((1.0/self.timerRateHz), self.timer_callback)

        #create TF static earth->map frame
        #self.map_tf_static_broadcaster = StaticTransformBroadcaster(self)
        #self.make_map_static_transform()
        
        #Create TF links earth->map->odom->base_link->lidar_link
        #                                    ->tofL4_link
        #                                    ->tofL5L_link
        #                                    ->tofL5R_link
        self.map_tf_broadcaster = TransformBroadcaster(self)
        self.odom_tf_broadcaster = TransformBroadcaster(self)
        self.base_link_tf_broadcaster = TransformBroadcaster(self)
        self.lidar_link_tf_broadcaster = TransformBroadcaster(self)
        self.tofL4_link_tf_broadcaster = TransformBroadcaster(self)
        self.tofL5L_link_tf_broadcaster = TransformBroadcaster(self)
        self.tofL5R_link_tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(1/10.0, self.broadcast_timer_callback)
        
        
        self.get_logger().info(f"SensorSerialNode Started")
        
    # def make_map_static_transform(self):
    #     t = TransformStamped()

    #     t.header.stamp = self.get_clock().now().to_msg()
    #     t.header.frame_id = "earth"
    #     t.child_frame_id = "map"

    #     t.transform.translation.x = 0.0
    #     t.transform.translation.y = 0.0
    #     t.transform.translation.z = 0.0
    #     t.transform.rotation.x = 0.0
    #     t.transform.rotation.y = 0.0
    #     t.transform.rotation.z = 0.0
    #     t.transform.rotation.w = 1.0

    #     self.map_tf_static_broadcaster.sendTransform(t)
        
    def zeroTransform(self, transform: Transform) :
        transform.translation.x = 0.0
        transform.translation.y = 0.0
        transform.translation.z = 0.0
        transform.rotation.x = 0.0
        transform.rotation.y = 0.0
        transform.rotation.z = 0.0
        transform.rotation.w = 1.0
        
    def broadcast_timer_callback(self):
        #Create TF links map->odom->base_link->lidar
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
                
        self.zeroTransform(t.transform)
        t.header.frame_id = 'earth'
        t.child_frame_id = 'map'
        self.map_tf_broadcaster.sendTransform(t)
        
        self.zeroTransform(t.transform)
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom'
        self.odom_tf_broadcaster.sendTransform(t)
        
        self.zeroTransform(t.transform)
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        self.base_link_tf_broadcaster.sendTransform(t)
        
        self.zeroTransform(t.transform)
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'lidar_link'
        # rotate 180 deg, raise 100mm
        t.transform.translation.z = 0.1
        # Simple numbers, calling the q function not needed
        t.transform.rotation.z = 1.0
        t.transform.rotation.w = 0.0
        self.lidar_link_tf_broadcaster.sendTransform(t)

        self.zeroTransform(t.transform)
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'tofL4_link'
        # TOF range detector is 100mm in front of center
        t.transform.translation.x = 0.1
        self.tofL4_link_tf_broadcaster.sendTransform(t)
        
        self.zeroTransform(t.transform)
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'tofL5L_link'
        # TOF range detector is offset x=100mm y=70mm theta=45/2
        t.transform.translation.x = 0.170
        t.transform.translation.y = 0.070
        yaw = +((45.0/2)+3.5)
        quaternion_from_euler(roll=0, pitch=0, yaw=math.pi*yaw/180, t_rot=t.transform.rotation)
        self.tofL5L_link_tf_broadcaster.sendTransform(t)
        
        self.zeroTransform(t.transform)
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'tofL5R_link'
        # TOF range detector is offset x=100mm y=70mm theta=-45/2
        t.transform.translation.x = 0.170
        t.transform.translation.y = -0.070
        yaw = -((45.0/2)+3.5)
        quaternion_from_euler(roll=0, pitch=0, yaw=math.pi*yaw/180, t_rot=t.transform.rotation)
        self.tofL5R_link_tf_broadcaster.sendTransform(t)

    # check serial port at timerRateHz and parse out messages to publish
    def timer_callback(self):
        # Check if a line has been received on the serial port
        if self.sensor_serial_port.in_waiting > 0:
            received_data = self.sensor_serial_port.readline().decode().strip()
            #self.get_logger().info(f"Received: {received_data}")
            
            strArray = received_data.split(" ")
            if strArray[0]=="L5" :
                self.L5_processing(strArray)
            elif strArray[0]=="L4" :
                self.L4_processing(strArray)
            elif strArray[0]=="IMU" :
                self.IMU_processing(strArray)
            elif strArray[0]=="CAL" :
                self.CAL_processing(strArray)
            elif strArray[0]=="BT":
                self.BT_processing(strArray)
            else :
                self.get_logger().error(f"Invalid serial sensor message {received_data=}")

    def L5_processing(self, strArray):
        num_data = 8*4
        if strArray[0]=="L5" and len(strArray)==1+num_data :
            # Publish the received serial line as a String message
            emsg = String()
            for i in range(1, num_data+1):
                emsg.data += strArray[i]+" "
            self.tofL5_msg_publisher.publish(emsg)

            # Publish the TOF distances as a point cloud
            fov = 45.0
            fovPt = fov/8 # FOV for each 8x8 sensor point
            fovPtRad = fovPt*(math.pi/180) #scaled to Radians
            # angle in degrees of each of the 2 sensors in pairs (LL,LR) and (Rl,RR)
            mntAngle = [fov/4, -fov/4]

            # There is a curvature in the distances of the sensors that needs to be corrected
            # Remove the curve by scaling each sensor with a inverted sin() curve over FOV
            # NOTE: This could be pre-computed outside the function since it is constant
            tofCurveCor = []
            for n in range(0,8) :
                theta:float = (n-4+0.5)*fovPtRad + math.pi/2
                s:float = math.sin(theta)
                if n == 0 : s0 = s
                tofCurveCor.append(s0/s)
            
            #convert string data to integer mm distance
            # break up into 4 sets of 8 for each sensor
            # data is in this order L to R LL[8] LR[8] RL[8] RR[8]
            dist:list[[]] = [[0 for x in range(8)]for y in range(4)]
            for s in range(0, 4):
                for i in range(0, 8):
                    d = int(strArray[(s*8)+i+1])
                    dist[s][i] = d
            #self.get_logger().info(f"{dist=}")
            
            xy0 = [[],[]]
            zz0 = 0.015 # 15mm from module bottom
            for m in [0,1]: # 2 sets of sensor modules L=0 R=1
                for s in [0,1]: # 2 Vl53L4 sensors on each module           
                    mntAngleRad = mntAngle[s]*(math.pi/(2*fov)) #scaled to Radians
                    for n in range(0, 8) :
                        theta = (n-4+0.5)*fovPtRad  - mntAngleRad # scaled to radians
                        d = dist[s+(2*m)][n]
                        if d==-1: 
                            # Bad data-Put point cloud at center of sensor module
                            xx0 = 0 # use INF ?
                            yy0 = 0
                        else :
                            Wx =  int(d*math.cos(theta)*tofCurveCor[n])
                            Wy = -int(d*math.sin(theta)*tofCurveCor[n])
                            #Convert mm to meters
                            xx0 = np.float32(Wx/1000.0)
                            yy0 = np.float32(Wy/1000.0)
                           
                        xy0[m].append((xx0,yy0,zz0))
                        
            #self.get_logger().info(f"{xy0=}")

            pcd = self.point_cloud(xy0[0], 'tofL5L_link')
            self.tofL5L_pcd_publisher.publish(pcd)
            pcd = self.point_cloud(xy0[1], 'tofL5R_link')
            self.tofL5R_pcd_publisher.publish(pcd)

            
    def L4_processing(self, strArray):
        num_data = 4
        if strArray[0]=="L4" and len(strArray)==1+num_data :
            # Publish the received serial line as a String message
            emsg = String()
            for i in range(1, num_data+1):
                emsg.data += strArray[i]+" "
            self.tofL4_msg_publisher.publish(emsg)
        
            # Create a single point point cloud 
            s = int(strArray[1]) #status
            d = int(strArray[2]) #distance mm
           
            d = d/1000.0
            fov = 2*math.pi*18.0/360
            
            rng = self.range_msg(d,fov,"tofL4_link")
            self.tofL4_rng_publisher.publish(rng)
            
        
    def IMU_processing(self, strArray):
        num_data = 11
        if strArray[0]=="IMU" and len(strArray)==1+num_data :
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

            self.IMU_msg_publisher.publish(msg)
        
    def CAL_processing(self, strArray):
        num_data = 4
        if strArray[0]=="CAL" and len(strArray)==1+num_data :
            # Publish the received serial line as a String message
            emsg = String()
            for i in range(1, num_data +1):
                emsg.data += strArray[i]+" "
            self.CAL_msg_publisher.publish(emsg)
            
    def BT_processing(self, strArray):
        if strArray[0]=="BT" and len(strArray)==4:
            volts:float = float(strArray[1])/1000
            amps:float = float(strArray[2])/1000
            tempC:float = float(strArray[3])
            # send Batter State message
            bmsg = BatteryState()
            bmsg.header.stamp = self.get_clock().now().to_msg()
            bmsg.header.frame_id = "base_link"
            bmsg.voltage = volts
            bmsg.current = amps
            bmsg.temperature = tempC
            bmsg.present = True
            self.battery_status_msg_publisher.publish(bmsg)
            # send Temperature message
            tmsg = Temperature()
            tmsg.header.stamp = self.get_clock().now().to_msg()
            tmsg.header.frame_id = "base_link"
            tmsg.temperature = tempC
            tmsg.variance = 0.0 # unknown
            self.temperature_msg_publisher.publish(tmsg)

    def range_msg(self, range:float=0, fov:float=0, parent_frame:str="map") -> Range:
        header = Header(
            frame_id = parent_frame,
            stamp = self.get_clock().now().to_msg(),
            )
        
        return Range(
            header = header,
            radiation_type = Range.INFRARED,
            field_of_view = fov,
            min_range = 0.010,
            max_range = 4.00,
            range = range
        )        
        
    def point_cloud(self, points_xy:list[tuple[np.float32]], parent_frame:str="map") -> PointCloud2:
        """
            Input list of tuples (x,y,z) the frame name for xy z is fixed relative offset usually "map"
            Returns a point cloud to publish - Rviz can display it
        """
        points = np.asarray(points_xy)

        ros_dtype = PointField.FLOAT32
        dtype = np.float32
        itemsize = np.dtype(dtype).itemsize # A 32-bit float takes 4 bytes.

        data = points.astype(dtype).tobytes() 

        # The fields specify what the bytes represents. The first 4 bytes 
        # represents the x-coordinate, the next 4 the y-coordinate
        fields = [PointField(
            name=n, offset=i*itemsize, datatype=ros_dtype, count=1)
            for i, n in enumerate('xyz')]

        #self.get_logger().info(f"{itemsize = } {fields = } {points = } {data = }")

        # The PointCloud2 message also has a header which specifies which 
        # coordinate frame it is represented in. 
        header = Header(
            frame_id=parent_frame,
            stamp = self.get_clock().now().to_msg(),
            )

        return PointCloud2(
            header=header,
            height=1, 
            width=points.shape[0],
            is_dense=False,
            is_bigendian=False, #Pi4
            fields=fields,
            point_step=(itemsize * 3), # Every point consists of two float32s.
            row_step=(itemsize * 3 * points.shape[0]), 
            data=data
        )


def main(args=None):
    rclpy.init(args=args)

    node = Roborama25SensorSerialNode()
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()

# This code is needed to run .py file directly
if __name__ == '__main__':
    main()

