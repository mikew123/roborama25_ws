#!/usr/bin/env python3
# MRW 5/26/2025 modified to be life cycle, changed name to match arduino 
# MRW 5/24/2025 create scan messages for reverse sensors when going backwards

import rclpy
import serial
import math
import time
import numpy as np
from urllib import response

from rclpy.node import Node
from std_msgs.msg import String, Float32, Header
from sensor_msgs.msg import BatteryState
from sensor_msgs.msg import Temperature
from sensor_msgs.msg import Imu
from sensor_msgs.msg import PointCloud2, PointField, Range, LaserScan
from geometry_msgs.msg import TransformStamped, Transform

from geometry_msgs.msg import Twist

from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle.node import LifecycleState, TransitionCallbackReturn

# create quanterion and inserts into transform (if r_rot is sent) also returns
# def quaternion_from_euler(roll:float, pitch:float, yaw:float, t_rot:Transform.rotation=0):
#     cy = math.cos(yaw * 0.5)
#     sy = math.sin(yaw * 0.5)
#     cp = math.cos(pitch * 0.5)
#     sp = math.sin(pitch * 0.5)
#     cr = math.cos(roll * 0.5)
#     sr = math.sin(roll * 0.5)

#     q = [0] * 4
#     q[0] = cy * cp * cr + sy * sp * sr #w
#     q[1] = cy * cp * sr - sy * sp * cr #x
#     q[2] = sy * cp * sr + cy * sp * cr #y
#     q[3] = sy * cp * cr - cy * sp * sr #z

#     if t_rot != 0:
#         t_rot.x = q[1]
#         t_rot.y = q[2]
#         t_rot.z = q[3]
#         t_rot.w = q[0]
    
#     return q   

class Roborama25FrontSensorsNodeLC(LifecycleNode):
    # parameters?

    timerRateHz = 30.0; # Rate to check serial port for messages

    reflVal:int = 50 # TOF8x8 reflectance default = 25 (> is good)
    sigmVal:int = 6 # TOF8x8 sigma value default = 10 (< is good)

    #serial_port = "/dev/ttyACM2"
    serial_port:str = "/dev/serial/by-id/usb-Waveshare_RP2040_Zero_E6625C05E790A423-if00"

    lifecycle_state_active = False

    movingBackward:bool = True # True if moving backwards, used for rear sensor scan messages
    
    def __init__(self):
        super().__init__('roborama25_front_sensors_node_lc')        

        self.get_logger().info(f"Roborama25FrontSensorsNodeLC Started")

    # Create ROS2 communications, connect to HW
    def on_configure(self, previous_state: LifecycleState):
        self.get_logger().info("IN on_configure")
        
        self.sensor_serial_port = serial.Serial(None, 2000000)

        self.serial_timer = self.create_timer((1.0/self.timerRateHz), self.serial_timer_callback)
        self.serial_timer.cancel()
        
        # self.broadcast_timer = self.create_timer(1/10.0, self.broadcast_timer_callback)
        # self.broadcast_timer.cancel()
        
        # There is no lifecycle support for subscription
        self.cmd_vel_subscription = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        self.tofRL_scan_publisher = self.create_lifecycle_publisher(LaserScan, 'tofRL_scan', 10)
        self.tofRC_scan_publisher = self.create_lifecycle_publisher(LaserScan, 'tofRC_scan', 10)
        self.tofRR_scan_publisher = self.create_lifecycle_publisher(LaserScan, 'tofRR_scan', 10)
        self.tofRL_rng_publisher = self.create_lifecycle_publisher(Range, 'tofRL_rng', 10)
        self.tofRC_rng_publisher = self.create_lifecycle_publisher(Range, 'tofRC_rng', 10)
        self.tofRR_rng_publisher = self.create_lifecycle_publisher(Range, 'tofRR_rng', 10)
        self.tofL4_rng_publisher = self.create_lifecycle_publisher(Range, 'tofL4_rng', 10)
        self.tofL5L_pcd_publisher = self.create_lifecycle_publisher(PointCloud2, 'tofL5L_pcd', 10)
        self.tofL5R_pcd_publisher = self.create_lifecycle_publisher(PointCloud2, 'tofL5R_pcd', 10)
        self.tofL4_pcd_publisher = self.create_lifecycle_publisher(PointCloud2, 'tofL4_pcd', 10)
        self.IMU_msg_publisher = self.create_lifecycle_publisher(Imu, 'IMU', 10)
        self.battery_status_msg_publisher = self.create_lifecycle_publisher(BatteryState, 'battery_status', 10)
        self.temperature_msg_publisher = self.create_lifecycle_publisher(Temperature, 'temperature', 10)
        
        #DEBUG publishers
        self.tofL5_msg_publisher = self.create_lifecycle_publisher(String, 'tofL5_msg', 10)
        self.tofL4_msg_publisher = self.create_lifecycle_publisher(String, 'tofL4_msg', 10)
        self.tofOPT_msg_publisher = self.create_lifecycle_publisher(String, 'tofOPT_msg', 10)
        self.CAL_msg_publisher = self.create_lifecycle_publisher(String, 'IMUCAL_msg', 10)

        return TransitionCallbackReturn.SUCCESS

    # Clean up stuff for cleanup, shutdown, error
    def cleanup_lc(self) :        
        self.destroy_lifecycle_publisher(self.tofRL_scan_publisher)
        self.destroy_lifecycle_publisher(self.tofRC_scan_publisher)
        self.destroy_lifecycle_publisher(self.tofRR_scan_publisher)
        self.destroy_lifecycle_publisher(self.tofRL_rng_publisher)
        self.destroy_lifecycle_publisher(self.tofRC_rng_publisher)
        self.destroy_lifecycle_publisher(self.tofRR_rng_publisher)
        self.destroy_lifecycle_publisher(self.tofL4_rng_publisher)
        self.destroy_lifecycle_publisher(self.tofL5L_pcd_publisher)
        self.destroy_lifecycle_publisher(self.tofL5R_pcd_publisher)
        self.destroy_lifecycle_publisher(self.tofL4_pcd_publisher)
        self.destroy_lifecycle_publisher(self.IMU_msg_publisher)
        self.destroy_lifecycle_publisher(self.battery_status_msg_publisher)
        self.destroy_lifecycle_publisher(self.temperature_msg_publisher)
        
        self.destroy_lifecycle_publisher(self.tofL5_msg_publisher)
        self.destroy_lifecycle_publisher(self.tofL4_msg_publisher)
        self.destroy_lifecycle_publisher(self.tofOPT_msg_publisher)
        self.destroy_lifecycle_publisher(self.CAL_msg_publisher)
        
    def cleanup(self) :                
        # self.destroy_timer(self.broadcast_timer)
        self.destroy_timer(self.serial_timer)
        self.sensor_serial_port=None
        # There is no lifecycle subscription
        self.destroy_subscription(self.cmd_vel_subscription)   

    # Destroy ROS2 communications, disconnect from HW
    def on_cleanup(self, previous_state: LifecycleState):
        self.get_logger().info("IN on_cleanup")
        self.cleanup_lc()
        self.cleanup()
        return TransitionCallbackReturn.SUCCESS

    # Activate/Enable HW
    def on_activate(self, previous_state: LifecycleState):
        self.get_logger().info("IN on_activate")
        self.serial_timer.reset()

        self.sensor_serial_port.port = self.serial_port
        self.sensor_serial_port.open() #= serial.Serial(self.serial_port, 2000000)
        # configure interface
        self.sensor_serial_port.write(f"MODE ROS2\n".encode()) 
        self.sensor_serial_port.write(f"MODE ROS2\n".encode()) # extra write for startup
        self.sensor_serial_port.write(f"REFL {self.reflVal}\n".encode())
        self.sensor_serial_port.write(f"SIGM {self.sigmVal}\n".encode())
        #self.sensor_serial_port.write(f"OPHZ 16\n".encode()) #rear sensor data rate
        self.sensor_serial_port.flush()

        self.lifecycle_state_active = True
        return super().on_activate(previous_state)

    # Deactivate stuff used in shutdown, error
    def deactivate(self):
        self.lifecycle_state_active = False
        # self.broadcast_timer.cancel()
        self.serial_timer.cancel()
        self.sensor_serial_port.close()
        
    # Deactivate/Disable HW
    def on_deactivate(self, previous_state: LifecycleState):
        self.get_logger().info("IN on_deactivate")
        self.deactivate()
        return super().on_deactivate(previous_state)
    
    # Cleanup everything
    def shutdown(self, previous_state: LifecycleState):
        if(previous_state.label != "unconfigured") :
            self.deactivate()        
            self.cleanup()
        
    def on_shutdown(self, previous_state: LifecycleState):
        self.get_logger().info(f"IN on_shutdown from {previous_state=}")
        self.shutdown(previous_state)
        return TransitionCallbackReturn.SUCCESS
    
    # Process errors, deactivate + cleanup
    def on_error(self, previous_state: LifecycleState):
        self.get_logger().info(f"IN on_error from {previous_state=}")
        self.shutdown(previous_state)
        # do some checks, if ok, then return SUCCESS, if not FAILURE
        return TransitionCallbackReturn.FAILURE

    # check serial port at timerRateHz and parse out messages to publish
    def serial_timer_callback(self):
        # Check if a line has been received on the serial port
        if self.sensor_serial_port.in_waiting > 0:
            received_data = self.sensor_serial_port.readline().decode().strip()
            #self.get_logger().info(f"Received: {received_data}")
            
            strArray = received_data.split(" ")
            if strArray[0]=="L5" :
                self.L5_processing(strArray)
            elif strArray[0]=="L4" :
                self.L4_processing(strArray)
            elif strArray[0]=="OPT" :
                self.OPT_processing(strArray)
            elif strArray[0]=="IMU" :
                self.IMU_processing(strArray)
            elif strArray[0]=="CAL" :
                self.CAL_processing(strArray)
            elif strArray[0]=="BT":
                self.BT_processing(strArray)
            else :
                self.get_logger().error(f"Invalid serial sensor message {received_data=}")

    def cmd_vel_callback(self, msg):
        """
        Get status of fwd/rev motion
        """
        if not self.lifecycle_state_active : return
        linear_velocity = msg.linear.x
        self.movingBackward = linear_velocity<-0.0001
        
        

    def L5_processing(self, strArray):
        num_data = 8*4
        trim_dist = 1.07 # trim the TOF distance to match Lidar
        if strArray[0]=="L5" and len(strArray)==1+num_data :
            try :
                # DEBUG: Publish the received serial line as a String message
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
                
                # Adjust for angle of each sensor in the array
                
                #convert string data to integer mm distance
                # break up into 4 sets of 8 for each sensor
                # data is in this order L to R LL[8] LR[8] RL[8] RR[8]
                dist:list[[]] = [[0 for x in range(8)]for y in range(4)]
                for s in range(0, 4):
                    for i in range(0, 8):
                        d = int(strArray[(s*8)+i+1])
                        if d==-1 : dist[s][i] = -1
                        else : dist[s][i] = d * trim_dist
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
                                # Bad data - set as infinate number (use NaN instead?)
                                xx0 = math.inf
                                yy0 = math.inf
                            else :
                                Wx =  int(d*math.cos(theta)*tofCurveCor[n])
                                Wy = -int(d*math.sin(theta)*tofCurveCor[n])
                                # Convert mm to meters
                                xx0 = np.float32(Wx/1000.0)
                                yy0 = np.float32(Wy/1000.0)
                            
                            xy0[m].append((xx0,yy0,zz0))
                            
                # self.get_logger().info(f"{xy0=}")

                pcd = self.point_cloud(xy0[0], 'tofL5L_link')
                self.tofL5L_pcd_publisher.publish(pcd)
                pcd = self.point_cloud(xy0[1], 'tofL5R_link')
                self.tofL5R_pcd_publisher.publish(pcd)
                
            except Exception as e:
                self.get_logger().error(f"L5_processing: Error in L5 message {e=} {strArray=}")

    def L4_processing(self, strArray):
        num_data = 4
        if strArray[0]=="L4" and len(strArray)==1+num_data :
            try :
                # Publish the received serial line as a String message
                emsg = String()
                for i in range(1, num_data+1):
                    emsg.data += strArray[i]+" "
                self.tofL4_msg_publisher.publish(emsg)
            
                # Create a single point range message 
                s = int(strArray[1]) #status
                d = int(strArray[2]) #distance mm
            
                # check data status - 0 is OK
                if s==0 : d = d/1000.0 # convert mm to meters
                else    : d = math.inf # Use NaN instead?
                fov = 2*math.pi*18.0/360
                
                rng = self.range_msg(d,fov,"tofL4_link")
                self.tofL4_rng_publisher.publish(rng)

                # Create a point cloud to visualize in Foxglove
                pcd = self.point_cloud([d, 0, 0], 'tofL4_link')                
                self.tofL4_pcd_publisher.publish(pcd)

                # self.get_logger().info(f"L4_processing: {s=} {d=} {rng=} {pcd=} {strArray=}")
                
            except Exception as e:
                self.get_logger().error(f"L4_processing: Error in L4 message {e=} {strArray=}")
            
    def OPT_processing(self, strArray):
        """
        Process the 3 OPT3101 rear distance sensors
        """
        num_data = 6
        min_amp = 100
        max_dist = 500
        invalid_dist = -1.0
        if strArray[0]=="OPT" and len(strArray)==1+num_data :
            try :
                # Publish the received serial line as a String message
                emsg = String()
                for i in range(1, num_data+1):
                    emsg.data += strArray[i]+" "
                self.tofOPT_msg_publisher.publish(emsg)
            
                # Create 3 range messages 
                # Rear Right sensor
                a = int(strArray[1]) #amplitude
                d = int(strArray[2]) #distance mm
                if a>=min_amp and d<=max_dist:
                    dR = d/1000.0
                else :
                    dR =  invalid_dist
                fovR = 2*math.pi*50.0/360
                rng = self.range_msg(dR,fovR,"tofRR_link")
                self.tofRR_rng_publisher.publish(rng)

                # Rear Center sensor
                a = int(strArray[3]) #amplitude
                
                d = int(strArray[4]) #distance mm
                if a>=min_amp and d<=max_dist:
                    dC = d/1000.0
                else :
                    dC =  invalid_dist
                fovC = 2*math.pi*50.0/360
                rng = self.range_msg(dC,fovC,"tofRC_link")
                self.tofRC_rng_publisher.publish(rng)

                # Rear Left sensor
                a = int(strArray[5]) #amplitude
                d = int(strArray[6]) #distance mm
                if a>=min_amp and d<=max_dist:
                    dL = d/1000.0
                else :
                    dL =  invalid_dist
                fovL = 2*math.pi*50.0/360
                rng = self.range_msg(dL,fovL,"tofRL_link")
                self.tofRL_rng_publisher.publish(rng)

                # self.get_logger().error(f"OPT_processing: {strArray=}")
            
                # Create scan messages for front center while moving backwards
                if self.movingBackward==True:
                    scan = self.scan_msg_from_range(dL, fovL, "tofRL_link")
                    self.tofRL_scan_publisher.publish(scan)
                    scan = self.scan_msg_from_range(dC, fovC, "tofRC_link")
                    self.tofRC_scan_publisher.publish(scan)
                    scan = self.scan_msg_from_range(dR, fovR, "tofRR_link")
                    self.tofRR_scan_publisher.publish(scan)
                    
            except Exception as e:
                self.get_logger().error(f"OPT_processing: Error in OPT message {e=} {strArray=}")
                
                
    def IMU_processing(self, strArray):
        num_data = 11
        if strArray[0]=="IMU" and len(strArray)==1+num_data :
            try :
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
                #self.get_logger().info(f"IMU_processing: {msg=}")
            except Exception as e:
                self.get_logger().error(f"IMU_processing: Error in IMU message {e=} {strArray=}")

    def CAL_processing(self, strArray):
        num_data = 4
        if strArray[0]=="CAL" and len(strArray)==1+num_data :
            try :
                # Publish the received serial line as a String message
                emsg = String()
                for i in range(1, num_data +1):
                    emsg.data += strArray[i]+" "
                self.CAL_msg_publisher.publish(emsg)
                
            except Exception as e:
                self.get_logger().error(f"CAL_processing: Error in CAL message {e=} {strArray=}")
            
    def BT_processing(self, strArray):
        """
        Process the message from the battery monitor
            BT <volts> <amps> <tempC>
        """
        if strArray[0]=="BT" and len(strArray)==4:
            try :
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
                
            except Exception as e:
                self.get_logger().error(f"BT_processing: Error in BT message {e=} {strArray=}")

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


    def scan_msg_from_range(self, range:float=0, fov:float=0, parent_frame:str="map") -> LaserScan:
        """
        Create a scan message from range data with points spaced about 10 degrees apart
        """
        scan = LaserScan()
        
        scan.header = Header(
            frame_id = parent_frame,
            stamp = self.get_clock().now().to_msg(),
            )
        scan.range_min= 0.010
        scan.range_max= 1.00
        np = fov/(10*(math.pi/180)) # number of points
        scan.angle_min = -fov/2
        scan.angle_max = fov/2
        scan.angle_increment = fov/np
        a = scan.angle_min
        while a < scan.angle_max:
            scan.ranges.append(range)
            a += scan.angle_increment
            
        return scan
        
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

    node = Roborama25FrontSensorsNodeLC()
    #rclpy.spin(node)
    # MultiThread for life cycle operation
    rclpy.spin(node, MultiThreadedExecutor()) 
    
    node.destroy_node()
    rclpy.shutdown()

# This code is needed to run .py file directly
if __name__ == '__main__':
    main()

