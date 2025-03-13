import rclpy
from rclpy.node import Node
import sys

import numpy as np
import math

from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry

from std_msgs.msg import String, Header
from sensor_msgs.msg import PointCloud2, PointField
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from tf2_ros import Duration

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

from geometry_msgs.msg import TransformStamped

from numpy.polynomial import Polynomial
import json

class Robo24DiySlamNode(Node):
    """
    Creates the 6-can arena maps for home and dprg - rviz2 can display it
    Creates a point cloud from TOF sensors
    Rviz2 can display the map and point cloud
    """
    
    # 24 data points for line of sensors in mm
    tof8Wall:list[list[int]] = [[]]
    pcd = PointCloud2()

    # Field of view for TOF 8x8 sensor
    fov8x8:float = 45.0
    mntAngle:float = 45.0

    # map->odom TF filter accumulated corrections
    slamTc:float = 1.0/3 # filter time constant
    mapOdomX:float = 0.0
    mapOdomY:float = 0.0
    mapOdomT:float = 0.0

    # map->odom point cloud data to use
    mapOdomPcd:list[np.float32, np.float32, np.float32] = []
    # handshake for PCD data to ensure it is stable and current
    mapOdomPcdReq = False
    mapOdomPcdRdy = False

    ft2m:float = 0.3048 # feet to meters

    mapResolution:float = 0.1 # pixels = 10 cm sq
    
    home_can6Width:int = int(((7.0+(7/12.0)) * ft2m)/mapResolution) # 6-can walls
    home_can6Height:int = int(((6.0+(10/12.0)) * ft2m)/mapResolution)
    home_can6GoalArea:int = int((2 * ft2m)/mapResolution) # goal area outside walls
    home_can6GoalOpening:int = int((3 * ft2m)/mapResolution) #width of goal openin
    home_mapWidth:int = home_can6Width + home_can6GoalArea
    home_mapHeight:int = home_can6Height
    home_startWpX0:int = (8/12.0*ft2m) # offset from back wall, inches to meters

    home_arena: dict = {
        "can6Width"       : home_can6Width,
        "can6Height"      : home_can6Height,
        "can6GoalArea"    : home_can6GoalArea,
        "can6GoalOpening" : home_can6GoalOpening,
        "mapWidth"        : home_mapWidth,
        "mapHeight"       : home_mapHeight,
        "startWpX0"       : home_startWpX0
    }

    dprg_can6Width:int = int((10.0 * ft2m)/mapResolution) # 6-can walls
    dprg_can6Height:int = int((7.0 * ft2m)/mapResolution)
    dprg_can6GoalArea:int = int((2 * ft2m)/mapResolution) # goal area outside walls
    dprg_can6GoalOpening:int = int((3 * ft2m)/mapResolution) #width of goal openin
    dprg_mapWidth:int = dprg_can6Width + dprg_can6GoalArea
    dprg_mapHeight:int = dprg_can6Height
    dprg_startWpX0:int = (8/12.0*ft2m) # offset from back wall, inches to meters

    dprg_arena: dict = {
        "can6Width"       : dprg_can6Width,
        "can6Height"      : dprg_can6Height,
        "can6GoalArea"    : dprg_can6GoalArea,
        "can6GoalOpening" : dprg_can6GoalOpening,
        "mapWidth"        : dprg_mapWidth,
        "mapHeight"       : dprg_mapHeight,
        "startWpX0"       : dprg_startWpX0
    }

    arenas = {
        "home" : home_arena,
        "dprg" : dprg_arena
    }


    nav_arena:str = "dprg"

    diyslamEnabled = True

    def __init__(self):
        super().__init__('robo24_diyslam_node')

        # Parameters
        #self.declare_parameter('enable_slam', True) # default to slam enabled

        # TOF sensors to extract point cloud from
        self.tof8x8x3_subscription = self.create_subscription(String, 'tof8x8x3_msg', self.tof8x8x3_callback, 10)
 
        # POINT CLOUD from TOF data
        self.pcd_publisher = self.create_publisher(PointCloud2, 'tof8_pcd', 10)
        # POINT CLOUD selected for SLAM
        self.slam_pcd_publisher = self.create_publisher(PointCloud2, 'slam_pcd', 10)
         # Areana simple map
        self.map_msg_publisher = self.create_publisher(OccupancyGrid, 'map', 10)

        # subscribe to wheel odometry to determine velocity == 0
        # it also generates the map->odom TF using point cloud data when Vel==0
        self.wheel_odom_subscription = self.create_subscription(Odometry, 'wheel_odom', self.wheel_odom_callback, 10)

        self.robo24_modes_subscription = self.create_subscription(String, 'robo24_modes', self.robo24_modes_callback, 10)
        self.robo24_json_subscription = self.create_subscription(String, 'robo24_json', self.robo24_json_callback, 10)

        self.tf_buffer = Buffer(Duration(seconds=0,nanoseconds=0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # publish the map 1/sec
        self.map_timer = self.create_timer(1.0, self.on_map_timer)

        # timer for generating map->odom TF
        #self.map_odom_timer = self.create_timer(1/10.0, self.on_map_odom_timer)

        # TF2 for map->odom drift correction
        self.tf_static_broadcasterOdom = StaticTransformBroadcaster(self)
        self.make_static_tf(self.tf_static_broadcasterOdom, "map", "odom", [0.0,0.0,0.0])

        self.get_logger().info("Robo24 DIY Slam Started")

    def process_nav_cmd(self, cmd) :
        self.get_logger().info(f"process_nav_cmd {cmd=}")
        if "arena" in cmd :
            arena = cmd["arena"]
            self.nav_arena = arena
            self.get_logger().info(f"{self.nav_arena=} {cmd=}")

    def robo24_json_callback(self, msg) :
        cmd = json.loads(msg.data)
        self.get_logger().info(f"process_nav_cmd {cmd=}")
        try :
            packet = json.loads(msg.data)
        except Exception as ex:
            self.get_logger().error(f"diyslam robo24_json_callback exception {ex}")
            return

        # if "arena" in packet :
        #     self.nav_arena = packet["arena"]
        #     self.get_logger().info(f"{self.nav_arena=} {msg=}")
        if "nav_cmd" in packet :
            nav_cmd = packet["nav_cmd"]
            self.process_nav_cmd(nav_cmd)
            self.get_logger().info(f"{self.nav_arena=} {msg=}")


    # robo24_modes topic to control diyslamEnabled
    def robo24_modes_callback(self, msg) :
        self.get_logger().info(f"robo24_modes_callback {msg=}")

        try:
            #split msg string
            tofStrArray = msg.data.split(" ")

            # message is 2 strings
            if len(tofStrArray)!=2:
                self.get_logger().error(f"robo24_modes type message error: {msg.data}")
                return
        except:
            self.get_logger().error(f"robo24_modes parse message error: {msg.data}")
            return

        # Parse for SLAM ON/OFF
        if(tofStrArray[0]=="SLAM" and tofStrArray[1]=="ON") :
            self.diyslamEnabled = True
        if(tofStrArray[0]=="SLAM" and tofStrArray[1]=="OFF") :
            self.diyslamEnabled = False

    def wheel_odom_callback(self, msg: Odometry) -> None :
        #self.get_logger().info(f"wheel_odom_callback")

        #diyslamEnabled:bool = self.get_parameter('enable_slam').get_parameter_value().bool_value
        if  self.diyslamEnabled == False:
            self.make_static_tf(self.tf_static_broadcasterOdom, "map", "odom", [0.0, 0.0, 0.0])
            return

        # Use velocity data in Odometry to determine when robot not moving
        # and point cloud is stable
        robotNotMoving = False
        vX:float = msg.twist.twist.linear.x
        aZ:float = msg.twist.twist.angular.z

        # angle offset using left and right walls since they have lots of points
        angleOffsetL:float = 0.0
        #polyCoeffL:Polynomial = 0
        linOffsetLy:float = 0.0

        angleOffsetR:float = 0.0
        #polyCoeffR:Polynomial = 0
        linOffsetRy:float = 0.0

        angleOffsetF:float = 0.0
        linOffsetFx:float = 0.0

        angleOffsetB:float = 0.0
        linOffsetBx:float = 0.0

        wallPointsLx:list[float] = [] 
        wallPointsLy:list[float] = [] 
        wallPointsRx:list[float] = [] 
        wallPointsRy:list[float] = [] 
        wallPointsFx:list[float] = [] 
        wallPointsFy:list[float] = [] 
        wallPointsBx:list[float] = [] 
        wallPointsBy:list[float] = [] 
 
#        if (math.fabs(vX)<0.001 and math.fabs(aZ)<0.001) :
        if (math.fabs(vX)<0.05 and math.fabs(aZ)<0.005) :
            robotNotMoving = True

        if robotNotMoving : 
            if (self.mapOdomPcdReq==False and self.mapOdomPcdRdy==False) : 
                self.mapOdomPcdReq = True

        arena = self.nav_arena
        if not arena in self.arenas : return

        mapResolution   = self.mapResolution
        can6Height      = self.arenas[arena]["can6Height"]
        can6Width       = self.arenas[arena]["can6Width"]
        startWpX0       = self.arenas[arena]["startWpX0"]


        if self.mapOdomPcdRdy == True :
            self.mapOdomPcdRdy = False
            # create seperate X Y lists for Mean calc and Poloynomial.fit()
            can6Y:float = can6Height*mapResolution
            can6X:float = can6Width*mapResolution
            if robotNotMoving :
                # gather points in PCD (x,y,z) that align with the walls
                # Left wall Y = +1.04 Right wall Y= -1.04
                for xyz in self.mapOdomPcd :
                    x = float(xyz[0])
                    y = float(xyz[1])
                    z = float(xyz[2])
                    
                    offset = 0.25

                    # points on Left side wall
                    if (math.fabs(can6Y/2-y)<0.25) and x>=0 and x<=can6X-(2*0.2) :
                        wallPointsLx.append(x)
                        wallPointsLy.append(y)
                    # points on Right side wall
                    if (math.fabs(-can6Y/2-y)<0.25) and x>=0 and x<=can6X-(2*0.2) :
                        wallPointsRx.append(x)
                        wallPointsRy.append(y)
                    # points on front goal wall
                    if (math.fabs((can6X - startWpX0) -x)<0.2) and (y > -(can6Y/2-0.2)) and (y < (can6Y/2-0.2)) :
                        wallPointsFx.append(x)
                        wallPointsFy.append(y)
                    # points on back non-goal wall
                    if (math.fabs(-startWpX0 -x)<0.2) and (y > -(can6Y/2-0.2)) and (y < (can6Y/2-0.2)) :
                        wallPointsBx.append(x)
                        wallPointsBy.append(y)

                wallPos = can6Y/2
                (linOffsetLy, angleOffsetL) = self.slamOffsets(wallPointsLx, wallPointsLy, wallPos, -1)

                wallPos = can6Y/2
                (linOffsetRy, angleOffsetR) = self.slamOffsets(wallPointsRx, wallPointsRy, wallPos, +1)

                wallPos = can6X - startWpX0
                (linOffsetFx, angleOffsetF) = self.slamOffsets(wallPointsFy, wallPointsFx, wallPos, -1)

                wallPos = -startWpX0
                #(linOffsetBx, angleOffsetB) = self.slamOffsets(wallPointsBy, wallPointsBx, wallPos, -1)
                (linOffsetBx, angleOffsetB) = self.slamOffsets(wallPointsBy, wallPointsBx, wallPos, -1)

                # NOTE: angle offsets only from left and right walls
                #self.mapOdomX += (linOffsetFx+linOffsetFx)*self.slamTc
                self.mapOdomX += (linOffsetFx+linOffsetBx)*self.slamTc
                self.mapOdomY += (linOffsetLy-linOffsetRy)*self.slamTc
                self.mapOdomT -= (angleOffsetL+angleOffsetR)*self.slamTc
            
            # self.get_logger().info(f"{self.mapOdomPcd = }\n")
            # self.get_logger().info(f"LEFT: {can6Y = } {wallPointsLx = } {wallPointsLy = } {polyCoeffL = } {angleOffsetL = } {self.mapOdomT = } {linOffsetLy = } {self.mapOdomY = } \n")
            # self.get_logger().info(f"RIGHT: {can6Y = } {wallPointsRx = } {wallPointsRy = } {polyCoeffR = } {angleOffsetR = } {self.mapOdomT = } {linOffsetRy = } {self.mapOdomY = } \n")
            # self.get_logger().info(f"FRONT: {can6X = } {self.startWpX0 = } {wallPointsFx = } {wallPointsFy = } {linOffsetFx = } {self.mapOdomX = } \n")
            # self.get_logger().info(f"BACK: {self.startWpX0 = } {wallPointsBx = } {wallPointsBy = } {linOffsetBx = } {self.mapOdomX = } \n\n")
            # self.get_logger().info(f"{self.mapOdomX=} {self.mapOdomY=} {self.mapOdomT=}")


                #if diyslamEnabled :
                xyt:list[float] = [self.mapOdomX, self.mapOdomY, self.mapOdomT]
                self.make_static_tf(self.tf_static_broadcasterOdom, "map", "odom", xyt)

                # create point cloud of data selected for SLAM
                xy0:list[np.float32, np.float32, np.float32] = []
                if (len(wallPointsLx)>=3): 
                    for x,y in zip(wallPointsLx,wallPointsLy): xy0.append((x,y,0.2))
                if (len(wallPointsRx)>=3): 
                    for x,y in zip(wallPointsRx,wallPointsRy): xy0.append((x,y,0.2))
                if (len(wallPointsFx)>=3): 
                    for x,y in zip(wallPointsFx,wallPointsFy): xy0.append((x,y,0.2))
                if (len(wallPointsBx)>=3): 
                    for x,y in zip(wallPointsBx,wallPointsBy): xy0.append((x,y,0.2))
                slam_pcd = self.point_cloud(xy0, 'map')
                self.slam_pcd_publisher.publish(slam_pcd)

    def slamOffsets(self, ptsX:list[float], ptsY:list[float], wallPos:float, mpy:int) -> tuple[float]:
        """
            ptsX[] is locations along the wall
            ptsY[] is the distance to the wall
            for Front and back ptsY is the distance data
            For front and back ptsX is the location data
            wallPos is the X or Y position of the wall from the origin 0,0
            Returns (linOffset, angleOffset)
        """
        
        # if values are duplicates then return zero tuple - would cause Polynomial.fit to fail
        if len(ptsX)>=2 and len(ptsY)>=2 :
            if (ptsX[0]==ptsX[1] and ptsY[0]==ptsY[1]) :
                return (0.0, 0.0)

        # remove outlier based on poly fit
        A:float = 0.0
        B:float = 0.0
        linOffset:float = 0.0
        angleOffset:float = 0.0

        # remove end point outliers but stop when less than 3 points left
        while len(ptsX) >= 3 :
            # calc offset angle and distance using a polynomial calc
            try :
                polyFit:Polynomial = Polynomial.fit(ptsX, ptsY, 1)
                (A,B) = polyFit.convert().coef
            except Exception as ex :
                self.get_logger().error(f"Polynomial exception {ptsX=} {ptsY=} {ex=}")
                #sys.exit()

            meanErr:float = 0  # mean error of entire list
            meanErrB:float = 0 # mean error - beginning point
            meanErrE:float = 0 # mean error - end point
            l = len(ptsX)
            for i in range(0,l) :
                Y = A + B*ptsX[i]
                err = Y - ptsY[i]
                absErr = math.fabs(err)
                meanErr += absErr
                if i!=0 : meanErrB += absErr
                if i!=l-1 : meanErrE += absErr
            meanErr = meanErr/l
            meanErrB = meanErrB/(l-1)
            meanErrE = meanErrE/(l-1)

            #self.get_logger().info(f"{l=} {meanErr=} {meanErrB=} {meanErrE=} {ptsX}")
            if meanErr < 0.01 :
                break

            if meanErrB<meanErrE :
                if meanErrB < 0.95*meanErr : 
                    ptsX.pop(0) # remove begining point
                    ptsY.pop(0)
                else : break
            else :
                if meanErrE < 0.95*meanErr : 
                    ptsX.pop(l-1) # remove end point
                    ptsY.pop(l-1)
                else : break

        if len(ptsX) >= 3 :
            angleOffset = math.atan(B)
            # Y offset from Left side wall
            meanWallPoints = sum(ptsY)/len(ptsY) # mean
            linOffset = wallPos + mpy*meanWallPoints

        return (linOffset,angleOffset)

    # xyt [x,y,theta]
    def make_static_tf(self, tf_static_broadcaster, 
                       parent: str, child: str, xyt: list) -> None:
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = parent
        t.child_frame_id = child

        t.transform.translation.x = xyt[0]
        t.transform.translation.y = xyt[1]
        t.transform.translation.z = 0.0
        quat = quaternion_from_euler(0.0, 0.0, xyt[2]) #x,y,theta
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]

        #self.tf_static_broadcaster.sendTransform(t)
        tf_static_broadcaster.sendTransform(t)

    # get TOF8x8x3 sensors [[[12,5],[13,5]], [[12,6],[13,6]], [[12,7],[13,7]]]
    def tof8x8x3_callback(self, msg: String) -> None:

        now = rclpy.time.Time() # Gets time=0 (I think simulation time)

        try:
            #split msg string into 193 seperate strings 1 for each element
            tofStrArray = msg.data.split(" ")

            # parse messsage of "name" then 192 integers (8 rows of 24 distances)
            if tofStrArray[0]!="TOF8x8x3" or len(tofStrArray)!=193:
                self.get_logger().error(f"TOF8x8x3 type message error: {msg.data}")
                return
        except:
            self.get_logger().error(f"TOF8x8x3 parse message error: {msg.data}")
            return

        # copy 1D sensor list into a 2D array
        for row in range(0,7) : # Rows
            for i in range(0,24): # Collumns
                dist = int(tofStrArray[i+(row*24)+1]) #[i,sensorRow]
                if dist>=0 : self.tof8Wall[row][i] = dist
                else       : self.tof8Wall[row][i] = 0

        # Remove the curve by scaling each sensor with a inverted sin() curve over FOV
        fovPt = self.fov8x8/8 # FOV for each sensor point
        fovPtRad = fovPt*(math.pi/180) #scaled to Radians
        tofCurveCor = []
        for n in range(0,8) :
            theta:float = (n-4+0.5)*fovPtRad + math.pi/2
            s:float = math.sin(theta)
            if n == 0 : s0 = s
            tofCurveCor.append(s0/s)

        #self.get_logger().info(f"{tofCurveCor = }")

        # calc xy coordinates relative to robot center with 0 deg pointing staight ahead
        # each sensor distance data point has an effective FOV of 60/8 = 7.5 deg
        # there are 24 data points n = 0, 1 to 23
        # theta = (60/8)*1/2 + (n-12)*(60/8) -> 0=-86.25 23=+86.25, 11=-3.75, 12=+3.75
        xyW:list = []
        mntAngleRad = self.mntAngle*(math.pi/180) #scaled to Radians
        fovPt = self.fov8x8/8 # FOV for each sensor point
        fovPtRad = fovPt*(math.pi/180) #scaled to Radians

        # min distance to save TOF data point for SLAM
        self.slamMinDist = 400

        # calc curve correction for each sensor set of 8
        # Left sensor 0 to 7
        for n in range(0,8) :
            theta = (n-4+0.5)*fovPtRad  - mntAngleRad# scaled to radians
            dist = self.tof8Wall[4][n]
            Wx =  int(dist*math.cos(theta)*tofCurveCor[n])
            Wy = -int(dist*math.sin(theta)*tofCurveCor[n])
            if dist > self.slamMinDist : xyW.append((Wx,Wy))
            else          : xyW.append((0,0))
        # Center sensor 8 to 15
        for n in range(8,16) :
            theta = (n-12+0.5)*fovPtRad# scaled to radians
            dist = self.tof8Wall[4][n]
            Wx =  int(dist*math.cos(theta)*tofCurveCor[n-8])
            Wy = -int(self.dist*math.sin(theta)*tofCurveCor[n-8])
            # discard sensor point if too close 
            if dist > self.slamMinDist : xyW.append((Wx,Wy))
            else          : xyW.append((0,0))
        # Right sensor 16 to 23
        for n in range(16,24) :
            theta = (n-20+0.5)*fovPtRad  + mntAngleRad# scaled to radians
            dist = self.tof8Wall[4][n]
            Wx =  int(dist*math.cos(theta)*tofCurveCor[n-16])
            Wy = -int(dist*math.sin(theta)*tofCurveCor[n-16])
            # discard sensor point if too close 
            if dist > self.slamMinDist : xyW.append((Wx,Wy))
            else          : xyW.append((0,0))

        #self.get_logger().info(f"\n{self.tof8Wall = }\n{xyW = }\n")

        # Create point cloud from TOF8 data
        # get map->base_link transform to translate XY coordinates of the wall to align with map
        try:
            t0 = self.tf_buffer.lookup_transform (
                'map',
                'base_link',
                now,
                timeout=rclpy.duration.Duration(seconds=0.0)
                )
            tf_OK = True

        except (LookupException, ConnectivityException, ExtrapolationException) as ex:
            self.get_logger().info(f'Could not transform map->base_link - will try again: {ex}')
            tf_OK = False

        # translate wall points to align with map coordinates
        if tf_OK :
            # get x, y, theta from TF
            x0:float = t0.transform.translation.x
            y0:float = t0.transform.translation.y
            q0:float = t0.transform.rotation
            # convert quaterion to euler
            e0:tuple = euler_from_quaternion(q0.x, q0.y, q0.z, q0.w)
            th0:float = -e0[2] # yaw theta
            
            c0:float = math.cos(th0)
            s0:float = math.sin(th0)

            #Create list of XYZ tupples converting int mm to float meters
            xy_:list[np.float32, np.float32, np.float32] = []
            xy0:list[np.float32, np.float32, np.float32] = []
            for n in range(0,24) :
                # rotate XY with map->base_link angle
                x0y0 = xyW[n] # (x,y)
                zz0 = np.float32(0.130) # height of sensor

                
                xx_ = np.float32(( (x0y0[0]*c0 + x0y0[1]*s0)/1000) )
                yy_ = np.float32((-(x0y0[0]*s0 - x0y0[1]*c0)/1000) )

                # debug
                xy_.append((xx_,yy_,zz0))

                xx0 = xx_ + x0 # - 0.08 # 8mm offset
                yy0 = yy_ + y0
                xy0.append((xx0,yy0,zz0))

            # local save point cloud for map-odom drift correction
            if self.mapOdomPcdReq == True :
                self.mapOdomPcdReq = False
                self.mapOdomPcd = xy0.copy()
                self.mapOdomPcdRdy = True

            self.pcd = self.point_cloud(xy0, 'map')

            self.pcd_publisher.publish(self.pcd)

            #DEBUG logging
            # self.get_logger().info(f"\n{xyW = }")
            # self.get_logger().info(f"\n{th0 = } {x0 = } {y0 = }")
            # self.get_logger().info(f"\n{xy_ = }")
            # self.get_logger().info(f"\n{xy0 = }")


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
            #stamp = self.get_clock().now().to_msg(),
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


    def on_map_timer(self) :
        self.createMap()

    def createMap(self) -> None:
        msg = OccupancyGrid()


        # leave header time 0
        msg.header.frame_id = "map"

        # leave info map_load_TIME 0
        arena = self.nav_arena
        if not arena in self.arenas : return

        mapResolution   = self.mapResolution
        mapWidth        = self.arenas[arena]["mapWidth"]
        mapHeight       = self.arenas[arena]["mapHeight"]
        can6Height      = self.arenas[arena]["can6Height"]
        can6Width       = self.arenas[arena]["can6Width"]
        can6GoalOpening = self.arenas[arena]["can6GoalOpening"]
        startWpX0       = self.arenas[arena]["startWpX0"]

        msg.info.resolution = mapResolution
        msg.info.width  = mapWidth
        msg.info.height = mapHeight

        msg.info.origin.orientation.w = 1.0
        msg.info.origin.orientation.x = 0.0
        msg.info.origin.orientation.y = 0.0
        msg.info.origin.orientation.z = 0.0

        msg.info.origin.position.x = -startWpX0
        msg.info.origin.position.y = -(mapHeight*mapResolution) /2
        msg.info.origin.position.z = 0.0

        # create 6 can course map of empty cells
        msg.data = []
        for i in range(0,mapHeight*mapWidth) : msg.data.append(0)

        # add side walls at height edges
        for i in range(0,can6Width) : 
            msg.data[i] = 100
            msg.data[i+(mapWidth*(mapHeight-1))] = 100

        # add wall segments next to goal opening, wall=100
        g = int((can6Height - can6GoalOpening)/2)
        for i in range(0,g) :
            msg.data[i*mapWidth] = 100
            msg.data[i*mapWidth + can6Width-1] = 100
        for i in range(can6Height-g, can6Height) :
            msg.data[i*mapWidth] = 100
            msg.data[i*mapWidth + can6Width-1] = 100

        self.map_msg_publisher.publish(msg)


# simplified code for 2D robot
def quaternion_from_euler(ai, aj, ak):
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

def euler_from_quaternion(x: float, y: float, z: float, w: float) -> tuple:
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians

def main(args=None):
    rclpy.init(args=args)

    node = Robo24DiySlamNode()
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()

# This code is needed to run .py file directly
if __name__ == '__main__':
    main()
