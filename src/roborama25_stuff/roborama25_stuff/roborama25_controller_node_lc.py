import rclpy
import math
import tf_transformations
import signal
import json
import time

from rclpy.node import Node
from functools import partial

from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle.node import LifecycleState, TransitionCallbackReturn

from rcl_interfaces.srv import SetParameters, GetParameters, ListParameters
from rcl_interfaces.msg import Parameter, ParameterDescriptor, ParameterValue, ParameterType, SetParametersResult
from rclpy.exceptions import ParameterNotDeclaredException

from rclpy.callback_groups import ReentrantCallbackGroup

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped

from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Twist

from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from tf2_ros import Duration
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

from sensor_msgs.msg import Joy
from sensor_msgs.msg import PointCloud2, Range
from sensor_msgs_py import point_cloud2

from std_msgs.msg import String, Int32, Float32, Header

class Roborama25ControllerNodeLc(LifecycleNode):
    """
    Creates the 6-can arena maps for home and dprg 
    - rviz2 can display it
    - amcl uses it for localization
    """

    # Game controller button interface
    gotoWaypoints = False
    gotoWaypoints_last = False

    gotoCan = False
    gotoCan_last = False

    gotoQtWaypoints = False
    gotoQtWaypoints_last = False

    goto4CornerWaypoints = False
    goto4CornerWaypoints_last = False

    XYLatched = False


    lifecycle_state_active = False
    amcl_set_param_successful = False

    ft2m:float = 0.3048 # feet to meters

    robotRadius:float = 0.180 # meters
    mapResolution:float = 0.05 # pixel size in meters
    
    # DPRG QT arena info in feet
    d:float = 12.0
    t:float = 1.25 # put in center of target zones
    lengthQuickTrip:dict = {
        "home" : 2, # meters
        "dprg" : (d+(2*t))*ft2m
    }
    
    # DPRG 4C arena info in feet
    # needs to be updated on site for actual size 8-15 ft sq
    d = 9.0 # dist between square corner markers
    t = 1.5 # distance from actual corner of square, center of 3ft clear zone
    size4corner:dict = {
        "home" : 1.0, # meters
        "dprg" : (d+(2*t))*ft2m
    }
    
    # dimensions in units of map resolution
    home_can6Width:int = int((9.0 * ft2m)/mapResolution) # 6-can walls
    home_can6Height:int = int((7.0 * ft2m)/mapResolution)
    home_can6GoalArea:int = int((3.0 * ft2m)/mapResolution) # goal area outside walls
    home_can6GoalOpening:int = int((3.0 * ft2m)/mapResolution) # width of goal opening
    home_mapWidth:int = home_can6Width + home_can6GoalArea
    home_mapHeight:int = home_can6Height
    # dimensions in units of meters
    home_startWpXoffM:float = (8/12.0*ft2m) # offset from back wall, inches to meters
    home_can6WidthM:float = 9.0 * ft2m # 6-can walls from end to end
    home_startWpX0M:float = home_startWpXoffM # X coordinate of the start waypoint xy = (0,0)
    home_goalOpeningX0M:float =  ( # X coordinate of the goal opening goto before going to dropoff
        home_can6WidthM - home_startWpXoffM - (1.5*ft2m)) 
    home_canDropoffX0M:float = ( # X coordinate of the dropoff can location
        home_can6WidthM - home_startWpXoffM + (1.0*ft2m))
    home_scanWpX0M:float = ( # X coordinate of the scan waypoint in the center of the arena
        home_can6WidthM/2 - home_startWpXoffM)
    home_canBackupM:float = 2*ft2m # distance to back up after dropping off the can
    home_arena: dict = {
        "can6Width"       : home_can6Width,
        "can6Height"      : home_can6Height,
        "can6GoalArea"    : home_can6GoalArea,
        "can6GoalOpening" : home_can6GoalOpening,
        "mapWidth"        : home_mapWidth,
        "mapHeight"       : home_mapHeight,
        "startWpX0"       : home_startWpX0M,
        "goalOpeningX0"   : home_goalOpeningX0M,
        "canDropoffX0"    : home_canDropoffX0M,
        "scanWpX0"        : home_scanWpX0M,
        "canBackup"       : home_canBackupM
    }

    # dimensions in units of map reolution
    dprg_can6Width:int = int((10.0 * ft2m)/mapResolution) # 6-can walls
    dprg_can6Height:int = int((7.0 * ft2m)/mapResolution)
    dprg_can6GoalArea:int = int((3.0 * ft2m)/mapResolution) # goal area outside walls
    dprg_can6GoalOpening:int = int((3.0 * ft2m)/mapResolution) #width of goal opening
    dprg_mapWidth:int = dprg_can6Width + dprg_can6GoalArea
    dprg_mapHeight:int = dprg_can6Height
    # dimensions in units of meters
    dprg_startWpX0:float = (8/12.0*ft2m) # offset from back wall, inches to meters
    dprg_goalOpeningX0:float = 2.5 # X coordinate of the goal opening goto before going to dropoff
    dprg_canDropoffX0:float = 3.25 # X coordinate of the dropoff can location
    dprg_scanWpX0:float = 2.0 # X coordinate of the scan waypoint
    # dimensions in units of meters
    dprg_startWpXoffM:float = (8/12.0*ft2m) # offset from back wall, inches to meters
    dprg_can6WidthM:float = 10.0 * ft2m # 6-can walls from end to end
    dprg_startWpX0M:float = dprg_startWpXoffM # X coordinate of the start waypoint xy = (0,0)
    dprg_goalOpeningX0M:float =  ( # X coordinate of the goal opening goto before going to dropoff
        dprg_can6WidthM - dprg_startWpXoffM - (1.5*ft2m)) 
    dprg_canDropoffX0M:float = ( # X coordinate of the dropoff can location
        dprg_can6WidthM - dprg_startWpXoffM + (1.0*ft2m))
    dprg_scanWpX0M:float = ( # X coordinate of the scan waypoint in the center of the arena
        dprg_can6WidthM/2 - dprg_startWpXoffM)
    dprg_canBackupM:float = 2*ft2m # distance to back up after dropping off the can

    dprg_arena: dict = {
        "can6Width"       : dprg_can6Width,
        "can6Height"      : dprg_can6Height,
        "can6GoalArea"    : dprg_can6GoalArea,
        "can6GoalOpening" : dprg_can6GoalOpening,
        "mapWidth"        : dprg_mapWidth,
        "mapHeight"       : dprg_mapHeight,
        "startWpX0"       : dprg_startWpX0M,
        "goalOpeningX0"   : dprg_goalOpeningX0M,
        "canDropoffX0"    : dprg_canDropoffX0M,
        "scanWpX0"        : dprg_scanWpX0M,
        "canBackup"       : dprg_canBackupM
    }


    new_locxy_idx:int = 0
    new_locxy_dict:dict = { 
        "home": [(1.5,0.0), (0.5,0.0)],
        "dprg": [(1.5,0.0), (0.5,0.0)]
    }

    arenas:dict = {
        "home" : home_arena,
        "dprg" : dprg_arena
    }

    # flag to start running the 6can state machine
    enable_6can_states:bool = False

    # can counter to know when to stop
    can_counter:int = 0

    nav_arena:str = "home"

    # diyslamEnabled = True
    
    nav2_run_first_exec:bool = True
    
    callback_set_param_done:bool = False


    feetToMeter:float = 0.3048

    def __init__(self, nav: BasicNavigator):
        super().__init__('roborama25_controller_node_lc')

        self.nav = nav

        self.cb_group_re = ReentrantCallbackGroup()
        self.cb_group_mx = MutuallyExclusiveCallbackGroup()
        self.cb_group_nav2_run = MutuallyExclusiveCallbackGroup()

        # Life cycle needed
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # map->odom static tf is broadcast when /amcl/tf_broadcast=False
        self.tf_static_broadcasterOdom = StaticTransformBroadcaster(self)

        self.amcl_set_param_svc = self.create_client(SetParameters, '/amcl/set_parameters')
        while not self.amcl_set_param_svc.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('/amcl/set_parameters service not available, waiting again...')

        self.local_costmap_set_param_svc = self.create_client(SetParameters, '/local_costmap/local_costmap/set_parameters')
        while not self.local_costmap_set_param_svc.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('/local_costmap/local_costmap/set_parameters service not available, waiting again...')

        self.global_costmap_set_param_svc = self.create_client(SetParameters, '/global_costmap/global_costmap/set_parameters')
        while not self.global_costmap_set_param_svc.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('/global_costmap/global_costmap/set_parameters service not available, waiting again...')

        self.set_param_request = SetParameters.Request()

        self.robot_json_publisher = self.create_publisher(String, '/robot_json',10)
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.run_state_publisher = self.create_publisher(Int32, '/run_state', 10)
        
        self.joy_subscription = self.create_subscription(Joy, '/joy', self.joy_callback, 
                                                                10, callback_group=self.cb_group_mx)
        self.tofL4_rng_subscription = self.create_subscription(Range, '/tofL4_rng', self.tofL4_rng_callback, 
                                                                10, callback_group=self.cb_group_nav2_run)
        self.tofL5L_pcd_subscription = self.create_subscription(PointCloud2, '/tofL5L_pcd', self.tofL5L_pcd_callback, 
                                                                10, callback_group=self.cb_group_re)
        self.tofL5R_pcd_subscription = self.create_subscription(PointCloud2, '/tofL5R_pcd', self.tofL5R_pcd_callback, 
                                                                10, callback_group=self.cb_group_re)

        self.get_logger().info(f"roborama25_controller_node Started {self.nav_arena=}")
    

    ############# Start Lifecycle stuff #############

    # Create ROS2 communications, connect to HW
    def on_configure(self, previous_state: LifecycleState):
        self.get_logger().info(f"IN on_configure")

        # self.nav = BasicNavigator()
        
        self.get_logger().info(f"on_configure: waitUntilNav2Active before starting configuration")
        self.nav.waitUntilNav2Active()
        self.get_logger().info(f"on_configure: waitUntilNav2Active done")   

        # publish the map 1/sec
        # self.map_timer = self.create_timer(1.0, self.on_map_timer)
        self.map_timer = self.create_timer(0.1, self.nav2_run, callback_group=self.cb_group_mx)
        self.map_timer.cancel()
        
        # the nav2 map saver qos needs to be transient local
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.map_msg_publisher = self.create_lifecycle_publisher(OccupancyGrid, 'map', qos_profile=qos_profile)

        return TransitionCallbackReturn.SUCCESS

    # Clean up stuff for cleanup, shutdown, error
    def cleanup_lc(self) :        
        self.destroy_lifecycle_publisher(self.map_msg_publisher)

                 
    def cleanup(self) :                
        self.destroy_timer(self.map_timer)
        # self.nav.destroy()
        
    # Destroy ROS2 communications, disconnect from HW
    def on_cleanup(self, previous_state: LifecycleState):
        self.get_logger().info("IN on_cleanup")
        self.cleanup_lc()
        self.cleanup()
        return TransitionCallbackReturn.SUCCESS

    # Activate/Enable HW
    def on_activate(self, previous_state: LifecycleState):
        self.lifecycle_state_active = True
        self.get_logger().info("IN on_activate")
        self.map_timer.reset()
        
        return super().on_activate(previous_state)

        
    # Deactivate stuff used in shutdown, error
    def deactivate(self):
        self.lifecycle_state_active = False
        self.map_timer.cancel()
        
        
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

    ############ End Lifecycle stuff ###########


    ############ Nav2 run stuff #############
    
    def nav2_run(self) :
        """
        Called in a timer 
        """
        # initialize
        if self.nav2_run_first_exec == True :
            self.get_logger().info(f"nav2_run: {self.nav2_run_first_exec=}")

            initial_pose = self.createPose(0,0,0)
            self.setInitialPose(initial_pose)    

            # rviz2 grid area 10x10 with origin 0,0 at the center

            self.publishEmptyMap(0.05, 10, 10, -5, -5)
            self.send_set_param_request(self.amcl_set_param_svc, 'tf_broadcast', False)
            self.nav2_run_first_exec = False
            
        # Start a course when gamepad button is pushed
        if self.gotoCan==True and self.gotoCan_last==False :
            self.can_counter = 0
            self.run6Can() # Button X
        elif self.gotoQtWaypoints==True and self.gotoQtWaypoints_last==False :
            self.runQTrip() # Button B
        elif self.goto4CornerWaypoints==True and self.goto4CornerWaypoints_last==False :
            self.run4Corner() # Button A
        elif self.gotoWaypoints==True and self.gotoWaypoints_last==False :
            self.runWPoints() # Button Y
        
        self.gotoWaypoints_last = self.gotoWaypoints
        self.goto4CornerWaypoints_last = self.goto4CornerWaypoints
        self.gotoQtWaypoints_last = self.gotoQtWaypoints
        self.gotoCan_last = self.gotoCan

    def runWPoints(self) :
        """
        button Y
        """
        self.get_logger().info(f"runWPoints: {self.nav_arena=} started (button Y)")
        
        self.createWPMap()
        self.send_set_param_request(self.amcl_set_param_svc, 'tf_broadcast', False)

        #DEBUG: testing can detect and goto
        tfOK = False
        can_pose = None
        while not tfOK :
            (tfOK, can_pose) = self.getCanPose()
            
        if tfOK :
            dist = self.gotoCanTF(45)
        else :
            dist = -1
            
        if dist > 0 :
            self.get_logger().info(f"runWPoints: Robot is close to the can at {dist=}")
        else :
            self.get_logger().info(f"runWPoints: Robot failed to get close to the can")
            
            
        # # Drive waypoint 
        # for wp in [(2.5,0), (1,0.5), (1,-0.5), (0,0)] :
        #     status = self.gotoXY(wp[0],wp[1], 30)

        # status = self.rotateToAngle(0,10)
        # self.get_logger().info(f"runWPoints: final rotation {status=}")    
        
    def driveDirectWiggle(self, dist: float, vx: float=0.5) :
        self.driveDirect(dist, vx)
        self.driveDirect( dist/1000, vx)
        self.driveDirect(-dist/1000, vx)
        
    def driveDirect(self, dist: float, vx: float=0.5) :
        """
        Drive with timing all in the wheels micro 
        Drive for a distance, return when done
        Drives in reverse when d<0
        vx must be > 0 and != 0
        """
        
        if dist == 0 or vx <= 0 :
            self.get_logger().info(f"driveDirect: invalid params {dist=:.3f} {vx=:.3f}, aborted driving")
            return
        
        if dist < 0 :
            # drive in reverse with neg velocity
            dist *= -1
            vx   *= -1
        
        sec = math.fabs(dist/vx)
        left = vx
        right = vx
    
        self.get_logger().info(f"driveDirect: send json msg {dist=:.3f} {vx=:.3f} {sec=:.3f} {left=:.3f} {right=:.3f}")
    
        # wheels drive at veloocity m/s for sec
        cmd_json = {"wheels": {"left": left, "right": right,  "sec": sec}}
        cmd_str = json.dumps(cmd_json)+"\0"
        self.robot_json_data_publish(cmd_str)

        # blocking wait for the expected drive movement time
        sec*=1.1
        self.get_logger().info(f"driveDirect: waiting {sec=:.3f}")
        time.sleep(sec)

    def rotateDirectWiggle(self, a: float, va: float=0.5) :
        self.rotateDirect(a, va)
        self.rotateDirect(a/25, va)
        self.rotateDirect(-a/25, va)
        
    def rotateDirect(self, a: float, va: float=0.5) :
        """
        Rotate simple using wheel odom only
        a: angle in rads
        va: angular velocity in rads/sec
        """
                
        # limit rotation angle to -pi to pi 
        while a>math.pi : a -= 2*math.pi
        while a<-math.pi : a += 2*math.pi

        if a == 0 or va <= 0 :
            self.get_logger().info(f"rotateDirect: invalid params {a=:.3f} {va=:.3f}, aborted driving")
            return
        
        va = math.fabs(va)
        if a < 0 : va = -va
        
        # time to rotate at va velocity to angle a
        sec = math.fabs(a/va)
    
        self.get_logger().info(f"rotateDirect: send json msg {a=:.3f} {va=:.3f} {sec=:.3f}")
    
        # wheels drive at veloocity m/s for sec
        cmd_json = {"wheels": {"angular": va,  "sec": sec}}
        cmd_str = json.dumps(cmd_json)+"\0"
        self.robot_json_data_publish(cmd_str)

        # blocking wait for the expected drive movement time
        sec*=1.1
        self.get_logger().info(f"rotateDirect: waiting {sec=:.3f}")
        time.sleep(sec)
          
    def driveOdom(self, dist: float, vx: float=0.5) :
        """
        Drive for a distance, return when done
        Drives in reverse when d<0
        vx must be > 0 and != 0
        """
        
        if dist == 0 or vx <= 0 :
            self.get_logger().info(f"driveOdom: invalid params {dist=:.3f} {vx=:.3f}, aborted driving")
            return
        
        if dist < 0 :
            # drive in reverse with neg velocity
            dist *= -1
            vx   *= -1
        
        dt = 0.05 # time step        
        
        msg = Twist()
        msg.linear.x = vx
        self.cmd_vel_publisher.publish(msg)
        
        # time to go distance at vx velocity
        t = math.fabs(dist/vx)
        elapsed_time = 0.0
        start_time = self.get_clock().now()
        while rclpy.ok() and elapsed_time <(t-dt) and self.lifecycle_state_active==True:
            self.cmd_vel_publisher.publish(msg)
            time.sleep(dt)
            current_time = self.get_clock().now()
            elapsed_time = (current_time - start_time).nanoseconds / 1e9
            # self.get_logger().info(f"drive: {elapsed_time=:.3f} {dist=:.3f} {vx=:.3f} {t=:.3f}")

        # wait for the remaining time (adjusting for publish time estimate)
        current_time = self.get_clock().now()
        elapsed_time = (current_time - start_time).nanoseconds / 1e9
        ts = t-elapsed_time - 0.005
        if (ts>0) : time.sleep(ts)
        
        # stop the robot motion
        msg.linear.x = 0.0
        self.cmd_vel_publisher.publish(msg)
        
        current_time = self.get_clock().now()
        elapsed_time = (current_time - start_time).nanoseconds / 1e9
        self.get_logger().info(f"driveOdom: done {elapsed_time=:.3f} {t=:.3f} {ts=:.3f} {dist=:.3f} {vx=:.3f} {dt=:.3f}")

    def rotateOdom(self, a: float, va: float = 0.05) :
        """
        Rotate simple using wheel odom only
        a: angle in rads
        va: angular velocity in rads/sec
        """
        if a == 0 or va <= 0 :
            self.get_logger().info(f"rotateOdom: invalid params {a=:.3f} {va=:.3f}, aborted driving")
            return
        
        dt = 0.05 # time step
                
        # limit rotation angle to -pi to pi 
        while a>math.pi : a -= 2*math.pi
        while a<-math.pi : a += 2*math.pi
        
        va = math.fabs(va)
        if a > 0 : va = -va
        
        msg = Twist()
        msg.angular.z = va
        self.cmd_vel_publisher.publish(msg)
        
        # time to rotate at va velocity to angle a
        t = math.fabs(a/va)
        elapsed_time = 0.0
        start_time = self.get_clock().now()
        while rclpy.ok() and elapsed_time <(t-dt) and self.lifecycle_state_active==True:
            self.cmd_vel_publisher.publish(msg)
            time.sleep(dt)
            current_time = self.get_clock().now()
            elapsed_time = (current_time - start_time).nanoseconds / 1e9
            # self.get_logger().info(f"rotateOdom: {elapsed_time=:.3f} {t=:.3f} {a=:.3f} {va=:.3f} {dt=:.3f}")  

        # wait for the remaining time (adjusting for publish time estimate)
        current_time = self.get_clock().now()
        elapsed_time = (current_time - start_time).nanoseconds / 1e9
        ts = t-elapsed_time - 0.005
        if (ts>0) : time.sleep(ts)

        # stop the robot rotation
        msg.angular.z = 0.0
        self.cmd_vel_publisher.publish(msg)
        
        current_time = self.get_clock().now()
        elapsed_time = (current_time - start_time).nanoseconds / 1e9
        self.get_logger().info(f"rotateOdom: finished rotation {elapsed_time=:.3f} {t=:.3f} {ts=:.3f} {a=:.3f} {va=:.3f} {dt=:.3f}")
        
    def run4Corner(self) :
        """
        button A
        """
        self.get_logger().info(f"run4Corner: {self.nav_arena=} started (button A)")
        
        self.create4CMap()
        self.send_set_param_request(self.amcl_set_param_svc, 'tf_broadcast', False)

        # Drive to 4 corners of a square area
        d = self.size4corner[self.nav_arena]
        
        # need to account for robot radius since nav2 stops that amount
        # because of the obstacle mapping             
        # r = self.robotRadius
        # for wp in [(d+r,0), (d,-(d+r)), (0,-(d+r)), (0,r)] :
        #     status = self.gotoXY(wp[0],wp[1], 30)

        # status = self.rotateToAngle(0,10)
        # self.get_logger().info(f"run4Corner: final rotation {status=}")    

        # Drive to 4 corners of a square area
        # using simple wheel odom only
        vx = 0.125
        va = 0.25
        self.driveDirectWiggle(d, vx)  
        self.rotateDirectWiggle(math.pi/2, va)
        self.driveDirectWiggle(d, vx)  
        self.rotateDirectWiggle(math.pi/2, va)
        self.driveDirectWiggle(d, vx)  
        self.rotateDirectWiggle(math.pi/2, va)
        self.driveDirectWiggle(d, vx)  
        self.rotateDirectWiggle(math.pi/2, va)

        
    def runQTrip(self) :
        """
        button B
        """
        self.get_logger().info(f"runQTrip: {self.nav_arena=} started (button B)")
        
        self.createQTMap()
        self.send_set_param_request(self.amcl_set_param_svc, 'tf_broadcast', False)
#        self.send_set_param_request(self.local_costmap_set_param_svc, 'obstacle_layer.enabled', False)
#        self.send_set_param_request(self.global_costmap_set_param_svc, 'obstacle_layer.enabled', False)
                
        # status = self.gotoXY(8*self.feetToMeter,0, 30)
        d = self.lengthQuickTrip[self.nav_arena]
        vx = 0.75 #0.25
        self.driveDirectWiggle( d, vx)
        time.sleep(0)
        self.driveDirectWiggle(-d, vx)
        
    def run6Can(self) :    
        """
        button X
        """
        self.get_logger().info(f"run6Can: {self.nav_arena=} started (button X) ")
        
        self.create6CMap()
        self.send_set_param_request(self.amcl_set_param_svc, 'tf_broadcast', True)
    
        # Enables the 6 can statemachine running in tofL4 callback
        self.enable_6can_states = True

        # 6 can runs when enable state is True using the tofL4 callback
        while self.enable_6can_states==True and self.lifecycle_state_active==True :
            time.sleep(0.1)
                 
        self.get_logger().info(f"run6Can: 6 can state machine finished {self.can_counter=}")

    def createPose(self,x,y,a) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.nav.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = 0.0
        (pose.pose.orientation.x,
        pose.pose.orientation.y,
        pose.pose.orientation.z,
        pose.pose.orientation.w) = tf_transformations.quaternion_from_euler(0.0,0.0,float(a))
        # self.get_logger().info(pose)
        return pose

    def waitTaskComplete(self,t) :
        if self.lifecycle_state_active==False : 
            self.nav.cancelTask()
        else :        
            while not self.nav.isTaskComplete():
                feedback = self.nav.getFeedback()
                # self.get_logger().info(f"{feedback=}")
                # spin does not provide time in feedback
                try :
                    nt = feedback.navigation_time.sec
                    if nt > t :
                        self.get_logger().info(f"waitTaskComplete: Canceling task {nt=} > {t=}")
                        self.nav.cancelTask()
                except :
                    pass
                
        feedback = self.nav.getFeedback()
        result = self.nav.getResult()
        # self.get_logger().info(f"{feedback=} {result=}")
        
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('waitTaskComplete: Goal succeeded!')
        elif result == TaskResult.CANCELED:
            self.get_logger().info('waitTaskComplete: Goal was canceled!')
        elif result == TaskResult.FAILED:
            self.get_logger().info('waitTaskComplete: Goal failed!')
        else :
            self.get_logger().info(f"waitTaskComplete: nav.getResult() {result=}")

        return (result, feedback)

    def setInitialPose(self, pose) -> None:
        if self.lifecycle_state_active==False : return
        
        self.nav.setInitialPose(pose)
       
    def gotoPose(self, pose, t):
        """
        Go to the pose within in the time limit
        """

        self.nav.goToPose(pose)
        # (result, feedback) = self.waitTaskComplete(t)
        (result, _) = self.waitTaskComplete(t)
       
        return result
    
    def gotoCanTF(self, t:float = 10) -> int:
        """
        Go to the can TF location, but stop 0.2m short
        Rotates to point to the can before moving to it
        gets new can position every 1 second as it approaches it
        Returns distance to the can, -1 if it did not succeed
        """
        if self.lifecycle_state_active==False : return -1
        
        d=100
        cnt=0
        timeStart = time.monotonic()
        timer = 0.0
        while d>0.25 and timer<t and self.lifecycle_state_active==True :
            (tf_OK, can_pose) = self.getCanPose()
            cnt+=1
            if tf_OK==False and cnt>5:
                self.get_logger().warn(f"gotoCanTF: Failed to get can pose")
                return -1
            if tf_OK==True :
                x = can_pose.pose.position.x
                y = can_pose.pose.position.y
                self.get_logger().info(f"gotoCanTF:b can TF {x=:.3f} {y=:.3f}")
                # get current pose to determine the angle offset
                # rotating to point to the desired is faster
                # maybe the navigation behavior can be "fixed" 
                (tf_OK, current_pose) = self.getCurrentPose()
                if tf_OK :
                    xd = float(x) - current_pose.pose.position.x
                    yd = float(y) - current_pose.pose.position.y
                    # Calc angle to target XY coordinate
                    a = math.atan2(yd,xd)
                    d = math.sqrt(xd*xd + yd*yd)
                    
                    # adjust the distance to the can by robot radius to stop at the cost map boundary
                    d = d - self.robotRadius
                    # d-=0.2
                    x = current_pose.pose.position.x + d*math.cos(a)
                    y = current_pose.pose.position.y + d*math.sin(a)
                    
                    goto_pose = self.createPose(x,y,a)
                    self.get_logger().info(f"gotoXY: goto {x=:.3f} {y=:.3f} {d=:.3f} {a=:.3f} {timer=:.3f} {t=:.3f}")

                    # rotate to point to goto xy position before moving to it
                    status = self.rotateToAngle(a,10)
                    if status != TaskResult.FAILED :
                        status = self.gotoPose(goto_pose, 5.0) #1.0)
                        if status != TaskResult.SUCCEEDED : d = -1
                    else :
                        self.get_logger().info(f"gotoXY: Failed to rotateToAngle {a=}")
                        d=-1
                else :
                    self.get_logger().info(f"gotoXY: Failed to get current pose")
                    d=-1
            timer = time.monotonic() - timeStart

        return d
    
    def gotoXY(self,x,y,t,obstacle_layer_enabled: bool=True) -> int:
        """
        Go to the X,Y coordinates from the current position within a lime limit
        Rotate to pint to the X,Y position then goto the position
        The angle of the Pose to go to is set as the angle from the 
        current X,Y to the goto X,Y positions
        """
        if self.lifecycle_state_active==False : return
        # get current pose to determine the angle offset
        # rotating to point to the desired is faster
        # maybe the navigation behavior can be "fixed" 
        (tf_OK, current_pose) = self.getCurrentPose()
        if not tf_OK : 
            self.get_logger().info(f"gotoXY: Failed to get current pose")
            return False
        cx = current_pose.pose.position.x
        cy = current_pose.pose.position.y
        xd = float(x) - cx
        yd = float(y) - cy
        # Calc angle to target XY coordinate
        a = math.atan2(yd,xd)

        goto_pose = self.createPose(x,y,a)
        self.get_logger().info(f"gotoXY: from {cx=:.3f} {cy=:.3f}, goto {x=:.3f} {y=:.3f} {a=:.3f}")

        # rotate to point to goto xy position before moving to it
        status = self.rotateToAngle(a,10)
        
        self.send_set_param_request(self.local_costmap_set_param_svc, 
                        'obstacle_layer.enabled', obstacle_layer_enabled)
        self.send_set_param_request(self.global_costmap_set_param_svc, 
                        'obstacle_layer.enabled', obstacle_layer_enabled)

        result = self.gotoPose(goto_pose,t)
        
        return result
    
    def rotateRad(self,a,t):
        """
        Rotate a radians within time t
        Adjust rotation angle to a minimum angle -pi to pi
        """
        if self.lifecycle_state_active==False : return
        # limit rotation angle to -pi to pi
        while  a>math.pi : a -= 2*math.pi
        while a<-math.pi : a += 2*math.pi
        
        self.send_set_param_request(self.amcl_set_param_svc, 'tf_broadcast', False)
        self.send_set_param_request(self.local_costmap_set_param_svc, 'obstacle_layer.enabled', False)
        
        
        self.nav.spin(float(a),t)
        (result, feedback) = self.waitTaskComplete(0)
        
        self.send_set_param_request(self.local_costmap_set_param_svc, 'obstacle_layer.enabled', True)
        self.send_set_param_request(self.amcl_set_param_svc, 'tf_broadcast', True)

        return result
    
    def rotateToAngle(self,a,t) :
        """
        Rotate to the given absolute angle awithin time t    
        """
        if self.lifecycle_state_active==False : return
        
        # continue to rotate toward desired angle until time out
        # Get the current time
        start_time = self.get_clock().now()
        elapsed_time = 0.0
        result = False
        spinThresh = 0.05 #0.02 #0.01 # about 3 degrees
        
        # Loop until the timeout is reached or the task is complete
        while rclpy.ok() and elapsed_time <t and self.lifecycle_state_active==True:
            # get current pose to determine the angle offset
            # rotating to point to the desired is faster
            # maybe the navigation behavior can be "fixed" 
            (tf_OK, current_pose) = self.getCurrentPose()
            if tf_OK==False:
                self.get_logger().warn(f"rotateToAngle: Failed to get current pose")
                return False
            # convert current pose euler from quaternion, discard xx and yy
            q = current_pose.pose.orientation
            (xx,yy,aa) = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
            spin = float(a) - aa
            
            if abs(spin) < spinThresh:
                self.get_logger().info(f"rotateToAngle: spin threshold reached {spin:.3f} < {spinThresh:.3f}, breaking loop")
                break
            
            result = self.rotateRad(spin,t)
            
            current_time = self.get_clock().now()
            elapsed_time = (current_time - start_time).nanoseconds / 1e9  # Convert nanoseconds to seconds

            self.get_logger().info(f"rotateToAngle: rotate {result=} {a=:.3f} {aa=:.3f} {spin=:.3f} {elapsed_time=}")
            
        return result
        
    def getPoseFromTF(self, target_frame:str) -> tuple:
        """
        get map->'target_frame' transform
        returns (tf_OK, pose) pose: PoseStamped
        """
        # try getting pose a few times
        cnt = 0
        tf_OK = False
        while tf_OK == False and cnt < 5 :
            try:
                tf = self.tf_buffer.lookup_transform (
                    'map',
                    target_frame,
                    #self.nav.get_clock().now().to_msg(),
                    rclpy.time.Time(), # default 0
                    timeout=rclpy.duration.Duration(seconds=0.1) #0.0)
                    )
                tf_OK = True

            except (LookupException, ConnectivityException, ExtrapolationException) as ex:
                self.get_logger().info(f'getPoseFromTF: Could not find transform map->{target_frame}: {ex}')
                tf_OK = False
                cnt += 1
                
        if tf_OK == False or cnt >= 5 :
            self.get_logger().info(f'getPoseFromTF: Failed to find transform map->{target_frame} after {cnt} tries {tf_OK=}')
            return (False,None) 
        
        # translate wall points to align with map coordinates
        if tf_OK :
            # get x, y, theta from TF
            x:float = tf.transform.translation.x
            y:float = tf.transform.translation.y
            q:float = tf.transform.rotation
            # convert quaterion to euler, discard xx and yy
            (xx,yy,a) = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
            pose = self.createPose(x,y,a)
        else :
            pose = None
            
        return (tf_OK,pose)
        
    def getCanPose(self) -> tuple:
        """
        get map->can transform
        returns (tf_OK, pose) pose: PoseStamped
        """
        return self.getPoseFromTF('can')
    
    def getOdomPose(self) -> tuple:
        """
        get map->odom transform
        returns (tf_OK, pose) pose: PoseStamped
        """
        return self.getPoseFromTF('odom')
    
    def getCurrentPose(self):
        """
        get map->base_footprint transform
        returns (tf_OK, pose) pose: PoseStamped
        """
        return self.getPoseFromTF('base_footprint')
    
    # findCanTime = 0.0

    # tf_OK: bool = False
    def getCanTFOK(self) ->bool:
        try:
            # discard TF ony used to detrmine if a TF is detected
            tf_OK = self.tf_buffer.can_transform (
                'map',
                'can',
                self.nav.get_clock().now().to_msg(),
                #
                # rclpy.time.Time(), # default 0 get latest
                timeout=rclpy.duration.Duration(seconds=0.1)
                )

        except (LookupException, ConnectivityException, ExtrapolationException) as ex:
            self.get_logger().info(f'getCanTFOK: Exception transforming transform map->can: {ex}')
            tf_OK = False

        if tf_OK : self.get_logger().info(f'getCanTFOK: Can detected map->can')
        else :     self.get_logger().info(f'getCanTFOK: Could not find transform map->can')

        return tf_OK
    
    def rotateCanDet(self, a: float, va: float = 0.5) ->bool:
        """
        Rotate using wheel odom only
        stops if a can is detected
        a: angle in rads (not limited)
        va: angular velocity in rads/sec
        Returns bool tk_OK (can detected)
        """
        tf_OK: bool = False
        if a == 0 or va <= 0 :
            self.get_logger().info(f"rotateCanDet: invalid params {a=:.3f} {va=:.3f}, aborted")
            return False
        
        if self.closeCanDet() : 
            return True
        
        va = math.fabs(va)
        if a < 0 : va = -va
        
        # time to rotate at va velocity to angle a
        sec = math.fabs(a/va)
    
        # wheels drive at velocity m/s for sec
        msg = Twist()
        msg.angular.z = va
        self.cmd_vel_publisher.publish(msg)

        self.get_logger().info(f"rotateCanDet: start rotating {a=:.3f} {va=:.3f} {sec=:.3f}")

        # wait for the expected drive movement time while looking for can
        waitSec=sec
        # self.findCanTime = time.monotonic()
        findCanTime = time.monotonic()
        timer = 0
        while (not tf_OK) and (timer < waitSec) :
            timer = time.monotonic() - findCanTime 
            self.cmd_vel_publisher.publish(msg)
            tf_OK = self.closeCanDet()
            if not tf_OK : tf_OK = self.getCanTFOK()
            self.get_logger().info(f"rotateCanDet: waiting {timer=:.3f} {tf_OK=} {a=:.3f} {va=:.3f} {waitSec=:.3f}")

        # stop rotation after can is detected or time out
        msg.angular.z = 0.0
        self.cmd_vel_publisher.publish(msg)

        self.get_logger().info(f"rotateCanDet: Can detect {tf_OK=}")

        return tf_OK
    
    def closeCanDet(self) -> bool :
        """
        Check if can is close enough to be detected with tof sensors
        Returns True if can signiture is detected 
        """
        canDet:bool = False

        dist = self.tofL4_rng
        Lnum = 0
        Rnum = 0
        distCanDet = 0.125
        distMinDet = 0.25
        l4NumMax = 9

        # get the minimum distance from the sensors
        distMinL = min(self.tofL5L_pcd)
        distMinR = min(self.tofL5R_pcd)
        distMin  = min(distMinL, distMinR, dist)
        for d in self.tofL5L_pcd :
            if d<(distMin+0.07) : Lnum += 1
        for d in self.tofL5R_pcd :
            if d<(distMin+0.07) : Rnum += 1

        distLim = distMin + distCanDet
        distMinMax = max(distMinL, distMinR, dist)
        if distMin<distMinDet and distMinMax<distLim and Lnum<=l4NumMax and Rnum<=l4NumMax:
            canDet = True

        self.get_logger().info(f"closeCanDet: {dist=:.3f} {distMin=:.3f} {distMinMax=:.3f} {distMinL=:.3f} {distMinR=:.3f} {distLim=:.3f} {Lnum=} {Rnum=} {canDet=}")

        return canDet

################################### 6CAN stuff ##########################    

    
    tofL4_rng = None
    tofL5L_pcd = None
    tofL5R_pcd = None
    changed_6can_state = False

    findCanCnt:int = 0       
        
    current_6can_state = "start"
    next_6can_state =  "findCan"
    
    def run_6can_states(self) :
        """
        Executes every tofL4_rng_callback
        Runs the upper level states for six can
        >findCan
        >gotoCanLocation
        >approachCan
        >grabCan
        >gotoGoalOpening
        >gotoCanDrop
        >dropCan
        >backupFromCan
        >findCanAtGoal
        >gotoNewLocation
        >returnHome
        >done
        """
        
        if self.lifecycle_state_active==False : return
        
        if self.enable_6can_states == False : return

        
        if self.current_6can_state != self.next_6can_state :
            self.get_logger().info(f"run_6can_states: State changed {self.current_6can_state=} {self.next_6can_state=}")
            self.current_6can_state = self.next_6can_state
            self.changed_6can_state = True
        else :
            self.changed_6can_state = False

        # self.get_logger().info(f"run_6can_states: {self.current_6can_state=}")
        
        msg = Int32()
        match self.current_6can_state :
            case "findCan" : 
                msg.data=1
                next_state = self.run_findCan()
            case "gotoCanLocation" :
                msg.data=2
                next_state = self.run_gotoCanLocation()
            case "approachCan" :
                msg.data=3
                next_state = self.run_approachCan()
            case "grabCan" :
                msg.data=4
                next_state = self.run_grabCan()
            case "gotoGoalOpening" :
                msg.data=5
                next_state = self.run_gotoGoalOpening()
            case "gotoCanDrop" :
                msg.data=6
                next_state = self.run_gotoCanDrop()
            case "dropCan" :
                msg.data=7
                next_state = self.run_dropCan()  
            case "backupFromCan" :
                msg.data=8
                next_state = self.run_backupFromCan()                  
            case "findCanAtGoal" :
                msg.data=9
                next_state = self.run_findCanAtGoal()                  
            case "gotoNewLocation" :
                msg.data=10
                next_state = self.run_gotoNewLocation()
            case "returnHome" :
                msg.data=11
                next_state = self.run_returnHome()
            case "done" :
                msg.data=12
                next_state = self.run_done()
            case _ :
                msg.data=-10
                self.get_logger().info(f"run_6can_states: Unknown state {self.current_6can_state=}")
                self.enable_6can_states = False

        self.next_6can_state = next_state
        self.run_state_publisher.publish(msg)

    
    def run_findCan(self) ->str:
        """
        Find a can using the camera can detection which generates a "can" TF
        or can detected using TOF sensors
        Returns instantly if a can is detected as close by
        Returns next state string
        """
        next_state = "findCan"

        if self.changed_6can_state : self.findCanCnt = 0

        # return with next state instantly if a can is detected as close by
        if self.closeCanDet() : return "gotoCanLocation"

        # Check if the can transform is detected, discard pose
        tf_OK = self.getCanTFOK()
        if not tf_OK :
            tf_OK = self.rotateCanDet(2*math.pi) 
        
        # Make sure can is still detected
        tf_OK = self.getCanTFOK()

        self.findCanCnt +=1
        if tf_OK : 
            next_state = "gotoCanLocation"
        elif self.findCanCnt >= 2 :
            self.get_logger().info(f"run_findCan: Failed to find cangoto new location {self.findCanCnt=}")
            next_state = "gotoNewLocation"

        return next_state
    
    def run_gotoCanLocation(self) ->str:
        """
        Try to go to the location of the can 
        Returns next state string
        """
        next_state = "gotoCanLocation"
        
        dist = self.gotoCanTF(20)
        
        if dist > 0 :
            self.get_logger().info(f"run_gotoCanLocation: Robot is close to the can, tofL4 range {dist=}")
            next_state = "approachCan"
        else :
            self.get_logger().info(f"run_gotoCanLocation: Robot failed to get close to the can")
            next_state = "findCan"
            
        return next_state
    
    def run_approachCan(self) ->str:
        """
        Approach the can using the TOF sensors
        Returns next state string
        """
        next_state = "approachCan"

        if self.changed_6can_state :
            self.approachCanTimeStart = time.monotonic()

        msg = Twist()

        if (time.monotonic() -  self.approachCanTimeStart) > 20.0 :
            self.get_logger().info(f"run_approachCan: Time out, going to findCan")
            next_state = "findCan"
            self.cmd_vel_publisher.publish(msg) # stop
            return next_state
        
        
        dist = self.tofL4_rng
        # distMin = 1000.0
        # distMinL = distMin
        # distMinR = distMin
        Lnum = 0
        Rnum = 0
        distCanDet = 0.06
        l4NumMax = 9
        distWallDet = 0.2

        # get the minimum distance from the sensors
        distMinL = min(self.tofL5L_pcd)
        distMinR = min(self.tofL5R_pcd)
        distMin  = min(distMinL, distMinR)
        for d in self.tofL5L_pcd :
            if d<(distMin+0.07) : Lnum += 1
        for d in self.tofL5R_pcd :
            if d<(distMin+0.07) : Rnum += 1

        if (Lnum>l4NumMax or Rnum>l4NumMax) and (distMin<distWallDet) and (dist>(distMin+0.05)):
            self.get_logger().info(f"run_approachCan: seems like I am approaching a wall - abort {dist=:.3f} {distMin=:.3f} {distMinL=:.3f} {distMinR=:.3f} {Lnum=} {Rnum=}")
            #Back up a bit
            self.driveOdom(-0.5, 0.5)
            next_state = "findCan"
            return next_state
        
        # TODO: Needs a time out
        if dist <= distCanDet :
            """
            Close enough to the can to grab it
            """
            self.get_logger().info(f"run_approachCan: at the can {dist=:.3f} {distMin=:.3f} {distMinL=:.3f} {distMinR=:.3f} {Lnum=} {Rnum=} {msg=}")
            next_state = "grabCan"
        
        elif math.isinf(dist) or distMinL==1000.0 or distMinR==1000.0 : # == 0.0 :
            """
            Can is close but not detected by the narrow FOV of the front sensor
            Use the L5 distances from sensors to rotate to the can
            """
            distMinLminusR = distMinL - distMinR
            if Lnum == 0 and Rnum == 0 :
                next_state = "findCan"
            # elif Lnum > Rnum :        
            #     msg.angular.z = 0.1 #0.05   
            # elif Rnum > Lnum :    
            #     msg.angular.z = -0.1 #-0.05
            # elif distMinL < distMinR :        
            elif distMinLminusR < 0.05:        
                msg.angular.z = 0.05 #0.05   
            # elif distMinR < distMinL :    
            elif distMinLminusR > 0.05 :    
                msg.angular.z = -0.05 #-0.05
            self.get_logger().info(f"run_approachCan: rotating to detect the can {dist=:.3f} {distMin=:.3f} {distMinL=:.3f} {distMinR=:.3f} {Lnum=} {Rnum=} {msg=}")
                
        elif not math.isinf(dist) : # < 0.65: # arbitrary 0.65
            """
            Move to approach the can
            Rotate Left when number of L data points > R data points
            Rotate Right when number of R data points > L data points
            Move towards can while range > 0.035
            """
            if (not math.isinf(dist) and (math.isinf(distMin)) or (math.fabs(dist - distMin) < 0.15)) :
                msg.linear.x = 0.1
            # if Lnum > Rnum :
            #     msg.angular.z = 0.1 #0.05
            # elif Rnum > Lnum :
            #     msg.angular.z = -0.1 #-0.05
            if distMinL < distMinR :        
                msg.angular.z = 0.1 #0.05   
            elif distMinR < distMinL :    
                msg.angular.z = -0.1 #-0.05
            self.get_logger().info(f"run_approachCan: approaching the can {dist=:.3f} {distMin=:.3f} {distMinL=:.3f} {distMinR=:.3f} {Lnum=} {Rnum=} {msg=}")
            
        else :
            """
            Not valid can range
            this can be handled better
            """
            self.get_logger().info(f"run_approachCan: can not in detectable range, start findCan {dist=} {distMin=} {distMinL=} {distMinR=} {Lnum=} {Rnum=} {msg=}")
            next_state = "findCan"
            
        self.cmd_vel_publisher.publish(msg)
        
        return next_state
    
    def run_grabCan(self) ->str:
        """
        Grab the can and go to next state
        Returns next state string
        """
        next_state = "grabCan"
        
        self.clawCmdClose()
        next_state = "gotoGoalOpening"
                               
        return next_state

    def run_gotoGoalOpening(self) ->str:
        """
        Go to the Goal opening location
        Returns next state string
        """
        next_state = "gotoGoalOpening"

        self.nav.clearAllCostmaps() 
        # self.gotoXY(2.0,0,30, obstacle_layer_enabled=True)
        
        # # The navigator stops with the edge of the robot at the XY coordinates
        # # The center of the robot needs to be at the XY coordinates
        # # a new XY coordinate needs to be calculated taking in consideration the 
        # # angle of the robot and the 0.180 radius of the circular robot       
        # calculate a new XY that takes into account the robot radius
        # x=2.0
        x = self.arenas[self.nav_arena]["goalOpeningX0"]
        y=0.0
        # get current pose to determine the angle offset
        (tf_OK,current_pose) = self.getCurrentPose()
        if tf_OK :
            xd = float(x) - current_pose.pose.position.x
            yd = float(y) - current_pose.pose.position.y
            # Calc angle to target XY coordinate
            a = math.atan2(yd,xd)
            d = math.sqrt(xd*xd + yd*yd)
            # adjust the distance to the can by robot radius to stop with center at xy
            d = d + self.robotRadius
            x = current_pose.pose.position.x + d*math.cos(a)
            y = current_pose.pose.position.y + d*math.sin(a)

            self.get_logger().info(f"run_gotoGoalOpening: adjusted {x=:.3f} {y=:.3f} {a=:.3f}")
        else : 
            self.get_logger().info(f"run_gotoGoalOpening: Failed to get current pose, goto non adjusted xy {x=:.3f} {y=:.3f}")

        self.gotoXY(x,y, 15.0,obstacle_layer_enabled=True)


        next_state = "gotoCanDrop"
        
        return next_state
        
    def run_gotoCanDrop(self) ->str:
        """
        Go to the drop location from the goal opening
        Returns next state string
        """
        next_state = "gotoCanDrop"
        
        
        # self.nav.driveOnHeading(0.5, 0.1, 10) # doesn't seem to exist in Humble?
        # "new" "MAC 2D planner and pure puruit/rotational shim rotates well
        x = self.arenas[self.nav_arena]["canDropoffX0"]
        y= 0.0
        self.gotoXY(x,y, 10, obstacle_layer_enabled=False)

        next_state = "dropCan"
        
        return next_state
    
    def run_dropCan(self) ->str:
        """
        Open claws to drop the can
        Returns next state string
        """
        next_state = "dropCan"

        self.clawCmdOpen()

        self.can_counter +=1

        next_state = "backupFromCan"
        
        return next_state
    
    def run_backupFromCan(self) ->str:  
        """
        Backup from the can so that it is not seen as an obstacle
        returns next state string
        """
        next_state = "backupFromCan"
        
        d = self.arenas[self.nav_arena]["canBackup"]

        self.driveOdom(-0.05, 0.25)
        self.rotateToAngle(0, 10)
        self.driveOdom(-(d-0.05), 0.25)

        if self.can_counter >=6 :
            next_state = "returnHome"
        else :
            next_state = "findCanAtGoal"
        
        return next_state

    def run_findCanAtGoal(self) ->str:  
        """
        Scan over 180 degrees for a can at the goal opening
        Stops when a can is detected and goes to find the can
        If a can is not detected it goes to a new location
        returns next state string
        """
        next_state = "findCanAtGoal"

        self.nav.clearAllCostmaps() 

        # point towards one side of the goal opening
        self.rotateToAngle(math.pi*0.4, 5)

        # Check if the can is detected while rotating 180 degrees 
        tf_OK = self.getCanTFOK()
        if not tf_OK :
            tf_OK = self.rotateCanDet(math.pi*1.2) 
        
        # Make sure can is still detected
        tf_OK = self.getCanTFOK()

        if tf_OK : 
            next_state = "gotoCanLocation"
        else :
            next_state = "gotoNewLocation"
        
        return next_state

    def run_gotoNewLocation(self) ->str:  
        """
        Go to a new location to search for a can
        Returns next state string
        """
        next_state = "gotoNewLocation"
              
        self.nav.clearAllCostmaps() 
        
        # Adjust offset so that center of robot is in the center of the arena
        new_locxy_list = self.new_locxy_dict[self.nav_arena]
        (x,y) = new_locxy_list[self.new_locxy_idx]
        if self.new_locxy_idx < len(new_locxy_list)-1 : 
            self.new_locxy_idx += 1
        else : 
            self.new_locxy_idx = 0

        self.gotoXY(x,y, 30)
        
        next_state = "findCan"
        
        return next_state

    def run_returnHome(self) ->str:  
        """
        Goto the home location xy = (0,0)
        Point towards goal similar to starting pose
        Returns next state string
        """
        next_state = "returnHome"
              
        self.nav.clearAllCostmaps() 

        # take into account the robot size to stop with center of robot at 0,0
        x = 0.0 - self.robotRadius
        y = 0.0
        self.rotateToAngle(math.pi , 5)
        self.gotoXY(x,y, 10)
        self.rotateToAngle(0, 5)
        
        next_state = "done"
        
        return next_state

    def run_done(self) ->str:  
        """
        Set the state machine to done
        This is the fianl state
        Returns next state string
        """
        next_state = "done"
        self.clawCmdClose()
        self.clawCmdOpen()
        self.clawCmdClose()
        self.clawCmdOpen()
        self.enable_6can_states = False
        return next_state

    # Sensors used to run 6 CAN
    def tofL4_rng_callback(self, msg: Range) :
        """
        Front range sensor message
        This sensor is used for the final can approach after T5 sensors quit detecting
        This call back also runs the 6 can state machine
        """
        self.tofL4_rng = msg.range
        self.run_6can_states()
        
    def tofL5L_pcd_callback(self, msg: PointCloud2) :
        """
        Front Left TOF sensors message
        This sensor (with the Right sensors) is used to align the can to the center
        compute the distance from the sensor
        """      
        nPoints = 16
        pc2_data = point_cloud2.read_points_list(msg, field_names=["x", "y"])
        data = []
        for i in range(15-nPoints,15) : 
            x = pc2_data[i].x
            y = pc2_data[i].y
            d = math.sqrt(x*x + y*y)
            data.append(d)
        #self.get_logger().info(f"tofL5L_pcd_callback: {pc2_data=} {data=}")
        self.tofL5L_pcd = data

    def tofL5R_pcd_callback(self, msg: PointCloud2) :
        """
        Front Right TOF sensors message
        This sensor (with the Left sensors) is used to align the can to the center
        compute the distance from the sensor
        """
        nPoints = 16
        pc2_data = point_cloud2.read_points_list(msg, field_names=["x", "y"])
        data = []
        for i in range(0,nPoints) : 
            x = pc2_data[i].x
            y = pc2_data[i].y
            d = math.sqrt(x*x + y*y)
            data.append(d)
        #self.get_logger().info(f"tofL5R_pcd_callback: {pc2_data=} {data=}")
        self.tofL5R_pcd = data

    # send a message to claw to open/close
    def clawCmdOpen(self) :
        """
        sends (publish) a message to open the claw
        """
        self.clawCmd(0, 100)
        
    def clawCmdClose(self) :
        """
        sends (publish) a message to close the claw
        """
        self.clawCmd(93, 100)
        
    def clawCmd(self, pct: int, msec: int) -> None:
        """
        sends (publish) a message to claw to open/close
        pct is percent claw closed (0 = 100% open)
        msec is how long the claw moves to the new position
        """
        cmd_json = {"claw": {"open": pct, "time": msec}}
        cmd_str = json.dumps(cmd_json)+"\0"
        self.robot_json_data_publish(cmd_str)

        # blocking wait for the expected claw movement time
        # blocking is OK since the robot should be stopped
        time.sleep(msec/1000.0)

    def robot_json_data_publish(self, data:str) -> None :
        msg = String()
        msg.data = data
        self.robot_json_publisher.publish(msg)


    def send_set_param_request(self, svc, name, value):
        """
        Set a parameter using the given param service
        command line example:
        cli -> ros2 param set /local_costmap/local_costmap obstacle_layer.enabled False
        """
        if isinstance(value, bool) :
            value_type = ParameterType.PARAMETER_BOOL
        else :
            value_type = None
        
        param = Parameter(
            name=name, 
            value=ParameterValue(
                type=value_type,
                bool_value=value
            )
        )

        # Doesthis need to be a global param for persistance?
        self.set_param_request.parameters = [param]
        
        # self.get_logger().info(f"send_set_param_request: Sending {name=} {value=}")
        future = svc.call_async(self.set_param_request)
        
        self.callback_set_param_done = False
        
        future.add_done_callback(partial(self.callback_set_param, name=name, value=value))
        
        # self.get_logger().info(f"send_set_param_request: waiting for callback")
        while not self.callback_set_param_done :
            time.sleep(0.1)
        # self.get_logger().info(f"send_set_param_request: callback wait done {name=} {value=}")

        if name=='tf_broadcast' and value==False :
            self.freeze_static_tf("map", "odom")
        

    def callback_set_param(self,future, name, value) :
        #SetParametersResult
        result = future.result()
        successful = result.results[0].successful
        self.set_param_successful = successful

        # self.get_logger().info(f"callback_set_param done {name=} {value=} {result=} {successful=}")
        self.callback_set_param_done = True

    def freeze_static_tf (self, parent: str, child: str) -> None:
        """
        make a static tf using the current tf values
        """
        tf_OK,pose = self.getOdomPose()
        
        x=pose.pose.position.x
        y=pose.pose.position.y
        self.get_logger().info(f"freeze_static_tf: {x=:.2f} {y=:.2f} {parent}->{child}")
        
        if pose != None :
            self.make_static_tf(self.tf_static_broadcasterOdom, parent, child, pose)
            # self.get_logger().info("freeze_static_tf: make tf_static_broadcasterOdom")
        
        
        

    ############ END Nav2 run stuff #############

    def make_static_tf(self, tf_static_broadcaster, 
                       parent: str, child: str, xyt) -> None:
        """
        
        """
        
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = parent
        t.child_frame_id = child
        
        # self.get_logger().info(f"make_static_tf: {xyt=} {type(xyt)=}")

        if isinstance(xyt,list) or isinstance(xyt,tuple) :
            t.transform.translation.x = float(xyt[0])
            t.transform.translation.y = float(xyt[1])
            t.transform.translation.z = 0.0
            quat = tf_transformations.quaternion_from_euler(0.0, 0.0, float(xyt[2])) #x,y,theta
            t.transform.rotation.x = quat[0]
            t.transform.rotation.y = quat[1]
            t.transform.rotation.z = quat[2]
            t.transform.rotation.w = quat[3]
        elif isinstance(xyt,PoseStamped) :
            t.transform.translation.x = xyt.pose.position.x
            t.transform.translation.y = xyt.pose.position.y
            t.transform.translation.z = xyt.pose.position.z
            t.transform.rotation.x = xyt.pose.orientation.x
            t.transform.rotation.y = xyt.pose.orientation.y
            t.transform.rotation.z = xyt.pose.orientation.z
            t.transform.rotation.w = xyt.pose.orientation.w
        else :
            self.get_logger().info(f"make_static_tf: invalid xyt type {type(xyt)=}")
            return
            
        tf_static_broadcaster.sendTransform(t)
        # self.get_logger().info(f"make_static_tf: {t=}")

    def on_map_timer(self) :
        #self.createMap()
        pass
            
    def publishEmptyMap(self,resolution_m, height_m, width_m, origin_x_m, origin_y_m) :
    
        width  = int(width_m/resolution_m)
        height = int(height_m/resolution_m)

                
        msg = OccupancyGrid()
        
        msg.header.frame_id = "map"

        msg.info.resolution = resolution_m
        msg.info.width  = width
        msg.info.height = height

        msg.info.origin.orientation.w = 1.0
        msg.info.origin.orientation.x = 0.0
        msg.info.origin.orientation.y = 0.0
        msg.info.origin.orientation.z = 0.0

        msg.info.origin.position.x = float(origin_x_m)
        msg.info.origin.position.y = float(origin_y_m)
        msg.info.origin.position.z = 0.0

        msg.data = []
        for i in range(0,height*width) : msg.data.append(0)

        self.map_msg_publisher.publish(msg)

    def createWPMap(self) :        
        resolution = 0.05
        self.publishEmptyMap(resolution, 10, 10, -5, -5)

    def create4CMap(self) :        
        resolution = 0.05
        self.publishEmptyMap(resolution, 10, 10, -5, -5)

    def createQTMap(self) :        
        resolution = 0.02
        height = 4*self.feetToMeter
        width = 9*self.feetToMeter
        origin_x = 0
        origin_y = float(-height/2)
        # self.publishEmptyMap(resolution, height, width, origin_x, origin_y)
        self.publishEmptyMap(resolution, 10, 10, -5, -5)

    def create6CMap(self) -> None:
        msg = OccupancyGrid()

        # leave header time 0
        msg.header.frame_id = "map"
        
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

    def joy_callback(self, msg: Joy) -> None:
        """
        game controller buttons select what to do
        """
        if self.XYLatched==False :
            self.gotoWaypoints        = msg.buttons[3]==1 # 1 = Y button pushed
            self.gotoCan              = msg.buttons[2]==1 # 1 = X button pushed
            self.gotoQtWaypoints      = msg.buttons[1]==1 # 1 = B button pushed
            self.goto4CornerWaypoints = msg.buttons[0]==1 # 1 = A button pushed

            # if   msg.buttons[3] : self.nav_ctrl["mode"] = "Waypoints"
            # elif msg.buttons[2] : self.nav_ctrl["mode"] = "6-can"
            # elif msg.buttons[1] : self.nav_ctrl["mode"] = "Quick-trip"
            # elif msg.buttons[0] : self.nav_ctrl["mode"] = "4-corner"
            # else : self.nav_ctrl["mode"] = "none"

        resetAxes = msg.buttons[6]==1 # 1 = select button pushed
        latchButton = msg.buttons[5]==1 # 1 = start button pushed
        arenaSelect = msg.axes[4] # right joystick up/dn -1.0 to 1.0
        
        if  self.XYLatched==False :
            if (   msg.buttons[2]==1 or  msg.buttons[3]==1  \
                or msg.buttons[0]==1 or  msg.buttons[1]==1) \
                and latchButton==True : 
                self.XYLatched = True
        else :
            if (    msg.buttons[2]==0 and msg.buttons[3]==0  \
                and msg.buttons[0]==0 and msg.buttons[1]==0) \
                and latchButton==True :
                self.XYLatched = False

        if arenaSelect >  0.5 : 
            if self.nav_arena != "dprg" :
                self.nav_arena = "dprg"
                self.get_logger().info(f"joy_callback: {arenaSelect=} {self.nav_arena=}")
        if arenaSelect < -0.5 : 
            if self.nav_arena != "home" :
                self.nav_arena = "home"
                self.get_logger().info(f"joy_callback: {arenaSelect=} {self.nav_arena=}")

        if resetAxes :
            self.cmd_vel_publisher.publish(Twist()) # Stop
            self.clawCmd(0, 100) #open claw
            rclpy.shutdown()

        # if (   msg.buttons[2]==1 or  msg.buttons[3]==1  \
        #     or msg.buttons[0]==1 or  msg.buttons[1]==1) :
        #     self.get_logger().info(f"joy_callback: XYAB button pushed {msg=} {self.gotoCan=}")
            

def main(args=None):
    rclpy.init(args=args)

    # Create instance and pass handle to the controller node
    nav = BasicNavigator()
    node = Roborama25ControllerNodeLc(nav)

    # rclpy.spin(node)
    # MultiThread for life cycle operation
    rclpy.spin(node, MultiThreadedExecutor()) 
    
    node.destroy_node()
    rclpy.shutdown()

# This code is needed to run .py file directly
if __name__ == '__main__':
    main()
