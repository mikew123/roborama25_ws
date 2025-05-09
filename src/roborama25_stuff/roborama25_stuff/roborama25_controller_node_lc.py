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

from std_msgs.msg import String, Header

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
    
    d = 12.0
    t = 1.25 # put in center of target zones
    lengthQuickTrip = {
        "home" : 2, # meters
        "dprg" : (d+(2*1.25))*ft2m
    }
    
    # needs to be updated on site for actual size 8-15 ft sq
    d = 10.0
    t = 1.0 # distance from actual corner of square, 3ft clear zone
    size4corner = {
        "home" : 1.0, # meters
        "dprg" : (d+(2*t))*ft2m
    }
    
    home_can6Width:int = int((9.0 * ft2m)/mapResolution) # 6-can walls
    home_can6Height:int = int((7.0 * ft2m)/mapResolution)
    home_can6GoalArea:int = int((2.0 * ft2m)/mapResolution) # goal area outside walls
    home_can6GoalOpening:int = int((3.0 * ft2m)/mapResolution) #width of goal openin
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

    # flag to start running the 6can state machine
    enable_6can_states = False


    nav_arena:str = "home"

    # diyslamEnabled = True
    
    nav2_run_first_exec = True
    
    callback_set_param_done = False


    feetToMeter = 0.3048

    def __init__(self):
        super().__init__('roborama25_controller_node_lc')

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

        self.set_param_request = SetParameters.Request()

        self.robot_json_publisher = self.create_publisher(String, '/robot_json',10)
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
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

        self.nav = BasicNavigator()
        
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
            dist = self.gotoCanTF(30)
        
        if dist > 0 :
            self.get_logger().info(f"runWPoints: Robot is close to the can at {dist=}")
        else :
            self.get_logger().info(f"runWPoints: Robot failed to get close to the can")
            
            
        # # Drive waypoint 
        # for wp in [(2.5,0), (1,0.5), (1,-0.5), (0,0)] :
        #     status = self.gotoXY(wp[0],wp[1], 30)

        # status = self.rotateToAngle(0,10)
        # self.get_logger().info(f"runWPoints: final rotation {status=}")    
        
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
        r = self.robotRadius
        for wp in [(d+r,0), (d,-(d+r)), (0,-(d+r)), (0,r)] :
            status = self.gotoXY(wp[0],wp[1], 30)

        status = self.rotateToAngle(0,10)
        self.get_logger().info(f"run4Corner: final rotation {status=}")    
    
    def runQTrip(self) :
        """
        button B
        """
        self.get_logger().info(f"runQTrip: {self.nav_arena=} started (button B)")
        
        self.createQTMap()
        self.send_set_param_request(self.amcl_set_param_svc, 'tf_broadcast', False)
                
        # status = self.gotoXY(8*self.feetToMeter,0, 30)
        d = self.lengthQuickTrip[self.nav_arena]
        # need to account for robot radius since nav2 stops that amount
        # because of the obstacle mapping             
        r = self.robotRadius
        
        # self.get_logger().info(f"runQTrip: d={d} {self.nav_arena=}")
        # return
    
        status = self.gotoXY(d+r,0, 30)
        status = self.gotoXY(-r,0, 30)
        
        status = self.rotateToAngle(0,10)
        self.get_logger().info(f"runQTrip: final rotation {status=}")    
        
    def run6Can(self) :    
        """
        button X
        """
        self.get_logger().info(f"run6Can: {self.nav_arena=} started (button X) ")
        
        self.create6CMap()
        self.send_set_param_request(self.amcl_set_param_svc, 'tf_broadcast', True)
    
        # Enables the 6 can statemachine running in tofL4 callback
        self.enable_6can_states = True
        

        # status = self.rotateToAngle(0,10)
        # self.get_logger().info(f"run6Can: final rotation {status=}")    
         
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
                    if feedback.navigation_time.sec > t :
                            self.nav.cancelTask()
                except :
                    pass
                
        feedback = self.nav.getFeedback()
        result = self.nav.getResult()
        # self.get_logger().info(f"{feedback=} {result=}")
        
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('Goal succeeded!')
        elif result == TaskResult.CANCELED:
            self.get_logger().info('Goal was canceled!')
        elif result == TaskResult.FAILED:
            self.get_logger().info('Goal failed!')
        else :
            self.get_logger().info(f"nav.getResult() {result=}")

        return (result, feedback)

    def setInitialPose(self, pose) -> None:
        if self.lifecycle_state_active==False : return
        
        self.nav.setInitialPose(pose)
       
    def gotoPose(self, goto_pose, t):
        """
        Go to the pose within in the time limit
        """

        self.nav.goToPose(goto_pose)
        (result, feedback) = self.waitTaskComplete(t)
        x = feedback.current_pose.pose.position.x
        y = feedback.current_pose.pose.position.y
        (xx,yy,a) = tf_transformations.euler_from_quaternion(
            [feedback.current_pose.pose.orientation.x,
            feedback.current_pose.pose.orientation.y,
            feedback.current_pose.pose.orientation.z,
            feedback.current_pose.pose.orientation.w])
        t = feedback.navigation_time.sec
        
        return result
    
    def gotoCanTF(self, t) -> int:
        """
        Go to the can TF location, but stop 0.2m short
        Rotates to point to the can before moving to it
        gets new can position every 1 second as it approaches it
        Returns distance to the can, -1 if it did not suceed
        """
        if self.lifecycle_state_active==False : return -1
        
        d=100
        cnt=0
        while d>0.25 :
            (tf_OK, can_pose) = self.getCanPose()
            cnt+=1
            if tf_OK==False and cnt>5:
                self.get_logger().warn(f"gotoCanTF: Failed to get can pose")
                return -1
            if tf_OK==True :
                x = can_pose.pose.position.x
                y = can_pose.pose.position.y
                self.get_logger().info(f"gotoCanTF:b can TF {x=} {y=}")
                # get current pose to determine the angle offset
                # rotating to point to the desired is faster
                # maybe the navigation behavior can be "fixed" 
                (tf_OK, current_pose) = self.getCurrentPose()
                xd = float(x) - current_pose.pose.position.x
                yd = float(y) - current_pose.pose.position.y
                # Calc angle to target XY coordinate
                a = math.atan2(yd,xd)
                d = math.sqrt(xd*xd + yd*yd)
                
                # adjust the distance to the can to be 0.2m (~robot radius)
                # d = d - self.robotRadius
                d-=0.2
                x = current_pose.pose.position.x + d*math.cos(a)
                y = current_pose.pose.position.y + d*math.sin(a)
                
                goto_pose = self.createPose(x,y,a)
                self.get_logger().info(f"gotoXY: goto {x=} {y=} {d=} {a=}")

                # rotate to point to goto xy position before moving to it
                status = self.rotateToAngle(a,10)
                if status != TaskResult.FAILED :
                    status = self.gotoPose(goto_pose,1.0)
                if status == TaskResult.FAILED :
                    d=-1
                
        return d
    
    def gotoXY(self,x,y,t):
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
        if not tf_OK : return False
        
        xd = float(x) - current_pose.pose.position.x
        yd = float(y) - current_pose.pose.position.y
        # Calc angle to target XY coordinate
        a = math.atan2(yd,xd)

        goto_pose = self.createPose(x,y,a)
        self.get_logger().info(f"gotoXY: {current_pose=}, goto {x=} {y=} {a=}")

        # rotate to point to goto xy position before moving to it
        status = self.rotateToAngle(a,10)
        result = self.gotoPose(goto_pose,t)
        
        return result
    
    def rotate(self,a,t):
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
        spinThresh = 0.02 #0.01 # about 3 degrees
        
        # Loop until the timeout is reached or the task is complete
        while rclpy.ok() and elapsed_time <t:
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
            
            result = self.rotate(spin,t)
            
            current_time = self.get_clock().now()
            elapsed_time = (current_time - start_time).nanoseconds / 1e9  # Convert nanoseconds to seconds

            self.get_logger().info(f"rotateToAngle: rotate {result=} {a=} {aa=} {spin=} {elapsed_time=}")
            
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
            cnt += 1
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
        if tf_OK == False or cnt >= 5 :
            self.get_logger().info(f'getPoseFromTF: Failed to find transform map->{target_frame} after {cnt} tries {tf_OK=}')
            return (tf_OK,None) 
        
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
    
################################### 6CAN stuff ##########################    

    
    tofL4_rng = None
    tofL5L_pcd = None
    tofL5R_pcd = None
    canTF_ok = False
        
    current_6can_state = "findCan"
    next_6can_state =  current_6can_state
    
    def run_6can_states(self) :
        """
        Executes every tofL4_rng_callback
        Runs the upper level states for six can
        >findCan
        >gotoCanLocation
        >approachCan
        >grabCan
        >gotoCanDrop
        >dropCan
        >backupFromCan
        >gotoNewLocation
        """
        
        if self.lifecycle_state_active==False : return
        
        if self.enable_6can_states == False : return

        
        if self.current_6can_state != self.next_6can_state :
            self.get_logger().info(f"run_6can_states: State changed {self.current_6can_state=} {self.next_6can_state=}")
            
        self.current_6can_state = self.next_6can_state
        self.get_logger().info(f"run_6can_states: {self.current_6can_state=}")
        match self.current_6can_state :
            case "findCan" : 
                next_state = self.run_findCan()
            case "gotoCanLocation" :
                next_state = self.run_gotoCanLocation()
            case "approachCan" :
                next_state = self.run_approachCan()
            case "grabCan" :
                next_state = self.run_grabCan()
            case "gotoCanDrop" :
                next_state = self.run_gotoCanDrop()
            case "gotoGoalOpening" :
                next_state = self.run_gotoGoalOpening()
            case "dropCan" :
                next_state = self.run_dropCan()  
            case "backupFromCan" :
                next_state = self.run_backupFromCan()                  
            case "gotoNewLocation" :
                next_state = self.run_gotoNewLocation()
            case _ :
                self.get_logger().info(f"run_6can_states: Unknown state {self.current_6can_state=}")
                self.enable_6can_states = False

        self.next_6can_state = next_state
            
    findCanTime = 0.0
    
    def run_findCan(self):
        """
        Find a can using the camera can detection which generates a "can" TF
        """
        next_state = "findCan"
        # Check if the can transform is detected, discard pose
        (tf_OK, can_pose) = self.getCanPose()
        
        # rotate every 2 seconds to find the can
        # seems like it takes being still for 2 seconds is needed to detect can!!
        timer = time.monotonic() - self.findCanTime
        if timer > 2 :
            self.findCanTime = time.monotonic()
            if tf_OK==False and timer < 3:
                self.rotate(math.pi/16, 10)
                self.get_logger().info(f"run_findCan: Failed to find can TF, rotating")
            
        if tf_OK == True : 
            next_state = "gotoCanLocation"

        return next_state
    
    def run_gotoCanLocation(self) :
        """
        Try to go to the location of the can 
        """
        next_state = "gotoCanLocation"
        
        dist = self.gotoCanTF(30)
        
        if dist > 0 :
            self.get_logger().info(f"run_gotoCanLocation: Robot is close to the can at {dist=}")
            next_state = "approachCan"
        else :
            self.get_logger().info(f"run_gotoCanLocation: Robot failed to get close to the can")
            next_state = "findCan"
            
        return next_state
    
    def run_approachCan(self) :
        """
        Approach the can using the TOF sensors
        """
        next_state = "approachCan"
        msg = Twist()
        
        dist = self.tofL4_rng
        if dist == 0 :
            """
            Can is close but not detected by the narrow FOV of the front sensor
            Use the L5 sensors to approach the can
            """
            Lnum = 0
            i = 16
            for x in self.tofL5L_pcd :
                if x>0 and x<0.35 : Lnum += i
                i -= 1
            Rnum = 0
            i = 1
            for x in self.tofL5R_pcd :
                if x>0 and x<0.35 : Rnum += i
                i += 1
            if Lnum > Rnum :        
                msg.angular.z = 0.025   
            if Rnum > Lnum :    
                msg.angular.z = -0.025
            self.get_logger().info(f"run_approachCan: rotating to detect the can {dist=} {Lnum=} {Rnum=} {msg=}")
                
        elif dist > 0.050 :
            """
            Move to approach the can
            Rotate Left when number of L data points > R data points
            Rotate Right when number of R data points > L data points
            Move towards can while range > 0.035
            """
            msg.linear.x = 0.1
            Lnum = 0
            for x in self.tofL5L_pcd :
                if x>0 and x<dist : Lnum += 1
            Rnum = 0
            for x in self.tofL5R_pcd :
                if x>0 and x<dist : Rnum += 1
            if Lnum > Rnum :
                msg.angular.z = 0.025
            if Rnum > Lnum :
                msg.angular.z = -0.025
            self.get_logger().info(f"run_approachCan: approaching the can {dist=} {Lnum=} {Rnum=} {msg=}")
            
        elif dist > 0.010 : # ensure it is a valid distance, maybe > 0 ???
            """
            Close enough to the can to grab it
            """
            self.get_logger().info(f"run_approachCan: at the can {dist=} {msg=}")
            next_state = "grabCan"
            
        self.cmd_vel_publisher.publish(msg)
        
        return next_state
    
    def run_grabCan(self) :
        """
        Grab the can and go to next state
        """
        next_state = "grabCan"
        
        self.clawCmd(90, 100)
        next_state = "gotoGoalOpening"
                               
        return next_state

    def run_gotoGoalOpening(self) :
        """
        Go to the Goal opening location
        """
        next_state = "gotoGoalOpening"

        # self.nav.clearAllCostmaps() 
        self.gotoXY(2.25,0,30)
        next_state = "gotoCanDrop"
        
        return next_state
    
    def run_gotoCanDrop(self) :
        """
        Go to the drop location from the goal opening
        """
        next_state = "gotoCanDrop"
        
        self.rotateToAngle(0, 10)
        
        # stays disabled until after backup
        # self.send_set_param_request(self.amcl_set_param_svc, 'tf_broadcast', False)
        self.send_set_param_request(self.local_costmap_set_param_svc, 'obstacle_layer.enabled', False)
        
        self.nav.clearAllCostmaps() 
        # self.nav.driveOnHeading(0.5, 0.1, 10) # doesnt seem to exist in Humble?
        self.gotoXY(2.75,0,30)

        next_state = "dropCan"
        
        return next_state
    
    def run_dropCan(self) :
        """
        Open claws to drop the can
        """
        next_state = "dropCan"

        self.clawCmd(0, 100)
        next_state = "backupFromCan"
        
        return next_state
    
    def run_backupFromCan(self) :  
        """
        Backup from the can so that it is not seen as an obstacle
        """
        next_state = "backupFromCan"
        
        self.rotateToAngle(0, 10)
        # self.nav.clearLocalCostmap() 
        self.send_set_param_request(self.local_costmap_set_param_svc, 'obstacle_layer.enabled', False)
        self.nav.backup(0.35, 0.1, 10)
        self.waitTaskComplete(10)
        self.rotateToAngle(math.pi, 10)

        
        # self.send_set_param_request(self.amcl_set_param_svc, 'tf_broadcast', True)
        self.send_set_param_request(self.local_costmap_set_param_svc, 'obstacle_layer.enabled', True)

        self.nav.clearAllCostmaps() 
        
        next_state = "gotoNewLocation"
        
        return next_state

    def run_gotoNewLocation(self) :  
        """
        Placeholder for the 'run_gotoNewLocation' function.
        """
        next_state = "gotoNewLocation"
              
        self.gotoXY(2.0,0,30)
        # TODO: ???? Rotate to point straight out ????
        next_state = "findCan"
        
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
        """      
        pc2_data = point_cloud2.read_points_list(msg, field_names=["x", "y"])
        #data = [pc2_data[13].x, pc2_data[14].x, pc2_data[15].x] 
        data = []
        for i in range(10,15) : data.append(pc2_data[i].x)
        #self.get_logger().info(f"tofL5L_pcd_callback: {pc2_data=} {data=}")
                
        self.tofL5L_pcd = data

    def tofL5R_pcd_callback(self, msg: PointCloud2) :
        """
        Front Right TOF sensors message
        This sensor (with the Left sensors) is used to align the can to the center
        """
        pc2_data = point_cloud2.read_points_list(msg, field_names=["x", "y"])
        #data = [pc2_data[0].x, pc2_data[1].x, pc2_data[2].x] 
        data = []
        for i in range(0,5) : data.append(pc2_data[i].x)
        #self.get_logger().info(f"tofL5R_pcd_callback: {pc2_data=} {data=}")
        self.tofL5R_pcd = data

    # send a message to claw to open/close
    #TODO: use custom msg (2 ints) instead of string
    def clawCmd(self, pct: int, msec: int) -> None:
        """
        sends (publish) a message to claw to open/close
        pct is percent claw closed (0 = 100%o pen)
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
        
        # self.get_logger().info(f"send_set_param_request: sending {svc=} {name=} {value=} {param=}")
        future = svc.call_async(self.set_param_request)
        
        self.callback_set_param_done = False
        
        future.add_done_callback(partial(self.callback_set_param, name=name, value=value))
        
        # self.get_logger().info(f"send_set_param_request: waiting for callback")
        while not self.callback_set_param_done :
            time.sleep(0.1)
        # self.get_logger().info(f"send_set_param_request: callback wait done {name=} {value=}")

        # if name=='tf_broadcast' and value==False :
        #     self.freeze_static_tf("map", "odom")
        

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
        self.get_logger().info(f"freeze_static_tf: {pose=} parent={parent} child={child}")
        if pose != None :
            self.make_static_tf(self.tf_static_broadcasterOdom, "map", "odom", pose)
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

        # if resetAxes :
        #     self.state = 0
        #     self.waypoint_num = 0
        #     self.clawCmd(0, 100) #open claw

        # if (   msg.buttons[2]==1 or  msg.buttons[3]==1  \
        #     or msg.buttons[0]==1 or  msg.buttons[1]==1) :
        #     self.get_logger().info(f"joy_callback: XYAB button pushed {msg=} {self.gotoCan=}")
            

def main(args=None):
    rclpy.init(args=args)

    node = Roborama25ControllerNodeLc()
    # rclpy.spin(node)
    # MultiThread for life cycle operation
    rclpy.spin(node, MultiThreadedExecutor()) 
    
    node.destroy_node()
    rclpy.shutdown()

# This code is needed to run .py file directly
if __name__ == '__main__':
    main()
