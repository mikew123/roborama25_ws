import rclpy
import math
import tf_transformations
import threading

from rclpy.node import Node

from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle.node import LifecycleState, TransitionCallbackReturn


from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped
# from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
# from tf2_ros import Duration
# from tf2_ros.buffer import Buffer
# from tf2_ros.transform_listener import TransformListener

class Roborama25ControllerNodeLc(LifecycleNode):
    """
    Creates the 6-can arena maps for home and dprg 
    - rviz2 can display it
    - amcl uses it for localization
    """

    lifecycle_state_active = False
    

    ft2m:float = 0.3048 # feet to meters

    mapResolution:float = 0.1 # pixels = 10 cm sq
    
    home_can6Width:int = int(((9.0+(7/12.0)) * ft2m)/mapResolution) # 6-can walls
    home_can6Height:int = int(((7.0+(10/12.0)) * ft2m)/mapResolution)
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

    dprg_can6Width:int = int((7.0 * ft2m)/mapResolution) # 6-can walls
    dprg_can6Height:int = int((10.0 * ft2m)/mapResolution)
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


    nav_arena:str = "home"

    # diyslamEnabled = True

    def __init__(self):
        super().__init__('roborama25_controller_node_lc')

        self.get_logger().info(f"roborama25_controller_node Started {self.nav_arena=}")

    ############# Start Lifecycle stuff #############

    # Create ROS2 communications, connect to HW
    def on_configure(self, previous_state: LifecycleState):
        self.get_logger().info(f"IN on_configure")
        
        # publish the map 1/sec
        self.map_timer = self.create_timer(1.0, self.on_map_timer)
        self.map_timer.cancel()
        
        # the nav2 map saver qos needs to be transient local
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.map_msg_publisher = self.create_lifecycle_publisher(OccupancyGrid, 'map', qos_profile=qos_profile)

        self.nav = BasicNavigator()
        self.nav2Run_thread = threading.Thread(target=self.nav2_run)

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
        self.nav2Run_thread.start()
        
        return super().on_activate(previous_state)

        
    # Deactivate stuff used in shutdown, error
    def deactivate(self):
        self.lifecycle_state_active = False
        self.map_timer.cancel()
        self.nav2Run_thread.join()
        
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
        # tf_buffer = Buffer()
        # tf_listener = TransformListener(self.tf_buffer, self)

        self.setInitialPose(0,0,0, 0)    
        
        # print(f"{getCurrentPose(nav, tf_buffer)=}")
        
        for i in range(10):
            status = self.gotoPose(2.5,0,0, 60)
            print(f"{status}")
            status = self.rotate(math.pi,10)
            print(f"{status}")    
            status = self.gotoPose(0,0,math.pi, 60)
            print(f"{status}")
            status = self.rotate(math.pi,10)
            print(f"{status}")    
        
        
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
        # print(pose)
        return pose

    def waitTaskComplete(self,t) :
        if self.lifecycle_state_active==False : 
            self.nav.cancelTask()
        else :        
            while not self.nav.isTaskComplete():
                feedback = self.nav.getFeedback()
                # print(f"{feedback=}")
                if t>0 :
                    if feedback.navigation_time.sec > t :
                            self.nav.cancelTask()

        feedback = self.nav.getFeedback()
        result = self.nav.getResult()
        # print(f"{feedback=} {result=}")
        
        if result == TaskResult.SUCCEEDED:
            print('Goal succeeded!')
        elif result == TaskResult.CANCELED:
            print('Goal was canceled!')
        elif result == TaskResult.FAILED:
            print('Goal failed!')
        else :
            print(f"nav.getResult() {result=}")

        return (result, feedback)

    def setInitialPose(self,x,y,a,t) :
        if self.lifecycle_state_active==False : return
        
        self.nav.setInitialPose(self.createPose(x,y,a))

        self.nav.waitUntilNav2Active()
        
    def gotoPose(self,x,y,a,t) :
        if self.lifecycle_state_active==False : return
        
        self.nav.goToPose(self.createPose(x,y,a))
        (result, feedback) = self.waitTaskComplete(t)
        x = feedback.current_pose.pose.position.x
        y = feedback.current_pose.pose.position.y
        (xx,yy,a) = tf_transformations.euler_from_quaternion(
            [feedback.current_pose.pose.orientation.x,
            feedback.current_pose.pose.orientation.y,
            feedback.current_pose.pose.orientation.z,
            feedback.current_pose.pose.orientation.w])
        t = feedback.navigation_time.sec
        
        return (result, (x,y,a,t))

    def rotate(self,a,t):
        if self.lifecycle_state_active==False : return
        
        self.nav.spin(float(a),t)
        (result, feedback) = self.waitTaskComplete(0)
        a = feedback.angular_distance_traveled
        return (result,a,t)
        
    # def getCurrentPose(nav, tf_buffer):
    #     # get map->base_foot transform
    #     try:
    #         tf = tf_buffer.lookup_transform (
    #             'map',
    #             'base_footprint',
    #             nav.get_clock().now().to_msg(),
    #             timeout=rclpy.duration.Duration(seconds=0.0)
    #             )
    #         tf_OK = True

    #     except (LookupException, ConnectivityException, ExtrapolationException) as ex:
    #         print(f'Could not transform map->base_footprint: {ex}')
    #         tf_OK = False

    #     # translate wall points to align with map coordinates
    #     if tf_OK :
    #         # get x, y, theta from TF
    #         x:float = tf.transform.translation.x
    #         y:float = tf.transform.translation.y
    #         q:float = tf.transform.rotation
    #         # convert quaterion to euler
    #         (xx,yy,a) = tf_transformations.euler_from_quaternion(q.x, q.y, q.z, q.w)
    #     else :
    #         x=math.nan
    #         y=math.nan
    #         a=math.nan
            
    #     return (tf_OK,x,y,a)

    ############ END Nav2 run stuff #############

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
