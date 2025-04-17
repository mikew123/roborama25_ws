#!/usr/bin/env python3
import rclpy
import math
import tf_transformations

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped
# from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
# from tf2_ros import Duration
# from tf2_ros.buffer import Buffer
# from tf2_ros.transform_listener import TransformListener


def main(args=None):
    rclpy.init(args=args)
    
    nav = BasicNavigator()
    # tf_buffer = Buffer()
    # tf_listener = TransformListener(self.tf_buffer, self)

    setInitialPose(nav,0,0,0, 0)    
    
    # print(f"{getCurrentPose(nav, tf_buffer)=}")
    
    for i in range(10):
        status=gotoPose(nav,2.5,0,0, 60)
        print(f"{status}")
        status = rotate(nav,math.pi,10)
        print(f"{status}")    
        status=gotoPose(nav,0,0,math.pi, 60)
        print(f"{status}")
        status = rotate(nav,math.pi,10)
        print(f"{status}")    
        
        
    rclpy.shutdown()

def createPose(nav,x,y,a) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = nav.get_clock().now().to_msg()
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.position.z = 0.0
    (pose.pose.orientation.x,
     pose.pose.orientation.y,
     pose.pose.orientation.z,
     pose.pose.orientation.w) = tf_transformations.quaternion_from_euler(0.0,0.0,float(a))
    # print(pose)
    return pose

def waitTaskComplete(nav,t) :
    while not nav.isTaskComplete():
        feedback = nav.getFeedback()
        # print(f"{feedback=}")
        if t>0 :
            if feedback.navigation_time.sec >  t:
                    nav.cancelTask()

    feedback = nav.getFeedback()
    result = nav.getResult()
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

def setInitialPose(nav,x,y,a,t) :
    nav.setInitialPose(createPose(nav,x,y,a))

    nav.waitUntilNav2Active()
    
def gotoPose(nav,x,y,a,t) :
    nav.goToPose(createPose(nav,x,y,a))
    (result, feedback) = waitTaskComplete(nav,t)
    x = feedback.current_pose.pose.position.x
    y = feedback.current_pose.pose.position.y
    (xx,yy,a) = tf_transformations.euler_from_quaternion(
        [feedback.current_pose.pose.orientation.x,
        feedback.current_pose.pose.orientation.y,
        feedback.current_pose.pose.orientation.z,
        feedback.current_pose.pose.orientation.w])
    t = feedback.navigation_time.sec
    
    return (result, (x,y,a,t))

def rotate(nav,a,t):
    nav.spin(float(a),t)
    (result, feedback) = waitTaskComplete(nav,0)
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
   
if __name__ == "__main__" :
    main()
