#!/usr/bin/env python3

import rclpy
import sys
import serial
import math
import time
import numpy as np
import random

from rclpy.node import Node
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from tf2_ros import Duration

from geometry_msgs.msg import Twist
from geometry_msgs.msg import TransformStamped

from sensor_msgs.msg import Joy
from std_msgs.msg import String, Header

from tf2_ros.transform_broadcaster import TransformBroadcaster
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

#from robo24_interfaces.srv import Claw
from datetime import datetime, timedelta

from rclpy.parameter import Parameter
import os

import json
from copy import copy, deepcopy


class Robo24DiynavNode(Node):

    pi = 3.14159265

    # waypoint simple 2D pose [x,y,theta]
    gotoWaypoints = 0
    gotoWaypoints_last = 0

    gotoCan = 0
    gotoCan_last = 0

    gotoQtWaypoints = 0
    gotoQtWaypoints_last = 0

    goto4CornerWaypoints = 0
    goto4CornerWaypoints_last = 0


    XYLatched = False
    calc_waypoint_hz = 20

    state = 0 
    state_last = 0
    wpstate = 0

    navTimerStart = 0

    tf_nanosec_last = 0
    tf_timeout_ns = 1000000000
    
    TF_Timeout_last = True

    wptf_nanosec_last = 0

    ft2m:float = 0.3048 # feet per meter

    # 6 can waypoints - home arena 7.5x7.0 9"start offset
    home_can6_CenterY = 6.75/2
    home_can6_startWaypoint = [0.0,0.0,0.0] # center 8" from bottom of arena
    home_can6_goalAlignWaypoint = [5.75*ft2m,0.0,0.0] # in front of goal entrance 1 ft into arena
    home_can6_CenterFWaypoint    = [(home_can6_CenterY+1)*ft2m,0.0,0.0] # center front of arena
    home_can6_CenterBWaypoint    = [(home_can6_CenterY-1)*ft2m,0.0,0.0] # center back of arena
    home_can6_goalEntryWaypoint = [6.75*ft2m,0.0,0.0] # middle of goal entrance
    home_can6_goalDropWaypoint  = [8.0*ft2m,0.0,0.0] # inside goal area
    home_can6_leftScanWaypoint  = [home_can6_CenterY*ft2m, 1.75*ft2m, 0.0]
    home_can6_rightScanWaypoint = [home_can6_CenterY*ft2m, -1.75*ft2m, 0.0]
#    home_can6_waypoints = ["home_can6_leftScanWaypoint","home_can6_rightScanWaypoint","home_can6_goalAlignWaypoint","home_can6_startWaypoint"]
    home_can6_waypoints = ["home_can6_CenterFWaypoint","home_can6_CenterBWaypoint"]
    
    # 4 corner waypoints - home arena 7.5x7.0 path 1ft from inside walls
    home_cor4_Waypoint0 = [0.0,0.0,0.0] # starting location (location it ends)
    home_cor4_Waypoint1 = [5.5*ft2m,0.0,0.0] 
    home_cor4_Waypoint2 = [5.5*ft2m,-5.0*ft2m,0.0] 
    home_cor4_Waypoint3 = [0.0,-5.0*ft2m,0.0] 
    home_cor4_waypoints = ["home_cor4_Waypoint1", "home_cor4_Waypoint2", "home_cor4_Waypoint3", "home_cor4_Waypoint0"]

    # quick trip waypoints - home arena
    home_qt_startWaypoint = [0.0, 0.0, 0.0]
    home_qt_turnWaypoint = [8.0*ft2m, 0.0, 0.0]
    home_qt_waypoints = ["home_qt_turnWaypoint", "home_qt_startWaypoint"]

    # Waypoints to continually travel to within arena
    # use can waypoints - home arean
    home_wp_waypoints = ["home_can6_leftScanWaypoint","home_can6_rightScanWaypoint","home_can6_goalAlignWaypoint","home_can6_startWaypoint"]
    
    
    # 6 can waypoints - dprg arena
    dprg_can6_CenterX = 9.5/2
    dprg_can6_startWaypoint = [0.0,0.0,0.0] # center 8" from bottom of arena
    dprg_can6_goalAlignWaypoint = [7.5*ft2m,   0.0,       0.0] # in front of goal entrance
    dprg_can6_CenterFWaypoint   = [(dprg_can6_CenterX+1)*ft2m, 0.0, 0.0]
    dprg_can6_CenterBWaypoint   = [(dprg_can6_CenterX-1)*ft2m, 0.0, 0.0]
    dprg_can6_goalEntryWaypoint = [9.0*ft2m,   0.0,       0.0] # middle of goal entrance
    dprg_can6_goalDropWaypoint  = [10.0*ft2m,  0.0,       0.0] # inside goal area
    dprg_can6_leftScanWaypoint  = [9.5/2*ft2m, 1.75*ft2m, 0.0]
    dprg_can6_rightScanWaypoint = [9.5/2*ft2m,-1.75*ft2m, 0.0]
    #dprg_can6_waypoints = ["dprg_can6_leftScanWaypoint","dprg_can6_rightScanWaypoint","dprg_can6_goalAlignWaypoint","dprg_can6_startWaypoint"]
    dprg_can6_waypoints = ["dprg_can6_CenterFWaypoint", "dprg_can6_CenterBWaypoint"]

    # 4 corner waypoints - dprg arena
    dprg_size = 12.0 *ft2m  # length, width of corner marked area
    dprg_offset = 1.5 *ft2m  # x,y offset from corner markers, clearance to stay outside marked area for odom drift error
    dprg_cor4_Waypoint0 = [0.0,0.0,0.0] # starting location offset from corner marker (location it ends at)
    dprg_cor4_Waypoint1 = [(dprg_size+2*dprg_offset),0.0                       ,0.0] 
    dprg_cor4_Waypoint2 = [(dprg_size+2*dprg_offset),-(dprg_size+2*dprg_offset),0.0] 
    dprg_cor4_Waypoint3 = [0.0                      ,-(dprg_size+2*dprg_offset),0.0] 
    dprg_cor4_waypoints = ["dprg_cor4_Waypoint1", "dprg_cor4_Waypoint2", "dprg_cor4_Waypoint3", "dprg_cor4_Waypoint0"]

    # quick trip waypoints - dprg arena
    dprg_qt_startWaypoint = [0.0, 0.0, 0.0]
    dprg_qt_turnWaypoint = [14.5*ft2m, 0.0, 0.0]
    dprg_qt_waypoints = ["dprg_qt_turnWaypoint", "dprg_qt_startWaypoint"]

    # Waypoints to continually travel to within arena
    # use can waypoints - dprg arean
    dprg_wp_waypoints = ["dprg_6can_leftScanWaypoint","dprg_can6_rightScanWaypoint","dprg_6can_goalAlignWaypoint","dprg_6can_startWaypoint"]


    # initial waypoint
    waypoint_num = 0

    nav_ctrl = {
        "mode":  "none",  # none, 6-can, 4-corner, Quick-trip, Waypoints
        "arena": "dprg", # home, dprg
        "state": "done", # init, running, paused, done
        # use arena and mode values to access waypoints
        "home" : {
            "6-can"      : home_can6_waypoints,
            "4-corner"   : home_cor4_waypoints,
            "Quick-trip" : home_qt_waypoints,
            "Waypoints"  : home_wp_waypoints
            },
        "dprg" : {
            "6-can"      : dprg_can6_waypoints,
            "4-corner"   : dprg_cor4_waypoints,
            "Quick-trip" : dprg_qt_waypoints,
            "Waypoints"  : dprg_wp_waypoints
            }
        }
    nav_ctrl_last = deepcopy(nav_ctrl)

    # TOF 8x8x3 data of interest to pull in the last 400mm before
    # grasping can
 
    tof8CanHor8x2:list[list[int]] = []

    tof8obstacle:list[int] = [-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1]


    canMaxDist = 3.0 # meters when searching for the can
    canMinDist = 0.450 # meters stop when driving towards can

    scaleRotationRate = 0.15 #0.15 #0.1
    scaleForwardSpeed = 0.6 #0.3 #0.2

    obstacleOffL = -1
    obstacleOffR = -1 
    obstacleMax = 400 # mm
    obstacleScale = 20.0
    obstacleZoff = 3 # offset from angle 0, 0 when not using TOF for can driving, otherwise maybe 3

    canZcnt = 0
    canZcross = 0
    az_last = 0
    
    tof8x8x3Ready = False

    navRunMode = "paused"

    slamEnabled = True
    slamWaitSec = 2.0 #3.0

    scanRotDir = 1 # scan rotation direction - used to alternate each scan
    scanWaypointsIdx = 0 # scan waypoint to goto next

    def __init__(self):
        super().__init__('robo24_diynav_node')


        self.declare_parameter('6can_arena', "dprg")
        param_6can_arena = self.get_parameter('6can_arena').value
        self.get_logger().info(f"{param_6can_arena=}")

        self.tf_OK_time_last = self.get_clock().now()

        self.wpstate3StartTime = self.get_clock().now()
        self.findCanStartTime = self.get_clock().now()

        self.joy_subscription = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.tof8x8x3_subscription = self.create_subscription(String, 'tof8x8x3_msg', self.tof8x8x3_callback, 10)
        self.robo24_json_subscription = self.create_subscription(String, 'robo24_json', self.robo24_json_callback, 10)

        self.robo24_modes_publisher = self.create_publisher(String, 'robo24_modes',10)
        self.robo24_json_publisher = self.create_publisher(String, 'robo24_json',10)

        # Calc new movement 10 times per second
        self.goto_timer = self.create_timer(1.0/self.calc_waypoint_hz, self.on_goto_timer)

        self.tf_buffer = Buffer(Duration(seconds=0,nanoseconds=0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Transform frame broadcasters
        self.tf_static_broadcasterOdom = StaticTransformBroadcaster(self)

        self.tf_static_broadcaster = StaticTransformBroadcaster(self)

        self.tf_broadcaster = TransformBroadcaster(self)

        # create world, map and odom static frames with no offset
        # map to odom becomes dynamic for transform moving robot relative to odom
        # when nav stack is used map to odom will also be dynamic to correct odom drift
        self.tf_static_broadcasterMap = StaticTransformBroadcaster(self)
        self.make_static_tf(self.tf_static_broadcasterMap,"world", "map", [0.0,0.0,0.0])

        #self.tf_static_broadcasterOdom = StaticTransformBroadcaster(self)
        #self.make_static_tf(self.tf_static_broadcasterOdom, "map", "odom", [0.0,0.0,0.0])

        #This TF is published dynamic in EFK filter node or in wheel_controller
        #self.make_static_tf("odom", "base_link", [0.0,0.0,0.0])

        self.tf_static_broadcaster0 = StaticTransformBroadcaster(self)
        self.tf_static_broadcaster1 = StaticTransformBroadcaster(self)
        self.tf_static_broadcaster2 = StaticTransformBroadcaster(self)
        self.tf_static_broadcaster3 = StaticTransformBroadcaster(self)
        self.tf_static_broadcaster4 = StaticTransformBroadcaster(self)
        self.tf_static_broadcaster5 = StaticTransformBroadcaster(self)
        self.tf_static_broadcaster6 = StaticTransformBroadcaster(self)
        self.tf_static_broadcaster7 = StaticTransformBroadcaster(self)
        self.tf_static_broadcaster8 = StaticTransformBroadcaster(self)
        self.tf_static_broadcaster9 = StaticTransformBroadcaster(self)
        self.tf_static_broadcaster10 = StaticTransformBroadcaster(self)
        self.tf_static_broadcaster11 = StaticTransformBroadcaster(self)
        self.tf_static_broadcaster12 = StaticTransformBroadcaster(self)
        self.tf_static_broadcaster13 = StaticTransformBroadcaster(self)

        self.create_static_waypoints()

        self.navTimerStart = time.time()

        self.get_logger().info("Robo24 DIY Navigation Started")
    #end init

    def create_static_waypoints(self) :

        if self.nav_ctrl["arena"] == "home" :
            self.make_static_tf(self.tf_static_broadcaster11,"map", "home_can6_startWaypoint",     self.home_can6_startWaypoint)
            self.make_static_tf(self.tf_static_broadcaster12,"map", "home_can6_CenterFWaypoint",    self.home_can6_CenterFWaypoint)
            self.make_static_tf(self.tf_static_broadcaster13,"map", "home_can6_CenterBWaypoint",    self.home_can6_CenterBWaypoint)
            self.make_static_tf(self.tf_static_broadcaster0, "map", "home_can6_goalDropWaypoint",  self.home_can6_goalDropWaypoint)
            self.make_static_tf(self.tf_static_broadcaster1, "map", "home_can6_goalEntryWaypoint", self.home_can6_goalEntryWaypoint)
            self.make_static_tf(self.tf_static_broadcaster2, "map", "home_can6_goalAlignWaypoint", self.home_can6_goalAlignWaypoint)
            self.make_static_tf(self.tf_static_broadcaster3, "map", "home_can6_leftScanWaypoint",  self.home_can6_leftScanWaypoint)
            self.make_static_tf(self.tf_static_broadcaster4, "map", "home_can6_rightScanWaypoint", self.home_can6_rightScanWaypoint)
            self.make_static_tf(self.tf_static_broadcaster5, "map", "home_qt_startWaypoint",       self.home_qt_startWaypoint)
            self.make_static_tf(self.tf_static_broadcaster6, "map", "home_qt_turnWaypoint",        self.home_qt_turnWaypoint)
            self.make_static_tf(self.tf_static_broadcaster7, "map", "home_cor4_Waypoint0",         self.home_cor4_Waypoint0)
            self.make_static_tf(self.tf_static_broadcaster8, "map", "home_cor4_Waypoint1",         self.home_cor4_Waypoint1)
            self.make_static_tf(self.tf_static_broadcaster9, "map", "home_cor4_Waypoint2",         self.home_cor4_Waypoint2)
            self.make_static_tf(self.tf_static_broadcaster10,"map", "home_cor4_Waypoint3",         self.home_cor4_Waypoint3)

        if self.nav_ctrl["arena"] == "dprg" :
            self.make_static_tf(self.tf_static_broadcaster11,"map", "dprg_can6_startWaypoint",     self.dprg_can6_startWaypoint)
            self.make_static_tf(self.tf_static_broadcaster12,"map", "dprg_can6_CenterFWaypoint",   self.dprg_can6_CenterFWaypoint)
            self.make_static_tf(self.tf_static_broadcaster13,"map", "dprg_can6_CenterBWaypoint",   self.dprg_can6_CenterBWaypoint)
            self.make_static_tf(self.tf_static_broadcaster0, "map", "dprg_can6_goalDropWaypoint",  self.dprg_can6_goalDropWaypoint)
            self.make_static_tf(self.tf_static_broadcaster1, "map", "dprg_can6_goalEntryWaypoint", self.dprg_can6_goalEntryWaypoint)
            self.make_static_tf(self.tf_static_broadcaster2, "map", "dprg_can6_goalAlignWaypoint", self.dprg_can6_goalAlignWaypoint)
            self.make_static_tf(self.tf_static_broadcaster3, "map", "dprg_can6_leftScanWaypoint",  self.dprg_can6_leftScanWaypoint)
            self.make_static_tf(self.tf_static_broadcaster4, "map", "dprg_can6_rightScanWaypoint", self.dprg_can6_rightScanWaypoint)
            self.make_static_tf(self.tf_static_broadcaster5, "map", "dprg_qt_startWaypoint",       self.dprg_qt_startWaypoint)
            self.make_static_tf(self.tf_static_broadcaster6, "map", "dprg_qt_turnWaypoint",        self.dprg_qt_turnWaypoint)
            self.make_static_tf(self.tf_static_broadcaster7, "map", "dprg_cor4_Waypoint0",         self.dprg_cor4_Waypoint0)
            self.make_static_tf(self.tf_static_broadcaster8, "map", "dprg_cor4_Waypoint1",         self.dprg_cor4_Waypoint1)
            self.make_static_tf(self.tf_static_broadcaster9, "map", "dprg_cor4_Waypoint2",         self.dprg_cor4_Waypoint2)
            self.make_static_tf(self.tf_static_broadcaster10,"map", "dprg_cor4_Waypoint3",         self.dprg_cor4_Waypoint3)


    def process_nav_cmd(self, cmd) :
        self.get_logger().info(f"process_nav_cmd {cmd=}")
        if "mode" in cmd :
            mode = cmd["mode"]
            self.nav_ctrl["mode"] = mode
        if "arena" in cmd :
            arena = cmd["arena"]
            self.nav_ctrl["arena"] = arena
        if "state" in cmd :
            state = cmd["state"]
            if state=="toggle" :
                if self.nav_ctrl["mode"] != "pause" :
                    self.toggled_mode = self.nav_ctrl["mode"]
                    self.nav_ctrl["mode"] = "pause"
                else :
                    self.nav_ctrl["mode"] = self.toggled_mode
                    
        self.create_static_waypoints()

    def robo24_json_callback(self, msg:String) -> None :
        self.get_logger().info(f"diynav robo24_json_callback {msg=}")
        try :
            packet = json.loads(msg.data)
        except Exception as ex:
            self.get_logger().error(f"diynav robo24_json_callback exception {ex}")
            return

        if "nav_cmd" in packet :
            nav_cmd = packet["nav_cmd"]
            self.process_nav_cmd(nav_cmd)

    def robo24_modes_callback(self, msg:String) ->None :
        self.get_logger().info(f"diynav robo24_modes_callback {msg=}")

        data = msg.data
        packet = None

        try :
            packet = json.loads(data)

        except Exception as ex:
            self.get_logger().error(f"diynav robo24_modes_callback exception {ex}")

        msg_str:str = None
        # send nav run state to nav publish
        if packet!=None :
            if "run_state" in packet :
                run_state = packet["run_state"]
                if run_state == "toggle" :
                    pass
                if self.navRunMode == "running" :
                    self.navRunMode = "paused"
                    msg_str = "{\"state\": \"paused\"}"
                if self.navRunMode == "paused" :
                    self.navRunMode = "running"
                    msg_str = "{\"state\": \"running\"}"

        if msg_str != None :
            self.robo24_modes_data_publish(msg_str)


        ############# Detect obstacle using TOF8x8x3 sensors ###############
    def calcObstacleAvoidance(self, obstacleZoff:int) -> None :
        """
            calcs self.obstacleOffL and R
        """
        
        # select first obstical scanning middle to side
        #g = 75.0/67 # 0 to 75 deg has cos range of 1 to 1/4
        g = 67.0/67 # 0 to 67 deg has cos range of 1 to 1/3
        o = 0.2 # max obstacle angle velocity to try to avoid it

        # detect obstacle on left side
        self.obstacleOffL = 0.0
        iL=-1
        for n in range(obstacleZoff, 12):
            r = math.radians((45.0/8) * (n+0.5) * g)
            k = math.cos(r)
            i = 11-n
            if self.tof8obstacle[i]>=0 and self.tof8obstacle[i]<=k*self.obstacleMax :
                self.obstacleOffL = k*o
                iL=i
                break
        
        # detect obstacle on right side
        self.obstacleOffR = 0.0
        iR=-1
        for n in range(obstacleZoff, 12):
            r = math.radians((45.0/8) * (n+0.5) * g)
            k = math.cos(r)
            i = 12+n
            if self.tof8obstacle[i]>=0 and self.tof8obstacle[i]<=k*self.obstacleMax:
                self.obstacleOffR = k*o
                iR=i
                break

        if(iL!=-1 or iR!=-1) :
            self.get_logger().info(f"{self.tof8obstacle = } [{iL}]{self.obstacleOffL = } [{iR}]{self.obstacleOffR = }")

    def robo24_modes_data_publish(self, data:str) -> None :
        msg = String()
        msg.data = data
        self.robo24_modes_publisher.publish(msg)

    def robo24_json_data_publish(self, data:str) -> None :
        msg = String()
        msg.data = data
        self.robo24_json_publisher.publish(msg)

    def diy_slam_enable(self, slam_enable:bool=True) ->None:
        """
            Enable or disable diy slam 
        """
        if slam_enable :
            self.robo24_modes_data_publish("SLAM ON")
        else :
            self.robo24_modes_data_publish("SLAM OFF")

    def find_can(self) -> tuple :
        """
            Locate a can using the TOF 8x8x3 array
            Use the 2nd and 3rd from bottom rows of 8 TOF sensors
            return(hdist, hrot, closeCanDet)
        """

        if len(self.tof8CanHor8x2) != 8 : return(-1,-1,-1)

        # process TOF from L->R first sequential valid distances used
        # 3 states 0=no valid yet, 1=valid current, 2=past valid
        closeCanDet = False
        hvalid:int = 750 #int(self.canMinDist*1.5*1000)
        hstate:int = 0
        hdist:int = 0
        hrot:int = 0
        l:int = 0
        r:int = 0
        ln:int = 0
        rn:int = 0
        
        canSect=None

        # process center 8 sensors in rows 2,3 from bottom
        #hmin = 1000 # arbitrary large
        distArray = []
        for i in range(0,8) :
            h2 = self.tof8CanHor8x2[i]
            hd=0
            hn=0
            # process data from each row as 1 value average
            if h2[0]>0 and h2[0]<hvalid :
                hd+=h2[0]
                hn+=1
            if h2[1]>0 and h2[1]<hvalid :
                hd+=h2[1]
                hn+=1
            if hn>0 :
                hd = int(hd/hn)
            else :
                hd = None
            distArray.append([hd])

        # calc distance diff between sensors
        distArray[0].append(None)
        for i in range(1,8) :
            d0 = distArray[i-1][0]
            d1 = distArray[i][0]
            if d0==None or d1==None :
                diff = None
            else :
                diff = int(d1 - d0)
            distArray[i].append(diff)

        # create distance sections of array
        sect = 0
        distArray[0].append(sect)
        for i in range(1,8) :
            d0 = distArray[i-1][1]
            d1 = distArray[i][1]
            if d1==None : 
                sect+=1
            elif math.fabs(d1) > 60 : 
                sect+=1
            distArray[i].append(sect)

        # find the "can" section 
        # This is normally the section with the smallest distance
        sect = 0
        n = 0
        dist = 0
        distAvg = None
        distMin = 10000

        for i in range(0,8) :
            # calc distance average
            da = distArray[i]
            d = da[0]
            df = da[1]
            s = da[2]
            # process 1 section at a time
            if s==sect :
                if d!=None :
                    dist += d
                    n+=1
            else :
                if n>0 : 
                    distAvg = int(dist/n)
                else : 
                    distAvg = None

            if s!=sect :
                # reset average measurement
                n = 0
                dist = 0
                if d!=None:
                    dist += d
                    n+=1

            # find distance minimum
            distArray[i].append(None) #[3]
            if s!=sect :
                distArray[i-1][3] = distAvg
                if distAvg!=None :
                    if distAvg<distMin : 
                        distMin = distAvg
            if i==7 :
                if n>0 : 
                    distAvg = int(dist/n)
                else : 
                    distAvg = None
                distArray[i][3] = distAvg
                if distAvg!=None :
                    if distAvg<distMin : 
                        distMin = distAvg
            sect = s

        # locate min averaged distance which should be the "can"
        canSect = None
        arrayIdx = None
        for i in range(0,8) :
            da = distArray[i]
            distArray[i].append("") #[4]
            if da[3]!= None :
                if da[3]==distMin :
                    distArray[i][4] = "can"
                    canSect = distArray[i][2]
                    arrayIdx = i
        if arrayIdx!=None :
            # get can distance offset
            hdist = distArray[arrayIdx][3]

            # get can rotation offset
            l = 0
            ln = 0
            r = 0
            rn = 0
            scnt = 0
            # process left sensors 0,1,2,3
            for i in range(0,4) :
                sect = distArray[i][2]
                dist = distArray[i][0]
                if sect!=None and dist!=None :
                    if sect==canSect :
                        scnt += 1
                        l  += dist
                        ln += 1
            # process right sensors 4,5,6,7
            for i in range(4,8) :
                sect = distArray[i][2]
                dist = distArray[i][0]
                if sect!=None and dist!=None :
                    if sect==canSect :
                        scnt += 1
                        r  += dist
                        rn += 1

            ld=0
            rd=0
            if ln > 0 :
                ld += l/ln
            if rn > 0 :
                rd += r/rn
            
            hdist = int(ld + rd)
            if ld>0 and rd>0 :
                hdist = int(hdist/2)

            hrot = -int(ld - rd)

        if hdist!=None :
            if (hdist>0 and hdist<(1000*self.canMinDist)) : closeCanDet = True
            
        #self.get_logger().info(f"{closeCanDet=} {canSect=} {hdist=} {hrot=} {distArray=}")

        return(hdist, hrot, closeCanDet)

    def new_scan_waypoint(self) -> None :
        l:int = len(self.can6_waypoints)
        self.newWaypoint = self.can6_waypoints[self.scanWaypointsIdx]
        self.scanWaypointsIdx+=1
        if self.scanWaypointsIdx>=l : self.scanWaypointsIdx=0

    def on_goto_timer(self) :

        (hdist, hrot, closeCanDet) = self.find_can()

        waypoint_distance = 0
        waypoint_theta = 0
        theta_err = 0
        state:int = self.state                           
        retVal:int = 0

        # flag that transform failed - used to find can
        tf_OK = False
        TF_Timeout = False

        #now = self.get_clock().now()
        now = rclpy.time.Time() # Gets time=0 (I think simulation time)
        
        nav_ctrl_arena = self.nav_ctrl["arena"] # home or dprg
        self.can6_waypoints = self.nav_ctrl[nav_ctrl_arena]["6-can"]
        cor4_waypoints = self.nav_ctrl[nav_ctrl_arena]["4-corner"]
        qt_waypoints   = self.nav_ctrl[nav_ctrl_arena]["Quick-trip"]
        wp_waypoints   = self.nav_ctrl[nav_ctrl_arena]["Waypoints"]

        nav_ctrl_mode = self.nav_ctrl["mode"]
        nav_ctrl_mode_last = self.nav_ctrl_last["mode"]

        if   nav_ctrl_mode == "Waypoints" :  waypoint = wp_waypoints[self.waypoint_num]
        elif nav_ctrl_mode == "6-can" :      waypoint = "can"
        elif nav_ctrl_mode == "Quick-trip" : waypoint = qt_waypoints[self.waypoint_num]
        elif nav_ctrl_mode == "4-corner" :   waypoint = cor4_waypoints[self.waypoint_num]

        # enable or disable diy slam when mode changes - takes a few seconds
        if nav_ctrl_mode!=nav_ctrl_mode_last :
            if nav_ctrl_mode=="4-corner" or nav_ctrl_mode=="Quick-trip" : self.slamEnabled= False
            else : self.slamEnabled = True
            self.diy_slam_enable(self.slamEnabled)

        # autonomous drive robot to waypoint or can
        # create /cmd_vel message 
        msg = Twist()
        
        if nav_ctrl_mode!=None :

            ############## GOTO WAYPOINTS STATES ############
            if nav_ctrl_mode=="Waypoints" :
                retVal = self.gotoWaypointStates(now, waypoint, msg)
                if retVal != 0 : 
                    # start goto next Waypoint 
                    self.waypoint_num += 1
                    # if self.waypoint_num >= len(self.home_wp_waypoints) :
                    if self.waypoint_num >= len(wp_waypoints) :
                        self.waypoint_num = 0
                    self.get_logger().info(f'Arrived at {waypoint}, next num = {self.waypoint_num} '.encode())
                
            ############### GOTO Quick Trip WAYPOINTS #################
            elif nav_ctrl_mode=="Quick-trip" :
                match self.state :
                  case 0:
                    retVal = self.gotoWaypointStates(now, waypoint, msg)
                    if retVal != 0 :
                        # start goto next Waypoint - only 2 waypoints 0,1 stop at #1
                        if self.waypoint_num<1 :
                            self.waypoint_num += 1
                        else :
                            self.state = 1
                            self.cor4Angle = -math.pi # 180 degree
                            self.cor4AngleTime = 0.0
                        self.get_logger().info(f'Arrived at QT {waypoint}, next num = {self.waypoint_num} '.encode())
                  case 1 :
                    # rotate to starting angle pose
                    dt = 1.0/self.calc_waypoint_hz # time interval sec
                    av = 2.0 #0.5 #rad/sec
                    dr = av*dt # rad/interval
                    self.cor4Angle += dr #new angle
                    msg.angular.z = -av / (2*math.pi) # NOTE: units are wrong!!!!
                    if self.cor4Angle >= 0.0 :
                        self.state = 2

                  case 2 :
                    # end
                    msg.angular.z = 0.0
                
            ############### GOTO 4 Corner WAYPOINTS #################
            elif nav_ctrl_mode=="4-corner" :

                match self.state :
                  case 0:
                    # goto waypoint
                    retVal = self.gotoWaypointStates(now, waypoint, msg)
                    if retVal != 0 :
                        # start goto next Waypoint - 4 waypoints 0,1,2,3 stop at #3 and rotate to begin angle
                        if self.waypoint_num<3 :
                            self.waypoint_num += 1
                        else :
                            self.state = 1
                        self.get_logger().info(f'Arrived at 4 CORNER {waypoint}, next num = {self.waypoint_num} '.encode())
                        self.cor4Angle = -(math.pi/2) # 90 degree
                        self.cor4AngleTime = 0.0

                  case 1 :
                    # rotate to starting angle pose
                    dt = 1.0/self.calc_waypoint_hz # time interval sec
                    av = 2.0 #0.5 #rad/sec
                    dr = av*dt # rad/interval
                    self.cor4Angle += dr #new angle
                    msg.angular.z = -av / (2*math.pi) # NOTE: units are wrong!!!!
                    if self.cor4Angle >= 0.0 :
                        self.state = 2

                  case 2 :
                    # end
                    msg.angular.z = 0.0


            ############### GOTO CAN STATES ###################
            elif nav_ctrl_mode=="6-can" :


                # reset can scan time out when started
                if nav_ctrl_mode!=nav_ctrl_mode_last :
                    self.wpScanStartTime = self.get_clock().now()
                    self.findCanStartTime = self.get_clock().now()
                    self.scanWaypointsIdx = 0
                    self.new_scan_waypoint()
                    self.canCount = 0


                ########## CAN STATES ##########
                self.findCanTimeout = 45.0
                if (self.get_clock().now() - self.findCanStartTime) >= rclpy.time.Duration(seconds=self.findCanTimeout) :
                    self.findCanStartTime = self.get_clock().now()
                    self.new_scan_waypoint()
                    self.state = 1
                    self.get_logger().info(f'Timeout finding can goto new {self.newWaypoint=}')

                # Timeout scanning for a can
                self.wpScanTimeout = 10.0
                if ((self.get_clock().now() - self.wpScanStartTime >= rclpy.time.Duration(seconds=self.wpScanTimeout))
                    and (self.state==0)) :
                    self.wpScanStartTime = self.get_clock().now()
                    self.new_scan_waypoint()
                    self.state = 1
                    self.get_logger().info(f'Timeout scanning for can goto new {self.newWaypoint=}')

                # Skip goto can using blob based TF when can is detected using TOF sensors
                if closeCanDet==True and (self.state==0 or self.state==1) :
                    self.state = 3
                    self.wpstate = 0
                    self.get_logger().info(f"[{state}->{self.state}] {closeCanDet=} {hdist=}")        

                match self.state:
                    
                # Scan for can
                  case 0: 
                    state = self.state
                    if self.state!=self.state_last :
                        # scan rotate different direction each time
                        self.scanRotDir *= -1
                        # start can scan timeout
                        self.wpScanStartTime = self.get_clock().now()
                        self.get_logger().info(f"State 0 started {self.wpScanStartTime=}")
                        
                    # scan for can and go to it, ignore center +-2 sensors when closing in on can
                    retVal = self.gotoWaypointStates(now, "can", msg, 2, True)
                    if retVal != 0 :
                        if retVal == 100 : 
                            self.state = 1
                            #self.get_logger().info(f'[{state}->{self.state}, {waypoint} {self.newWaypoint=}]')
                            self.get_logger().info(f'WP Timeout {retVal=} [{state}->{self.state}, {waypoint}]')
                        else :
                            self.state = 3
                            self.get_logger().info(f'WP {retVal=} [{state}->{self.state}, {waypoint}]')

                # Go to new scan waypoint after timeout
                  case 1: 
                    state = self.state
                    retVal = self.gotoWaypointStates(now, self.newWaypoint, msg, 0, True)
                    if retVal != 0 :
                        self.state = 0
                        # start can scan timeout
                        self.wpScanStartTime = self.get_clock().now()
                        self.get_logger().info(f'[{state}->{self.state}, {waypoint}] {self.wpScanStartTime=}')

                # removed state 2
                
                # Use TOF horizontal sensors to center can
                # timer rate can be faster than TOF data is ready
                  case 3:
                    state = self.state
                    if hdist==0 :
                        # TOF sensors have no data
                        self.state = 0
                    else :
                        if hdist>600 : 
                            self.state = 0 # too far, start searching again
                        else :
                            if hrot > 100 :
                                msg.angular.z = -0.05
                            elif hrot > 0 :
                                msg.angular.z = -0.01
                            elif hrot < -100 :
                                msg.angular.z = +0.05
                            elif hrot < 0 :
                                msg.angular.z = +0.01
                            
                            if msg.angular.z==0 : self.canZcnt += 1
                            else : self.canZcnt = 0

                            if self.az_last > 0 and msg.angular.z < 0 : self.canZcross += 1
                            if self.az_last < 0 and msg.angular.z > 0 : self.canZcross += 1
                            self.az_last = msg.angular.z

                            # after crossing zero multiple times or staying on zero for a while go to can
                            if self.canZcross > 2 or self.canZcnt > 6 :
                                self.state = 4
                                self.get_logger().info(f'[{state}->{self.state}, {waypoint}]')

                    # self.get_logger().info(f'[{state}->{self.state}, {waypoint}] {hdist = } {hrot = } Zct{self.canZcnt} Zcr{self.canZcross} fv{msg.linear.x} av{msg.angular.z}')
                                           
                # continue to can using TOF 8x8x3 Center sensors to go to within a few mm
                  case 4:
                    state = self.state
                    self.canDistGrab = 140
                    if hdist==0 :
                        # TOF sensors have no data
                        self.state = 0
                    else :
                        if hdist>600 : 
                            self.state = 0 # too far, start searching again
                        elif hdist <= self.canDistGrab : 
                            self.state = 5 # Can is close enough to grab
                            # reset find can timeout when grabbed
                            self.findCanStartTime = self.get_clock().now()
                        else :
                            # foward and rotational speed is variable a bit
                            if hrot > 100 :
                                msg.angular.z = -0.05
                            elif hrot > 0 :
                                msg.angular.z = -0.01
                            if hrot < -100 :
                                msg.angular.z = +0.05
                            elif hrot < 0 :
                                msg.angular.z = +0.01

                            if hdist > self.canDistGrab+150 :
                                msg.linear.x = 0.2 #0.1
                            elif hdist > self.canDistGrab :
                                msg.linear.x = 0.1 #0.05
                            else : # ???? blocked logic ???
                                self.state = 4
                                # stop all movement
                                msg.angular.z = 0.0
                                msg.linear.x = 0.0
                                self.get_logger().info(f'[{state}->{self.state}, {waypoint}]')

                    # self.get_logger().info(f'[{state}->{self.state}, {waypoint}] d{hdist} r{hrot} fv{msg.linear.x} av{msg.angular.z}')

                # grab can
                  case 5:
                    state = self.state
                    resp = "bad hdist"
                    grabMax = self.obstacleMax*1.5 #600
                    # NOTE: timer rate can be faster than TOF data is ready
                    if hdist<=0 or hdist>grabMax : 
                        self.state = 0
                    else :
                        resp = self.clawCmd(100, 1000)
                        self.state = 6
                        self.get_logger().info(f'[{state}->{self.state}, {waypoint}]')

                    # self.get_logger().info(f'[{state}->{self.state}, {waypoint}], {hdist = } {resp = }')
                
                # Bring can to Goal Entrance Waypoint before entering
                # This aligns the robot to the entrance before entering
                # block out center TOF obstical sensors which see the grabbed can
                  case 6:
                    state = self.state
                    if self.nav_ctrl["arena"] == "home" :
                        retVal = self.gotoWaypointStates(now, "home_can6_goalAlignWaypoint", msg, 4, True)
                    else :
                        retVal = self.gotoWaypointStates(now, "dprg_can6_goalAlignWaypoint", msg, 4, True)
                    if retVal != 0 :
                        self.state = 7
                        self.get_logger().info(f'[{state}->{self.state}, {waypoint}]')

                # Bring can to Goal Drop off location
                  case 7:
                    state = self.state
                    if self.nav_ctrl["arena"] == "home" :
                        retVal = self.gotoWaypointStates(now, "home_can6_goalDropWaypoint", msg, 0, False)
                    else :
                        retVal = self.gotoWaypointStates(now, "dprg_can6_goalDropWaypoint", msg, 0, False)

                    if retVal != 0 :
                        self.state = 8
                        self.get_logger().info(f'[{state}->{self.state}, {waypoint}]')

                # Drop off CAN
                  case 8:
                    state = self.state
                    resp = self.clawCmd(0, 1000)
                    self.state = 9
                    self.canCount += 1
                    self.get_logger().info(f'[{state}->{self.state}, {waypoint} {self.canCount=}]')

                # Backup 10cm from can dropped off in the Goal area
                  case 9:
                    state = self.state
                    if self.state_last != self.state :
                        self.navTimerStart = time.time()

                    if (time.time() - self.navTimerStart) <  1.0 :
                        msg.linear.x = -0.1 # reverse at 10 cm/sec for 1 sec
                    # elif self.canCount >= 6 :
                    #     msg.linear.x = 0.0 # stop
                    #     self.state = 11
                    #     self.get_logger().info(f'[{state}->{self.state}, {waypoint} {self.canCount=}]')
                    else :
                        msg.linear.x = 0.0 # stop
                        self.state = 10
                        self.get_logger().info(f'[{state}->{self.state}, {waypoint} {self.canCount=}]')

                 # Complete exiting Goal area, then start search for another CAN
                  case 10:
                    if state != self.state : self.wpstate = 0
                    state = self.state
                    retVal = self.gotoWaypointStates(now, self.newWaypoint, msg, 0, False)
                    if retVal != 0 :
                        self.state = 0
                        # reset find can timeout when starting new can find
                        self.findCanStartTime = self.get_clock().now()
                        self.get_logger().info(f'[{state}->{self.state}, {waypoint}]')

                  # go to start waypoint to finish 6-can, timed until getting there
                  case 11 :
                    if state != self.state : 
                        self.wpstate = 0
                    state = self.state
                    if self.nav_ctrl["arena"] == "home" :
                        retVal = self.gotoWaypointStates(now, "home_can6_startWaypoint", msg, 0, False)
                    else :
                        retVal = self.gotoWaypointStates(now, "dprg_can6_startWaypoint", msg, 0, False)
                    if retVal != 0 :
                        self.state = 12
                        self.get_logger().info(f'[{state}->{self.state}, {waypoint}]')
                    
                  # wait until restarted
                  case 12 :
                    if state != self.state :
                        self.get_logger().info(f">>>>>>>>>>>>>> FINISHED 6-CAN <<<<<<<<<<<<")
                    state = self.state
                    # keep timeout timers reset
                    self.findCanStartTime = self.get_clock().now()
                    self.wpScanStartTime = self.get_clock().now()

            ######## END STATES ############

            #msg.angular.z = 0.0
            #msg.linear.x = 0.0
            self.cmd_vel_publisher.publish(msg)

        if nav_ctrl_mode!=nav_ctrl_mode_last :
            # stop motion when button released
            # single twist /cmd_vel published so that joy can be used
            # without the fancy twist mux node
            msg = Twist()
            self.cmd_vel_publisher.publish(msg)


        self.nav_ctrl_last = deepcopy(self.nav_ctrl)

        self.gotoWaypoints_last = self.gotoWaypoints
        self.gotoCan_last = self.gotoCan
        self.gotoQtWaypoints_last = self.gotoQtWaypoints
        self.goto4CornerWaypoints_last = self.goto4CornerWaypoints

        self.state_last = state

    # send a message to claw to open/close
    #TODO: use custom msg (2 ints) instead of string
    def clawCmd(self, pct: int, msec: int) -> None:
        """
        sends (publish) a message to claw to open/close
        pct is percent claw closed (0 = 100%o pen)
        msec is how long the claw moves to the new position
        """
        cmd_json = {"claw": {"open": pct, "time": 1000}}
        cmd_str = json.dumps(cmd_json)+"\0"
        self.robo24_json_data_publish(cmd_str)

        # blocking wait for the expected claw movement time
        # blocking is OK since the robot should be stopped
        time.sleep(msec/1000.0)
    


    # go to a waypoint with TOF can options
    
    def gotoWaypointStates(self, now:time, waypoint:str, msg:Twist, obsOff:int = 0, obsEna:bool=False) -> int:
        """
        go to a waypoint with TOF can options
        returns 0:Running, 1:Completed, 10:Undefined state, 100:Timeout scanning for cans
        """

        # DEBUG: disable obstical detection
        #obsEna = False 

        retVal:int = 0 #Running

        waypoint_distance:float = 0.0
        waypoint_theta:float = 0.0
        theta_err:float = 0.0
                                      
        # flags that transform failed 
        tf_OK = False
        TF_Timeout = False
        tf_Tdiff = False
        
        tf_lookupTimeout = rclpy.duration.Duration(seconds=0.0)
        
        # get waypoint offsets relative to robot
        try:
            t1 = self.tf_buffer.lookup_transform(
                waypoint,
                'base_link',
                now,
                timeout=tf_lookupTimeout
                )
            
            # Get angle of waypoint relative to base_link
            e = euler_from_quaternion(
                t1.transform.rotation.x,
                t1.transform.rotation.y,
                t1.transform.rotation.z,
                t1.transform.rotation.w)
            robot_wp_angle = -e[2] #yaw
                    
            # XY from robot to waypoint ie distance from waypoint to source is negative
            # negate xy so distance is positive and gets smaller positive
            robot_wp_x = -t1.transform.translation.x
            robot_wp_y = -t1.transform.translation.y

            tf_nanosec = t1.header.stamp.nanosec
            if tf_nanosec != self.wptf_nanosec_last : 
                tf_Tdiff = True
            else : 
                tf_Tdiff = False
                #self.get_logger().info(f'{tf_Tdiff=} {tf_nanosec=} {self.wptf_nanosec_last=}')

            tf_OK = tf_Tdiff

            #self.get_logger().info(f'{robot_wp_angle = } {robot_wp_x = } {robot_wp_y = } {tf_OK = } {tf_nanosec = } {self.wptf_nanosec_last = } {t1 = }'.encode())
            
            self.wptf_nanosec_last = tf_nanosec
            
        except (LookupException, ConnectivityException, ExtrapolationException) as ex:
            self.get_logger().info(f'Could not transform base_link to {waypoint} - will try again: {ex}')
            tf_OK = False
                
        # Get XY for can relative to map, compare to goal entrance location to disqualify can in goal
        if tf_OK==True and waypoint=="can" :
            # if self.gotoCan == 1:
            if self.nav_ctrl["mode"]=="6-can" :
                try:
                    t2 = self.tf_buffer.lookup_transform (
                        'map',
                        'base_link',
                        now,
                        timeout=tf_lookupTimeout
                        )
                    # Get angle of can relative to base_link
                    e = euler_from_quaternion(
                        t2.transform.rotation.x,
                        t2.transform.rotation.y,
                        t2.transform.rotation.z,
                        t2.transform.rotation.w)
                    map_robot_angle = -e[2] #yaw
                            
                    # self.get_logger().info(f'{map_robot_angle = } {t2 = }'.encode())

                    # changed direction to goal!
                    if self.nav_ctrl["arena"] == "home" :
                        goalEntranceX = (self.home_can6_goalDropWaypoint[0] + self.home_can6_goalAlignWaypoint[0])/2.0
                    else :
                        goalEntranceX = (self.dprg_can6_goalDropWaypoint[0] + self.dprg_can6_goalAlignWaypoint[0])/2.0
                    robotCanDist = math.sqrt(t1.transform.translation.x**2 + t1.transform.translation.y**2)

                    robotCanX = robotCanDist * math.cos(map_robot_angle)

                    mapRobotX = t2.transform.translation.x
                    mapCanX = robotCanX + mapRobotX
                    # self.get_logger().info(f'{robotCanDist = } {mapRobotX = } {robotCanX = } {mapCanX = } {goalEntranceX = }'.encode())

                    if mapCanX > goalEntranceX :
                        # force TF invalid
                        tf_OK = False
                        pass
                    
                except (LookupException, ConnectivityException, ExtrapolationException) as ex:
                    self.get_logger().info(f'Could not transform map to base_link - will try again: {ex}')
                    #tf_OK = False


        if tf_OK==True :
            #self.get_logger().info(f'transform robo24 to {waypoint} {tf_nanosec}')
            x=robot_wp_x
            y=robot_wp_y
            waypoint_distance = math.sqrt((x*x) + (y*y))
            waypoint_theta = -math.atan2(y,x)

            theta_err =  robot_wp_angle - waypoint_theta
            # minimize rotation
            if theta_err > math.pi :
                theta_err -= 2*math.pi
            if theta_err < -math.pi :
                theta_err += 2*math.pi

        #if tf_Tdiff==True :
        if tf_OK==True :
            self.tf_OK_time_last = self.get_clock().now()

        else :
            # timeout to restart can search
            timeSinceLastOK = self.get_clock().now() - self.tf_OK_time_last
            if timeSinceLastOK.nanoseconds > self.tf_timeout_ns :
                if self.TF_Timeout_last == False :
                    self.get_logger().info(f"WP>>>>>>>>>>  TF Timeout  <<<<<<<<<<<")
                TF_Timeout = True
        
        # Process state transition to 0 with state transition flag 4 or 5
        if self.wpstate == 4 :
            self.wpstate = 0

        if self.wpstate == 5 :
            self.wpstate = 0
        
        ##########################################
        # Waypoint states
        match self.wpstate:

          case 0:
            state = self.wpstate

            if waypoint!="can":
                # Not in can mode so wait for TF detection
                if tf_OK==True :
                    self.wpstate = 1
                    self.get_logger().info(f'WP[{state}->{self.wpstate}, {waypoint}] {tf_OK=}')
            else :
                if tf_OK==True :
                    if waypoint_distance<self.canMaxDist: 
                        self.wpstate = 1 
                        self.get_logger().info(f'WP[{state}->{self.wpstate}, {waypoint}]  {tf_OK=} {waypoint_distance=}')
                        
                    else :
                        # locate can by rotating body until can is detected
                        msg.angular.z = self.scanRotDir * self.scaleRotationRate

                else :
                    # scan for a can
                    msg.angular.z = self.scanRotDir * self.scaleRotationRate


            # self.get_logger().info(f'WP[{state}->{self.wpstate}, {waypoint}] d{waypoint_distance} {tf_OK = } fv{msg.linear.x} av{msg.angular.z}')

        # rotate to point toward waypoint/can
          case 1:
            state = self.wpstate
            
            if TF_Timeout==True : 
                self.wpstate = 5
                self.get_logger().info(f'WP[{state}->{self.wpstate}, {waypoint}] {TF_Timeout = }{tf_OK = }')
            elif tf_OK == True: 
                # point to waypoint/can
                minD = 0.05
                # if waypoint=="can" : minD = self.canMinDist
                # if waypoint_distance>minD and abs(theta_err) > 0.05 :
                if abs(theta_err) > 0.05 :
                    # limit low end of velocity for theta err                
                    if theta_err>0 : err=theta_err+0.2
                    else :           err=theta_err-0.2
                    msg.angular.z = self.scaleRotationRate * err
                else :
                    # reset WP scan timeout when can or waypoint is located and pointed toward
                    self.wpScanStartTime = self.get_clock().now()
                    self.wpstate11StartTime = self.get_clock().now()
                    self.wpstate = 11
                    self.get_logger().info(f'WP[{state}->{self.wpstate}, {waypoint} {TF_Timeout = } {waypoint_distance = } {theta_err = } {tf_OK = }]')
                    
            else :
                pass
                #self.get_logger().info(f'WP[{state}->{self.wpstate}, {waypoint}]{TF_Timeout = } {tf_OK = }')

            # self.get_logger().info(f'WP[{state}->{self.wpstate}, {waypoint}] d{waypoint_distance} a{theta_err} {TF_Timeout = } {tf_OK = } fv{msg.linear.x} av{msg.angular.z}')

        # extra wait to perform SLAM odom correction
          case 11:
            state = self.wpstate
            if self.slamEnabled :
                # wait for time
                if (self.get_clock().now() - self.wpstate11StartTime) >= rclpy.time.Duration(seconds=self.slamWaitSec) :
                    self.wpstate = 2
                    self.get_logger().info(f'WP[{state}->{self.wpstate}, {self.slamEnabled} {waypoint} {retVal = }]')
            else :
                # dont wait for slam wall detection
                self.wpstate = 2
                self.get_logger().info(f'WP[{state}->{self.wpstate}, {waypoint} {retVal = }]')
            # self.get_logger().info(f'WP[{state}->{self.wpstate}, {waypoint}] {self.obstacleOffL = } {TF_Timeout = } {tf_OK = } d{waypoint_distance} a{theta_err} fv{msg.linear.x} av{msg.angular.z}')
    

        # drive to waypoint/can, stop at limit of blob distance detection capability
          case 2:
            state = self.wpstate
            if TF_Timeout==True : 
                self.wpstate = 4
                self.get_logger().info(f'WP[{state}->{self.wpstate}, {waypoint}]{TF_Timeout = } {tf_OK = }')

            elif tf_OK == True: 
                minD = 0.05 # distance to stop before waypoint
                #obsOff = 0 # obstacle avoidance sensor center offset about 8 deg each
                if waypoint=="can": minD = self.canMinDist
                    
                if waypoint_distance>minD :
                    # correct for angular offset as it drives forward
                    msg.angular.z = 8*self.scaleRotationRate/3 * theta_err

                    # avoid obstacles
                    if obsEna : 
                        self.calcObstacleAvoidance(obsOff)
                    else : 
                        self.obstacleOffR = self.obstacleOffL = 0.0

                    msg.angular.z += self.obstacleOffR - self.obstacleOffL

                    #start full max speed then slow down, HW deceleration is 1M/sec
                    if (self.obstacleOffR + self.obstacleOffL) == 0 : 
                        fwdSpeed = self.scaleForwardSpeed
                    else : 
                        fwdSpeed = 0.5 * self.scaleForwardSpeed

                    if waypoint_distance > 1.0 : 
                        msg.linear.x = fwdSpeed
                    else : 
                        msg.linear.x = (waypoint_distance+0.2) * fwdSpeed

                else :
                    self.wpstate3StartTime = self.get_clock().now()
                    self.wpstate = 3
                    self.get_logger().info(f'WP[{state}->{self.wpstate}, {waypoint}] {waypoint_distance=}')

            #self.get_logger().info(f"WP[{state}, {waypoint}] {waypoint_distance=} {theta_err=}")

            # pause with no movement to allow diyslam to make mapping odom correction
          case 3:
            state = self.wpstate
            if self.slamEnabled :
                # wait for time
                if (self.get_clock().now() - self.wpstate3StartTime) >= rclpy.time.Duration(seconds=self.slamWaitSec) :
                    self.wpstate = 4
                    retVal = 1 # finished
                    self.get_logger().info(f'WP[{state}->{self.wpstate}, {self.slamEnabled} {waypoint} {retVal = }]')
            else :
                # dont wait for slam wall detection
                self.wpstate = 4
                retVal = 1 # finished
                self.get_logger().info(f'WP[{state}->{self.wpstate}, {waypoint} {retVal = }]')
            # self.get_logger().info(f'WP[{state}->{self.wpstate}, {waypoint}] {self.obstacleOffL = } {TF_Timeout = } {tf_OK = } d{waypoint_distance} a{theta_err} fv{msg.linear.x} av{msg.angular.z}')
    
          case _:
            self.wpstate = 4
            retVal = 10 # undefined state
            self.get_logger().info(f'WP[{state}->{self.wpstate}, {waypoint} {retVal = }]')
            

        self.TF_Timeout_last - TF_Timeout

        return retVal

    #END def gotoWayPoints()

    # get TOF8x8x3 sensors [[[12,5],[13,5]], [[12,6],[13,6]], [[12,7],[13,7]]]
    def tof8x8x3_callback(self, msg: String) -> None:

        try:
            #split msg string into 193 seperate strings 1 for each element
            tofStrArray = msg.data.split(" ")

            # parse messsage of "name" then 192 integers (8 rows of 24 distances)
            if tofStrArray[0]!="TOF8x8x3" or len(tofStrArray)!=193:
                self.get_logger().error(f"TOF8x8x3 type message error: {msg.data}")
                return
    
            # select center 8 sensor in row 2,3 from bottom of 8x8x3 sensor array
            self.tof8CanHor8x2 = []
            for i in range(0,8) :
                h2:list = []
                for j in range(0,2) :
                    ########## dont understand why (9+i)gets center instaed of (8+i)
                    h2.append(int(tofStrArray[(9+i)+((5+j)*24)])) # from rows 5 or 6
                self.tof8CanHor8x2.append(h2)

            # select sensors on 3rd row from bottom (5) for obstacle avoidance
            for i in range(0,24):
                self.tof8obstacle[i] = int(tofStrArray[i+(5*24)+1]) #[i,5]

            self.tof8x8x3Ready = True
        except:
            self.get_logger().error(f"TOF8x8x3 parse message error: {msg.data}")
            return


        # self.get_logger().info(f"TOF8x8x3 Horiz  data of interest = {self.tof8CanHor}")
        # self.get_logger().info(f"TOF8x8x3 Center data of interest = {self.tof8CanCtr}")
        # self.get_logger().info(f"{self.tof8CanHor8x2=}")


    # This should be replaced with an action command from teleop_robo24
    # TODO: move to robo24_teleop_node - use robo24_json topic to control nav
    def joy_callback(self, msg: Joy) -> None:
        if self.XYLatched==False :
            self.gotoWaypoints        = msg.buttons[3] # 1 = Y button pushed
            self.gotoCan              = msg.buttons[2] # 1 = X button pushed
            self.gotoQtWaypoints      = msg.buttons[1] # 1 = B button pushed
            self.goto4CornerWaypoints = msg.buttons[0] # 1 = A button pushed

            if   msg.buttons[3] : self.nav_ctrl["mode"] = "Waypoints"
            elif msg.buttons[2] : self.nav_ctrl["mode"] = "6-can"
            elif msg.buttons[1] : self.nav_ctrl["mode"] = "Quick-trip"
            elif msg.buttons[0] : self.nav_ctrl["mode"] = "4-corner"
            else : self.nav_ctrl["mode"] = "none"

        resetAxes = msg.buttons[6] # 1 = select button pushed
        latchButton = msg.buttons[5] # 1 = select button pushed

        if  self.XYLatched==False :
            if (   msg.buttons[2]==1 or  msg.buttons[3]==1  \
                or msg.buttons[0]==1 or  msg.buttons[1]==1) \
                and latchButton==1 : 
                self.XYLatched = True
        else :
            if (    msg.buttons[2]==0 and msg.buttons[3]==0  \
                and msg.buttons[0]==0 and msg.buttons[1]==0) \
                and latchButton==1 :
                self.XYLatched = False

        if resetAxes :
            self.state = 0
            self.waypoint_num = 0
            self.clawCmd(0, 100) #open claw

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


    def broadcast_tf(self, parent: str, child: str, xyt: list ) -> None:
        # Create and broadcast the transform message 
        tfs = TransformStamped()
        tfs.header.stamp = self.get_clock().now().to_msg()
        tfs.header.frame_id = parent
        tfs._child_frame_id = child
        tfs.transform.translation.x = xyt[0]
        tfs.transform.translation.y = xyt[1]
        tfs.transform.translation.z = 0.0 #theta # for debug should be 0.0  

        q = quaternion_from_euler(0.0, 0.0, xyt[2]) #x,y,theta

        tfs.transform.rotation.x = q[0]
        tfs.transform.rotation.y = q[1]
        tfs.transform.rotation.z = q[2]
        tfs.transform.rotation.w = q[3]

#        self.get_logger().info(f"Broadcast {parent} {tfs = }".encode())

        self.tf_broadcaster.sendTransform(tfs)    


# simplified code for 2D robot
def quaternion_from_euler(ai: float, aj: float, ak: float) -> np.ndarray:
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

    node = Robo24DiynavNode()
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()

# This code is needed to run .py file directly
if __name__ == '__main__':
    main()

