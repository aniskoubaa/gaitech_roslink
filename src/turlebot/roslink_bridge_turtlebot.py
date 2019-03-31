#!/usr/bin/env python

import socket
import threading
import sys
import time
import json
import rospy
import tf
from std_msgs.msg import Empty
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from ardrone_autonomy.msg import  Navdata
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion


# import ROSLink constants 
from enums.ROSLINK_VERSION import ROSLINK_VERSION
from enums.ROS_VERSION import ROS_VERSION
from enums.ROBOT_TYPE import ROBOT_TYPE
from enums.ROBOT_STATE import ROBOT_STATE
from enums.MESSAGE_TYPE import MESSAGE_TYPE
from enums.ROBOT_MODE import ROBOT_MODE

# import ROSLink messages
from messages.ROSLinkHeader import ROSLinkHeader 
from messages.HeartBeat import HeartBeat
from messages.RobotStatus import RobotStatus
from messages.GlobalMotion import GlobalMotion
from messages.GPSRawInfo import GPSRawInfo
from messages.RangeFinderData import RangeFinderData

# import configuration parameters
from configuration.CONFIG import *
from configuration.ROSLINK_MESSAGE_PERIOD import *

# import state variables 
from states.ROSLinkStateVariables import ROSLinkStateVariables


class ROSLinkBridgeTurtlebot:
        
    '''''''''''''''''''''''''''''''''''''''''''''
    Represents ROSLink Bridge for AR Drone
    '''''''''''''''''''''''''''''''''''''''''''''
    def __init__(self):
        '''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
        Constructor: read and initialize robot and roslink bridge parameters
        '''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
        # initialize ROS node for this client
        rospy.init_node('roslink_bridge_ardrone_node', anonymous=True) 
        
        # init the parameters from launch file
        ROSLinkBridgeTurtlebot.init_params()
        
        #start ROS publishers
        ROSLinkBridgeTurtlebot.start_ros_publishers()
        
        #start ROS publishers
        ROSLinkBridgeTurtlebot.start_ros_subscribers()
        
        #start UDP server socket
        ROSLinkBridgeTurtlebot.init_servers() 
        
        #create threads for sending roslink messages
        ROSLinkBridgeTurtlebot.create_roslink_message_threads()
        
        #create threads for processing received roslink command messages
        ROSLinkBridgeTurtlebot.create_roslink_command_processing_thread()
    
        #sping ROS forever
        rospy.spin()
    
    @staticmethod
    def init_servers():
        ROSLinkBridgeTurtlebot.client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) #create server socket
        ROSLinkBridgeTurtlebot.client_socket.settimeout(2) #set a timeout on a blocking socket operation
        #ROSLinkBridgeTurtlebot.gcs_server_ip = rospy.get_param("/ground_station_ip", "127.0.0.1")
        #ROSLinkBridgeTurtlebot.gcs_server_ip = rospy.get_param("/ground_station_ip", "192.168.100.17")
        #ROSLinkBridgeTurtlebot.gcs_server_port =rospy.get_param("/ground_station_port", 10000)
	ROSLinkBridgeTurtlebot.gcs_server_ip = rospy.get_param("/ground_station_ip", "208.113.133.197")
        ROSLinkBridgeTurtlebot.gcs_server_port =rospy.get_param("/ground_station_port", 25500)
        ROSLinkBridgeTurtlebot.server_address  = ( ROSLinkBridgeTurtlebot.gcs_server_ip, ROSLinkBridgeTurtlebot.gcs_server_port)
        print ROSLinkBridgeTurtlebot.gcs_server_ip
        print ROSLinkBridgeTurtlebot.gcs_server_port 
        #while True:
        #    print 'sending ...'
        #    self.client_socket.sendto(json.dumps(self.build_heartbeat_message()), self.server_address)
        #    time.sleep(1)   
           
    @staticmethod
    def init_params():   
        rospy.loginfo('[ROSLink Bridge] reading initialization parameters')
        # get roslink version
        ROSLinkStateVariables.roslink_version = rospy.get_param("/roslink_version", ROSLINK_VERSION.ABUBAKR)  
        # get ROS version  
        ROSLinkStateVariables.ros_version = rospy.get_param("/ros_version", ROS_VERSION.INDIGO)    
        # get system id
        ROSLinkStateVariables.system_id = rospy.get_param("/system_id", 15)
        # get robot name
        ROSLinkStateVariables.robot_name = rospy.get_param("/robot_name", "TURTLEBOT")
        # get robot type
        ROSLinkStateVariables.type = rospy.get_param("/type", ROBOT_TYPE.ROBOT_TYPE_TURTLEBOT)
        # get owner id
        ROSLinkStateVariables.owner_id = rospy.get_param("/owner_id", 3)
        # get key
        ROSLinkStateVariables.key = rospy.get_param("/key", "1243-0000-0000-FGFG")
        
        # define periods of updates
        ROSLinkBridgeTurtlebot.heartbeat_msg_rate = rospy.get_param("/heartbeat_msg_period", ROSLINK_MESSAGE_PERIOD.ROSLINK_HEARTBEAT_MESSAGE_RATE)
        ROSLinkBridgeTurtlebot.robot_status_msg_rate = rospy.get_param("/robot_status_msg_period", ROSLINK_MESSAGE_PERIOD.ROSLINK_ROBOT_STATUS_MESSAGE_RATE)
        ROSLinkBridgeTurtlebot.global_motion_msg_rate = rospy.get_param("/global_motion_msg_period", ROSLINK_MESSAGE_PERIOD.ROSLINK_GLOBAL_MOTION_MESSAGE_RATE)
        ROSLinkBridgeTurtlebot.gps_raw_info_msg_rate = rospy.get_param("/gps_raw_info_msg_period", ROSLINK_MESSAGE_PERIOD.ROSLINK_GPS_RAW_INFO_MESSAGE_RATE)
        ROSLinkBridgeTurtlebot.range_finder_data_msg_rate = rospy.get_param("/range_finder_data_msg_period", ROSLINK_MESSAGE_PERIOD.ROSLINK_RANGE_FINDER_DATA_MESSAGE_RATE)

    #start ROS publishers 
    @staticmethod
    def start_ros_publishers():
        # ROS publishers: for executing ROSLink commands
        #ROSLinkBridgeTurtlebot.move_publisher = rospy.Publisher('/cmd_vel_mux/input/teleop',Twist, queue_size=10)    
        sROSLinkBridgeTurtlebot.move_publisher = rospy.Publisher('/cmd_vel',Twist, queue_size=10)    

    #start ROS subscribers
    @staticmethod
    def start_ros_subscribers():
        rospy.Subscriber("/odom", Odometry, ROSLinkBridgeTurtlebot.odometryCallback)
    
    @staticmethod   
    def odometryCallback(msg):
        #position 
        ROSLinkStateVariables.time_boot_ms=time.time()
        ROSLinkStateVariables.x= msg.pose.pose.position.x
        ROSLinkStateVariables.y= msg.pose.pose.position.y
        ROSLinkStateVariables.z= msg.pose.pose.position.z
        #orientation
        quaternion = (msg.pose.pose.orientation.x,msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        ROSLinkStateVariables.roll = euler[0]
        ROSLinkStateVariables.pitch = euler[1]
        ROSLinkStateVariables.yaw = euler[2]
        #twist: linear
        ROSLinkStateVariables.vx_truth = msg.twist.twist.linear.x
        ROSLinkStateVariables.vy_truth = msg.twist.twist.linear.y
        ROSLinkStateVariables.vz_truth = msg.twist.twist.linear.z
        #twist: angular
        ROSLinkBridgeTurtlebot.wx_truth = msg.twist.twist.angular.x
        ROSLinkStateVariables.wy_truth = msg.twist.twist.angular.y
        ROSLinkStateVariables.wz_truth = msg.twist.twist.angular.z
             
    
    #this method create threads for sending roslink message at specific rate
    @staticmethod
    def create_roslink_message_threads(): 
        ROSLinkBridgeTurtlebot.heatrbeat_message_thread = ROSLinkMessageThread(ROSLinkBridgeTurtlebot.client_socket, ROSLinkBridgeTurtlebot.server_address, MESSAGE_TYPE.ROSLINK_MESSAGE_HEARTBEAT, "heatrbeat_message_thread", ROSLinkBridgeTurtlebot.heartbeat_msg_rate)
        ROSLinkBridgeTurtlebot.robot_status_thread = ROSLinkMessageThread(ROSLinkBridgeTurtlebot.client_socket, ROSLinkBridgeTurtlebot.server_address, MESSAGE_TYPE.ROSLINK_MESSAGE_ROBOT_STATUS, "robot_status_thread", ROSLinkBridgeTurtlebot.robot_status_msg_rate)
        ROSLinkBridgeTurtlebot.global_motion_thread = ROSLinkMessageThread(ROSLinkBridgeTurtlebot.client_socket, ROSLinkBridgeTurtlebot.server_address, MESSAGE_TYPE.ROSLINK_MESSAGE_GLOBAL_MOTION, "global_motion_thread", ROSLinkBridgeTurtlebot.global_motion_msg_rate)
        ROSLinkBridgeTurtlebot.gps_raw_info_thread = ROSLinkMessageThread(ROSLinkBridgeTurtlebot.client_socket,  ROSLinkBridgeTurtlebot.server_address, MESSAGE_TYPE.ROSLINK_MESSAGE_GPS_RAW_INFO, "gps_raw_info_thread", ROSLinkBridgeTurtlebot.gps_raw_info_msg_rate)
        #ROSLinkBridgeTurtlebot.range_finder_data_thread = ROSLinkMessageThread(ROSLinkBridgeTurtlebot.client_socket, ROSLinkBridgeTurtlebot.server_address, "range_finder_data_thread", 0.333)
        
    @staticmethod
    def create_roslink_command_processing_thread(): 
        ROSLinkBridgeTurtlebot.command_processing_thread =  ROSLinkCommandProcessingThread(ROSLinkBridgeTurtlebot.client_socket, 'ROSLink Command Processing Thread')
    
    @staticmethod
    def static_build_roslink_header_message(message_id):
        header = ROSLinkHeader(ROSLinkStateVariables.roslink_version, ROSLinkStateVariables.ros_version, ROSLinkStateVariables.system_id, message_id, ROSLinkStateVariables.sequence_number,ROSLinkStateVariables.key)
        ROSLinkStateVariables.sequence_number = ROSLinkStateVariables.sequence_number + 1
        return header.__dict__
    
    @staticmethod 
    def static_build_heartbeat_message():
        message_header = ROSLinkBridgeTurtlebot.static_build_roslink_header_message(MESSAGE_TYPE.ROSLINK_MESSAGE_HEARTBEAT)
        heartbeat_message = HeartBeat(message_header, ROBOT_TYPE.ROBOT_TYPE_TURTLEBOT, ROSLinkStateVariables.robot_name, ROBOT_STATE.ROBOT_STATE_ACTIVE, ROSLinkStateVariables.owner_id ,ROBOT_MODE.ROBOT_STATE_UNKNOWN)
        return heartbeat_message.__dict__
    
    @staticmethod 
    def static_build_robot_status_message():
        message_header = ROSLinkBridgeTurtlebot.static_build_roslink_header_message(MESSAGE_TYPE.ROSLINK_MESSAGE_ROBOT_STATUS)
        robot_status_message = HeartBeat(message_header, 0, ROSLinkStateVariables.robot_name, 0, 0 ,0)
        return robot_status_message.__dict__
    
    @staticmethod
    def static_build_global_motion_message():
        message_header = ROSLinkBridgeTurtlebot.static_build_roslink_header_message(MESSAGE_TYPE.ROSLINK_MESSAGE_GLOBAL_MOTION) 
        global_motion_message = GlobalMotion(message_header, ROSLinkStateVariables.time_boot_ms , ROSLinkStateVariables.x, ROSLinkStateVariables.y, ROSLinkStateVariables.yaw, ROSLinkStateVariables.vx, ROSLinkStateVariables.vy, ROSLinkStateVariables.vz, ROSLinkStateVariables.wx, ROSLinkStateVariables.wy, ROSLinkStateVariables.wz, ROSLinkStateVariables.pitch, ROSLinkStateVariables.roll, ROSLinkStateVariables.yaw)
        return global_motion_message.__dict__  
    
    @staticmethod
    def static_build_gps_raw_info_message():
        message_header = ROSLinkBridgeTurtlebot.static_build_roslink_header_message(MESSAGE_TYPE.ROSLINK_MESSAGE_GPS_RAW_INFO) 
        global_motion_message = GPSRawInfo(message_header, ROSLinkStateVariables.time_boot_ms , ROSLinkStateVariables.fix_type, ROSLinkStateVariables.lat, ROSLinkStateVariables.lon, ROSLinkStateVariables.alt, ROSLinkStateVariables.eph, ROSLinkStateVariables.epv, ROSLinkStateVariables.vel, ROSLinkStateVariables.cog, ROSLinkStateVariables.satellites_visible)
        return global_motion_message.__dict__  
    
    
    @staticmethod
    def process_roslink_command_message(msg):
        #print 'msg is ', msg 
        command = json.loads(msg)
        print 'ROSLink command received ..'
        print msg
        
            
        if command['header']['message_id'] == MESSAGE_TYPE.ROSLINK_MESSAGE_COMMAND_TWIST:  
            print 'Twist command received'
            TwistCommand = Twist() 
            TwistCommand.linear.x = command['vx']
            TwistCommand.linear.y = command['vy'] 
            TwistCommand.linear.z = command['vz'] 
            TwistCommand.angular.x = command['wx']
            TwistCommand.angular.y = command['wy'] 
            TwistCommand.angular.z = command['wz']             
            print TwistCommand
            ROSLinkBridgeTurtlebot.move_publisher.publish (TwistCommand)
            
        if command['header']['message_id'] == MESSAGE_TYPE.ROSLINK_MESSAGE_COMMAND_GO_TO_WAYPOINT:  
            print 'GO tO Waypoint command received'
            GoToCommand = PoseStamped()
            GoToCommand.pose.position.x = command['x']
            GoToCommand.pose.position.y = command['y']
            GoToCommand.pose.orientation.z = command['z']
            
            print GoToCommand
            #ROSLinkBridgeTurtlebot.GoTo_publisher.publish(GoToCommand)
	    try:
		#rospy.init_node('nav_test', anonymous=False)
		navigator = GoToPose()

		# Customize the following values so they are appropriate for your location
		position = {'x': command['x'], 'y' : command['y']}
		quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0.000, 'r4' : 1.000}
		print 'position: '
		print position
		rospy.loginfo("Go to (%s, %s) pose", position['x'], position['y'])
		success = navigator.goto(position, quaternion)

		if success:
		    rospy.loginfo("Hooray, reached the desired pose")
		else:
		    rospy.loginfo("The base failed to reach the desired pose")

		# Sleep to give the last log messages time to be sent
		rospy.sleep(1)

	    except rospy.ROSInterruptException:
		rospy.loginfo("Ctrl-C caught. Quitting")
  
        # finds what kinds of message in that data  

class ROSLinkMessageThread ():
    count = 0
    def __init__(self, sock, server_address,message_type ,thread_name='noname', data_rate=1.0):
        self.count = self.count +1
        self.name = thread_name
        self.socket = sock
        self.server_address = server_address
        self.data_rate = data_rate
        self.roslink_message_type = message_type
        t = threading.Thread(target=self.run)
        t.setName(thread_name)
        t.start()
    
    def run ( self ):
        while True:
            self.count=self.count+1
            time.sleep(1.0/self.data_rate)
            print 'thread %s %d\n'%(self.name, self.count)
            #self.send(self.socket, json.dumps(self.roslink_message.__dict__))
            if (self.roslink_message_type == MESSAGE_TYPE.ROSLINK_MESSAGE_HEARTBEAT):
                self.send(self.socket, json.dumps(ROSLinkBridgeTurtlebot.static_build_heartbeat_message()))
            elif (self.roslink_message_type == MESSAGE_TYPE.ROSLINK_MESSAGE_ROBOT_STATUS):
                self.send(self.socket, json.dumps(ROSLinkBridgeTurtlebot.static_build_robot_status_message()))
            elif (self.roslink_message_type == MESSAGE_TYPE.ROSLINK_MESSAGE_GLOBAL_MOTION):
                self.send(self.socket, json.dumps(ROSLinkBridgeTurtlebot.static_build_global_motion_message()))
            elif (self.roslink_message_type == MESSAGE_TYPE.ROSLINK_MESSAGE_GPS_RAW_INFO):
                self.send(self.socket, json.dumps(ROSLinkBridgeTurtlebot.static_build_gps_raw_info_message()))

    '''
        Sending method
    '''
    def send (self, sock , msg): 
        self.socket.sendto(msg, self.server_address)   


class GoToPose():
    def __init__(self):

        self.goal_sent = False

	# What to do if shut down (e.g. Ctrl-C or failure)
	rospy.on_shutdown(self.shutdown)
	
	# Tell the action client that we want to spin a thread by default
	self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
	rospy.loginfo("Wait for the action server to come up")

	# Allow up to 5 seconds for the action server to come up
	self.move_base.wait_for_server(rospy.Duration(5))

    def goto(self, pos, quat):

        # Send a goal
        self.goal_sent = True
	goal = MoveBaseGoal()
	goal.target_pose.header.frame_id = 'map'
	goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = Pose(Point(pos['x'], pos['y'], 0.000),
                                     Quaternion(quat['r1'], quat['r2'], quat['r3'], quat['r4']))

	# Start moving
        self.move_base.send_goal(goal)

	# Allow TurtleBot up to 60 seconds to complete task
	success = self.move_base.wait_for_result(rospy.Duration(60)) 

        state = self.move_base.get_state()
        result = False

        if success and state == GoalStatus.SUCCEEDED:
            # We made it!
            result = True
        else:
            self.move_base.cancel_goal()

        self.goal_sent = False
        return result

    def shutdown(self):
        if self.goal_sent:
            self.move_base.cancel_goal()
        rospy.loginfo("Stop")
        rospy.sleep(1)

class ROSLinkCommandProcessingThread ( ):
    def __init__(self, sock,thread_name='noname'):
        self.name = thread_name
        self.socket = sock
        t = threading.Thread(target=self.run)
        t.setName(thread_name)
        t.start()
    def run ( self):
        print "Start ROSLINK Command Processing Thread"
        while True:
            try:
                msg, address = self.socket.recvfrom(MESSAGE_MAX_LENGTH)
                ROSLinkBridgeTurtlebot.process_roslink_command_message(msg)
            except socket.timeout:
                continue   


if __name__ == '__main__':
    print '\n************** Starting ROSLink Bridge for Turtlebot **************\n' 
    # initialize ROS node for this client
    myDroneBridge = ROSLinkBridgeTurtlebot() 
