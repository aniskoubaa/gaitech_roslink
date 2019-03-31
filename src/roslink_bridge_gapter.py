#!/usr/bin/env python

import socket
import threading
import sys
import time
import json
import rospy
# import tf
from std_msgs.msg import Empty, Float64
from nav_msgs.msg import Odometry
from mavros_msgs.msg import State, Mavlink
#from cv_bridge import CvBridge, CvBridgeError
import cv2
import base64

from mavros_msgs.srv import *
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import NavSatFix
import numpy as np


# import ROSLink constants 
from enums.ROSLINK_VERSION import ROSLINK_VERSION
from enums.ROS_VERSION import ROS_VERSION
from enums.ROBOT_TYPE import ROBOT_TYPE
from enums.ROBOT_STATE import ROBOT_STATE
from enums.MESSAGE_TYPE import MESSAGE_TYPE
from enums.ROBOT_MODE import ROBOT_MODE
from enums.FLIGHT_MODE import FLIGHT_MODE

# import ROSLink messages
from messages.ROSLinkHeader import ROSLinkHeader 
from messages.HeartBeat import HeartBeat
from messages.RobotStatus import RobotStatus
from messages.GlobalMotion import GlobalMotion
from messages.GPSRawInfo import GPSRawInfo
from messages.RangeFinderData import RangeFinderData
from messages.SetMode import SetMode
from messages.RoslinkImage import RoslinkImage

# import configuration parameters
from configuration.CONFIG import *
from configuration.ROSLINK_MESSAGE_PERIOD import *

# import state variables 
from states.ROSLinkStateVariables import ROSLinkStateVariables


class ROSLinkBridgeGapter:
        
    '''''''''''''''''''''''''''''''''''''''''''''
    Represents ROSLink Bridge for Gapter
    '''''''''''''''''''''''''''''''''''''''''''''
    def __init__(self):
        '''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
        Constructor: read and initialize robot and roslink bridge parameters
        '''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
        # initialize ROS node for this client
        rospy.init_node('roslink_bridge_gapter_node', anonymous=True) 
        
        # init the parameters from launch file
        ROSLinkBridgeGapter.init_params()
        
        #start ROS publishers
        ROSLinkBridgeGapter.start_ros_publishers()
        
        #start ROS publishers
        ROSLinkBridgeGapter.start_ros_subscribers()
        
        #start UDP server socket
        ROSLinkBridgeGapter.init_servers() 
        
        #create threads for sending roslink messages
        ROSLinkBridgeGapter.create_roslink_message_threads()
        
        #create threads for processing received roslink command messages
        ROSLinkBridgeGapter.create_roslink_command_processing_thread()
    
        #sping ROS forever
        rospy.spin()
    
    @staticmethod
    def init_servers():
        ROSLinkBridgeGapter.client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) #create server socket
        ROSLinkBridgeGapter.client_socket.settimeout(2) #set a timeout on a blocking socket operation
        ROSLinkBridgeGapter.gcs_server_ip = rospy.get_param("/ground_station_ip", "192.168.100.13")
        ROSLinkBridgeGapter.gcs_server_port =rospy.get_param("/ground_station_port", 25500)
        ROSLinkBridgeGapter.server_address  = ( ROSLinkBridgeGapter.gcs_server_ip, ROSLinkBridgeGapter.gcs_server_port)
        print ROSLinkBridgeGapter.gcs_server_ip
        print ROSLinkBridgeGapter.gcs_server_port 
        ROSLinkBridgeGapter.cap = cv2.VideoCapture(0)
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
        ROSLinkStateVariables.robot_name = rospy.get_param("/robot_name", "Gapter")
        # get robot type
        ROSLinkStateVariables.type = rospy.get_param("/type", ROBOT_TYPE.ROBOT_TYPE_GENERIC)
        # get owner id
        ROSLinkStateVariables.owner_id = rospy.get_param("/owner_id", 3)
        # get key
        ROSLinkStateVariables.key = rospy.get_param("/key", "1243-0000-0000-FGFG")
        
        # define periods of updates
        ROSLinkBridgeGapter.heartbeat_msg_rate = rospy.get_param("/heartbeat_msg_rate", ROSLINK_MESSAGE_PERIOD.ROSLINK_HEARTBEAT_MESSAGE_RATE)
        ROSLinkBridgeGapter.robot_status_msg_rate = rospy.get_param("/robot_status_msg_rate", ROSLINK_MESSAGE_PERIOD.ROSLINK_ROBOT_STATUS_MESSAGE_RATE)
        ROSLinkBridgeGapter.global_motion_msg_rate = rospy.get_param("/global_motion_msg_rate", ROSLINK_MESSAGE_PERIOD.ROSLINK_GLOBAL_MOTION_MESSAGE_RATE)
        ROSLinkBridgeGapter.gps_raw_info_msg_rate = rospy.get_param("/gps_raw_info_msg_rate", ROSLINK_MESSAGE_PERIOD.ROSLINK_GPS_RAW_INFO_MESSAGE_RATE)
        ROSLinkBridgeGapter.range_finder_data_msg_rate = rospy.get_param("/range_finder_data_msg_rate", ROSLINK_MESSAGE_PERIOD.ROSLINK_RANGE_FINDER_DATA_MESSAGE_RATE)
        ROSLinkBridgeGapter.ROSLing_Image_msg_rate = rospy.get_param("/ROSLing_Image_msg_rate", 1)

        # ROSLinkBridgeGapter.bridge = CvBridge()
        
    #start ROS publishers 
    @staticmethod
    def start_ros_publishers():
        # ROS publishers: for executing ROSLink commands
        #ROSLinkBridgeGapter.takeoff_publisher = rospy.Publisher('/ardrone/takeoff', Empty, queue_size=10)
        #ROSLinkBridgeGapter.land_publisher    = rospy.Publisher('/ardrone/land', Empty, queue_size=10)
        #ROSLinkBridgeGapter.reset_publisher   = rospy.Publisher('/ardrone/reset',Empty, queue_size=10)
        #ROSLinkBridgeGapter.move_publisher    = rospy.Publisher('/cmd_vel',Twist, queue_size=10)    
     
        ROSLinkBridgeGapter.move_publisher = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)

    #start ROS subscribers
    @staticmethod
    def start_ros_subscribers():
        #rospy.Subscriber("/ground_truth/state", Odometry, ROSLinkBridgeGapter.odometryCallback)
        #rospy.Subscriber("/ardrone/navdata", Navdata, ROSLinkBridgeGapter.navdataCallback)
        #rospy.Subscriber("/ardrone/front/image_raw/compressed", CompressedImage, ROSLinkBridgeGapter.frontCompressedImageCallback)
        #rospy.Subscriber("/ardrone/front/image_raw", Image, ROSLinkBridgeGapter.frontImageCallback)
        rospy.Subscriber("/mavros/global_position/raw/fix", NavSatFix, ROSLinkBridgeGapter.globalPositionCallback)
        rospy.Subscriber("/mavros/state", State, ROSLinkBridgeGapter.mavrosStateCallback)
        rospy.Subscriber("/mavros/global_position/rel_alt", Float64, ROSLinkBridgeGapter.altitudeCallback)
        rospy.Subscriber("/mavlink/from", Mavlink, ROSLinkBridgeGapter.mavlinkCallback)

    @staticmethod 
    def setFlightMode(mode):
        rospy.wait_for_service('/mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('/mavros/set_mode', mavros_msgs.srv.SetMode)
            #http://wiki.ros.org/mavros/CustomModes for custom modes
            isModeChanged = flightModeService(custom_mode=FLIGHT_MODE.MODEs[mode]) #return true or false
        except rospy.ServiceException, e:
            print "service set_mode call failed: %s. GUIDED Mode could not be set. Check that GPS is enabled"%e



    @staticmethod 
    def setGuidedMode():
        rospy.wait_for_service('/mavros/set_mode')
        try:
        	flightModeService = rospy.ServiceProxy('/mavros/set_mode', mavros_msgs.srv.SetMode)
        	#http://wiki.ros.org/mavros/CustomModes for custom modes
        	isModeChanged = flightModeService(custom_mode='GUIDED') #return true or false
        except rospy.ServiceException, e:
        	print "service set_mode call failed: %s. GUIDED Mode could not be set. Check that GPS is enabled"%e
    
    @staticmethod     
    def setStabilizeMode():
        rospy.wait_for_service('/mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('/mavros/set_mode', mavros_msgs.srv.SetMode)
            #http://wiki.ros.org/mavros/CustomModes for custom modes
            isModeChanged = flightModeService(custom_mode='STABILIZE') #return true or false
        except rospy.ServiceException, e:
            print "service set_mode call failed: %s. v Mode could not be set"%e

    @staticmethod 
    def setLandMode():
        rospy.wait_for_service('/mavros/cmd/land')
        try:
            landService = rospy.ServiceProxy('/mavros/cmd/land', mavros_msgs.srv.CommandTOL)
            #http://wiki.ros.org/mavros/CustomModes for custom modes
            isLanding = landService(altitude = 0, latitude = 0, longitude = 0, min_pitch = 0, yaw = 0)
        except rospy.ServiceException, e:
            print "service land call failed: %s. The vehicle cannot land "%e
    
    @staticmethod           
    def setArm():
        rospy.wait_for_service('/mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService(True)
        except rospy.ServiceException, e:
            print "Service arm call failed: %s"%e
      
    @staticmethod     
    def setDisarm():
        rospy.wait_for_service('/mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService(False)
        except rospy.ServiceException, e:
            print "Service Disarm call failed: %s"%e
    
    @staticmethod 
    def setTakeoffMode(user_altitude):
        print "Test take off"
        rospy.wait_for_service('/mavros/cmd/takeoff')
        try:
            takeoffService = rospy.ServiceProxy('/mavros/cmd/takeoff', mavros_msgs.srv.CommandTOL) 
            takeoffService(altitude = user_altitude, latitude = 0, longitude = 0, min_pitch = 0, yaw = 0)
        except rospy.ServiceException, e:
            print "Service takeoff call failed: %s"%e

    @staticmethod
    def globalPositionCallback(msg):
        global latitude
        global longitude
        ROSLinkStateVariables.lat = msg.latitude
        ROSLinkStateVariables.lon = msg.longitude
        #print ("longitude: %.7f" %longitude)
        #print ("latitude: %.7f" %latitude) 
    
    @staticmethod
    def mavlinkCallback(msg):
        # in this array ith emlent = (2^(i*8+1))
        mod_numbers=[1,512,131072,33554432,4294967296,1099511627776,281474976710656,72057594037927936]


        # this part jsut for simulater, in real drone we have /mavros/state topic 

        if(msg.msgid==MESSAGE_TYPE.MAVLINK_MESSAGE_HEARTBEAT):
        	pass;
        #     payload_data = msg.payload64[0] #the first element of the array (in heartbeat message we have jsut one (8 bytes))
        #     payload_bytes = [0]*8

            

        #     x =0;
        #     while(x<7):
        #         # parseing bytes form int as binnary digits e.g to extract ith byte as number  = (payload % (2^(8*i+1)))/ (2^((8*i)-8))
        #         payload_bytes[x]= payload_data % mod_numbers[x+1] # to get rid of digits on the left
        #         payload_bytes[x] = payload_bytes[x] / mod_numbers[x] #to get rid of digits on the right 
        #         x+=1;

            
        #     # no '%' for last one becose no digits at left
        #     payload_bytes[7] = payload_data / mod_numbers[7]
                        
        #     custom_mode_bytes = payload_data % mod_numbers[4]  # or as an array of bytes [payload_bytes[3],payload_bytes[2],payload_bytes[1],payload_bytes[0]]

        #     base_mode_byte = payload_bytes[6] 

        #     system_status_byte = payload_bytes[7] 



            # ROSLinkStateVariables.state = int(custom_mode_bytes);
            
            # # explination : 
            # # to avoid changing the structure of robot state message I have to add the information of both mode and arming in one vareble 
            # # so to parse armign status just divide by 100 you will get 1 if armed 0 if not 
            # # to get the flight mode  mod by 100 (state%100) you will get the flight mode  digits 

            # #the flag  that shows if it is armed or not is the last flag 2^7 so to parse it we basicly divide by 2^7 
            # arm_flag = base_mode_byte / 128 
            # if( arm_flag==1):
            #     ROSLinkStateVariables.state+=100;

            
            
            # # MAV_STATE_UNINIT = 0; /* Uninitialized system, state is unknown. | */
            # # MAV_STATE_BOOT = 1; /* System is booting up. | */
            # # MAV_STATE_CALIBRATING = 2; /* System is calibrating and not flight-ready. | */
            # # MAV_STATE_STANDBY = 3; /* System is grounded and on standby. It can be launched any time. | */
            # # MAV_STATE_ACTIVE = 4; /* System is active and might be already airborne. Motors are engaged. | */
            # # MAV_STATE_CRITICAL = 5; /* System is in a non-normal flight mode. It can however still navigate. | */
            # # MAV_STATE_EMERGENCY = 6; /* System is in a non-normal flight mode. It lost control over parts or over the whole airframe. It is in mayday and going down. | */
            # # MAV_STATE_POWEROFF = 7; /* System just initialized its power-down sequence, will shut down now. | */
            # # MAV_STATE_ENUM_END = 8; 

            # # to get the flight mode just divie by 100 then  mod by 1000 (state%1000) you will get the arming  digit
            # ROSLinkStateVariables.state+=1000*system_status_byte;

            
        elif(msg.msgid==MESSAGE_TYPE.MAVLINK_GPS_RAW_INT):
            print '\n\n\n'
            print msg.payload64
            l = len(hex(msg.payload64[1]))

            #print 'unsigned lat ' , int(hex(msg.payload64[1])[(l-8):l],16) # lat
            #print 'unsigned lon ' , int(hex(msg.payload64[1])[2:l-8],16) # int
            #print 'alt (mm)  ', int(hex(msg.payload64[2])[8:16],16); # alt
            
           
            
            print int(msg.payload64[3])

            payload3 = int(msg.payload64[3])
            
            ROSLinkStateVariables.satellites_visible = ((payload3% mod_numbers[6])/mod_numbers[5])
            ROSLinkStateVariables.fix_type = ((payload3% mod_numbers[5])/mod_numbers[4])
	    print(ROSLinkStateVariables.satellites_visible)
            



    @staticmethod
    def altitudeCallback(msg):
        ROSLinkStateVariables.altitude = msg.data;
        

    #not used now 
    @staticmethod  
    def mavrosStateCallback(msg):
    	# explination : 
            # to avoid changing the structure of robot state message I have to add the information of both mode and arming in one vareble 
            # so to parse armign status just divide by 100 you will get 1 if armed 0 if not 
            # to get the flight mode  mod by 100 (state%100) you will get the flight mode  digits 
        system_status = msg.system_status*1000

        armed = msg.armed
        flightMode = msg.mode
        if (armed):
            armInt=100;
        else:
            armInt=0;
        modeInt=FLIGHT_MODE.MODEs[flightMode]
        ROSLinkStateVariables.state = armInt+modeInt+system_status;
       



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
        ROSLinkStateVariables.wx_truth = msg.twist.twist.angular.x
        ROSLinkStateVariables.wy_truth = msg.twist.twist.angular.y
        ROSLinkStateVariables.wz_truth = msg.twist.twist.angular.z
        
        #print self.x
        #print self.y
     
    @staticmethod   
    def navdataCallback(msg):
        ROSLinkStateVariables.time_boot_ms=time.time()
        ROSLinkStateVariables.seq_number = ROSLinkStateVariables.seq_number +1;
        ROSLinkStateVariables.vx = msg.vx
        ROSLinkStateVariables.vy = msg.vy
        ROSLinkStateVariables.vz = msg.vz
        ROSLinkStateVariables.wx = msg.ax
        ROSLinkStateVariables.wy = msg.ay
        ROSLinkStateVariables.wz = msg.az
        ROSLinkStateVariables.battery = msg.batteryPercent
        ROSLinkStateVariables.state = msg.state
        ROSLinkStateVariables.magX = msg.magX
        ROSLinkStateVariables.magY = msg.magY
        ROSLinkStateVariables.magZ = msg.magZ
        ROSLinkStateVariables.pressure = msg.pressure
        ROSLinkStateVariables.temp = msg.temp
        ROSLinkStateVariables.wind_speed = msg.wind_speed
        ROSLinkStateVariables.wind_angle = msg.wind_angle
        ROSLinkStateVariables.rotX = msg.rotX
        ROSLinkStateVariables.rotY = msg.rotY
        ROSLinkStateVariables.rotZ = msg.rotZ
        ROSLinkStateVariables.altitude = msg.altd
        ROSLinkStateVariables.motor1 = msg.motor1
        ROSLinkStateVariables.motor2 = msg.motor2
        ROSLinkStateVariables.motor3 = msg.motor3
        ROSLinkStateVariables.motor4 = msg.motor4
        ROSLinkStateVariables.tags_count = msg.tags_count
        ROSLinkStateVariables.tags_type = msg.tags_type
        
        
        
    
    #this method create threads for sending roslink message at specific rate
    @staticmethod
    def create_roslink_message_threads(): 
        ROSLinkBridgeGapter.heatrbeat_message_thread = ROSLinkMessageThread(ROSLinkBridgeGapter.client_socket, ROSLinkBridgeGapter.server_address, MESSAGE_TYPE.ROSLINK_MESSAGE_HEARTBEAT, "heatrbeat_message_thread", ROSLinkBridgeGapter.heartbeat_msg_rate)
        ROSLinkBridgeGapter.robot_status_thread = ROSLinkMessageThread(ROSLinkBridgeGapter.client_socket, ROSLinkBridgeGapter.server_address, MESSAGE_TYPE.ROSLINK_MESSAGE_ROBOT_STATUS, "robot_status_thread", ROSLinkBridgeGapter.robot_status_msg_rate)
        ROSLinkBridgeGapter.global_motion_thread = ROSLinkMessageThread(ROSLinkBridgeGapter.client_socket, ROSLinkBridgeGapter.server_address, MESSAGE_TYPE.ROSLINK_MESSAGE_GLOBAL_MOTION, "global_motion_thread", ROSLinkBridgeGapter.global_motion_msg_rate)
        ROSLinkBridgeGapter.gps_raw_info_thread = ROSLinkMessageThread(ROSLinkBridgeGapter.client_socket,  ROSLinkBridgeGapter.server_address, MESSAGE_TYPE.ROSLINK_MESSAGE_GPS_RAW_INFO, "gps_raw_info_thread", ROSLinkBridgeGapter.gps_raw_info_msg_rate)
        ROSLinkBridgeGapter.roslink_image_thread = ROSLinkMessageThread(ROSLinkBridgeGapter.client_socket,  ROSLinkBridgeGapter.server_address, MESSAGE_TYPE.ROSLINK_MESSAGE_ROSLINK_IMAGE, "roslink_image_thread", ROSLinkBridgeGapter.ROSLing_Image_msg_rate )

        #ROSLinkBridgeGapter.range_finder_data_thread = ROSLinkMessageThread(ROSLinkBridgeGapter.client_socket, ROSLinkBridgeGapter.server_address, "range_finder_data_thread", 0.333)
        
    @staticmethod
    def create_roslink_command_processing_thread(): 
        ROSLinkBridgeGapter.command_processing_thread =  ROSLinkCommandProcessingThread(ROSLinkBridgeGapter.client_socket, 'ROSLink Command Processing Thread')
    
    @staticmethod
    def static_build_roslink_header_message(message_id):
        header = ROSLinkHeader(ROSLinkStateVariables.roslink_version, ROSLinkStateVariables.ros_version, ROSLinkStateVariables.system_id, ROSLinkStateVariables.owner_id, message_id, ROSLinkStateVariables.sequence_number,ROSLinkStateVariables.key)
        ROSLinkStateVariables.sequence_number = ROSLinkStateVariables.sequence_number + 1
        return header.__dict__
    
    @staticmethod 
    def static_build_heartbeat_message():
        message_header = ROSLinkBridgeGapter.static_build_roslink_header_message(MESSAGE_TYPE.ROSLINK_MESSAGE_HEARTBEAT)
        heartbeat_message = HeartBeat(message_header, ROSLinkStateVariables.type, ROSLinkStateVariables.robot_name, ROBOT_STATE.ROBOT_STATE_ACTIVE, ROSLinkStateVariables.owner_id ,ROBOT_MODE.ROBOT_STATE_UNKNOWN)
        return heartbeat_message.__dict__
    
    @staticmethod 
    def static_build_robot_status_message():
        message_header = ROSLinkBridgeGapter.static_build_roslink_header_message(MESSAGE_TYPE.ROSLINK_MESSAGE_ROBOT_STATUS)
        robot_status_message = RobotStatus(message_header,ROSLinkStateVariables.state, [0], [0], 0, 0 ,ROSLinkStateVariables.battery)
        return robot_status_message.__dict__
    
    @staticmethod
    def static_build_global_motion_message():
        message_header = ROSLinkBridgeGapter.static_build_roslink_header_message(MESSAGE_TYPE.ROSLINK_MESSAGE_GLOBAL_MOTION) 
        global_motion_message = GlobalMotion(message_header, ROSLinkStateVariables.time_boot_ms , ROSLinkStateVariables.x, ROSLinkStateVariables.y, ROSLinkStateVariables.altitude, ROSLinkStateVariables.vx, ROSLinkStateVariables.vy, ROSLinkStateVariables.vz, ROSLinkStateVariables.wx, ROSLinkStateVariables.wy, ROSLinkStateVariables.wz, ROSLinkStateVariables.pitch, ROSLinkStateVariables.roll, ROSLinkStateVariables.yaw)
        return global_motion_message.__dict__  
    
    @staticmethod
    def static_build_gps_raw_info_message():
        message_header = ROSLinkBridgeGapter.static_build_roslink_header_message(MESSAGE_TYPE.ROSLINK_MESSAGE_GPS_RAW_INFO) 
        global_motion_message = GPSRawInfo(message_header, ROSLinkStateVariables.time_boot_ms , ROSLinkStateVariables.fix_type, ROSLinkStateVariables.lat, ROSLinkStateVariables.lon, ROSLinkStateVariables.alt, ROSLinkStateVariables.eph, ROSLinkStateVariables.epv, ROSLinkStateVariables.vel, ROSLinkStateVariables.cog, ROSLinkStateVariables.satellites_visible)
        return global_motion_message.__dict__  
    
    @staticmethod
    def static_build_Roslink_Image_message():

        ret, frame = ROSLinkBridgeGapter.cap.read()
        frame = cv2.resize(frame,(160,120))

        encoded = cv2.imencode('.png',frame)[1]
        data = base64.encodestring(encoded)
	print(len(data))
        message_header = ROSLinkBridgeGapter.static_build_roslink_header_message(MESSAGE_TYPE.ROSLINK_MESSAGE_ROSLINK_IMAGE) 
        
        ROSLink_image_message = RoslinkImage(message_header, ROSLinkStateVariables.owner_id ,ROSLinkStateVariables.key,data,ROSLinkStateVariables.ROSLinkImageHeight,ROSLinkStateVariables.ROSLinkImageWidth)
        return ROSLink_image_message.__dict__ 
    
    @staticmethod
    def process_roslink_command_message(msg):
        #print 'msg is ', msg 
        command = json.loads(msg)
        print 'ROSLink command received ..\n\n'
        print msg
        
        if command['header']['message_id'] == MESSAGE_TYPE.ROSLINK_MESSAGE_COMMAND_TAKEOFF:  
            print 'I received Takeoff command' 
            print '\n\nThe robot is Taking off with altitude',command['altitude']  , '\n\n'  
               
            ROSLinkBridgeGapter.setTakeoffMode(command['altitude'])
   
        elif command['header']['message_id'] == MESSAGE_TYPE.ROSLINK_MESSAGE_COMMAND_LAND:
            print 'I received Land command' 
            print '\n\nThe robot is landing\n\n'
            
            ROSLinkBridgeGapter.setLandMode() 
            
        elif command['header']['message_id'] == MESSAGE_TYPE.ROSLINK_MESSAGE_COMMAND_ARM:
            print 'I received Arm command' 
            print 'APM: ARMING MOTORS'
            ROSLinkBridgeGapter.setArm() 
            
        elif command['header']['message_id'] == MESSAGE_TYPE.ROSLINK_MESSAGE_COMMAND_DISARM:
            print 'I received Disarm command' 
            print 'DISARMED: DISARMING MOTORS'
            ROSLinkBridgeGapter.setDisarm() 
            
        elif command['header']['message_id'] == MESSAGE_TYPE.ROSLINK_MESSAGE_COMMAND_SET_MODE:
            print 'I received change mode command' 
            print 'GUIDED> Mode GUIDED'
            ROSLinkBridgeGapter.setFlightMode(command["flightMode"]) 
            
        # elif command['header']['message_id'] == 107:
        #     print 'I received change mode  command' 
        #     print 'STABILIZE> Mode STABILIZE'
        #     ROSLinkBridgeGapter.setStabilizeMode() 
            
        if command['header']['message_id'] == MESSAGE_TYPE.ROSLINK_MESSAGE_COMMAND_TWIST:  
            print 'I received Twist command successfully'
            TwistCommand = TwistStamped() 
            TwistCommand.twist.linear.x = command['vx']
            TwistCommand.twist.linear.y = command['vy'] 
            TwistCommand.twist.linear.z = command['vz'] 
            TwistCommand.twist.angular.x = command['wx']
            TwistCommand.twist.angular.y = command['wy'] 
            TwistCommand.twist.angular.z = command['wz']             
            print TwistCommand
            ROSLinkBridgeGapter.move_publisher.publish (TwistCommand)
  
        # finds what kinds of message in that data  
        '''
        if command['header']['message_id'] == 100:  
            print 'I received Move command successfully'
            TwistCommand = Twist() 
            TwistCommand.linear.x = command['vx']
            TwistCommand.linear.y = command['vy'] 
            TwistCommand.linear.z = command['vz'] 
            TwistCommand.angular.x = command['wx']
            TwistCommand.angular.y = command['wy'] 
            TwistCommand.angular.z = command['wz']             
            #print TwistCommand
            #pubMove.publish (TwistCommand)
              
        if command['header']['message_id'] == 101:  
            print 'I received GO tO Waypoint command successfully'
            print 'But no topic published!!' 
   
        elif command['header']['message_id'] == 103:
            print 'I received Land command' 
            print '\n\nThe robot is landing\n\n'
            #pub_land.publish(Empty())    
        '''

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
		#print ROSLinkBridgeGapter.static_build_heartbeat_message(), '\n' 
                self.send(self.socket, json.dumps(ROSLinkBridgeGapter.static_build_heartbeat_message()))
            elif (self.roslink_message_type == MESSAGE_TYPE.ROSLINK_MESSAGE_ROBOT_STATUS):
		#print ROSLinkBridgeGapter.static_build_robot_status_message(), '\n' 
                self.send(self.socket, json.dumps(ROSLinkBridgeGapter.static_build_robot_status_message()))
            elif (self.roslink_message_type == MESSAGE_TYPE.ROSLINK_MESSAGE_GLOBAL_MOTION):
		#print ROSLinkBridgeGapter.static_build_global_motion_message(), '\n' 
                self.send(self.socket, json.dumps(ROSLinkBridgeGapter.static_build_global_motion_message()))
            elif (self.roslink_message_type == MESSAGE_TYPE.ROSLINK_MESSAGE_GPS_RAW_INFO):
		#print ROSLinkBridgeGapter.static_build_gps_raw_info_message(), '\n' 
                self.send(self.socket, json.dumps(ROSLinkBridgeGapter.static_build_gps_raw_info_message()))
            elif (self.roslink_message_type == MESSAGE_TYPE.ROSLINK_MESSAGE_ROSLINK_IMAGE):
        #print ROSLinkBridgeGapter.static_build_gps_raw_info_message(), '\n' 
                self.send(self.socket, json.dumps(ROSLinkBridgeGapter.static_build_Roslink_Image_message()))

    '''
        Sending method
    '''
    def send (self, sock , msg): 
        self.socket.sendto(msg, self.server_address)   




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
                ROSLinkBridgeGapter.process_roslink_command_message(msg)
            except socket.timeout:
                continue   


if __name__ == '__main__':
    print '\n************** Starting ROSLink Bridge for Gapter **************\n' 
    # initialize ROS node for this client
    #rospy.init_node('udp_client_drone_node', anonymous=True) 
    myDroneBridge = ROSLinkBridgeGapter() 
