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
from sensor_msgs.msg import Image, CameraInfo, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import cv2
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from ardrone_autonomy.msg import  Navdata
import numpy as np
import base64

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
from messages.RoslinkImage import RoslinkImage

# import configuration parameters
from configuration.CONFIG import *
from configuration.ROSLINK_MESSAGE_PERIOD import *

# import state variables 
from states.ROSLinkStateVariables import ROSLinkStateVariables


class ROSLinkBridgeARDrone:
        
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
        ROSLinkBridgeARDrone.init_params()
        
        if (ROSLinkStateVariables.key == "NOKEY"):
            print("ROBOT KEY HAS NOT BEEN SET. PROGRAM WILL EXIT. SET YOUR ROBOT KEY BEFORE STARTING THE ROSLINK BRIDGE")
            sys.exit()
        if (ROSLinkStateVariables.owner_id == -1):
            print("OWNER ID NOT BEEN SET. PROGRAM WILL EXIT. SET YOUR OWNER ID BEFORE STARTING THE ROSLINK BRIDGE")
            sys.exit()
        
        #start ROS publishers
        ROSLinkBridgeARDrone.start_ros_publishers()
        
        #start ROS publishers
        ROSLinkBridgeARDrone.start_ros_subscribers()
        
        #start UDP server socket
        ROSLinkBridgeARDrone.init_servers() 
        
        #create threads for sending roslink messages
        ROSLinkBridgeARDrone.create_roslink_message_threads()
        
        #create threads for processing received roslink command messages
        ROSLinkBridgeARDrone.create_roslink_command_processing_thread()
    
        #sping ROS forever
        rospy.spin()
    
    @staticmethod
    def init_servers():
        ROSLinkBridgeARDrone.client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) #create server socket
        ROSLinkBridgeARDrone.client_socket.settimeout(2) #set a timeout on a blocking socket operation
        ROSLinkBridgeARDrone.gcs_server_ip = rospy.get_param("/ground_station_ip", "208.113.133.197")
        ROSLinkBridgeARDrone.gcs_server_port =rospy.get_param("/ground_station_port", 25500)
        ROSLinkBridgeARDrone.server_address  = ( ROSLinkBridgeARDrone.gcs_server_ip, ROSLinkBridgeARDrone.gcs_server_port)
        print ROSLinkBridgeARDrone.gcs_server_ip
        print ROSLinkBridgeARDrone.gcs_server_port 
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
        ROSLinkStateVariables.robot_name = rospy.get_param("/robot_name", "ARDRONE")
        # get robot type
        ROSLinkStateVariables.type = rospy.get_param("/type", ROBOT_TYPE.ROBOT_TYPE_PARROT)
        # get owner id
        ROSLinkStateVariables.owner_id = rospy.get_param("/owner_id", -1)
        # get key
        ROSLinkStateVariables.key = rospy.get_param("/key", "NOKEY")
        
        # define periods of updates
        ROSLinkBridgeARDrone.heartbeat_msg_rate = rospy.get_param("/heartbeat_msg_rate", ROSLINK_MESSAGE_PERIOD.ROSLINK_HEARTBEAT_MESSAGE_RATE)
        ROSLinkBridgeARDrone.robot_status_msg_rate = rospy.get_param("/robot_status_msg_rate", ROSLINK_MESSAGE_PERIOD.ROSLINK_ROBOT_STATUS_MESSAGE_RATE)
        ROSLinkBridgeARDrone.global_motion_msg_rate = rospy.get_param("/global_motion_msg_rate", ROSLINK_MESSAGE_PERIOD.ROSLINK_GLOBAL_MOTION_MESSAGE_RATE)
        ROSLinkBridgeARDrone.gps_raw_info_msg_rate = rospy.get_param("/gps_raw_info_msg_rate", ROSLINK_MESSAGE_PERIOD.ROSLINK_GPS_RAW_INFO_MESSAGE_RATE)
        ROSLinkBridgeARDrone.range_finder_data_msg_rate = rospy.get_param("/range_finder_data_msg_rate", ROSLINK_MESSAGE_PERIOD.ROSLINK_RANGE_FINDER_DATA_MESSAGE_RATE)
        ROSLinkBridgeARDrone.ROSLing_Image_msg_rate = rospy.get_param("/ROSLing_Image_msg_rate", 1)
        
        ROSLinkBridgeARDrone.bridge = CvBridge()
        
    #start ROS publishers 
    @staticmethod
    def start_ros_publishers():
        # ROS publishers: for executing ROSLink commands
        ROSLinkBridgeARDrone.takeoff_publisher = rospy.Publisher('/ardrone/takeoff', Empty, queue_size=10)
        ROSLinkBridgeARDrone.land_publisher    = rospy.Publisher('/ardrone/land', Empty, queue_size=10)
        ROSLinkBridgeARDrone.reset_publisher   = rospy.Publisher('/ardrone/reset',Empty, queue_size=10)
        ROSLinkBridgeARDrone.move_publisher    = rospy.Publisher('/cmd_vel',Twist, queue_size=10)    
     
    #start ROS subscribers
    @staticmethod
    def start_ros_subscribers():
        rospy.Subscriber("/ground_truth/state", Odometry, ROSLinkBridgeARDrone.odometryCallback)
        rospy.Subscriber("/ardrone/navdata", Navdata, ROSLinkBridgeARDrone.navdataCallback)
        rospy.Subscriber("/ardrone/front/image_raw/compressed", CompressedImage, ROSLinkBridgeARDrone.frontCompressedImageCallback)
        # rospy.Subscriber("/ardrone/front/image_raw", Image, ROSLinkBridgeARDrone.frontImageCallback)
    
    
    
    @staticmethod
    def frontImageCallback(data):
        try:
          cv_image = ROSLinkBridgeARDrone.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
          print(e)
        cv_image = cv2.resize(cv_image,(ROSLinkStateVariables.ROSLinkImageWidth,ROSLinkStateVariables.ROSLinkImageHeight))
        encoded = cv2.imencode('.png',cv_image)[1]
        ROSLinkStateVariables.ROSLinkImageData = u''+base64.encodestring(encoded)
        print(len(ROSLinkStateVariables.ROSLinkImageData))
        # cv2.imshow("ARDrone Front Image Viewer", cv_image)
        # cv2.waitKey(3)
    
    @staticmethod   
    def frontCompressedImageCallback(data):
        try:
          #cv_image = ROSLinkBridgeARDrone.bridge.imgmsg_to_cv2(data, "bgr8")
          np_arr = np.fromstring(data.data, np.uint8)
          cv_image = cv2.imdecode(np_arr, cv2.CV_16U)
        except CvBridgeError as e:
          print(e)

        ROSLinkStateVariables.ROSLinkImageData = u''+base64.encodestring(np_arr)#[:8000];
        print(len(ROSLinkStateVariables.ROSLinkImageData))
        # print("bytes")
        # cv2.imshow("ARDrone Front Compressed Image Viewer", cv_image)
        # cv2.waitKey(3)
    
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
        ROSLinkBridgeARDrone.wx_truth = msg.twist.twist.angular.x
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

    '''''''''''''''''''''''''''''''''''''''''''''
    Create ROSLink Threads
    '''''''''''''''''''''''''''''''''''''''''''''
    #this method create threads for sending roslink message at specific rate
    @staticmethod
    def create_roslink_message_threads(): 
        ROSLinkBridgeARDrone.heatrbeat_message_thread = ROSLinkMessageThread(ROSLinkBridgeARDrone.client_socket, ROSLinkBridgeARDrone.server_address, MESSAGE_TYPE.ROSLINK_MESSAGE_HEARTBEAT, "heatrbeat_message_thread", ROSLinkBridgeARDrone.heartbeat_msg_rate)
        ROSLinkBridgeARDrone.robot_status_thread = ROSLinkMessageThread(ROSLinkBridgeARDrone.client_socket, ROSLinkBridgeARDrone.server_address, MESSAGE_TYPE.ROSLINK_MESSAGE_ROBOT_STATUS, "robot_status_thread", ROSLinkBridgeARDrone.robot_status_msg_rate)
        ROSLinkBridgeARDrone.global_motion_thread = ROSLinkMessageThread(ROSLinkBridgeARDrone.client_socket, ROSLinkBridgeARDrone.server_address, MESSAGE_TYPE.ROSLINK_MESSAGE_GLOBAL_MOTION, "global_motion_thread", ROSLinkBridgeARDrone.global_motion_msg_rate)
        ROSLinkBridgeARDrone.gps_raw_info_thread = ROSLinkMessageThread(ROSLinkBridgeARDrone.client_socket,  ROSLinkBridgeARDrone.server_address, MESSAGE_TYPE.ROSLINK_MESSAGE_GPS_RAW_INFO, "gps_raw_info_thread", ROSLinkBridgeARDrone.gps_raw_info_msg_rate)
        ROSLinkBridgeARDrone.ROSLinkImage_message_thread = ROSLinkMessageThread(ROSLinkBridgeARDrone.client_socket,  ROSLinkBridgeARDrone.server_address, MESSAGE_TYPE.ROSLINK_MESSAGE_ROSLINK_IMAGE, "ROSLink_image_thread", ROSLinkBridgeARDrone.ROSLing_Image_msg_rate)
        #ROSLinkBridgeARDrone.range_finder_data_thread = ROSLinkMessageThread(ROSLinkBridgeARDrone.client_socket, ROSLinkBridgeARDrone.server_address, "range_finder_data_thread", 0.333)
        
    @staticmethod
    def create_roslink_command_processing_thread(): 
        ROSLinkBridgeARDrone.command_processing_thread =  ROSLinkCommandProcessingThread(ROSLinkBridgeARDrone.client_socket, 'ROSLink Command Processing Thread')


    '''''''''''''''''''''''''''''''''''''''''''''
    Building ROSLink Status Messages
    '''''''''''''''''''''''''''''''''''''''''''''

    @staticmethod
    def static_build_roslink_header_message(message_id):
        header = ROSLinkHeader(ROSLinkStateVariables.roslink_version, ROSLinkStateVariables.ros_version, ROSLinkStateVariables.system_id,ROSLinkStateVariables.owner_id , message_id, ROSLinkStateVariables.sequence_number,ROSLinkStateVariables.key)
        ROSLinkStateVariables.sequence_number = ROSLinkStateVariables.sequence_number + 1
        return header.__dict__
    
    @staticmethod 
    def static_build_heartbeat_message():
        message_header = ROSLinkBridgeARDrone.static_build_roslink_header_message(MESSAGE_TYPE.ROSLINK_MESSAGE_HEARTBEAT)
        heartbeat_message = HeartBeat(message_header, ROBOT_TYPE.ROBOT_TYPE_PARROT, ROSLinkStateVariables.robot_name, ROBOT_STATE.ROBOT_STATE_ACTIVE, ROSLinkStateVariables.owner_id ,ROBOT_MODE.ROBOT_STATE_UNKNOWN)
        return heartbeat_message.__dict__
    
    @staticmethod 
    def static_build_robot_status_message():
        message_header = ROSLinkBridgeARDrone.static_build_roslink_header_message(MESSAGE_TYPE.ROSLINK_MESSAGE_ROBOT_STATUS)
        robot_status_message = RobotStatus(message_header,ROSLinkStateVariables.state, [0], [0], 0, 0 ,ROSLinkStateVariables.battery)
        return robot_status_message.__dict__
    
    @staticmethod
    def static_build_global_motion_message():
        message_header = ROSLinkBridgeARDrone.static_build_roslink_header_message(MESSAGE_TYPE.ROSLINK_MESSAGE_GLOBAL_MOTION) 
        global_motion_message = GlobalMotion(message_header, ROSLinkStateVariables.time_boot_ms , ROSLinkStateVariables.x, ROSLinkStateVariables.y,ROSLinkStateVariables.altitude,  ROSLinkStateVariables.vx, ROSLinkStateVariables.vy, ROSLinkStateVariables.vz, ROSLinkStateVariables.wx, ROSLinkStateVariables.wy, ROSLinkStateVariables.wz, ROSLinkStateVariables.pitch, ROSLinkStateVariables.roll, ROSLinkStateVariables.yaw)
        return global_motion_message.__dict__  
    
    @staticmethod
    def static_build_gps_raw_info_message():
        message_header = ROSLinkBridgeARDrone.static_build_roslink_header_message(MESSAGE_TYPE.ROSLINK_MESSAGE_GPS_RAW_INFO) 
        global_motion_message = GPSRawInfo(message_header, ROSLinkStateVariables.time_boot_ms , ROSLinkStateVariables.fix_type, ROSLinkStateVariables.lat, ROSLinkStateVariables.lon, ROSLinkStateVariables.alt, ROSLinkStateVariables.eph, ROSLinkStateVariables.epv, ROSLinkStateVariables.vel, ROSLinkStateVariables.cog, ROSLinkStateVariables.satellites_visible)
        return global_motion_message.__dict__  

    @staticmethod
    def static_build_ROSLink_image_message():
        message_header = ROSLinkBridgeARDrone.static_build_roslink_header_message(MESSAGE_TYPE.ROSLINK_MESSAGE_ROSLINK_IMAGE) 
        ROSLink_image_message = RoslinkImage(message_header, ROSLinkStateVariables.owner_id ,ROSLinkStateVariables.key,ROSLinkStateVariables.ROSLinkImageData,ROSLinkStateVariables.ROSLinkImageHeight,ROSLinkStateVariables.ROSLinkImageWidth)
        return ROSLink_image_message.__dict__  

    '''''''''''''''''''''''''''''''''''''''''''''
    Building ROSLink Command Messages
    '''''''''''''''''''''''''''''''''''''''''''''
    
    @staticmethod
    def process_roslink_command_message(msg):
        #print 'msg is ', msg 
        command = json.loads(msg)
        print 'ROSLink command received ...'
        print msg
        
        if command['header']['message_id'] == MESSAGE_TYPE.ROSLINK_MESSAGE_COMMAND_TAKEOFF:  
            print '[TAKEOFF]'
            print '\n\n -> The drone is taking-off ...\n\n'
            ROSLinkBridgeARDrone.takeoff_publisher.publish(Empty())
            print '[TAKEOFF COMPLETED]'
   
        elif command['header']['message_id'] == MESSAGE_TYPE.ROSLINK_MESSAGE_COMMAND_LAND:
            print '[LAND]'
            print '\n\nThe drone is landing\n\n'
            ROSLinkBridgeARDrone.land_publisher.publish(Empty()) 
            
        if command['header']['message_id'] == MESSAGE_TYPE.ROSLINK_MESSAGE_COMMAND_TWIST:  
            print '[Twist]'
            TwistCommand = Twist() 
            TwistCommand.linear.x = command['vx']
            TwistCommand.linear.y = command['vy'] 
            TwistCommand.linear.z = command['vz'] 
            TwistCommand.angular.x = command['wx']
            TwistCommand.angular.y = command['wy'] 
            TwistCommand.angular.z = command['wz']             
            print TwistCommand
            ROSLinkBridgeARDrone.move_publisher.publish (TwistCommand)


'''''''''''''''''''''''''''''''''''''''''''''
Building ROSLink Message Thread Classes
'''''''''''''''''''''''''''''''''''''''''''''

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
            #print 'ROSLink Message sent ... \n'
            #self.send(self.socket, json.dumps(self.roslink_message.__dict__))
            if (self.roslink_message_type == MESSAGE_TYPE.ROSLINK_MESSAGE_HEARTBEAT):
                self.send(self.socket, json.dumps(ROSLinkBridgeARDrone.static_build_heartbeat_message()))
                print '[HEARTBEAT:%d] \n'%(self.count)
            elif (self.roslink_message_type == MESSAGE_TYPE.ROSLINK_MESSAGE_ROBOT_STATUS):
                self.send(self.socket, json.dumps(ROSLinkBridgeARDrone.static_build_robot_status_message()))
                print '[ROBOT_STATUS:%d] \n' % (self.count)
            elif (self.roslink_message_type == MESSAGE_TYPE.ROSLINK_MESSAGE_GLOBAL_MOTION):
                self.send(self.socket, json.dumps(ROSLinkBridgeARDrone.static_build_global_motion_message()))
                print '[GLOBAL_MOTION:%d] \n' % (self.count)
            elif (self.roslink_message_type == MESSAGE_TYPE.ROSLINK_MESSAGE_GPS_RAW_INFO):
                self.send(self.socket, json.dumps(ROSLinkBridgeARDrone.static_build_gps_raw_info_message()))
                print '[GPS_RAW_INFO:%d] \n' % (self.count)
            elif (self.roslink_message_type == MESSAGE_TYPE.ROSLINK_MESSAGE_ROSLINK_IMAGE):
                self.send(self.socket, json.dumps(ROSLinkBridgeARDrone.static_build_ROSLink_image_message()))
                print ('ROSLink_image sent')


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
                ROSLinkBridgeARDrone.process_roslink_command_message(msg)
            except socket.timeout:
                continue   

'''''''''''''''''''''''''''''''''''''''''''''
The Main Function
'''''''''''''''''''''''''''''''''''''''''''''

if __name__ == '__main__':
    print '\n************** Starting ROSLink Bridge for ARDrone **************\n' 
    # initialize ROS node for this client
    #rospy.init_node('udp_client_drone_node', anonymous=True) 
    myDroneBridge = ROSLinkBridgeARDrone() 








