#!/usr/bin/env python

import socket
import threading
import sys
import time
import json
import rospy
import tf
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
#import csv

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
from messages.GlobalMotion import GlobalMotion

# import configuration parameters
from configuration.CONFIG import *
from configuration.ROSLINK_MESSAGE_PERIOD import *

# import state variables 
from states.ROSLinkStateVariables import ROSLinkStateVariables



#cvs_file_writer = csv.writer(f, dialect='excel')

class ROSLinkBridgeTurtlesim:
        
    '''''''''''''''''''''''''''''''''''''''''''''
    Represents ROSLink Bridge for Turtlesim
    '''''''''''''''''''''''''''''''''''''''''''''
    def __init__(self):
        '''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
        Constructor: read and initialize robot and roslink bridge parameters
        '''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
        # initialize ROS node for this client
        rospy.init_node('roslink_bridge_turtlesim_node', anonymous=True) 
        
        # init the parameters from launch file
        ROSLinkBridgeTurtlesim.init_params()
        
        #start ROS publishers
        ROSLinkBridgeTurtlesim.start_ros_publishers()
        
        #start ROS publishers
        ROSLinkBridgeTurtlesim.start_ros_subscribers()
        
        #start UDP server socket
        ROSLinkBridgeTurtlesim.init_servers() 
        
        #create threads for sending roslink messages
        ROSLinkBridgeTurtlesim.create_roslink_message_threads()
        
        #create threads for processing received roslink command messages
        ROSLinkBridgeTurtlesim.create_roslink_command_processing_thread()
    
        #for statistics
        ROSLinkBridgeTurtlesim.stat_velocities = list()
        ROSLinkBridgeTurtlesim.stat_poses=list()
        ROSLinkBridgeTurtlesim.stat_file_vel = open("roslink_turtlesim_vel",'w')
        ROSLinkBridgeTurtlesim.stat_file_pose = open("roslink_turtlesim_pose",'w')
        ROSLinkBridgeTurtlesim.stat_file_x_roslink = open("x_roslink.txt",'w')
        ROSLinkBridgeTurtlesim.stat_file_y_roslink = open("y_roslink.txt",'w')
        ROSLinkBridgeTurtlesim.stat_file_delay = open("delay_roslink.txt",'w')
        
    
        #sping ROS forever
        rospy.spin()
    
    @staticmethod
    def init_servers():
        ROSLinkBridgeTurtlesim.client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) #create server socket
        ROSLinkBridgeTurtlesim.client_socket.settimeout(2) #set a timeout on a blocking socket operation
        ROSLinkBridgeTurtlesim.gcs_server_ip = rospy.get_param("/ground_station_ip", "192.168.100.17")
        ROSLinkBridgeTurtlesim.gcs_server_port =rospy.get_param("/ground_station_port", 25500)
        ROSLinkBridgeTurtlesim.server_address  = ( ROSLinkBridgeTurtlesim.gcs_server_ip, ROSLinkBridgeTurtlesim.gcs_server_port)
        print ROSLinkBridgeTurtlesim.gcs_server_ip
        print ROSLinkBridgeTurtlesim.gcs_server_port 
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
        ROSLinkStateVariables.robot_name = rospy.get_param("/robot_name", "Turtlsim")
        # get robot type
        ROSLinkStateVariables.type = rospy.get_param("/type", ROBOT_TYPE.ROBOT_TYPE_GENERIC)
        # get owner id
        ROSLinkStateVariables.owner_id = rospy.get_param("/owner_id", 3)
        # get key
        ROSLinkStateVariables.key = rospy.get_param("/key", "1243-0000-0000-FGFG")
        
        # define periods of updates
        ROSLinkBridgeTurtlesim.heartbeat_msg_rate = rospy.get_param("/heartbeat_msg_period", ROSLINK_MESSAGE_PERIOD.ROSLINK_HEARTBEAT_MESSAGE_RATE)
        #ROSLinkBridgeTurtlesim.robot_status_msg_rate = rospy.get_param("/robot_status_msg_period", ROSLINK_MESSAGE_PERIOD.ROSLINK_ROBOT_STATUS_MESSAGE_RATE)
        ROSLinkBridgeTurtlesim.global_motion_msg_rate = rospy.get_param("/global_motion_msg_period", ROSLINK_MESSAGE_PERIOD.ROSLINK_GLOBAL_MOTION_MESSAGE_RATE)
        #ROSLinkBridgeTurtlesim.gps_raw_info_msg_rate = rospy.get_param("/gps_raw_info_msg_period", ROSLINK_MESSAGE_PERIOD.ROSLINK_GPS_RAW_INFO_MESSAGE_RATE)
        #ROSLinkBridgeTurtlesim.range_finder_data_msg_rate = rospy.get_param("/range_finder_data_msg_period", ROSLINK_MESSAGE_PERIOD.ROSLINK_RANGE_FINDER_DATA_MESSAGE_RATE)

    #start ROS publishers 
    @staticmethod
    def start_ros_publishers():
        # ROS publishers: for executing ROSLink commands
        ROSLinkBridgeTurtlesim.move_publisher    = rospy.Publisher('/turtle1/cmd_vel',Twist, queue_size=10)    
     
    #start ROS subscribers
    @staticmethod
    def start_ros_subscribers():
        rospy.Subscriber("/turtle1/pose", Pose, ROSLinkBridgeTurtlesim.poseCallback)
    
    @staticmethod   
    def poseCallback(msg):
        #position 
        ROSLinkStateVariables.time_boot_ms=time.time()
        ROSLinkStateVariables.x= msg.x
        ROSLinkStateVariables.y= msg.y
        ROSLinkStateVariables.yaw = msg.theta
        
        if (msg.linear_velocity != 0.0000):
            value = (ROSLinkStateVariables.x,ROSLinkStateVariables.y,ROSLinkStateVariables.yaw)
            #ROSLinkBridgeTurtlesim.stat_file_pose.write(str(value)+"\n")
            #ROSLinkBridgeTurtlesim.stat_file_pose.flush()
            ROSLinkBridgeTurtlesim.stat_file_x_roslink.write(str(ROSLinkStateVariables.x)+"\n")
            ROSLinkBridgeTurtlesim.stat_file_x_roslink.flush()
            ROSLinkBridgeTurtlesim.stat_file_y_roslink.write(str(ROSLinkStateVariables.y)+"\n")
            ROSLinkBridgeTurtlesim.stat_file_y_roslink.flush()
        #heading = msg.theta

        #twist: linear
        ROSLinkStateVariables.vx = msg.linear_velocity
        #twist: angular
        ROSLinkStateVariables.wx = msg.angular_velocity
        #print  ROSLinkStateVariables.x
     
    
    #this method create threads for sending roslink message at specific rate
    @staticmethod
    def create_roslink_message_threads(): 
        ROSLinkBridgeTurtlesim.heatrbeat_message_thread = ROSLinkMessageThread(ROSLinkBridgeTurtlesim.client_socket, ROSLinkBridgeTurtlesim.server_address, MESSAGE_TYPE.ROSLINK_MESSAGE_HEARTBEAT, "heatrbeat_message_thread", ROSLinkBridgeTurtlesim.heartbeat_msg_rate)
        ROSLinkBridgeTurtlesim.global_motion_thread = ROSLinkMessageThread(ROSLinkBridgeTurtlesim.client_socket, ROSLinkBridgeTurtlesim.server_address, MESSAGE_TYPE.ROSLINK_MESSAGE_GLOBAL_MOTION, "global_motion_thread", ROSLinkBridgeTurtlesim.global_motion_msg_rate)   
    
    @staticmethod
    def create_roslink_command_processing_thread(): 
        ROSLinkBridgeTurtlesim.command_processing_thread =  ROSLinkCommandProcessingThread(ROSLinkBridgeTurtlesim.client_socket, 'ROSLink Command Processing Thread')
    
    @staticmethod
    def static_build_roslink_header_message(message_id):
        header = ROSLinkHeader(ROSLinkStateVariables.roslink_version, ROSLinkStateVariables.ros_version, ROSLinkStateVariables.system_id, message_id, ROSLinkStateVariables.sequence_number,ROSLinkStateVariables.key)
        ROSLinkStateVariables.sequence_number = ROSLinkStateVariables.sequence_number + 1
        return header.__dict__
    
    @staticmethod 
    def static_build_heartbeat_message():
        message_header = ROSLinkBridgeTurtlesim.static_build_roslink_header_message(MESSAGE_TYPE.ROSLINK_MESSAGE_HEARTBEAT)
        heartbeat_message = HeartBeat(message_header, ROBOT_TYPE.ROBOT_TYPE_GENERIC, ROSLinkStateVariables.robot_name, ROBOT_STATE.ROBOT_STATE_ACTIVE, ROSLinkStateVariables.owner_id ,ROBOT_MODE.ROBOT_STATE_UNKNOWN)
        return heartbeat_message.__dict__
    
    @staticmethod
    def static_build_global_motion_message():
        message_header = ROSLinkBridgeTurtlesim.static_build_roslink_header_message(MESSAGE_TYPE.ROSLINK_MESSAGE_GLOBAL_MOTION) 
        global_motion_message = GlobalMotion(message_header, ROSLinkStateVariables.time_boot_ms , ROSLinkStateVariables.x, ROSLinkStateVariables.y, 0, ROSLinkStateVariables.vx, 0, 0, ROSLinkStateVariables.wx, 0, 0, 0, 0, ROSLinkStateVariables.yaw)
        return global_motion_message.__dict__  
    
    
    @staticmethod
    def process_roslink_command_message(msg):
        print 'msg is ', msg 
        command = json.loads(msg)
        print 'ROSLink command received ..'
        #print msg
            
        if command['header']['message_id'] == MESSAGE_TYPE.ROSLINK_MESSAGE_COMMAND_TWIST:  
            print 'I received Twist command successfully'
            TwistCommand = Twist() 
            TwistCommand.linear.x = command['vx']
            TwistCommand.linear.y = command['vy'] 
            TwistCommand.linear.z = command['vz'] 
            TwistCommand.angular.x = command['wx']
            TwistCommand.angular.y = command['wy'] 
            TwistCommand.angular.z = command['wz']             
            #print TwistCommand
            ROSLinkBridgeTurtlesim.stat_velocities.append((TwistCommand.linear.x,TwistCommand.angular.z))
            ROSLinkBridgeTurtlesim.stat_poses.append((ROSLinkStateVariables.x,ROSLinkStateVariables.y,ROSLinkStateVariables.yaw))
            #print 'velocities: ', ROSLinkBridgeTurtlesim.stat_velocities
            #print 'size: ', len(ROSLinkBridgeTurtlesim.stat_velocities)
            #print 'poses: ', ROSLinkBridgeTurtlesim.stat_poses
            value = (TwistCommand.linear.x, TwistCommand.angular.z, ROSLinkStateVariables.x,ROSLinkStateVariables.y,ROSLinkStateVariables.yaw)
            ROSLinkBridgeTurtlesim.stat_file_vel.write(str(TwistCommand.linear.x)+"\n")
            ROSLinkBridgeTurtlesim.stat_file_vel.flush()
            #f.write("test")
            #s=('velocities: ', velocities)
            #f.write(str(s))
            #s=('size: ', len(velocities))
            #f.write(str(s))
            #f.flush()

            
            #f.write(str(s))
            
            ROSLinkBridgeTurtlesim.move_publisher.publish (TwistCommand)
  

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
        self.t0=time.time()
        while True:
            self.count=self.count+1
            time.sleep(1.0/self.data_rate)
            #print 'thread %s %d\n'%(self.name, self.count)
            #self.send(self.socket, json.dumps(self.roslink_message.__dict__))
            if (self.roslink_message_type == MESSAGE_TYPE.ROSLINK_MESSAGE_HEARTBEAT):
                self.send(self.socket, json.dumps(ROSLinkBridgeTurtlesim.static_build_heartbeat_message()))
            elif (self.roslink_message_type == MESSAGE_TYPE.ROSLINK_MESSAGE_GLOBAL_MOTION):
                self.send(self.socket, json.dumps(ROSLinkBridgeTurtlesim.static_build_global_motion_message()))

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
        self.t0=time.time()
        while True:
            try:
                msg, address = self.socket.recvfrom(MESSAGE_MAX_LENGTH)
                self.t1=time.time()
                #print self.t1-self.t0
                print msg	
                ROSLinkBridgeTurtlesim.stat_file_delay.write(str(self.t1-self.t0)+"\n")
                ROSLinkBridgeTurtlesim.stat_file_delay.flush()
                self.t0=self.t1
                ROSLinkBridgeTurtlesim.process_roslink_command_message(msg)
                
            except socket.timeout:
                continue   


if __name__ == '__main__':
    print '\n************** Starting ROSLink Bridge for Turtlesim **************\n' 
    # initialize ROS node for this client
    myTurtlesimBridge = ROSLinkBridgeTurtlesim() 
    #rospy.spin
