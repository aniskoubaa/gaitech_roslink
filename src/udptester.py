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
# from ardrone_autonomy.msg import  Navdata
import numpy as np
import base64

#from G.Gvariables import Gvariables 

Image_message_Type=5;

class gvariables:
    cmdport=1109
    owner_id=1
    key=1
    vx = 0.0
    vy = 0.0
    vz = 0.0
    wx = 0.0
    wy = 0.0
    wz = 0.0
    battery = 0.0
    state = 0.0
    time_boot_ms=0.0
    seq_number =0;
    magX = 0.0
    magY = 0.0
    magZ = 0.0
    pressure = 0.0
    temp = 0.0
    wind_speed = 0.0
    wind_angle = 0.0
    rotX = 0.0
    rotY = 0.0
    rotZ = 0.0
    altitude = 0.0
    motor1 = 0.0
    motor2 = 0.0
    motor3 = 0.0
    motor4 = 0.0
    tags_count = 0.0
    tags_type = 0.0


    ROSLinkImageData="test"
    ROSLinkImageHeight=240
    ROSLinkImageWidth=320
    

    x= 0.0
    y= 0.0
    z= 0.0
    roll =0.0
    pitch = 0.0
    yaw = 0.0
    vx_truth = 0.0
    vy_truth = 0.0
    vz_truth = 0.0
    wx_truth = 0.0
    wy_truth = 0.0
    wz_truth = 0.0
    
    
    
    fix_type = 0
    lat = 0.0
    lon = 0.0
    alt = 0.0
    eph = 0.0
    epv = 0.0
    vel = 0.0
    cog = 0.0
    satellites_visible = 0.0


Gvariables = gvariables


class msgImage(object):
    def __init__ (self, header=None,owner_id=None, robot_key=None, data=None, height=None,  width=None):
        self.header = header
        self.owner_id = owner_id
        self.key = robot_key
        self.data = data
        self.height = height
        self.width = width
        
    def from_json(self, message):
        self.__dict__ = json.loads(message)

        
    def printMessage(self):
        print 'header: ', self.header
        print 'owner_id: ', self.owner_id
        print 'key: ' , self.key
        print 'height: ' , self.height
        print 'width: ' , self.width


class ardronecode:
        
    '''''''''''''''''''''''''''''''''''''''''''''
    Represents  Bridge for AR Drone
    '''''''''''''''''''''''''''''''''''''''''''''
    def __init__(self):
        '''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
        Constructor: read and initialize robot and  bridge parameters
        '''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
        # initialize ROS node for this client
        rospy.init_node('ardronecode_node', anonymous=True) 
        
        # init the parameters from launch file
        ardronecode.init_params()
        
       
        #start ROS publishers
        ardronecode.start_ros_publishers()
        
        #start ROS publishers
        ardronecode.start_ros_subscribers()
        
        #start UDP server socket
        ardronecode.init_servers() 
        
        #create threads for sending roslink messages
        ardronecode.Image_message_thread = MessageThread(ardronecode.client_socket,  ardronecode.server_address, 5, "image_thread", 9)

        
        #create threads for processing received  command messages
        ardronecode.create_command_processing_thread()
    
        #sping ROS forever
        rospy.spin()
    
    @staticmethod
    def init_servers():
        ardronecode.client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) #create server socket
        ardronecode.client_socket.settimeout(2) #set a timeout on a blocking socket operation

        ardronecode.gcs_server_ip = "192.168.1.176"
        ardronecode.gcs_server_port =25500
        ardronecode.server_address  = ( ardronecode.gcs_server_ip, ardronecode.gcs_server_port)
        #print ardronecode.gcs_server_ip
        #print ardronecode.gcs_server_port 
    

    @staticmethod
    def process_command_message(msg):
        #print 'msg is ', msg 
        command = json.loads(msg)
        #print 'command received ...'
        print msg
        
        if command['header'] == "2":#takeoff
            #print '[TAKEOFF]'
            #print '\n\n -> The drone is taking-off ...\n\n'
            ardronecode.takeoff_publisher.publish(Empty())
            print '[TAKEOFF COMPLETED]'
   
        elif command['header'] == "1":
            #print '[LAND]'
            #print '\n\nThe drone is landing\n\n'
            ardronecode.land_publisher.publish(Empty()) 
            
        if command['header'] == "3":  
            #print '[Twist]'
            TwistCommand = Twist() 
            TwistCommand.linear.x = command['lx']
            TwistCommand.linear.y = command['ly'] 
            TwistCommand.linear.z = command['lz'] 
            TwistCommand.angular.x = command['ax']
            TwistCommand.angular.y = command['ay'] 
            TwistCommand.angular.z = command['az']             
            print TwistCommand
            ardronecode.move_publisher.publish (TwistCommand)
   
           
    @staticmethod
    def init_params():   
        
       
        # get robot name
        ardronecode.robot_name = "rifd"
        # get robot type
        ardronecode.type = 'drone'
        # get owner id
        ardronecode.owner_id = "1"
        # get key
        ardronecode.key = "rifd1"
        
        
        ardronecode.bridge = CvBridge()
        
    #start ROS publishers 
    @staticmethod
    def start_ros_publishers():
        # ROS publishers: for executing ROSLink commands
        ardronecode.takeoff_publisher = rospy.Publisher('/ardrone/takeoff', Empty, queue_size=10)
        ardronecode.takeoff_publisher.publish(Empty)
        ardronecode.land_publisher    = rospy.Publisher('/ardrone/land', Empty, queue_size=10)
        ardronecode.reset_publisher   = rospy.Publisher('/ardrone/reset',Empty, queue_size=10)
        ardronecode.move_publisher    = rospy.Publisher('/cmd_vel',Twist, queue_size=10)    
     
    #start ROS subscribers
    @staticmethod
    def start_ros_subscribers():
        rospy.Subscriber("/ground_truth/state", Odometry, ardronecode.odometryCallback)
        # rospy.Subscriber("/ardrone/navdata", Navdata, ardronecode.navdataCallback)
        # rospy.Subscriber("/ardrone/front/image_raw/compressed", CompressedImage, ardronecode.frontCompressedImageCallback)
        # rospy.Subscriber("/ardrone/front/image_raw", Image, ardronecode.frontImageCallback)
    
    
    
    @staticmethod
    def frontImageCallback(data):
        try:
          cv_image = ardronecode.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
          print(e)
        cv_image = cv2.resize(cv_image,(Gvariables.ROSLinkImageWidth,Gvariables.ROSLinkImageHeight))
        encoded = cv2.imencode('.png',cv_image)[1]
        Gvariables.ROSLinkImageData = u''+base64.encodestring(encoded)#[:555]
        print(len(Gvariables.ROSLinkImageData))
        # cv2.imshow("ARDrone Front Image Viewer", cv_image)
        # cv2.waitKey(3)
    
    @staticmethod   
    def frontCompressedImageCallback(data):
        try:
          #cv_image = ardronecode.bridge.imgmsg_to_cv2(data, "bgr8")
          np_arr = np.fromstring(data.data, np.uint8)
          cv_image = cv2.imdecode(np_arr, cv2.CV_16U)
        except CvBridgeError as e:
          print(e)

        Gvariables.ROSLinkImageData = u''+base64.encodestring(np_arr)
        # print(len(Gvariables.ROSLinkImageData))
        # cv2.imshow("ARDrone Front Compressed Image Viewer", cv_image)
        # cv2.waitKey(3)
    
    @staticmethod   
    def odometryCallback(msg):
        #position 
        Gvariables.time_boot_ms=time.time()
        Gvariables.x= msg.pose.pose.position.x
        Gvariables.y= msg.pose.pose.position.y
        Gvariables.z= msg.pose.pose.position.z
        #orientation
        quaternion = (msg.pose.pose.orientation.x,msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        Gvariables.roll = euler[0]
        Gvariables.pitch = euler[1]
        Gvariables.yaw = euler[2]
        #twist: linear
        Gvariables.vx_truth = msg.twist.twist.linear.x
        Gvariables.vy_truth = msg.twist.twist.linear.y
        Gvariables.vz_truth = msg.twist.twist.linear.z
        #twist: angular
        ardronecode.wx_truth = msg.twist.twist.angular.x
        Gvariables.wy_truth = msg.twist.twist.angular.y
        Gvariables.wz_truth = msg.twist.twist.angular.z
        
        #print self.x
        #print self.y
     
    @staticmethod   
    def navdataCallback(msg):
        Gvariables.time_boot_ms=time.time()
        Gvariables.seq_number = Gvariables.seq_number +1;
        Gvariables.vx = msg.vx
        Gvariables.vy = msg.vy
        Gvariables.vz = msg.vz
        Gvariables.wx = msg.ax
        Gvariables.wy = msg.ay
        Gvariables.wz = msg.az
        Gvariables.battery = msg.batteryPercent
        Gvariables.state = msg.state
        Gvariables.magX = msg.magX
        Gvariables.magY = msg.magY
        Gvariables.magZ = msg.magZ
        Gvariables.pressure = msg.pressure
        Gvariables.temp = msg.temp
        Gvariables.wind_speed = msg.wind_speed
        Gvariables.wind_angle = msg.wind_angle
        Gvariables.rotX = msg.rotX
        Gvariables.rotY = msg.rotY
        Gvariables.rotZ = msg.rotZ
        Gvariables.altitude = msg.altd
        Gvariables.motor1 = msg.motor1
        Gvariables.motor2 = msg.motor2
        Gvariables.motor3 = msg.motor3
        Gvariables.motor4 = msg.motor4
        Gvariables.tags_count = msg.tags_count
        Gvariables.tags_type = msg.tags_type

    '''''''''''''''''''''''''''''''''''''''''''''
    Create  Threads
    '''''''''''''''''''''''''''''''''''''''''''''
   
    @staticmethod
    def create_command_processing_thread(): 
        ardronecode.command_processing_thread =  CommandProcessingThread(ardronecode.client_socket, ' Command Processing Thread')


 
    @staticmethod
    def static_build_image_message():
        ROSLink_image_message = msgImage("5",Gvariables.owner_id ,Gvariables.key,Gvariables.ROSLinkImageData,Gvariables.ROSLinkImageHeight,Gvariables.ROSLinkImageWidth)
        return ROSLink_image_message.__dict__  

   
'''''''''''''''''''''''''''''''''''''''''''''
Building Message Thread Classes
'''''''''''''''''''''''''''''''''''''''''''''

class MessageThread ():
    def __init__(self, sock, server_address,message_type ,thread_name='noname', data_rate=1.0):
        self.name = thread_name
        self.socket = sock
        self.server_address = server_address
        self.data_rate = data_rate
        self.message_type = message_type
        t = threading.Thread(target=self.run)
        t.setName(thread_name)
        t.start()
    
    def run ( self ):
        while True:
            time.sleep(1.0/self.data_rate)
            #print 'ROSLink Message sent ... \n'
            #self.send(self.socket, json.dumps(self.roslink_message.__dict__))
            if (self.message_type == 5):
                self.send(self.socket, json.dumps(ardronecode.static_build_image_message()))
                #print ('image sent')


    '''
        Sending method
    '''
    def send (self, sock , msg): 
        self.socket.sendto(msg, self.server_address)   


class CommandProcessingThread():
    def __init__(self, sock,thread_name='noname'):
        self.name = thread_name
        self.socket = sock
        self.socket.bind(('', 1109))
        print('createing thread')
        t = threading.Thread(target=self.run)
        t.setName(thread_name)
        t.start()
    def run( self):
        print "Start Command Processing Thread"
        while True:
            try:
                msg, address = self.socket.recvfrom(65000)
                print"asdasdfasdf"
                ardronecode.process_command_message(msg)
            except socket.timeout:
                print('timeout')
                continue   




'''''''''''''''''''''''''''''''''''''''''''''
The Main Function
'''''''''''''''''''''''''''''''''''''''''''''

if __name__ == '__main__':
    print '\n************** Starting ROSLink Bridge for ARDrone **************\n' 
    # initialize ROS node for this client
    #rospy.init_node('udp_client_drone_node', anonymous=True) 
    myDroneBridge = ardronecode() 
