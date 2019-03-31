#!/usr/bin/env python  
import roslib
import time
import rospy
import math
import tf
import geometry_msgs.msg
import turtlesim.srv
from turtlesim.msg import Pose



    
velocities=list()
poses=list()
x=0.0
y=0.0
yaw=0.0

fvel = open("turtlesim_vel",'w')
fpose = open("turtlesim_pose",'w')
fx = open("ros_x.txt",'w')
fy= open("ros_y.txt",'w')
fdelay= open("ros_delay.txt",'w')


def poseCallback(msg):
    x= msg.x
    y= msg.y    
    yaw = msg.theta
    poses.append((x,y,yaw))
    if (msg.linear_velocity != 0.0000):
            value = (x,y,yaw)
            #fpose.write(str(value)+"\n")
            #fpose.flush()
            fx.write(str(x)+"\n")
            fx.flush()
            fy.write(str(y)+"\n")
            fy.flush()

if __name__ == '__main__':
    rospy.init_node('turtlesim2_node')

    listener = tf.TransformListener()

    rospy.wait_for_service('spawn')
    spawner = rospy.ServiceProxy('spawn', turtlesim.srv.Spawn)
    spawner(5.544444561, 5.544444561, 0, 'turtle2')
    #spawner(1.5, 3.2, 0, 'turtle2')

    velocity_publisher = rospy.Publisher('turtle2/cmd_vel', geometry_msgs.msg.Twist,queue_size=1)
    pose_sub = rospy.Subscriber("/turtle2/pose", Pose, poseCallback)
    
    #SPIRAL TRAJECTORY 
    constant_speed=4
    vk = 1
    wk = 2
    
    rk =1.0
    rk_step = 0.1
    duration = 10.0 #10 seconds
    rate = 20.0
    number_of_iteration = duration*rate
    loop = rospy.Rate(rate)

    i=0
    t0=time.time()
    while(i<number_of_iteration):
        t1=time.time()
        #print t1-t0
        fdelay.write(str(t1-t0)+"\n")
        fdelay.flush()
        t0=t1
        vel_msg = geometry_msgs.msg.Twist()
        i=i+1
        rk=rk+rk_step
        vel_msg.linear.x =rk
        vel_msg.linear.y =0
        vel_msg.linear.z =0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z =constant_speed
        
        velocities.append((vel_msg.linear.x, vel_msg.angular.z))
        value = (vel_msg.linear.x, vel_msg.angular.z, x,y,yaw)
        #fvel.write(str(value)+"\n")
        #fvel.flush()

        #publish the velocity
        velocity_publisher.publish(vel_msg)
        loop.sleep()

    #stop the robot
    vel_msg.linear.x =0
    velocity_publisher.publish(vel_msg)
    
    #print 'velocities: ', velocities
   # print 'size: ', len(velocities)
    #print 'poses: ', velocities
    #print 'size: ', len(poses)
    
    fvel.close()
    fpose.close()
    
