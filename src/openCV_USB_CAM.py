#!/usr/bin/env python

import sys
import time
import cv2
import base64
import rospy
from std_msgs.msg import String
#from sensor_msgs.msg import String
rospy.init_node('USB-CAM-Image-node', anonymous=True)
pub = rospy.Publisher('USB-CAM-Image', String, queue_size=65535)
cap = cv2.VideoCapture(0)
# cap.set(1 ,cv2.VideoWriter_fourcc('H','2','6','4'));
while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()
    # cv2.imshow('frmae',frame)
    # frame = cv2.resize(frame, (480, 360), interpolation = cv2.INTER_LINEAR)
    frame = cv2.resize(frame, (160, 120), interpolation = cv2.INTER_LINEAR)

    # encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 85]
    # encoded = cv2.imencode('.png',frame,encode_param)[1]
    encoded = cv2.imencode('.png',frame)[1]

    data = u''+base64.encodestring(encoded)
    print(len(data))
    
    data = data[:len(data)-1]
    pub.publish(data)
    # img = cv2.imdecode(encoded,cv2.IMREAD_COLOR)
    # cv2.imshow('frame',img)
    rospy.sleep(1)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()