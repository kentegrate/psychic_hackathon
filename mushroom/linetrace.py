#!/usr/bin/env python
import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
import numpy as np

current_image = None
kp = 0.1

def angle_between(p1, p2):
    ang1 = np.arctan2(*p1[::-1])
    ang2 = np.arctan2(*p2[::-1])
    return np.rad2deg((ang1 - ang2) % (2 * np.pi))

class ImageConverter:

    def __init__(self):

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/ardrone/bottom/image_raw",Image,self.callback)

    def callback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            current_image = cv_image
        except CvBridgeError as e:
	    print(e)

def createControl(img):
    if img is None:
        return msg
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    # define range of red color in HSV
    lower_red1 = np.array([300,50,50])
    upper_red1 = np.array([360,255,255])
    lower_red2 = np.array([0,50,50])
    upper_red2 = np.array([60,255,255])
    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask = cv2.bitwise_or(mask1, mask2)
    img = cv2.bitwise_and(img, img, mask=mask)
    
    edges = cv2.Canny(mask, 50, 150, apertureSize=3)
    lines = cv2.HoughLinesP(image=edges, rho=0.02, theta=np.pi/500, threshold=10, maxLineGap=100, minLineLength=200)#,lines=np.array([]), minLineLength=minLineLength,maxLineGap=100)

    print(len(lines))
    degrees = []
    for line in lines:
        for x1,y1,x2,y2 in line:
            cv2.line(img,(x1,y1),(x2,y2),(0,255,0),2)
            degree = angle_between((x1,y1),(x2,y2))
            print(degree)


    print(degree)
    cv2.imshow('hsv',img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    msg = Twist()
    msg.linear_velocity.x = 0.5
    msg.angular_velocity.z = degree * kp
    return msg


def main():
    rospy.init_node('linetrace')
    ic = ImageConverter()
    rate = rospy.Rate(10)
    ctrl_pub = rospy.Publisher('/ardrone/cmd_vel', Twist, queue_size=10)
    current_image = cv2.imread('image2.jpg')
    
    while not rospy.is_shutdown():
        control_msg = createControl(current_image)
        ctrl_pub.publish(control_msg)
        rate.sleep()

    
    
    

    


            
if __name__ == '__main__':
    main()
