#!/usr/bin/env python
import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
from std_msgs.msg import Empty
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
import numpy as np

kp = 0.05
topic_name = "/ardrone/bottom/image_raw"



def reject_outliers(data):
    m = 2
    u = np.mean(data)
    s = np.std(data)
    filtered = [e for e in data if (u - 2 * s < e < u + 2 * s)]
    return filtered

def calculate_mean(data):
    filtered_d = reject_outliers(data)
    return np.mean(filtered_d)

def unit_vector(vector):
    return vector / np.linalg.norm(vector)
def angle_between(v1, v2):
    v1_u = unit_vector(v1)
    v2_u = unit_vector(v2)
    return np.rad2deg(np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0)))

class ImageConverter:

    def __init__(self):

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(topic_name,Image,self.callback)
        self.image_pub = rospy.Publisher("debug", Image, queue_size=10)
        self.current_image = None
    def callback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.current_image = cv_image.copy()
            print("image came")
        except CvBridgeError as e:
	    print(e)
    def get_image(self):
        if self.current_image is None:
            return None
        return self.current_image.copy()
    def publish_image(self, cv_img):
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_img, "bgr8"))
    
def createControl(img):
    msg = Twist()


    if img is None:
        print "none"
        return msg,img
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    # define range of red color in HSV
#    lower_red1 = np.array([300,50,50])
#    upper_red1 = np.array([360,150,150])
    lower_red2 = np.array([0,30,30])
    upper_red2 = np.array([30,255,255])
#    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
#    mask = cv2.bitwise_or(mask1, mask2)
    img = cv2.bitwise_and(img, img, mask=mask2)
    
    edges = cv2.Canny(mask2, 50, 150, apertureSize=3)
    lines = cv2.HoughLinesP(image=edges, rho=0.02, theta=np.pi/500, threshold=10, maxLineGap=100, minLineLength=300)#,lines=np.array([]), minLineLength=minLineLength,maxLineGap=100)
    print(lines)
    if lines is None or len(lines) == 0:
        return msg,img
    print(len(lines))
    degrees = []
    for line in lines:
        for x1,y1,x2,y2 in line:
            cv2.line(img,(x1,y1),(x2,y2),(0,255,0),2)
            degree = angle_between((x1-x2,y1-y2),(0,1))
            degrees.append(degree)
            print(degree)

    print("mean")
    degree = calculate_mean(degrees)
    print(degree)
    if(abs(degree) > 45):
        return msg,img
    
    msg.linear.x = 1.0
    msg.angular.z = degree * kp
    return msg,img


def main():
    rospy.init_node('linetrace')
    ic = ImageConverter()
    rate = rospy.Rate(10)
    ctrl_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    takeoff_pub = rospy.Publisher('/ardrone/takeoff', Empty, queue_size=10)
    land_pub = rospy.Publisher('/ardrone/land', Empty, queue_size=10)
    empty_msg = Empty()
    count = 0
    rospy.sleep(1)
    takeoff_pub.publish(empty_msg)
    rospy.sleep(3)    
    while not rospy.is_shutdown() and count < 10*100:
        
        control_msg,img = createControl(ic.get_image())
        ic.publish_image(img)
        print(control_msg)
        ctrl_pub.publish(control_msg)
        rate.sleep()
        count+=1
    land_pub.publish(empty_msg)
    


            
if __name__ == '__main__':
    main()
