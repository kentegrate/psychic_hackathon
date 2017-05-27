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
from common import *
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
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
    ##FIXME img is a opencv color img

    # This is for debig
    img = cv2.imread("carreteras.jpg")
    # 1. Change the image to gray schale.
    grayImage = grayscale(img)

    plt.imshow(grayImage, cmap='gray')

    ### Canny Edge detection
    low_threshold = 50
    high_threshold = 150
    edges = canny(grayImage, low_threshold, high_threshold)

    ### 2-1 Region select
    ##Copy from Q.15 Next we'll create a masked edges image using cv2.fillPoly()
    mask = np.zeros_like(edges)
    ignore_mask_color = 255

    ##Copy from Q.15 This time we are defining a four sided polygon to mask
    imshape = grayImage.shape
    vertices = np.array([[(120,imshape[0]),(450, 320), (500, 320), (imshape[1]-100,imshape[0])]], dtype=np.int32)
    cv2.fillPoly(mask, vertices, ignore_mask_color)
    masked_edges = cv2.bitwise_and(edges, mask)


    ### 2-2 Hough Trasformation
    ## Define Hough transform parameters
    # Make a blank the same size as our image to draw on
    rho = 2 # distance resolution in pixels of the Hough grid
    theta = np.pi/180 # angular resolution in radians of the Hough grid
    threshold = 10     # minimum number of votes (intersections in Hough grid cell)
    min_line_len = 45 #minimum number of pixels making up a line
    max_line_gap = 10    # maximum gap in pixels between connectable line segments

    ## Hough Transormation
    line_img = hough_lines(masked_edges, rho, theta, threshold, min_line_len, max_line_gap)
    ### Create the color
    color_edges = np.dstack((edges, edges, edges))

    # Draw the lines on the edge image
    lines_edges = cv2.addWeighted(color_edges, 0.8, line_img, 1, 0)
    weighted_image = weighted_img(line_img, img, a=0.8, b=1., c=0.)

    #FIXME : The output of the image processing is weighted image, which detect the lines inside the image.
    #Take weighted_img for detection if you need.



def main():
    #rospy.init_node('linetrace')
    ic = ImageConverter()
    img = cv2.imread("carreteras.jpg")
    createControl(img)
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
