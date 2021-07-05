# -*- coding: utf-8 -*-
import sys
import rospy
import math
import numpy as np
import matplotlib.pyplot as plt
import message_filters
import json

from helper.utils import *
from helper.ambiente import Pontos
from classic import rrt as alg
from coverageFolder.grid_based_sweep_coverage_path_planner import *

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range, Image, CameraInfo
from geometry_msgs.msg import PoseArray, Pose, Twist
from tf.transformations import euler_from_quaternion

from std_msgs.msg import String, Int32
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import BatteryState

from datetime import datetime
import statistics as stc
from sys import exit

from utilsUAV import *

import numpy as np
import cv2
import os
from cv_bridge import CvBridge


########################### Functions
def stationary_camera_transform(point, alturaZ ):
    K = [215.6810060961547, 0.0, 376.5, 0.0, 215.6810060961547, 240.5, 0.0, 0.0, 1.0]
    
    baseTerrestreAltura = 0.000009 # 0.5

    Z = alturaZ - baseTerrestreAltura # Distancia do solo 
    
    dX = (point[0] - K[2]) * Z / K[0]
    dY = (point[1] - K[5]) * Z / K[4]

    dist = (dX, dY)
    return dist

def algae_detector(img):
    #print(img)
    img = img[:,:,[0,1,2]]
    img_ch0 = img[:,:,0]

#thr
    min_thr = 20
    _, thr = cv2.threshold(img_ch0, min_thr, 255, cv2.THRESH_BINARY)
 
#blur
    size_b = 5
    kernel_blur = np.ones((size_b, size_b),np.float32)/size_b*size_b
    dst = cv2.filter2D(thr, -1, kernel_blur)

#closing
    size_c = 10
    kernel_closing = np.ones((size_c, size_c),np.float32)/size_c*size_c
    img_binary = cv2.morphologyEx(dst, cv2.MORPH_CLOSE, kernel_closing)

    mostTrue = thr.shape[0]*thr.shape[1]*255*0.98 # 98% of the image's pixels have the same color (255)
    mostFalse = thr.shape[0]*thr.shape[1]*255*0.02    #  2% of the image's pixels have the same color (255)
    img_pixels = thr.shape[0]*thr.shape[1]

    if (thr.sum() >= mostTrue) or (thr.sum() <= mostFalse):
        #print("Is the world blue?") 
    
        B_sum = img[:,:,2].sum()
        G_sum = img[:,:,1].sum()
        R_sum = img[:,:,0].sum()

        red_limit = 60
        blue_limit = 150
        if (R_sum < red_limit*thr.shape[0]*thr.shape[1]) and (B_sum > blue_limit*thr.shape[0]*thr.shape[1]) :
            #print("yes, it is!")
            img_binary = np.zeros(thr.shape)
    return img_binary
    
########################## Node          
def launch():
    image_sub = message_filters.Subscriber('/uav1/bluefox_optflow/image_raw', Image)
    GPS_sub = message_filters.Subscriber('/uav1/mavros/global_position/local', Odometry)
    
    ts = message_filters.ApproximateTimeSynchronizer([image_sub, GPS_sub], 10, 0.05)
    ts.registerCallback(callbackImg_coord)


def callbackImg_coord(image_ros, location):
    pub = rospy.Publisher('classified_image', Image,queue_size=10)
    bridge = CvBridge()
    image = bridge.imgmsg_to_cv2(image_ros, desired_encoding='passthrough')
    classified = algae_detector(image)
    uav_z = location.pose.pose.position.z
    uav_x = location.pose.pose.position.x
    uav_y = location.pose.pose.position.y
    pub.publish(bridge.cv2_to_imgmsg(classified))
    algae_loc = {}
    w = np.where(classified == 255 )
    idy = w[0]
    idx = w[1]
    for i in range(len(idx)):
        p = stationary_camera_transform((idx[i], 480-idy[i]), uav_z)
        algae_coord = (round(uav_x+p[0]+116,1), round(uav_y+p[1]+120, 1))        
        algae_loc[str(algae_coord)] = 1
        #print(p)
        #print([idx[i], idy[i]])
      
    if len(algae_loc)>0:
        
        a_file = open("algae.json", "r")
        algae_full = json.load(a_file)
        a_file.close()
        
        algae_full.update(algae_loc)
        a_file = open("algae.json", "w")
        json.dump(algae_full, a_file)
        a_file.close()
        print('recorded')
    else:
        print('Nothing to record')
    

def main():
    a_file = open("algae.json", "w")
    json.dump({}, a_file)
    a_file.close()
    rospy.init_node("mrs_map")
    launch()
    rospy.sleep(2)
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    # plt.show()

if __name__ == "__main__":
    print("Start Recording Algae Location")
    main()
