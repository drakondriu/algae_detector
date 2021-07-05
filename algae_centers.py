# -*- coding: utf-8 -*-
import sys
import rospy
import math
import numpy as np
#import matplotlib.pyplot as plt
import json
from sklearn.cluster import KMeans
from sklearn import metrics
from scipy.spatial.distance import cdist
from sklearn.metrics import silhouette_samples, silhouette_score

from helper.utils import *
from helper.ambiente import Pontos
from classic import rrt as alg
from coverageFolder.grid_based_sweep_coverage_path_planner import *

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range
from geometry_msgs.msg import PoseArray, Pose, Twist
from tf.transformations import euler_from_quaternion

from std_msgs.msg import String, Int32
from sensor_msgs.msg import BatteryState

from datetime import datetime
import statistics as stc
from sys import exit

from utilsUAV import *


           
def launch():
    #with open("algae.json") as jsonFile:
    with open("algae.json") as jsonFile:
        jsonObject = json.load(jsonFile)
        jsonFile.close()
    
    sample_list = list(jsonObject.keys())
    if len(sample_list) > 0 :
        l = []
        for i in sample_list:
            l.append( tuple(map(float, i[1:-1].split(', '))))
            distortions = []
            sse_list = []
            
        for i in range(1, 16):
            print(i)
            kmeans = KMeans(n_clusters=i).fit(l)
            sse_list.append(kmeans.inertia_)
            distortions.append(sum(np.min(cdist(l, kmeans.cluster_centers_, 'euclidean'), axis=1)) / len(l))

        print(sse_list)    
        plt.plot(range(1,16), distortions, 'bx-')
        plt.xlabel('k clusters')
        plt.ylabel('Distortion')
        plt.title('The Elbow Method showing the optimal k')
        plt.show()
        
    else : 
        print('No algae')
def callbackHeight(sonar):
    rospy.loginfo(sonar.range)
    

    
def main():
    rospy.init_node("mission_goals")
    launch()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    # plt.show()

if __name__ == "__main__":
    print("Start")
    main()
