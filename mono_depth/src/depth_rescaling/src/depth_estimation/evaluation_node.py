#!/usr/bin/env python3
import warnings
warnings.filterwarnings("ignore", category=RuntimeWarning) 

import os
import cv2
import numpy as np
from sklearn.linear_model import RANSACRegressor, LinearRegression


import rospy
import message_filters
import ros_numpy
from ros_numpy.point_cloud2 import fields_to_dtype
from sensor_msgs.msg import Image, PointCloud, PointCloud2, CameraInfo
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge

from depth_estimation.depth_prediction_node import DepthEstimation
from depth_estimation.utils import DepthPreprocessing, ExponentialMovingAverage, Evaluation


class EvalDepthEstimation(DepthPreprocessing):
    
    def __init__(self) -> None:
        # super().__init__()
        
        self.bridge = CvBridge()
        
        #ROS Publisher init
        
        #ROS Subscriber init
        depth_pred_sub = message_filters.Subscriber('depth_estimation/depth_map', Image)
        eval_name = rospy.get_param('eval_name')
        eval_topic = rospy.get_param('eval_topic')
        if eval_name.lower() in ['vins-mono']:
            depth_eval_sub = message_filters.Subscriber(eval_topic, PointCloud2)
            self.rescaling_preprocessing = self.preprocess_pc
        elif eval_name.lower() == 'stereo':
            depth_eval_sub = message_filters.Subscriber(eval_topic, Image)
            self.rescaling_preprocessing = self.preprocess_stereo
        else:
            raise NotImplementedError
            
        ts = message_filters.TimeSynchronizer([depth_pred_sub, depth_eval_sub], 100)
        ts.registerCallback(self.compute_eval)
        
        self.max_depth = rospy.get_param('max_depth_eval')
        
        #Evaluation init
        self.eval = Evaluation(eval_name)        
            
    def compute_eval(self, depth_pred_msg, depth_gt_msg):
        
        depth_pred = self.bridge.imgmsg_to_cv2(depth_pred_msg, desired_encoding='passthrough')
                
        pred_depth_valid, _, gt_depth_valid = self.rescaling_preprocessing(depth_pred, depth_gt_msg)
        
        self.eval.update_eval(pred_depth_valid, gt_depth_valid)
        
        
if __name__ == '__main__':
    try:
        # Initiate the node
        rospy.init_node('depth_estimation', anonymous=True)

        rescaled_depth_estimator = EvalDepthEstimation()
        
        rospy.loginfo("Depth Model: Start")
        
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass