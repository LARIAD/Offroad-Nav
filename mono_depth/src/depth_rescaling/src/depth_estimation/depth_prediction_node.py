#!/usr/bin/env python3
import warnings
warnings.filterwarnings("ignore", category=RuntimeWarning) 

import os
import cv2
import numpy as np
from sklearn.linear_model import RANSACRegressor, LinearRegression

from PIL import Image


import rospy
import message_filters
import ros_numpy
#from numpy_ros import to_numpy
from rospy.numpy_msg import numpy_msg
from ros_numpy.point_cloud2 import fields_to_dtype
from sensor_msgs.msg import Image, PointCloud, PointCloud2, CameraInfo
from std_msgs.msg import Float32, Float32MultiArray
from cv_bridge import CvBridge

from depth_estimation.utils import *


class DepthEstimation(object):
    def __init__(self) -> None:
        
        self.bridge = CvBridge()
        
        self.cv_image = None
        self.pil_image = None
        self.header = None    
        self.scale_factor = None
        self.offset = None   
        
        #Depth model init
        self.model = load_model()

        self.ref_topic = rospy.get_param('~image_topic')
        # Extract the camera name
        cam_name = self.ref_topic.split('/')[1]
        cam_number = cam_name[3:]
                
        #ROS Publishers init
        self.pub_disp = rospy.Publisher(f'depth_estimation/{cam_name}/disp_map', Image, queue_size=10)
        self.pub_depth_map = rospy.Publisher(f'depth_estimation/{cam_name}/disp_map_rescaled', Image, queue_size=10)
        self.pub_pcl = DepthToPointCloud(cam_name, cam_number)

        self.pub_img_mask = rospy.Publisher(f'depth_estimation/{cam_name}/mask_img', Image, queue_size=10)
        
        #ROS Subscriber init
        self.im_sub = rospy.Subscriber(rospy.get_param('~image_topic'), Image, self.listen_im_msg)
        self.coef_sub = rospy.Subscriber(f'depth_estimation/{cam_name}/scale_offset', Float32MultiArray, self.listen_coefs_msg)
        
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            self.compute_disp()
            rate.sleep()
        
    def listen_im_msg(self, im_msg):
        # im_msg to im
        self.cv_image = self.bridge.imgmsg_to_cv2(im_msg, desired_encoding='passthrough')
        #print("shape raw img = ", self.cv_image.shape)
        #self.cv_image = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2RGB).astype(np.float32)

        if self.cv_image.dtype == np.float32:
            cv_image_uint8 = self.cv_image.astype(np.uint8)
        else:
            cv_image_uint8 = self.cv_image

        # Convert to PIL (for Hugging Face model)
        #self.pil_image = Image.fromarray(cv_image_uint8)
        
        self.header = im_msg.header
        
    def listen_coefs_msg(self, coefs_msg):
        data = np.asarray(coefs_msg.data, dtype=np.float32)
        self.scale_factor = data[0]
        self.offset = data[1]      
    
    def rescale(self, disp):
        return np.clip(self.scale_factor * disp + self.offset, 0, None)
    
    def compute_disp(self):
        header = self.header
        # predict disp map
        
        if self.cv_image is not None:
            start_time_disp = rospy.Time.now()

            #cv_img_mask = mask_edges(self.cv_image)
            #print("shape mask img = ", self.cv_image.shape)
            disp_pred = self.model(self.cv_image)
            #print("image raw shape = ", self.cv_image.shape)
            #print("dist pred shape = ", disp_pred.shape)
            # publish disparity
            disp_msg = self.bridge.cv2_to_imgmsg(disp_pred, encoding="passthrough")
            disp_msg.header = header
            self.pub_disp.publish(disp_msg)

            # img_mask = self.bridge.cv2_to_imgmsg(cv_img_mask, encoding="passthrough")
            # img_mask.header = header
            # self.pub_img_mask.publish(img_mask)

            #self.pub_depth_map.publish(disp_msg)
            #self.scale_factor=None

            # processing_time = (rospy.Time.now() - start_time_disp).to_sec()
            # rospy.loginfo(f"Disparity completed in {processing_time:.2f}s")

            if self.scale_factor is not None:
                depth_rescaled = 1 / self.rescale(disp_pred)
                #print("shape depth map rescaled  = ", depth_rescaled.shape)

                # depth_rescaled = cv2.ximgproc.guidedFilter(
                #     guide=self.cv_image, 
                #     src=depth_rescaled.astype(np.float32), 
                #     radius=30, 
                #     eps=1e-3
                # )

                depth_rescaled_msg = self.bridge.cv2_to_imgmsg(depth_rescaled, encoding='passthrough')
                depth_rescaled_msg.header = disp_msg.header
                self.pub_depth_map.publish(depth_rescaled_msg)
                
                processing_time = (rospy.Time.now() - start_time_disp).to_sec()
            
                #rospy.loginfo(f"Depth estimation processing in {processing_time:.2f}s")

        # publish visualization
        # disp_visu = (disp_pred - disp_pred.min()) / (disp_pred.max() - disp_pred.min()) * 255.0
        # disp_visu = cv2.applyColorMap(disp_visu.astype(np.uint8), cv2.COLORMAP_INFERNO)
        # disp_visu_msg = self.bridge.cv2_to_imgmsg(disp_visu, encoding="bgr8")
        # self.pub_disp_visu.publish(disp_visu_msg)
        
        # return disp_pred        
        
if __name__ == '__main__':
    try:
        # Initiate the node
        rospy.init_node('depth_estimation', anonymous=True)

        depth_estimation = DepthEstimation()
        
        rospy.loginfo("Depth Model: Start")
        
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass
    