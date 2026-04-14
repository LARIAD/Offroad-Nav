#!/usr/bin/env python3
import warnings
warnings.filterwarnings("ignore", category=RuntimeWarning) 

import os
import cv2
import numpy as np
from sklearn.linear_model import RANSACRegressor, LinearRegression


import rospy
import message_filters
from sensor_msgs.msg import Image, PointCloud, PointCloud2, CameraInfo
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge

from depth_estimation.utils import DepthPreprocessing, ExponentialMovingAverage, Evaluation, edge_detection


class RescaledDepthEstimation(DepthPreprocessing):
    
    def __init__(self) -> None:
        # super().__init__()
        
        self.bridge = CvBridge()

        ref_name = rospy.get_param('~ref_name')
        ref_topic = rospy.get_param('~ref_topic')
        # Extract the camera name
        cam_name = ref_topic.split('/')[1]
        cam_number = cam_name[3:]
        
        #ROS Publisher init
        self.pub_scale_offset = rospy.Publisher(f'depth_estimation/{cam_name}/scale_offset', Float32MultiArray, queue_size=10)
        self.pub_scaled_depth = rospy.Publisher(f'/depth_estimation/{cam_name}/disp_map_rescaled_v2', Image, queue_size=10)
        self.pub_camera_info = rospy.Publisher(f'/depth_estimation/{cam_name}/camera_info', CameraInfo, queue_size=10)
        self.pub_keypoints = rospy.Publisher(f'/depth_estimation/{cam_name}/keypoints', Image, queue_size=10)
        self.pub_scaled_depth_color = rospy.Publisher(f'/depth_estimation/{cam_name}/disp_map_rescaled_color', Image, queue_size=10)
        
        #ROS Subscriber init
        disp_sub = message_filters.Subscriber(f'depth_estimation/{cam_name}/disp_map', Image)
        im_sub = message_filters.Subscriber(rospy.get_param('~image_topic'), Image)
        # normalized_disp_topic = rospy.get_param('normalized_disp_topic')
        # normalized_disp_sub = message_filters.Subscriber(normalized_disp_topic, Image)

        camera_info_topic = rospy.get_param('~cam_info_topic')
        camera_info_sub = message_filters.Subscriber(camera_info_topic, CameraInfo)

        ref_type = rospy.get_param('~ref_name')
        ref_topic = rospy.get_param('~ref_topic')
        
        if ref_name in ['VINS-Mono']:
            depth_ref_sub = message_filters.Subscriber(ref_topic, PointCloud2)
            self.rescaling_preprocessing = self.preprocessing_pc
        elif ref_name == 'stereo':
            depth_ref_sub = message_filters.Subscriber(ref_topic, Image)
            self.rescaling_preprocessing = self.preprocessing_stereo
        elif ref_name == 'pseudo_vins_mono':
            depth_ref_sub = message_filters.Subscriber(ref_topic, Image)
            self.rescaling_preprocessing = self.preprocessing_pseudo_vins_mono
        else:
            raise NotImplementedError
        
        self.mask_edge = rospy.get_param('~mask_edge')
        if self.mask_edge:
            self.pub_edges = rospy.Publisher(f'/depth_estimation/{cam_name}/edges', Image, queue_size=10)
        self.edge_thickness = rospy.get_param('~edge_thickness')
        self.edge_detection_method = rospy.get_param('~edge_detection_method')
        
        self.eval = Evaluation(ref_type)
            
        ts = message_filters.TimeSynchronizer([disp_sub, depth_ref_sub, camera_info_sub, im_sub], 10) #10
        ts.registerCallback(self.predict_scaled_depth)
        
        #Scale factor and offset init
        smoothing_factor = rospy.get_param('~smoothing_factor')
        assert 0 <= smoothing_factor < 1., "0 <= smoothing_factor < 1"
        self.scale_factor = ExponentialMovingAverage(smoothing_factor)
        self.offset = ExponentialMovingAverage(smoothing_factor)
        self.use_offset = rospy.get_param('~use_offset')
        
        #Max depth of the refmax_depth
        self.max_depth = rospy.get_param('~max_depth_rescaling')
            
    def update_coefs(self, disp_match_valid, disp_ref_valid):
        #print("update coef")
        if disp_match_valid.shape[0] >= 50:
            ransac = RANSACRegressor(LinearRegression(fit_intercept=self.use_offset)).fit(disp_match_valid.reshape(-1, 1), disp_ref_valid.reshape(-1, 1))
        
            self.scale_factor.update(ransac.estimator_.coef_[0, 0])
            self.offset.update(ransac.estimator_.intercept_[0] if self.use_offset else 0.)
            #rospy.loginfo(f'coefs updated: {self.scale_factor.get(), self.offset.get()}')
            self.send_coefs()
        
        else:
            rospy.logwarn('Not enough key points')
            os.system("pkill -2 -f 'rosbag record'")
            
    def send_coefs(self):
        self.scale_offset = Float32MultiArray(data=np.array([self.scale_factor.get(), self.offset.get()], dtype=np.float32))
        self.pub_scale_offset.publish(self.scale_offset)
        
    def rescale(self, disp):
        disp_scaled = self.scale_factor.get() * disp + self.offset.get()
        return disp_scaled
        
            
    def predict_scaled_depth(self, disp_msg, ref_msg, camera_info_msg, im_msg):

        pred_disp = self.bridge.imgmsg_to_cv2(disp_msg, desired_encoding='passthrough')
        
        if self.mask_edge:
            edges = edge_detection(pred_disp, thickness=self.edge_thickness, method=self.edge_detection_method)
        else:
            edges = None
        
        pred_disp_valid, ref_disp_valid, ref_depth_valid, keypoints_img = self.rescaling_preprocessing(pred_disp, ref_msg, edges, im_msg)
        self.update_coefs(pred_disp_valid, ref_disp_valid)

        # processing_time = (rospy.Time.now() - start_time_rescale).to_sec()
        # rospy.loginfo(f"Rescaling completed in {processing_time:.2f}s")
        
        disp_valid_rescaled = self.rescale(pred_disp_valid)

        # depth_valid_rescaled = 1 / disp_valid_rescaled
        # self.eval.update_eval(depth_valid_rescaled, ref_depth_valid)

        disp_rescaled = np.clip(self.rescale(pred_disp), disp_valid_rescaled.min(), disp_valid_rescaled.max())

        if self.mask_edge:
            depth_rescaled = 1 / (disp_rescaled * (edges < 1))
        else:
            depth_rescaled = 1 / disp_rescaled

        # depth_rescaled_msg = self.bridge.cv2_to_imgmsg(depth_rescaled.astype(np.float32), encoding='passthrough')
        # depth_rescaled_msg.header = camera_info_msg.header
        # self.pub_scaled_depth.publish(depth_rescaled_msg)
        # self.pub_camera_info.publish(camera_info_msg)

        # Keep original float32 depth
        depth_rescaled_msg = self.bridge.cv2_to_imgmsg(depth_rescaled.astype(np.float32), encoding='passthrough')
        depth_rescaled_msg.header = camera_info_msg.header
        self.pub_scaled_depth.publish(depth_rescaled_msg)

        # Add colorized version on a separate topic
        #self.pub_scaled_depth_color = rospy.Publisher(f'/depth_estimation/{cam_name}/disp_map_rescaled_color', Image, queue_size=10)

        # Mask out inf and nan values
        depth_for_color = depth_rescaled.copy()
        valid_mask = np.isfinite(depth_for_color)

        if valid_mask.any():
            min_val = np.percentile(depth_for_color[valid_mask], 2)
            max_val = np.percentile(depth_for_color[valid_mask], 98)
            depth_for_color = np.clip(depth_for_color, min_val, max_val)
            depth_normalized = ((depth_for_color - min_val) / (max_val - min_val) * 255).astype(np.uint8)
        else:
            depth_normalized = np.zeros_like(depth_for_color, dtype=np.uint8)

        # Set invalid pixels to 0
        depth_normalized[~valid_mask] = 0

        depth_colormap = cv2.applyColorMap(depth_normalized, cv2.COLORMAP_JET)
        depth_color_msg = self.bridge.cv2_to_imgmsg(depth_colormap, encoding='bgr8')
        depth_color_msg.header = camera_info_msg.header
        self.pub_scaled_depth_color.publish(depth_color_msg)
        
        if self.mask_edge:
            edges_msg = self.bridge.cv2_to_imgmsg(edges, encoding='passthrough')
            self.pub_edges.publish(edges_msg)
        
        if keypoints_img is not None:
            keypoints_msg = self.bridge.cv2_to_imgmsg(keypoints_img, encoding='bgr8')
            keypoints_msg.header = camera_info_msg.header
            self.pub_keypoints.publish(keypoints_msg)

        
if __name__ == '__main__':
    try:
        # Initiate the node
        rospy.init_node('depth_estimation', anonymous=True)

        rescaled_depth_estimator = RescaledDepthEstimation()
        
        rospy.loginfo("Depth rescaling Model: Start")
        
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass