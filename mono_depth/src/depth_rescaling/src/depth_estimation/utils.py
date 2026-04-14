#!/usr/bin/env python3
import numpy as np
import cv2
import rospy
# import ros_numpy

import depth_estimation.depth_models as depth_models

def load_model():
    return getattr(depth_models, rospy.get_param('~model'))()

class ExponentialMovingAverage:
    def __init__(self, alpha):
        self.alpha = alpha  # Smoothing factor
        self.ema = None  # Initialize EMA to None

    def update(self, data_point):
        if self.ema is None:
            self.ema = data_point  # First data point is the initial EMA
        else:
            self.ema = (1 - self.alpha) * data_point + self.alpha * self.ema  # Update EMA

    def get(self):
        return self.ema
    

def edge_detection(img, thickness=1, method='canny'):
    """
    Compute an edge mask from a grayscale or color image (or a single-channel float image).
    Returns a float32 mask in range [0, 1].

    method: 'canny' or 'sobel'
    thickness: int >= 1, larger values produce thicker edges.
    """
    # Normalize/convert to 8-bit grayscale
    if img.dtype == np.uint8:
        img8 = img.copy()
    else:
        # handle float images in [0,1] or larger ranges, and other dtypes
        if np.issubdtype(img.dtype, np.floating):
            if img.max() <= 1.0:
                img8 = (np.clip(img, 0.0, 1.0) * 255.0).astype(np.uint8)
            else:
                img8 = np.clip(img, 0.0, 255.0).astype(np.uint8)
        else:
            img8 = cv2.convertScaleAbs(img)

    if img8.ndim == 3 and img8.shape[2] == 3:
        gray = cv2.cvtColor(img8, cv2.COLOR_BGR2GRAY)
    elif img8.ndim == 3 and img8.shape[2] == 4:
        gray = cv2.cvtColor(img8, cv2.COLOR_BGRA2GRAY)
    else:
        gray = img8 if img8.ndim == 2 else img8[:, :, 0]

    method = method.lower()
    if method == 'canny':
        # Smooth to reduce noise, then use automatic Canny thresholds based on median
        med = np.median(gray)
        lower = int(max(int(gray.min()), 0.66 * med))
        upper = int(min(int(gray.max()), 1.33 * med))
        edges = cv2.Canny(gray, lower, upper)
    elif method == 'sobel':
        # Smooth first, compute gradient magnitude, normalize and binarize
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        gx = cv2.Sobel(blurred, cv2.CV_64F, 1, 0, ksize=3)
        gy = cv2.Sobel(blurred, cv2.CV_64F, 0, 1, ksize=3)
        mag = np.hypot(gx, gy)
        maxv = mag.max()
        if maxv > 0:
            mag = (mag / maxv) * 255.0
        else:
            mag = np.zeros_like(mag)
        mag_u8 = mag.astype(np.uint8)
        # Automatic threshold using median; ensures a reasonably crisp binary edge map
        # thresh = int(max(np.median(mag_u8), 10))
        # _, edges = cv2.threshold(mag_u8, thresh, 255, cv2.THRESH_BINARY)
        # Let Otsu find an optimal threshold automatically
        _, edges = cv2.threshold(mag_u8, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    else:
        raise ValueError("Unsupported method '{}'. Use 'canny' or 'sobel'.".format(method))

    # Optionally thicken edges using dilation
    thickness = int(max(1, thickness))
    if thickness > 1:
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (thickness, thickness))
        edges = cv2.dilate(edges, kernel, iterations=1)

    # Return normalized float mask
    return edges.astype(np.float32) / 255.0

    
class DepthPreprocessing(object):
    
    def get_valid_mask(self, depth, edges=None):
        if edges is not None:
            valid = np.logical_and(
                np.logical_and(0 < depth, depth < self.max_depth),
                np.logical_and(~np.isnan(depth), edges == 0)
            )
        valid = np.logical_and(np.logical_and(0 < depth, depth < self.max_depth), ~np.isnan(depth))
        return valid
        
    def preprocessing_pseudo_vins_mono(self, pred, stereo_msg, edges=None, im_msg=None):
        stereo_depth = self.bridge.imgmsg_to_cv2(stereo_msg, desired_encoding='passthrough')
        img = self.bridge.imgmsg_to_cv2(im_msg, desired_encoding='passthrough')

        img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        corners = cv2.goodFeaturesToTrack(
            img_gray, 
            maxCorners=150, 
            qualityLevel=0.01, 
            minDistance=30
        ).reshape(-1, 2).astype(int)

        rows, cols = corners[:, 1], corners[:, 0]
        pc_depth = stereo_depth[rows, cols]
        pred_at_corners = pred[rows, cols]
        edges_at_corners = edges[rows, cols] if edges is not None else None

        pc_depth = stereo_depth[corners[:, 1], corners[:, 0]]

        valid = self.get_valid_mask(pc_depth, edges_at_corners)
        pred_valid = pred_at_corners[valid]
        stereo_depth_valid = pc_depth[valid]
        stereo_disp_valid = 1 / stereo_depth_valid

        # Visualize keypoints on disparity map
        disp_norm = cv2.normalize(pred, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
        keypoints_img = cv2.cvtColor(disp_norm, cv2.COLOR_GRAY2BGR)

        # Draw all detected corners in red
        for c in corners:
            cv2.circle(keypoints_img, (c[0], c[1]), 3, (0, 0, 255), -1)

        # Draw valid corners (after masking) in green on top
        valid_corners = corners[valid]
        for c in valid_corners:
            cv2.circle(keypoints_img, (c[0], c[1]), 3, (0, 255, 0), -1)

        return pred_valid, stereo_disp_valid, stereo_depth_valid, keypoints_img
        
    def preprocessing_stereo(self, pred, stereo_msg, edges=None, im_msg=None):
        stereo_depth = self.bridge.imgmsg_to_cv2(stereo_msg, desired_encoding='passthrough')
        valid = self.get_valid_mask(stereo_depth, edges)
        pred_valid = pred[valid]
        stereo_depth_valid = stereo_depth[valid]
        stereo_disp_valid = 1 / stereo_depth_valid
        return pred_valid, stereo_disp_valid, stereo_depth_valid, None
        
    def preprocessing_pc(self, pred, pc_msg, edges=None, im_msg=None):
        pc = ros_numpy.point_cloud2.pointcloud2_to_array(pc_msg)
        pc = np.array(pc.tolist()).astype(np.float32)
        pc_depth = pc[:, 2]
        pred_match = cv2.remap(pred, pc[:, 3], pc[:, 4], cv2.INTER_LINEAR)[:, 0]
        valid = self.get_valid_mask(pc_depth, edges)
        pred_valid = pred_match[valid]
        pc_depth_valid = pc_depth[valid]
        pc_disp_valid = 1 / pc_depth_valid
        return pred_valid, pc_disp_valid, pc_depth_valid, None
    
    def preprocess_pc_lidar(self, pred, pc_msg): # not tested
        pc = ros_numpy.point_cloud2.pointcloud2_to_array(pc_msg)
        pc = np.array(pc.tolist()).astype(np.float32)
        pc_depth = pc[:, 2]
        pc_depth_valid, pc_disp_valid, valid = self.get_valid_depth_disp(pc_depth)
        pred_valid = pred[valid]
        return pred_valid, pc_disp_valid, pc_depth_valid, None
    
    
class Evaluation:
    def __init__(self, name) -> None:
        self.name = name
        self.counter = 0
        self.metric_dict = {
            'abs': None,
            'rel': None,
            'err5': None,
            'err2': None,
            'err1': None,
        }
    
    def update_metric(self, key, new):
        if self.metric_dict[key] is not None:
            update = self.metric_dict[key] * self.counter / (self.counter + 1) + new / (self.counter + 1)
        else:
            update = new
        self.metric_dict[key] = np.round(update, 2)
        
    def update_eval(self, depth_rescaled, ref_depth):
        err_abs = np.abs(depth_rescaled - ref_depth)
        err_rel = err_abs / ref_depth
        err5 = np.mean(err_abs < 0.5) * 100
        err2 = np.mean(err_abs < 0.2) * 100
        err1 = np.mean(err_abs < 0.1) * 100
        
        self.update_metric('abs', np.mean(err_abs))
        self.update_metric('rel', np.mean(err_rel))
        self.update_metric('err5', err5)
        self.update_metric('err2', err2)
        self.update_metric('err1', err1)
        
        self.counter += 1
        
        #rospy.loginfo(f'{self.name}: {self.metric_dict}')

import rospy
import numpy as np
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from cv_bridge import CvBridge
import cv2
from image_geometry import PinholeCameraModel
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header

class DepthToPointCloud:
    def __init__(self, cam_name, cam_number):
        self.bridge = CvBridge()
        self.camera_model = PinholeCameraModel()
        self.camera_info_received = False  # Add this flag
        self.points = None
        self.cam_name = cam_name
        self.cam_number = cam_number
        
        # Subscribers
        self.depth_sub = rospy.Subscriber(f'depth_estimation/{self.cam_name}/disp_map_rescaled_v2', Image, self.depth_callback)
        self.info_sub = rospy.Subscriber(f'/{self.cam_name}/camera_info', CameraInfo, self.info_callback)
        #self.gt_pcl_sub = rospy.Subscriber('/ouster/points', PointCloud2, self.evaluation_pcl)
        
        # Publisher
        self.pc_pub = rospy.Publisher(f'/depth_estimation/{self.cam_name}/point_cloud', PointCloud2, queue_size=1)
        
    def info_callback(self, msg):
        self.camera_model.fromCameraInfo(msg)
        self.camera_info_received = True  # Set flag when we receive camera info
        
    def depth_callback(self, msg):
        # Fix: Use the flag instead of checking the array directly
        #start_time_pcl = rospy.Time.now()
        if not self.camera_info_received:
            return
            
        # Convert ROS image to OpenCV
        depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        
        # Get camera parameters
        fx = self.camera_model.fx()
        fy = self.camera_model.fy()
        cx = self.camera_model.cx()
        cy = self.camera_model.cy()

        # Create point cloud
        points = self.depth_to_pointcloud(depth_image, fx, fy, cx, cy)

        #print("shape point cloud = ", points.shape)
        
        # Create PointCloud2 message
        header = Header()
        header.stamp = msg.header.stamp
        header.frame_id = f"ZED{self.cam_number}_pcl"
        
        pc_msg = pc2.create_cloud_xyz32(header, points)
        self.pc_pub.publish(pc_msg)

        # processing_time = (rospy.Time.now() - start_time_pcl).to_sec()
        # rospy.loginfo(f"Point cloud extraction completed in {processing_time:.2f}s")

    def depth_to_pointcloud(self, depth_image, fx, fy, cx, cy):
        height, width = depth_image.shape
        #print("depth pred shape = ", depth_image.shape)

        # Create coordinate grids
        u, v = np.meshgrid(np.arange(width), np.arange(height))
        
        # Convert to 3D coordinates
        X = (u - cx) * depth_image / fx
        Y = (v - cy) * depth_image / fy
        Z = depth_image
        
        valid_mask = (Z > 0) & np.isfinite(Z)

        # Add distance filtering
        max_depth = rospy.get_param('~max_depth_pcl')
        valid_mask = valid_mask & (Z <= max_depth)

        # Stack into point cloud (N x 3)
        # Apply mask and flatten
        self.points = np.stack([X[valid_mask], Y[valid_mask], Z[valid_mask]], axis=1)
        #print("len total point cloud = ", len(self.points))
        
        return self.points
    
    def evaluation_pcl(self, gt_pcl_msg):
        from sklearn.neighbors import NearestNeighbors
        """
        Compute Chamfer Distance between point clouds.
        Returns the symmetric Chamfer distance.
        """
        if self.points is not None: # also requires synchronisation between both point cloud
            self.estimated = self.points
            self.ground_truth = self.pointcloud2_to_numpy(gt_pcl_msg)

            # Distance from estimated to ground truth
            nbrs_gt = NearestNeighbors(n_neighbors=1, algorithm='auto').fit(self.ground_truth)
            distances_est_to_gt, _ = nbrs_gt.kneighbors(self.estimated)
            
            # Distance from ground truth to estimated
            nbrs_est = NearestNeighbors(n_neighbors=1, algorithm='auto').fit(self.estimated)
            distances_gt_to_est, _ = nbrs_est.kneighbors(self.ground_truth)
            
            # Chamfer distance (symmetric)
            chamfer_dist = np.mean(distances_est_to_gt) + np.mean(distances_gt_to_est)
            est_to_gt_mean = np.mean(distances_est_to_gt)
            gt_to_est_mean = np.mean(distances_gt_to_est)
            
            rospy.loginfo(f"Chamfer Distance (m): {chamfer_dist:.4f}")
            # rospy.loginfo(f"  Estimated to Ground Truth: {est_to_gt_mean:.4f}")
            # rospy.loginfo(f"  Ground Truth to Estimated: {gt_to_est_mean:.4f}")
            """
            Compute completeness metrics (how well ground truth is covered).
            """
            threshold = 0.1
            nbrs = NearestNeighbors(n_neighbors=1, algorithm='auto').fit(self.estimated)
            distances, _ = nbrs.kneighbors(self.ground_truth)
            distances = distances.flatten()
            
            completeness = np.sum(distances <= threshold) / len(distances)

            rospy.loginfo(f"Completeness: {completeness}")
            rospy.loginfo(f"num_covered_points: {np.sum(distances <= threshold)}")
            rospy.loginfo(f"total_gt_points: {len(distances)}")

    def pointcloud2_to_numpy(self, cloud_msg):
        from sensor_msgs import point_cloud2
        """
        Convert PointCloud2 message to numpy array.
        
        Args:
            cloud_msg: sensor_msgs/PointCloud2 message
            
        Returns:
            numpy array of shape (N, 3) with x, y, z coordinates
        """
        # Method 1: Using point_cloud2 utilities (recommended)
        points_list = []
        
        for point in point_cloud2.read_points(cloud_msg, skip_nans=True, field_names=("x", "y", "z")):
            points_list.append([point[0], point[1], point[2]])
        
        if len(points_list) == 0:
            return np.array([]).reshape(0, 3)
        
        return np.array(points_list)

# def mask_edges(raw_img):
#     gray = cv2.cvtColor(raw_img, cv2.COLOR_BGR2GRAY)
#     # sobelx = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=3)
#     # sobely = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=3)
#     # edges = cv2.magnitude(sobelx, sobely).astype(np.uint8)
#     edges = cv2.Canny(gray, 100, 200)
#     mask = cv2.bitwise_not(edges)
#     mask_3ch = cv2.merge([mask, mask, mask])
#     mask_img = cv2.bitwise_and(raw_img, mask_3ch)
#     return mask_img