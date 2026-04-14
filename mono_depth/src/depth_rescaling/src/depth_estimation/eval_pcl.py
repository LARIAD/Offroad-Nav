#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Float32MultiArray
import message_filters
from sklearn.neighbors import NearestNeighbors
import time

class PointCloudChamferEvaluator:
    def __init__(self):
        rospy.init_node('pointcloud_chamfer_evaluator', anonymous=True)
        
        # Parameters
        self.gt_topic = rospy.get_param('~gt_topic', '/ground_truth_pointcloud')
        self.est_topic = rospy.get_param('~estimated_topic', '/estimated_pointcloud')
        self.target_points = rospy.get_param('~target_points', 30000)
        self.max_distance = rospy.get_param('~max_distance', 5.0)  # meters
        self.publish_rate = rospy.get_param('~publish_rate', 1.0)  # Hz
        
        # Storage for point clouds
        self.gt_points = None
        self.est_points = None
        self.last_evaluation_time = 0
        
        # Publishers
        self.metrics_pub = rospy.Publisher('/chamfer_metrics', Float32MultiArray, queue_size=1)
        
        # Synchronized subscribers
        gt_sub = message_filters.Subscriber(self.gt_topic, PointCloud2)
        est_sub = message_filters.Subscriber(self.est_topic, PointCloud2)
        
        # Synchronize the topics
        ts = message_filters.ApproximateTimeSynchronizer(
            [gt_sub, est_sub], queue_size=10, slop=100
        )
        ts.registerCallback(self.pointcloud_callback)
        
        rospy.loginfo(f"Subscribed to GT: {self.gt_topic}")
        rospy.loginfo(f"Subscribed to EST: {self.est_topic}")
        rospy.loginfo(f"Target points for comparison: {self.target_points}")
        rospy.loginfo("Node started. Waiting for synchronized point clouds...")

    def pointcloud_to_array(self, pc_msg):
        """Convert ROS PointCloud2 to numpy array"""
        points = []
        for point in pc2.read_points(pc_msg, field_names=("x", "y", "z"), skip_nans=True):
            points.append([point[0], point[1], point[2]])
        return np.array(points)

    def downsample_random(self, points, target_size):
        """Randomly downsample points to target size"""
        if len(points) <= target_size:
            return points
        indices = np.random.choice(len(points), target_size, replace=False)
        return points[indices]

    def downsample_voxel_grid(self, points, target_size):
        """Downsample using voxel grid approach"""
        if len(points) <= target_size:
            return points
        
        # Calculate voxel size based on point cloud bounds and target size
        bounds = np.max(points, axis=0) - np.min(points, axis=0)
        volume = np.prod(bounds)
        voxel_volume = volume / target_size
        voxel_size = np.power(voxel_volume, 1/3)
        
        # Create voxel indices
        min_coords = np.min(points, axis=0)
        voxel_indices = ((points - min_coords) / voxel_size).astype(int)
        
        # Get unique voxels and select one point per voxel
        unique_voxels, unique_indices = np.unique(voxel_indices, axis=0, return_index=True)
        downsampled = points[unique_indices]
        
        # If still too many points, randomly sample
        if len(downsampled) > target_size:
            indices = np.random.choice(len(downsampled), target_size, replace=False)
            downsampled = downsampled[indices]
            
        return downsampled

    def compute_chamfer_distance(self, points1, points2):
        """Compute bidirectional chamfer distance"""
        # Remove any NaN or infinite values
        points1 = points1[~np.any(np.isnan(points1) | np.isinf(points1), axis=1)]
        points2 = points2[~np.any(np.isnan(points2) | np.isinf(points2), axis=1)]
        
        if len(points1) == 0 or len(points2) == 0:
            return None, None, None
        
        # Compute distances from points1 to points2
        nbrs1to2 = NearestNeighbors(n_neighbors=1, algorithm='kd_tree').fit(points2)
        distances1to2, _ = nbrs1to2.kneighbors(points1)
        distances1to2 = distances1to2.flatten()
        
        # Compute distances from points2 to points1
        nbrs2to1 = NearestNeighbors(n_neighbors=1, algorithm='kd_tree').fit(points1)
        distances2to1, _ = nbrs2to1.kneighbors(points2)
        distances2to1 = distances2to1.flatten()
        
        # Filter out extreme outliers
        distances1to2 = distances1to2[distances1to2 <= self.max_distance]
        distances2to1 = distances2to1[distances2to1 <= self.max_distance]
        
        return distances1to2, distances2to1

    def compute_metrics(self, distances1to2, distances2to1):
        """Compute various distance metrics"""
        metrics = {}
        
        if distances1to2 is not None and len(distances1to2) > 0:
            metrics['gt_to_est_mean'] = np.mean(distances1to2)
            metrics['gt_to_est_rmse'] = np.sqrt(np.mean(distances1to2**2))
            metrics['gt_to_est_median'] = np.median(distances1to2)
            metrics['gt_to_est_95th'] = np.percentile(distances1to2, 95)
            
            # Precision at different thresholds
            for thresh in [0.1, 0.5, 1.0]:
                metrics[f'gt_to_est_precision_{thresh}m'] = np.mean(distances1to2 <= thresh)
        
        if distances2to1 is not None and len(distances2to1) > 0:
            metrics['est_to_gt_mean'] = np.mean(distances2to1)
            metrics['est_to_gt_rmse'] = np.sqrt(np.mean(distances2to1**2))
            metrics['est_to_gt_median'] = np.median(distances2to1)
            metrics['est_to_gt_95th'] = np.percentile(distances2to1, 95)
            
            # Precision at different thresholds
            for thresh in [0.1, 0.5, 1.0]:
                metrics[f'est_to_gt_precision_{thresh}m'] = np.mean(distances2to1 <= thresh)
        
        if distances1to2 is not None and distances2to1 is not None:
            # Symmetric chamfer distance
            all_distances = np.concatenate([distances1to2, distances2to1])
            metrics['symmetric_mean'] = np.mean(all_distances)
            metrics['symmetric_rmse'] = np.sqrt(np.mean(all_distances**2))
            
            # Hausdorff distance (maximum of maximums)
            metrics['hausdorff'] = max(np.max(distances1to2), np.max(distances2to1))
        
        return metrics

    def pointcloud_callback(self, gt_msg, est_msg):
        """Callback for synchronized point cloud messages"""
        current_time = time.time()
        
        # Rate limiting
        if current_time - self.last_evaluation_time < 1.0 / self.publish_rate:
            return
        
        try:
            rospy.loginfo("Processing synchronized point clouds...")
            
            # Convert to numpy arrays
            gt_points = self.pointcloud_to_array(gt_msg)
            est_points = self.pointcloud_to_array(est_msg)
            
            rospy.loginfo(f"GT points: {len(gt_points)}, Estimated points: {len(est_points)}")
            
            if len(gt_points) == 0 or len(est_points) == 0:
                rospy.logwarn("Empty point cloud received")
                return
            
            # Downsample to target size
            if len(gt_points) > self.target_points:
                gt_downsampled = self.downsample_voxel_grid(gt_points, self.target_points)
            else:
                gt_downsampled = gt_points
                
            if len(est_points) > self.target_points:
                est_downsampled = self.downsample_voxel_grid(est_points, self.target_points)
            else:
                est_downsampled = est_points
            
            rospy.loginfo(f"After downsampling - GT: {len(gt_downsampled)}, EST: {len(est_downsampled)}")
            
            # Compute chamfer distances
            distances1to2, distances2to1 = self.compute_chamfer_distance(gt_downsampled, est_downsampled)
            
            if distances1to2 is None or distances2to1 is None:
                rospy.logwarn("Failed to compute chamfer distances")
                return
            
            # Compute metrics
            metrics = self.compute_metrics(distances1to2, distances2to1)
            
            # Log results
            rospy.loginfo("=== CHAMFER DISTANCE METRICS ===")
            for key, value in metrics.items():
                rospy.loginfo(f"{key}: {value:.4f}")
            
            # Publish metrics
            msg = Float32MultiArray()
            msg.data = list(metrics.values())
            self.metrics_pub.publish(msg)
            
            self.last_evaluation_time = current_time
            
        except Exception as e:
            rospy.logerr(f"Error in pointcloud_callback: {str(e)}")

    def run(self):
        """Run the node"""
        rospy.spin()

if __name__ == '__main__':
    try:
        evaluator = PointCloudChamferEvaluator()
        evaluator.run()
    except rospy.ROSInterruptException:
        pass