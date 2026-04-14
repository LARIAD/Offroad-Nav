#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header
import message_filters
import open3d as o3d
import copy
import random
from threading import Lock
import tf2_ros
import tf2_geometry_msgs
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from geometry_msgs.msg import PointStamped

class PointCloudRegistration:
    def __init__(self):
        rospy.init_node('pointcloud_registration_node', anonymous=True)
        
        # Parameters
        self.cloud1_topic = rospy.get_param('~cloud1_topic')
        self.cloud2_topic = rospy.get_param('~cloud2_topic')
        self.output_topic = rospy.get_param('~output_topic')
        self.target_frame = rospy.get_param('~target_frame')
        
        # Registration parameters
        self.voxel_size = rospy.get_param('~voxel_size')  # 5cm voxels
        self.distance_threshold = rospy.get_param('~distance_threshold')  # 2cm
        self.max_iterations = rospy.get_param('~max_iterations')
        self.use_coarse_registration = rospy.get_param('~use_coarse_registration')
        self.use_point_to_plane = rospy.get_param('~use_point_to_plane')  # True for point-to-plane, False for point-to-point
        self.estimate_normals = rospy.get_param('~estimate_normals')  # Skip normal estimation if using point-to-point
        self.coarse_method = rospy.get_param('~coarse_method')  # 'fpfh', 'closest_point'
        self.enable_downsampling = rospy.get_param('~enable_downsampling')  # Enable/disable voxel downsampling
        
        # Thread safety
        self.lock = Lock()

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # Publisher
        self.merged_pub = rospy.Publisher(self.output_topic, PointCloud2, queue_size=1)
        
        # Synchronized subscribers
        self.setup_subscribers()
        
        # rospy.loginfo(f"Point cloud registration node started")
        # rospy.loginfo(f"Subscribing to: {self.cloud1_topic}, {self.cloud2_topic}")
        # rospy.loginfo(f"Publishing to: {self.output_topic}")

    def setup_subscribers(self):
        """Setup synchronized subscribers for both point clouds"""
        cloud1_sub = message_filters.Subscriber(self.cloud1_topic, PointCloud2)
        cloud2_sub = message_filters.Subscriber(self.cloud2_topic, PointCloud2)
        
        # Synchronize messages with 0.1 second tolerance
        ts = message_filters.ApproximateTimeSynchronizer(
            [cloud1_sub, cloud2_sub], 
            queue_size=10, 
            slop=0.1
        )
        ts.registerCallback(self.pointcloud_callback)


    def transform_pointcloud(self, pointcloud_msg):
        """Transform point cloud to target frame"""
        if pointcloud_msg.header.frame_id == self.target_frame:
            return pointcloud_msg
            
        try:
            # Get transform
            transform = self.tf_buffer.lookup_transform(
                self.target_frame, 
                pointcloud_msg.header.frame_id,
                pointcloud_msg.header.stamp,
                rospy.Duration(0.5)
            )

            transformed_pc = do_transform_cloud(pointcloud_msg, transform)
            
            # Extract points from point cloud
            # points = []
            # for point in pc2.read_points(pointcloud_msg, field_names=("x", "y", "z"), skip_nans=True):
            #     points.append([point[0], point[1], point[2]])
            
            # if not points:
            #     return None
                
            # points = np.array(points)
            
            # # Transform points
            # transformed_points = []
            # for point in points:
            #     point_stamped = PointStamped()
            #     point_stamped.header = pointcloud_msg.header
            #     point_stamped.point.x = point[0]
            #     point_stamped.point.y = point[1]
            #     point_stamped.point.z = point[2]
                
            #     # Apply transformation
            #     transformed_point = tf2_geometry_msgs.do_transform_point(point_stamped, transform)
            #     transformed_points.append([
            #         transformed_point.point.x,
            #         transformed_point.point.y,
            #         transformed_point.point.z
            #     ])
            
            # Create new point cloud message
            # header = Header()
            # header.stamp = pointcloud_msg.header.stamp
            # header.frame_id = self.target_frame
            
            # transformed_pc = pc2.create_cloud_xyz32(header, transformed_points)
            return transformed_pc
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(f"Transform failed: {str(e)}")
            return None

    def ros_to_o3d(self, ros_cloud):
        """Convert ROS PointCloud2 to Open3D point cloud"""
        try:
            # Extract points from ROS message
            points = []
            for point in pc2.read_points(ros_cloud, field_names=("x", "y", "z"), skip_nans=True):
                points.append([point[0], point[1], point[2]])
            
            if len(points) == 0:
                rospy.logwarn("Empty point cloud received")
                return None
                
            # Create Open3D point cloud
            o3d_cloud = o3d.geometry.PointCloud()
            o3d_cloud.points = o3d.utility.Vector3dVector(np.array(points))
            
            return o3d_cloud
            
        except Exception as e:
            rospy.logerr(f"Error converting ROS to Open3D: {e}")
            return None

    def o3d_to_ros(self, o3d_cloud, header):
        """Convert Open3D point cloud to ROS PointCloud2"""
        try:
            points = np.asarray(o3d_cloud.points)
            
            # Create ROS PointCloud2 message
            ros_cloud = pc2.create_cloud_xyz32(header, points)
            return ros_cloud
            
        except Exception as e:
            rospy.logerr(f"Error converting Open3D to ROS: {e}")
            return None

    def preprocess_cloud(self, cloud):
        """Downsample and optionally estimate normals"""
        if cloud is None or len(cloud.points) == 0:
            return None
        
        # Downsampling (optional)
        if self.enable_downsampling:
            cloud_down = cloud.voxel_down_sample(self.voxel_size)
            rospy.logdebug(f"Downsampled from {len(cloud.points)} to {len(cloud_down.points)} points")
        else:
            cloud_down = cloud
            rospy.logdebug("Skipping downsampling (using original point density)")
        
        # Estimate normals only if needed for point-to-plane ICP
        if self.estimate_normals and self.use_point_to_plane:
            cloud_down.estimate_normals(
                search_param=o3d.geometry.KDTreeSearchParamHybrid(
                    radius=self.voxel_size * 2, max_nn=30))
            rospy.logdebug("Normals estimated for point-to-plane ICP")
        else:
            rospy.logdebug("Skipping normal estimation (using point-to-point ICP)")
        
        return cloud_down
    
    def compute_fpfh_features(self, cloud):
        """Compute FPFH features for coarse registration"""
        if cloud is None:
            return None
        
        # FPFH requires normals - check if they exist
        if not cloud.has_normals():
            rospy.logwarn("Cannot compute FPFH features: point cloud has no normals")
            return None
            
        try:
            fpfh = o3d.pipelines.registration.compute_fpfh_feature(
                cloud,
                o3d.geometry.KDTreeSearchParamHybrid(
                    radius=self.voxel_size * 5, max_nn=100))
            return fpfh
        except Exception as e:
            rospy.logerr(f"Failed to compute FPFH features: {e}")
            return None

    def coarse_registration_fpfh(self, source, target):
        """Perform coarse registration using RANSAC + FPFH (requires normals)"""
        try:
            # Compute features
            source_fpfh = self.compute_fpfh_features(source)
            target_fpfh = self.compute_fpfh_features(target)
            
            if source_fpfh is None or target_fpfh is None:
                rospy.logwarn("Failed to compute FPFH features. Skipping coarse registration.")
                return np.eye(4)
            
            # RANSAC registration
            result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
                source, target, source_fpfh, target_fpfh,
                mutual_filter=True,
                max_correspondence_distance=self.distance_threshold * 10,
                estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
                ransac_n=3,
                checkers=[
                    o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
                    o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(self.distance_threshold * 10)
                ],
                criteria=o3d.pipelines.registration.RANSACConvergenceCriteria(100000, 0.999)
            )
            
            #rospy.loginfo(f"FPFH coarse registration fitness: {result.fitness:.3f}")
            return result.transformation
            
        except Exception as e:
            rospy.logerr(f"FPFH coarse registration failed: {e}")
            return np.eye(4)

    def coarse_registration_no_features(self, source, target):
        """Perform coarse registration using closest point correspondences (no features needed)"""
        try:
            #rospy.loginfo("Using closest-point correspondences for RANSAC")
            
            # Create correspondences by finding closest points
            source_points = np.asarray(source.points)
            target_tree = o3d.geometry.KDTreeFlann(target)
            
            correspondences = []
            # Use stricter distance for initial correspondences to avoid false matches
            max_distance = min(self.distance_threshold * 3, 0.1)  # Cap at 10cm
            
            #rospy.loginfo(f"Finding correspondences with max distance: {max_distance:.3f}m")
            
            for i, point in enumerate(source_points):
                [k, idx, distances] = target_tree.search_knn_vector_3d(point, 1)
                if k > 0 and distances[0] < max_distance:
                    correspondences.append([i, idx[0]])
            
            #rospy.loginfo(f"Found {len(correspondences)} initial correspondences")
            
            if len(correspondences) < 10:  # Need more correspondences for reliable registration
                rospy.logwarn(f"Not enough correspondences found ({len(correspondences)}). Try increasing distance_threshold or improving initial alignment.")
                return np.eye(4)
            
            # Filter correspondences to keep only the most reliable ones
            if len(correspondences) > 1000:
                # If too many correspondences, sample them to avoid computational overhead
                import random
                correspondences = random.sample(correspondences, 1000)
                #rospy.loginfo("Sampled correspondences to 1000 for efficiency")
            
            # Convert to Open3D format
            corres = o3d.utility.Vector2iVector(correspondences)
            
            # Use Open3D's RANSAC with correspondences
            result = o3d.pipelines.registration.registration_ransac_based_on_correspondence(
                source, target, corres,
                max_correspondence_distance=max_distance,
                estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
                ransac_n=3,
                checkers=[
                    o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
                    o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(max_distance)
                ],
                criteria=o3d.pipelines.registration.RANSACConvergenceCriteria(100000, 0.999)  # More iterations for better results
            )
            
            #rospy.loginfo(f"Coarse registration (closest-point) fitness: {result.fitness:.3f}")
            
            # Check if registration quality is reasonable
            if result.fitness < 0.1:
                rospy.logwarn(f"Poor coarse registration quality (fitness: {result.fitness:.3f}). Consider manual initial alignment.")
            
            return result.transformation
            
        except Exception as e:
            rospy.logerr(f"Coarse registration without features failed: {e}")
            return np.eye(4)



    def fine_registration(self, source, target, init_transform=None):
        """Perform fine registration using ICP (point-to-point or point-to-plane)"""
        try:
            if init_transform is None:
                init_transform = np.eye(4)
            
            # Choose ICP method based on configuration
            if self.use_point_to_plane:
                estimation_method = o3d.pipelines.registration.TransformationEstimationPointToPlane()
                method_name = "Point-to-plane"
            else:
                estimation_method = o3d.pipelines.registration.TransformationEstimationPointToPoint()
                method_name = "Point-to-point"
            
            #rospy.loginfo(f"Using {method_name} ICP")
            
            # ICP registration
            result = o3d.pipelines.registration.registration_icp(
                source, target,
                max_correspondence_distance=self.distance_threshold,
                init=init_transform,
                estimation_method=estimation_method,
                criteria=o3d.pipelines.registration.ICPConvergenceCriteria(
                    max_iteration=self.max_iterations)
            )
            
            # rospy.loginfo(f"Fine registration ({method_name}) fitness: {result.fitness:.3f}, RMSE: {result.inlier_rmse:.3f}")
            return result.transformation, result.fitness
            
        except Exception as e:
            rospy.logerr(f"Fine registration failed: {e}")
            return np.eye(4), 0.0

    def register_clouds(self, cloud1, cloud2):
        """Main registration pipeline"""
        if cloud1 is None or cloud2 is None:
            return None
            
        #start_time_prep = rospy.Time.now()

        # Preprocess clouds
        source = self.preprocess_cloud(cloud1)
        target = self.preprocess_cloud(cloud2)

        # processing_time = (rospy.Time.now() - start_time_prep).to_sec()
        # rospy.loginfo(f"Preprocessing completed in {processing_time:.2f}s")
        
        if source is None or target is None:
            rospy.logwarn("Preprocessing failed")
            return None
            
        #rospy.loginfo(f"Source: {len(source.points)} points, Target: {len(target.points)} points")
        
        # Registration pipeline
        init_transform = np.eye(4)
        
        #start_time_coarse = rospy.Time.now()

        if self.use_coarse_registration:
            if self.coarse_method == 'fpfh':
                if source.has_normals() and target.has_normals():
                    #rospy.loginfo("Performing FPFH-based coarse registration...")
                    init_transform = self.coarse_registration_fpfh(source, target)
                else:
                    rospy.loginfo("FPFH requires normals, falling back to closest-point RANSAC")
                    init_transform = self.coarse_registration_no_features(source, target)
            elif self.coarse_method == 'closest_point':
                #rospy.loginfo("Performing closest-point RANSAC...")
                init_transform = self.coarse_registration_no_features(source, target)
            else:
                rospy.logwarn(f"Unknown coarse method: {self.coarse_method}. Skipping coarse registration.")

        # processing_time = (rospy.Time.now() - start_time_coarse).to_sec()
        # rospy.loginfo(f"Coarse Registration completed in {processing_time:.2f}s")
        
        #start_time_fine = rospy.Time.now()

        #rospy.loginfo("Performing fine registration...")
        final_transform, fitness = self.fine_registration(source, target, init_transform)

        # processing_time = (rospy.Time.now() - start_time_fine).to_sec()
        # rospy.loginfo(f"Fine Registration completed in {processing_time:.2f}s")
        
        # Check registration quality
        # if fitness < 0.2:  # More strict threshold for overlap detection
        #     rospy.logwarn(f"Poor registration quality (fitness: {fitness:.3f}). This might indicate:")
        #     rospy.logwarn("  1. Insufficient overlap between point clouds")
        #     rospy.logwarn("  2. Poor initial alignment - consider manual pre-alignment")
        #     rospy.logwarn("  3. Very different point cloud characteristics")
        #     rospy.logwarn("  4. Distance threshold might be too strict")
        
        # Apply transformation to original cloud1
        cloud1_transformed = cloud1.transform(final_transform)
        
        # Merge clouds - this should give you LARGER FOV, not just overlap
        merged_cloud = cloud1_transformed + cloud2
        
        # rospy.loginfo(f"Registration summary:")
        # rospy.loginfo(f"  Cloud 1 original: {len(cloud1.points)} points")
        # rospy.loginfo(f"  Cloud 2 original: {len(cloud2.points)} points") 
        # rospy.loginfo(f"  Merged result: {len(merged_cloud.points)} points")
        # rospy.loginfo(f"  Expected: ~{len(cloud1.points) + len(cloud2.points)} points (if little overlap)")
        
        # Verify the merge makes sense
        expected_points = len(cloud1.points) + len(cloud2.points)
        if len(merged_cloud.points) < expected_points * 0.7:  # Less than 70% of expected
            rospy.logwarn("Merged cloud has surprisingly few points. Check if registration is correctly aligning non-overlapping regions.")
        
        return merged_cloud

    def pointcloud_callback(self, cloud1_msg, cloud2_msg):
        """Callback for synchronized point cloud messages"""
        with self.lock:
            start_time = rospy.Time.now()
            
            #rospy.loginfo("Received synchronized point clouds, starting registration...")

            transform_ros_cloud1 = self.transform_pointcloud(cloud1_msg)
            transform_ros_cloud2 = self.transform_pointcloud(cloud2_msg)

            # processing_time = (rospy.Time.now() - start_time).to_sec()
            # rospy.loginfo(f"Transform completed in {processing_time:.2f}s")
            
            # Convert ROS messages to Open3D
            o3d_cloud1 = self.ros_to_o3d(transform_ros_cloud1)
            o3d_cloud2 = self.ros_to_o3d(transform_ros_cloud2)
            
            if o3d_cloud1 is None or o3d_cloud2 is None:
                rospy.logwarn("Failed to convert point clouds")
                return
            
            # Perform registration
            merged_cloud = self.register_clouds(o3d_cloud1, o3d_cloud2)
            
            if merged_cloud is None:
                rospy.logwarn("Registration failed")
                return
            
            # Convert back to ROS and publish
            header = Header()
            header.stamp = rospy.Time.now()
            header.frame_id = self.target_frame
            
            merged_msg = self.o3d_to_ros(merged_cloud, header)
            
            if merged_msg is not None:
                self.merged_pub.publish(merged_msg)
                
                processing_time = (rospy.Time.now() - start_time).to_sec()
                rospy.loginfo(f"Registration completed in {processing_time:.2f}s")
            else:
                rospy.logwarn("Failed to convert merged cloud to ROS message")

if __name__ == '__main__':
    try:
        node = PointCloudRegistration()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Point cloud registration node terminated")