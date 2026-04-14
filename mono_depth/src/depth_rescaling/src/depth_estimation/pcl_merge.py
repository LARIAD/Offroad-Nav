#!/usr/bin/env python

import rospy
import numpy as np
import ros_numpy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header
import tf2_ros
import tf2_geometry_msgs
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from geometry_msgs.msg import PointStamped
from sklearn.neighbors import NearestNeighbors
import message_filters
from tf.transformations import quaternion_matrix, translation_matrix

from concurrent.futures import ThreadPoolExecutor, as_completed

class PointCloudMerger:
    def __init__(self):
        rospy.init_node('pointcloud_merger', anonymous=True)
        
        # Parameters
        self.topic1 = rospy.get_param('~topic1', '/pointcloud1')
        self.topic2 = rospy.get_param('~topic2', '/pointcloud2')
        self.output_topic = rospy.get_param('~output_topic', '/merged_pointcloud')
        self.target_frame = rospy.get_param('~target_frame', 'base_link')
        self.merge_threshold = rospy.get_param('~merge_threshold', 0.05)  # 5cm threshold for merging
        self.sync_slop = rospy.get_param('~sync_slop', 0.1)  # Time synchronization tolerance
        self.queue_size = rospy.get_param('~queue_size', 10)  # Queue size for message filters
        self.use_voxel_filter = rospy.get_param('~use_voxel_filter', True)  # Use voxel filtering for speed
        self.voxel_size = rospy.get_param('~voxel_size', 0.02)  # 2cm voxel size
        
        # TF2 buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.transform_cache = {}
        
        self.thread_pool = ThreadPoolExecutor(max_workers=2)
        
        # Wait for TF to be ready
        rospy.sleep(1.0)
        
        # Message filters for synchronization
        self.sub1 = message_filters.Subscriber(self.topic1, PointCloud2)
        self.sub2 = message_filters.Subscriber(self.topic2, PointCloud2)
        
        # Approximate time synchronizer
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.sub1, self.sub2], 
            queue_size=self.queue_size, 
            slop=self.sync_slop,
            allow_headerless=False
        )
        self.sync.registerCallback(self.synchronized_callback)
        
        # Publisher
        self.pub = rospy.Publisher(self.output_topic, PointCloud2, queue_size=1)
        
        rospy.loginfo(f"Point Cloud Merger initialized")
        rospy.loginfo(f"Topic 1: {self.topic1}")
        rospy.loginfo(f"Topic 2: {self.topic2}")
        rospy.loginfo(f"Output: {self.output_topic}")
        rospy.loginfo(f"Target frame: {self.target_frame}")
        rospy.loginfo(f"Merge threshold: {self.merge_threshold}m")
        rospy.loginfo(f"Sync slop: {self.sync_slop}s")
        rospy.loginfo(f"Queue size: {self.queue_size}")

    def synchronized_callback(self, pc1_msg, pc2_msg):
        """Callback for synchronized point cloud messages"""
        try:
            start_time = rospy.Time.now()
            rospy.logdebug(f"Processing synchronized point clouds - "
                          f"PC1: {pc1_msg.header.stamp.to_sec():.3f}, "
                          f"PC2: {pc2_msg.header.stamp.to_sec():.3f}")
            
            #merged_pc = self.merge_pointclouds(pc1_msg, pc2_msg)

            future1 = self.thread_pool.submit(self.transform_pointcloud, pc1_msg, self.target_frame)
            future2 = self.thread_pool.submit(self.transform_pointcloud, pc2_msg, self.target_frame)

            # Wait for both transformations to complete
            transformed_clouds = []
            for future in as_completed([future1, future2]):
                try:
                    result = future.result(timeout=5.0)  # 5 second timeout
                    if result is not None:
                        transformed_clouds.append(result)
                except Exception as e:
                    rospy.logerr(f"Error in point cloud transformation: {e}")
            
            # Merge point clouds if we have both transformed clouds
            if len(transformed_clouds) == 2:
                merged_pc = self.concat_pcl_new(transformed_clouds[0], transformed_clouds[1])
                #merged_pc = self.remove_overlap(transformed_clouds[0], transformed_clouds[1])
            else:
                rospy.logwarn("Could not transform both point clouds, skipping merge")

            if merged_pc is not None:
                self.pub.publish(merged_pc)
                rospy.logdebug("Published merged point cloud")
                processing_time = (rospy.Time.now() - start_time).to_sec()
                rospy.loginfo(f"Registration completed in {processing_time:.2f}s")
            else:
                rospy.logwarn("Failed to merge point clouds")
                
        except Exception as e:
            rospy.logerr(f"Error in synchronized callback: {str(e)}")

    def concat_pcl_new(self, pc1_msg, pc2_msg):
        # Convert to numpy arrays
        pc1_array = ros_numpy.point_cloud2.pointcloud2_to_array(pc1_msg)
        pc2_array = ros_numpy.point_cloud2.pointcloud2_to_array(pc2_msg)
        # rospy.loginfo(f"len pc1 array = {len(pc1_array)}")
        # rospy.loginfo(f"len pc2 array = {len(pc2_array)}")
        
        # Concatenate arrays
        merged_array = np.concatenate([pc1_array, pc2_array])
        #rospy.loginfo(f"len merged_array array = {len(merged_array)}")
        
        # Convert back to PointCloud2
        merged_msg = ros_numpy.point_cloud2.array_to_pointcloud2(merged_array)
        merged_msg.header.frame_id = self.target_frame
        merged_msg.header.stamp = rospy.Time.now()

        return merged_msg

    def concat_pcl(self, pc1_transformed, pc2_transformed):
        merged_pc = PointCloud2()
        merged_pc.header = pc1_transformed.header  # Use first point cloud's header
        merged_pc.header.frame_id = self.target_frame  # Set to target frame
        merged_pc.height = 1  # Unorganized point cloud
        merged_pc.width = pc1_transformed.width + pc2_transformed.width
        merged_pc.fields = pc1_transformed.fields
        merged_pc.is_bigendian = pc1_transformed.is_bigendian
        merged_pc.point_step = pc1_transformed.point_step
        merged_pc.row_step = merged_pc.width * merged_pc.point_step
        merged_pc.data = pc1_transformed.data + pc2_transformed.data
        merged_pc.is_dense = pc1_transformed.is_dense and pc2_transformed.is_dense
        return merged_pc

    def transform_pointcloud(self, pointcloud_msg, target_frame):
        """Transform point cloud to target frame"""
        if pointcloud_msg.header.frame_id == target_frame:
            return pointcloud_msg
            
        try:        
            # Get transform at the time of the point cloud
            transform = self.tf_buffer.lookup_transform(
                target_frame, 
                pointcloud_msg.header.frame_id,
                pointcloud_msg.header.stamp,
                rospy.Duration(1.0)  # Wait up to 1 second for transform
            )

            transformed_pc = do_transform_cloud(pointcloud_msg, transform)
            
            return transformed_pc
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(f"Transform failed from {pointcloud_msg.header.frame_id} to {target_frame}: {str(e)}")
            return None
    
    def voxel_filter(self, points, voxel_size):
        """Simple voxel filtering to reduce point density"""
        if len(points) == 0:
            return points
            
        # Convert to voxel coordinates
        voxel_coords = np.floor(points / voxel_size).astype(np.int32)
        
        # Get unique voxels and their indices
        _, unique_indices = np.unique(voxel_coords, axis=0, return_index=True)
        
        return points[unique_indices]

    def merge_pointclouds(self, pc1_msg, pc2_msg):
        """Merge two synchronized point clouds, removing overlapping points"""
        start_time_transform = rospy.Time.now()

        # Transform both point clouds to target frame
        pc1_transformed = self.transform_pointcloud(pc1_msg, self.target_frame)
        pc2_transformed = self.transform_pointcloud(pc2_msg, self.target_frame)

        processing_time = (rospy.Time.now() - start_time_transform).to_sec()
        rospy.loginfo(f"transform point cloud completed in {processing_time:.2f}s")
        
        if pc1_transformed is None and pc2_transformed is None:
            rospy.logwarn("Failed to transform both point clouds")
            return None
        elif pc1_transformed is None:
            rospy.logwarn("Failed to transform first point cloud, using only second")
            return pc2_transformed
        elif pc2_transformed is None:
            rospy.logwarn("Failed to transform second point cloud, using only first")
            return pc1_transformed

        return self.remove_overlap(pc1_transformed, pc2_transformed)

    def remove_overlap(self, pc1_transformed, pc2_transformed):
        start_time_prep = rospy.Time.now()
        # Extract points
        points1 = np.array(list(pc2.read_points(pc1_transformed, field_names=("x", "y", "z"), skip_nans=True)))
        points2 = np.array(list(pc2.read_points(pc2_transformed, field_names=("x", "y", "z"), skip_nans=True)))

        # processing_time = (rospy.Time.now() - start_time_prep).to_sec()
        # rospy.loginfo(f"preprocess extract point cloud completed in {processing_time:.2f}s")

        if self.use_voxel_filter:
            points1 = self.voxel_filter(points1, self.voxel_size)
            points2 = self.voxel_filter(points2, self.voxel_size)

        processing_time = (rospy.Time.now() - start_time_prep).to_sec()
        rospy.loginfo(f"preprocess point cloud completed in {processing_time:.2f}s")
        
        if len(points1) == 0 and len(points2) == 0:
            rospy.logwarn("Both point clouds are empty after transformation")
            return None
        elif len(points1) == 0:
            rospy.loginfo("First point cloud is empty, using only second")
            points1 = np.empty((0, 3))
        elif len(points2) == 0:
            rospy.loginfo("Second point cloud is empty, using only first")
            points2 = np.empty((0, 3))
        
        start_time_merge = rospy.Time.now()

        # Find overlapping points and merge
        merged_points = self.remove_overlapping_points(points1, points2)

        processing_time = (rospy.Time.now() - start_time_merge).to_sec()
        rospy.loginfo(f"merge point cloud completed in {processing_time:.2f}s")
        
        # Create merged point cloud message with synchronized timestamp
        header = Header()
        # Use the average timestamp of both input clouds for better synchronization
        stamp1 = pc1_transformed.header.stamp.to_sec()
        stamp2 = pc2_transformed.header.stamp.to_sec()
        avg_timestamp = (stamp1 + stamp2) / 2.0
        header.stamp = rospy.Time.from_sec(avg_timestamp)
        header.frame_id = self.target_frame
        
        merged_pc = pc2.create_cloud_xyz32(header, merged_points.tolist())
        
        rospy.loginfo(f"Merged point clouds: {len(points1)} + {len(points2)} -> {len(merged_points)} points "
                     f"(sync diff: {abs(stamp1 - stamp2)*1000:.1f}ms)")

        return merged_pc

    def remove_overlapping_points(self, points1, points2):
        """Remove overlapping points between two point clouds"""
        if len(points1) == 0:
            return points2
        if len(points2) == 0:
            return points1
            
        # Use KD-tree for efficient nearest neighbor search
        try:
            nbrs = NearestNeighbors(n_neighbors=1, algorithm='kd_tree').fit(points1)
            distances, indices = nbrs.kneighbors(points2)
            
            # Keep points from points2 that are far enough from points1
            mask = distances.flatten() > self.merge_threshold
            points2_filtered = points2[mask]
            
            # Combine points1 (all kept) with filtered points2
            if len(points2_filtered) == 0:
                merged_points = points1
            else:
                merged_points = np.vstack([points1, points2_filtered])
            
            overlap_count = len(points2) - len(points2_filtered)
            if overlap_count > 0:
                rospy.logdebug(f"Removed {overlap_count} overlapping points from second cloud")
            
            return merged_points
            
        except Exception as e:
            rospy.logerr(f"Error in overlap removal: {str(e)}")
            # Fallback: just concatenate the point clouds
            return np.vstack([points1, points2])

    def run(self):
        """Main run loop"""
        rospy.loginfo("Point Cloud Merger running... Waiting for synchronized messages")
        rospy.loginfo(f"Subscribing to:")
        rospy.loginfo(f"  - {self.topic1}")
        rospy.loginfo(f"  - {self.topic2}")
        rospy.loginfo(f"Publishing to: {self.output_topic}")
        
        # Check if topics exist
        rospy.sleep(2.0)  # Give some time for publishers to start
        
        try:
            # Wait for at least one message on each topic to confirm they exist
            rospy.loginfo("Waiting for first messages...")
            
            # This will timeout if topics don't exist
            rospy.wait_for_message(self.topic1, PointCloud2, timeout=10.0)
            rospy.loginfo(f"✓ Detected messages on {self.topic1}")
            
            rospy.wait_for_message(self.topic2, PointCloud2, timeout=10.0)
            rospy.loginfo(f"✓ Detected messages on {self.topic2}")
            
            rospy.loginfo("Both topics detected. Starting synchronization...")
            
        except rospy.ROSException as e:
            rospy.logerr(f"Timeout waiting for topics: {str(e)}")
            rospy.logerr("Please check that both point cloud topics are publishing")
            return
        
        rospy.spin()

if __name__ == '__main__':
    try:
        merger = PointCloudMerger()
        merger.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Point Cloud Merger shutting down")
    except Exception as e:
        rospy.logerr(f"Point Cloud Merger failed: {str(e)}")