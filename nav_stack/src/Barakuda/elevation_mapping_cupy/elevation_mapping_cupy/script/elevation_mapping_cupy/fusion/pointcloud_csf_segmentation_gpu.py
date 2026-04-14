"""
GPU-Accelerated CSF Point Cloud Fusion Module - Optimized Version

This version keeps all data on GPU and avoids CPU-GPU transfers for maximum performance.
"""
 
import cupy as cp
import numpy as np
import string
from scipy.spatial.transform import Rotation
from time import time_ns as time
from typing import Tuple, Optional, List

from .fusion_manager import FusionBase
from .csf_gpu import CSF_GPU


def map_utils(
    resolution,
    width,
    height,
    sensor_noise_factor,
    min_valid_distance,
    max_height_range,
    ramped_height_range_a,
    ramped_height_range_b,
    ramped_height_range_c,
):
    """Generate CUDA utility functions for elevation mapping"""
    util_preamble = string.Template(
        """
        __device__ float16 clamp(float16 x, float16 min_x, float16 max_x) {
            return max(min(x, max_x), min_x);
        }
        __device__ int get_x_idx(float16 x, float16 center) {
            int i = (x - center) / ${resolution} + 0.5 * ${width};
            return i;
        }
        __device__ int get_y_idx(float16 y, float16 center) {
            int i = (y - center) / ${resolution} + 0.5 * ${height};
            return i;
        }
        __device__ bool is_inside(int idx) {
            int idx_x = idx / ${width};
            int idx_y = idx % ${width};
            if (idx_x == 0 || idx_x == ${width} - 1) {
                return false;
            }
            if (idx_y == 0 || idx_y == ${height} - 1) {
                return false;
            }
            return true;
        }
        __device__ int get_idx(float16 x, float16 y, float16 center_x, float16 center_y) {
            int idx_x = clamp(get_x_idx(x, center_x), 0, ${width} - 1);
            int idx_y = clamp(get_y_idx(y, center_y), 0, ${height} - 1);
            return ${width} * idx_x + idx_y;
        }
        __device__ int get_map_idx(int idx, int layer_n) {
            const int layer = ${width} * ${height};
            return layer * layer_n + idx;
        }
        __device__ float transform_p(float16 x, float16 y, float16 z,
                                     float16 r0, float16 r1, float16 r2, float16 t) {
            return r0 * x + r1 * y + r2 * z + t;
        }
        __device__ float z_noise(float16 z){
            return ${sensor_noise_factor} * z * z;
        }
        __device__ float point_sensor_distance(float16 x, float16 y, float16 z,
                                               float16 sx, float16 sy, float16 sz) {
            float d = (x - sx) * (x - sx) + (y - sy) * (y - sy) + (z - sz) * (z - sz);
            return d;
        }
        __device__ bool is_valid(float16 x, float16 y, float16 z,
                               float16 sx, float16 sy, float16 sz) {
            float d = point_sensor_distance(x, y, z, sx, sy, sz);
            float dxy = max(sqrt(x * x + y * y) - ${ramped_height_range_b}, 0.0);
            if (d < ${min_valid_distance} * ${min_valid_distance}) {
                return false;
            }
            else if (z - sz > dxy * ${ramped_height_range_a} + ${ramped_height_range_c} || z - sz > ${max_height_range}) {
                return false;
            }
            else {
                return true;
            }
        }
        __device__ float ray_vector(float16 tx, float16 ty, float16 tz,
                                    float16 px, float16 py, float16 pz,
                                    float16& rx, float16& ry, float16& rz){
            float16 vx = px - tx;
            float16 vy = py - ty;
            float16 vz = pz - tz;
            float16 norm = sqrt(vx * vx + vy * vy + vz * vz);
            if (norm > 0) {
                rx = vx / norm;
                ry = vy / norm;
                rz = vz / norm;
            }
            else {
                rx = 0;
                ry = 0;
                rz = 0;
            }
            return norm;
        }
        __device__ float inner_product(float16 x1, float16 y1, float16 z1,
                                       float16 x2, float16 y2, float16 z2) {
            float product = (x1 * x2 + y1 * y2 + z1 * z2);
            return product;
       }
        """
    ).substitute(
        resolution=resolution,
        width=width,
        height=height,
        sensor_noise_factor=sensor_noise_factor,
        min_valid_distance=min_valid_distance,
        max_height_range=max_height_range,
        ramped_height_range_a=ramped_height_range_a,
        ramped_height_range_b=ramped_height_range_b,
        ramped_height_range_c=ramped_height_range_c,
    )
    return util_preamble


def average_map_kernel(
    resolution,
    width,
    height,
    sensor_noise_factor,
    min_valid_distance,
    max_height_range,
    cleanup_cos_thresh,
    ramped_height_range_a,
    ramped_height_range_b,
    ramped_height_range_c,
):
    """Create kernel for averaging height map"""
    kernel = cp.ElementwiseKernel(
        in_params="raw int32 l_id_in,raw U p, raw U center_x, raw U center_y",
        out_params="raw U newmap, raw U map, raw U pop_array",
        preamble=map_utils(
            resolution,
            width,
            height,
            sensor_noise_factor,
            min_valid_distance,
            max_height_range,
            ramped_height_range_a,
            ramped_height_range_b,
            ramped_height_range_c,
        ),
        operation=string.Template(
            """
            U x = p[i * 3];
            U y = p[i * 3 + 1];
            U z = -1.0 * p[i * 3 + 2];
            int l_id=l_id_in[0];
            int idx = get_idx(x, y, center_x[0], center_y[1]);
            U h = map[get_map_idx(idx, l_id)];
            U new_cnt = newmap[get_map_idx(idx, l_id)];
            pop_array[idx] = 0;
            if (new_cnt > 0) {
                float cnt = new_cnt;
                atomicExch(&map[get_map_idx(idx, l_id)], z);
                atomicAdd(&newmap[get_map_idx(idx, l_id)], 1.0f);
            } else {
                atomicExch(&map[get_map_idx(idx, l_id)], z);
                atomicExch(&newmap[get_map_idx(idx, l_id)], 1.0f);
            }
            """#(h*cnt+z)/(cnt+1)
        ).substitute(
            resolution=resolution,
            width=width,
            height=height,
        ),
        name="average_map_kernel",
    )
    return kernel

def big_cloth_map_complet_kernel(
    resolution,
    width,
    height,
    sensor_noise_factor,
    min_valid_distance,
    max_height_range,
    cleanup_cos_thresh,
    ramped_height_range_a,
    ramped_height_range_b,
    ramped_height_range_c,
):
    big_cloth_map_complet_kernel = cp.ElementwiseKernel(
        in_params="raw int32 l_id, raw U bound,raw U c_resolution, raw V c_width, raw V c_height,raw U p, raw U center_x, raw U center_y",
        out_params="raw U newmap,raw U map, raw U pop_array",
        options=("--device-debug",),
        preamble=map_utils(
            resolution,
            width,
            height,
            sensor_noise_factor,
            min_valid_distance,
            max_height_range,
            ramped_height_range_a,
            ramped_height_range_b,
            ramped_height_range_c,
        ),
        #(h*cnt+z)/(cnt+1.0f);
        operation=string.Template(
            """
            int index=i;
            int ind_x = index / ${width};
            int ind_y = index % ${height};
            
            
            U left_down_x = center_x[0] - ${width}*0.5*${resolution};
            U left_down_y = center_y[1] - ${height}*0.5*${resolution};
            float x=ind_x*${resolution}+left_down_x;
            float y=ind_y*${resolution}+left_down_y;
            
            
            if ((x>bound[0] and x<bound[2]) and (y>bound[1] and y<bound[3])){
                int c_id_x=(x-bound[0])/c_resolution[0];
                int c_id_y=(y-bound[1])/c_resolution[0];
                int c_id=c_id_x*c_width[0]+c_id_y;
                float value=-1 * p[c_id*3+2];
                atomicExch(&map[get_map_idx(i, l_id[0])], value);
                atomicExch(&newmap[get_map_idx(i, l_id[0])] , 1.0f);
            }
            """
            #int x_grid = index*${resolution}; atomicExch(&map[get_map_idx(i, 2)], 2);
            # int y_grid = (index*${resolution}-id_x*${width});
        ).substitute(resolution=resolution,
        width=width,
        height=height,),
        name="big_cloth_map_complet_kernel",
    )
    return big_cloth_map_complet_kernel

class CSF_pcl_gpu(FusionBase):
    """
    GPU-Accelerated CSF Point Cloud Ground Filtering for Elevation Mapping.
    
    OPTIMIZED VERSION - All operations stay on GPU for maximum performance.
    
    Key optimizations:
    - No CPU-GPU transfers during filtering
    - No Python loops for classification
    - Embedded CSF_GPU_Fast for zero import overhead
    """
    
    def __init__(self, params, csf_gpu_id: int = 0,
                 csf_bSloopSmooth=False, 
                 csf_cloth_resolution=0.5,
                 csf_class_threshold=0.05,
                 csf_time_step=0.65,
                 csf_rigidness=1,
                 csf_interations=20,
                 lenght_view=60.,
                 csf_gravity=-0.2,
                 decimate=0,
                 *args, **kwargs):
        self.name = "pointcloud_csf_segmentation_gpu"
        gpu_id = csf_gpu_id
        
        # Map parameters
        self.cell_n = params.cell_n
        self.resolution = params.resolution
        self.data_type = params.data_type
        
        # CSF parameters
        self.csf_bSloopSmooth = csf_bSloopSmooth
        self.csf_cloth_resolution = csf_cloth_resolution
        self.csf_time_step = csf_time_step
        self.csf_class_threshold = csf_class_threshold
        self.csf_rigidness = csf_rigidness
        self.csf_interations = csf_interations
        self.csf_gravity = csf_gravity
        self.csf_max_diff_factor = 0.01
        
        # Initialize GPU CSF (embedded, no import overhead)
        self.csf = CSF_GPU(gpu_id=gpu_id)
        self.csf.params.bSloopSmooth = self.csf_bSloopSmooth
        self.csf.params.cloth_resolution = self.csf_cloth_resolution
        self.csf.params.time_step = self.csf_time_step
        self.csf.params.class_threshold = self.csf_class_threshold
        self.csf.params.rigidness = self.csf_rigidness
        self.csf.params.interations = self.csf_interations
        self.csf.params.gravity = self.csf_gravity
        self.csf.params.factor_max_diff = self.csf_max_diff_factor
        
        # Storage for cloth data
        self.array_cloth = cp.zeros((2, 3), dtype=self.data_type)
        self.cloth_local_t = cp.array([0.0, 0.0, 0.0], dtype=self.data_type)
        self.cloth_r = cp.eye(3, dtype=self.data_type)
        
        # Base frame offset
        self.base_frame_in_pc_frame = cp.array([0.670, -0.000, -0.840], dtype=self.data_type) # lidar
        #self.base_frame_in_pc_frame = cp.array([-0.039, 0.954, -0.733], dtype=self.data_type) # mono

        # Preprocess points options
        self.decimate=decimate
        
        # Timing and state
        self.last_time_update = time()
        self.current_R = cp.eye(3, dtype=self.data_type)
        self.current_T = cp.zeros(3, dtype=self.data_type)
        self.shape_crop = lenght_view
        
        # Statistics
        self.processing_times = []
        
        # Create averaging kernel
        self.average_map = average_map_kernel(
            self.resolution,
            self.cell_n,
            self.cell_n,
            params.sensor_noise_factor,
            params.min_valid_distance,
            params.max_height_range,
            params.cleanup_cos_thresh,
            params.ramped_height_range_a,
            params.ramped_height_range_b,
            params.ramped_height_range_c,
        )
    
    def _preprocess_points(self, points: cp.ndarray, R: cp.ndarray, t: cp.ndarray) -> cp.ndarray:
        """
        Preprocess point cloud: decimate, transform, and crop.
        All operations on GPU.
        """
        # Decimate (keep every 4th point)
        if self.decimate:
            decimation_mask = cp.arange(points.shape[0]) % int(self.decimate) == 0
            points = points[decimation_mask]
        
        # Remove zero points
        non_zero_mask = cp.sum(points != 0, axis=1) == 3
        points = points[non_zero_mask]
        
        if len(points) == 0:
            return points
        
        # Transform to robot frame
        points = (R @ (points - self.base_frame_in_pc_frame).T).T
        # points = points * cp.array([1.,1.,-1.])
         
        # Crop to view range
        shape_crop = self.shape_crop
        
        # X bounds
        mask = (points[:, 0] > np.float32(-shape_crop)) & (points[:, 0] < np.float32(shape_crop))
        points = points[mask]
        
        if len(points) == 0:
            return points
        
        # Y bounds
        mask = (points[:, 1] > np.float32(-shape_crop)) & (points[:, 1] < np.float32(shape_crop))
        points = points[mask]
        
        if len(points) == 0:
            return points
        
        # Z bounds
        mask = (points[:, 2] > -1.7) & (points[:, 2] < 1.7)
        points = points[mask]

        # mask = (cp.sum((points < cp.array([shape_crop,shape_crop,2.],dtype=self.data_type)) & 
        #        (points > cp.array([-shape_crop,-shape_crop,-2.],dtype=self.data_type)) ,1)==3)
        # points = points[mask]
        
        return points
    
    def __call__(self, points_all, R, t, pcl_ids, layer_ids, elevation_map, 
                 semantic_map, new_map, *args):
        """
        Process point cloud with GPU-accelerated CSF.
        """
        self.base_frame_in_pc_frame=cp.array([t[6], t[7], t[8]])
        time_beg = time()
        
        # Extract translation
        local_t = cp.array([t[0] + t[3], t[1] + t[4], t[2] + t[5]], dtype=self.data_type)
        
        # Preprocess points (all on GPU)
        points = points_all[:, :3].astype(self.data_type)
        processed_points = self._preprocess_points(points, R, local_t)
        
        if len(processed_points) < 100:
            # print(f"Warning: Too few points after preprocessing ({len(processed_points)})")
            return
        
        # Run GPU CSF (entirely on GPU, no transfers!) 
        csf_start = time()
        cloth_positions, ground_mask, non_ground_mask = self.csf.filter_points_gpu(processed_points)
        csf_time = (time() - csf_start) * 1e-9
        
        # Update cloth data (all on GPU)
        self.array_cloth = cloth_positions
        self.cloth_local_t = local_t
        self.cloth_r = R
        self.current_R = R
        self.current_T = local_t
        self.last_time_update = time()
        
        # Prepare cloth for elevation map update
        new_array = self.array_cloth.copy()
        new_array[:, 0] += self.cloth_local_t[0]
        new_array[:, 1] += self.cloth_local_t[1]
        
        # Update elevation map
        index_array = cp.ones((self.cell_n, self.cell_n), dtype=self.data_type)
        
        if self.resolution>=self.csf_cloth_resolution:
            self.average_map(
                layer_ids,
                new_array,
                local_t,
                local_t,
                new_map,
                semantic_map,
                index_array,
                size=new_array.shape[0],
            )
        else:
            bound=cp.array([new_array[0,0],new_array[0,1],new_array[-1,0],new_array[-1,1]])
            c_resolution=cp.array([self.csf_cloth_resolution],dtype=self.data_type,)
            c_width=cp.array([abs(bound[2]-bound[0])])
            c_height=cp.array([abs(bound[3]-bound[1])])
            self.big_cloth_map_complet(
                layer_ids,
                bound,
                c_resolution, 
                c_width, 
                c_height,
                new_array, 
                local_t,
                local_t,
                new_map,
                semantic_map,
                index_array,
                size=new_map.shape[1]*new_map.shape[2],
            )
        
        # Set NaN for empty cells
        semantic_map[layer_ids, :, :] = cp.where(
            semantic_map[layer_ids, :, :] == 0.,
            cp.nan,
            semantic_map[layer_ids, :, :]
        )
        
        total_time = (time() - time_beg) * 1e-9
        self.processing_times.append(total_time)
        
        # Uncomment for debugging:
        # print(f"GPU CSF: {len(processed_points)} pts, CSF={csf_time*1000:.1f}ms, Total={total_time*1000:.1f}ms")
    
    def get_ground_points(self, points: cp.ndarray, R: cp.ndarray, t: cp.ndarray) -> Tuple[cp.ndarray, cp.ndarray]:
        """Get ground and non-ground points separately."""
        processed = self._preprocess_points(points, R, t)
        if len(processed) < 100:
            return cp.zeros((0, 3), dtype=self.data_type), processed
        
        _, ground_mask, _ = self.csf.filter_points_gpu(processed)
        
        return processed[ground_mask], processed[~ground_mask]
    
    def get_cloth_mesh(self) -> cp.ndarray:
        """Get the current cloth mesh vertices."""
        return self.array_cloth
    
    def get_average_processing_time(self) -> float:
        """Get average processing time in seconds."""
        if not self.processing_times:
            return 0.0
        return float(np.mean(self.processing_times[-100:]))
    
    def __del__(self):
        """Cleanup resources."""
        pass


# Alias for backward compatibility
#CSF_pcl = CSF_pcl_GPU


if __name__ == "__main__":
    """Test the GPU CSF implementation"""
    import time as time_module
    
    print("Testing GPU CSF Point Cloud Fusion (Optimized)")
    print("=" * 60)
    
    # Create mock parameters
    class MockParams:
        cell_n = 256
        resolution = 0.1
        data_type = cp.float32
        csf_bSloopSmooth = False
        csf_cloth_resolution = 0.5
        csf_time_step = 0.65
        csf_class_threshold = 0.5
        csf_rigidness = 3
        csf_interations = 100  # Reduced for speed
        sensor_noise_factor = 0.01
        min_valid_distance = 0.5
        max_height_range = 2.0
        cleanup_cos_thresh = 0.7
        ramped_height_range_a = 0.3
        ramped_height_range_b = 1.0
        ramped_height_range_c = 0.2
        lenght_view = 10.0
    
    params = MockParams()
    
    # Create CSF processor
    csf_processor = CSF_pcl_GPU(params)
    
    # Generate synthetic point cloud
    np.random.seed(42)
    n_points = 50000
    
    # Ground points
    x = np.random.uniform(-10, 10, n_points)
    y = np.random.uniform(-10, 10, n_points)
    z = 0.1 * np.sin(x * 0.3) + 0.1 * np.cos(y * 0.3) + np.random.normal(0, 0.05, n_points)
    
    # Add objects
    n_objects = 5000
    obj_x = np.random.uniform(-8, 8, n_objects)
    obj_y = np.random.uniform(-8, 8, n_objects)
    obj_z = np.random.uniform(0.5, 3, n_objects)
    
    x = np.concatenate([x, obj_x])
    y = np.concatenate([y, obj_y])
    z = np.concatenate([z, obj_z])
    
    # Create point cloud
    points = np.column_stack([x, y, z, np.ones_like(x)]).astype(np.float32)
    points_gpu = cp.asarray(points)
    
    print(f"Input points: {len(points)}")
    
    # Create mock transforms
    R = cp.eye(3, dtype=cp.float32)
    t = cp.zeros(6, dtype=cp.float32)
    
    # Create mock maps
    elevation_map = cp.zeros((3, params.cell_n, params.cell_n), dtype=cp.float32)
    semantic_map = cp.zeros((3, params.cell_n, params.cell_n), dtype=cp.float32)
    new_map = cp.zeros((3, params.cell_n, params.cell_n), dtype=cp.float32)
    
    # Warmup
    print("Warmup...")
    for _ in range(3):
        csf_processor(points_gpu, R, t, None, None, elevation_map, semantic_map, new_map)
    
    # Benchmark
    n_iterations = 20
    times = []
    
    print(f"Running {n_iterations} iterations...")
    
    for i in range(n_iterations):
        # Reset maps
        new_map.fill(0)
        
        start = time_module.time()
        csf_processor(points_gpu, R, t, None, None, elevation_map, semantic_map, new_map)
        cp.cuda.Stream.null.synchronize()  # Ensure GPU work is done
        elapsed = time_module.time() - start
        times.append(elapsed)
        print(f"  Iteration {i+1}: {elapsed*1000:.2f} ms")
    
    avg_time = np.mean(times)
    std_time = np.std(times)
    min_time = np.min(times)
    max_time = np.max(times)
    
    print(f"\nPerformance Summary:")
    print(f"  Average: {avg_time*1000:.2f} ± {std_time*1000:.2f} ms")
    print(f"  Min: {min_time*1000:.2f} ms")
    print(f"  Max: {max_time*1000:.2f} ms")
    print(f"  Throughput: {n_points / avg_time / 1000:.1f}k points/sec")
    
    print("\nTest completed successfully!")
