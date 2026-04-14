"""
GPU-Accelerated Cloth Simulation Filter (CSF) for Point Cloud Ground Filtering

This module implements the CSF algorithm on GPU using CuPy, providing significant
speedup over the CPU-based implementation.

The CSF algorithm simulates a piece of cloth falling onto an inverted point cloud
to extract ground points. Key steps:
1. Invert the point cloud (flip Z-axis) -> the ground becomes the highest surface
2. Create a cloth grid above the inverted surface -> resolution determines the granularity of the grid
3. Build height field -> for each cloth cell, find the MINIMUM Z of points in that cell
4. Init cloth above surface
5. Simulate cloth falling under gravity with constraints
6. Generate cloth particle positions
7. Compare final cloth positions with point cloud to classify ground/non-ground

Reference:
Zhang W, Qi J, Wan P, Wang H, Xie D, Wang X, Yan G. 
"An Easy-to-Use Airborne LiDAR Data Filtering Method Based on Cloth Simulation"
Remote Sensing. 2016; 8(6):501.
"""

import cupy as cp
import numpy as np
from typing import Tuple, Optional

class CSFParams:
    """Parameters for the Cloth Simulation Filter"""
    def __init__(self):
        self.bSloopSmooth: bool = False
        self.cloth_resolution: float = 0.5
        self.time_step: float = 0.65
        self.class_threshold: float = 0.5
        self.rigidness: int = 3
        self.interations: int = 500
        self.gravity: float = 0.2
        self.max_delta_height: float = 4.0
        self.fratio_movable_fix : float = 1.0

class CSF_GPU:
    """
    Fast GPU-accelerated Cloth Simulation Filter.
    
    Optimized for real-time point cloud ground filtering using
    grid-based spatial acceleration and unified CUDA kernels.
    
    ALL operations stay on GPU - no CPU-GPU transfers during filtering.
    """
    
    def __init__(self, gpu_id: int = 0):
        self.gpu_id = gpu_id
        self.params = CSFParams()
        self._compile_kernels()
    
    def _compile_kernels(self):
        """Compile optimized CUDA kernels"""
        
        # Grid-based height computation kernel
        self.grid_height_kernel = cp.RawKernel(r'''
        extern "C" __global__
        void compute_grid_heights(
            const float* points, int num_points,
            float* grid_heights, int* grid_counts,
            float min_x, float min_y, float resolution,
            int rows, int cols)
        {
            int idx = blockIdx.x * blockDim.x + threadIdx.x;
            if (idx >= num_points) return;
            
            float px = points[idx * 3];
            float py = points[idx * 3 + 1];
            float pz = points[idx * 3 + 2];
            
            int col = (int)((px - min_x) / resolution);
            int row = (int)((py - min_y) / resolution);
            
            if (col >= 0 && col < cols && row >= 0 && row < rows) {
                int grid_idx = row * cols + col;
                float old_height = grid_heights[grid_idx];
                while (pz < old_height) {
                    old_height = atomicExch(&grid_heights[grid_idx], 
                                           fminf(old_height, pz));
                }
                atomicAdd(&grid_counts[grid_idx], 1);
            }
        }
        ''', 'compute_grid_heights')
        
        # NEW: Compute internal forces from neighbors (spring-like cloth forces)
        self.compute_forces_kernel = cp.RawKernel(r'''
        extern "C" __global__
        void compute_internal_forces(
            const float* cloth_z, float* forces, int* grid_counts,
            const bool* is_movable, float resolution, float rigidness, float ratio,
            int rows, int cols)
        {
            int idx = blockIdx.x * blockDim.x + threadIdx.x;
            int num_particles = rows * cols;
            if (idx >= num_particles) return;
            if (!is_movable[idx]) return;
            int row = idx / cols;
            int col = idx % cols;
            float z = cloth_z[idx];
            float force = 0.0f;
            int count = 0;
            
            // Check all 4 neighbors and apply spring-like forces
            // This maintains cloth structure and allows smooth draping
            int neighbor_offsets[4] = {-1, 1, -cols, cols};
            bool neighbor_valid[4] = {
                col > 0,           // left
                col < cols - 1,    // right
                row > 0,           // up
                row < rows - 1     // down
            };
            
            for (int i = 0; i < 4; i++) {
                
                if (!neighbor_valid[i]) continue;
                int nidx = idx + neighbor_offsets[i];
                if (grid_counts[nidx]<1) continue;
                
                float neighbor_z = cloth_z[nidx];
                
                // Spring force: tries to minimize height difference
                // This is what was missing - forces from ALL neighbors
                float diff = neighbor_z - z;

                // use rigidness to influance forces as csf original implementation
                float tmp_force = diff * rigidness;
                if (is_movable[nidx]){
                   tmp_force *= ratio; // magic number wheight on moving particules
                }
                force += tmp_force;  
                count++;
            }
            
            forces[idx] = float(count) *force; // as csf original implementation
        }
        ''', 'compute_internal_forces')
        
        # NEW: Apply forces and handle collisions (separated from force computation)
        self.update_positions_kernel = cp.RawKernel(r'''
        extern "C" __global__
        void update_positions(
            float* cloth_z, float* cloth_old_z, float* cloth_vel_z, const float* forces,
            const float* grid_heights, const int* grid_counts,
            bool* is_movable, float time_step, float gravity,
            int rows, int cols, int rigidness, int iteration)
        {
            int idx = blockIdx.x * blockDim.x + threadIdx.x;
            int num_particles = rows * cols;
            if (idx >= num_particles) return;
            if (!is_movable[idx]) return;
            
            // Apply gravity + internal forces
            float total_force = gravity + forces[idx];
            
            cloth_vel_z[idx] = total_force * time_step * time_step; // minimize forces by time step as csf original implementation
            
            // Update position with dump value 0.01
            float new_z = cloth_z[idx] + 0.01* (cloth_z[idx] - cloth_old_z[idx])+ cloth_vel_z[idx] * time_step ; // minimize forces by time step as csf original implementation
            
            // Collision detection with terrain
            if (grid_counts[idx] > 0) {
                float ground_h = grid_heights[idx];
                if (new_z <= ground_h) {
                    new_z = ground_h;
                    cloth_vel_z[idx] = 0.0f;
                    // Make particle immovable because it reach ground
                    is_movable[idx] = false;
                    // Make particle immovable based on rigidness
                    //if ((iteration + 1) % rigidness == 0) {
                    //    is_movable[idx] = false;
                    //}
                }
            }

            // prevent up deplacement 
            if (cloth_old_z[idx] > new_z ){
               cloth_old_z[idx]=cloth_z[idx];
               cloth_z[idx] = new_z;
            } else {
               cloth_old_z[idx]=cloth_z[idx];
            }
            
        }
        ''', 'update_positions')
        
        # NEW: Terrain smoothing kernel
        self.smooth_terrain_kernel = cp.RawKernel(r'''
        extern "C" __global__
        void smooth_terrain(
            float* cloth_z, const bool* is_movable,
            int rows, int cols, int num_iterations)
        {
            int idx = blockIdx.x * blockDim.x + threadIdx.x;
            int num_particles = rows * cols;
            if (idx >= num_particles) return;
            
            // Only smooth movable particles (ground-touching ones are fixed)
            if (!is_movable[idx]) return;
            
            int row = idx / cols;
            int col = idx % cols;
            
            // Average with valid neighbors
            float sum = cloth_z[idx];
            int count = 1;
            
            if (col > 0) {
                sum += cloth_z[idx - 1];
                count++;
            }
            if (col < cols - 1) {
                sum += cloth_z[idx + 1];
                count++;
            }
            if (row > 0) {
                sum += cloth_z[idx - cols];
                count++;
            }
            if (row < rows - 1) {
                sum += cloth_z[idx + cols];
                count++;
            }
            
            cloth_z[idx] = sum / count;
        }
        ''', 'smooth_terrain')
    
    def filter_points_gpu(self, points: cp.ndarray) -> Tuple[cp.ndarray, cp.ndarray, cp.ndarray]:
        """
        Perform CSF filtering entirely on GPU with proper terrain handling.
        
        Args:
            points: CuPy array of shape (N, 3) with point coordinates (already on GPU)
            
        Returns:
            Tuple of (cloth_positions, ground_mask, non_ground_mask) - all on GPU
        """
        with cp.cuda.Device(self.gpu_id):
            # 1. Invert the point cloud (flip Z-axis) -> the ground becomes the highest surface
            # Invert Z for cloth simulation (work on copy)
            points_inv = points.copy()
            points_inv[:, 2] = -points_inv[:, 2]
            
            # 2. Create a cloth grid above the inverted surface -> resolution determines the granularity of the grid
            # Get bounds (these are small scalar operations)
            min_x = float(cp.min(points_inv[:, 0]))
            max_x = float(cp.max(points_inv[:, 0]))  
            min_y = float(cp.min(points_inv[:, 1]))
            max_y = float(cp.max(points_inv[:, 1]))
            max_z = float(cp.max(points_inv[:, 2]))
            
            # Calculate grid dimensions based on cloth_resolution
            resolution = np.float32(self.params.cloth_resolution)
            cols = int(np.ceil((max_x - min_x) / resolution)) + 1
            rows = int(np.ceil((max_y - min_y) / resolution)) + 1
            num_particles = rows * cols
            
            # 3. Build height field -> for each cloth cell, find the MINIMUM Z of points in that cell
            # Compute grid heights using spatial hashing
            grid_heights = cp.full(num_particles, cp.float32(1e10), dtype=cp.float32)
            grid_counts = cp.zeros(num_particles, dtype=cp.int32)
            
            block_size = 256
            grid_size_pts = (len(points_inv) + block_size - 1) // block_size
            
            self.grid_height_kernel(
                (grid_size_pts,), (block_size,),
                (points_inv, np.int32(len(points_inv)),
                 grid_heights, grid_counts,
                 np.float32(min_x), np.float32(min_y),
                 np.float32(resolution), np.int32(rows), np.int32(cols))
            )
            
            # 4. Init cloth above surface
            # Initialize cloth
            z_range = float(max_z - cp.min(points_inv[:, 2]))
            init_height = 0.5 #max(2.0, z_range + 1.0)  # At least 2m above highest point
            cloth_z = cp.full(num_particles, max_z + init_height, dtype=cp.float32)
            cloth_old_z = cp.full(num_particles, max_z + init_height, dtype=cp.float32)
            cloth_vel_z = cp.zeros(num_particles, dtype=cp.float32)
            is_movable = cp.ones(num_particles, dtype=cp.bool_)
            forces = cp.zeros(num_particles, dtype=cp.float32)
            
            grid_size = (num_particles + block_size - 1) // block_size

            # compute rigidness
            rigidness = 1. - (0.9**self.params.rigidness)

            # set ratio 
            ratio = self.params.fratio_movable_fix
            if (ratio >= 1.0):
                # compute ratio base on rigidness
                ratio =  1. - (0.85**self.params.rigidness)

            # copute limit of iteration
            max_iter = round(self.params.max_delta_height/abs(self.params.gravity * self.params.time_step**3))  

            # 5. Simulate cloth falling under gravity with constraints
            # Run simulation
            for iteration in range(self.params.interations):
                # Step 1: Compute internal forces from neighbors
                self.compute_forces_kernel(
                    (grid_size,), (block_size,),
                    (cloth_z, forces, grid_counts, is_movable, 
                     np.float32(resolution),np.float32(rigidness), np.float32(ratio),
                     np.int32(rows), np.int32(cols))
                )
                
                # Step 2: Update positions with forces + gravity + collisions
                self.update_positions_kernel(
                    (grid_size,), (block_size,),
                    (cloth_z, cloth_old_z, cloth_vel_z, forces,
                     grid_heights, grid_counts, is_movable,
                     np.float32(self.params.time_step),
                     np.float32(self.params.gravity),
                     np.int32(rows), np.int32(cols),
                     np.int32(self.params.rigidness),
                     np.int32(iteration))
                )
                
                # Early termination
                if iteration >= max_iter-1:
                    break
                if cp.sum(is_movable) == 0:
                    break
            
            if self.params.bSloopSmooth:
                smooth_iterations = 5
                for _ in range(smooth_iterations):
                    self.smooth_terrain_kernel(
                        (grid_size,), (block_size,),
                        (cloth_z, is_movable,
                         np.int32(rows), np.int32(cols),
                         np.int32(smooth_iterations))
                    )
            
            # 6. Generate cloth particle positions
            # Generate cloth positions (on GPU)
            indices = cp.arange(num_particles, dtype=cp.int32)
            cloth_x = (min_x + (indices % cols).astype(cp.float32) * resolution).astype(cp.float32)
            cloth_y = (min_y + (indices // cols).astype(cp.float32) * resolution).astype(cp.float32)
            cloth_positions = cp.stack([cloth_x, cloth_y, cloth_z], axis=1)
            
            # 7. Compare final cloth positions with point cloud to classify ground/non-ground
            # Classify points (entirely on GPU - no Python loops!)
            col_idx = cp.clip(((points_inv[:, 0] - min_x) / resolution).astype(cp.int32), 0, cols - 1)
            row_idx = cp.clip(((points_inv[:, 1] - min_y) / resolution).astype(cp.int32), 0, rows - 1)
            cloth_idx = row_idx * cols + col_idx
            cloth_heights = cloth_z[cloth_idx]
            
            height_diff = points_inv[:, 2] - cloth_heights
            ground_mask = cp.abs(height_diff) < np.float32(self.params.class_threshold)
            non_ground_mask = ~ground_mask
            
            return cloth_positions, ground_mask, non_ground_mask


# Example usage
if __name__ == "__main__":
    # Create sample terrain with elevation changes
    np.random.seed(42)
    x = np.random.uniform(-10, 10, 10000)
    y = np.random.uniform(-10, 10, 10000)
    # Simulate hilly terrain
    z = np.sin(x * 0.5) * np.cos(y * 0.5) * 2.0 + np.random.normal(0, 0.05, 10000)
    points_cpu = np.column_stack([x, y, z]).astype(np.float32)
    
    # Transfer to GPU
    points_gpu = cp.asarray(points_cpu)
    
    # Create CSF instance with terrain-friendly parameters
    csf = CSF_GPU(gpu_id=0)
    csf.params.bSloopSmooth = True      # Enable smoothing for terrain
    csf.params.cloth_resolution = 0.3    # Coarser for large terrain features
    csf.params.time_step = 0.65
    csf.params.class_threshold = 0.1     # Reasonable threshold
    csf.params.rigidness = 1             # Flexible for slopes
    csf.params.interations = 500
    csf.params.gravity = 0.2
    
    # Filter
    cloth_pos, ground_mask, non_ground_mask = csf.filter_points_gpu(points_gpu)
    
    print(f"Total points: {len(points_gpu)}")
    print(f"Ground points: {cp.sum(ground_mask)}")
    print(f"Non-ground points: {cp.sum(non_ground_mask)}")
    print(f"Ground percentage: {100.0 * cp.sum(ground_mask) / len(points_gpu):.2f}%")