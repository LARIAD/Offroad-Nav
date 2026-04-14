
# import multiprocessing.managers
from multiprocessing.managers import SharedMemoryManager
# from multiprocessing.shared_memory import SharedMemory
import multiprocessing.shared_memory
import cupy as cp
import numpy as np
import string
import CSF
from scipy.spatial.transform import Rotation  

import ctypes
from .fusion_manager import FusionBase
import os, sys
from time import time_ns as time
from time import sleep
import multiprocessing
from typing import Tuple
import threading
#import ros_numpy
from sensor_msgs.msg import PointCloud2

MAX_NB_THREAD=6
#LOCK = threading.Lock()
# class SharedObjAA(object):
#     def __init__(self,dat_type):
#         self.T=np.zeros((3),dtype=dat_type)
#         self.R=np.zeros((3,3),dtype=dat_type)
#         self.points=np.zeros((1,3),dtype=dat_type)
#         # print ("tytytytytytytytytytytytytty: ",dat_type)

#         #self.time_begin=time()
        
#         self.crop=[[50.,-50],[50.,-50],[0.,-10.]]#[[1024.,0],[128.,0],[0.,-10.]]#
#         self.cloth_resolution=0.25
#         self.size_for_cloth=[(self.crop[0][0]-self.crop[0][1])/self.cloth_resolution,(self.crop[1][0]-self.crop[1][1])/self.cloth_resolution]
#         self.cloth=np.zeros(int(self.size_for_cloth[0]*self.size_for_cloth[1]*3),dtype=dat_type)
#         self.SHARED_DATA_DTYPE = self.cloth.dtype
#         self.SHARED_DATA_SHAPE = self.cloth.shape
#         self.SHARED_DATA_NBYTES = self.cloth.nbytes
#         self.last_time_update=time()
#         #self.arr = multiprocessing.Array(ctypes.c_float, self.cloth.shape[0])
#     def update(self,T,R,points):
#         self.T=T
#         self.R=R
#         self.points=points
#         self.last_time_update=time()
#         #self.size_for_cloth=[(self.crop[0][0]-self.crop[0][1])/self.cloth_resolution,(self.crop[1][0]-self.crop[1][1])/self.cloth_resolution]
#         #self.time_begin=time_begin

# def create_np_array_from_shared_mem(
#     shared_mem: multiprocessing.shared_memory.SharedMemory, shared_data_dtype: np.dtype, shared_data_shape: Tuple[int, ...]
# ) -> np.ndarray:
#     arr = np.frombuffer(shared_mem.buf, dtype=shared_data_dtype)
#     arr = arr.reshape(shared_data_shape)
#     return arr 

# class Csf_threading(multiprocessing.Process):
#     def __init__(self, shared, *args, **kwargs):
#         super(Csf_threading,self).__init__(*args, **kwargs)
#         self.shared = shared
#         #self.lock = lock  ,lock

#     def run(self):
#         #LOCK.acquire()
#         new_pcl=(self.shared.R).dot(self.shared.points.T).T
#         mask0=(np.sum(new_pcl != [0,0,0],1)==3)
#         tempo=new_pcl[mask0,:]
#         #print("shapes pointcloud 1: ",new_pcl, " lllllllll : ",points_all)
#         #print("shapes pointcloud 1: ",tempo.shape," shape cupyarrconvert: ",(cp.asnumpy(R)), " original: ",R)
#         #new_pcl=(cp.asnumpy(R)).dot(tempo.T).T
#         #print("shapes pointcloud 1: ",new_pcl)
#         mask=(np.sum(tempo < [10,10,0.],1)==3)
#         tempo=tempo[mask,:]
#         #print("shapes pointcloud 2: ",tempo.shape)
#         mask=(np.sum((tempo < [0.6,1.5,np.Inf]) & (tempo > [-np.Inf,-1.5,np.Inf]),1)!=3)
#         tempo=tempo[mask,:]
#         # print("shapes pointcloud 3: ",tempo.shape)
#         # mask=(np.sum(tempo < [np.Inf,0.5,0.8],1)!=3)
#         # tempo=tempo[mask,:]
#         mask2=(np.sum(tempo >[-1.5,-10,-10],1)==3)
#         xyz= tempo + self.shared.T

#         newtime=time()
#         #print("end prepare: ",newtime-self.shared.time_begin)
        
#         # print("shapes pointcloud bcsf: ",xyz.shape)
#         # print("ouiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiii")
#         csf = CSF.CSF()
#         #print("pcl_size: ",t[0]," ",t[1]," ",t[2])
#         # prameter settings
#         csf.params.bSloopSmooth = False
#         csf.params.cloth_resolution = 0.25
#         csf.params.time_step=0.65#1.5
#         csf.params.class_threshold=0.05
#         csf.params.rigidness=1
#         csf.params.interations=10 
#         csf.setPointCloud(xyz)#np.c_[xyz[:, 2], xyz[:, 1], -1*xyz[:, 0]])
#         ground = CSF.VecInt()  # a list to indicate the index of ground points after calculation
#         non_ground = CSF.VecInt()
        
#         csf.do_filtering(ground, non_ground,False)
#         csf.setPointCloud(np.c_[xyz[ground, 0], xyz[ground, 1], -xyz[ground, 2]])
#         csf.params.time_step=0.5#1.5
#         ground2 = CSF.VecInt()
#         non_ground2 = CSF.VecInt() 
        
#         csf.do_filtering(ground2, non_ground2,False)
#         #print("shape vector: ",kl.shape)
#         newtime1=time()
#         print("end csf: ",newtime1-newtime)
#         newtime=newtime1
#         print("uuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuu")
#         #self.shared.points= (xyz[ground, :])[ground2]
#         arr = create_np_array_from_shared_mem(shared_mem, self.shared.SHARED_DATA_DTYPE, self.shared.SHARED_DATA_SHAPE)
#         arraycloth=np.array( csf.do_cloth_export() )
#         # self.shared.cloth=
#         #self.shared.cloth=np.reshape(self.shared.cloth,(int(self.shared.cloth.shape[0]/3),3))
#         #LOCK.release()
# def do_CSF(list_shared,shared,connection):
    
#     new_pcl=(shared.R).dot(shared.points.T).T
#     mask0=(np.sum(new_pcl != [0.,0.,0.],1)==3)
#     tempo=new_pcl[mask0,:]
#     #print("shapes pointcloud 1: ",new_pcl, " lllllllll : ",points_all)
#     #print("shapes pointcloud 1: ",tempo.shape," shape cupyarrconvert: ",(cp.asnumpy(R)), " original: ",R)
#     #new_pcl=(cp.asnumpy(R)).dot(tempo.T).T
#     #print("shapes pointcloud 1: ",new_pcl)
#     mask=(np.sum(tempo < [10,10,0.],1)==3)
#     tempo=tempo[mask,:]
#     #print("shapes pointcloud 2: ",tempo.shape)
#     mask=(np.sum((tempo < [0.6,1.5,np.Inf]) & (tempo > [-np.Inf,-1.5,np.Inf]),1)!=3)
#     tempo=tempo[mask,:]
#     # print("shapes pointcloud 3: ",tempo.shape)
#     # mask=(np.sum(tempo < [np.Inf,0.5,0.8],1)!=3)
#     # tempo=tempo[mask,:]
#     mask2=(np.sum(tempo >[-1.5,-10,-10],1)==3)
#     tempo=tempo[mask2,:]
#     xyz= tempo + shared.T
#     newtime=time()
#     #print("end prepare: ",newtime-self.shared.time_begin)
        
#     # print("shapes pointcloud bcsf: ",xyz.shape)
#     # print("ouiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiii")
#     csf = CSF.CSF()
#     #print("pcl_size: ",t[0]," ",t[1]," ",t[2])
#     # prameter settings
#     csf.params.bSloopSmooth = False
#     csf.params.cloth_resolution = 0.25
#     csf.params.time_step=0.65#1.5
#     csf.params.class_threshold=0.05
#     csf.params.rigidness=1
#     csf.params.interations=25 
#     csf.setPointCloud(xyz)#np.c_[xyz[:, 2], xyz[:, 1], -1*xyz[:, 0]])
#     ground = CSF.VecInt()  # a list to indicate the index of ground points after calculation
#     non_ground = CSF.VecInt()
    
#     csf.do_filtering(ground, non_ground,False)
#     csf.setPointCloud(np.c_[xyz[ground, 0], xyz[ground, 1], -xyz[ground, 2]])
#     csf.params.time_step=0.5#1.5
#     ground2 = CSF.VecInt()
#     non_ground2 = CSF.VecInt() 
    
#     csf.do_filtering(ground2, non_ground2,False)
#     #print("shape vector: ",kl.shape)
#     newtime1=time()
#     print("end csf: ",newtime1-newtime)
#     newtime=newtime1
#     print("uuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuuu")
#     #self.shared.points= (xyz[ground, :])[ground2]
#     shared.cloth=np.array( csf.do_cloth_export() )
#     #shared_arr=create_np_array_from_shared_mem(shared_mem, shared_data_dtype, shared_data_shape)
#     list_shared[:]=shared.cloth.tolist()
#     connection.send(["end job"])
#     #queue.put(shared)
def fromeulertorotationmatrix(theta1, theta2, theta3, order='xyz'):
    """
    input
        theta1, theta2, theta3 = rotation angles in rotation order (degrees)
        oreder = rotation order of x,y,z　e.g. XZY rotation -- 'xzy'
    output
        3x3 rotation matrix (numpy array)
    """
    c1 = np.cos(theta1  )
    s1 = np.sin(theta1 )
    c2 = np.cos(theta2  )
    s2 = np.sin(theta2  )
    c3 = np.cos(theta3  )
    s3 = np.sin(theta3  )

    
    if order=='xyz':
        matrix=np.array([[c2*c3, -c2*s3, s2],
                         [c1*s3+c3*s1*s2, c1*c3-s1*s2*s3, -c2*s1],
                         [s1*s3-c1*c3*s2, c3*s1+c1*s2*s3, c1*c2]])
  
    return matrix

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

def max_map_kernel(
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
    max_map_kernel = cp.ElementwiseKernel(
        in_params="raw int32 l_id, raw B p, raw U center_x, raw U center_y",
        out_params="raw U newmap,raw U map, raw U pop_array",
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
            U x = p[i * 3 ];
            U y = p[i * 3 + 1];
            U z = -1.0 * p[i * 3 + 2];
            int idx = get_idx(x,y,center_x[0],center_y[1]);
            if (is_inside(get_map_idx(idx,l_id[0]))){
               
               atomicMax(&map[get_map_idx(idx, l_id[0])],z);

            }
            """
        ).substitute(resolution=resolution,
        width=width,
        height=height,),
        name="max_map_kernel",
    )
    return max_map_kernel

            #    if (z > h) {

                  
            #       map[get_map_idx(idx, 1)]=z;
            #    }
# U h = map[get_map_idx(idx, 1)];
#                U new_cnt = newmap[get_map_idx(idx, 1)];
#                pop_array[idx]=z;

def max_map_atomic_kernel(
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
    max_map_atomic_kernel = cp.ElementwiseKernel(
        in_params="raw B p, raw U center_x, raw U center_y",
        out_params="raw U newmap,raw U map, raw U pop_array",
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
            U x = p[i * 3 ];
            U y = p[i * 3 + 1];
            U z = -1.0 * p[i * 3 + 2];
            int idx = get_idx(x,y,center_x[0],center_y[1]);
            if (is_inside(get_map_idx(idx,1))){
               U h = map[get_map_idx(idx, 1)];
               U new_cnt = newmap[get_map_idx(idx, 1)];
               pop_array[idx]=z;
               atomicMax(&newmap[get_map_idx(idx, 1)],z);
               
               
            }
            """
        ).substitute(resolution=resolution,
        width=width,
        height=height,),
        name="max_map_atomic_kernel",
    )
    return max_map_atomic_kernel
    # #if (z > h) {

                  
    #               map[get_map_idx(idx, 1)]=z;
    #            }

def max_moy_min_map_kernel(
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
    max_quars_min_map_kernel = cp.ElementwiseKernel(
        in_params="raw B p, raw U center_x, raw U center_y",
        out_params="raw U newmap,raw U map, raw U pop_array",
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
            U x = p[i * 3 ];
            U y = p[i * 3 + 1];
            U z = -1.0 * p[i * 3 + 2];
            int idx = get_idx(x,y,center_x[0],center_y[1]);
            if (is_inside(get_map_idx(idx,1))){
               U new_cnt = newmap[get_map_idx(idx, 0)];
               atomicMin(&newmap[get_map_idx(idx, 3)],z);
               atomicMax(&newmap[get_map_idx(idx, 2)],z);
               
               if (new_cnt > 0) {
                  float cnt=new_cnt;
               
                  atomicExch(&newmap[get_map_idx(idx, 1)],(newmap[get_map_idx(idx, 1)]*cnt+z)/(cnt+1); 
                  atomicAdd(&newmap[get_map_idx(idx, 0)], 1.0f);
               }else{
                  atomicExch(&newmap[get_map_idx(idx, 1)], z);
                  atomicExch(&newmap[get_map_idx(idx, 1)],1.0f);
            }
            }
            """
        ).substitute(resolution=resolution,
        width=width,
        height=height,),
        name="max_moy_min_map_kernel",
    )
    return max_moy_min_map_kernel

def decimate_map_kernel(
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
    decimate_map_kernel = cp.ElementwiseKernel(
        in_params="raw U p",
        out_params="raw U map",
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
            U non_valid = p[i];
            if (non_valid > 0) {               
                map[get_map_idx(i, 0)]= null;
            }
            """
        ).substitute(resolution=resolution,
        width=width,
        height=height,),
        name="decimate_map_kernel",
    )
    return decimate_map_kernel

def set_points_kernel(
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
    set_points_kernel = cp.ElementwiseKernel(
        in_params="raw U center_x, raw U center_y, raw U R, raw U t",
        out_params="raw U p, raw U map, raw T newmap",
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
            U rx = p[i * 3];
            U ry = p[i * 3 + 1];
            U rz = p[i * 3 + 2];
            if (rx != 0. or ry != 0. or rz  != 0.){
                U x = transform_p(rx, ry, rz, R[0], R[1], R[2], t[0]);
                U y = transform_p(rx, ry, rz, R[3], R[4], R[5], t[1]);
                U z = transform_p(rx, ry, rz, R[6], R[7], R[8], t[2]);
                U v = z_noise(rz);
                int idx = get_idx(x, y, center_x[0], center_y[0]);
                if (is_valid(x, y, z, t[0], t[1], t[2])) {
                    if (is_inside(idx)) {
                        atomicMax(&map[get_map_idx(idx, 1)],z);
            
                    }
                        
                    
                }
            }
            """
        ).substitute(
            ray_step=resolution / 2 ** 0.5,
        ),
        name="set_points_kernel",
    )
    return set_points_kernel


class CSF_layer(FusionBase):
    def __init__(self, params, *args, **kwargs):
        self.name = "pointcloud_csf_base_layer"
        self.cell_n = params.cell_n
        self.resolution = params.resolution
        self.data_type=params.data_type
        self.threads={}
        self.indexplus=0
        self.array_cloth=np.zeros((2,3),dtype=self.data_type)
        self.queulist=[]

        self.base_frame_in_pc_frame=cp.array([0.670, -0.000, -0.840]) # lidar
        #self.base_frame_in_pc_frame = cp.array([-0.039, 0.954, -0.733], dtype=self.data_type) # mono

        self.shape_crop=10.
        self.comp_vect_zeros=cp.array([0.,0.,0.],dtype=self.data_type)
        self.comp_vect_xpl=cp.array([self.shape_crop,cp.Inf,cp.Inf],dtype=self.data_type)
        self.comp_vect_xmo=cp.array([-self.shape_crop,-cp.Inf,-cp.Inf],dtype=self.data_type)
        self.comp_vect_ypl=cp.array([cp.Inf,self.shape_crop,cp.Inf],dtype=self.data_type)
        self.comp_vect_ymo=cp.array([-cp.Inf,-self.shape_crop,-cp.Inf],dtype=self.data_type)
        self.comp_vect_zpl=cp.array([cp.Inf,cp.Inf,2.],dtype=self.data_type)
        self.comp_vect_zmo=cp.array([-cp.Inf,-cp.Inf,-2.],dtype=self.data_type)
        self.comp_vect_pl=cp.array([self.shape_crop,self.shape_crop,2.],dtype=self.data_type)
        self.comp_vect_mo=cp.array([-self.shape_crop,-self.shape_crop,-2.],dtype=self.data_type)
        self.comp_vect_invrtz=cp.array([1.,1.,-1.],dtype=self.data_type)
        self.first_init=True
        self.local_map_array=cp.ones((self.cell_n,self.cell_n),dtype=self.data_type)

        self.max_map=max_map_kernel(
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
        self.add_point=set_points_kernel(
            self.resolution,
            self.cell_n,
            self.cell_n,
            params.sensor_noise_factor,
            params.min_valid_distance,
            params.max_height_range,
            params.ramped_height_range_a,
            params.ramped_height_range_b,
            params.ramped_height_range_c,
        )

    def __call__(self, points_all, R, t, pcl_ids, layer_ids, elevation_map, semantic_map, new_map, *args):

        time_beg=time()

        # print( "enterrrrrrrrrrrrrrrrrrrrrrrrr")
        # print(" t: ",t)
        self.base_frame_in_pc_frame=cp.array([t[6],t[7],t[8]])
        # import transforms3d.euler as eul
        # angujjj = eul.mat2euler(cp.asnumpy(R), axes='sxyz')
        # print (" r: ",angujjj)

        ###########print("base_layer iiiiiiiiiiiii33333333333333333333333333: ",(time()-time_beg)*0.000000001)
        local_t=cp.array([t[0]+t[3],t[1]+t[4],t[2]+t[5]])
        # local_t2=cp.array([-t[0]+t[3],-t[1]+t[4],-t[2]+t[5]])
        # newlocal=(cp.linalg.inv(R)).dot(cp.array([t[0],t[1],t[2]]).T).T
        # np_local_t=cp.asnumpy(local_t)
        # print(" local_t: ",local_t)
        # print(" local_t2: ",local_t2)
        # print(" newlocal: ",newlocal)
        # print(" newlocal1: ",(cp.linalg.inv(R)).dot(local_t.T).T)
        # print(" newlocal2: ",(cp.linalg.inv(R)).dot(local_t2.T).T)
        # k=cp.array([1.018, 0.000, 0.342])

        # print("R.shape: ",R.shape)
        # print("R: ",R)
        # print("          ",type(local_t.dtype))

        rot=np.array([[1.,0.,0.],[0.,1.,0.],[0.,0.,-1]],dtype=self.data_type)
        # points=cp.asnumpy(points_all[:,:3])
        points=points_all[:,:3]
        #self.list_of_pcl[index].points 
        new_pcl=points #-self.base_frame_in_pc_frame #(self.list_of_pcl[index].R).dot(self.list_of_pcl[index].points.T).T
        
        shape_crop=self.shape_crop
        # new_pcl+=cp.array([t[0],t[1],t[2]])
        # translate=(self.list_of_pcl[index].R).dot(self.list_of_pcl[index].T.T).T
        # new_pcl=new_pcl+translate
        # new_pcl= (cp.asnumpy(R)).dot(new_pcl.T).T #+ self.list_of_pcl[index].T
        # new_pcl= R.dot(new_pcl.T).T #+ self.list_of_pcl[index].T
        # new_pcl=new_pcl+np.array([self.list_of_pcl[index].T[0],0,0])
        #######print("base_layer iiiiiiiiiiiii2222222222222222222: ",(time()-time_beg)*0.000000001)
        #decimate half
        # print("first temposhape: ",new_pcl.shape)
        # mask_decim=(np.arange(0,new_pcl.shape[0])%4==0)
        # mask_decim=(cp.arange(0,new_pcl.shape[0])%4==0)
        # tempo1=new_pcl[mask_decim,:]
        tempo1=new_pcl
        ########print("base_layer iiiiiiiiiiiii2121212121212121212121212121212121212: ",(time()-time_beg)*0.000000001)
        # print("first temposhape: ",tempo1.shape)
        # mask0=(np.sum(tempo1 != [0.,0.,0.],1)==3)
        mask0=(cp.sum(tempo1 != self.comp_vect_zeros,1)==3)
        
        tempo=R.dot((tempo1[mask0,:]-self.base_frame_in_pc_frame).T).T
        tempo=tempo*self.comp_vect_invrtz
        # print("NEWWWW first temposhape: ",tempo.shape)
        # mask=(np.sum((tempo < [-10.+self.list_of_pcl[index].T[0],np.Inf,np.Inf]) | (tempo > [10.+self.list_of_pcl[index].T[0],-np.Inf,-np.Inf]) |
        #              (tempo < [np.Inf,-10.+self.list_of_pcl[index].T[1],np.Inf]) | (tempo > [-np.Inf,10.+self.list_of_pcl[index].T[1],-np.Inf]) | 
        #              (tempo < [np.Inf,np.Inf,-1.4+self.list_of_pcl[index].T[2]]) ,1)!=3)
        # tempo=tempo[mask,:]
        #########print("base_layer iiiiiiiiiiiii1111111111: ",(time()-time_beg)*0.000000001)
        
        # mask=(np.sum((tempo < [shape_crop+self.list_of_pcl[index].T[0],np.Inf,np.Inf]) & (tempo > [-shape_crop+self.list_of_pcl[index].T[0],-np.Inf,-np.Inf]),1)==3)
        """ mask=(cp.sum((tempo < self.comp_vect_xpl) & (tempo > self.comp_vect_xmo),1)==3)
        tempo=tempo[mask,:]
        # print("NEWWWWsecond temposhape: ",tempo.shape)
         
        # mask=(np.sum((tempo < [np.Inf,shape_crop+self.list_of_pcl[index].T[1],np.Inf]) & (tempo > [-np.Inf,-shape_crop+self.list_of_pcl[index].T[1],-np.Inf]),1)==3) 
        mask=(cp.sum((tempo < self.comp_vect_ypl) & (tempo > self.comp_vect_ymo),1)==3) 
        tempo=tempo[mask,:]
        # print("NEWWWWtierce temposhape: ",tempo.shape)

        # mask=(np.sum((tempo < [np.Inf,np.Inf,2.+self.list_of_pcl[index].T[2]]) & (tempo > [-np.Inf,-np.Inf,-2.+self.list_of_pcl[index].T[2]]) ,1)==3)
        mask=(cp.sum((tempo < self.comp_vect_zpl) & (tempo > self.comp_vect_zmo) ,1)==3)
        tempo=tempo[mask,:] """
        # print("NEWWWWquatre temposhape: ",tempo.shape) 

        mask=(cp.sum((tempo < self.comp_vect_pl) & (tempo > self.comp_vect_mo) ,1)==3)
        tempo=tempo[mask,:]

       
        
        # self.count+=1
        # map_update=False
        # if len(self.threads.keys())==0:
        #     #print("init first")
        #     points=cp.asnumpy(points_all[:,:3])
        #     self.list_of_pcl[0].update(cp.asnumpy(local_t),cp.asnumpy(R),points)
        #     with SharedMemoryManager() as smm:
        #         self.list_of_sharedmem += [smm.SharedMemory(size=self.list_of_pcl[0].SHARED_DATA_NBYTES)]
                    
        #         k=0
        #         arr = create_np_array_from_shared_mem(self.list_of_sharedmem[0], self.list_of_pcl[0].SHARED_DATA_DTYPE, self.list_of_pcl[0].SHARED_DATA_SHAPE)
        #         arr [:] = self.list_of_pcl[0].cloth
        #         self.threads[str(0)]=multiprocessing.Process(target=self.do_CSF, args=(0,),daemon=True )
        #         self.threads[str(0)].start()
        #     self.actif_thread[0]=True
        #     self.last_time_launch=time()

        # else:
        #     last_thread_launch=-1
        #     started_thread=False
        #     first_free_thread=-1
        #     for index_thread in range(len(self.list_com)):
        #         if self.list_com[index_thread][0].poll():
        #             self.actif_thread[index_thread]=False
        #             msg=self.list_com[index_thread][0].recv()
        #             #print("message : ",msg)
        #             if(self.aready_update_with_gap==index_thread):
        #                 self.aready_update_with_gap=-1
        #             self.threads[str(index_thread)].join()
        #             if self.last_time_update<self.list_of_pcl[index_thread].last_time_update:
        #                 array=create_np_array_from_shared_mem(self.list_of_sharedmem[index_thread], self.list_of_pcl[index_thread].SHARED_DATA_DTYPE, self.list_of_pcl[index_thread].SHARED_DATA_SHAPE)
        #                 self.array_cloth=np.reshape(array,(int(array.shape[0]/3),3))
        #                 self.last_time_update=self.list_of_pcl[index_thread].last_time_update
        #                 self.current_R=R
        #                 self.current_T=t
        #                 self.cloth_r=self.list_of_pcl[index_thread].R
        #                 self.cloth_local_t=self.list_of_pcl[index_thread].T
        #                 map_update=True
        #             self.threads[str(index_thread)].terminate()
        #             if not started_thread :
        #                 points=cp.asnumpy(points_all[:,:3])
        #                 self.list_of_pcl[index_thread].update(cp.asnumpy(local_t),cp.asnumpy(R),points)
        #                 self.threads[str(index_thread)]=multiprocessing.Process(target=self.do_CSF, args=(index_thread,),daemon=True )
        #                 self.threads[str(index_thread)].start()
        #                 started_thread=True
        #                 self.actif_thread[index_thread]=True
        #         elif not started_thread and not self.actif_thread[index_thread]:
        #             points=cp.asnumpy(points_all[:,:3])
        #             self.list_of_pcl[index_thread].update(cp.asnumpy(local_t),cp.asnumpy(R),points)
        #             if len(self.list_of_sharedmem)<=index_thread:
        #                 with SharedMemoryManager() as smm:
        #                     self.list_of_sharedmem += [smm.SharedMemory(size=self.list_of_pcl[index_thread].SHARED_DATA_NBYTES)]
                                
        #                     k=0
        #                     arr = create_np_array_from_shared_mem(self.list_of_sharedmem[index_thread], self.list_of_pcl[index_thread].SHARED_DATA_DTYPE, self.list_of_pcl[index_thread].SHARED_DATA_SHAPE)
        #                     arr [:] = self.list_of_pcl[index_thread].cloth
        #             self.threads[str(index_thread)]=multiprocessing.Process(target=self.do_CSF, args=(index_thread,),daemon=True )
        #             self.threads[str(index_thread)].start()
        #             started_thread=True
        #             self.actif_thread[index_thread]=True
        #         else:
        #             if last_thread_launch==-1:
        #                 last_thread_launch=index_thread
        #             elif self.list_of_pcl[index_thread].last_time_update>self.list_of_pcl[last_thread_launch].last_time_update:
        #                 last_thread_launch=index_thread
        #             self.current_R=R
        #             self.current_T=t
        #     if (not started_thread) and ((time_beg-self.list_of_pcl[last_thread_launch].last_time_update)*0.000000001>=0.05) and (self.aready_update_with_gap==-1):
        #         self.threads[str(last_thread_launch)].terminate()
        #         self.aready_update_with_gap=last_thread_launch
        #         points=cp.asnumpy(points_all[:,:3])
        #         self.list_of_pcl[last_thread_launch].update(cp.asnumpy(local_t),cp.asnumpy(R),points)
        #         if len(self.list_of_sharedmem)<=last_thread_launch:
        #             with SharedMemoryManager() as smm:
        #                 self.list_of_sharedmem += [smm.SharedMemory(size=self.list_of_pcl[last_thread_launch].SHARED_DATA_NBYTES)]
                                
        #                 k=0
        #             arr = create_np_array_from_shared_mem(self.list_of_sharedmem[last_thread_launch], self.list_of_pcl[last_thread_launch].SHARED_DATA_DTYPE, self.list_of_pcl[last_thread_launch].SHARED_DATA_SHAPE)
        #             arr [:] = self.list_of_pcl[last_thread_launch].cloth
        #         self.threads[str(last_thread_launch)]=multiprocessing.Process(target=self.do_CSF, args=(last_thread_launch,),daemon=True )
        #         self.threads[str(last_thread_launch)].start()
        #         started_thread=True
        #         self.actif_thread[last_thread_launch]=True
        #     if map_update :
        #         self.count=0


        #print("nb threads: ",len(threading.enumerate())," and ",threading.enumerate())
        #print("true MAX_NB_THREAD : ",self.actif_thread," brabrabrabrabrabrabrabrabrabrabrabrabrabrabrabrabrabrabra")

        # new_pcl=cp.asnumpy(points_all[:,:3]) #(self.list_of_pcl[index].R).dot(self.list_of_pcl[index].points.T).T
        # mask0=(np.sum(new_pcl != [0.,0.,0.],1)==3)
        # tempo=new_pcl[mask0,:]

        # mask=(np.sum(tempo < [6,10,10.],1)==3)
        # tempo=tempo[mask,:]

        # mask=(np.sum((tempo < [0.6,1.5,np.Inf]) & (tempo > [-np.Inf,-1.5,np.Inf]),1)!=3)
        # tempo=tempo[mask,:]

        # mask2=(np.sum(tempo >[-6,-10,-10],1)==3)
        # tempo=tempo[mask2,:]
        # print("ooooooooooooooooooooooooooooooooooooooooooooo")
        # rot=fromeulertorotationmatrix(-3.141, 1.347, -3.141)
        # print("kkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkk  ",cp.asnumpy(t).shape)
        # # mpoo=np.array(rot.dot(new_pcl.T).T)
        # print("vector t : ",cp.asnumpy(t))
        # print("matrix R : ",cp.asnumpy(R))
        # translate=cp.asnumpy(R).dot(cp.asnumpy(t).T).T
        # tempo=tempo+translate
        # mpoo=np.array(cp.asnumpy(R).dot(tempo.T).T )#+ cp.asnumpy(t))
        # mpoo+=np.array([cp.asnumpy(t)[0],0,0])
        # print("ppppppppppp")
        # ici_xyz= cp.array(np.c_[mpoo[:,0],mpoo[:,1],-mpoo[:,2]],dtype=self.data_type) #+ cp.asnumpy(t))
        # print("ppppppppppp",mpoo.shape)
        # # ici_xyz= cp.array(new_pcl)
        # print("shape array out : ",self.array_cloth.shape)
        # newtime=time()
        # print("OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO")
        # print("O                                                                                       O")
        # # print("O                                  t:",self.count)
        # print("O                                                                                       O")
        # print("OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO")
        
        # new_array= cp.array(self.array_cloth*np.array([1.,1.,-1.])+np.array([0.,0.,cp.asnumpy(t)[5]]),dtype=self.data_type) 
        #+np.array([cp.asnumpy(t)[0],cp.asnumpy(t)[1],-cp.asnumpy(t)[2]])
        # z_shift=self.cloth_local_t[2]-cp.asnumpy(local_t)[2]
        # -z_shift-cp.asnumpy(t)[2]
        # comp=self.comp_vect_zeros-0.9
        #*self.comp_vect_invrtz
        ########print("base_layer iiiiiiiiiiiii: ",(time()-time_beg)*0.000000001)
        #+cp.array([local_t[0],local_t[2],self.comp_vect_zeros[2]])
        
        
        new_array= (tempo+cp.array([local_t[0],local_t[1],self.comp_vect_zeros[2]])) #*self.comp_vect_invrtz
        # print("array: ")
        # print(new_array)
        if (self.first_init):
            semantic_map[layer_ids,:,:]=-90.
            self.first_init=False
            # print("    oooooooooooooo                oooooooooooooo")
            # print("      oooooooooo                    oooooooooo  ")
            # print("      oooooooooo                    oooooooooo  ")
            # print("       oooooooo                      oooooooo   ")
            # print("          oo                            oo      ")
            # print("          oo                            oo      ")
            # print("          oo                            oo      ")
            # print("          oo                            oo      ")
            # print("                       nnn                      ")
            # print("                 nnnnnn   nnnnnn                ")
            # print("            nnnnnn             nnnnnn           ")
            # print("            nnnnnn             nnnnnn           ")
        # else:
        #     semantic_map[1,:,:] =  cp.where((semantic_map[1,:,:] == cp.nan) ,-9000,semantic_map[1,:,:])
            # semantic_map[1,:,:] =  cp.where((semantic_map[1,:,:] == cp.nan) + (semantic_map[1,:,:] == 0.)>0,-9000,semantic_map[1,:,:])
        # new_map[1,:,:] =-9000.
        new_map[layer_ids,:,:] = semantic_map[layer_ids,:,:].copy()
        
        semantic_map[layer_ids,:,:]=-90.
        
        # self.local_map_array=cp.ones((self.cell_n,self.cell_n),dtype=self.data_type)

        
        self.max_map(
            layer_ids,
            new_array, #ici_xyz, #
            local_t,
            local_t,
            new_map,
            semantic_map,
            self.local_map_array,
            size=new_array.shape[0] #xyz.shape[0],
        )
        # print()
        
        # print()
        # new_map[1,:,:] = cp.where(new_map[1,:,:] == -9000., 0 ,new_map[1,:,:])
        # new_map[1,:,:] =  cp.where(new_map[1,:,:] == -9000., cp.nan,new_map[1,:,:])
        # semantic_map[1,:,:] =  cp.where(new_map[1,:,:] != -9000., new_map[1,:,:],semantic_map[1,:,:])
        
        
        # semantic_map[layer_ids,:,:] =  cp.where(semantic_map[layer_ids,:,:] <= -80., new_map[layer_ids,:,:],semantic_map[layer_ids,:,:])
        # semantic_map[layer_ids,:,:] =  cp.where((semantic_map[layer_ids,:,:] >= -80.) & (new_map[layer_ids,:,:] >= -80.), (new_map[layer_ids,:,:]+semantic_map[layer_ids,:,:])/2,semantic_map[layer_ids,:,:])
       
        # semantic_map[1,:,:] =  cp.where(semantic_map[1,:,:] <= -8000.,cp.nan,semantic_map[1,:,:])
#  self.local_map_array[:,:] #cp.where(self.local_map_array[:,:] == -9000.,cp.nan,self.local_map_array[:,:])
        """ print("VVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVV")
        print()
        print("base_layer: ",(time()-time_beg)*0.000000001)
        print()
        print("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA") """

