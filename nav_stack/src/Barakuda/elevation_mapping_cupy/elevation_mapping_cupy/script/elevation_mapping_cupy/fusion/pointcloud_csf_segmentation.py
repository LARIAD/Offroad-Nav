
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
import os, sys, signal, psutil
from time import time_ns as time
from time import sleep
import multiprocessing
from typing import Tuple
import threading
#import ros_numpy
from sensor_msgs.msg import PointCloud2

MAX_NB_THREAD=3
#LOCK = threading.Lock()

class Worker(multiprocessing.Process):
    def __init__(self, gpu_id,group=None, target=None, name=None, args=(), kwargs={},
                 *, daemon=None):
        super().__init__()
        self.gpu_id = gpu_id
        
    def run(self):
        with cp.cuda.Device(self.gpu_id):
            if self._target:
                self._target(*self._args, **self._kwargs)

class SharedObj(object):
    def __init__(self,dat_type,funct,arg):
        T=np.zeros((3),dtype=dat_type)
        R=np.zeros((3,3),dtype=dat_type)
        points=np.zeros((1,3),dtype=dat_type)
        result=np.zeros((1,3),dtype=dat_type)
        self.parent_PID=os.getpid()
        self.signal_to_end=None
        
        self.crop=[[50.,-50],[50.,-50],[0.,-10.]]#[[1024.,0],[128.,0],[0.,-10.]]#
        self.cloth_resolution=0.25
        self.SHARED_DATA_DTYPE = dat_type

        self.com_in=multiprocessing.Pipe()
        self.com_out=multiprocessing.Pipe()
        self.msg_in=[]

        self.shared_mem_T =None # multiprocessing.shared_memory.SharedMemory(create=True, size=T.nbytes)
        self.T=None #np.ndarray(T.shape, dtype=T.dtype, buffer=self.shared_mem_T.buf)
        # self.T[:]=T[:]
        self.T_mem_name=None #self.shared_mem_T.name
        # self.T_mem_owner=True

        self.shared_mem_R =None # multiprocessing.shared_memory.SharedMemory(create=True, size=R.nbytes)
        self.R=None #np.ndarray(R.shape, dtype=R.dtype, buffer=self.shared_mem_R.buf)
        # self.R[:]=R[:]
        self.R_mem_name=None #self.shared_mem_R.name

        self.shared_mem_points =None # multiprocessing.shared_memory.SharedMemory(create=True, size=points.nbytes)
        self.points=None #np.ndarray(points.shape, dtype=points.dtype, buffer=self.shared_mem_points.buf)
        # self.points[:]=points[:]
        self.points_mem_name=None #self.shared_mem_points.name

        self.should_init_result=True
        self.shared_mem_result = None#multiprocessing.shared_memory.SharedMemory(create=True, size=result.nbytes)
        self.result=None#np.ndarray(result.shape, dtype=result.dtype, buffer=self.shared_mem_result.buf)
        # self.result[:]=result[:]
        self.result_mem_name=None#self.shared_mem_result.name

        # print("33333333333333333333333333333333333333333333333333")
        # print("3                                                3")
        # print("3     T  name: ",self.T_mem_name)
        # print("3                                                3")
        # print("3     R  name: ",self.R_mem_name)
        # print("3                                                3")
        # print("3     point  name: ",self.points_mem_name)
        # print("3                                                3")
        # print("33333333333333333333333333333333333333333333333333")
        
        self.last_time_update=time()
        self.actif_thread=False
        self.thread=multiprocessing.Process(target=funct, args=arg,daemon=True )
        self.already_start=False
        self.child=False
        self.continue_exec=True

    def clear_mem(self):
        if self.R_mem_name!=None:
            self.shared_mem_R.close()
            if not self.child or True:
                try :
                    self.shared_mem_R.unlink()
                except (IOError, OSError) as e:
                    pass
        if self.points_mem_name!=None:
            self.shared_mem_points.close()
            if not self.child or True:
                try :
                    self.shared_mem_points.unlink()
                except (IOError, OSError) as e:
                    pass
                    
        if self.T_mem_name!=None:
            self.shared_mem_T.close()
            if not self.child or True:
                try :
                    self.shared_mem_T.unlink()
                except (IOError, OSError) as e:
                    pass
        if self.result_mem_name!=None:
            if not self.should_init_result:
                
                self.shared_mem_result.close()
                if self.child :
                    
                    try :
                        self.shared_mem_result.unlink()
                    except (IOError, OSError) as e:
                        pass
                    
    def signal_handler(self,sig, frame):
        self.finish_properly()
        sys.exit(0)
    
    def hard_clear_mem(self):
        self.shared_mem_R.close()
        self.shared_mem_R.unlink()
        
        self.shared_mem_points.close()
        self.shared_mem_points.unlink()
        
        self.shared_mem_T.close()
        self.shared_mem_T.unlink()
        
        if not self.should_init_result:
            self.shared_mem_result.close()
            try :
                self.shared_mem_result.unlink()
            except (IOError, OSError) as e:
                pass

    def close_all_mem(self):
        if self.R_mem_name!=None:
            self.shared_mem_R.close()
        if self.points_mem_name!=None:
            self.shared_mem_points.close()
        if self.T_mem_name!=None:
            self.shared_mem_T.close()
        if not self.should_init_result and self.result_mem_name!=None:
            self.shared_mem_result.close()

        

    def finish_properly(self):
        
        self.close_all_mem()
        self.continue_exec=False
        
        
        if not self.child and self.already_start:
            self.stop(2)
            try:
                os.kill(self.thread.pid, signal.SIGINT)
            except OSError as err:
                pass
            self.thread.join(3)
        self.clear_mem()
        
    def force_finish(self):

        self.close_all_mem()
        self.continue_exec=False
        self.stop()
        self.hard_clear_mem()
        sleep(0.5)

    def update_mem_changes(self,obj,ind):


        if ind==0:
            self.shared_mem_points.close()
            self.shared_mem_points = multiprocessing.shared_memory.SharedMemory(create=True, size=obj.nbytes)
            self.points=np.ndarray(obj.shape, dtype=obj.dtype, buffer=self.shared_mem_points.buf)
            self.points[:]=obj[:]
            self.points_mem_name=self.shared_mem_points.name
        elif ind==1:
            self.shared_mem_T.close()
            self.shared_mem_T = multiprocessing.shared_memory.SharedMemory(create=True, size=obj.nbytes)
            self.T=np.ndarray(obj.shape, dtype=obj.dtype, buffer=self.shared_mem_T.buf)
            self.T[:]=obj[:]
            self.T_mem_name=self.shared_mem_T.name
        elif ind==2:
            self.shared_mem_R.close()
            self.shared_mem_R = multiprocessing.shared_memory.SharedMemory(create=True, size=obj.nbytes)
            self.R=np.ndarray(obj.shape, dtype=obj.dtype, buffer=self.shared_mem_R.buf)
            self.R[:]=obj[:]
            self.R_mem_name=self.shared_mem_R.name
        elif ind==3:
            if self.result_mem_name!=None:
                self.shared_mem_result.close()
                self.shared_mem_result.unlink()
            self.shared_mem_result = multiprocessing.shared_memory.SharedMemory(create=True, size=obj.nbytes)
            self.result=np.ndarray(obj.shape, dtype=obj.dtype, buffer=self.shared_mem_result.buf)
            self.result[:]=obj[:]
            self.result_mem_name=self.shared_mem_result.name

    def distroy_mem_due_changes(self,name,shape,ind):

        if ind==0:
            if self.shared_mem_result!=None:
                self.shared_mem_points.close()
                self.shared_mem_points.unlink()
            self.shared_mem_points = multiprocessing.shared_memory.SharedMemory(name)
            self.points=np.ndarray(shape, dtype=self.SHARED_DATA_DTYPE, buffer=self.shared_mem_points.buf)
            self.points_mem_name=self.shared_mem_points.name
        elif ind==1:
            if self.shared_mem_result!=None:
                self.shared_mem_T.close()
                self.shared_mem_T.unlink()
            self.shared_mem_T = multiprocessing.shared_memory.SharedMemory(name)
            self.T=np.ndarray(shape, dtype=self.SHARED_DATA_DTYPE, buffer=self.shared_mem_T.buf)
            self.T_mem_name=self.shared_mem_T.name
        elif ind==2:
            if self.shared_mem_result!=None:
                self.shared_mem_R.close()
                self.shared_mem_R.unlink()
            self.shared_mem_R = multiprocessing.shared_memory.SharedMemory(name)
            self.R=np.ndarray(shape, dtype=self.SHARED_DATA_DTYPE, buffer=self.shared_mem_R.buf)
            self.R_mem_name=self.shared_mem_R.name
        elif ind==3:
            try:
                if self.shared_mem_result!=None:
                    self.shared_mem_result.close()
            
                self.shared_mem_result = multiprocessing.shared_memory.SharedMemory(name)
                self.result=np.ndarray(shape, dtype=self.SHARED_DATA_DTYPE, buffer=self.shared_mem_result.buf)
                self.result_mem_name=self.shared_mem_result.name
            except (IOError, OSError) as e:
                pass


    def update(self,T,R,points):

        
        if self.already_start and self.T_mem_name!=None:
            self.update_mem_changes(T,1)     
        else:
            self.shared_mem_T = multiprocessing.shared_memory.SharedMemory(create=True, size=T.nbytes)
            self.T=np.ndarray(T.shape, dtype=T.dtype, buffer=self.shared_mem_T.buf)
            self.T[:]=T[:]
            self.T_mem_name=self.shared_mem_T.name

        if self.already_start and self.R_mem_name!=None:
            self.update_mem_changes(R,2) 
        else:
            self.shared_mem_R = multiprocessing.shared_memory.SharedMemory(create=True, size=R.nbytes)
            self.R=np.ndarray(R.shape, dtype=R.dtype, buffer=self.shared_mem_R.buf)
            self.R[:]=R[:]
            self.R_mem_name=self.shared_mem_R.name

        if self.already_start and self.points_mem_name!=None:
            self.update_mem_changes(points,0) 
        else:
            self.shared_mem_points = multiprocessing.shared_memory.SharedMemory(create=True, size=points.nbytes)
            self.points=np.ndarray(points.shape, dtype=points.dtype, buffer=self.shared_mem_points.buf)
            self.points[:]=points[:]
            self.points_mem_name=self.shared_mem_points.name

        self.last_time_update=time()

    def update_changes_from_other_thread(self,T_name,Tshape,R_name,R_shape,points_name,points_shape):

        if T_name!=self.T_mem_name:
            self.distroy_mem_due_changes(T_name,Tshape,1)        

        if R_name!=self.R_mem_name:
            self.distroy_mem_due_changes(R_name,R_shape,2)        

        if points_name!=self.points_mem_name:
            self.distroy_mem_due_changes(points_name,points_shape,0)      
  
    
    def csf_finished(self):
        
        if self.com_out[0].poll():
            msg=self.com_out[0].recv()
            if msg[0]!=self.result_mem_name:
                if self.should_init_result:
                    self.shared_mem_result = multiprocessing.shared_memory.SharedMemory(msg[0])
                    self.result=np.ndarray(msg[1], dtype=self.SHARED_DATA_DTYPE, buffer=self.shared_mem_result.buf)
                    self.result_mem_name=self.shared_mem_result.name
                else:
                    self.distroy_mem_due_changes(msg[0],msg[1],3)
            self.set_wait()

            return 1
        else:
            return 0

    def get_result(self):
        out=self.result.copy()
        if self.result_mem_name!=None:
            self.shared_mem_result.close()
        return out

    def set_result(self,arr):
        
        if self.should_init_result:
            array_good_type=arr.astype(self.SHARED_DATA_DTYPE)
            self.shared_mem_result = multiprocessing.shared_memory.SharedMemory(create=True, size=array_good_type.nbytes)
            self.result=np.ndarray(array_good_type.shape, dtype=array_good_type.dtype, buffer=self.shared_mem_result.buf)
            self.result[:]=array_good_type[:]
            self.result_mem_name=self.shared_mem_result.name
            self.should_init_result=False
        else:
            if psutil.pid_exists(self.parent_PID):
                self.update_mem_changes(arr.astype(self.result.dtype),3)
            else:
                pass
        
        self.send([self.result_mem_name,self.result.shape])
        
        self.set_wait()



    def send(self,msg):
        self.com_out[1].send(msg)

    def recive(self):
        self.child=True
        self.signal_to_end=signal.signal(signal.SIGINT, self.signal_handler)
        if self.com_in[0].poll():
            msg=self.com_in[0].recv()
            if len(msg)==5:
                self.shared_mem_result.close()
                self.shared_mem_result.unlink()
                return 0

            elif len(msg)<6:
                return 0
            
            self.update_changes_from_other_thread(msg[0],msg[1],msg[2],msg[3],msg[4],msg[5])
            
            self.set_actif()
            return 1
        else:
            return 0

    def re_init_thread(self,funct,arg):
        self.thread=multiprocessing.Process(target=funct, args=arg,daemon=True )

    def start(self):

        if not self.already_start:
            self.thread.start()
        self.should_init_result=False
        self.com_in[1].send([self.T_mem_name,self.T.shape,self.R_mem_name,self.R.shape,self.points_mem_name,self.points.shape])
        self.already_start=True
        self.set_actif()

    def is_actif(self):
        return self.actif_thread

    def set_actif(self):
        self.actif_thread=True

    def set_wait(self):
        self.actif_thread=False
    
    def stop(self,nb_sec=0.):
        self.com_in[1].send([0,0,0,0,0])
        
        sleep(nb_sec)
        
    def __del__(self):
        self.finish_properly()

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
    average_map_kernel = cp.ElementwiseKernel(
        in_params="raw int32 l_id_in, raw U p, raw U center_x, raw U center_y",
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
            U z = p[i * 3 + 2];
            int l_id=l_id_in[0];
            int idx = get_idx(x,y,center_x[0],center_y[1]);
            if (is_inside(idx)){
                U h = map[get_map_idx(idx, l_id)];
                U new_cnt = newmap[get_map_idx(idx, l_id)];
                pop_array[idx]=0;
            
                if (new_cnt > 0) {
                    float cnt=new_cnt;
               
                    atomicExch(&map[get_map_idx(idx, l_id)],z); 
                    atomicAdd(&newmap[get_map_idx(idx, l_id)], 1.0f);
                }else{
                    atomicExch(&map[get_map_idx(idx, l_id)], z);
                    atomicExch(&newmap[get_map_idx(idx, l_id)] , 1.0f);
                }
            }
            """ #(h*cnt+z)/(cnt+1)
        ).substitute(resolution=resolution,
        width=width,
        height=height,),
        name="average_map_kernel",
    )
    return average_map_kernel
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
                float value=p[c_id*3+2];
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


class CSF_pcl(FusionBase):
    def __init__(self, params,
                 csf_bSloopSmooth=False, 
                 csf_cloth_resolution=0.5,
                 csf_class_threshold=0.05,
                 csf_time_step=0.65,
                 csf_rigidness=2,
                 csf_interations=20,
                 lenght_view=35.,
                 *args, **kwargs):
        self.name = "pointcloud_csf_segmentation"
        self.cell_n = params.cell_n
        self.resolution = params.resolution
        self.data_type=params.data_type

        self.array_cloth=np.zeros((2,3),dtype=self.data_type)


        self.base_frame_in_pc_frame=np.array([0.670, -0.000, -0.840],dtype=self.data_type) # lidar
        #self.base_frame_in_pc_frame = cp.array([-0.039, 0.954, -0.733], dtype=self.data_type) # mono
        
        self.list_of_pcl=[SharedObj(self.data_type,self.loop_csf,(i,)) for i in range(MAX_NB_THREAD)]

        # self.list_com=[multiprocessing.Pipe() for _ in range(MAX_NB_THREAD)]
        # self.list_of_sharedmem=[]
        self.last_time_update=time()
        # self.mutex=[ multiprocessing.Manager().Lock() for _ in range(MAX_NB_THREAD)]
        # self.actif_thread=[False for _ in range(MAX_NB_THREAD)]
        self.current_R=cp.array([[1.,0.,0.],[0.,1.,0.],[0.,0.,1.]])
        self.current_T=cp.array([0.,0.,0.])
        self.rotinv=np.array([[1.,0.,0.],[0.,1.,0.],[0.,0.,-1]],dtype=self.data_type)
        
        self.last_time_launch=time()
        self.shape_crop=lenght_view
        self.cloth_local_t=np.array([0.0,0.0,0.0])
        self.cloth_r=np.array([[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0]])

        self.count_msg_drop=0
        self.average_drop=0
        

        self.average_map=average_map_kernel(
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
        self.big_cloth_map_complet=big_cloth_map_complet_kernel(
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

        self.csf_bSloopSmooth = csf_bSloopSmooth #False
        self.csf_cloth_resolution = csf_cloth_resolution #0.50
        self.csf_time_step=csf_time_step #0.65#0.65
        self.csf_class_threshold=csf_class_threshold #0.05#0.05
        self.csf_rigidness=csf_rigidness #2
        self.csf_interations=csf_interations #20
        
    def loop_csf(self,index=0):
        old_time=time()
        while self.list_of_pcl[index].continue_exec:
            # try:
            now=time()
            if self.list_of_pcl[index].recive():
                self.do_CSF(index)
                old_time=now
            
            if not psutil.pid_exists(self.list_of_pcl[index].parent_PID): #(now-old_time)*0.000000001>20.:
                break
            else:
                pass
            
        self.list_of_pcl[index].finish_properly()


    def do_CSF(self,index=0):
        
        local_time=time()
        # print("gggggggggggggggggggggg",self.list_of_pcl[index].T,"ggggggggggggggggggggggggggggg")
        rot=np.array([[1.,0.,0.],[0.,1.,0.],[0.,0.,-1]],dtype=self.data_type)
        #self.list_of_pcl[index].points 
        new_pcl=self.list_of_pcl[index].points #-self.base_frame_in_pc_frame #(self.list_of_pcl[index].R).dot(self.list_of_pcl[index].points.T).T
        # print("ttttttttttttttttttttttttttttttttttttttttttttttttttttttttttttttttttttttype ",type(new_pcl))
        shape_crop=self.shape_crop
        # translate=(self.list_of_pcl[index].R).dot(self.list_of_pcl[index].T.T).T
        # new_pcl=new_pcl+translate
        #point_zero=(self.list_of_pcl[index].R).dot(-self.base_frame_in_pc_frame.T).T
        #new_pcl= (self.list_of_pcl[index].R).dot(new_pcl.T).T #+ self.list_of_pcl[index].T
        # new_pcl=new_pcl+np.array([self.list_of_pcl[index].T[0],0,0])
        
        #decimate half
        # print("first temposhape: ",new_pcl.shape)
        mask_decim=(np.arange(0,new_pcl.shape[0])%4==0)
        tempo1=new_pcl[mask_decim,:]
        # print("first temposhape: ",tempo1.shape) 
        mask0=(np.sum(tempo1 != [0.,0.,0.],1)==3)
        #tempo=tempo1[mask0,:]
        tempo= (self.list_of_pcl[index].R).dot((tempo1[mask0,:]-self.base_frame_in_pc_frame).T).T#+np.array([self.list_of_pcl[index].T[0],self.list_of_pcl[index].T[1],0])#-self.base_frame_in_pc_frame
        tempo= tempo*np.array([1.,1.,-1.])+np.array([self.list_of_pcl[index].T[0],self.list_of_pcl[index].T[1],0])#*np.array([1.,1.,-1.])
        # print("first temposhape: ",tempo.shape)
        # mask=(np.sum((tempo < [-10.+self.list_of_pcl[index].T[0],np.Inf,np.Inf]) | (tempo > [10.+self.list_of_pcl[index].T[0],-np.Inf,-np.Inf]) |
        #              (tempo < [np.Inf,-10.+self.list_of_pcl[index].T[1],np.Inf]) | (tempo > [-np.Inf,10.+self.list_of_pcl[index].T[1],-np.Inf]) | 
        #              (tempo < [np.Inf,np.Inf,-1.4+self.list_of_pcl[index].T[2]]) ,1)!=3)
        # tempo=tempo[mask,:]

        """ # mask=(np.sum((tempo < [shape_crop+self.list_of_pcl[index].T[0],np.Inf,np.Inf]) & (tempo > [-shape_crop+self.list_of_pcl[index].T[0],-np.Inf,-np.Inf]),1)==3)
        mask=(np.sum((tempo < [shape_crop,np.Inf,np.Inf]) & (tempo > [-shape_crop,-np.Inf,-np.Inf]),1)==3)
        tempo=tempo[mask,:]
        # print("second temposhape: ",tempo.shape)
         
        # mask=(np.sum((tempo < [np.Inf,shape_crop+self.list_of_pcl[index].T[1],np.Inf]) & (tempo > [-np.Inf,-shape_crop+self.list_of_pcl[index].T[1],-np.Inf]),1)==3) 
        mask=(np.sum((tempo < [np.Inf,shape_crop,np.Inf]) & (tempo > [-np.Inf,-shape_crop,-np.Inf]),1)==3) 
        tempo=tempo[mask,:]
        # print("tierce temposhape: ",tempo.shape)

        # mask=(np.sum((tempo < [np.Inf,np.Inf,2.+self.list_of_pcl[index].T[2]]) & (tempo > [-np.Inf,-np.Inf,-2.+self.list_of_pcl[index].T[2]]) ,1)==3)
        mask=(np.sum((tempo < [np.Inf,np.Inf,1.7]) & (tempo > [-np.Inf,-np.Inf,-1.7]) ,1)==3)
        tempo=tempo[mask,:] """

        mask=(np.sum((tempo < [shape_crop+self.list_of_pcl[index].T[0],shape_crop+self.list_of_pcl[index].T[1],2.+self.list_of_pcl[index].T[2]]) & 
               (tempo > [-shape_crop+self.list_of_pcl[index].T[0],-shape_crop+self.list_of_pcl[index].T[1],-2.+self.list_of_pcl[index].T[2]]) ,1)==3)
        tempo=tempo[mask,:]


        # print("quatre temposhape: ",tempo.shape)

        # mask0=(np.sum(new_pcl != [0.,0.,0.],1)==3)
        # tempo=new_pcl[mask0,:]

        # mask=(np.sum(tempo < [10.,10.,1.],1)==3)
        # tempo=tempo[mask,:]
        
        # mask=(np.sum((tempo < [0.6,1.5,np.Inf]) & (tempo > [-np.Inf,-1.5,np.Inf]),1)!=3)
        # tempo=tempo[mask,:]

        # mask2=(np.sum(tempo >[-6,-10,-10],1)==3)
        # tempo=tempo[mask2,:]

        # translate=(self.list_of_pcl[index].R).dot(self.list_of_pcl[index].T.T).T
        # tempo=tempo+translate
        # mpoo= (self.list_of_pcl[index].R).dot(tempo.T).T #+ self.list_of_pcl[index].T
        # xyz=mpoo+np.array([self.list_of_pcl[index].T[0],0,0])

        xyz=tempo
        # print("temposhape: ",xyz.shape)
        # xyz+=np
        newtime=time()

        csf = CSF.CSF()

        # prameter settings
        csf.params.bSloopSmooth = self.csf_bSloopSmooth
        csf.params.cloth_resolution = self.csf_cloth_resolution
        csf.params.time_step=self.csf_time_step#0.65
        csf.params.class_threshold=self.csf_class_threshold#0.05
        csf.params.rigidness=self.csf_rigidness
        csf.params.interations=self.csf_interations
        # csf.params.bSloopSmooth = False
        # csf.params.cloth_resolution = 0.25
        # csf.params.time_step=0.5#1.5
        # csf.params.class_threshold=0.05
        # csf.params.rigidness=1
        # csf.params.interations=200
        csf.setPointCloud(np.c_[xyz[:, 0], xyz[:, 1], -1*xyz[:, 2]]) #-1*
        ground = CSF.VecInt()  # a list to indicate the index of ground points after calculation
        non_ground = CSF.VecInt()
        # print("?????????????????????????????????????????")
        # print("               time: ",local_time*0.000000001)
        # csf.do_filtering(ground, non_ground,False)
        # print ("len gound point ",ground.size())
        # arraycloth=np.array( csf.do_cloth_export() )
        # csf.setPointCloud(np.c_[xyz[ground, 0], xyz[ground, 1], xyz[ground, 2]])
        # csf.params.time_step=0.95#1.5
        ground2 = CSF.VecInt()
        non_ground2 = CSF.VecInt() 
        
        #csf.do_filtering(ground2, non_ground2,False)

        newtime1=time()

        newtime=newtime1

        arraycloth=np.array( csf.do_cloth_export(False,0.1/0.3) )#False,0.1/0.3
        # arraycloth=np.array()
        array_to_send=np.reshape(arraycloth.astype(self.data_type),(int(arraycloth.shape[0]/3),3))
        # print("max: ",np.max(array_to_send,1)[0]," | ",np.max(array_to_send,1)[-1])
        # print("min: ",np.min(array_to_send,1)[0]," | ",np.min(array_to_send,1)[-1])
        self.list_of_pcl[index].set_result(array_to_send)
        # self.list_of_pcl[index].set_result(xyz*np.array([1.,1.,-1.]))


        # print()
        # print("QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ")
        # print("csf_time_cost: ",(time()-local_time)*0.000000001)
        # print("QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ")
        # print()

    def __call__(self, points_all, R, t, pcl_ids, layer_ids, elevation_map, semantic_map, new_map, *args):

        time_beg=time()

        local_t=cp.array([t[0]+t[3],t[1]+t[4],t[2]+t[5]])

        started_thread=False
        for index_thread in range(len(self.list_of_pcl)):
            if self.list_of_pcl[index_thread].csf_finished():
                if self.last_time_update<self.list_of_pcl[index_thread].last_time_update:
                        array=self.list_of_pcl[index_thread].get_result()
                        self.array_cloth=array
                        self.last_time_update=self.list_of_pcl[index_thread].last_time_update
                        self.current_R=R
                        self.current_T=t
                        self.cloth_r=self.list_of_pcl[index_thread].R
                        self.cloth_local_t=self.list_of_pcl[index_thread].T
                        map_update=True
            if not started_thread and not self.list_of_pcl[index_thread].is_actif() and self.count_msg_drop>=(int)(self.average_drop/MAX_NB_THREAD):
                        points=cp.asnumpy(points_all[:,:3])
                        self.list_of_pcl[index_thread].update(cp.asnumpy(local_t),cp.asnumpy(R),points)
                        self.list_of_pcl[index_thread].start()
                        started_thread=True
        if not started_thread:
            self.count_msg_drop+=1
        elif self.average_drop==0:
            self.average_drop=self.count_msg_drop
            self.count_msg_drop=0
        else:
            self.average_drop=(int)((5*self.average_drop+self.count_msg_drop)/5)
            self.count_msg_drop=0

        newtime=time()
        # print("OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO")
        # print("O                                                                                       O")
        # print("O                                  t:",t)
        # print("O                                                                                       O")
        # print("OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO")
        
        new_array= cp.array(self.array_cloth,dtype=self.data_type)#*np.array([1.,1.,1.])+np.array([self.cloth_local_t[0],self.cloth_local_t[1],0.])
        semantic_map[layer_ids,:,:]=0.0
        index_array=cp.zeros((self.cell_n,self.cell_n),dtype=self.data_type,)

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
        
        semantic_map[layer_ids,:,:] = cp.where(semantic_map[layer_ids,:,:] == 0., cp.nan ,semantic_map[layer_ids,:,:])

        newtime1=time()

        k=newtime1-time_beg
        """ print("OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO")
        print("O                                                                                       O")
        print("O                                 end toototo: ",k*0.000000001)
        print("O                                                                                       O")
        print("OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO") """

    def __del__(self):
        del self.array_cloth
        del self.cloth_r
        del self.cloth_local_t
        for item in self.list_of_pcl:
            # print("del: ")
            # print("    R: ",item.R_mem_name)
            # print("    T: ",item.T_mem_name)
            # print("    points: ",item.points_mem_name)
            # print("    Result: ",item.result_mem_name)
            del item

