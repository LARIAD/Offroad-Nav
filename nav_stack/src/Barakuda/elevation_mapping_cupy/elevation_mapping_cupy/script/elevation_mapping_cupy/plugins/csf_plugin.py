import cupy as cp
from typing import List
from .plugin_manager import PluginBase
import cupyx.scipy.ndimage as ndimage
import CSF
import numpy as np
import time

class MaskInvalid(PluginBase):
    def __init__(self, input_layer_name="csf_ground", bSloopSmooth= False,
                time_step= 0.65, class_threshold= 0.05, cloth_resolution= 0.25, rigidness= 2., interations= 100, **kwargs):
        super().__init__()
        self.input_layer_name = "elevation"
        self.bSloopSmooth = (bSloopSmooth)
        self.time_step = (time_step)
        self.class_threshold = (class_threshold)
        self.cloth_resolution = (cloth_resolution)
        self.rigidness = rigidness
        self.interations = (interations)
        self.default_layer_name = "elevation"
        self.flag = 1
        self.time = time.time()

    def __call__(
        self,
        elevation_map: cp.ndarray,
        layer_names: List[str],
        plugin_layers: cp.ndarray,
        plugin_layer_names: List[str],
        semantic_map: cp.ndarray,
        semantic_layer_names: List[str],
        *args
    ) -> cp.ndarray:
        """
        Masks invalid data out of smoothened grid map
        """
        # Process maps here
        # You can also use the other plugin's data through plugin_layers.
        # if self.flag:
        #     print(layer_names)
        #     print(elevation_map.shape)
        #     print(plugin_layer_names)
        #     self.flag = 0
        # layer_data = self.get_layer_data(
        #     elevation_map,
        #     layer_names,
        #     plugin_layers,
        #     plugin_layer_names,
        #     semantic_map,
        #     semantic_layer_names,
        #     self.input_layer_name,
        # )
        # if layer_data is None:
        #     print(f"No layers are found, using {self.default_layer_name}!")
        #     layer_data = self.get_layer_data(
        #         elevation_map,
        #         layer_names,
        #         plugin_layers,
        #         plugin_layer_names,
        #         semantic_map,
        #         semantic_layer_names,
        #         self.default_layer_name,
        #     )

        layer_data = self.get_layer_data(
            elevation_map,
            layer_names,
            plugin_layers,
            plugin_layer_names,
            semantic_map,
            semantic_layer_names,
            self.input_layer_name,
        )
        if layer_data is None:
            if (time.time - self.time> 3.0):
                print(f"No layers are found, using {self.default_layer_name}!")
                self.time = time.time
            layer_data = self.get_layer_data(
                elevation_map,
                layer_names,
                plugin_layers,
                plugin_layer_names,
                semantic_map,
                semantic_layer_names,
                self.default_layer_name,
            )
            if layer_data is None:
                if (time.time - self.time> 3.0):
                    print(f"No layers are found, using elevation!")
                    self.time = time.time
                layer_data = self.get_layer_data(
                    elevation_map,
                    layer_names,
                    plugin_layers,
                    plugin_layer_names,
                    semantic_map,
                    semantic_layer_names,
                    "elevation",
                )
        layer_np = cp.asnumpy(elevation_map[0])
        dim = np.sqrt(layer_np.shape[0])
        X=(np.repeat([np.arange(0,dim,1,dtype=float)],repeats=dim,axis=0)).flatten()
        Y=np.repeat(np.arange(0,dim,1,dtype=float),dim,axis=0)
        pointcloud=np.c_[X[:], Y[:], layer_np[:]]
        csf = CSF.CSF()
        csf.params.bSloopSmooth = self.bSloopSmooth
        csf.params.cloth_resolution = self.cloth_resolution
        csf.params.time_step = self.time_step
        csf.params.class_threshold = self.class_threshold
        # csf.params.rigidness = self.rigidness
        csf.params.interations = self.interations
        csf.setPointCloud(pointcloud)
        ground = CSF.VecInt()  # a list to indicate the index of ground points after calculation
        non_ground = CSF.VecInt()
        csf.do_filtering(ground, non_ground)
        newlayer=pointcloud[ground]
        layerexclude=layer_np[ground]
        csf.setPointCloud(np.c_[newlayer[:,0],newlayer[:,1],-newlayer[:,2]])
        ground1 = CSF.VecInt()  # a list to indicate the index of ground points after calculation
        non_ground1 = CSF.VecInt()
        csf.params.class_threshold=0.2
        csf.do_filtering(ground1, non_ground1)
        itermediate=np.zeros_like(layerexclude)
        itermediate[ground1]+=layerexclude[ground1]
        new=np.zeros_like(layer_np)
        new[ground ]+=itermediate
        cpnew=cp.asarray(new)
        cpnew[:] = cp.where(cpnew[:] == 0., cp.nan ,cpnew[:])

        return cpnew

