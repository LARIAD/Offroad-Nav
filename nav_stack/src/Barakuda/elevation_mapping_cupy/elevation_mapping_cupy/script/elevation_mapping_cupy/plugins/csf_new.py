import cupy as cp
from typing import List
from .plugin_manager import PluginBase
import cupyx.scipy.ndimage as ndimage


class Csf_new(PluginBase):
    def __init__(
        self,
        input_layer_name="obli10",
        **kwargs,
    ):
        super().__init__()
        self.input_layer_name = input_layer_name

    def __call__(
        self, 
        elevation_map: cp.ndarray,
        layer_names: List[str],
        plugin_layers: cp.ndarray,
        plugin_layer_names: List[str],
        semantic_map: cp.ndarray,
        semantic_layer_names: List[str],
        *args,
    ) -> cp.ndarray:
        # print("sementic_layer_map: ",semantic_layer_names, " we search ",self.input_layer_name)
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
            #print(f"No layers are found, using elevation!")
            layer_data = self.get_layer_data(
                elevation_map,
                layer_names,
                plugin_layers,
                plugin_layer_names,
                semantic_map,
                semantic_layer_names,
                "elevation",
            )
        # layer_np = cp.asnumpy(layer_data)
        # print("shape-semmmmmmmmmmmm: ",semantic_map.shape)
        # print("IIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIII")
        # print("I                                                                                       I")
        # print("I                                  sementic_map:",semantic_map[0,23,21])
        # print("I                                  elevation_map:",elevation_map[0,23,21])
        # print("I                                                                                       I")
        # print("IIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIII")
        if semantic_layer_names==[]:
            return cp.array([0.0], dtype=cp.float32)
        return layer_data #semantic_map[1,:,:] #elevation_map[0,:,:]-
