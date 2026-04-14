# import cv2 as cv
import scipy.ndimage as scim
import cupy as cp
import numpy as np
from typing import List
from .plugin_manager import PluginBase


class Inflation(PluginBase):
    def __init__(
        self,
        input_layer_name="csf",
        kernel_size: int = 5, # Controls how far the inflation extends from obstacles.
        iterations: int = 1, # Controls how many times the dilation is applied
        inflation_cost: float =1.,  # Cost value for inflated area
        threshold: float = 0.31, # Adjust based on your csf values
        **kwargs,
    ):
        super().__init__()
        self.input_layer_name = input_layer_name
        self.kernel_size = kernel_size
        self.iterations = iterations
        self.inflation_cost = inflation_cost
        self.threshold = threshold

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
        
        layer_data = self.get_layer_data(
            elevation_map, layer_names, plugin_layers, plugin_layer_names,
            semantic_map, semantic_layer_names, self.input_layer_name,
        )
        
        if layer_data is None:
            return cp.zeros_like(elevation_map[0])
        
        layer_np = cp.asnumpy(layer_data)
        
        # Create binary mask of obstacles (high values)
        obstacle_mask = (layer_np > self.threshold)#.astype(np.uint8) * 255
        
        # Dilate the obstacle mask
        kernel = np.ones((self.kernel_size, self.kernel_size), np.bool)
        dilated_mask = scim.binary_dilation(obstacle_mask, kernel, iterations=self.iterations)
        # dilated_mask = cv.dilate(obstacle_mask, kernel, iterations=self.iterations)
        
        # Create inflation zone (dilated - original)
        inflation_zone = (dilated_mask > 0) #& (obstacle_mask == 0)
        
        # Build result: original values + inflation cost in inflation zone
        result = layer_np.copy()
        result[inflation_zone] = np.maximum(result[inflation_zone], self.inflation_cost)
        
        return cp.asarray(result)
