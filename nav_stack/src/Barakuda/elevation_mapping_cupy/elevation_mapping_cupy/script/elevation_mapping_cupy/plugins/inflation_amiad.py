import cupy as cp
import numpy as np
import scipy.ndimage as scim
from typing import List
from .plugin_manager import PluginBase


class Inflation(PluginBase):
    """
    Distance-based inflation layer for global costmaps.

    - Obstacles are detected from an input layer using a threshold
    - Inflation cost decays exponentially with distance to obstacles
    - Outputs a dedicated cost layer (does NOT modify the input layer)
    """

    def __init__(
        self,
        input_layer_name: str = "csf",
        inflation_radius_m: float = 0.5,     # Inflation radius in meters
        lethal_cost: float = 1.0,             # Cost for obstacle cells
        inflation_cost: float = 0.8,           # Max cost at obstacle boundary
        cost_scaling_factor: float = 10.0,     # Controls decay steepness
        obstacle_threshold: float = 0.5,       # Threshold to detect obstacles
        map_resolution: float = 0.05,          # meters / cell
        **kwargs,
    ):
        super().__init__()
        self.input_layer_name = input_layer_name
        self.inflation_radius_m = inflation_radius_m
        self.lethal_cost = lethal_cost
        self.inflation_cost = inflation_cost
        self.cost_scaling_factor = cost_scaling_factor
        self.obstacle_threshold = obstacle_threshold
        self.map_resolution = map_resolution

        # Precompute inflation radius in cells
        self.inflation_radius_cells = max(
            1, int(self.inflation_radius_m / self.map_resolution)
        )

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

        # Retrieve input layer
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
            # No data → return empty costmap
            return cp.zeros_like(elevation_map[0], dtype=cp.float32)

        # ============== DEBUG PRINTS ==============
        # print(f"[INFLATION] Input layer: {self.input_layer_name}")
        # print(f"[INFLATION] Layer shape: {layer_data.shape}")
        # print(f"[INFLATION] Min value: {cp.nanmin(layer_data)}")
        # print(f"[INFLATION] Max value: {cp.nanmax(layer_data)}")
        # print(f"[INFLATION] Obstacle threshold: {self.obstacle_threshold}")
        
        # layer_np = cp.asnumpy(layer_data)
        # obstacle_mask = (layer_np > self.obstacle_threshold).astype(np.uint8)
        # num_obstacles = np.sum(obstacle_mask)
        # print(f"[INFLATION] Cells above threshold: {num_obstacles}")
        # print(f"[INFLATION] Inflation radius (cells): {self.inflation_radius_cells}")
        # ==========================================

        # Move to CPU for OpenCV processing
        layer_np = cp.asnumpy(layer_data)

        # ------------------------------------------------------------
        # 1. Detect obstacles
        # ------------------------------------------------------------
        obstacle_mask = (layer_np > self.obstacle_threshold).astype(np.uint8)

        # ------------------------------------------------------------
        # 2. Compute distance to nearest obstacle (in cells)
        # ------------------------------------------------------------
        # distanceTransform expects free space = 1, obstacles = 0
        # dist_cells = cv.distanceTransform(
        #     1 - obstacle_mask, cv.DIST_L2, 5
        # )
        dist_cells = scim.distance_transform_edt (1 - obstacle_mask)

        # Convert to meters
        dist_m = dist_cells * self.map_resolution

        # ------------------------------------------------------------
        # 3. Compute inflation costs
        # ------------------------------------------------------------
        costmap = np.zeros_like(layer_np, dtype=np.float32)

        # Lethal obstacles
        costmap[obstacle_mask == 1] = self.lethal_cost

        # Inflation zone
        inflation_zone = (dist_m > 0.0) & (dist_m <= self.inflation_radius_m)

        # Exponential decay (Nav2-style)
        inflation_values = self.inflation_cost * np.exp(
            -self.cost_scaling_factor * dist_m[inflation_zone]
        )

        costmap[inflation_zone] = np.maximum(
            costmap[inflation_zone], inflation_values
        )

        # ------------------------------------------------------------
        # 4. Return as CuPy array
        # ------------------------------------------------------------
        return cp.asarray(costmap)
