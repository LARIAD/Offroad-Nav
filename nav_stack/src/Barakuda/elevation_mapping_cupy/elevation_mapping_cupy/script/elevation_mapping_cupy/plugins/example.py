import cupy as cp
from typing import List
from .plugin_manager import PluginBase
import cupyx.scipy.ndimage as ndimage


class MaskInvalid(PluginBase):
    def __init__(self, input_layer_name="smooth", add_value: float = 1.0, **kwargs):
        super().__init__()
        self.input_layer_name = "smooth"
        self.add_value = float(add_value)
        self.default_layer_name = "smooth"
        self.flag = 1

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
        idx = plugin_layer_names.index(self.input_layer_name)
        h = plugin_layers[idx]
        mask = cp.asnumpy((elevation_map[2] < 0.5).astype("uint8"))
        h[mask] = cp.nan
        return h
