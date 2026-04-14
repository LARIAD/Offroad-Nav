import cupy as cp
from typing import List
from .plugin_manager import PluginBase
import cupyx.scipy.ndimage as ndimage


class EdgeDetection(PluginBase):
    def __init__(self, input_layer_name, algo, sigma=1,**kwargs):
        super().__init__()
        self.input_layer_name = input_layer_name
        self.possible_types = ['sobel', 'prewitt', 'laplace', 'gaussian_laplace']
        self.default_algo = "sobel"
        self.flag = 1
        self.sigma = sigma
        self.algo = algo
        self.thresh = 0.1

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
        
        if self.input_layer_name in layer_names:
            idx = layer_names.index(self.input_layer_name)
            h = elevation_map[idx]
        elif self.input_layer_name in plugin_layer_names:
            idx = plugin_layer_names.index(self.input_layer_name)
            h = plugin_layers[idx]
        elif self.input_layer_name == 'csf':
            h=semantic_map[0]
        else:
            print(
                "layer name {} was not found. Using elevation layer.".format(
                    self.input_layer_name
                )
            )
            h = elevation_map[0]
        if self.algo.lower() not in self.possible_types:
            print(f"Undefined edge detection algo. Defaulting to {self.default_algo}.")
            self.algo = self.default_algo

        if self.algo.lower() == "sobel":
            x, y = ndimage.sobel(h, axis = 0, mode="nearest"), ndimage.sobel(h, axis = 1, mode="nearest")
        elif self.algo.lower() == "prewitt":
            x, y = ndimage.prewitt(h, axis = 0, mode="nearest"), ndimage.prewitt(h, axis = 1, mode="nearest")
        elif self.algo.lower() == "laplace":
            hs1 = cp.absolute(ndimage.laplace(h))
            hs1 /= cp.max(hs1)
            hs1 = hs1>=self.thresh
            return hs1
        elif self.algo.lower() == "gaussian_laplace":
            hs1 = cp.absolute(ndimage.gaussian_laplace(h, self.sigma))
            hs1 /= cp.max(hs1)
            hs1 = hs1>=self.thresh
            return hs1
    
        hs1 = cp.sqrt(x**2 + y**2)
        hs1 /= cp.max(hs1)
        hs1 = hs1>=self.thresh
        return hs1
