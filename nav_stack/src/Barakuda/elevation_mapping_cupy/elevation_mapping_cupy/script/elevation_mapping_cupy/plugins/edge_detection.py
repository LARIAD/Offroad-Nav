import cupy as cp
from typing import List
from .plugin_manager import PluginBase
import cupyx.scipy.ndimage as ndimage
import cv2

class EdgeDetection(PluginBase):
    def __init__(self, input_layer_name, algo, sigma=1, min_h = 0.0, **kwargs):
        super().__init__()
        self.input_layer_name = input_layer_name
        self.possible_types = ['sobel', 'prewitt', 'laplace', 'gaussian_laplace']
        self.default_algo = "sobel"
        self.flag = 1
        self.sigma = sigma
        self.algo = algo
        self.min_h = min_h
        self.thresh = 0.0

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
            hs1 = (hs1>=self.thresh) * (h>=self.min_h)
            return hs1
        elif self.algo.lower() == "gaussian_laplace":
            hs1 = cp.absolute(ndimage.gaussian_laplace(h, self.sigma))
            hs1 /= cp.max(hs1)
            hs1 = (hs1>=self.thresh) * (h>=self.min_h)
            return hs1
        # Convert cupy array to numpy array and normalize to 0-255
        # h_numpy = cv2.normalize(cp.asnumpy(h), None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)

        # # Display the image
        # cv2.imshow('Image', h_numpy)
        # cv2.waitKey(1)  # This will refresh the image for every call 
        hs1 = cp.sqrt(x**2 + y**2)
        hs1 /= cp.max(hs1)
        # hs1 = (hs1>=self.thresh) #* (h>=self.min_h)
        return hs1
