import cv2
import torch
import numpy as np
import matplotlib
matplotlib.use('Agg')  # Set backend before importing pyplot
import matplotlib.pyplot as plt

import rospy

# export PYTHONPATH="${PYTHONPATH}:/workspace/barakuda_isaac_sim/mono_depth_ws/src/depth_rescaling/src/depth_estimation/depth_models/depth_anything/Depth-Anything-V2/"
from depth_anything_v2.dpt import DepthAnythingV2

class Depth_AnythingV2:
    def __init__(self, device=None):
        device = 'cuda' if torch.cuda.is_available() else 'mps' if torch.backends.mps.is_available() else 'cpu'
        model_name = rospy.get_param('~engine_path')
        model_configs = {
            'vits': {'encoder': 'vits', 'features': 64, 'out_channels': [48, 96, 192, 384]},
            'vitb': {'encoder': 'vitb', 'features': 128, 'out_channels': [96, 192, 384, 768]},
            'vitl': {'encoder': 'vitl', 'features': 256, 'out_channels': [256, 512, 1024, 1024]},
            'vitg': {'encoder': 'vitg', 'features': 384, 'out_channels': [1536, 1536, 1536, 1536]}
        }

        encoder = 'vits' # 'vits', 'vitb' or 'vitl'
        print(encoder)

        self.model = DepthAnythingV2(**model_configs[encoder])
        self.model.load_state_dict(torch.load(f'/workspace/Depth-Anything-V2/checkpoints/depth_anything_v2_{encoder}.pth', map_location='cpu'))
        self.model = self.model.to(device).eval()
        self.cmap = plt.get_cmap('Spectral_r')
        self.color=False

    def __call__(self, image):
        depth = self.model.infer_image(image) # HxW raw depth map in numpy
        orig_w, orig_h = image.shape[:2]
        # print(depth.shape)
        depth = cv2.resize(depth, (orig_h, orig_w))
        depth = (depth - depth.min()) / (depth.max() - depth.min()) * 255.0
        depth = depth.astype(np.uint8)
        if self.color:
            depth = (self.cmap(depth)[:, :, :3] * 255)[:, :, ::-1].astype(np.uint8)
        return depth