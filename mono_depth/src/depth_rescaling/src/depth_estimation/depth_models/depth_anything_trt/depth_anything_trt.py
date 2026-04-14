import argparse
import os
import cv2
import numpy as np
import pycuda.autoinit
import pycuda.driver as cuda
import tensorrt as trt
from .util.transform import load_image

import rospy

class DepthAnythingTRT:
    def __init__(self):
        self.cfx = cuda.Device(0).make_context()

        logger = trt.Logger(trt.Logger.WARNING)
        with open(rospy.get_param('engine_path'), 'rb') as f, trt.Runtime(logger) as runtime:
            engine = runtime.deserialize_cuda_engine(f.read())

        self.context = engine.create_execution_context()
        input_shape = self.context.get_tensor_shape('input')
        self.output_shape = self.context.get_tensor_shape('output')
        self.h_input = cuda.pagelocked_empty(trt.volume(input_shape), dtype=np.float32)
        self.h_output = cuda.pagelocked_empty(trt.volume(self.output_shape), dtype=np.float32)
        self.d_input = cuda.mem_alloc(self.h_input.nbytes)
        self.d_output = cuda.mem_alloc(self.h_output.nbytes)
        self.stream = cuda.Stream()

    def __call__(self, image):
        self.cfx.push()

        input_image, (orig_h, orig_w) = load_image(image)
            
        # Copy the input image to the pagelocked memory
        np.copyto(self.h_input, input_image.ravel())
        
        # Copy the input to the GPU, execute the inference, and copy the output back to the CPU
        cuda.memcpy_htod_async(self.d_input, self.h_input, self.stream)
        self.context.execute_async_v2(bindings=[int(self.d_input), int(self.d_output)], stream_handle=self.stream.handle)
        cuda.memcpy_dtoh_async(self.h_output, self.d_output, self.stream)
        self.stream.synchronize()
        depth = self.h_output

        self.cfx.pop()
        
        # Process the depth output
        depth = np.reshape(depth, self.output_shape[1:])
        depth = cv2.resize(depth, (orig_w, orig_h))
        # depth = (depth - depth.min()) / (depth.max() - depth.min()) * 255.0
        # depth = depth.astype(np.uint8)

        # colored_depth = cv2.applyColorMap(depth, cv2.COLORMAP_INFERNO)
        # cv2.imwrite(f'/workspace/my_catkin_ws/src/depth.png', colored_depth)

        return depth