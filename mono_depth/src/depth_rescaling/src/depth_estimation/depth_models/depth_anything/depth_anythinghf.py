import torch
import numpy as np
from PIL import Image
from transformers import pipeline
import cv2

import rospy

class Depth_AnythingHF:
    def __init__(self, device=None):
        """
        Initialize the DepthAnything model.
        
        Args:
            model_name (str): The model name/path.
            device (str): Device to run inference on ('cpu', 'cuda', or None for auto)
        """
        self.device = device if device else ("cuda" if torch.cuda.is_available() else "cpu")
        
        model_name = rospy.get_param('engine_path')

        # Initialize the depth estimation pipeline
        self.pipe = pipeline(
            task="depth-estimation",
            model=model_name,
            device=0 if self.device == "cuda" else -1
        )
        
        print(f"DepthAnything model loaded on {self.device}")
        
    def __call__(self, image):
        """
        Estimate depth from an input image.
        
        Args:
            image: Input image. Can be:
                - PIL Image
                - numpy array (H, W, 3) in RGB format
                - string path to image file
                
        Returns:
            dict: Dictionary containing:
                - 'depth': numpy array of depth values (H, W)
                - 'predicted_depth': PIL Image of depth visualization
        """
        # Handle different input types
        if isinstance(image, str):
            # Load image from file path
            image = Image.open(image).convert('RGB')
        elif isinstance(image, np.ndarray):
            # Convert numpy array to PIL Image
            if image.dtype != np.uint8:
                image = (image * 255).astype(np.uint8)
            image = Image.fromarray(image)
        elif not isinstance(image, Image.Image):
            raise TypeError("Input must be PIL Image, numpy array, or file path string")
        
        # Ensure image is in RGB format
        if image.mode != 'RGB':
            image = image.convert('RGB')
        
        # Run depth estimation
        result = self.pipe(image)
        
        # Extract depth information
        depth_image = result["predicted_depth"]  # PIL Image
        depth_array = np.array(result["depth"])   # numpy array
        
        return {
            'depth': depth_array,
            'predicted_depth': depth_image,
            'original_size': image.size
        }
    
    def save_depth(self, depth_result, output_path, format='png'):
        """
        Save the depth estimation result.
        
        Args:
            depth_result: Result from __call__ method
            output_path (str): Path to save the depth image
            format (str): Image format ('png', 'jpg', etc.)
        """
        depth_result['predicted_depth'].save(output_path, format=format.upper())
        print(f"Depth image saved to {output_path}")
    
    def get_depth_at_point(self, depth_result, x, y):
        """
        Get depth value at a specific pixel coordinate.
        
        Args:
            depth_result: Result from __call__ method
            x, y (int): Pixel coordinates
            
        Returns:
            float: Depth value at the specified point
        """
        depth_array = depth_result['depth']
        if 0 <= y < depth_array.shape[0] and 0 <= x < depth_array.shape[1]:
            return float(depth_array[y, x])
        else:
            raise ValueError(f"Coordinates ({x}, {y}) are out of bounds for image size {depth_array.shape}")

# Example usage
if __name__ == "__main__":
    # Initialize the depth estimator
    depth_estimator = DepthAnything()
    
    # Example with image file path
    # result = depth_estimator("path/to/your/image.jpg")
    
    # Example with PIL Image
    # from PIL import Image
    # img = Image.open("path/to/your/image.jpg")
    # result = depth_estimator(img)
    
    # Example with numpy array
    # img_array = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
    # result = depth_estimator(img_array)
    
    # Access results
    # depth_map = result['depth']  # numpy array with depth values
    # depth_visualization = result['predicted_depth']  # PIL Image for visualization
    
    # Save depth image
    # depth_estimator.save_depth(result, "output_depth.png")
    
    # Get depth at specific point
    # depth_value = depth_estimator.get_depth_at_point(result, x=100, y=200)
    
    print("DepthAnything class is ready to use!")