import numpy as np
import carb
import random as r
from typing import Tuple

# Import constants relative to the package
from .constants import METERS_PER_UNIT

from perlin_noise import PerlinNoise
try:
    from scipy import interpolate
except ImportError:
    interpolate = None # Handled in GeneratorLogic/Extension startup check


# --- Terrain Data Structure ---
class SubTerrain:
    """Helper class to hold terrain heightfield data and parameters."""
    def __init__(self, width=256, length=256, vertical_scale=1.0, horizontal_scale=1.0):
        self.vertical_scale = vertical_scale
        self.horizontal_scale = horizontal_scale
        self.width = width # Number of vertices along X axis
        self.length = length # Number of vertices along Y axis
        # Raw heightfield data (integer units based on vertical_scale)
        self.height_field_raw = np.zeros((self.width, self.length), dtype=np.int16)


def random_uniform_terrain(terrain: SubTerrain, min_height: float, max_height: float,
                           noise_seed: int, octaves: int = 6, persistence: float = 0.5, lacunarity: float = 2.0) -> SubTerrain:
    """
    Generates terrain heightfield using Perlin noise (Pure Python Version).
    Operates on the terrain.height_field_raw numpy array.
    Heights are relative to 0 in world units.
    Manually implements Fractal Brownian Motion (fBm) using persistence and lacunarity.
    """
    # --- NO CHANGES ABOVE THIS LINE (Except docstring update) ---

    # Convert world height limits to raw integer units based on terrain scaling
    min_h_disc = int(min_height / terrain.vertical_scale)
    max_h_disc = int(max_height / terrain.vertical_scale)

    rows, cols = terrain.width, terrain.length
    if rows < 2 or cols < 2:
        carb.log_warn("Terrain dimensions too small (<2). Generating flat terrain.")
        if terrain.height_field_raw is None or terrain.height_field_raw.shape != (rows, cols):
            terrain.height_field_raw = np.zeros((rows, cols), dtype=np.int16)
        terrain.height_field_raw.fill(0)
        return terrain
    elif terrain.height_field_raw is None or terrain.height_field_raw.shape != (rows, cols):
        terrain.height_field_raw = np.zeros((rows, cols), dtype=np.int16)

    height_field = np.zeros((rows, cols), dtype=np.float64) # Use float64 for noise intermediate

    # Scale noise coordinates based on terrain dimensions and horizontal scale
    noise_scale_x = rows * terrain.horizontal_scale
    noise_scale_y = cols * terrain.horizontal_scale
    max_noise_dim = max(noise_scale_x, noise_scale_y, 1e-6)

    # Clamp parameters for safety, especially persistence and lacunarity
    octaves = max(1, octaves)
    persistence = np.clip(persistence, 0.01, 1.0) # Avoid 0 persistence
    lacunarity = max(1.0, lacunarity) # Must be >= 1

    carb.log_info(f"Generating Perlin noise with seed={noise_seed}, octaves={octaves}, persistence={persistence}, lacunarity={lacunarity} (Manual fBm)")

    # === MODIFICATION START ===

    # 1. Instantiate a SINGLE OCTAVE noise generator instance
    #    We will call this multiple times with scaled coordinates.
    try:
        # Use octaves=1 for the base noise function
        noise_generator_base = PerlinNoise(octaves=1, seed=noise_seed)
        carb.log_info("Instantiated base PerlinNoise(octaves=1) generator.")
    except Exception as e_init:
        carb.log_error(f"Failed to initialize base PerlinNoise: {e_init}. Generating flat terrain.")
        terrain.height_field_raw.fill(0)
        return terrain

    # Calculate maximum possible amplitude for normalization (optional, np.interp handles it too)
    # max_amplitude = sum(persistence ** k for k in range(octaves)) # Not strictly needed if using np.interp later

    # Generate noise value for each vertex by summing octaves manually (fBm)
    for i in range(rows):
        for j in range(cols):
            # Normalize coordinates for the BASE frequency (octave 0)
            nx_base = (i * terrain.horizontal_scale) / max_noise_dim
            ny_base = (j * terrain.horizontal_scale) / max_noise_dim

            total_noise = 0.0
            current_amplitude = 1.0
            current_frequency = 1.0

            # Input validation for base coordinates
            if not (np.isfinite(nx_base) and np.isfinite(ny_base)):
                carb.log_warn(f"Invalid base noise coords: nx={nx_base}, ny={ny_base}. Skipping point.")
                height_field[i, j] = 0.0 # Assign 0 directly and continue outer loop
                continue

            # 2. Loop through octaves to build the fractal noise
            for k in range(octaves):
                # Calculate coordinates for this octave (scaled by frequency)
                nx_octave = nx_base * current_frequency
                ny_octave = ny_base * current_frequency

                try:
                    # Get noise value from the base generator at scaled coordinates
                    noise_val_octave = noise_generator_base([nx_octave, ny_octave])

                    # Add scaled noise to the total
                    total_noise += noise_val_octave * current_amplitude

                except Exception as e:
                    carb.log_error(f"PerlinNoise error at octave {k} ({nx_octave},{ny_octave}): {e}")
                    # Decide how to handle error: skip octave, use 0, etc. Using 0 here.
                    pass # Effectively adds 0 for this octave if error occurs

                # Update amplitude and frequency for the next octave
                current_amplitude *= persistence
                current_frequency *= lacunarity

            # Store the final combined noise value for this point
            height_field[i, j] = total_noise

    # === MODIFICATION END ===

    # Normalize noise output (typically between -1 and 1, but can vary) to the desired height range
    # --- THIS NORMALIZATION LOGIC REMAINS UNCHANGED ---
    min_val, max_val = np.min(height_field), np.max(height_field)
    range_val = max_val - min_val
    if range_val > 1e-6: # Avoid division by zero if noise output is flat
        # Interpolate linearly between min/max noise values to min/max discrete height values
        normalized_height = np.interp(height_field, (min_val, max_val), (min_h_disc, max_h_disc))
    else:
        # If noise is flat, set height to the middle of the desired range
        normalized_height = np.full_like(height_field, (min_h_disc + max_h_disc) / 2.0)

    # Convert to final integer heightfield, rounding appropriately
    # --- THIS FINAL CONVERSION REMAINS UNCHANGED ---
    terrain.height_field_raw = np.rint(normalized_height).astype(np.int16)
    carb.log_info("Heightfield generation complete.")
    return terrain


# --- Mesh Conversion ---
def convert_heightfield_to_trimesh(height_field_raw: np.ndarray, horizontal_scale: float, vertical_scale: float) -> Tuple[np.ndarray, np.ndarray]:
    """
    Converts a raw integer heightfield grid into vertices and triangle indices
    suitable for a USD Mesh.
    Vertices are returned in WORLD units.
    Triangle indices are signed integers (int32).
    """
    if height_field_raw is None or height_field_raw.size == 0:
        return np.array([]).reshape(0, 3), np.array([]).reshape(0, 3)

    hf = height_field_raw
    num_rows, num_cols = hf.shape
    if num_rows < 2 or num_cols < 2:
        carb.log_warn("Heightfield dimensions too small (<2x2) to create triangles.")
        # Return empty arrays if no triangles can be formed
        return np.array([]).reshape(0, 3), np.array([]).reshape(0, 3)

    # Create vertex grid coordinates in world units
    # Linspace generates coordinates for vertices, including endpoints
    y_coords = np.linspace(0.0, (num_cols - 1) * horizontal_scale, num_cols, dtype=np.float32)
    x_coords = np.linspace(0.0, (num_rows - 1) * horizontal_scale, num_rows, dtype=np.float32)
    # Create 2D grids of X and Y coordinates
    yy, xx = np.meshgrid(y_coords, x_coords)

    # Create the vertices array
    num_vertices = num_rows * num_cols
    vertices = np.zeros((num_vertices, 3), dtype=np.float32)
    vertices[:, 0] = xx.flatten() # X coordinates
    vertices[:, 1] = yy.flatten() # Y coordinates
    # Z coordinates from heightfield, scaled by vertical_scale to get world height
    vertices[:, 2] = hf.flatten().astype(np.float32) * vertical_scale

    # Calculate number of triangles (2 per quad in the grid)
    num_quad_rows = num_rows - 1
    num_quad_cols = num_cols - 1
    num_triangles = 2 * num_quad_rows * num_quad_cols

    # Pre-allocate triangles array with correct signed integer type
    triangles = np.zeros((num_triangles, 3), dtype=np.int32) # Use int32 for USD compatibility

    # Iterate through grid quads to create triangle pairs
    tris_idx = 0
    for i in range(num_quad_rows):
        for j in range(num_quad_cols):
            # Calculate vertex indices for the current quad
            # Base index for the top-left vertex of the quad
            base_idx = j + i * num_cols
            # Indices: 0=TL, 1=TR, 2=BL, 3=BR
            idx0 = base_idx
            idx1 = base_idx + 1
            idx2 = base_idx + num_cols
            idx3 = base_idx + num_cols + 1

            # Check if indices are within bounds (should be, but safe)
            if idx3 >= num_vertices:
                carb.log_error(f"Calculated index {idx3} exceeds vertex count {num_vertices} during triangulation. Stopping.")
                # Resize triangles to only include valid ones created so far
                return vertices / METERS_PER_UNIT, triangles[:tris_idx]

            # Create two triangles for the quad (ensure consistent winding order, e.g., counter-clockwise)
            # Triangle 1: Top-left, Bottom-right, Top-right (0, 3, 1)
            triangles[tris_idx] = [idx0, idx3, idx1]
            # Triangle 2: Top-left, Bottom-left, Bottom-right (0, 2, 3)
            triangles[tris_idx + 1] = [idx0, idx2, idx3]
            tris_idx += 2

    # Convert vertex positions from local grid units to world units (if needed)
    # Currently assumes 0,0 of grid corresponds to desired world origin offset
    # Return vertices scaled based on METERS_PER_UNIT for USD stage
    return vertices / METERS_PER_UNIT, triangles