import os
import math
from typing import Optional, Tuple, Dict, Any

import carb
import omni.kit.app
from isaacsim.core.utils.stage import get_current_stage
from isaacsim.core.utils.prims import is_prim_path_valid
from omni.physx import get_physx_scene_query_interface

from pxr import Gf, UsdGeom, Sdf, Usd

# Import constants relative to the package
from .constants import METERS_PER_UNIT

# --- General Utilities ---

def get_extension_path(ext_id: str) -> Optional[str]:
    """Gets the filesystem path of the extension."""
    ext_mgr = omni.kit.app.get_app().get_extension_manager()
    if not ext_mgr:
        carb.log_error("Extension Manager not found.")
        return None
    return ext_mgr.get_extension_path(ext_id)

def get_asset_path(ext_path: str, asset_filename: str) -> Optional[str]:
    """Gets the absolute path to an asset within the extension's data directory."""
    if not ext_path:
        carb.log_error("Extension path not provided for asset path resolution.")
        return None
    path = os.path.join(ext_path, "data", asset_filename)
    if not os.path.exists(path):
        carb.log_warn(f"Asset file not found at expected path: {path}")
        return None
    return path

# --- Math / Transform Utilities ---

def convert_euler_to_quat(roll_deg: float, pitch_deg: float, yaw_deg: float) -> Gf.Quatd:
    """Converts Euler angles (degrees, XYZ order) to Gf.Quatd (scalar first: w, x, y, z)."""
    roll, pitch, yaw = math.radians(roll_deg), math.radians(pitch_deg), math.radians(yaw_deg)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return Gf.Quatd(qw, qx, qy, qz) # Note: Gf.Quatd constructor takes (real, vec3) or (w,x,y,z)

def quat_to_euler_angles(quat: Gf.Quatd) -> Tuple[float, float, float]:
    """Converts a Gf.Quatd to Euler angles (roll, pitch, yaw) in degrees (XYZ order)."""
    w = quat.GetReal()
    x, y, z = quat.GetImaginary()
    # Compute Euler angles in radians
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = max(-1.0, min(1.0, t2))  # Clamp to [-1, 1] due to potential float inaccuracies
    pitch = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)

    # Convert radians to degrees
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)

def apply_transform_xform_api(prim_path: str, pos: Gf.Vec3d, rot_quat: Gf.Quatd, scale: Gf.Vec3d):
    """
    Applies transform properties using UsdGeom.XformCommonAPI.
    Takes position/scale in world units, converts position to scene units.
    Rotation is applied using Euler angles derived from the input quaternion.
    """
    stage = get_current_stage()
    if not stage: return
    prim = stage.GetPrimAtPath(prim_path)
    if not prim: return

    try:
        xform_api = UsdGeom.XformCommonAPI(prim)
        # Apply translation (convert world units to scene units)
        xform_api.SetTranslate(pos / METERS_PER_UNIT)

        # Apply rotation (API expects Euler angles)
        roll, pitch, yaw = quat_to_euler_angles(rot_quat)
        rotation_euler_deg = Gf.Vec3f(roll, pitch, yaw)
        xform_api.SetRotate(rotation_euler_deg, UsdGeom.XformCommonAPI.RotationOrderXYZ)

        # Apply scale (assumed to be uniform scaling factor or specific vec)
        xform_api.SetScale(Gf.Vec3f(scale[0], scale[1], scale[2]))

    except Exception as e:
        carb.log_error(f"Failed to apply transform to {prim_path}: {e}")


# --- Physics Utilities ---

def check_raycast_down(origin_world: Gf.Vec3d, max_distance: float = 10000.0) -> Tuple[bool, float, Gf.Vec3d, Gf.Vec3d]:
    """
    Projects a raycast downwards in world coordinates.
    Returns: (hit_success, distance_in_world_units, hit_position_world, hit_normal_world).
    Returns default values far below origin on failure.
    """
    # Default return values adjusted to be in world units
    default_pos = Gf.Vec3d(origin_world[0], origin_world[1], origin_world[2] - max_distance)
    default_normal = Gf.Vec3d(0, 0, 1)

    try:
        physx_query_interface = get_physx_scene_query_interface()
        if physx_query_interface is None:
            carb.log_warn("PhysX query interface not available for raycast.")
            return False, max_distance, default_pos, default_normal

        # PhysX interface expects scene units
        origin_scene = origin_world / METERS_PER_UNIT
        max_dist_scene = max_distance / METERS_PER_UNIT

        rayDir = carb.Float3(0.0, 0.0, -1.0)
        carb_origin = carb.Float3(origin_scene[0], origin_scene[1], origin_scene[2])

        hit = physx_query_interface.raycast_closest(carb_origin, rayDir, max_dist_scene)

        if hit and hit["hit"]:
            # Convert results back to world units
            hit_pos_scene = Gf.Vec3d(hit["position"][0], hit["position"][1], hit["position"][2])
            hit_pos_world = hit_pos_scene * METERS_PER_UNIT
            # Normal is a direction vector, doesn't need unit conversion, but ensure normalization
            hit_normal_world = Gf.Vec3d(hit["normal"][0], hit["normal"][1], hit["normal"][2]).GetNormalized()
            distance_world = hit["distance"] * METERS_PER_UNIT
            return True, distance_world, hit_pos_world, hit_normal_world
        else:
            return False, max_distance, default_pos, default_normal
    except Exception as e:
        carb.log_error(f"Exception during raycast: {e}")
        return False, max_distance, default_pos, default_normal

def get_height_and_normal_from_noise_data(world_x: float, world_y: float, terrain_info: Dict[str, Any]) -> Tuple[Optional[float], Optional[Gf.Vec3d]]:
    """
    Calculates terrain height and surface normal at world (x, y) using bilinear interpolation
    on the raw heightfield data.

    Args:
        world_x: World X coordinate.
        world_y: World Y coordinate.
        terrain_info: Dictionary containing:
            'height_field_raw': (np.ndarray) The raw height data grid.
            'h_scale': (float) Horizontal grid spacing (world units).
            'v_scale': (float) Vertical scaling factor (world units per raw unit).
            'area_x': (float) Total width of the terrain area.
            'area_y': (float) Total length of the terrain area.

    Returns:
        A tuple (world_z, normal_vector). Returns (None, None) on error or if out of bounds.
        world_z is the interpolated height in world units.
        normal_vector is a normalized Gf.Vec3d representing the surface normal.
    """
    hf_raw = terrain_info.get("height_field_raw")
    h_scale = terrain_info.get("h_scale")
    v_scale = terrain_info.get("v_scale")
    area_x = terrain_info.get("area_x")
    area_y = terrain_info.get("area_y")

    if hf_raw is None or h_scale is None or v_scale is None or area_x is None or area_y is None or h_scale <= 1e-6:
        carb.log_error("Missing terrain info for height/normal calculation.")
        return None, None

    rows, cols = hf_raw.shape

    # Convert world coordinates to local terrain grid coordinates
    # Terrain origin in world is (-area_x/2, -area_y/2)
    local_x = world_x + area_x / 2.0
    local_y = world_y + area_y / 2.0

    # Convert local coordinates to fractional grid indices
    i_frac = local_x / h_scale
    j_frac = local_y / h_scale

    # Get integer indices and interpolation weights
    i0 = math.floor(i_frac)
    j0 = math.floor(j_frac)
    u = i_frac - i0
    v = j_frac - j0

    # Check bounds (allow interpolation slightly outside the strict 0..N-1 range if needed, e.g. i0 == rows-1)
    if not (0 <= i0 < rows and 0 <= j0 < cols):
        # carb.log_warn(f"Coordinates ({world_x:.2f}, {world_y:.2f}) -> indices ({i0}, {j0}) are out of terrain bounds ({rows}x{cols}).")
        # Handle out of bounds: clamp or return error? Let's return error for now.
        return None, None

    # Clamp indices for safety when accessing neighbors
    i0_c = max(0, min(i0, rows - 2)) # Clamp to ensure i1 is valid
    j0_c = max(0, min(j0, cols - 2)) # Clamp to ensure j1 is valid
    i1_c = i0_c + 1
    j1_c = j0_c + 1

    # Get heights of the 4 surrounding points (raw units)
    try:
        h00 = float(hf_raw[i0_c,   j0_c])
        h10 = float(hf_raw[i1_c,   j0_c])
        h01 = float(hf_raw[i0_c,   j1_c])
        h11 = float(hf_raw[i1_c,   j1_c])
    except IndexError:
        carb.log_error(f"IndexError accessing heightfield at ({i0_c}-{i1_c}, {j0_c}-{j1_c}) for input ({world_x:.2f}, {world_y:.2f})")
        return None, None

    # Bilinear interpolation for height
    h_raw = (1.0 - u) * (1.0 - v) * h00 + u * (1.0 - v) * h10 + (1.0 - u) * v * h01 + u * v * h11
    world_z = h_raw * v_scale

    # Calculate Normal using finite differences (simplified)
    # Using the difference between adjacent grid points around the interpolated point
    dz_di = (h10 - h00) # Change in raw height per grid step in i
    dz_dj = (h01 - h00) # Change in raw height per grid step in j

    # Convert to world gradients
    grad_x = dz_di * v_scale / h_scale
    grad_y = dz_dj * v_scale / h_scale

    # Construct normal vector (unnormalized) - based on cross product derivation
    # Normal = (-grad_x, -grad_y, 1) * some_scaling_factor (h_scale in earlier derivation)
    # Let's use the simplified (-vx, -vy, 1) and normalize
    normal_unnormalized = Gf.Vec3d(-grad_x, -grad_y, 1.0)
    normal_vector = normal_unnormalized.GetNormalized()

    return world_z, normal_vector

# --- USD Utilities ---

def _ensure_prim_exists(stage: Usd.Stage, prim_path: str, type_name: str = "Xform"):
    """Ensures a prim exists at the path, creating ancestors if needed."""
    if not stage: return None
    path = Sdf.Path(prim_path)
    if stage.GetPrimAtPath(path).IsValid():
        return stage.GetPrimAtPath(path) # Already exists

    # Walk up to find the nearest existing ancestor
    current = path
    missing_paths = []
    while not stage.GetPrimAtPath(current).IsValid() and current != Sdf.Path.absoluteRootPath:
        missing_paths.append(current)
        current = current.GetParentPath()

    # Check if the nearest ancestor exists
    if not stage.GetPrimAtPath(current).IsValid() and current != Sdf.Path.absoluteRootPath:
        carb.log_error(f"Could not find valid ancestor path for {prim_path} starting from {current}")
        return None # Should not happen unless root is invalid

    # Create missing prims from ancestor down to target
    carb.log_info(f"Creating missing prims for {prim_path}: {missing_paths}")
    prim = None # Initialize prim variable
    for p in reversed(missing_paths):
        try:
            # Define using the specified type only for the final prim, use Xform for parents
            define_type = type_name if p == path else "Xform"
            prim = stage.DefinePrim(p, define_type)
            if not prim:
                carb.log_error(f"Failed to define prim at {p}")
                return None # Stop if creation fails
        except Exception as e:
            carb.log_error(f"Exception defining prim {p}: {e}")
            return None

    return prim # Return the final created prim


def find_collision_target_prim(start_prim: Usd.Prim) -> Optional[Usd.Prim]:
    """
    Searches the hierarchy under start_prim for a suitable collision target.

    Priority:
    1. First prim found whose name contains "trunk" (case-insensitive).
    2. If no "trunk" found, the first prim of type UsdGeom.Mesh encountered.
    3. If neither found, returns None.

    Args:
        start_prim: The root prim of the hierarchy to search (e.g., the copied instance root).

    Returns:
        The found Usd.Prim target, or None if neither priority target is found.
    """
    if not start_prim or not start_prim.IsValid():
        return None

    stack = [start_prim]  # Use a stack for iterative depth-first traversal
    visited = set()
    first_mesh_prim = None

    while stack:
        current_prim = stack.pop()

        if not current_prim or current_prim in visited:
            continue
        visited.add(current_prim)

        # Priority 1: Check for "trunk" in name (case-insensitive)
        prim_name_lower = current_prim.GetName().lower()
        if "trunk" in prim_name_lower:
            return current_prim  # Found highest priority target

        # Priority 2: Check if it's the first Mesh encountered
        # We only store it, but continue searching for a "trunk"
        if first_mesh_prim is None and current_prim.IsA(UsdGeom.Mesh):
            first_mesh_prim = current_prim

        # Add children to the stack for further processing
        # Process children in reverse to maintain a more intuitive DFS order if needed
        children = current_prim.GetChildren()
        for child in reversed(children):
            if child not in visited:
                stack.append(child)

    # If loop finished, no "trunk" was found. Return the first mesh if one was seen.
    if first_mesh_prim:
        return first_mesh_prim
    else:
        return None # Indicate nothing suitable was found