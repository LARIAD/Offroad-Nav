import os
import threading
import time
import traceback
import gc
import random as r
import numpy as np
from typing import Optional, Dict, Any, Tuple, List

import carb
import omni.kit.app
import omni.usd
import omni.graph.core as og
from isaacsim.core.utils.stage import get_stage_units
from isaacsim.core.prims import XFormPrim
from isaacsim.core.utils.prims import create_prim, delete_prim, is_prim_path_valid, get_prim_at_path
from isaacsim.core.utils.stage import get_current_stage, add_reference_to_stage
from pxr import Gf, Usd, UsdGeom, UsdLux, UsdPhysics, UsdShade, PhysxSchema, Sdf, Vt, Tf, Kind

# Import relative utils, terrain, constants
from . import utils
from . import terrain
from . import constants

class GeneratorLogic:
    """Handles the core logic for generating the forest environment."""

    def __init__(self, stage: Usd.Stage, ext_path: str):
        """
        Initialize the GeneratorLogic.

        Args:
            stage: The current USD stage.
            ext_path: Filesystem path to the extension.
        """
        self._ui = None # Initialize as None, will be set later by extension
        self._stage = stage
        self._ext_path = ext_path

        # Store active push subscriptions that need manual cleanup
        self._active_push_subscriptions = []
        # State for managing asynchronous tasks
        self._generation_lock = threading.Lock()
        self._pending_generation_tasks = 0
        self._current_terrain_info: Optional[Dict] = None # Add this if not present
        self._temp_params_for_objs: Optional[Dict] = None # To store params temporarily

        self._task_definitions = {
            # Map Task Name -> Dictionary containing 'param' func and 'bg' func
            "Terrain": {
                "param": self._get_terrain_params,
                "bg": self._generate_terrain_background
            },
            "Forest": {
                "param": self._get_forest_params,
                "bg": self._generate_placements_background
            },
            "Rocks": {
                "param": self._get_rock_params,
                "bg": self._generate_placements_background
            },
            "Vegetation": {
                 "param": self._get_veg_params,
                 "bg": self._generate_placements_background
            },
            "Objects": {
                "param": self._get_objects_params,
                "bg": self._generate_placements_background
            },
        }

        self._asset_configs: Dict[str, Dict[str, Any]] = {
            # Define asset configurations (relative paths within data/)
            "Birch": {"file": "Gray_Birch/Gray_Birch.usd", "base_scale": 0.01, "height_mult": 1.35},
            "Spruce": {"file": "Norway_Spruce/Norway_Spruce.usd", "base_scale": 0.01, "height_mult": 1.25},
            "Pine": {"file": "Douglas_Fir/Douglas_Fir.usd", "base_scale": 0.01, "height_mult": 1.35},
            "Rock": {"file": "Rock_obj/Rock.usd", "base_scale": 1.0},
            "Blueberry": {"file": "Blueberry_obj/Blueberry.usd"},
            "Bush": {"file": "Bush_obj/Bush.usd"},
            "Grass": {"file": "Grass_Short_B/Grass_Short_B.usd", "base_scale": 0.01,},
            "Switchgrass": {"file": "Switchgrass/Switchgrass.usd", "base_scale": 0.01,},
            "Container": {"file": "Container_J01/Container_J01_126x120x133cm_PR_V_NVD_01.usd", "base_scale": 0.01,},
        }
        self._resolve_asset_paths() # Resolve full paths on init

        # Stores: asset_usd_path -> prototype_prim_path
        self._prototype_cache: Dict[str, str] = {}
        # Stores: asset_usd_path -> instancer_prim_path (Optional, but can be useful)
        self._instancer_cache: Dict[str, str] = {}

    def cleanup(self):
        """Clean up resources like subscriptions."""
        carb.log_info("GeneratorLogic cleanup...")
        self._active_push_subscriptions.clear()
        self._prototype_cache.clear() # Clear caches on cleanup
        self._instancer_cache.clear()
        self._stage = None
        self._ui = None
        self._current_terrain_info = None
        self._temp_params_for_objs = None
        carb.log_info("GeneratorLogic cleanup complete.")

    # --- Asset Path Resolution ---
    def _resolve_asset_paths(self):
        """Resolves relative asset paths to full paths."""
        if not self._ext_path:
            carb.log_error("Extension path not set, cannot resolve asset paths.")
            return
        carb.log_info("Resolving asset paths...")
        for key, config in self._asset_configs.items():
            if "file" in config:
                full_path = utils.get_asset_path(self._ext_path, config["file"])

                config["full_path"] = full_path # Store None if not found
                if full_path:
                    carb.log_info(f"  Resolved '{key}': {full_path}")
                else:
                    carb.log_warn(f"  Asset '{key}' not found at expected location.")

    # --- Asynchronous Scheduling Helpers ---
    def _run_on_main_thread(self, func):
        """Schedules func on main thread using push subscription workaround."""
        main_schedule_log = "[_run_on_main_thread]"
        func_name = getattr(func, '__name__', repr(func))
        carb.log_info(f"{main_schedule_log} Attempting to schedule (push): {func_name}")
        try:
            stream = omni.kit.app.get_app().get_update_event_stream()
            sub_holder = {"sub": None}

            def execute_callback_once(e, function_to_run=func, self_ref=self):
                current_sub = sub_holder["sub"]
                if current_sub:
                    if current_sub in self_ref._active_push_subscriptions:
                        self_ref._active_push_subscriptions.remove(current_sub)
                    sub_holder["sub"] = None
                else: return # Already run or invalid

                self_ref._log_and_run_scheduled(function_to_run)

            subscription = stream.create_subscription_to_push(
                execute_callback_once, name=f"RunOncePushClosure_{func_name}" )

            if subscription:
                sub_holder["sub"] = subscription
                self._active_push_subscriptions.append(subscription)
                carb.log_info(f"{main_schedule_log} PUSH Subscription created for {func_name}. List size: {len(self._active_push_subscriptions)}")
            else: carb.log_error(f"{main_schedule_log} create_subscription_to_push failed for {func_name}!")

        except Exception as e:
            carb.log_error(f"{main_schedule_log} FAILED TO SCHEDULE {func_name}: {e}")
            traceback.print_exc()
            if func == self._generation_finished_main_thread:
                carb.log_error(f"{main_schedule_log} Failed to schedule FINAL cleanup. Attempting direct call.")
                try: self._generation_finished_main_thread(error=True)
                except Exception as ce: carb.log_error(f"Direct cleanup call failed: {ce}")

    def _log_and_run_scheduled(self, func_to_run):
        """Logs before/after executing the function scheduled on the main thread."""
        func_name = getattr(func_to_run, '__name__', repr(func_to_run))
        carb.log_info(f"[MainThread EXECUTE] >>> Running scheduled: {func_name}")
        try:
            func_to_run()
            carb.log_info(f"[MainThread EXECUTE] <<< Finished scheduled successfully: {func_name}")
        except Exception as e:
            carb.log_error(f"[MainThread EXECUTE] !!! Error during {func_name}: {e}")
            traceback.print_exc()
            if func_to_run == self._generation_finished_main_thread:
                carb.log_error("Error occurred within the final cleanup itself!")
            carb.log_info(f"[MainThread EXECUTE] <<< Finished scheduled with ERROR: {func_name}")


    # --- Core Actions Triggered by UI ---

    def start_generation(self):
        """
        Starts the ASYNCHRONOUS generation process:
        1. Gathers all parameters synchronously.
        2. Clears previous assets synchronously.
        3. Launches Terrain background thread.
        4. Terrain main thread update stores terrain data and launches object threads.
        5. Object background threads calculate poses using stored terrain data.
        6. Object main thread updates add objects to the stage.
        """
        log_prefix = "[start_generation ASYNC+NoiseHeight v3]"

        # --- 1. Pre-checks and UI Disable ---
        if self._ui and self._ui._models.get("generate_all_button") and not self._ui._models["generate_all_button"].enabled:
            carb.log_warn(f"{log_prefix} Generation already in progress."); return

        if self._ui:
            self._ui.set_generate_button_enabled(False)
            carb.log_info(f"{log_prefix} Generate button disabled.")
        else:
            carb.log_warn(f"{log_prefix} UI proxy invalid, cannot disable button.")

        if not self._stage:
            carb.log_error(f"{log_prefix} Stage not available.")
            if self._ui: self._ui.set_generate_button_enabled(True) # Re-enable if stage invalid
            return # Exit early

        # --- 2. Reset Internal State ---
        self._current_terrain_info = None
        self._temp_params_for_objs = {} # Reset temp storage
        self._prototype_cache = {}

        # --- 3. Synchronous Setup (Keep minimal) ---
        parent_path = self._ui._get_value("parent_path", "str", constants.DEFAULT_PARENT_PATH) if self._ui else constants.DEFAULT_PARENT_PATH
        # Ensuring parent exists is usually fast and needed before param gathering
        utils._ensure_prim_exists(self._stage, parent_path)

        global_seed = self._ui._get_value("global_seed", "int", 0) if self._ui else 0
        if global_seed > 0: r.seed(global_seed); np.random.seed(global_seed)

        carb.log_info(f"--- [{constants.EXTENSION_NAME}] Starting Full Generation Process ---")

        # --- 4. Clear Previous Assets (Synchronous) ---
        carb.log_info(f"{log_prefix} Clearing previous assets...")
        self.clear_all_generated_assets(log_info=False) # Calls _setup_physics_scene internally
        # REMOVED: omni.kit.app.get_app().update(); time.sleep(0.2) # Removed potentially problematic sync delay
        carb.log_info(f"{log_prefix} Asset clearing finished.")

        carb.log_info(f"{log_prefix} Loading/Updating HDRI Dome Light...")
        try:
            self.load_update_hdr() # Call the function directly here
            carb.log_info(f"{log_prefix} HDRI Dome Light processed.")
        except Exception as e_hdr:
            # Log error but continue generation if HDRI fails
            carb.log_error(f"{log_prefix} Failed to load/update HDRI: {e_hdr}")
            traceback.print_exc()

        # --- 6. Gather ALL Parameters (Synchronous) ---
        # This part calls _ensure_prim_exists for category parents, keep sync
        num_tasks_to_run = 0
        carb.log_info(f"{log_prefix} Gathering parameters...")
        for name, task_info in self._task_definitions.items():
            param_func = task_info["param"]
            params = param_func()
            if params is not None:
                if name != "Terrain" and "object_type" not in params:
                    params["object_type"] = name

                self._temp_params_for_objs[name.lower()] = params
                num_tasks_to_run += 1
                carb.log_info(f"{log_prefix} Parameters gathered for '{name}'.")
            else:
                carb.log_info(f"{log_prefix} Skipping '{name}' based on parameters.")
                self._temp_params_for_objs[name.lower()] = None # Mark as skipped
        carb.log_info(f"{log_prefix} Parameter gathering finished.")


        # --- 7. Initialize Task Counter ---
        with self._generation_lock:
            self._pending_generation_tasks = num_tasks_to_run
            carb.log_info(f"{log_prefix} Initialized pending tasks: {self._pending_generation_tasks}")
            # Check if any tasks are actually scheduled
            if self._pending_generation_tasks == 0:
                carb.log_warn(f"{log_prefix} No tasks to run after parameter check.")
                self._generation_finished_main_thread(error=False) # Cleanup and re-enable button
                return # Exit if nothing to do

        # --- 8. Launch ONLY Terrain Background Thread ---
        terrain_params = self._temp_params_for_objs.get("terrain")
        if terrain_params: # Check if terrain task is supposed to run
            try:
                carb.log_info(f"{log_prefix} Starting background thread for 'Terrain'...")
                thread = threading.Thread(target=self._task_definitions["Terrain"]["bg"], args=(terrain_params,))
                thread.daemon = True
                thread.start()
                carb.log_info(f"{log_prefix} Terrain thread launched. Waiting for terrain stage update to launch object threads...")
            except Exception as e:
                carb.log_error(f"{log_prefix} Error launching background thread for 'Terrain': {e}")
                traceback.print_exc()
                # If terrain launch fails, generation cannot proceed correctly
                self._generation_finished_main_thread(error=True)
        elif "terrain" in self._temp_params_for_objs:
            # Terrain was considered but skipped based on params
            carb.log_info(f"{log_prefix} Terrain task skipped based on params. Dependent object tasks may fail or be skipped.")
            # We need to manually decrement the counter for terrain now if other tasks exist
            # and potentially trigger finish if ONLY terrain was left.
            if self._pending_generation_tasks == 1: # Only terrain was pending
                self._generation_finished_main_thread(error=False)
            else: # Other tasks might still run (but will likely fail if they need terrain info)
                with self._generation_lock:
                    self._pending_generation_tasks -= 1
                # No need to launch anything else here, the flow stops if terrain info isn't set
        else:
            # Terrain wasn't even in tasks_to_consider or definitions
            carb.log_error(f"{log_prefix} Terrain task not defined or parameters missing, cannot start generation.")
            self._generation_finished_main_thread(error=True)

    def clear_all_generated_assets(self, log_info=True):
        """Clears generated assets AND prototypes synchronously."""
        if not self._stage:
            carb.log_error("Clear All: Stage not available.")
            return

        parent_path = self._ui._get_value("parent_path", "str", constants.DEFAULT_PARENT_PATH) if self._ui else constants.DEFAULT_PARENT_PATH

        # Clear main generated content parent
        if is_prim_path_valid(parent_path):
            if delete_prim(parent_path):
                if log_info: carb.log_info(f"Removed generated assets under {parent_path}.")
            else:
                if log_info: carb.log_error(f"Failed to remove parent prim {parent_path}.")
        elif log_info:
            carb.log_info("Main parent path not found, nothing to clear there.")

        # Clear prototypes parent
        if is_prim_path_valid(constants.PROTOTYPES_PARENT_PATH):
            if delete_prim(constants.PROTOTYPES_PARENT_PATH):
                if log_info: carb.log_info(f"Removed prototypes under {constants.PROTOTYPES_PARENT_PATH}.")
                # Clear caches since prototypes are gone
                self._prototype_cache.clear()
                self._instancer_cache.clear() # Instancers are under main parent, but related
            else:
                if log_info: carb.log_error(f"Failed to remove prototypes parent {constants.PROTOTYPES_PARENT_PATH}.")
        elif log_info:
            carb.log_info("Prototypes parent path not found, nothing to clear there.")

        # Re-create the physics scene if needed after clearing
        # self._setup_physics_scene(self._stage)
        gc.collect()

    def load_update_hdr(self):
        """Loads or updates the HDRI dome light synchronously."""
        if not self._stage:
            carb.log_error("Load HDRI: Stage not available.")
            return

        log_prefix = "[load_update_hdr]"
        carb.log_info(f"{log_prefix} Starting HDRI update...")

        # Get parameters from UI
        parent_path = self._ui._get_value("parent_path", "str", constants.DEFAULT_PARENT_PATH) if self._ui else constants.DEFAULT_PARENT_PATH
        hdri_path = self._ui._get_value("hdri_path", "str", "") if self._ui else ""
        intensity = self._ui._get_value("light_intensity", "float", 1000.0) if self._ui else 1000.0
        rot_x = self._ui._get_value("light_rot_x", "float", 0.0) if self._ui else 0.0
        rot_y = self._ui._get_value("light_rot_y", "float", 0.0) if self._ui else 0.0
        rot_z = self._ui._get_value("light_rot_z", "float", 0.0) if self._ui else 0.0

        if not hdri_path:
            carb.log_info(f"{log_prefix} UI HDRI path is empty, attempting default.")
            if self._ext_path and hasattr(constants, "DEFAULT_HDR_REL_PATH"):
                default_full_path = utils.get_asset_path(self._ext_path, constants.DEFAULT_HDR_REL_PATH)
                if default_full_path:
                    hdri_path = default_full_path
                    carb.log_info(f"{log_prefix} Using default HDRI: {hdri_path}")
                else:
                    carb.log_warn(f"{log_prefix} Default HDRI asset path '{constants.DEFAULT_HDR_REL_PATH}' could not be resolved. Skipping HDRI.")
                    return # Exit if default also fails
            else:
                carb.log_warn(f"{log_prefix} Extension path or DEFAULT_HDR_REL_PATH constant missing. Skipping HDRI.")
                return

        # Define paths
        utils._ensure_prim_exists(self._stage, parent_path) # Ensure main parent exists
        light_parent_path = f"{parent_path}/Lights_Parent"
        utils._ensure_prim_exists(self._stage, light_parent_path) # Ensure light parent exists
        light_prim_path = f"{light_parent_path}/GeneratedDomeLight"

        try:
            # Define or get the DomeLight prim
            light_prim = UsdLux.DomeLight.Define(self._stage, light_prim_path).GetPrim()
            if not light_prim:
                carb.log_error(f"{log_prefix} Failed to define DomeLight prim at {light_prim_path}")
                return

            # Set attributes
            light_prim.GetAttribute("inputs:texture:file").Set(hdri_path)
            light_prim.GetAttribute("inputs:intensity").Set(intensity)
            # Apply rotation using XformCommonAPI
            xform_api = UsdGeom.XformCommonAPI(light_prim)
            rot_quat = utils.convert_euler_to_quat(rot_x, rot_y, rot_z)
            # Fix: XformCommonAPI needs Euler angles (Vec3f)
            roll, pitch, yaw = utils.quat_to_euler_angles(rot_quat)
            rotation_euler_deg = Gf.Vec3f(roll, pitch, yaw)
            xform_api.SetRotate(rotation_euler_deg, UsdGeom.XformCommonAPI.RotationOrderXYZ)

            carb.log_info(f"{log_prefix} DomeLight created/updated at {light_prim_path}")
        except Exception as e:
            carb.log_error(f"{log_prefix} Failed to create/update DomeLight: {e}")
            traceback.print_exc()


    # --- Parameter Gathering Helpers (Called on Main Thread) ---
    def _get_seed(self, category_seed_key: str) -> Optional[int]:
        """Gets the seed for a category, considering the global seed."""
        global_seed = self._ui._get_value("global_seed", "int", 0) if self._ui else 0
        if global_seed > 0: return global_seed
        category_seed = self._ui._get_value(category_seed_key, "int", 0) if self._ui else 0
        # Return a random seed if category seed is 0 (or None)
        return category_seed if category_seed > 0 else r.randint(1, 2**31 - 1)

    def _get_terrain_params(self) -> Optional[dict]:
        """Gathers parameters for terrain generation from the UI."""
        if self._ui is None: return None # Need UI to get params

        # Ensure parent category exists synchronously before background task needs it
        parent_cat = self._create_category_parent("terrain")
        target_material_path = f"{parent_cat}/GeneratedTerrainMaterial"

        return {
            "area_x": self._ui._get_value("area_x", "float", 50.0),
            "area_y": self._ui._get_value("area_y", "float", 50.0),
            "roughness": self._ui._get_value("terrain_roughness", "float", 5.0),
            "terrain_seed": self._get_seed("terrain_seed"),
            #"mat_path": self._ui._get_value("terrain_material", "str", None),
            "target_material_path": target_material_path,
            "h_scale": self._ui._get_value("terrain_h_scale", "float", 0.25),
            "v_scale": self._ui._get_value("terrain_v_scale", "float", 0.01),
            "octaves": self._ui._get_value("terrain_octaves", "int", 6),
            "persistence": self._ui._get_value("terrain_persistence", "float", 0.5),
            "lacunarity": self._ui._get_value("terrain_lacunarity", "float", 2.0),
            "category_parent": parent_cat
        }

    def _get_forest_params(self) -> Optional[dict]:
        """Gathers parameters for forest generation."""
        if self._ui is None: return None
        props = {k: self._ui._get_value(f"{k.lower()}_prop", "float", 0.0) for k in ["Birch", "Spruce", "Pine"]}
        if sum(props.values()) <= 1e-6: # Use tolerance for float comparison
            carb.log_info("Skipping forest task: No tree proportions set > 0.")
            return None
        parent_cat = self._create_category_parent("trees")
        return {
            "tree_seed": self._get_seed("tree_seed"),
            "density": self._ui._get_value("tree_density", "int", 10),
            "age_min": self._ui._get_value("tree_age_min", "int", 80),
            "age_max": self._ui._get_value("tree_age_max", "int", 120),
            "scale_var": self._ui._get_value("tree_scale_variation", "float", 10.0) / 100.0,
            "height_var": self._ui._get_value("tree_height_variation", "float", 10.0) / 100.0,
            "max_tilt": self._ui._get_value("tree_max_tilt", "float", 5.0),
            "props": props,
            "area_x": self._ui._get_value("area_x", "float", 50.0), # Need area for density calc
            "area_y": self._ui._get_value("area_y", "float", 50.0),
            "collision_type": "triangleMesh",
            "asset_configs": self._asset_configs,
            "category_parent": parent_cat
        }

    def _get_rock_params(self) -> Optional[dict]:
        """Gathers parameters for rock generation."""
        if self._ui is None: return None
        config = self._asset_configs.get("Rock")
        if not config or not config.get("full_path"):
            carb.log_info("Skipping rocks task: Rock asset config/path missing.")
            return None
        parent_cat = self._create_category_parent("rocks")
        return {
            "rock_seed": self._get_seed("rock_seed"),
            "density": self._ui._get_value("rock_density", "int", 2),
            "scale_min": self._ui._get_value("rock_scale_min", "float", 0.2),
            "scale_max": self._ui._get_value("rock_scale_max", "float", 0.5),
            "orient_var": self._ui._get_value("rock_orient_variation", "float", 15.0),
            "area_x": self._ui._get_value("area_x", "float", 50.0),
            "area_y": self._ui._get_value("area_y", "float", 50.0),
            "collision_type": "convexHull",
            "asset_configs": self._asset_configs,
            "category_parent": parent_cat
        }

    def _get_objects_params(self) -> Optional[dict]:
        """Gathers parameters for rock generation."""
        if self._ui is None: return None
        config = self._asset_configs.get("Container")
        if not config or not config.get("full_path"):
            carb.log_info("Skipping container task: Rock asset config/path missing.")
            return None
        parent_cat = self._create_category_parent("objects")
        return {
            "objects_seed": self._get_seed("objects_seed"),
            "density": self._ui._get_value("container_density", "int", 0),
            "scale_min": self._ui._get_value("objects_scale_min", "float", 0.2),
            "scale_max": self._ui._get_value("objects_scale_max", "float", 0.5),
            "orient_var": self._ui._get_value("objects_orient_variation", "float", 15.0),
            "area_x": self._ui._get_value("area_x", "float", 50.0),
            "area_y": self._ui._get_value("area_y", "float", 50.0),
            "collision_type": "convexHull",
            "asset_configs": self._asset_configs,
            "category_parent": parent_cat
        }

    def _get_veg_params(self) -> Optional[dict]:
        """Gathers parameters for vegetation generation."""
        if self._ui is None: return None
        if not self._ui._get_value("spawn_vegetation", "bool", True):
            carb.log_info("Skipping vegetation task: Disabled via checkbox.")
            return None
        # Check if *any* relevant assets exist
        bush_cfg = self._asset_configs.get("Bush")
        bb_cfg = self._asset_configs.get("Blueberry")
        grass_cfg = self._asset_configs.get("GrassPatch") # Check for grass

        can_spawn_bush = bush_cfg and bush_cfg.get("full_path")
        can_spawn_blueberry = bb_cfg and bb_cfg.get("full_path")
        can_spawn_grass = grass_cfg and grass_cfg.get("full_path") # Check grass path

        if not can_spawn_bush and not can_spawn_blueberry and not can_spawn_grass: # Modified condition
            carb.log_info("Skipping vegetation task: Asset configs/paths missing for bush, blueberry, AND grass.")
            return None
        parent_cat = self._create_category_parent("vegetation")
        return {
            "veg_seed": self._get_seed("veg_seed"),
            "density": self._ui._get_value("veg_density", "int", 10),
            "grass_density": self._ui._get_value("grass_density", "int", 50),
            "switchgrass_density": self._ui._get_value("switchgrass_density", "int", 5),
            "cluster_chance": self._ui._get_value("veg_cluster_chance", "float", 30.0) / 100.0,
            "cluster_min": self._ui._get_value("veg_cluster_min", "int", 5),
            "cluster_max": self._ui._get_value("veg_cluster_max", "int", 15),
            "cluster_radius": self._ui._get_value("veg_cluster_radius", "float", 0.15),
            "xy_min": self._ui._get_value("veg_scale_xy_min", "float", 0.7),
            "xy_max": self._ui._get_value("veg_scale_xy_max", "float", 1.1),
            "z_min": self._ui._get_value("veg_scale_z_min", "float", 0.8),
            "z_max": self._ui._get_value("veg_scale_z_max", "float", 1.0),
            "area_x": self._ui._get_value("area_x", "float", 50.0),
            "area_y": self._ui._get_value("area_y", "float", 50.0),
            "collision_type": "none",
            "asset_configs": self._asset_configs,
            "category_parent": parent_cat
        }

    def _get_or_create_prototype_main(self, asset_usd_path: str, asset_name: str) -> Optional[Usd.Prim]:
        """
        Gets or creates a shared, instanceable prototype prim for a given asset USD path.

        Manages a single prototype per unique asset USD path under PROTOTYPES_PARENT_PATH.
        Ensures the prototype exists, references the asset, and is marked instanceable.
        Does NOT set visibility or transforms on the prototype itself.

        Args:
            asset_usd_path: Absolute filesystem path to the asset USD file.
            asset_name: A clean base name for the prototype (e.g., "Bush", "Pine").

        Returns:
            The Usd.Prim of the prototype, or None on failure.
        """
        log_prefix = f"[GetCreateProto {asset_name}]"

        if not self._stage:
            carb.log_error(f"{log_prefix} Invalid USD stage.")
            return None
        if not os.path.isabs(asset_usd_path):
            carb.log_error(f"{log_prefix} Asset path must be absolute: {asset_usd_path}")
            return None
        if not self._prototype_cache:
            self._prototype_cache = {} # Ensure cache exists

        # --- 1. Ensure Global Prototypes Parent Path Exists ---
        try:
            utils._ensure_prim_exists(self._stage, constants.PROTOTYPES_PARENT_PATH, "Xform")
            proto_parent_prim = self._stage.GetPrimAtPath(constants.PROTOTYPES_PARENT_PATH)
            # Remove potentially problematic purpose from parent (adjust if you need guide purpose)
            if proto_parent_prim and proto_parent_prim.HasAttribute("purpose"):
                carb.log_info(f"{log_prefix} Removing purpose attribute from {constants.PROTOTYPES_PARENT_PATH}")
                proto_parent_prim.RemoveProperty("purpose")

        except Exception as e_proto_parent:
            carb.log_error(f"{log_prefix} Failed to ensure prototype parent {constants.PROTOTYPES_PARENT_PATH}: {e_proto_parent}")
            return None

        # --- 2. Get or Create Shared Prototype ---
        prototype_prim_path_str = self._prototype_cache.get(asset_usd_path)
        prototype_usd_prim = get_prim_at_path(prototype_prim_path_str) if prototype_prim_path_str else None

        if not prototype_usd_prim: # Check if prim at cached path is valid
            safe_asset_name = asset_name.replace(" ", "_").replace(".", "_") # Sanitize name
            prototype_prim_path_str = f"{constants.PROTOTYPES_PARENT_PATH}/{safe_asset_name}/{safe_asset_name}_Prototype"
            carb.log_info(f"{log_prefix} Creating prototype at {prototype_prim_path_str} for {asset_usd_path}")

            # Ensure clean slate at the target path
            if is_prim_path_valid(prototype_prim_path_str):
                delete_prim(prototype_prim_path_str)

            try:
                proto_xform_parent_prim = create_prim(f"{constants.PROTOTYPES_PARENT_PATH}/{safe_asset_name}", "Xform")
                # Create the prototype Xform prim - DO NOT set visibility here
                proto_xform_prim = create_prim(prototype_prim_path_str, "Xform")
                if not proto_xform_prim:
                    raise RuntimeError("Failed to create prototype Xform prim.")

                # Add reference to the actual asset *under* the prototype Xform
                ref_payload = proto_xform_prim.GetReferences().AddReference(assetPath=asset_usd_path)
                if not ref_payload:
                    carb.log_warn(f"{log_prefix} AddReference call returned None/False for {asset_usd_path}. Reference might have failed.")
                # else: carb.log_info(f"{log_prefix} AddReference called for {asset_usd_path}.") # Optional success log

                # Mark the prototype Xform as instanceable *after* referencing content
                proto_xform_prim.SetInstanceable(True)

                # Ensure the *referenced content* has the render purpose if needed
                # This is ideally set IN the source asset file, but can be attempted here
                # Note: This might not be robust if the referenced prim path changes
                # referenced_content_path = f"{prototype_prim_path_str}/{asset_name}" # GUESSING the root prim name inside asset
                # content_prim = get_prim_at_path(referenced_content_path)
                # if content_prim:
                #    try: UsdGeom.Imageable(content_prim).SetPurposeAttr(UsdGeom.Tokens.render)
                #    except Exception as e_purpose: carb.log_warn(f"{log_prefix} Failed to set purpose on guessed referenced content {referenced_content_path}: {e_purpose}")

                # Cache the validated path
                self._prototype_cache[asset_usd_path] = prototype_prim_path_str
                prototype_usd_prim = proto_xform_prim # Use the newly created prim
                carb.log_info(f"{log_prefix} Prototype created and cached: {prototype_prim_path_str}")

            except Exception as e_proto_create:
                carb.log_error(f"{log_prefix} Failed during prototype creation/referencing for {asset_usd_path} at {prototype_prim_path_str}: {e_proto_create}")
                if is_prim_path_valid(prototype_prim_path_str): delete_prim(prototype_prim_path_str)
                return None
        # else:
        # carb.log_info(f"{log_prefix} Found existing prototype in cache: {prototype_prim_path_str}")

        # Ensure existing prototype is still instanceable (important check)
        if prototype_usd_prim and not prototype_usd_prim.IsInstanceable():
            carb.log_warn(f"{log_prefix} Cached prototype {prototype_prim_path_str} was not instanceable. Setting it now.")
            prototype_usd_prim.SetInstanceable(True)

        return prototype_usd_prim

    # --- USD Prim/Scene Helpers (Called on Main Thread) ---
    def _create_category_parent(self, category_key: str) -> str:
        """Ensures the parent prim for a generation category exists."""
        parent_path = self._ui._get_value("parent_path", "str", constants.DEFAULT_PARENT_PATH) if self._ui else constants.DEFAULT_PARENT_PATH
        utils._ensure_prim_exists(self._stage, parent_path, "Xform")
        category_parent_path = f"{parent_path}/{category_key.capitalize()}_Parent"
        utils._ensure_prim_exists(self._stage, category_parent_path, "Xform")
        return category_parent_path

    def _setup_physics_scene(self, stage: Usd.Stage):
        """Creates or configures the default PhysicsScene prim."""
        # SUGGESTION: Move default physics scene path to constants.py
        physics_scene_path = "/World/physicsScene"
        log_prefix = "[Setup Physics]"
        if not stage:
            carb.log_error(f"{log_prefix} Stage is invalid.")
            return

        scene_prim = stage.GetPrimAtPath(physics_scene_path)
        if not scene_prim.IsValid():
            carb.log_info(f"{log_prefix} Physics scene not found at {physics_scene_path}. Creating...")
            try:
                # Define prim first
                scene_prim = stage.DefinePrim(Sdf.Path(physics_scene_path), "PhysicsScene")
                if not scene_prim: raise RuntimeError("Failed to define PhysicsScene prim.")
                # Apply APIs AFTER defining prim
                physx_api = PhysxSchema.PhysxSceneAPI.Apply(scene_prim) # Apply PhysX specific API

                # Set gravity direction and magnitude (using scene units)
                gravity_magnitude = 9.81 * (100.0 if constants.METERS_PER_UNIT == 1.0 else 1.0)
                scene_prim.CreateAttribute("physics:gravityDirection", Sdf.ValueTypeNames.Vector3f, False).Set(Gf.Vec3f(0.0, 0.0, -1.0))
                scene_prim.CreateAttribute("physics:gravityMagnitude", Sdf.ValueTypeNames.Float, False).Set(gravity_magnitude)

                # Apply common PhysX settings
                physx_api.CreateEnableCCDAttr(True)
                physx_api.CreateEnableGPUDynamicsAttr(True) # Use GPU if available
                physx_api.CreateEnableStabilizationAttr(True)
                # physx_api.CreateSolverTypeAttr().Set(PhysxSchema.Tokens.tgs) # TGS solver often better

                carb.log_info(f"{log_prefix} Created and configured Physics Scene at {physics_scene_path}")
            except Exception as e:
                carb.log_error(f"{log_prefix} Failed to create physics scene: {e}")
                traceback.print_exc()
        else:
            carb.log_info(f"{log_prefix} Physics scene already exists at {physics_scene_path}.")
            # Optionally: ensure existing scene has necessary APIs/settings


    # --- Background Generation Functions ---

    def _generate_terrain_background(self, params: dict) -> Optional[dict]:
        """Generates terrain data (vertices, triangles, heightfield) synchronously."""
        bg_log_prefix = "[BG Terrain]"
        carb.log_info(f"{bg_log_prefix} Starting...")

        try:
            # Seed specific to this thread/task if provided
            seed = params.get("terrain_seed")
            if seed: r.seed(seed); np.random.seed(seed)

            sub_terrain = terrain.SubTerrain(
                width=max(2, int(params["area_x"] / params["h_scale"])),
                length=max(2, int(params["area_y"] / params["h_scale"])),
                vertical_scale=params["v_scale"],
                horizontal_scale=params["h_scale"] )

            sub_terrain = terrain.random_uniform_terrain(
                sub_terrain, min_height=-params["roughness"] / 2.0, max_height=params["roughness"] / 2.0,
                noise_seed=seed, octaves=params["octaves"],
                persistence=params["persistence"], lacunarity=params["lacunarity"] )

            # Copy the raw heightfield data for lookup
            hf_raw_for_lookup = sub_terrain.height_field_raw.copy()

            vertices, triangles = terrain.convert_heightfield_to_trimesh(
                sub_terrain.height_field_raw, params["h_scale"], params["v_scale"] )

            carb.log_info(f"{bg_log_prefix} Data generated (Verts: {len(vertices)}, Tris: {len(triangles)}).")

            # Validation
            valid = True
            if vertices is None or vertices.size == 0: valid = False; carb.log_error(f"{bg_log_prefix} Verts empty")
            if valid and triangles is None: valid = False; carb.log_error(f"{bg_log_prefix} Tris is None")
            if valid: vertices = np.ascontiguousarray(vertices, dtype=np.float32)
            if valid: triangles = np.ascontiguousarray(triangles, dtype=np.int32)
            if valid and (np.any(np.isnan(vertices)) or np.any(np.isinf(vertices))): valid = False; carb.log_error(f"{bg_log_prefix} NaN/Inf")
            if valid and triangles.size > 0 and (np.min(triangles) < 0 or np.max(triangles) >= len(vertices)): valid = False; carb.log_error(f"{bg_log_prefix} Indices OOB")
            if not valid: raise ValueError("Validation failed")

            terrain_data_for_main = {
                "vertices": vertices, "triangles": triangles,
                "position": np.array([-params["area_x"] / 2.0, -params["area_y"] / 2.0, 0.0]),
                "orientation": np.array([1.0, 0.0, 0.0, 0.0]), # WXYZ default
                "terrain_prim_path": f"{params['category_parent']}/mesh",
                # "material_path": params["mat_path"]
                "target_material_path": params["target_material_path"],
                "height_field_raw": hf_raw_for_lookup,
                "h_scale": params["h_scale"],
                "v_scale": params["v_scale"],
                "area_x": params["area_x"],
                "area_y": params["area_y"],
            }

            carb.log_info(f"{bg_log_prefix} Data packaged.")

        except Exception as e:
            carb.log_error(f"{bg_log_prefix} Error: {e}")
            traceback.print_exc()
            terrain_data_for_main = None
        finally:
            carb.log_info(f"{bg_log_prefix} Scheduling stage update...")
            self._run_on_main_thread(lambda data=terrain_data_for_main: self._add_terrain_to_stage_main_thread(data))
            # Cleanup local vars
            vertices = triangles = sub_terrain = params = None; gc.collect()
            carb.log_info(f"{bg_log_prefix} Finished.")

    # --- Background Generation Functions ---
    def _generate_single_asset_type(self, params, asset_name, terrain_info, total_potential_points, add_item_callback):
        bg_log_prefix = f"[BG {asset_name}]"
        carb.log_info(f"{bg_log_prefix} Generating {asset_name}...")
        asset_config = params["asset_configs"].get(asset_name)
        if not asset_config or not asset_config.get("full_path"):
            raise ValueError(f"{asset_name} asset config/path missing.")
        asset_usd_path = asset_config["full_path"]
        base_scale = asset_config.get("base_scale", 1.0)

        for i in range(total_potential_points):
            spawn_x = r.uniform(-params["area_x"] / 2 * 0.98, params["area_x"] / 2 * 0.98)
            spawn_y = r.uniform(-params["area_y"] / 2 * 0.98, params["area_y"] / 2 * 0.98)
            spawn_z, surface_normal = utils.get_height_and_normal_from_noise_data(spawn_x, spawn_y, terrain_info)
            if spawn_z is None or surface_normal is None: continue

            scale_val = r.uniform(params["scale_min"], params["scale_max"]) * base_scale
            scale_vec = Gf.Vec3d(scale_val)

            up_vector = Gf.Vec3d(0, 0, 1)
            align_rot = Gf.Rotation(up_vector, surface_normal); align_quat_f = align_rot.GetQuaternion()
            yaw = r.uniform(0, 360); yaw_rot = Gf.Rotation(up_vector, yaw); yaw_quat_f = yaw_rot.GetQuaternion()
            roll_deg = r.uniform(-params["orient_var"], params["orient_var"]); pitch_deg = r.uniform(-params["orient_var"], params["orient_var"])
            tilt_quat_d = utils.convert_euler_to_quat(roll_deg, pitch_deg, 0)

            align_quat_d = Gf.Quatd(float(align_quat_f.GetReal()), Gf.Vec3d(align_quat_f.GetImaginary()))
            yaw_quat_d = Gf.Quatd(float(yaw_quat_f.GetReal()), Gf.Vec3d(yaw_quat_f.GetImaginary()))
            if not isinstance(tilt_quat_d, Gf.Quatd):
                tilt_quat_d = Gf.Quatd(tilt_quat_d.GetReal(), tilt_quat_d.GetImaginary())

            final_rot_quat = align_quat_d * yaw_quat_d * tilt_quat_d
            add_item_callback(asset_usd_path, asset_name, Gf.Vec3d(spawn_x, spawn_y, spawn_z), final_rot_quat, scale_vec)

    def _generate_placements_background(self, params: Dict[str, Any], task_name: str):
        """
        Generic background function to generate placement data for various object types
        (Rocks, Trees, Vegetation), aggregating results for Point Instancers.

        Calculates position (x, y, z from terrain), scale, and rotation based on
        parameters specific to the object type. Handles proportions (Forest) and
        clustering (Vegetation). Aggregates data by asset USD path. Schedules the
        generic main thread update function.

        Args:
            params: Dictionary of parameters gathered for this object type, MUST include
                    an 'object_type' key ("Rock", "Forest", "Vegetation") and other
                    necessary keys (density, scale ranges, asset configs, etc.).
            task_name: The name of the task (e.g., "Rocks", "Forest", "Vegetation")
                       for logging and cleanup tracking.
        """
        object_type = params.get("object_type")
        if not object_type: # Basic validation
            carb.log_error(f"[BG Placements] 'object_type' missing in params for task '{task_name}'. Aborting.")
            self._run_on_main_thread(lambda data=None, name=task_name: self._add_objects_to_stage_main_thread(data, name))
            return

        bg_log_prefix = f"[BG {object_type} ({task_name})]" # More specific logging
        carb.log_info(f"{bg_log_prefix} Starting generation...")

        # --- Aggregated Data Storage ---
        # Structure: { asset_usd_path: { "positions": [], "orientations": [], "scales": [], "asset_name": str } }
        items_by_asset: Dict[str, Dict[str, Any]] = {}
        aggregated_data_for_main: Optional[Dict[str, Dict]] = None # Final structure with NumPy arrays

        # --- Access stored terrain info (Read-Only) ---
        terrain_info = self._current_terrain_info
        if terrain_info is None:
            carb.log_error(f"{bg_log_prefix} Terrain info not available. Aborting task.")
            self._run_on_main_thread(lambda data=None, name=task_name: self._add_objects_to_stage_main_thread(data, name))
            return
        # ---

        try:
            # --- Seeding ---
            seed_key = f"{object_type.lower()}_seed" # Assumes param keys like 'rock_seed', 'forest_seed', 'veg_seed'
            seed = params.get(seed_key)
            if seed:
                r.seed(seed); np.random.seed(seed)
                carb.log_info(f"{bg_log_prefix} Random generators seeded with {seed}.")

            # --- Common Parameters ---
            asset_configs = params["asset_configs"]
            category_parent = params["category_parent"] # Needed for prim path construction later, but not directly used here
            area_x = params["area_x"]
            area_y = params["area_y"]
            density = params.get("density", 0)
            total_potential_points = int(density * area_x * area_y / 100.0) # Base number of points/centers
            carb.log_info(f"{bg_log_prefix} Calculated {total_potential_points} potential spawn points/centers based on density.")

            # --- Define a Helper Function for Adding Items ---
            def add_item(asset_path: str, asset_name: str, pos: Gf.Vec3d, rot: Gf.Quatd, scale: Gf.Vec3d):
                nonlocal items_by_asset
                if not asset_path:
                    carb.log_warn(f"{bg_log_prefix} Attempted to add item with invalid asset path. Skipping.")
                    return
                if asset_path not in items_by_asset:
                    items_by_asset[asset_path] = {
                        "positions": [], "orientations": [], "scales": [], "asset_name": asset_name
                    }
                # Append raw Python types/Gf types for now, convert to NumPy later
                items_by_asset[asset_path]["positions"].append(list(pos))
                # Ensure WXYZ format for Quatd
                items_by_asset[asset_path]["orientations"].append([rot.GetReal()] + list(rot.GetImaginary()))
                items_by_asset[asset_path]["scales"].append(list(scale))
            # --- End Helper Function ---


            # --- Type-Specific Generation Logic ---

            if object_type == "Objects":
                self._generate_single_asset_type(params, "Container", terrain_info, total_potential_points, add_item)

            elif object_type == "Rocks":
                self._generate_single_asset_type(params, "Rock", terrain_info, total_potential_points, add_item)

            elif object_type == "Forest":
                carb.log_info(f"{bg_log_prefix} Generating Forest...")
                # Determine tree types and counts based on proportions
                props = params["props"]
                total_prop = sum(props.values())
                scale_factor = 1.0 / total_prop if total_prop > 1e-6 else 1.0
                normalized_props = {k: v * scale_factor for k, v in props.items()}
                tree_spawn_list = [] # List of {"type": str, "config": dict}
                total_trees_calculated = 0

                for tree_type, proportion in normalized_props.items():
                    count = int(round(total_potential_points * proportion)) # Apply proportion to density points
                    config = asset_configs.get(tree_type)
                    if config and config.get("full_path"):
                        tree_spawn_list.extend([{"type": tree_type, "config": config}] * count)
                        total_trees_calculated += count
                    else:
                        carb.log_warn(f"{bg_log_prefix} Skipping tree type '{tree_type}': Config/path missing.")
                r.shuffle(tree_spawn_list)
                carb.log_info(f"{bg_log_prefix} Calculated {total_trees_calculated} total trees across types.")

                for tree_info in tree_spawn_list:
                    current_asset_config = tree_info["config"]
                    asset_usd_path = current_asset_config["full_path"]
                    asset_base_name = tree_info["type"]

                    spawn_x = r.uniform(-area_x / 2 * 0.98, area_x / 2 * 0.98)
                    spawn_y = r.uniform(-area_y / 2 * 0.98, area_y / 2 * 0.98)
                    spawn_z, _ = utils.get_height_and_normal_from_noise_data(spawn_x, spawn_y, terrain_info) # Normal not needed for trees
                    if spawn_z is None: continue
                    spawn_z -= 0.1

                    # Scale
                    growth = max(0.01, r.uniform(params["age_min"], params["age_max"]) / 100.0)
                    base_scale = current_asset_config.get("base_scale", 1.0) * growth
                    height_mult = current_asset_config.get("height_mult", 1.0)
                    scale_xy = base_scale * (1.0 + r.uniform(-params["scale_var"], params["scale_var"]))
                    scale_z = base_scale * height_mult * (1.0 + r.uniform(-params["height_var"], params["height_var"]))
                    scale_vec = Gf.Vec3d(scale_xy, scale_xy, scale_z)
                    # Rotation (Minimal alignment, random tilt/yaw)
                    tilt_x = r.uniform(-params["max_tilt"], params["max_tilt"])
                    tilt_y = r.uniform(-params["max_tilt"], params["max_tilt"])
                    yaw = r.uniform(0, 360)
                    final_rot_quat = utils.convert_euler_to_quat(tilt_x, tilt_y, yaw)
                    # Ensure Gf.Quatd
                    if not isinstance(final_rot_quat, Gf.Quatd):
                        final_rot_quat = Gf.Quatd(final_rot_quat.GetReal(), final_rot_quat.GetImaginary())


                    add_item(asset_usd_path, asset_base_name, Gf.Vec3d(spawn_x, spawn_y, spawn_z), final_rot_quat, scale_vec)

            elif object_type == "Vegetation":
                carb.log_info(f"{bg_log_prefix} Generating Vegetation (bushes, blueberries, grass)...")
                bush_cfg = asset_configs.get("Bush")
                bb_cfg = asset_configs.get("Blueberry")

                grass_cfg = asset_configs.get("Grass")
                grass_density = params.get("grass_density", 0)

                switchgrass_cfg = asset_configs.get("Switchgrass")
                switchgrass_density = params.get("switchgrass_density", 0)

                # --- Get new tilt params ---
                max_tilt_roll = params.get("veg_max_tilt_roll", 5.0)
                max_tilt_pitch = params.get("veg_max_tilt_pitch", 5.0)

                can_spawn_bush = bush_cfg and bush_cfg.get("full_path")
                can_spawn_blueberry = bb_cfg and bb_cfg.get("full_path")
                can_spawn_grass = grass_cfg and grass_cfg.get("full_path")

                if not can_spawn_bush and not can_spawn_blueberry and not can_spawn_grass:
                    raise ValueError("No valid vegetation assets found (Bush, Blueberry, or Grass).")

                # --- A) Generate Bushes and Blueberries ---
                carb.log_info(f"{bg_log_prefix} Generating bushes/blueberries ({total_potential_points} potential points)...")
                for i in range(total_potential_points):
                    # ... (clustering logic as before) ...
                    is_cluster = r.random() < params["cluster_chance"]
                    spawn_x_center = r.uniform(-area_x / 2 * 0.98, area_x / 2 * 0.98)
                    spawn_y_center = r.uniform(-area_y / 2 * 0.98, area_y / 2 * 0.98)

                    items_in_this_group = []
                    asset_to_spawn_config = None
                    if is_cluster and can_spawn_blueberry:
                        asset_to_spawn_config = bb_cfg
                        asset_path = bb_cfg["full_path"]
                        asset_name = "Blueberry"
                        cluster_size = r.randint(params["cluster_min"], params["cluster_max"])
                        # Generate points within the cluster radius around the center
                        for _ in range(cluster_size):
                            # Offset using Gaussian distribution for a more natural cluster shape
                            offset_x = r.gauss(0, params["cluster_radius"])
                            offset_y = r.gauss(0, params["cluster_radius"])
                            # Calculate final item position, clamping to terrain bounds
                            item_x = max(-area_x / 2 * 0.99, min(area_x / 2 * 0.99, spawn_x_center + offset_x))
                            item_y = max(-area_y / 2 * 0.99, min(area_y / 2 * 0.99, spawn_y_center + offset_y))
                            # Add the specific item's details to the list for this group
                            items_in_this_group.append((asset_path, asset_name, item_x, item_y))

                    elif not is_cluster and can_spawn_bush:
                        asset_to_spawn_config = bush_cfg
                        asset_path = bush_cfg["full_path"]
                        asset_name = "Bush"
                        # For a single bush, the item position *is* the center position
                        item_x = spawn_x_center
                        item_y = spawn_y_center
                        # Add the single item's details to the list
                        items_in_this_group.append((asset_path, asset_name, item_x, item_y))

                    if asset_to_spawn_config:
                        current_base_scale = asset_to_spawn_config.get("base_scale", 1.0)

                        for asset_path, asset_name, item_x, item_y in items_in_this_group:
                            # --- Get Normal Vector ---
                            spawn_z, surface_normal = utils.get_height_and_normal_from_noise_data(item_x, item_y, terrain_info)
                            if spawn_z is None or surface_normal is None: continue # Need normal now

                            # --- Scale (as before) ---
                            s_xy_rand = r.uniform(params["xy_min"], params["xy_max"])
                            s_z_mult_rand = r.uniform(params["z_min"], params["z_max"])
                            final_scale_xy = current_base_scale * s_xy_rand
                            final_scale_z = final_scale_xy * s_z_mult_rand
                            scale_vec = Gf.Vec3d(final_scale_xy, final_scale_xy, final_scale_z)

                            # --- Rotation: Partial Alignment + Tilt (Bushes/Blueberries) ---
                            world_up = Gf.Vec3d(0, 0, 1)
                            # Rotation to align world up with terrain normal
                            # Calculate the Gf.Rotation
                            align_rot = Gf.Rotation(world_up, surface_normal)
                            # Get the float-based Gf.Quaternion
                            source_align_quat_f = align_rot.GetQuaternion()
                            # Construct the Gf.Quatd explicitly from components
                            # Gf.Vec3d constructor can typically handle Gf.Vec3f input directly
                            align_quat = Gf.Quatd(float(source_align_quat_f.GetReal()),
                                                  Gf.Vec3d(source_align_quat_f.GetImaginary()))

                            # Random Yaw rotation around World Z
                            yaw_deg = r.uniform(0, 360)
                            yaw_quat = utils.convert_euler_to_quat(0, 0, yaw_deg) # Already Quatd

                            # Random independent Tilt (Roll/Pitch) relative to local frame
                            tilt_roll = r.uniform(-max_tilt_roll, max_tilt_roll)
                            tilt_pitch = r.uniform(-max_tilt_pitch, max_tilt_pitch)
                            tilt_quat = utils.convert_euler_to_quat(tilt_roll, tilt_pitch, 0) # Already Quatd

                            # Combine: Align to terrain first, then apply world yaw, then local tilt
                            # Note: Quaternion multiplication order is right-to-left application
                            final_rot_quat = align_quat * yaw_quat * tilt_quat
                            # Optional Normalization (good practice after multiplications)
                            final_rot_quat.Normalize()

                            add_item(asset_path, asset_name, Gf.Vec3d(item_x, item_y, spawn_z), final_rot_quat, scale_vec)


                # --- B) Generate Grass ---
                # --- UPDATED ASSET NAME ---
                if can_spawn_grass and grass_density > 0:
                    num_grass = int(grass_density * area_x * area_y / 100.0)
                    grass_asset_path = grass_cfg["full_path"]
                    grass_asset_name = "Grass" # Updated name
                    grass_base_scale = grass_cfg.get("base_scale", 1.0)
                    carb.log_info(f"{bg_log_prefix} Generating {num_grass} grass patches (Base Scale: {grass_base_scale})...")

                    for _ in range(num_grass):
                        spawn_x = r.uniform(-area_x / 2 * 0.98, area_x / 2 * 0.98)
                        spawn_y = r.uniform(-area_y / 2 * 0.98, area_y / 2 * 0.98)
                        # --- Get Normal Vector ---
                        spawn_z, surface_normal = utils.get_height_and_normal_from_noise_data(spawn_x, spawn_y, terrain_info)
                        if spawn_z is None or surface_normal is None: continue # Need normal
                        spawn_z -= 0.05

                        # --- Scale (as before, with multiplier) ---
                        s_xy_rand = r.uniform(params["xy_min"], params["xy_max"])
                        s_z_mult_rand = r.uniform(params["z_min"], params["z_max"])
                        final_scale_xy = grass_base_scale * s_xy_rand
                        final_scale_z = final_scale_xy * s_z_mult_rand
                        patch_size_multiplier = 2.0
                        final_scale_xy *= patch_size_multiplier
                        final_scale_z *= patch_size_multiplier
                        scale_vec = Gf.Vec3d(final_scale_xy, final_scale_xy, final_scale_z)

                        # --- Rotation: Perfect Alignment + Yaw (Grass) ---
                        object_up = Gf.Vec3d(0, 0, 1)

                        # Calculate the Gf.Rotation for alignment
                        align_rot = Gf.Rotation(object_up, surface_normal)
                        # Get the float-based Gf.Quaternion
                        source_align_quat_f = align_rot.GetQuaternion()
                        # Construct the Gf.Quatd explicitly from components
                        align_quat = Gf.Quatd(float(source_align_quat_f.GetReal()),
                                              Gf.Vec3d(source_align_quat_f.GetImaginary()))

                        # Random Yaw rotation *around the surface normal*
                        yaw_deg = r.uniform(0, 360)
                        yaw_around_normal_rot = Gf.Rotation(surface_normal, yaw_deg)
                        # Get the float-based Gf.Quaternion
                        source_yaw_quat_f = yaw_around_normal_rot.GetQuaternion()
                        # Construct the Gf.Quatd explicitly from components
                        yaw_around_normal_quat = Gf.Quatd(float(source_yaw_quat_f.GetReal()),
                                                          Gf.Vec3d(source_yaw_quat_f.GetImaginary()))

                        # Combine: Align first, then yaw around the new 'up' (the normal)
                        final_rot_quat = yaw_around_normal_quat * align_quat # Apply alignment first
                        final_rot_quat.Normalize()

                        add_item(grass_asset_path, grass_asset_name, Gf.Vec3d(spawn_x, spawn_y, spawn_z), final_rot_quat, scale_vec)
                else:
                    carb.log_info(f"{bg_log_prefix} Skipping grass generation (Asset invalid or density is 0).")

                if switchgrass_density > 0:
                    num_switchgrass = int(switchgrass_density * area_x * area_y / 100.0)
                    switchgrass_asset_path = switchgrass_cfg["full_path"]
                    switchgrass_asset_name = "Switchgrass" # Updated name
                    switchgrass_base_scale = switchgrass_cfg.get("base_scale", 1.0)
                    carb.log_info(f"{bg_log_prefix} Generating {num_switchgrass} grass patches (Base Scale: {switchgrass_base_scale})...")

                    for _ in range(num_switchgrass):
                        spawn_x = r.uniform(-area_x / 2 * 0.98, area_x / 2 * 0.98)
                        spawn_y = r.uniform(-area_y / 2 * 0.98, area_y / 2 * 0.98)
                        # --- Get Normal Vector ---
                        spawn_z, surface_normal = utils.get_height_and_normal_from_noise_data(spawn_x, spawn_y, terrain_info)
                        if spawn_z is None or surface_normal is None: continue # Need normal
                        spawn_z -= 0.05

                        # --- Scale (as before, with multiplier) ---
                        s_xy_rand = r.uniform(params["xy_min"], params["xy_max"])
                        s_z_mult_rand = r.uniform(params["z_min"], params["z_max"])
                        final_scale_xy = switchgrass_base_scale * s_xy_rand
                        final_scale_z = final_scale_xy * s_z_mult_rand
                        patch_size_multiplier = 1.0
                        final_scale_xy *= patch_size_multiplier
                        final_scale_z *= patch_size_multiplier
                        scale_vec = Gf.Vec3d(final_scale_xy, final_scale_xy, final_scale_z)

                        # --- Rotation: Perfect Alignment + Yaw (Grass) ---
                        object_up = Gf.Vec3d(0, 0, 1)

                        # Calculate the Gf.Rotation for alignment
                        align_rot = Gf.Rotation(object_up, surface_normal)
                        # Get the float-based Gf.Quaternion
                        source_align_quat_f = align_rot.GetQuaternion()
                        # Construct the Gf.Quatd explicitly from components
                        align_quat = Gf.Quatd(float(source_align_quat_f.GetReal()),
                                              Gf.Vec3d(source_align_quat_f.GetImaginary()))

                        # Random Yaw rotation *around the surface normal*
                        yaw_deg = r.uniform(0, 360)
                        yaw_around_normal_rot = Gf.Rotation(surface_normal, yaw_deg)
                        # Get the float-based Gf.Quaternion
                        source_yaw_quat_f = yaw_around_normal_rot.GetQuaternion()
                        # Construct the Gf.Quatd explicitly from components
                        yaw_around_normal_quat = Gf.Quatd(float(source_yaw_quat_f.GetReal()),
                                                          Gf.Vec3d(source_yaw_quat_f.GetImaginary()))

                        # Combine: Align first, then yaw around the new 'up' (the normal)
                        final_rot_quat = yaw_around_normal_quat * align_quat # Apply alignment first
                        final_rot_quat.Normalize()

                        add_item(switchgrass_asset_path, switchgrass_asset_name, Gf.Vec3d(spawn_x, spawn_y, spawn_z), final_rot_quat, scale_vec)
                else:
                    carb.log_info(f"{bg_log_prefix} Skipping switchgrass generation (Asset invalid or density is 0).")

            else:
                raise NotImplementedError(f"Object type '{object_type}' generation not implemented.")


            # --- Final Aggregation: Convert lists to NumPy arrays ---
            aggregated_data_for_main = {}
            total_final_instances = 0
            if not items_by_asset:
                carb.log_info(f"{bg_log_prefix} No items were generated for any asset type.")
            else:
                carb.log_info(f"{bg_log_prefix} Aggregating data for {len(items_by_asset)} asset types...")
                for asset_path, data in items_by_asset.items():
                    count = len(data["positions"])
                    if count > 0:
                        aggregated_data_for_main[asset_path] = {
                            "positions": np.array(data["positions"], dtype=np.float32),
                            "orientations": np.array(data["orientations"], dtype=np.float32), # Should be WXYZ
                            "scales": np.array(data["scales"], dtype=np.float32),
                            "asset_name": data["asset_name"],
                            # Add collision type from params - main thread function will use this
                            "collision_type": params.get("collision_type", "none")
                        }
                        total_final_instances += count
                    else:
                        carb.log_warn(f"{bg_log_prefix} Asset path {asset_path} had 0 instances after processing. Excluding.")
                carb.log_info(f"{bg_log_prefix} Aggregated data for {total_final_instances} total instances across {len(aggregated_data_for_main)} asset types.")


        except Exception as e:
            carb.log_error(f"{bg_log_prefix} Error during generation: {e}"); traceback.print_exc()
            aggregated_data_for_main = None # Signal failure
        finally:
            carb.log_info(f"{bg_log_prefix} Scheduling stage update for task '{task_name}'...")
            # Schedule the GENERIC main thread update function with the aggregated data
            self._run_on_main_thread(lambda data=aggregated_data_for_main, name=task_name: self._add_objects_to_stage_main_thread(data, name))

            # Cleanup local vars
            params = terrain_info = asset_configs = items_by_asset = None
            if object_type == "Forest": tree_spawn_list = None
            # aggregated_data_for_main passed to lambda, GC handles later
            gc.collect()
            carb.log_info(f"{bg_log_prefix} Background task '{task_name}' finished.")


    # --- Main Thread Stage Update Functions ---
    def _add_terrain_to_stage_main_thread(self, terrain_data: Optional[dict]):
        """Adds terrain mesh, sets pose, stores height info, launches object threads."""
        main_log_prefix = "[Main Add Terrain+Launch]"
        success = False
        terrain_prim = None
        # Retrieve stored parameters needed for subsequent object threads
        params_for_objs = self._temp_params_for_objs

        try:
            if terrain_data and self._stage and params_for_objs:
                target_mat_path = terrain_data.get("target_material_path")
                material_created = False # Assume failure unless successful
                if target_mat_path:
                    material_created = self._create_terrain_material(target_mat_path)
                    if not material_created:
                        carb.log_warn(f"{main_log_prefix} Failed to create terrain material. Continuing without.")
                else:
                    carb.log_warn(f"{main_log_prefix} Target material path missing. Continuing without.")

                # --- Call add_terrain_to_stage WITHOUT pose data ---
                terrain_prim_obj = add_terrain_to_stage( # Rename variable to avoid conflict
                    stage=self._stage,
                    vertices=terrain_data["vertices"],
                    triangles=terrain_data["triangles"],
                    terrain_prim_path=terrain_data["terrain_prim_path"],
                    material_path=target_mat_path if material_created else None
                )

                if terrain_prim_obj:

                    carb.log_info(f"{main_log_prefix} Yielding to app loop after mesh creation...")
                    omni.kit.app.get_app().update() # Allow renderer/app to process
                    time.sleep(0.01) # Small sleep can sometimes help ensure update takes effect
                    carb.log_info(f"{main_log_prefix} Resuming terrain stage update.")

                    # --- Set Pose AFTER mesh creation ---
                    pos_world = terrain_data.get("position")
                    orient_quat_wxyz = terrain_data.get("orientation")
                    if pos_world is not None and orient_quat_wxyz is not None:
                        carb.log_info(f"{main_log_prefix} Setting world pose for {terrain_prim_obj}...")
                        terrain_prim_obj.set_world_poses(positions=np.expand_dims(pos_world, axis=0), orientations=np.expand_dims(orient_quat_wxyz, axis=0))
                        carb.log_info(f"{main_log_prefix} World pose set.")
                    else:
                        carb.log_warn(f"{main_log_prefix} Pose data missing. Skipping pose setting.")

                    # --- Store terrain lookup info ---
                    self._current_terrain_info = {
                        k: terrain_data.get(k) for k in
                        ["height_field_raw", "h_scale", "v_scale", "area_x", "area_y"]
                    }
                    terrain_info_valid = True
                    for key, value in self._current_terrain_info.items():
                        # Specifically check if the height_field_raw is None or empty, handle other Nones too
                        if key == "height_field_raw":
                            if value is None or value.size == 0:
                                carb.log_error(f"{main_log_prefix} Stored terrain info is invalid: height_field_raw is missing or empty.")
                                terrain_info_valid = False
                                break
                        elif value is None:
                            carb.log_error(f"{main_log_prefix} Stored terrain info is invalid: '{key}' is None.")
                            terrain_info_valid = False
                            break # Exit loop early if any required value is None

                    if not terrain_info_valid:
                        self._current_terrain_info = None # Invalidate if incomplete
                    else:
                        carb.log_info(f"{main_log_prefix} Stored terrain info for object placement.")
                    success = True # Mark terrain part successful

                    # --- Launch Object Background Threads (only if terrain info stored) ---
                    if self._current_terrain_info:
                        carb.log_info(f"{main_log_prefix} Launching object generation threads...")
                        # Retrieve task definitions (could be stored instead of recreating)
                        for name, task_info in self._task_definitions.items():

                            if name == "Terrain":
                                continue # Already done

                            bg_func = task_info["bg"]
                            task_params = params_for_objs.get(name.lower())
                            if task_params: # Only launch if params were gathered
                                try:
                                    carb.log_info(f"{main_log_prefix} Starting background thread for '{name}'...")
                                    thread = threading.Thread(target=bg_func, args=(task_params, name))
                                    thread.daemon = True
                                    thread.start()
                                except Exception as e:
                                    carb.log_error(f"{main_log_prefix} Error launching thread for '{name}': {e}")
                                    # How to handle launch failure? Maybe call check_finish directly?
                                    self._check_and_finish_generation(task_name=f"{name}_LaunchFail", was_success=False)
                            else:
                                carb.log_info(f"{main_log_prefix} Skipping object task '{name}' (no params).")
                    else:
                        carb.log_error(f"{main_log_prefix} Cannot launch object threads, terrain info invalid.")
                        # Need to ensure pending tasks are decremented if objects won't run
                        num_skipped = len([p for p in params_for_objs.values() if p is not None]) - 1 # -1 for terrain
                        with self._generation_lock:
                            self._pending_generation_tasks -= num_skipped
                        if self._pending_generation_tasks <= 1: # Only terrain left potentially
                            self._run_on_main_thread(self._generation_finished_main_thread)

                else:
                    carb.log_error(f"{main_log_prefix} add_terrain_to_stage failed.")
                    success = False

            elif not params_for_objs: carb.log_error(f"{main_log_prefix} Params for object tasks missing!")
            elif not terrain_data: carb.log_error(f"{main_log_prefix} Invalid terrain data received.")
            else: carb.log_error(f"{main_log_prefix} Stage invalid.")

        except Exception as e: carb.log_error(f"{main_log_prefix} Error: {e}"); traceback.print_exc(); success = False
        finally:
            terrain_data = None; gc.collect()
            # Decrement counter for the TERRAIN task itself
            self._check_and_finish_generation(task_name="Terrain", was_success=success)


    # --- Main Thread Stage Update Functions ---
    def _add_objects_to_stage_main_thread(
            self,
            grouped_items_data: Optional[Dict[str, Dict]],
            task_name: str # e.g., "Rocks", "Vegetation", "Forest"
    ):
        """
        Adds generated objects to the stage using a conditional approach:
        - If collision_type is 'none' (e.g., Vegetation): Uses hybrid instancing
          (Instance0 Xform referencing prototype + PointInstancer targeting Instance0).
        - If collision_type is NOT 'none' (e.g., Rocks, Trees): Creates individual
          referenced copies for each instance ("copy-paste"), applying transforms
          and physics directly to each copy. Tree collision is attempted only on
          a 'Trunk' sub-prim.

        Args:
            grouped_items_data: Aggregated data from the background thread. Keys are
                                asset USD paths, values contain transform arrays,
                                'asset_name', and 'collision_type'. Can be None.
            task_name: The name of the task (e.g., "Rocks").
        """
        # --- START MODIFIED FUNCTION ---
        main_log_prefix = f"[Main Add {task_name} Conditional]" # Updated prefix
        success = False
        total_spawned_count = 0

        task_params = self._temp_params_for_objs.get(task_name.lower()) if self._temp_params_for_objs else None
        if not task_params:
            carb.log_error(f"{main_log_prefix} Failed to retrieve task parameters for '{task_name}'.")
            self._check_and_finish_generation(task_name=task_name, was_success=False)
            return

        category_parent_path = task_params.get("category_parent")
        if not category_parent_path:
            carb.log_error(f"{main_log_prefix} 'category_parent' missing for task '{task_name}'.")
            self._check_and_finish_generation(task_name=task_name, was_success=False)
            return

        scene_units_meter = 1.0
        try: scene_units_meter = get_stage_units()
        except Exception as e_units: carb.log_warn(f"{main_log_prefix} Could not get stage units: {e_units}. Using default 1.0.")

        try:
            if grouped_items_data is not None and self._stage:
                if not grouped_items_data:
                    carb.log_info(f"{main_log_prefix} No aggregated object data provided.")
                    success = True
                else:
                    carb.log_info(f"{main_log_prefix} Processing {len(grouped_items_data)} asset types...")
                    utils._ensure_prim_exists(self._stage, category_parent_path, "Xform")

                    asset_success_count = 0
                    processed_asset_names_log = [] # More descriptive name

                    for asset_usd_path, asset_data in grouped_items_data.items():
                        positions = asset_data.get("positions")
                        orientations = asset_data.get("orientations") # WXYZ, float32
                        scales = asset_data.get("scales")
                        asset_base_name = asset_data.get("asset_name", "UnnamedAsset")
                        collision_type = asset_data.get("collision_type", "none")

                        if positions is None or orientations is None or scales is None:
                            carb.log_error(f"{main_log_prefix} Missing transform arrays for {asset_base_name}. Skipping.")
                            continue
                        num_instances = positions.shape[0]
                        if num_instances == 0:
                            carb.log_info(f"{main_log_prefix} Skipping asset {asset_base_name}: Zero instances.")
                            continue

                        carb.log_info(f"{main_log_prefix} Processing {num_instances} of '{asset_base_name}' (Collision: {collision_type})")

                        # ===========================================================
                        # BRANCHING LOGIC: Instance OR Copy-Paste
                        # ===========================================================
                        if collision_type == 'none':
                            # --- METHOD 1: INSTANCING (Instance0 + PointInstancer) for Non-Physics ---
                            carb.log_info(f"{main_log_prefix} Using INSTANCING for {asset_base_name}")

                            # IMPORTANT WORKAROUND RATIONALE:
                            # We use this intermediate Xform because directly transforming the original prototype
                            # (shared via the cache) causes transforms to leak to other objects using the same prototype.
                            # Ideally, prototypes should remain untransformed templates.
                            # However, simply using a PointInstancer for *all* instances (including the first)
                            # led to unresolved visibility issues with the prototypes themselves
                            # (e.g., needing to make the original prototype invisible or non-collidable without
                            # affecting instances). Creating this explicit, transformed "Instance 0" Xform that
                            # references the prototype avoids modifying the shared prototype while keeping the
                            # first instance tangible and visible, sidestepping the aforementioned issues without
                            # needing to debug the root cause of the visibility/collision problems right now.
                            # This is a pragmatic workaround, not necessarily the fundamentally "correct" USD approach.

                            # 1. Get or Create the UNMODIFIED Prototype
                            original_prototype_prim = self._get_or_create_prototype_main(asset_usd_path, asset_base_name)
                            if not original_prototype_prim:
                                carb.log_error(f"{main_log_prefix} Failed to get/create prototype for {asset_base_name}. Skipping asset.")
                                continue
                            original_prototype_prim_path = str(original_prototype_prim.GetPath())

                            # 2. Get Intermediate Xform for Instance 0 (under category parent)
                            instance0_prim_path_str = str(original_prototype_prim.GetPath().GetParentPath())

                            # 3. Apply Transform to Instance 0
                            try:
                                pos0_np = positions[0]; orn0_np_wxyz = orientations[0]; scl0_np = scales[0]
                                instance0_xform_prim = XFormPrim(prim_paths_expr=instance0_prim_path_str)
                                instance0_xform_prim.set_world_poses(positions=np.expand_dims(pos0_np, axis=0), orientations=np.expand_dims(orn0_np_wxyz, axis=0))
                                instance0_xform_prim.set_local_scales(np.expand_dims(scl0_np, axis=0))
                                total_spawned_count += 1
                                processed_asset_names_log.append(f"{asset_base_name}_I0") # Short log name
                            except Exception as e_instance0_xform:
                                carb.log_error(f"{main_log_prefix} Failed to transform Instance0 {instance0_prim_path_str}: {e_instance0_xform}")
                                continue # Skip rest of this asset if Instance0 fails

                            # 4. Create Point Instancer for Remaining Instances (if num > 1)
                            if num_instances > 1:
                                instancer_path = f"{category_parent_path}/{asset_base_name}_Instancer"
                                try:
                                    positions_rest = positions[1:]; orientations_rest = orientations[1:]; scales_rest = scales[1:]
                                    num_rest = num_instances - 1

                                    instancer_geom = UsdGeom.PointInstancer.Define(self._stage, instancer_path)
                                    instancer_prim = instancer_geom.GetPrim()
                                    if not instancer_prim: raise RuntimeError(f"Failed define PointInstancer {instancer_path}")

                                    instancer_geom.GetPrototypesRel().SetTargets([Sdf.Path(original_prototype_prim_path)])
                                    proto_indices = np.zeros(num_rest, dtype=np.int32) # All remaining instances use prototype 0 (which is Instance0_Xform)
                                    instancer_geom.CreateProtoIndicesAttr().Set(Vt.IntArray.FromNumpy(proto_indices))
                                    instancer_geom.CreatePositionsAttr().Set(Vt.Vec3fArray.FromNumpy(positions_rest))
                                    # Convert the array from WXYZ to XYZW by moving the scalar (w) from index 0 to index 3.
                                    orientations_rest_reordered = orientations_rest.copy()
                                    orientations_rest_reordered[:, :3] = orientations_rest[:, 1:4]  # Set X, Y, Z from indices 1,2,3
                                    orientations_rest_reordered[:, 3] = orientations_rest[:, 0]     # Set W (scalar) as the last element
                                    instancer_geom.CreateOrientationsAttr().Set(Vt.QuathArray.FromNumpy(orientations_rest_reordered))
                                    instancer_geom.CreateScalesAttr().Set(Vt.Vec3fArray.FromNumpy(scales_rest))
                                    Usd.ModelAPI(instancer_prim).SetKind("prop") # Use Kind token

                                    # NO COLLISION APPLIED TO INSTANCER/INSTANCE0 in this branch

                                    total_spawned_count += num_rest
                                    processed_asset_names_log.append(f"{asset_base_name}_Inst({num_rest})")
                                    carb.log_info(f"{main_log_prefix} Configured PointInstancer for {num_rest} '{asset_base_name}' instances.")

                                except Exception as e_instancer:
                                    carb.log_error(f"{main_log_prefix} Failed config PointInstancer {instancer_path}: {e_instancer}")
                                    traceback.print_exc() # Log full error
                                    # Instance0 might still exist, don't necessarily 'continue'

                            asset_success_count += 1 # Mark asset type as processed if Instance0 succeeded

                        else:
                            # --- METHOD 2: COPY-PASTE (Individual References) for Physics Objects ---
                            carb.log_info(f"{main_log_prefix} Using COPY-PASTE for {asset_base_name}")
                            num_copied = 0
                            for i in range(num_instances):
                                instance_prim_path_str = f"{category_parent_path}/{asset_base_name}_{i}"

                                # 1. Create Unique Prim and Reference Original Asset
                                if is_prim_path_valid(instance_prim_path_str): delete_prim(instance_prim_path_str)
                                instance_prim = self._stage.DefinePrim(Sdf.Path(instance_prim_path_str), "Xform")
                                if not instance_prim:
                                    carb.log_error(f"{main_log_prefix} Failed define copy prim {instance_prim_path_str}. Skipping instance {i}.")
                                    continue
                                # --- Reference the ASSET DIRECTLY (not the prototype) ---
                                ref_added = instance_prim.GetReferences().AddReference(assetPath=asset_usd_path)
                                if not ref_added:
                                    carb.log_warn(f"{main_log_prefix} AddReference failed for {instance_prim_path_str} to {asset_usd_path}.")
                                    delete_prim(instance_prim_path_str)
                                    continue

                                # 2. Apply Individual Transform
                                try:
                                    pos_i = positions[i]; orn_i_wxyz = orientations[i]; scl_i = scales[i]
                                    instance_xform = XFormPrim(prim_paths_expr=instance_prim_path_str)
                                    instance_xform.set_world_poses(positions=np.expand_dims(pos_i, axis=0), orientations=np.expand_dims(orn_i_wxyz, axis=0))
                                    instance_xform.set_local_scales(np.expand_dims(scl_i, axis=0))
                                except Exception as e_copy_xform:
                                    carb.log_error(f"{main_log_prefix} Failed transform copy {instance_prim_path_str}: {e_copy_xform}")
                                    continue

                                # 3. Find and Apply Physics Collision
                                target_collision_prim = None
                                target_collision_prim_path = ""

                                # --- START USING HELPER FUNCTION ---
                                collision_target_prim = utils.find_collision_target_prim(instance_prim)

                                if collision_target_prim:
                                    target_collision_prim_path = str(collision_target_prim.GetPath())
                                else:
                                    # Fallback to the root of the copied instance
                                    target_collision_prim = instance_prim
                                    target_collision_prim_path = instance_prim_path_str
                                    carb.log_warn(f"{main_log_prefix} No specific collision target (trunk/mesh) found for {instance_prim_path_str}. Applying collision to root as fallback.")
                                # --- END USING HELPER FUNCTION ---


                                # --- Apply Collision APIs to the determined target prim ---
                                if collision_target_prim and collision_target_prim.IsValid():

                                    # Clear existing physics APIs first
                                    if collision_target_prim.HasAPI(UsdPhysics.CollisionAPI): collision_target_prim.RemoveAPI(UsdPhysics.CollisionAPI)
                                    if collision_target_prim.HasAPI(UsdPhysics.MeshCollisionAPI): collision_target_prim.RemoveAPI(UsdPhysics.MeshCollisionAPI)
                                    if collision_target_prim.HasAPI(PhysxSchema.PhysxCollisionAPI): collision_target_prim.RemoveAPI(PhysxSchema.PhysxCollisionAPI)

                                    # Apply new APIs
                                    UsdPhysics.CollisionAPI.Apply(collision_target_prim)
                                    mesh_api = UsdPhysics.MeshCollisionAPI.Apply(collision_target_prim)
                                    approx_token = UsdPhysics.Tokens.none
                                    if collision_type == 'convexHull': approx_token = UsdPhysics.Tokens.convexHull
                                    elif collision_type == 'boundingCube': approx_token = UsdPhysics.Tokens.boundingCube
                                    elif collision_type == 'triangleMesh': approx_token = UsdPhysics.Tokens.none
                                    else: carb.log_warn(f"{main_log_prefix} Unsupported collision type '{collision_type}' mapped to 'none' approximation for {target_collision_prim_path}")
                                    mesh_api.CreateApproximationAttr().Set(approx_token)

                                    # Apply PhysX settings
                                    try:
                                        physx_api = PhysxSchema.PhysxCollisionAPI.Apply(collision_target_prim)
                                        unit_factor = scene_units_meter
                                        contact_offset_scene = constants.DEFAULT_CONTACT_OFFSET_METERS / unit_factor
                                        rest_offset_scene = constants.DEFAULT_REST_OFFSET_METERS / unit_factor
                                        contact_attr = physx_api.CreateContactOffsetAttr()
                                        rest_attr = physx_api.CreateRestOffsetAttr()
                                        if contact_attr: contact_attr.Set(float(contact_offset_scene))
                                        if rest_attr: rest_attr.Set(float(rest_offset_scene))
                                    except Exception as e_physx_api:
                                        carb.log_error(f"{main_log_prefix} Failed apply PhysxCollisionAPI to {target_collision_prim_path}: {e_physx_api}")
                                else:
                                    carb.log_error(f"{main_log_prefix} Collision target prim {target_collision_prim_path} is invalid. Cannot apply physics.")

                                num_copied += 1
                                total_spawned_count += 1

                            processed_asset_names_log.append(f"{asset_base_name}_Copy({num_copied})")
                            if num_copied > 0:
                                asset_success_count += 1

                        # --- End of Instancing vs Copy-Paste branch ---

                    # --- End of loop through asset types ---

                    success = asset_success_count > 0 or len(grouped_items_data) == 0
                    if success:
                        carb.log_info(f"{main_log_prefix} Finished processing. {asset_success_count}/{len(grouped_items_data)} asset types processed. Total objects/instances: {total_spawned_count}. Processed: {', '.join(processed_asset_names_log)}")
                    else:
                        carb.log_error(f"{main_log_prefix} Finished processing, but failed for all asset types or no data.")
            else: # Stage is None
                carb.log_error(f"{main_log_prefix} Stage is invalid.")
                success = False

        except Exception as e:
            carb.log_error(f"{main_log_prefix} Unexpected error during stage update: {e}")
            traceback.print_exc()
            success = False
        finally:
            grouped_items_data = task_params = None
            gc.collect()
            self._check_and_finish_generation(task_name=task_name, was_success=success)


    def _create_terrain_material(self, material_path_str: str) -> bool:
        """Creates a UsdShade Material with OmniPBR using the correct UsdShade.Shader API and confirmed input names."""
        mat_log_prefix = "[Create Material v12]"
        carb.log_info(f"{mat_log_prefix} Attempting material creation via correct UsdShade API at {material_path_str}")
        material_path = Sdf.Path(material_path_str)

        # --- Checks and path resolution ---
        if not self._stage: carb.log_error(f"{mat_log_prefix} Stage is invalid."); return False
        if not self._ext_path: carb.log_error(f"{mat_log_prefix} Extension path not set."); return False

        shader_prim_path = material_path.AppendPath("OmniPBRShader")
        # Texture paths
        tex_base_rel = "textures/Steinwurzel_Albedo.tif"
        tex_ao_rel = "textures/Steinwurzel_AmbientOcclusion.tif"
        # tex_height_rel = "textures/Steinwurzel_Height.tif" # Not using height/displacement for now
        tex_normal_rel = "textures/Steinwurzel_Normal.tif"
        tex_rough_rel = "textures/Steinwurzel_Roughness.tif" # Use if you have a specific roughness map

        tex_base_abs = utils.get_asset_path(self._ext_path, tex_base_rel)
        tex_ao_abs = utils.get_asset_path(self._ext_path, tex_ao_rel)
        # tex_height_abs = utils.get_asset_path(self._ext_path, tex_height_rel)
        tex_normal_abs = utils.get_asset_path(self._ext_path, tex_normal_rel)
        tex_rough_abs = utils.get_asset_path(self._ext_path, tex_rough_rel) if tex_rough_rel else None

        required_textures = {"Albedo": tex_base_abs, "Normal": tex_normal_abs, "AO": tex_ao_abs} # Added AO as required now we have name
        missing = [name for name, path in required_textures.items() if not path]
        if missing: carb.log_error(f"{mat_log_prefix} Required texture files not found: {', '.join(missing)}. Aborting."); return False
        # --- End checks ---

        try:
            # 1. Create Material Prim
            material = UsdShade.Material.Define(self._stage, material_path)
            if not material: raise RuntimeError("Failed to define Material prim.")

            # 2. Create the Shader Prim
            shader = UsdShade.Shader.Define(self._stage, shader_prim_path)
            if not shader: raise RuntimeError("Failed to define Shader prim.")

            # 3. Set implementation source using the CORRECT API calls
            token_source_asset = getattr(UsdShade.Tokens, "sourceAsset", "sourceAsset")
            shader.CreateImplementationSourceAttr().Set(token_source_asset)
            shader.SetSourceAsset(Sdf.AssetPath("OmniPBR.mdl"), "mdl")
            shader.SetSourceAssetSubIdentifier("OmniPBR", "mdl")
            carb.log_info(f"{mat_log_prefix} Set sourceAsset attributes correctly on {shader_prim_path}")

            # 4. Connect Material Outputs to the Shader's "out" terminal
            shader_connectable_api = shader.ConnectableAPI()
            if not shader_connectable_api: raise RuntimeError(f"Failed to get ConnectableAPI for shader {shader_prim_path}.")
            # Connect only surface for now, as displacement input seems missing/unused
            material.CreateSurfaceOutput("mdl").ConnectToSource(shader_connectable_api, "out")
            # material.CreateDisplacementOutput("mdl").ConnectToSource(shader_connectable_api, "out") # Comment out displacement

            # 5. Set Texture Inputs directly via CreateInput + Set using CONFIRMED NAMES
            carb.log_info(f"{mat_log_prefix} Setting texture inputs directly via Set...")
            def set_texture_input(input_name: str, texture_path: Optional[str]):
                if texture_path:
                    input_attr = shader.CreateInput(input_name, Sdf.ValueTypeNames.Asset)
                    input_attr.Set(Sdf.AssetPath(texture_path))
                    carb.log_info(f"{mat_log_prefix} Set input '{input_name}' to asset '{texture_path}'")
                else:
                    carb.log_info(f"{mat_log_prefix} Skipping input '{input_name}': Texture path not provided.")

            # --- Use CONFIRMED Names ---
            set_texture_input("diffuse_texture", tex_base_abs)
            set_texture_input("normalmap_texture", tex_normal_abs)
            set_texture_input("ao_texture", tex_ao_abs)
            set_texture_input("reflectionroughness_texture", tex_rough_abs) # Use if you have the map

            # 6. Set Constant Parameters using CONFIRMED NAMES
            # --- Set project_uvw ---
            shader.CreateInput("project_uvw", Sdf.ValueTypeNames.Bool).Set(True)
            carb.log_info(f"{mat_log_prefix} Set input 'project_uvw' to True")

            # --- Set AO mix factor ---
            # Default 1.0 means full AO effect, 0.0 means no effect
            shader.CreateInput("ao_to_diffuse", Sdf.ValueTypeNames.Float).Set(1.0) # Default to full effect
            carb.log_info(f"{mat_log_prefix} Set input 'ao_to_diffuse' to 1.0")

            # Set roughness constant only if texture wasn't assigned
            roughness_input_name = "reflectionroughness_texture" # CONFIRMED NAME
            roughness_const_name = "reflection_roughness_constant" # CONFIRMED NAME
            roughness_input = shader.GetInput(roughness_input_name)
            if not (roughness_input and roughness_input.GetAttr().HasValue()):
                shader.CreateInput(roughness_const_name, Sdf.ValueTypeNames.Float).Set(0.5) # Default roughness
                carb.log_info(f"{mat_log_prefix} Setting {roughness_const_name} to 0.5 (no texture set).")

            # Set metallic constant
            metallic_input_name = "metallic_texture" # Check if this exists if using metallic map
            metallic_const_name = "metallic_constant" # CONFIRMED NAME (from previous check)
            metallic_input = shader.GetInput(metallic_input_name)
            if not (metallic_input and metallic_input.GetAttr().HasValue()):
                shader.CreateInput(metallic_const_name, Sdf.ValueTypeNames.Float).Set(0.0) # Default non-metallic
                carb.log_info(f"{mat_log_prefix} Setting {metallic_const_name} to 0.0 (no texture set).")

            # --- Skip Displacement/Height settings as inputs are unclear ---

            carb.log_info(f"{mat_log_prefix} Material setup complete using correct UsdShade API for {material_path_str}")
            return True

        except Exception as e:
            carb.log_error(f"{mat_log_prefix} Failed during material creation with correct API (v12): {e}")
            traceback.print_exc()
            # Cleanup
            if self._stage and self._stage.GetPrimAtPath(material_path):
                try: delete_prim(material_path_str)
                except Exception as del_e: carb.log_error(f"{mat_log_prefix} Cleanup failed: {del_e}")
            return False

    # --- Final Cleanup Logic ---
    def _check_and_finish_generation(self, task_name: str, was_success: bool):
        """Decrements task counter and calls final cleanup if all tasks are done."""
        check_log_prefix = "[CheckFinish]"
        schedule_cleanup = False
        try:
            with self._generation_lock:
                # Check counter before decrementing
                if self._pending_generation_tasks <= 0:
                    carb.log_warn(f"{check_log_prefix} Task '{task_name}' finished, but counter is already {self._pending_generation_tasks}. Race condition or error?")
                    # Don't decrement if already zero or less
                else:
                    self._pending_generation_tasks -= 1
                    carb.log_info(f"{check_log_prefix} Task '{task_name}' finished (Success: {was_success}). Pending: {self._pending_generation_tasks}")
                    if self._pending_generation_tasks == 0:
                        carb.log_info(f"{check_log_prefix} Counter is 0. Scheduling final cleanup.")
                        schedule_cleanup = True
            # Schedule outside the lock
            if schedule_cleanup:
                self._run_on_main_thread(self._generation_finished_main_thread)
        except Exception as e:
            carb.log_error(f"{check_log_prefix} Error: {e}"); traceback.print_exc()
            # Attempt cleanup directly as a fallback if lock/check fails
            self._run_on_main_thread(lambda: self._generation_finished_main_thread(error=True))


    def _generation_finished_main_thread(self, error:bool = False):
        """Final cleanup and UI re-enabling on main thread."""
        final_log_prefix = "[FinalCleanup]"
        run_cleanup = False
        try:
            # Use lock to ensure atomicity of check and reset
            with self._generation_lock:
                # Only run if counter is exactly 0 or if called explicitly with error
                if self._pending_generation_tasks == 0 or error:
                    if self._pending_generation_tasks != -1: # Prevent re-entry
                        run_cleanup = True
                        self._pending_generation_tasks = -1 # Mark as finished
                        carb.log_info(f"{final_log_prefix} Proceeding with final actions (Error context: {error}). Counter set to -1.")
                    else:
                        carb.log_info(f"{final_log_prefix} Already ran (counter is -1), skipping.")
                elif self._pending_generation_tasks > 0:
                    carb.log_warn(f"{final_log_prefix} Called, but {self._pending_generation_tasks} tasks still pending? Not running.")
                # No else needed for < -1

            if run_cleanup:
                carb.log_info(f"{final_log_prefix} Running final actions...")

                # Re-enable the Generate button via UI proxy
                if self._ui:
                    self._ui.set_generate_button_enabled(True)
                else:
                    carb.log_error(f"{final_log_prefix} UI window proxy invalid, cannot re-enable button.")

                # Final garbage collect
                    self._temp_params_for_objs = None # Clear the temp params now
                gc.collect()
                carb.log_info(f"--- [{constants.EXTENSION_NAME}] Generation Finished ---")

        except Exception as e:
            carb.log_error(f"{final_log_prefix} Error during final cleanup: {e}")
            traceback.print_exc()

def add_terrain_to_stage(stage, vertices, triangles, terrain_prim_path, position=None, orientation=None, material_path=None):
    # ... (Keep the full implementation of add_terrain_to_stage here, including physics/materials/transform fix) ...
    func_name = "add_terrain_to_stage"
    carb.log_info(f"[{func_name}] Entered for {terrain_prim_path}")

    if vertices.size == 0:
        carb.log_error(f"[{func_name}] Vertices array is empty. Aborting.")
        if is_prim_path_valid(terrain_prim_path):
            delete_prim(terrain_prim_path) # Cleanup potentially existing invalid prim
        return None
    if not stage or not stage.GetPrimAtPath(Sdf.Path.absoluteRootPath).IsValid():
        carb.log_error(f"[{func_name}] Stage is not valid. Aborting.")
        return None

    terrain_mesh_prim = None
    try:
        carb.log_info(f"[{func_name}] Defining prim: {terrain_prim_path}")
        terrain_mesh_prim = stage.DefinePrim(Sdf.Path(terrain_prim_path), "Mesh")
        if not terrain_mesh_prim or not terrain_mesh_prim.IsValid():
            carb.log_error(f"[{func_name}] Failed to define prim {terrain_prim_path}.")
            return None

        # Set geometry attributes
        carb.log_info(f"[{func_name}] Setting geometry attributes...")
        num_faces = len(triangles)
        if num_faces == 0:
            carb.log_warn(f"[{func_name}] Triangle array is empty for {terrain_prim_path}, creating mesh with no faces.")
            terrain_mesh_prim.GetAttribute("points").Set(Vt.Vec3fArray.FromNumpy(np.ascontiguousarray(vertices, dtype=np.float32)))
            terrain_mesh_prim.GetAttribute("faceVertexIndices").Set(Vt.IntArray())
            terrain_mesh_prim.GetAttribute("faceVertexCounts").Set(Vt.IntArray())
        else:
            vertices_np = np.ascontiguousarray(vertices, dtype=np.float32)
            triangles_np = np.ascontiguousarray(triangles, dtype=np.int32)
            max_idx = np.max(triangles_np)
            num_verts = len(vertices_np)
            if max_idx >= num_verts:
                carb.log_error(f"[{func_name}] FATAL PRE-SET CHECK: Max triangle index {max_idx} >= num vertices {num_verts}. Aborting.")
                delete_prim(str(terrain_mesh_prim.GetPath()))
                return None
            if not terrain_mesh_prim.GetAttribute("points").Set(Vt.Vec3fArray.FromNumpy(vertices_np)):
                carb.log_error(f"[{func_name}] Failed to set 'points'."); delete_prim(str(terrain_mesh_prim.GetPath())); return None
            if not terrain_mesh_prim.GetAttribute("faceVertexIndices").Set(Vt.IntArray.FromNumpy(triangles_np.flatten())):
                carb.log_error(f"[{func_name}] Failed to set 'faceVertexIndices'."); delete_prim(str(terrain_mesh_prim.GetPath())); return None
            if not terrain_mesh_prim.GetAttribute("faceVertexCounts").Set(Vt.IntArray([3] * num_faces)):
                carb.log_error(f"[{func_name}] Failed to set 'faceVertexCounts'."); delete_prim(str(terrain_mesh_prim.GetPath())); return None
        carb.log_info(f"[{func_name}] Geometry attributes set.")

        # --- Collision & PhysX Setup ---
        UsdPhysics.CollisionAPI.Apply(terrain_mesh_prim)
        collision_api = UsdPhysics.MeshCollisionAPI.Apply(terrain_mesh_prim)
        # SUGGESTION: meshSimplification can be slow/unreliable. Consider 'triangleMesh' for higher fidelity
        # or 'convexHull'/'convexDecomposition' if approximate shape is okay and performance matters.
        collision_api.CreateApproximationAttr().Set(UsdPhysics.Tokens.none)
        try:
            physx_collision_api = PhysxSchema.PhysxCollisionAPI.Apply(terrain_mesh_prim)
            contact_offset_scene = 0.02
            rest_offset_scene = 0.005
            physx_collision_api.GetContactOffsetAttr().Set(contact_offset_scene)
            physx_collision_api.GetRestOffsetAttr().Set(rest_offset_scene)
        except Exception as e: carb.log_warn(f"[{func_name}] Failed PhysX settings: {e}")

        # --- Normals and Material ---
        mesh = UsdGeom.Mesh(terrain_mesh_prim)
        normals_attr = mesh.GetNormalsAttr()
        if normals_attr.IsValid(): normals_attr.Clear()
        mesh.SetNormalsInterpolation(UsdGeom.Tokens.vertex)
        if material_path:
            mat_prim = stage.GetPrimAtPath(material_path)
            # Check if prim exists AND is specifically a Material
            if mat_prim and mat_prim.IsA(UsdShade.Material):
                try:
                    UsdShade.MaterialBindingAPI(terrain_mesh_prim).Bind(UsdShade.Material(mat_prim))
                    carb.log_info(f"[{func_name}] Material '{material_path}' bound successfully.")
                except Exception as bind_err:
                    carb.log_error(f"[{func_name}] Failed to bind material '{material_path}': {bind_err}")
            elif mat_prim:
                carb.log_warn(f"[{func_name}] Prim at '{material_path}' exists but is not a UsdShade.Material (Type: {mat_prim.GetTypeName()}). No material bound.")
            else:
                # This case might happen if _create_terrain_material failed earlier
                carb.log_warn(f"[{func_name}] Material prim not found at '{material_path}'. No material bound.")
        else:
            carb.log_info(f"[{func_name}] No material path provided. No material bound.")

        carb.log_info(f"[{func_name}] Successfully completed for {terrain_prim_path}")
        return XFormPrim(prim_paths_expr=str(terrain_mesh_prim.GetPath()))

    except Exception as e:
        carb.log_error(f"[{func_name}] Exception occurred for {terrain_prim_path}: {e}")
        traceback.print_exc()
        if terrain_mesh_prim and terrain_mesh_prim.IsValid():
            carb.log_warn(f"[{func_name}] Deleting prim {terrain_mesh_prim.GetPath()} due to exception.")
            try: delete_prim(str(terrain_mesh_prim.GetPath()))
            except Exception as del_e: carb.log_error(f"Failed to delete prim: {del_e}")
        return None
    finally:
        vertices_np = triangles_np = None; gc.collect()
        carb.log_info(f"[{func_name}] Exiting.")