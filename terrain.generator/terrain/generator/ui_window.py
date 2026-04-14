import weakref
from typing import Optional, Dict, Any

import carb
import omni.ui as ui
import omni.kit.app # For getting extension manager later if needed for icon path etc.

# Import UI utils and styling
from isaacsim.gui.components.style import get_style, VERTICAL_SPACING
from isaacsim.gui.components.ui_utils import (
    btn_builder, cb_builder, dropdown_builder, float_builder, int_builder, str_builder, xyz_builder,
    setup_ui_headers
)

# Import constants and utils relative to the package
from .constants import EXTENSION_NAME, DOC_LINK, DEFAULT_PARENT_PATH
# from .utils import get_extension_path # Example if needed later

class ForestGeneratorWindow:
    """Manages the UI window, elements, and callbacks for the Forest Generator."""

    def __init__(self, ext_id: str):
        """
        Initialize the UI Window.

        Args:
            ext_id: The extension ID, used for headers etc.
        """
        self._logic = None # Initialize as None, will be set later by extension
        self._ext_id = ext_id
        self._window: Optional[ui.Window] = None
        self._models: Dict[str, ui.AbstractValueModel] = {}
        # If using file pickers from ui_utils, might need self._filepickers = {} here

        # Build the window immediately on initialization
        # self.build_window() # Or call explicitly from extension.py

    def destroy(self):
        """Destroy the window and release resources."""
        if self._window:
            self._window.destroy()
        self._window = None
        self._models.clear()
        self._logic = None # Break weakref

    def build_window(self):
        """Builds the main UI window or makes it visible if it already exists."""
        if self._window:
            self._window.visible = True
            return

        # Get style can be defined globally or imported if needed elsewhere
        # style = get_style()

        self._window = ui.Window(
            title=EXTENSION_NAME, width=400, height=700, visible=True,
            dockPreference=ui.DockPreference.LEFT_BOTTOM,
            closed_handler=self._on_window_close # Use method from this class
        )
        self._window.set_visibility_changed_fn(self._on_visibility_changed) # Use method from this class

        with self._window.frame:
            # Main VStack to handle sections: Header, Scrolling Content, Buttons
            with ui.VStack(spacing=VERTICAL_SPACING): # Removed height=0

                # --- Fixed Buttons Area (remains outside scrolling) ---
                self._build_action_buttons_ui()
                ui.Separator()

                with ui.ScrollingFrame(
                        horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
                        vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
                        height=680 # Take remaining vertical space
                ):
                    # --- Inner VStack contains ALL scrollable content ---
                    # YES height=0 here (grow with content)
                    with ui.VStack(spacing=VERTICAL_SPACING, height=0):
                        setup_ui_headers(self._ext_id, __file__, EXTENSION_NAME, DOC_LINK,
                                         "Procedurally generate forest environments using modern Isaac Sim tools.")

                        # Parameter Sections (as before)
                        self._build_general_params_ui()
                        self._build_terrain_params_ui()
                        self._build_forest_params_ui()
                        self._build_rock_params_ui()
                        self._build_objects_params_ui()
                        self._build_vegetation_params_ui()
                        self._build_environment_params_ui()

                        # Optional padding at the very bottom of scroll area
                        ui.Spacer(height=5)

    # --- Window Callbacks ---
    def _on_window_close(self, window):
        """Called when the window is closed."""
        if window == self._window:
            self._window = None
            # if hasattr(self, "_filepickers"): self._filepickers.clear() # Example cleanup

    def _on_visibility_changed(self, visible):
        """Called when the window visibility changes."""
        # Can be used to pause/resume updates if needed
        pass

    # --- UI Getters ---
    def _get_value(self, key: str, value_type: str, default: Any = None) -> Any:
        """Safely retrieves a value from the UI models dictionary."""
        model = self._models.get(key)
        if model is None:
            # carb.log_warn(f"UI model key '{key}' not found. Returning default.") # Optional warning
            return default
        try:
            # Use 'string' for string type, map others if needed
            method_suffix = value_type
            if value_type == "str":
                method_suffix = "string"

            # Use built-in methods like get_value_as_int, get_value_as_float, get_value_as_string etc.
            getter = getattr(model, f"get_value_as_{method_suffix}", None) # Use corrected suffix
            if getter:
                return getter()
            else:
                # --- FIX: Improve error message ---
                carb.log_error(f"UI model '{key}' (type: {type(model)}) has no method 'get_value_as_{method_suffix}'.")
                # --- FIX END ---
                return default
        except Exception as e:
            carb.log_error(f"Error getting value for UI model '{key}': {e}")
            return default

    # --- UI Parameter Sections (Copied from original, ensure they use self._models) ---
    def _build_general_params_ui(self):
        # Use self._models["key"] = builder(...)
        with ui.CollapsableFrame("General Settings", collapsed=False, style=get_style(), height=0):
            with ui.VStack(spacing=5, height=0):
                model_parent_path = str_builder("Parent Path", default_val=DEFAULT_PARENT_PATH)
                self._models["parent_path"] = model_parent_path # Use self._models
                model_global_seed = int_builder("Global Seed", default_val=0, min=0, tooltip="Master seed (0=random)")
                self._models["global_seed"] = model_global_seed # Use self._models

    def _build_terrain_params_ui(self):
        with ui.CollapsableFrame("Terrain", collapsed=False, style=get_style()):
            with ui.VStack(spacing=5, height=0):
                self._models["area_x"] = float_builder("Area Length (X, m)", default_val=20.0, min=1.0)
                self._models["area_y"] = float_builder("Area Width (Y, m)", default_val=20.0, min=1.0)
                self._models["terrain_roughness"] = float_builder("Max Elevation Diff (m)", default_val=3.0, min=0.0)
                self._models["terrain_h_scale"] = float_builder("Horiz. Scale (m)", default_val=0.25, min=0.05, tooltip="Grid size for generation")
                self._models["terrain_v_scale"] = float_builder("Vert. Scale (m)", default_val=0.01, min=0.001, tooltip="Height multiplier")
                self._models["terrain_seed"] = int_builder("Terrain Seed", default_val=0, min=0)
                self._models["terrain_octaves"] = int_builder("Noise Octaves", default_val=6, min=1, max=12)
                self._models["terrain_persistence"] = float_builder("Noise Persistence", default_val=0.3, min=0.1, max=1.0)
                self._models["terrain_lacunarity"] = float_builder("Noise Lacunarity", default_val=2.0, min=1.0, max=4.0)


#                self._models["terrain_material"] = str_builder(
#                    "Material Path", default_val="",
#                    tooltip="Path to a UsdShade Material prim (e.g., /World/Looks/GroundMaterial)",
#                    use_folder_picker=True, )

    def _build_forest_params_ui(self):
        with ui.CollapsableFrame("Trees", collapsed=True, style=get_style()): # Start collapsed
            with ui.VStack(spacing=5, height=0):
                self._models["tree_density"] = int_builder("Density (/100m²)", default_val=2, min=0)
                self._models["tree_seed"] = int_builder("Tree Seed", default_val=0, min=0)
                with ui.HStack():
                    ui.Label("Age Range:", width=ui.Percent(30))
                    self._models["tree_age_min"] = int_builder("Min", default_val=80, min=1)
                    self._models["tree_age_max"] = int_builder("Max", default_val=120, min=1)
                self._models["tree_scale_variation"] = float_builder("Scale Rand %", default_val=10.0, min=0.0)
                self._models["tree_height_variation"] = float_builder("Height Rand %", default_val=10.0, min=0.0)
                self._models["tree_max_tilt"] = float_builder("Max Tilt (Deg)", default_val=5.0, min=0.0, max=45.0)
                ui.Label("Proportions (%):")
                with ui.VStack(spacing=2):
                    self._models["birch_prop"] = float_builder("  Birch", default_val=30.0, min=0.0)
                    self._models["spruce_prop"] = float_builder("  Spruce", default_val=00.0, min=0.0)
                    self._models["pine_prop"] = float_builder("  Pine", default_val=60.0, min=0.0)

    def _build_rock_params_ui(self):
        with ui.CollapsableFrame("Rocks", collapsed=True, style=get_style()):
            with ui.VStack(spacing=5, height=0):
                self._models["rock_density"] = int_builder("Density (/100m²)", default_val=2, min=0)
                self._models["rock_seed"] = int_builder("Rock Seed", default_val=0, min=0)
                with ui.HStack():
                    ui.Label("Scale Range:", width=ui.Percent(30))
                    self._models["rock_scale_min"] = float_builder("Min", default_val=0.2, min=0.05)
                    self._models["rock_scale_max"] = float_builder("Max", default_val=0.5, min=0.1)
                self._models["rock_orient_variation"] = float_builder("Max Orient Rand (Deg)", default_val=15.0, min=0.0, max=180.0)

    def _build_objects_params_ui(self):
        with ui.CollapsableFrame("Objects", collapsed=True, style=get_style()):
            with ui.VStack(spacing=5, height=0):
                self._models["container_density"] = int_builder("Container Density (/100m²)", default_val=0, min=0)
                self._models["objects_seed"] = int_builder("Objects Seed", default_val=0, min=0)
                with ui.HStack():
                    ui.Label("Scale Range:", width=ui.Percent(30))
                    self._models["objects_scale_min"] = float_builder("Min", default_val=1, min=0.05)
                    self._models["objects_scale_max"] = float_builder("Max", default_val=1, min=0.1)
                self._models["objects_orient_variation"] = float_builder("Max Orient Rand (Deg)", default_val=15.0, min=0.0, max=180.0)

    def _build_vegetation_params_ui(self):
        with ui.CollapsableFrame("Ground Vegetation", collapsed=True, style=get_style()):
            with ui.VStack(spacing=5, height=0):
                cb_model = cb_builder("Spawn Vegetation", default_val=True)
                self._models["spawn_vegetation"] = cb_model
                self._models["veg_density"] = int_builder("Density (/100m²)", default_val=10, min=0)
                self._models["grass_density"] = int_builder("Grass Density (/100m²)", default_val=10, min=0)
                self._models["switchgrass_density"] = int_builder("Switchgrass Density (/100m²)", default_val=5, min=0)
                self._models["veg_seed"] = int_builder("Veg. Seed", default_val=0, min=0)
                # Nested frames for blueberry/scale variation
                cluster_frame = ui.CollapsableFrame("Blueberry Cluster Settings", collapsed=True, style=get_style())
                with cluster_frame:
                    with ui.VStack(spacing=5, height=0):
                        self._models["veg_cluster_chance"] = float_builder("Cluster Chance %", default_val=30.0, min=0.0, max=100.0)
                        self._models["veg_cluster_min"] = int_builder("Min Size", default_val=5, min=1)
                        self._models["veg_cluster_max"] = int_builder("Max Size", default_val=15, min=2)
                        self._models["veg_cluster_radius"] = float_builder("Radius (m)", default_val=0.15, min=0.01)
                scale_frame = ui.CollapsableFrame("Scale Variation", collapsed=True, style=get_style())
                with scale_frame:
                    with ui.VStack(spacing=5, height=0):
                        with ui.HStack():
                            ui.Label("XY Scale:", width=ui.Percent(30))
                            self._models["veg_scale_xy_min"] = float_builder("Min", default_val=0.7, min=0.1)
                            self._models["veg_scale_xy_max"] = float_builder("Max", default_val=1.1, min=0.2)
                        with ui.HStack():
                            ui.Label("Z Scale Mult:", width=ui.Percent(30))
                            self._models["veg_scale_z_min"] = float_builder("Min", default_val=0.8, min=0.1)
                            self._models["veg_scale_z_max"] = float_builder("Max", default_val=1.0, min=0.2)
                # Enable/disable logic
                def toggle_veg_params(enabled):
                    for w in [cluster_frame, scale_frame]: w.enabled = enabled
                toggle_veg_params(cb_model.get_value_as_bool()) # Initial state
                cb_model.add_value_changed_fn(lambda m: toggle_veg_params(m.get_value_as_bool()))

    def _build_environment_params_ui(self):
        with ui.CollapsableFrame("Environment Lighting", collapsed=True, style=get_style()):
            with ui.VStack(spacing=5, height=0):
                ui.Label("HDRI Dome Light:")
                self._models["hdri_path"] = str_builder(
                    "Path (.hdr)", default_val="", tooltip="Path to .hdr file",
                    use_folder_picker=True, item_filter_fn=lambda item: not item.is_folder and item.path.lower().endswith(".hdr"),)
                self._models["light_intensity"] = float_builder("Intensity", default_val=1000.0, min=0.0)
                xyz_models = xyz_builder("Rotation (Deg)", default_val=[0,0,0], axis_count=3)
                if xyz_models and len(xyz_models) == 3:
                    self._models["light_rot_x"] = xyz_models[0]
                    self._models["light_rot_y"] = xyz_models[1]
                    self._models["light_rot_z"] = xyz_models[2]
                else: carb.log_error("Failed to get models from xyz_builder for light rotation.")
                # Button to trigger HDRI load action in generator_logic
                btn_builder("Load/Update HDRI", on_clicked_fn=self._on_load_update_hdr_clicked)


    # --- Action Buttons ---
    def _build_action_buttons_ui(self):
        """Builds the Generate All and Clear All buttons."""
        with ui.VStack(spacing=8):
            with ui.HStack(spacing=5):
                # Store the generate button model for enabling/disabling by GeneratorLogic
                self._models["generate_all_button"] = btn_builder(
                    "Generate All",
                    on_clicked_fn=self._on_generate_all_clicked, # Call helper
                    tooltip="Generate world based on current settings"
                )
                btn_builder("Clear All", on_clicked_fn=self._on_clear_all_clicked, # Call helper
                            tooltip="Remove all generated assets")

    # --- UI Action Callbacks (Trigger GeneratorLogic) ---
    def _on_generate_all_clicked(self):
        """Callback when 'Generate All' button is clicked."""
        if self._logic:
            self._logic.start_generation() # Trigger the logic class

    def _on_clear_all_clicked(self):
        """Callback when 'Clear All' button is clicked."""
        if self._logic:
            self._logic.clear_all_generated_assets() # Trigger the logic class

    def _on_load_update_hdr_clicked(self):
        """Callback when 'Load/Update HDRI' button is clicked."""
        if self._logic:
            self._logic.load_update_hdr() # Trigger the logic class

    # --- Methods called by GeneratorLogic ---
    def set_generate_button_enabled(self, enabled: bool):
        """Allows GeneratorLogic to enable/disable the button."""
        if "generate_all_button" in self._models:
            btn_model = self._models["generate_all_button"]
            if btn_model: # Check model is not None
                btn_model.enabled = enabled
            else: carb.log_error("Generate button model is None, cannot set enabled state.")
        else: carb.log_warn("Generate button model key not found.")