import traceback
import weakref
from typing import Optional, List

import carb
import omni.ext
import omni.kit.app
import omni.usd
from omni.kit.menu.utils import MenuItemDescription, add_menu_items, remove_menu_items
from pxr import Gf, Usd, UsdGeom, UsdLux, UsdPhysics, UsdShade, PhysxSchema, Sdf, Vt

# Import relative package components
from .constants import EXTENSION_NAME
from .ui_window import ForestGeneratorWindow
from .generator_logic import GeneratorLogic # Need the logic class
from .utils import get_extension_path # Need helper

class ForestGeneratorExtension(omni.ext.IExt):
    """Main extension class for the Modern Forest Generator."""

    def on_startup(self, ext_id: str):
        self._ext_id = ext_id
        self._ext_path = get_extension_path(self._ext_id)
        carb.log_info(f"[{EXTENSION_NAME}] Starting up. Path: {self._ext_path}")

        self._usd_context = omni.usd.get_context()
        self._stage = self._usd_context.get_stage() # Get initial stage if open

        # Instances of our separated logic and UI classes
        self._window: Optional[ForestGeneratorWindow] = None
        # Pass None initially for stage, update in _on_stage_event
        self._logic: Optional[GeneratorLogic] = None
        # Keep track of stage event subscription
        self._stage_event_sub = None

        # Subscribe to stage events to handle stage opening/closing
        self._stage_event_sub = self._usd_context.get_stage_event_stream().create_subscription_to_pop(
            self._on_stage_event, name=f"{EXTENSION_NAME} Stage Listener"
        )

        # Add menu item to open the window
        self._menu_items = [
            #MenuItemDescription(name="Procedural", sub_menu=[
                MenuItemDescription(name=EXTENSION_NAME, onclick_fn=self._build_or_show_window)
            #])
        ]
        add_menu_items(self._menu_items, "Window")

        # Initialize logic and UI if stage is already open
        if self._stage:
            self._initialize_logic_and_ui()
        else:
            carb.log_warn(f"[{EXTENSION_NAME}] Stage not open on startup. UI/Logic will initialize on stage open.")

    def on_shutdown(self):
        carb.log_info(f"[{EXTENSION_NAME}] Shutting down.")
        remove_menu_items(self._menu_items, "Window")

        # Clean up subscriptions
        self._stage_event_sub = None

        # Clean up logic and UI instances
        if self._logic:
            self._logic.cleanup()
            self._logic = None
        if self._window:
            self._window.destroy()
            self._window = None

        carb.log_info(f"[{EXTENSION_NAME}] Shutdown complete.")

    def _initialize_logic_and_ui(self):
        """Creates instances and sets up weak references between Logic and UI."""
        if not self._stage or not self._stage.GetPrimAtPath(Sdf.Path.absoluteRootPath).IsValid():
            carb.log_error("Cannot initialize logic and UI without a valid stage.")
            return

        if self._logic is not None and self._window is not None:
            carb.log_info("GeneratorLogic and ForestGeneratorWindow already initialized.")
            self._logic._stage = self._stage # Ensure stage is up-to-date
            return

        carb.log_info("Initializing GeneratorLogic and ForestGeneratorWindow...")
        logic_instance = None
        window_instance = None

        try:
            # --- STEP 1: Create instances WITHOUT cross-references ---
            carb.log_info("Creating GeneratorLogic instance...")
            logic_instance = GeneratorLogic( # No UI instance passed here
                stage=self._stage,
                ext_path=self._ext_path
            )
            if not logic_instance: raise RuntimeError("GeneratorLogic creation failed")

            carb.log_info("Creating ForestGeneratorWindow instance...")
            window_instance = ForestGeneratorWindow( # No logic instance passed here
                ext_id=self._ext_id
            )
            if not window_instance: raise RuntimeError("ForestGeneratorWindow creation failed")

            # --- STEP 2: Set weak references AFTER both instances exist ---
            carb.log_info("Setting weak references between instances...")
            logic_instance._ui = weakref.proxy(window_instance)
            window_instance._logic = weakref.proxy(logic_instance)

            # --- STEP 3: Assign fully initialized instances to self ---
            self._logic = logic_instance
            self._window = window_instance

            carb.log_info("GeneratorLogic and ForestGeneratorWindow initialization complete.")

        except Exception as e:
            carb.log_error(f"Failed during initialization: {e}")
            traceback.print_exc()
            # Cleanup partially created instances on error
            if logic_instance and hasattr(logic_instance, "cleanup"): logic_instance.cleanup()
            if window_instance and hasattr(window_instance, "destroy"): window_instance.destroy()
            self._logic = None
            self._window = None

    def _build_or_show_window(self):
        """Builds the UI window if it doesn't exist, or makes it visible."""
        if not self._stage:
            carb.log_error("Cannot build window, stage is not open.")
            # Optionally show a user notification
            omni.kit.notification_manager.post_notification("Please open a stage first.", status=omni.kit.notification_manager.Status.WARNING)
            return

        if self._window is None:
            # Should have been initialized by _on_stage_event if stage is open
            carb.log_warn("UI Window instance is None, attempting re-initialization.")
            self._initialize_logic_and_ui()
            if self._window is None: # Still None after attempt
                carb.log_error("Failed to initialize UI window instance.")
                return

        # Call the build method on the window instance
        self._window.build_window()

    def _on_stage_event(self, event):
        """Handles stage events (Open, Close)."""
        event_type = event.type
        if event_type == int(omni.usd.StageEventType.OPENED):
            self._stage = self._usd_context.get_stage()
            carb.log_info(f"[{EXTENSION_NAME}] Stage opened: {self._stage.GetRootLayer().identifier}")
            # Initialize or update logic and UI now that stage is ready
            self._initialize_logic_and_ui()

        elif event_type == int(omni.usd.StageEventType.CLOSED):
            carb.log_info(f"[{EXTENSION_NAME}] Stage closed.")
            self._stage = None
            # Clean up logic and UI related to the closed stage
            if self._logic:
                self._logic.cleanup() # Call cleanup method
                self._logic = None
            if self._window:
                # Optionally destroy window on stage close, or just disable parts
                self._window.destroy() # Destroy window completely
                self._window = None

        elif event_type == int(omni.usd.StageEventType.SAVED) or event_type == int(omni.usd.StageEventType.SAVE_FAILED):
            # May not need handling here, but useful place if needed
            pass