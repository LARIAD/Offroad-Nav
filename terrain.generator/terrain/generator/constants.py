# Constants used throughout the extension

EXTENSION_NAME = "Terrain Generator"
DOC_LINK = "" # Optional: Add a link to your documentation

# Scene Units configuration
# If your assets (trees, rocks) are modeled in cm, set this to 100.0
# If they are modeled in meters (like the default Isaac Sim assets), keep it 1.0
# This affects how scale and physics offsets are applied.
METERS_PER_UNIT = 1.0

# Default location for generated assets
DEFAULT_PARENT_PATH = "/World/GeneratedForest"
PROTOTYPES_PARENT_PATH = DEFAULT_PARENT_PATH + "/Prototypes"

# Default spawn height for raycasting down (in scene units)
DEFAULT_SPAWN_HEIGHT_SCENE_UNITS = 100.0

DEFAULT_CONTACT_OFFSET_METERS = 0.02
DEFAULT_REST_OFFSET_METERS = 0.005

DEFAULT_HDR_REL_PATH = "textures/autumn_park_2k.hdr"