#!/usr/bin/env python3

import json
import numpy as np
import sys
import math

# ==============================================================================
# == OSDome Simulation Configuration Parameters ==
# ==============================================================================

# -- Output File --
FILENAME = "osdome_singleshot_datasheet_spec.json"

# -- Fidelity / Resolution Control --
NUM_VERTICAL_CHANNELS = 128
HORIZONTAL_POINTS_PER_360_RING = 512

# -- Sensor Characteristics (Based on OSDome Datasheet) --
VERTICAL_FOV_DEG = 90
HORIZONTAL_AZIMUTH_RANGE_DEG = 360.0
# Adjust this value based on how the sensor's native 0-degree azimuth
# is oriented relative to your desired "forward" direction.
# 0.0   = Sensor's 0 azimuth is forward (No offset needed)
# 180.0 = Sensor's 0 azimuth points backward (Apply 180 deg offset)
# 90.0  = Sensor's 0 azimuth points left (Apply 90 deg offset)
# -90.0 = Sensor's 0 azimuth points right (Apply -90 deg offset / 270 deg)
AZIMUTH_OFFSET_DEG = 180.0 # <--- Set based on "opposite forward"

USE_UNIFORM_VERTICAL_SPACING = True # False = Ouster-like non-uniform
NEAR_RANGE_M = 0.5
FAR_RANGE_M = 200.0
SCAN_RATE_HZ = 10.0
MAX_RETURNS = 2
WAVELENGTH_NM = 865.0
AZIMUTH_ERROR_STD_DEG = 0.05
ELEVATION_ERROR_STD_DEG = 0.05

# ==============================================================================
# == Script Logic ==
# ==============================================================================

total_points = NUM_VERTICAL_CHANNELS * HORIZONTAL_POINTS_PER_360_RING

print("--- Ouster OSDome LiDAR Singleshot Config Generator (Datasheet Spec) ---")
print(f"Filename: {FILENAME}")
print(f"Vertical Channels: {NUM_VERTICAL_CHANNELS}")
print(f"Horizontal Points/Ring (360°): {HORIZONTAL_POINTS_PER_360_RING}")
print(f"Total Points: {total_points}")
print(f"Vertical FoV Configured: +/- {VERTICAL_FOV_DEG / 2.0}° ({-(VERTICAL_FOV_DEG / 2.0):.1f} to +{VERTICAL_FOV_DEG / 2.0:.1f})")
print(f"Horizontal Azimuth Configured: 0° to {HORIZONTAL_AZIMUTH_RANGE_DEG}°")
print(f"Applying Azimuth Offset: {AZIMUTH_OFFSET_DEG}°") # <--- Added print statement
print(f"Vertical Spacing: {'Uniform' if USE_UNIFORM_VERTICAL_SPACING else 'Non-Uniform (Ouster Dome-like)'}")
print(f"Range: {NEAR_RANGE_M} m to {FAR_RANGE_M} m")
print(f"Scan Rate: {SCAN_RATE_HZ} Hz")
print(f"Max Returns: {MAX_RETURNS}")
print("--------------------------------------------------------------------")

# --- Generate Vertical Elevation Angles (-90 to +90) ---
min_elevation_deg = -0
max_elevation_deg = VERTICAL_FOV_DEG  # +90 degrees
min_elevation_rad = math.radians(min_elevation_deg)
max_elevation_rad = math.radians(max_elevation_deg)

if USE_UNIFORM_VERTICAL_SPACING:
    unique_elevations_rad = np.linspace(min_elevation_rad, max_elevation_rad, NUM_VERTICAL_CHANNELS)
else:
    linear_spacing = np.linspace(-math.pi / 2.0, math.pi / 2.0, NUM_VERTICAL_CHANNELS)
    unique_elevations_rad = np.sin(linear_spacing) * max_elevation_rad

unique_elevations_deg = np.degrees(unique_elevations_rad)
#print(f"Generated {len(unique_elevations_deg)} unique elevation angles spanning {unique_elevations_deg.min():.2f}° to {unique_elevations_deg.max():.2f}°")
elevation_deg_full = np.repeat(unique_elevations_deg, HORIZONTAL_POINTS_PER_360_RING).round(3).tolist()
#print(f"Generated {len(elevation_deg_full)} total elevation angle entries.")

# --- Generate Horizontal Azimuth Angles (0 to 360) with Offset ---
# Generate base points from 0 up to (but not including) 360
unique_azimuths_deg_base = np.linspace(0.0, HORIZONTAL_AZIMUTH_RANGE_DEG, HORIZONTAL_POINTS_PER_360_RING, endpoint=False)

# --- Apply the offset and wrap angles to [0, 360) range ---
unique_azimuths_deg_rotated = (unique_azimuths_deg_base + AZIMUTH_OFFSET_DEG) % 360.0

# Ensure the order is correct if needed (e.g., strictly increasing for some tools)
# If the offset causes a wrap-around (e.g., 180 deg offset makes 270 original become 90 rotated),
# the order might change. For most simulators, the order within the list doesn't matter
# as long as it corresponds correctly to the other arrays (elevation, channelId etc.).
# The tiling below preserves the correspondence.

# Repeat this rotated 360 pattern for each vertical channel/line
azimuth_deg_full = np.tile(unique_azimuths_deg_rotated, NUM_VERTICAL_CHANNELS).round(3).tolist()
#print(f"Generated {len(azimuth_deg_full)} total azimuth angle entries (0-360 range, offset applied).")

# --- Generate Other Arrays ---
fire_time_ns_full = [0] * total_points
channel_id_full = np.repeat(np.arange(NUM_VERTICAL_CHANNELS), HORIZONTAL_POINTS_PER_360_RING).tolist()
range_id_full = [0] * total_points
bank_full = channel_id_full[:]
#print(f"Generated fire times, channel IDs, range IDs, and bank IDs.")

# --- Create the Base JSON Structure ---
lidar_config = {
    "class": "sensor",
    "type": "lidar",
    "name": f"OusterDome Snapshot {NUM_VERTICAL_CHANNELS}x{HORIZONTAL_POINTS_PER_360_RING} ({total_points}) AzOffset{AZIMUTH_OFFSET_DEG}", # Added offset to name
    "driveWorksId": "GENERIC",
    "profile": {
        "scanType": "solidState",
        "intensityProcessing": "normalization",
        "rotationDirection": "CW", # Defines sensor's *internal* rotation if applicable
        "rayType": "IDEALIZED",
        "nearRangeM": NEAR_RANGE_M,
        "farRangeM": FAR_RANGE_M,
        "effectiveApertureSize": 0.01,
        "focusDistM": 0.1,
        "rangeResolutionM": 0.01,
        "rangeAccuracyM": 0.03,
        "avgPowerW": 0.002,
        "minReflectance": 0.05,
        "minReflectanceRange": FAR_RANGE_M / 2.0,
        "wavelengthNm": WAVELENGTH_NM,
        "pulseTimeNs": 10,
        "maxReturns": MAX_RETURNS,
        "scanRateBaseHz": SCAN_RATE_HZ,
        "reportRateBaseHz": SCAN_RATE_HZ,
        "numberOfEmitters": total_points,
        "numberOfChannels": total_points,
        "rangeCount": 1,
        "ranges": [
            {"min": NEAR_RANGE_M, "max": FAR_RANGE_M}
        ],
        "azimuthErrorMean": 0.0,
        "azimuthErrorStd": AZIMUTH_ERROR_STD_DEG,
        "elevationErrorMean": 0.0,
        "elevationErrorStd": ELEVATION_ERROR_STD_DEG,
        "stateResolutionStep": 1,
        "numLines": NUM_VERTICAL_CHANNELS,
        "numRaysPerLine": [HORIZONTAL_POINTS_PER_360_RING] * NUM_VERTICAL_CHANNELS,
        "emitterStateCount": 1,
        "emitterStates": [
            {
                "azimuthDeg": azimuth_deg_full,
                "elevationDeg": elevation_deg_full,
                "fireTimeNs": fire_time_ns_full,
                "channelId": channel_id_full,
                "rangeId": range_id_full,
                "bank": bank_full
            }
        ],
        "intensityMappingType": "LINEAR"
    }
}
#print("Constructed JSON data structure in memory.")

# --- Save the updated JSON data back to the file ---
try:
    with open(FILENAME, 'w') as f:
        json.dump(lidar_config, f, indent=2)
    print(f"Successfully saved OSDome singleshot configuration to {FILENAME}.")
except IOError as e:
    print(f"ERROR: Could not write to file {FILENAME}: {e}")
    sys.exit(1)

print("--- Script finished ---")