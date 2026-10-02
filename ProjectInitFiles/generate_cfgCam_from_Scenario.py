import json
import numpy as np
import math
from scipy.spatial.transform import Rotation as R_sci

# --- 1. File Paths ---
json_filepath = "/home/base/ADC_27/launch/scenario_launch/silesia_scenario.json"
output_filepath = "/home/base/ADC_27/configData/cam_config/CameraConfig.yaml"

# --- 2. Load BeamNG Scenario JSON ---
with open(json_filepath, "r") as f:
    scenario_data = json.load(f)

# --- 3. Dynamically Find Left and Right Cameras ---
ego_sensors = scenario_data["vehicles"][0]["sensors"]

cam_l = next((s for s in ego_sensors if "camera_L" in s["name"] or "cam_l" in s["name"].lower()), None)
cam_r = next((s for s in ego_sensors if "camera_R" in s["name"] or "cam_r" in s["name"].lower()), None)

if not cam_l or not cam_r:
    raise ValueError("Could not find both stereo cameras ('camera_L' and 'camera_R') in the JSON configuration!")

# Extract resolution and vertical field of view
width, height = cam_l["resolution"]
fov_y = cam_l["field_of_view_y"]

# --- 4. Compute Dynamic Translation (T) and 3D Baseline Magnitude ---
pos_l = np.array(cam_l["pos"])
pos_r = np.array(cam_r["pos"])
T_vector = pos_r - pos_l

# General 3D Euclidean distance (baseline)
baseline = np.linalg.norm(T_vector)

# --- 5. Compute Dynamic Relative Rotation (R) ---
rot_l = R_sci.from_quat(cam_l.get("rot", [0, 0, 0, 1]))
rot_r = R_sci.from_quat(cam_r.get("rot", [0, 0, 0, 1]))

# Relative rotation of right camera with respect to left camera: R = R_right * inv(R_left)
R_relative = rot_r * rot_l.inv()
R_matrix = R_relative.as_matrix()

# --- 6. Calculate Intrinsic Parameters ---
fy = height / (2.0 * math.tan(math.radians(fov_y / 2.0)))
fx = fy  # Assuming square pixels in BeamNG
cx = width / 2.0
cy = height / 2.0
q_baseline_term = 1.0 / baseline if baseline > 0 else 0.0

# --- 7. Format OpenCV YAML Content ---
yaml_content = f"""%YAML:1.0
---
image_width: {width}
image_height: {height}

K1: !!opencv-matrix
   rows: 3
   cols: 3
   dt: d
   data: [ {fx:.4f}, 0.0, {cx:.4f},
             0.0, {fy:.4f}, {cy:.4f},
             0.0, 0.0, 1.0 ]

D1: !!opencv-matrix
   rows: 1
   cols: 5
   dt: d
   data: [ 0.0, 0.0, 0.0, 0.0, 0.0 ]

K2: !!opencv-matrix
   rows: 3
   cols: 3
   dt: d
   data: [ {fx:.4f}, 0.0, {cx:.4f},
             0.0, {fy:.4f}, {cy:.4f},
             0.0, 0.0, 1.0 ]

D2: !!opencv-matrix
   rows: 1
   cols: 5
   dt: d
   data: [ 0.0, 0.0, 0.0, 0.0, 0.0 ]

R: !!opencv-matrix
   rows: 3
   cols: 3
   dt: d
   data: [ {R_matrix[0,0]:.6f}, {R_matrix[0,1]:.6f}, {R_matrix[0,2]:.6f},
           {R_matrix[1,0]:.6f}, {R_matrix[1,1]:.6f}, {R_matrix[1,2]:.6f},
           {R_matrix[2,0]:.6f}, {R_matrix[2,1]:.6f}, {R_matrix[2,2]:.6f} ]

T: !!opencv-matrix
   rows: 3
   cols: 1
   dt: d
   data: [ {T_vector[0]:.5f}, {T_vector[1]:.5f}, {T_vector[2]:.5f} ]

Q: !!opencv-matrix
   rows: 4
   cols: 4
   dt: d
   data: [ 1.0, 0.0, 0.0, {-cx:.4f},
           0.0, 1.0, 0.0, {-cy:.4f},
           0.0, 0.0, 0.0, {fx:.4f},
           0.0, 0.0, {q_baseline_term:.6f}, 0.0 ]
"""

# --- 8. Write to Output YAML File ---
with open(output_filepath, "w") as f:
    f.write(yaml_content)

print(f"Successfully generated {output_filepath} from {json_filepath}!")