# Transformation Library

## Description

Nav2 / ROS 2 Convention: Uses FLU (Front-Left-Up) for the robot (base_link) and ENU (East-North-Up) for the world/map (map / odom).
Camera Convention: Standard computer vision cameras (like OpenCV) use RDF (Right-Down-Forward, where Z is depth out of the lens, X is right, Y is down).
The Goal: Your transformation function takes points or poses measured in the camera's frame and rotates/translates them into the robot's body frame (base_link), aligning with Nav2's expectations.

### Transformation

The transformation is:



## Usage

1. Build the MATLAB Coder library.
2. Include the generated header.
3. Link the library to your C++ project.