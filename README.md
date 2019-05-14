# aruco_positioning

This Program tracks aruco marker positions in a 3D space with a single webcam
![Screenshot](screenshot.png?raw=true "Screenshot")

## Building

This Program needs opencv and opencv_contrib > 2.8 to compile

clone the repository
cmake .
make


## Usage

./aruco_tracking -arg

-g -> Generate aruco markers in root directory
-c -> Calibrate camera
-m -> Start position tracking
