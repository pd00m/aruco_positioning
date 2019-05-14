
# aruco_positioning

This Program tracks aruco marker positions in a 3D space with a single webcam.

![Screenshot](screenshot.png?raw=true "Screenshot")



## Building

### Configuration
You will need to measure some sizes of your printed calibration chessboard and markers.

Grid size of calibartion rectangles (meters)
```c++
const float CALIBRATION_SQUARE_DIMENSION; 
```

Overall size of calibration chessboard pattern, this should be fine if you use the provided chessboard from the 
```c++
const cv::Size CHESSBOARD_DIMENSIONS;
```

Size of aruco markers (meters)
```c++
const float ARUCO_SQUARE_DIMENSION;
```

Camera resolution
```c++
const double CAMERA_RESOLUTION_X;
const double CAMERA_RESOLUTION_Y; 
```

### Compiling
This Program needs opencv and opencv_contrib > 2.8 to compile

```
cmake .
make
```

## Usage

### Generating markers

```
./aruco_tracking -g
```
Generated markers will be saved in the root directory.

To change the marker dictionary size, edit `PREDEFINED_DICTIONARY_NAME` to your desire.
For more detail please refer to the [OpenCV documentation](https://docs.opencv.org/3.4.0/d9/d6a/group__aruco.html#gac84398a9ed9dd01306592dd616c2c975)

### Calibrate your camera

The calibration file `cal` will be saved in the root directory.
There is already a file provided in the repository. You will get way better results if you calibrate your camera yorself.

Use `SPACE` to take a picture
Use `ENTER` to compute the distortion matrix

You will need to take at least 16 pictures before computing

```
./aruco_tracking -c
```
[Camera calibration documentation](https://docs.opencv.org/2.4/doc/tutorials/calib3d/camera_calibration/camera_calibration.html)

### Start position tracking

Some cameras/operating systems seem to behave different when starting the capture, it can take up to 30 seconds to get camera output shown. Be patient ;) If you have a solution, feel free to tell me!

```
./aruco_tracking -m
```
