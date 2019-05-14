
# aruco_positioning

This Program tracks aruco marker positions in a 3D space with a single webcam.

![Screenshot](screenshot.png?raw=true "Screenshot")


## Building

This Program needs opencv and opencv_contrib > 2.8 to compile

```
clone the repository
cmake .
make
```

## Usage

### Generating markers
```
./aruco_tracking -g
```

### Calibrate your camera
```
./aruco_tracking -c
```

### Start position tracking
```
./aruco_tracking -m
```
