#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>

#include <sstream>
#include <iostream>
#include <fstream>

const float CALIBRATION_SQUARE_DIMENSION = 0.0251f; // Grid size of calibartion rectangles (meters)
const cv::Size CHESSBOARD_DIMENSIONS = cv::Size(6, 9); // Overall size of calibration chessboard pattern
const float ARUCO_SQUARE_DIMENSION = 0.0251f; // Size of aruco markers (meters)
const double CAMERA_RESOLUTION_X = 1920; // Camera Resolution X
const double CAMERA_RESOLUTION_Y = 1080; // Camera Resolution Y

// Creates Chessboard data structure for calibration
void createKnownBoardPositions(cv::Size boardSize, float squareEdgeLength, std::vector<cv::Point3f>& corners) 
{
    for (int i = 0; i < boardSize.height; i++) 
    {
        for (int j = 0; j < boardSize.width; j++) 
        {
            corners.push_back(cv::Point3f(j * squareEdgeLength, i * squareEdgeLength, 0.0f));
        }
    }
}

// Camera Calibration helper function
void getChessboardCorners(std::vector<cv::Mat> images, std::vector<std::vector<cv::Point2f>>& allFoundCorners, bool showResults = false)
{
    for (std::vector<cv::Mat>::iterator iter = images.begin(); iter != images.end(); iter++) {
        std::vector<cv::Point2f> pointBuf;
        bool found = findChessboardCorners(*iter, CHESSBOARD_DIMENSIONS, pointBuf, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);
        if (found) 
        {
            allFoundCorners.push_back(pointBuf);
        }
        if (showResults)
        {
            drawChessboardCorners(*iter, CHESSBOARD_DIMENSIONS, pointBuf, found);
            imshow("Searching for Corners...", *iter);
        }
    }
}

void cameraCalibration(std::vector<cv::Mat> calibrationImages, cv::Size boardSize, float squareEdgeLength, cv::Mat& cameraMatrix, cv::Mat& distortionCoefficients) {
    std::vector<std::vector<cv::Point2f>> checkerboardImageSpacePoints;
    getChessboardCorners(calibrationImages, checkerboardImageSpacePoints, false);

    std::vector<std::vector<cv::Point3f>> worldSpaceCornerPoints(1);
    createKnownBoardPositions(boardSize, squareEdgeLength, worldSpaceCornerPoints[0]);
    worldSpaceCornerPoints.resize(checkerboardImageSpacePoints.size(), worldSpaceCornerPoints[0]);

    std::vector<cv::Mat> rVectors, tVectors;
    distortionCoefficients = cv::Mat::zeros(8, 1, CV_64F);

    cv::calibrateCamera(worldSpaceCornerPoints, checkerboardImageSpacePoints, boardSize, cameraMatrix, distortionCoefficients, rVectors, tVectors);
}

// Saves camera calibration in a file in root dir
bool saveCameraCalibration(std::string name, cv::Mat cameraMatrix, cv::Mat distortionCoefficients) {
    std::ofstream outStream(name);
    if (outStream)
    {
        uint16_t rows = cameraMatrix.rows;
        uint16_t columns = cameraMatrix.cols;


        outStream << rows << std::endl;
        outStream << columns << std::endl;

        std::cout << "Camera matrix rows: " << rows << std::endl;
        std::cout << "Camera matrix rows: " << columns << std::endl;

        for (int r = 0; r < rows; r++) {
            for (int c = 0; c < columns; c++) {
                double value = cameraMatrix.at<double>(r, c);
                std::cout << "..";
                outStream << value << std::endl;
            }
        }
        std::cout << "\n";

        rows = distortionCoefficients.rows;
        columns = distortionCoefficients.cols;

        outStream << rows << std::endl;
        outStream << columns << std::endl;

        std::cout << "Distortion coefficient rows: " << rows << std::endl;
        std::cout << "Distortion coefficient columns: " << columns << std::endl;

        for (int r = 0; r < rows; r++) {
            for (int c = 0; c < columns; c++) {
                double value = distortionCoefficients.at<double>(r, c);
                std::cout << "..";
                outStream << value << std::endl;
            }
        }
        std::cout << "\n";

        outStream.close();
        return true;
    }
    return false;
}

// Calibrates the camera
int cameraCalibrationProcess(cv::Mat& cameraMatrix, cv::Mat& distortionCoefficients)
{
    cv::Mat frame;
    cv::Mat drawToFrame;

    std::vector<cv::Mat> savedImages;

    std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
    cv::VideoCapture vid(0);
    vid.set(cv::CAP_PROP_FRAME_WIDTH,CAMERA_RESOLUTION_X);
    vid.set(cv::CAP_PROP_FRAME_HEIGHT,CAMERA_RESOLUTION_Y);

    if (!vid.isOpened())
    {
        return -1;
    }
    int fps = 20;
    cv::namedWindow("Camera Calibration", cv::WINDOW_AUTOSIZE);

    while (true)
    {
        if (!vid.read(frame))
        {
            break;
        }

        std::vector<cv::Vec2f> foundPoints;
        bool found = false;

        found = findChessboardCorners(frame, CHESSBOARD_DIMENSIONS, foundPoints, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);
        frame.copyTo(drawToFrame);
        drawChessboardCorners(drawToFrame, CHESSBOARD_DIMENSIONS, foundPoints, found);

        if (found)
        {
            imshow("Camera Calibration", drawToFrame);
        }
        else
        {
            imshow("Camera Calibration", frame);
        }

        char character = cv::waitKey(1000 / fps);

        switch (character)
        {
        case ' ': //SPACE, save
            if (found)
            {
                std::cout << "Found valid Pattern" << std::endl;
                cv::Mat temp;
                frame.copyTo(temp);
                savedImages.push_back(temp);
            }
            break;

        case 13: //ENTER, start				 
            if (savedImages.size() > 15)
            {
                std::cout << "Computing radial distortion coefficients. This may take a few a few minutes" << std::endl;
                cameraCalibration(savedImages, CHESSBOARD_DIMENSIONS, CALIBRATION_SQUARE_DIMENSION, cameraMatrix, distortionCoefficients);
                std::cout << "Saving calibration data..." << std::endl;
                saveCameraCalibration("cal", cameraMatrix, distortionCoefficients);
                std::cout << "Done!" << std::endl;
            }
            break;

        case 27: //ESC, exit
            return 0;
        }

    }

    
    return 0;
}



// Creates aruco marker images in root dir
void createArucoMarker(cv::aruco::PREDEFINED_DICTIONARY_NAME dictName)
{
    cv::Mat outMarker;
    cv::Ptr<cv::aruco::Dictionary> markerDict = cv::aruco::getPredefinedDictionary(dictName);

    for (int i = 0; i < 50; i++) {
        cv::aruco::drawMarker(markerDict, i, 500, outMarker, 1);
        std::ostringstream convert;
        std::string imgName = "Marker_";
        convert << imgName << i << ".png";
        imwrite(convert.str(), outMarker);
    }
}

// Checks if a matrix is a valid rotation matrix.
bool isRotationMatrix(cv::Mat &R)
{
    cv::Mat Rt;
    transpose(R, Rt);
    cv::Mat shouldBeIdentity = Rt * R;
    cv::Mat I = cv::Mat::eye(3,3, shouldBeIdentity.type());
     
    return  norm(I, shouldBeIdentity) < 1e-6;
     
}
 
// Calculates rotation matrix to euler angles
cv::Vec3f rotationMatrixToEulerAngles(cv::Mat &R)
{
    assert(isRotationMatrix(R));
     
    float sy = sqrt(R.at<double>(0,0) * R.at<double>(0,0) +  R.at<double>(1,0) * R.at<double>(1,0) );
 
    bool singular = sy < 1e-6; // If
 
    float x, y, z;
    if (!singular)
    {
        x = atan2(R.at<double>(2,1) , R.at<double>(2,2));
        y = atan2(-R.at<double>(2,0), sy);
        z = atan2(R.at<double>(1,0), R.at<double>(0,0));
    }
    else
    {
        x = atan2(-R.at<double>(1,2), R.at<double>(1,1));
        y = atan2(-R.at<double>(2,0), sy);
        z = 0;
    }
    return cv::Vec3f(x, y, z);
}

// Tracks markers
int startWebcamMonitoring(const cv::Mat& cameraMatrix, const cv::Mat& distortionCoefficients, float arucoSquareDimensions)
{
    cv::Mat frame;
    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;	
    cv::aruco::DetectorParameters parameters;
    cv::Ptr<cv::aruco::Dictionary> markerDictionary = cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_4X4_50);

    cv::VideoCapture vid(0 + cv::CAP_ANY);
    vid.set(cv::CAP_PROP_FRAME_WIDTH,CAMERA_RESOLUTION_X);
    vid.set(cv::CAP_PROP_FRAME_HEIGHT,CAMERA_RESOLUTION_Y);
    
    if (!vid.isOpened()) 
    {
        std::cout << "Cannont open video stream!" << std::endl;
        return -1;
    }

    cv::namedWindow("Webcam", cv::WINDOW_AUTOSIZE);
    std::vector<cv::Vec3d> rotationVectors, translationVectors;

    while (true) 
    {
        if (!vid.read(frame))
        {
            break;
        }

        cv::aruco::detectMarkers(frame, markerDictionary, markerCorners, markerIds);
        cv::aruco::estimatePoseSingleMarkers(markerCorners, arucoSquareDimensions, cameraMatrix, distortionCoefficients, rotationVectors, translationVectors);

        if (markerIds.size() > 0) {
            cv::aruco::drawDetectedMarkers(frame, markerCorners, markerIds);
        }

        std::cout << "\033[2J\033[1;1H";
        for (int i = 0; i < markerIds.size(); i++)
        {
            cv::aruco::drawAxis(frame, cameraMatrix, distortionCoefficients, rotationVectors[i], translationVectors[i], 0.1f);
            
            // Get euler angles of Markers
            cv::Mat matTmp;
            cv::Rodrigues(rotationVectors[i], matTmp);
            cv::Vec3f eulerAngles = rotationMatrixToEulerAngles(matTmp);

            // Console output of marker positions
            std::cout << " ------------------------------------------------------- " << std::endl;
            std::cout << "Marker: " << i << std::endl;
            std::cout << "X: " << translationVectors[i][0] << "  Y: " << translationVectors[i][0] << "  Z: " << translationVectors[i][0] << std::endl;
            std::cout << "Yaw: " << eulerAngles[0] << "  Pitch: " << eulerAngles[1] << "  Roll: " << eulerAngles[2] << std::endl;
            std::cout << std::endl;

            
            rectangle(frame, markerCorners[i].at(0), cv::Point(markerCorners[i].at(0).x + 25.0, markerCorners[i].at(0).y + 25.0), cv::Scalar(255, 0, 0), 2);
        }
        

        imshow("Webcam", frame);
        if (cv::waitKey(30) >= 0)
        {
            break;
        }

    }
    return 1;
}

// Loads camera calibration
bool loadCameraCalibration(std::string name, cv::Mat& cameraMatrix, cv::Mat& distortionCoefficients) 
{
    std::ifstream inStream(name);
    if (inStream)
    {
        uint16_t rows;
        uint16_t columns;

        inStream >> rows;
        inStream >> columns;

        cameraMatrix = cv::Mat(cv::Size(columns, rows), CV_64F);

        for (int r = 0; r < rows; r++)
        {
            for (int c = 0; c < columns; c++)
            {
                double temp = 0.0f;
                inStream >> temp;
                cameraMatrix.at<double>(r, c) = temp;
                std::cout << cameraMatrix.at<double>(r, c) << std::endl;
            }
        }

        // Distance coefficients
        inStream >> rows;
        inStream >> columns;

        distortionCoefficients = cv::Mat::zeros(rows, columns, CV_64F);
        for (int r = 0; r < rows; r++)
        {
            for (int c = 0; c < columns; c++)
            {
                double temp = 0.0f;
                inStream >> temp;
                distortionCoefficients.at<double>(r, c) = temp;
                std::cout << distortionCoefficients.at<double>(r, c) << std::endl;
            }
        }
        
        inStream.close();
        return true;
    }
    return false;
}

int main(int argc, char* argv[])
{
    cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat distortionCoefficients;

    if (argc != 2 ){
        std::cout << "Error: No argument given!" << std::endl;
        std::cout << "-c >> Camera calibration mode" << std::endl;
        std::cout << "-g >> Generate new aruco dictionary" << std::endl;
        std::cout << "-m >> Marker tracking mode" << std::endl;
        return -1;
    }
    if (strcmp(argv[1], "-c") == 0){
        bool res = cameraCalibrationProcess(cameraMatrix, distortionCoefficients);
        if (res == false){
            std::cout << "Calibration failed! Exiting..." << std::endl;
            return -1;
        } 
        else{
            std::cout << "Calibration sucessful! Exiting..." << std::endl;
            return 0;
        }
    }
    else if(strcmp(argv[1], "-g") == 0){
        createArucoMarker(cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_4X4_50);
        std::cout << "Created new Markers in root directory. Exiting..." << std::endl;
        return 0;
    }
    else if(strcmp(argv[1], "-m") == 0) {
        loadCameraCalibration("cal", cameraMatrix, distortionCoefficients);
        startWebcamMonitoring(cameraMatrix, distortionCoefficients, ARUCO_SQUARE_DIMENSION);
    }
    else{
        std::cout << "Unknown argument: " << argv[1] << " Exiting..." << std::endl;
        return -1;
    }
}
