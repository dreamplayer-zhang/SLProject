#ifndef WAICALIBRATION
#define WAICALIBRATION

#define _USE_MATH_DEFINES
#include <math.h>
#include <opencv2/opencv.hpp>
using namespace std;

enum CalibrationState
{
    CalibrationState_None,
    CalibrationState_Guess,
    CalibrationState_Calibrated
};

class WAICalibration
{
public:
    WAICalibration();
    virtual bool loadFromFile(std::string path);
    virtual bool saveToFile(std::string path);
    virtual void reset();
    virtual void changeImageSize(int width, int height);

    std::string computerInfo() { return _computerInfo; }
    std::string filename() { return _filename; }
    float       calcCameraVerticalFOV();
    float       calcCameraHorizontalFOV();
    float       calcCameraVerticalFOV(cv::Mat& cameraMat);
    float       calcCameraHorizontalFOV(cv::Mat& cameraMat);
    float       aspectRatio() { return (float)_imgSize.width / (float)_imgSize.height; }

    void             computeMatrix(cv::Mat& mat, float fov);
    cv::Mat&         cameraMat() { return _cameraMat; }
    cv::Mat&         distortion() { return _distortion; }
    CalibrationState getState() { return _state; }
    std::string      getCalibrationPath() { return _calibrationPath; }
    std::string      stateStr();

    float fx() { return _cameraMat.cols == 3 && _cameraMat.rows == 3 ? (float)_cameraMat.at<double>(0, 0) : 0.0f; }
    float fy() { return _cameraMat.cols == 3 && _cameraMat.rows == 3 ? (float)_cameraMat.at<double>(1, 1) : 0.0f; }
    float cx() { return _cameraMat.cols == 3 && _cameraMat.rows == 3 ? (float)_cameraMat.at<double>(0, 2) : 0.0f; }
    float cy() { return _cameraMat.cols == 3 && _cameraMat.rows == 3 ? (float)_cameraMat.at<double>(1, 2) : 0.0f; }
    float k1() { return _distortion.rows >= 4 ? (float)_distortion.at<double>(0, 0) : 0.0f; }
    float k2() { return _distortion.rows >= 4 ? (float)_distortion.at<double>(1, 0) : 0.0f; }
    float p1() { return _distortion.rows >= 4 ? (float)_distortion.at<double>(2, 0) : 0.0f; }
    float p2() { return _distortion.rows >= 4 ? (float)_distortion.at<double>(3, 0) : 0.0f; }

protected:
    CalibrationState _state;
    cv::Mat          _cameraMat;
    cv::Mat          _distortion;
    cv::Size         _imgSize;
    float            _cameraFovDeg;
    std::string      _calibrationPath;
    int              _numCaptured;
    bool             _isMirroredH;
    bool             _isMirroredV;
    bool             _calibFixAspectRatio;
    bool             _calibFixPrincipalPoint;
    bool             _calibZeroTangentDist;
    float            _reprojectionError;
    float            _calibrationTime;
    int              _camSizeIndex;
    std::string      _computerInfo;
    std::string      _creationDate;
    std::string      _filename;
};
#endif
