#ifndef WAI_SENSOR_CAMERA_H
#define WAI_SENSOR_CAMERA_H

#include <WAIHelper.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <WAISensor.h>

namespace WAI
{
struct CameraFrame
{
    int   width;
    int   height;
    int   pitch;
    int   bytesPerPixel;
    void* memory;
};

struct CameraCalibration
{
    public:
    CameraCalibration(float fx, float fy, float cx, float cy, float k1, float k2, float p1, float p2, float k3, cv::Size size)
      : fx(fx), fy(fy), cx(cx), cy(cy), k1(k1), k2(k2), p1(p1), p2(p2), k3(k3), imgSize(size)
    {
        aspectRatio = (float)imgSize.width / (float)imgSize.height;
    }
    CameraCalibration() {}

    float    fx, fy, cx, cy, k1, k2, p1, p2, k3;
    cv::Size imgSize;
    float    scaleFactor = 1.0f;
    float    aspectRatio;
    //!Resize calibration to meet targetSize. Checks that aspect ratio is not changed.
    //!Returns false if aspect ratio would be changed and
    bool resize(cv::Size targetSize)
    {
        //check that aspect ratio does not change
        float targetAspectRatio = (float)targetSize.width / (float)targetSize.height;
        if (std::abs(aspectRatio - targetAspectRatio) > 0.00001)
            return false;

        if (targetSize.width != imgSize.width)
        {
            float factor = (float)targetSize.width / (float)imgSize.width;
            scale(factor);
        }

        return true;
    }

    private:
    void scale(float factor)
    {
        scaleFactor    = factor;
        imgSize.width  = std::roundf((float)imgSize.width * factor);
        imgSize.height = std::roundf((float)imgSize.height * factor);
        fx *= factor;
        fy *= factor;
        cx *= factor;
        cy *= factor;
    }
};

struct CameraData
{
    cv::Mat* imageGray;
    cv::Mat* imageRGB;
};

class SensorCamera : public Sensor
{
    public:
    SensorCamera(CameraCalibration* cameraCalibration);
    void              update(void* cameraData);
    cv::Mat           getImageGray() { return _imageGray; }
    cv::Mat           getImageRGB() { return _imageRGB; }
    CameraCalibration getCameraCalibration() { return _cameraCalibration; }
    cv::Mat           getCameraMatrix() { return _cameraMatrix; }
    cv::Mat           getDistortionMatrix() { return _distortionMatrix; }
    void              subscribeToUpdate(Mode* mode);

    private:
    cv::Mat           _imageGray;
    cv::Mat           _imageRGB;
    cv::Mat           _cameraMatrix;
    cv::Mat           _distortionMatrix;
    CameraCalibration _cameraCalibration;
    Mode*             _mode = 0;
};
}

#endif
