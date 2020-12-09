#include <mutex>
#include <atomic>
#include <vector>
#include <thread>
#include <SENSARCore.h>

void SENSARCore::configure(int  targetWidth,
                           int  targetHeight,
                           int  manipWidth,
                           int  manipHeight,
                           bool convertManipToGray)
{
    _config.targetWidth        = targetWidth;
    _config.targetHeight       = targetHeight;
    _config.manipWidth         = manipWidth;
    _config.manipHeight        = manipHeight;
    _config.convertManipToGray = convertManipToGray;
}

SENSFramePtr SENSARCore::latestFrame()
{
    SENSFrameBasePtr frameBase;
    {
        std::lock_guard<std::mutex> lock(_frameMutex);
        frameBase = _frame;
    }
    
    SENSFramePtr latestFrame;
    if (frameBase)
        latestFrame = processNewFrame(frameBase->timePt, frameBase->imgBGR, frameBase->intrinsics);
    return latestFrame;
}

SENSFramePtr SENSARCore::processNewFrame(const SENSTimePt& timePt, cv::Mat& bgrImg, cv::Mat intrinsics)
{
    //todo: accessing config readonly should be no problem  here, as the config only changes when camera is stopped
    cv::Size inputSize = bgrImg.size();

    // Crop Video image to required aspect ratio
    int cropW = 0, cropH = 0;
    SENS::cropImage(bgrImg, (float)_config.targetWidth / (float)_config.targetHeight, cropW, cropH);

    cv::Mat manipImg;
    float   scale = 1.0f;

    manipImg = bgrImg;
    //problem: eingangsbild 16:9 -> targetImg 4:3 -> crop left and right -> manipImg 16:9 -> weiterer crop oben und unten -> FALSCH
    if (_config.manipWidth > 0 && _config.manipHeight > 0)
    {
        int cropW = 0, cropH = 0;
        SENS::cropImage(manipImg, (float)_config.manipWidth / (float)_config.manipHeight, cropW, cropH);
        scale = (float)_config.manipWidth / (float)manipImg.size().width;
        cv::resize(manipImg, manipImg, cv::Size(), scale, scale);
    }

    // Create grayscale
    if (_config.convertManipToGray)
    {
        cv::cvtColor(manipImg, manipImg, cv::COLOR_BGR2GRAY);
    }

    SENSFramePtr sensFrame = std::make_unique<SENSFrame>(timePt,
                                                         bgrImg,
                                                         manipImg,
                                                         false,
                                                         false,
                                                         1 / scale,
                                                         intrinsics);

    return sensFrame;
}