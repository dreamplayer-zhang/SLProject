#ifndef SENS_FRAME_H
#define SENS_FRAME_H

#include <opencv2/core.hpp>

//Camera frame obeject
struct SENSFrame
{
    SENSFrame(cv::Mat imgBGR,
              cv::Mat imgManip,
              int     captureWidth,
              int     captureHeight,
              int     cropW,
              int     cropH,
              bool    mirroredH,
              bool    mirroredV,
              float   scaleToManip,
              cv::Mat intrinsics)
      : imgBGR(imgBGR),
        imgManip(imgManip),
        captureWidth(captureWidth),
        captureHeight(captureHeight),
        cropW(cropW),
        cropH(cropH),
        mirroredH(mirroredH),
        mirroredV(mirroredV),
        scaleToManip(scaleToManip),
        intrinsics(intrinsics) //!transfer by reference
    {
    }

    //! cropped input image
    cv::Mat imgBGR;
    //! scaled and maybe gray manipulated image
    cv::Mat imgManip;

    const int  captureWidth;
    const int  captureHeight;
    const int  cropW;
    const int  cropH;
    const bool mirroredH;
    const bool mirroredV;
    //! scale from imgManip to imgBGR
    const float scaleToManip;
    //! new intrinsics matrix (if valid it can be used to define a new calibration)
    cv::Mat intrinsics;
};
typedef std::shared_ptr<SENSFrame> SENSFramePtr;

#endif //SENS_FRAME_H
