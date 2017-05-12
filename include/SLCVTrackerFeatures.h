//#############################################################################
//  File:      SLCVTrackerFeatures.h
//  Author:    Pascal Zingg, Timon Tschanz
//  Date:      Spring 2017
//  Codestyle: https://github.com/cpvrlab/SLProject/wiki/Coding-Style-Guidelines
//  Copyright: Marcus Hudritsch, Michael Goettlicher
//             This softwareis provide under the GNU General Public License
//             Please visit: http://opensource.org/licenses/GPL-3.0
//#############################################################################

#ifndef SLCVTrackerFeatures_H
#define SLCVTrackerFeatures_H

/*
The OpenCV library version 3.1 with extra module must be present.
If the application captures the live video stream with OpenCV you have
to define in addition the constant SL_USES_CVCAPTURE.
All classes that use OpenCV begin with SLCV.
See also the class docs for SLCVCapture, SLCVCalibration and SLCVTracker
for a good top down information.
*/
#include <SLCV.h>
#include <SLCVTracker.h>
#include <SLNode.h>
#include <SLCVRaulMurOrb.h>

using namespace cv;


#define DEBUG_OUTPUT 0
#define FORCE_REPOSE 0
#define DISTINGUISH_FEATURE_DETECT_COMPUTE 0
#define TRACKING_MEASUREMENT 1

// Set stones Tracker as default reference image
#ifndef SL_TRACKER_IMAGE_NAME
    #define SL_TRACKER_IMAGE_NAME "stones"
#endif

#ifdef SL_SAVE_DEBUG_OUTPUT
    #if defined(SL_OS_LINUX) || defined(SL_OS_MACOS) || defined(SL_OS_MACIOS)
    #define SAVE_SNAPSHOTS_OUTPUT "/tmp/cv_tracking/"
    #elif defined(SL_OS_WINDOWS)
    #define SAVE_SNAPSHOTS_OUTPUT "cv_tracking/"
    #endif
#endif

// Settings for drawing things into current camera frame
#define DRAW_KEYPOINTS 1
#define DRAW_REPROJECTION 1
#define DRAW_REPOSE_INFO 1

// Feature detection and extraction
const int nFeatures = 2000;
const float minRatio = 0.7f;

// RANSAC parameters
const int iterations = 500;
const float reprojection_error = 2.0f;
const double confidence = 0.95;

// Repose patch size
const int reposeFrequency = 10;
const int initialPatchSize = 2;
const int maxPatchSize = 60;


class SLCVTrackerFeatures : public SLCVTracker
{
public:
        SLCVTrackerFeatures     (SLNode* node);
        ~SLCVTrackerFeatures    ();
        SLbool  track           (SLCVMat imageGray,
                                SLCVMat image,
                                SLCVCalibration* calib,
                                SLSceneView* sv);

private:
        static SLVMat4f         objectViewMats; //!< object view SLCVMatrices
        SLCVRaulMurOrb*         _detector;
        Ptr<ORB>                _descriptor;
        Ptr<DescriptorMatcher>  _matcher;

        SLMat4f                 _pose;
        SLCVCalibration         *_calib;
        int                     frameCount = 0, reposePatchSize;

        struct map {
            vector<Point3f>     model;
            SLCVMat             frameGray;
            SLCVMat             imgDrawing;
            SLCVVKeyPoint       keypoints;
            SLCVMat             descriptors;
            SLCVVKeyPoint       bboxModelKeypoints;
        } _map;

        struct FrameData {
            SLCVMat             image;
            SLCVMat             imageGray;

            vector<Point2f>     inlierPoints2D;
            vector<Point3f>     inlierPoints3D;

            SLCVVKeyPoint       keypoints;
            SLCVMat             descriptors;
            vector<DMatch>      matches;
            vector<DMatch>      inlierMatches;

            SLCVMat             rvec;
            SLCVMat             tvec;

            bool                foundPose;
            float               reprojectionError;
            bool                useExtrinsicGuess;
        };

        FrameData               _current, _prev;

        void                    initModel();
        void                    relocate();
        void                    tracking();
        void                    saveImageOutput();
        void                    updateSceneCam(SLSceneView* sv);
        void                    transferFrameData();
        SLCVVKeyPoint           getKeypoints();
        SLCVMat                 getDescriptors();
        void                    getKeypointsAndDescriptors();
        vector<DMatch>          getFeatureMatches();
        bool                    calculatePose();
        bool                    solvePnP();
        void                    optimizeMatches(float reprojectionError=0);
        bool                    trackWithOptFlow(Mat rvec, Mat tvec);
};
//-----------------------------------------------------------------------------
#endif // SLCVTrackerFeatures_H
