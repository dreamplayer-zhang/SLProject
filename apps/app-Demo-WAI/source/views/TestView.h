#ifndef TEST_VIEW_H
#define TEST_VIEW_H

#include <views/View.h>
#include <AppWAIScene.h>
#include <SLSceneView.h>
#include <AppDemoWaiGui.h>
#include <SlamParams.h>
#include <AppDemoGuiSlamLoad.h>
#include <SENSVideoStream.h>
#include <CVCalibration.h>

class WAISlam;
class WAIEvent;
class SENSCamera;

class TestView : public View
{
public:
    TestView(sm::EventHandler& eventHandler,
             SLInputManager&   inputManager,
             SENSCamera*       camera,
             int               screenWidth,
             int               screenHeight,
             int               dotsPerInch,
             std::string       fontPath,
             std::string       configDir,
             std::string       vocabularyDir,
             std::string       calibDir,
             std::string       videoDir);
    ~TestView();

    bool update() override;
    //try to load slam params and start slam
    void start();
    void postStart();

protected:
    //try to load last slam (without clicking for convenience)
    void tryLoadLastSlam();
    //void setupGUI();
    void handleEvents();
    void loadWAISceneView(std::string location, std::string area);
    void saveMap(std::string location, std::string area, std::string marker);
    void saveVideo(std::string filename);
    void startOrbSlam(SlamParams slamParams);
    void transformMapNode(SLTransformSpace tSpace,
                          SLVec3f          rotation,
                          SLVec3f          translation,
                          float            scale);
    void downloadCalibrationFilesTo(std::string dir);
    void updateVideoTracking();
    void updateTrackingVisualization(const bool iKnowWhereIAm, cv::Mat& imgRGB);
    void setupDefaultErlebARDirTo(std::string dir);
    void startAsync();

    //video
    CVCalibration                    _calibration = {CVCameraType::FRONTFACING, ""};
    SENSCamera*                      _camera      = nullptr;
    cv::VideoWriter*                 _videoWriter = nullptr;
    std::unique_ptr<SENSVideoStream> _videoFileStream;
    bool                             _pauseVideo           = false;
    int                              _videoCursorMoveIndex = 0;
    bool                             _showUndistorted      = true;
    cv::Size2i                       _videoFrameSize;

    int     _lastFrameIdx;
    cv::Mat _undistortedLastFrame[2];
    bool    _doubleBufferedOutput;

    //slam
    WAISlam*   _mode = nullptr;
    SlamParams _currentSlamParams;

    FeatureExtractorFactory      _featureExtractorFactory;
    std::unique_ptr<KPextractor> _trackingExtractor;
    std::unique_ptr<KPextractor> _initializationExtractor;
    std::unique_ptr<KPextractor> _markerExtractor;

    std::queue<WAIEvent*> _eventQueue;
    //scene
    AppWAIScene _s;
    SLSceneView _sv;

    SLAssetManager _assets;

    std::string _configDir;
    std::string _vocabularyDir;
    std::string _calibDir;
    std::string _videoDir;

    std::thread _startThread;

    //gui (declaration down here because it depends on a lot of members in initializer list of constructor)
    AppDemoWaiGui _gui;
};

#endif //TEST_VIEW_H