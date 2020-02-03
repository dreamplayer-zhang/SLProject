#include <WAIApp.h>

#include <SLApplication.h>
#include <SLInterface.h>
#include <SLKeyframeCamera.h>
#include <Utils.h>
#include <AverageTiming.h>

#include <WAIModeOrbSlam2.h>
#include <WAIMapStorage.h>
#include <WAICalibration.h>
#include <AppWAIScene.h>

#include <AppDemoGuiInfosDialog.h>
#include <AppDemoGuiAbout.h>
#include <AppDemoGuiInfosFrameworks.h>
#include <AppDemoGuiInfosMapNodeTransform.h>
#include <AppDemoGuiInfosScene.h>
#include <AppDemoGuiInfosSensors.h>
#include <AppDemoGuiInfosTracking.h>
#include <AppDemoGuiProperties.h>
#include <AppDemoGuiSceneGraph.h>
#include <AppDemoGuiStatsDebugTiming.h>
#include <AppDemoGuiStatsTiming.h>
#include <AppDemoGuiStatsVideo.h>
#include <AppDemoGuiTrackedMapping.h>
#include <AppDemoGuiTransform.h>
#include <AppDemoGuiUIPrefs.h>
#include <AppDemoGuiVideoControls.h>
#include <AppDemoGuiVideoStorage.h>
#include <AppDemoGuiSlamLoad.h>
#include <AppDemoGuiTestOpen.h>
#include <AppDemoGuiTestWrite.h>
#include <AppDemoGuiError.h>

#include <AppDirectories.h>
#include <AppWAISlamParamHelper.h>
#include <FtpUtils.h>
#include <GUIPreferences.h>

#include <GLSLextractor.h>
#include <SLSceneView.h>
#include <SLPoints.h>
#include <SLQuat4.h>
#include <SLPolyline.h>
#include <opencv2/imgproc/imgproc.hpp>

//move
#include <SLAssimpImporter.h>

//basic information
//-the sceen has a fixed size on android and also a fixed aspect ratio on desktop to simulate android behaviour on desktop
//-the viewport gets adjusted according to the target video aspect ratio (for live video WAIApp::liveVideoTargetWidth and WAIApp::_liveVideoTargetHeight are used and for video file the video frame size is used)
//-the live video gets cropped according to the viewport aspect ratio on android and according to WAIApp::videoFrameWdivH on desktop (which is also used to define the viewport aspect ratio)
//-the calibration gets adjusted according to the video (live and file)
//-the live video gets cropped to the aspect ratio that is defined by the transferred values in load(..) and assigned to liveVideoTargetWidth and liveVideoTargetHeight

//-----------------------------------------------------------------------------
WAIApp::WAIApp()
  : SLInputEventInterface(SLApplication::inputManager) //todo: local input manager
{
}
//-----------------------------------------------------------------------------
WAIApp::~WAIApp()
{
    //todo: destructor is not called on android (at the right position)
    if (_videoWriter)
        delete _videoWriter;
    if (_videoWriterInfo)
        delete _videoWriterInfo;
}

//-----------------------------------------------------------------------------
int WAIApp::load(SENSCamera* camera, int liveVideoTargetW, int liveVideoTargetH, int scrWidth, int scrHeight, float scr2fbX, float scr2fbY, int dpi, AppDirectories directories)
{
    _camera                = camera;
    _liveVideoTargetWidth  = liveVideoTargetW;
    _liveVideoTargetHeight = liveVideoTargetH;

    _dirs = directories;

    SLApplication::devRot.isUsed(true);
    SLApplication::devLoc.isUsed(true);

    videoDir = _dirs.writableDir + "erleb-AR/locations/";
    calibDir = _dirs.writableDir + "calibrations/";
    mapDir   = _dirs.writableDir + "maps/";
    vocDir   = _dirs.writableDir + "calibrations/";

    _waiScene        = std::make_unique<AppWAIScene>();
    _videoWriter     = new cv::VideoWriter();
    _videoWriterInfo = new cv::VideoWriter();
    _loaded          = true;

    //init scene as soon as possible to allow visualization of error msgs
    int svIndex = initSLProject(scrWidth, scrHeight, scr2fbX, scr2fbY, dpi);

    setupDefaultErlebARDirTo(_dirs.writableDir);
    //todo: only do this, when the app is installed (on android and ios, maybe by a function call when permissions are given)
    downloadCalibratinFilesTo(calibDir);

    return svIndex;
}

SENSFramePtr WAIApp::updateVideoOrCamera()
{
    SENSFramePtr frame;

    if (_videoFileStream)
    {
        while (_videoCursorMoveIndex < 0)
        {
            frame = _videoFileStream->grabPreviousFrame();
            if (frame)
                updateTracking(frame);

            _videoCursorMoveIndex++;
        }

        while (_videoCursorMoveIndex > 0)
        {
            frame = _videoFileStream->grabNextFrame();
            if (frame)
                updateTracking(frame);

            _videoCursorMoveIndex--;
        }

        if (!_pauseVideo)
        {
            frame = _videoFileStream->grabNextFrame();
        }
    }

    else if (_camera)
    {
        frame = _camera->getLatestFrame();
    }
    else
    {
        WAI_WARN("WAIApp::updateVideoOrCamera: No active video stream or camera available!");
    }

    return frame;
}

bool WAIApp::update()
{
    try
    {
        handleEvents();

        if (_mode && _loaded)
        {
            bool iKnowWhereIAm = false;
            //get new frame: in case of video this may already call updateTracking several times
            SENSFramePtr frame = updateVideoOrCamera();

            if (frame)
            {
                iKnowWhereIAm = updateTracking(frame);

                //update tracking infos visualization
                updateTrackingVisualization(iKnowWhereIAm, frame->imgRGB);
            }

            if (iKnowWhereIAm)
            {
                _lastKnowPoseQuaternion = SLApplication::devRot.quaternion();
                _IMUQuaternion          = SLQuat4f(0, 0, 0, 1);

                // TODO(dgj1): maybe make this API cleaner
                cv::Mat pose = cv::Mat(4, 4, CV_32F);
                pose = _mode->getPose();

                // update camera node position
                cv::Mat Rwc(3, 3, CV_32F);
                cv::Mat twc(3, 1, CV_32F);

                Rwc = (pose.rowRange(0, 3).colRange(0, 3)).t();
                twc = -Rwc * pose.rowRange(0, 3).col(3);

                cv::Mat PoseInv = cv::Mat::eye(4, 4, CV_32F);

                Rwc.copyTo(PoseInv.colRange(0, 3).rowRange(0, 3));
                twc.copyTo(PoseInv.rowRange(0, 3).col(3));
                SLMat4f om;

                om.setMatrix(PoseInv.at<float>(0, 0),
                             -PoseInv.at<float>(0, 1),
                             -PoseInv.at<float>(0, 2),
                             PoseInv.at<float>(0, 3),
                             PoseInv.at<float>(1, 0),
                             -PoseInv.at<float>(1, 1),
                             -PoseInv.at<float>(1, 2),
                             PoseInv.at<float>(1, 3),
                             PoseInv.at<float>(2, 0),
                             -PoseInv.at<float>(2, 1),
                             -PoseInv.at<float>(2, 2),
                             PoseInv.at<float>(2, 3),
                             PoseInv.at<float>(3, 0),
                             -PoseInv.at<float>(3, 1),
                             -PoseInv.at<float>(3, 2),
                             PoseInv.at<float>(3, 3));

                _waiScene->cameraNode->om(om);
            }
            else
            {
                SLQuat4f q1 = _lastKnowPoseQuaternion;
                SLQuat4f q2 = SLApplication::devRot.quaternion();
                q1.invert();
                SLQuat4f q              = q1 * q2;
                _IMUQuaternion          = SLQuat4f(q.y(), -q.x(), -q.z(), -q.w());
                SLMat4f imuRot          = _IMUQuaternion.toMat4();
                _lastKnowPoseQuaternion = q2;

                SLMat4f cameraMat = _waiScene->cameraNode->om();
                _waiScene->cameraNode->om(cameraMat * imuRot);
            }

            //AVERAGE_TIMING_STOP("WAIAppUpdate");
        }
    }
    catch (std::exception& e)
    {
        Utils::log("WAIApp", "Std exception catched in update() %s", e.what());
    }
    catch (...)
    {
        Utils::log("WAIApp", "Unknown exception catched in update()");
    }

    //update scene (before it was slUpdateScene)
    SLApplication::scene->onUpdate();
    return updateSceneViews();
}

//-----------------------------------------------------------------------------
void WAIApp::close()
{
    // Deletes all remaining sceneviews the current scene instance
    SLApplication::deleteAppAndScene();
}

//-----------------------------------------------------------------------------
/*
videoFile: path to a video or empty if live video should be used
calibrationFile: path to a calibration or empty if calibration should be searched automatically
mapFile: path to a map or empty if no map should be used
*/
void WAIApp::startOrbSlam(SlamParams* slamParams)
{
    _lastFrameIdx         = 0;
    _doubleBufferedOutput = false;
    if (_videoFileStream)
        _videoFileStream.release();

    std::string               videoFile       = "";
    std::string               calibrationFile = "";
    std::string               mapFile         = "";
    std::string               vocFile         = "";
    std::string               markerFile      = "";
    WAI::ModeOrbSlam2::Params params;

    if (slamParams)
    {
        videoFile       = slamParams->videoFile;
        calibrationFile = slamParams->calibrationFile;
        mapFile         = slamParams->mapFile;
        vocFile         = slamParams->vocabularyFile;
        markerFile      = slamParams->markerFile;
        params          = slamParams->params;

    }

    bool useVideoFile             = !videoFile.empty();
    bool detectCalibAutomatically = calibrationFile.empty();
    bool useMapFile               = !mapFile.empty();

    // reset stuff
    if (_mode)
    {
        _mode->requestStateIdle();
        while (!_mode->hasStateIdle())
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        delete _mode;
        _mode = nullptr;
    }

    // Check that files exist
    if (useVideoFile && !Utils::fileExists(videoFile))
    {
        showErrorMsg("Video file " + videoFile + " does not exist.");
        return;
    }

    // determine correct calibration file
    std::string calibrationFileName;
    if (detectCalibAutomatically)
    {
        std::string computerInfo;

        if (useVideoFile)
        {
            SlamVideoInfos slamVideoInfos;
            std::string    videoFileName = Utils::getFileNameWOExt(videoFile);

            if (!extractSlamVideoInfosFromFileName(videoFileName, &slamVideoInfos))
            {
                showErrorMsg("Could not extract computer infos from video filename.");
                return;
            }

            computerInfo = slamVideoInfos.deviceString;
        }
        else
        {
            computerInfo = SLApplication::getComputerInfos();
        }

        calibrationFileName = "camCalib_" + computerInfo + "_main.xml";
        calibrationFile     = calibDir + calibrationFileName;
    }
    else
    {
        calibrationFileName = Utils::getFileName(calibrationFile);
    }

    if (!Utils::fileExists(calibrationFile))
    {
        showErrorMsg("Calibration file " + calibrationFile + " does not exist.");
        return;
    }

    if (!checkCalibration(calibDir, calibrationFileName))
    {
        showErrorMsg("Calibration file " + calibrationFile + " is incorrect.");
        return;
    }

    if (vocFile.empty())
    {
        vocFile = vocDir + "ORBvoc.bin";
    }

    if (!Utils::fileExists(vocFile))
    {
        showErrorMsg("Vocabulary file does not exist: " + vocFile);
        return;
    }

    if (useMapFile && !Utils::fileExists(mapFile))
    {
        showErrorMsg("Map file " + mapFile + " does not exist.");
        return;
    }

    // 1. Initialize video stream
    if (useVideoFile)
    {
        _videoFileStream = std::make_unique<SENSVideoStream>(videoFile, true, false, false);
        _videoFrameSize  = _videoFileStream->getFrameSize();
    }
    else
    {
        _videoFrameSize = cv::Size2i(_liveVideoTargetWidth, _liveVideoTargetHeight);
    }
    _videoFrameWdivH = (float)_videoFrameSize.width / (float)_videoFrameSize.height;

    // 2. Load Calibration
    if (!_calibration.load(calibDir, Utils::getFileName(calibrationFile)))
    {
        showErrorMsg("Error when loading calibration from file: " +
                     calibrationFile);
        return;
    }

    if (_calibration.imageSize() != _videoFrameSize)
    {
        _calibration.adaptForNewResolution(_videoFrameSize);
    }

    // 3. Adjust FOV of camera node according to new calibration (fov is used in projection->prespective _mode)
    _waiScene->cameraNode->fov(_calibration.cameraFovVDeg());
    // Set camera intrinsics for scene camera frustum. (used in projection->intrinsics mode)
    cv::Mat scMat = _calibration.cameraMatUndistorted();
    std::cout << "scMat: " << scMat << std::endl;
    _waiScene->cameraNode->intrinsics(scMat.at<double>(0, 0),
                                      scMat.at<double>(1, 1),
                                      scMat.at<double>(0, 2),
                                      scMat.at<double>(1, 2));

    //enable projection -> intrinsics mode
    _waiScene->cameraNode->projection(P_monoIntrinsic);

    // 4. Create new mode ORBSlam
    if (!markerFile.empty())
    {
        params.cullRedundantPerc = 0.99f;
    }

    _trackingExtractor       = _featureExtractorFactory.make(slamParams->extractorIds.trackingExtractorId, _videoFrameSize);
    /*
    _initializationExtractor = _featureExtractorFactory.make(slamParams->extractorIds.initializationExtractorId, _videoFrameSize);
    _markerExtractor         = _featureExtractorFactory.make(slamParams->extractorIds.markerExtractorId, _videoFrameSize);
    */
    _doubleBufferedOutput    = _trackingExtractor->doubleBufferedOutput();

    _mode = new WAISlam(_calibration.cameraMat(),
                        _calibration.distortion(),
                        vocFile,
                        _trackingExtractor.get(),
                        params.onlyTracking,
                        params.serial,
                        params.retainImg);

    // 5. Load map data
    if (useMapFile)
    {
        _mode->requestStateIdle();
        while (!_mode->hasStateIdle())
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        _mode->reset();

        bool mapLoadingSuccess = WAIMapStorage::loadMap(_mode->getMap(),
                                                        _mode->getKfDB(),
                                                        _waiScene->mapNode,
                                                        mapFile,
                                                        _mode->retainImage(),
                                                        params.fixOldKfs);

        if (!mapLoadingSuccess)
        {
            delete _mode;
            _mode = nullptr;

            showErrorMsg("Could not load map from file " + mapFile);
            return;
        }

        SlamMapInfos slamMapInfos = {};
        extractSlamMapInfosFromFileName(mapFile, &slamMapInfos);

        _mode->resume();
        _mode->setState(TrackingState_TrackingLost);
    }

    // 6. save current params
    _currentSlamParams.calibrationFile  = calibrationFile;
    _currentSlamParams.mapFile          = mapFile;
    _currentSlamParams.params.retainImg = params.retainImg;
    _currentSlamParams.videoFile        = videoFile;
    _currentSlamParams.vocabularyFile   = vocFile;

    _sv->setViewportFromRatio(SLVec2i(_videoFrameSize.width, _videoFrameSize.height), SLViewportAlign::VA_center, true);
    //_resizeWindow = true;
    _undistortedLastFrame[0] = cv::Mat(_videoFrameSize.height, _videoFrameSize.width, CV_8UC3);
    _undistortedLastFrame[1] = cv::Mat(_videoFrameSize.height, _videoFrameSize.width, CV_8UC3);
}

//-----------------------------------------------------------------------------
void WAIApp::showErrorMsg(std::string msg)
{
    assert(_errorDial && "errorDial is not initialized");

    _errorDial->setErrorMsg(msg);
    _gui->uiPrefs->showError = true;
}

//-----------------------------------------------------------------------------
std::string WAIApp::name()
{
    return SLApplication::name;
}

//-----------------------------------------------------------------------------
void WAIApp::setDeviceParameter(const std::string& parameter,
                                std::string        value)
{
    SLApplication::deviceParameter[parameter] = std::move(value);
}
//-----------------------------------------------------------------------------
void WAIApp::setRotationQuat(float quatX,
                             float quatY,
                             float quatZ,
                             float quatW)
{
    //todo: replace
    SLApplication::devRot.onRotationQUAT(quatX, quatY, quatZ, quatW);
}

//-----------------------------------------------------------------------------
bool WAIApp::usesRotationSensor()
{
    //todo: replace
    return SLApplication::devRot.isUsed();
}

//-----------------------------------------------------------------------------
void WAIApp::setLocationLLA(float latitudeDEG, float longitudeDEG, float altitudeM, float accuracyM)
{
    //todo: replace
    SLApplication::devLoc.onLocationLLA(latitudeDEG,
                                        longitudeDEG,
                                        altitudeM,
                                        accuracyM);
}
//-----------------------------------------------------------------------------
bool WAIApp::usesLocationSensor()
{
    //todo: replace
    return SLApplication::devLoc.isUsed();
}

//-----------------------------------------------------------------------------
void WAIApp::initExternalDataDirectory(std::string path)
{
    if (Utils::dirExists(path))
    {
        Utils::log("External directory: %s\n", path.c_str());
        SLApplication::externalPath = path;
    }
    else
    {
        Utils::log("ERROR: external directory does not exists: %s\n", path.c_str());
    }
}

//-----------------------------------------------------------------------------
bool WAIApp::updateTracking(SENSFramePtr frame)
{
    bool iKnowWhereIAm = false;

    if (_videoWriter->isOpened())
    {
        _videoWriter->write(frame->imgRGB);
    }

    iKnowWhereIAm = _mode->update(frame->imgGray);

    /*
    if (_videoWriterInfo->isOpened())
    {
        _videoWriterInfo->write(frame->imgRGB);
    }*/

    if (_gpsDataStream.is_open())
    {
        if (SLApplication::devLoc.isUsed())
        {
            SLVec3d v = SLApplication::devLoc.locLLA();
            _gpsDataStream << SLApplication::devLoc.locAccuracyM();
            _gpsDataStream << std::to_string(v.x) + " " + std::to_string(v.y) + " " + std::to_string(v.z);
            _gpsDataStream << std::to_string(SLApplication::devRot.yawRAD());
            _gpsDataStream << std::to_string(SLApplication::devRot.pitchRAD());
            _gpsDataStream << std::to_string(SLApplication::devRot.rollRAD());
        }
    }

    return iKnowWhereIAm;
}
//-----------------------------------------------------------------------------
bool WAIApp::initSLProject(int scrWidth, int scrHeight, float scr2fbX, float scr2fbY, int dpi)
{
    assert(SLApplication::scene == nullptr && "SLScene is already created!");

    // Default paths for all loaded resources
    SLGLProgram::defaultPath      = _dirs.slDataRoot + "/shaders/";
    SLGLTexture::defaultPath      = _dirs.slDataRoot + "/images/textures/";
    SLGLTexture::defaultPathFonts = _dirs.slDataRoot + "/images/fonts/";
    SLAssimpImporter::defaultPath = _dirs.slDataRoot + "/models/";
    SLApplication::configPath     = _dirs.writableDir;

    SLApplication::name  = "WAI Demo App";
    SLApplication::scene = new SLScene("WAI Demo App", nullptr);

    int screenWidth  = (int)(scrWidth * scr2fbX);
    int screenHeight = (int)(scrHeight * scr2fbY);
    assert(SLApplication::scene && "No SLApplication::scene!");

    setupGUI(SLApplication::name, SLApplication::configPath, dpi);
    // Set default font sizes depending on the dpi no matter if ImGui is used
    //todo: is this still needed?
    if (!SLApplication::dpi)
        SLApplication::dpi = dpi;

    _sv = new SLSceneView();
    _sv->init("SceneView",
              screenWidth,
              screenHeight,
              nullptr,
              nullptr,
              _gui.get());

    loadWAISceneView(SLApplication::scene, _sv, "default", "default");
    return (SLint)_sv->index();
}

//-----------------------------------------------------------------------------
void WAIApp::loadWAISceneView(SLScene* s, SLSceneView* sv, std::string location, std::string area)
{
    s->init();
    _waiScene->rebuild(location, area);

    // Set scene name and info string
    s->name("Track Keyframe based Features");
    s->info("Example for loading an existing pose graph with map points.");

    // Save no energy
    sv->doWaitOnIdle(false); //for constant video feed
    sv->camera(_waiScene->cameraNode);

    _videoImage = new SLGLTexture("LiveVideoError.png", GL_LINEAR, GL_LINEAR);
    //_testTexture = new SLGLTexture("LiveVideoError.png", GL_LINEAR, GL_LINEAR);
    _waiScene->cameraNode->background().texture(_videoImage);

    s->root3D(_waiScene->rootNode);

    sv->onInitialize();
    sv->doWaitOnIdle(false);

    /*OrbSlamStartResult orbSlamStartResult = startOrbSlam();

    if (!orbSlamStartResult.wasSuccessful)
    {
        errorDial->setErrorMsg(orbSlamStartResult.errorString);
        _gui->uiPrefs->showError = true;
    }*/

    sv->setViewportFromRatio(SLVec2i(_liveVideoTargetWidth, _liveVideoTargetHeight), SLViewportAlign::VA_center, true);

    //do once an onResize in update loop so that everything is aligned correctly
    //_resizeWindow = true;
}

//-----------------------------------------------------------------------------
void WAIApp::setupGUI(std::string appName, std::string configDir, int dotsPerInch)
{
    _gui = std::make_unique<AppDemoWaiGui>(SLApplication::name, SLApplication::configPath, dotsPerInch);
    //aboutDial = new AppDemoGuiAbout("about", cpvrLogo, &uiPrefs.showAbout);
    _gui->addInfoDialog(new AppDemoGuiInfosFrameworks("frameworks", &_gui->uiPrefs->showInfosFrameworks));
    _gui->addInfoDialog(new AppDemoGuiInfosMapNodeTransform("map node",
                                                            &_gui->uiPrefs->showInfosMapNodeTransform,
                                                            &_eventQueue));

    _gui->addInfoDialog(new AppDemoGuiInfosScene("scene", &_gui->uiPrefs->showInfosScene));
    _gui->addInfoDialog(new AppDemoGuiInfosSensors("sensors", &_gui->uiPrefs->showInfosSensors));
    _gui->addInfoDialog(new AppDemoGuiInfosTracking("tracking", *_gui->uiPrefs.get(), *this));
    _gui->addInfoDialog(new AppDemoGuiSlamLoad("slam load",
                                               &_eventQueue,
                                               _dirs.writableDir + "erleb-AR/locations/",
                                               _dirs.writableDir + "calibrations/",
                                               _dirs.writableDir + "voc/",
                                               _featureExtractorFactory.getExtractorIdToNames(),
                                               &_gui->uiPrefs->showSlamLoad,
                                               this->_currentSlamParams));

    _gui->addInfoDialog(new AppDemoGuiProperties("properties", &_gui->uiPrefs->showProperties));
    _gui->addInfoDialog(new AppDemoGuiSceneGraph("scene graph", &_gui->uiPrefs->showSceneGraph));
    _gui->addInfoDialog(new AppDemoGuiStatsDebugTiming("debug timing", &_gui->uiPrefs->showStatsDebugTiming));
    _gui->addInfoDialog(new AppDemoGuiStatsTiming("timing", &_gui->uiPrefs->showStatsTiming));
    _gui->addInfoDialog(new AppDemoGuiStatsVideo("video", &_gui->uiPrefs->showStatsVideo, *this));
    _gui->addInfoDialog(new AppDemoGuiTrackedMapping("tracked mapping", &_gui->uiPrefs->showTrackedMapping, *this));

    _gui->addInfoDialog(new AppDemoGuiTransform("transform", &_gui->uiPrefs->showTransform));
    _gui->addInfoDialog(new AppDemoGuiUIPrefs("prefs", _gui->uiPrefs.get(), &_gui->uiPrefs->showUIPrefs));

    _gui->addInfoDialog(new AppDemoGuiVideoStorage("video storage", &_gui->uiPrefs->showVideoStorage, &_eventQueue, *this));
    _gui->addInfoDialog(new AppDemoGuiVideoControls("video load", &_gui->uiPrefs->showVideoControls, &_eventQueue, *this));

    _errorDial = new AppDemoGuiError("Error", &_gui->uiPrefs->showError);
    _gui->addInfoDialog(_errorDial);
    //TODO: AppDemoGuiInfosDialog are never deleted. Why not use smart pointer when the reponsibility for an object is not clear?
}

//-----------------------------------------------------------------------------
void WAIApp::setupDefaultErlebARDirTo(std::string dir)
{
    dir = Utils::unifySlashes(dir);
    if (!Utils::dirExists(dir))
    {
        Utils::makeDir(dir);
    }
    //calibrations directory
    if (!Utils::dirExists(dir + "calibrations/"))
    {
        Utils::makeDir(dir + "calibrations/");
    }

    dir += "erleb-AR/";
    if (!Utils::dirExists(dir))
    {
        Utils::makeDir(dir);
    }

    dir += "locations/";
    if (!Utils::dirExists(dir))
    {
        Utils::makeDir(dir);
    }

    dir += "default/";
    if (!Utils::dirExists(dir))
    {
        Utils::makeDir(dir);
    }

    dir += "default/";
    if (!Utils::dirExists(dir))
    {
        Utils::makeDir(dir);
    }
}
//-----------------------------------------------------------------------------
void WAIApp::downloadCalibratinFilesTo(std::string dir)
{
    const std::string ftpHost = "pallas.bfh.ch:21";
    const std::string ftpUser = "upload";
    const std::string ftpPwd  = "FaAdbD3F2a";
    const std::string ftpDir  = "erleb-AR/calibrations/";
    std::string       errorMsg;
    if (!FtpUtils::downloadAllFilesFromDir(dir,
                                           ftpHost,
                                           ftpUser,
                                           ftpPwd,
                                           ftpDir,
                                           errorMsg))
    {
        errorMsg = "Failed to download calibration files. Error: " + errorMsg;
        this->showErrorMsg(errorMsg);
    }
}
//-----------------------------------------------------------------------------
bool WAIApp::checkCalibration(const std::string& calibDir, const std::string& calibFileName)
{
    CVCalibration testCalib(CVCameraType::FRONTFACING, "");
    testCalib.load(calibDir, calibFileName);
    if (testCalib.cameraMat().empty() || testCalib.distortion().empty()) //app will crash if distortion is empty
        return false;
    if (testCalib.numCapturedImgs() == 0) //if this is 0 then the calibration is automatically invalidated in load()
        return false;
    if (testCalib.imageSize() == cv::Size(0, 0))
        return false;

    return true;
}

//-----------------------------------------------------------------------------
bool WAIApp::updateSceneViews()
{
    bool needUpdate = false;

    for (auto sv : SLApplication::scene->sceneViews())
        if (sv->onPaint() && !needUpdate)
            needUpdate = true;

    return needUpdate;
}

//-----------------------------------------------------------------------------
void WAIApp::updateTrackingVisualization(const bool iKnowWhereIAm, cv::Mat& imgRGB)
{
    //undistort image and copy image to video texture
    if (_videoImage)
    {

        if (_calibration.state() == CS_calibrated && _showUndistorted)
        {
            _calibration.remap(imgRGB, _undistortedLastFrame[_lastFrameIdx]);
        }
        else
        {
            _undistortedLastFrame[_lastFrameIdx] = imgRGB;
        }

        if (_doubleBufferedOutput)
        {
            _lastFrameIdx = (_lastFrameIdx + 1) % 2;
        }

        _mode->drawInfo(_undistortedLastFrame[_lastFrameIdx], true, _gui->uiPrefs->showKeyPoints, _gui->uiPrefs->showKeyPointsMatched);

        _videoImage->copyVideoImage(_undistortedLastFrame[_lastFrameIdx].cols,
                                    _undistortedLastFrame[_lastFrameIdx].rows,
                                    CVImage::cv2glPixelFormat(_undistortedLastFrame[_lastFrameIdx].type()),
                                    _undistortedLastFrame[_lastFrameIdx].data,
                                    _undistortedLastFrame[_lastFrameIdx].isContinuous(),
                                    true);
    }

    //update map point visualization:
    //if we still want to visualize the point cloud
    if (_gui->uiPrefs->showMapPC)
    {
        //get new points and add them
        renderMapPoints("MapPoints",
                        _mode->getMapPoints(),
                        _waiScene->mapPC,
                        _waiScene->mappointsMesh,
                        _waiScene->redMat);

        /*
        //get new points and add them
        renderMapPoints("MarkerCornerMapPoints",
                        _mode->getMarkerCornerMapPoints(),
                        _waiScene->mapMarkerCornerPC,
                        _waiScene->mappointsMarkerCornerMesh,
                        _waiScene->blueMat);
        */
    }
    else
    {
        if (_waiScene->mappointsMesh)
        {
            //delete mesh if we do not want do visualize it anymore
            _waiScene->mapPC->deleteMesh(_waiScene->mappointsMesh);
        }
        if (_waiScene->mappointsMarkerCornerMesh)
        {
            //delete mesh if we do not want do visualize it anymore
            _waiScene->mapMarkerCornerPC->deleteMesh(_waiScene->mappointsMarkerCornerMesh);
        }
    }

    //update visualization of local map points:
    //only update them with a valid pose from WAI
    if (_gui->uiPrefs->showLocalMapPC && iKnowWhereIAm)
    {
        renderMapPoints("LocalMapPoints",
                        _mode->getLocalMapPoints(),
                        _waiScene->mapLocalPC,
                        _waiScene->mappointsLocalMesh,
                        _waiScene->blueMat);
    }
    else if (_waiScene->mappointsLocalMesh)
    {
        //delete mesh if we do not want do visualize it anymore
        _waiScene->mapLocalPC->deleteMesh(_waiScene->mappointsLocalMesh);
    }

    //update visualization of matched map points
    //only update them with a valid pose from WAI
    if (_gui->uiPrefs->showMatchesPC && iKnowWhereIAm)
    {
        renderMapPoints("MatchedMapPoints",
                        _mode->getMatchedMapPoints(_mode->getLastFrame()),
                        _waiScene->mapMatchedPC,
                        _waiScene->mappointsMatchedMesh,
                        _waiScene->greenMat);
    }
    else if (_waiScene->mappointsMatchedMesh)
    {
        //delete mesh if we do not want do visualize it anymore
        _waiScene->mapMatchedPC->deleteMesh(_waiScene->mappointsMatchedMesh);
    }

    //update keyframe visualization
    _waiScene->keyFrameNode->deleteChildren();
    if (_gui->uiPrefs->showKeyFrames)
    {
        renderKeyframes();
    }

    //update pose graph visualization
    renderGraphs();
}

//-----------------------------------------------------------------------------
void WAIApp::renderMapPoints(std::string                      name,
                             const std::vector<WAIMapPoint*>& pts,
                             SLNode*&                         node,
                             SLPoints*&                       mesh,
                             SLMaterial*&                     material)
{
    //remove old mesh, if it exists
    if (mesh)
        node->deleteMesh(mesh);

    //instantiate and add new mesh
    if (pts.size())
    {
        //get points as Vec3f
        std::vector<SLVec3f> points, normals;
        for (auto mapPt : pts)
        {
            WAI::V3 wP = mapPt->worldPosVec();
            WAI::V3 wN = mapPt->normalVec();
            points.push_back(SLVec3f(wP.x, wP.y, wP.z));
            normals.push_back(SLVec3f(wN.x, wN.y, wN.z));
        }

        mesh = new SLPoints(points, normals, name, material);
        node->addMesh(mesh);
        node->updateAABBRec();
    }
}
//-----------------------------------------------------------------------------
void WAIApp::renderKeyframes()
{
    std::vector<WAIKeyFrame*> keyframes = _mode->getKeyFrames();

    // TODO(jan): delete keyframe textures
    for (WAIKeyFrame* kf : keyframes)
    {
        if (kf->isBad())
            continue;

        SLKeyframeCamera* cam = new SLKeyframeCamera("KeyFrame " + std::to_string(kf->mnId));
        //set background
        if (kf->getTexturePath().size())
        {
            // TODO(jan): textures are saved in a global textures vector (scene->textures)
            // and should be deleted from there. Otherwise we have a yuuuuge memory leak.
#if 0
        SLGLTexture* texture = new SLGLTexture(kf->getTexturePath());
        _kfTextures.push_back(texture);
        cam->background().texture(texture);
#endif
        }

        cv::Mat Twc = kf->getObjectMatrix();

        SLMat4f om;
        om.setMatrix(Twc.at<float>(0, 0),
                     -Twc.at<float>(0, 1),
                     -Twc.at<float>(0, 2),
                     Twc.at<float>(0, 3),
                     Twc.at<float>(1, 0),
                     -Twc.at<float>(1, 1),
                     -Twc.at<float>(1, 2),
                     Twc.at<float>(1, 3),
                     Twc.at<float>(2, 0),
                     -Twc.at<float>(2, 1),
                     -Twc.at<float>(2, 2),
                     Twc.at<float>(2, 3),
                     Twc.at<float>(3, 0),
                     -Twc.at<float>(3, 1),
                     -Twc.at<float>(3, 2),
                     Twc.at<float>(3, 3));
        //om.rotate(180, 1, 0, 0);

        cam->om(om);

        //calculate vertical field of view
        SLfloat fy     = (SLfloat)kf->fy;
        SLfloat cy     = (SLfloat)kf->cy;
        SLfloat fovDeg = 2 * (SLfloat)atan2(cy, fy) * Utils::RAD2DEG;
        cam->fov(fovDeg);
        cam->focalDist(0.11f);
        cam->clipNear(0.1f);
        cam->clipFar(1000.0f);

        _waiScene->keyFrameNode->addChild(cam);
    }
}
//-----------------------------------------------------------------------------
void WAIApp::renderGraphs()
{
    std::vector<WAIKeyFrame*> kfs = _mode->getKeyFrames();

    SLVVec3f covisGraphPts;
    SLVVec3f spanningTreePts;
    SLVVec3f loopEdgesPts;
    for (auto* kf : kfs)
    {
        cv::Mat Ow = kf->GetCameraCenter();

        //covisibility graph
        const std::vector<WAIKeyFrame*> vCovKFs = kf->GetBestCovisibilityKeyFrames(_gui->uiPrefs->minNumOfCovisibles);

        if (!vCovKFs.empty())
        {
            for (vector<WAIKeyFrame*>::const_iterator vit = vCovKFs.begin(), vend = vCovKFs.end(); vit != vend; vit++)
            {
                if ((*vit)->mnId < kf->mnId)
                    continue;
                cv::Mat Ow2 = (*vit)->GetCameraCenter();

                covisGraphPts.push_back(SLVec3f(Ow.at<float>(0), Ow.at<float>(1), Ow.at<float>(2)));
                covisGraphPts.push_back(SLVec3f(Ow2.at<float>(0), Ow2.at<float>(1), Ow2.at<float>(2)));
            }
        }

        //spanning tree
        WAIKeyFrame* parent = kf->GetParent();
        if (parent)
        {
            cv::Mat Owp = parent->GetCameraCenter();
            spanningTreePts.push_back(SLVec3f(Ow.at<float>(0), Ow.at<float>(1), Ow.at<float>(2)));
            spanningTreePts.push_back(SLVec3f(Owp.at<float>(0), Owp.at<float>(1), Owp.at<float>(2)));
        }

        //loop edges
        std::set<WAIKeyFrame*> loopKFs = kf->GetLoopEdges();
        for (set<WAIKeyFrame*>::iterator sit = loopKFs.begin(), send = loopKFs.end(); sit != send; sit++)
        {
            if ((*sit)->mnId < kf->mnId)
                continue;
            cv::Mat Owl = (*sit)->GetCameraCenter();
            loopEdgesPts.push_back(SLVec3f(Ow.at<float>(0), Ow.at<float>(1), Ow.at<float>(2)));
            loopEdgesPts.push_back(SLVec3f(Owl.at<float>(0), Owl.at<float>(1), Owl.at<float>(2)));
        }
    }

    if (_waiScene->covisibilityGraphMesh)
        _waiScene->covisibilityGraph->deleteMesh(_waiScene->covisibilityGraphMesh);

    if (covisGraphPts.size() && _gui->uiPrefs->showCovisibilityGraph)
    {
        _waiScene->covisibilityGraphMesh = new SLPolyline(covisGraphPts, false, "CovisibilityGraph", _waiScene->covisibilityGraphMat);
        _waiScene->covisibilityGraph->addMesh(_waiScene->covisibilityGraphMesh);
        _waiScene->covisibilityGraph->updateAABBRec();
    }

    if (_waiScene->spanningTreeMesh)
        _waiScene->spanningTree->deleteMesh(_waiScene->spanningTreeMesh);

    if (spanningTreePts.size() && _gui->uiPrefs->showSpanningTree)
    {
        _waiScene->spanningTreeMesh = new SLPolyline(spanningTreePts, false, "SpanningTree", _waiScene->spanningTreeMat);
        _waiScene->spanningTree->addMesh(_waiScene->spanningTreeMesh);
        _waiScene->spanningTree->updateAABBRec();
    }

    if (_waiScene->loopEdgesMesh)
        _waiScene->loopEdges->deleteMesh(_waiScene->loopEdgesMesh);

    if (loopEdgesPts.size() && _gui->uiPrefs->showLoopEdges)
    {
        _waiScene->loopEdgesMesh = new SLPolyline(loopEdgesPts, false, "LoopEdges", _waiScene->loopEdgesMat);
        _waiScene->loopEdges->addMesh(_waiScene->loopEdgesMesh);
        _waiScene->loopEdges->updateAABBRec();
    }
}

void WAIApp::saveMap(std::string location,
                     std::string area,
                     std::string marker)
{
    _mode->requestStateIdle();

    std::string slamRootDir = _dirs.writableDir + "erleb-AR/locations/";
    std::string mapDir      = constructSlamMapDir(slamRootDir, location, area);
    if (!Utils::dirExists(mapDir))
        Utils::makeDir(mapDir);

    std::string filename = constructSlamMapFileName(location, area, _mode->getKPextractor()->GetName());
    std::string imgDir   = constructSlamMapImgDir(mapDir, filename);

    if (_mode->retainImage())
    {
        if (!Utils::dirExists(imgDir))
            Utils::makeDir(imgDir);
    }
    /*
    if (!marker.empty())
    {
        cv::Mat nodeTransform;
        if (!_mode->doMarkerMapPreprocessing(constructSlamMarkerDir(slamRootDir, location, area) + marker, nodeTransform, 0.75f))
        {
            showErrorMsg("Failed to do marker map preprocessing");
        }
        else
        {
            std::cout << "nodeTransform: " << nodeTransform << std::endl;
            //_waiScene->mapNode->om(WAIMapStorage::convertToSLMat(nodeTransform));
            if (!WAIMapStorage::saveMap(_mode->getMap(),
                                        _waiScene->mapNode,
                                        mapDir + filename,
                                        imgDir))
            {
                showErrorMsg("Failed to save map " + mapDir + filename);
            }
        }
    }
    else
    {
        */
        if (!WAIMapStorage::saveMap(_mode->getMap(),
                                    _waiScene->mapNode,
                                    mapDir + filename,
                                    imgDir))
        {
            showErrorMsg("Failed to save map " + mapDir + filename);
        }
    //}

    _mode->resume();
}

void WAIApp::saveVideo(std::string filename)
{
    std::string infoDir  = videoDir + "info/";
    std::string infoPath = infoDir + filename;
    std::string path     = videoDir + filename;

    if (!Utils::dirExists(videoDir))
    {
        Utils::makeDir(videoDir);
    }
    else
    {
        if (Utils::fileExists(path))
        {
            Utils::deleteFile(path);
        }
    }

    if (!Utils::dirExists(infoDir))
    {
        Utils::makeDir(infoDir);
    }
    else
    {
        if (Utils::fileExists(infoPath))
        {
            Utils::deleteFile(infoPath);
        }
    }

    if (_videoWriter->isOpened())
    {
        _videoWriter->release();
    }
    if (_videoWriterInfo->isOpened())
    {
        _videoWriterInfo->release();
    }

    bool ret = false;
    if (_videoFileStream)
        ret = _videoWriter->open(path, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 30, _videoFileStream->getFrameSize(), true);
    else if (_camera)
        ret = _videoWriter->open(path, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 30, _camera->getFrameSize(), true);
    else
        WAI_WARN("WAIApp::saveVideo: No active video stream or camera available!");
}

void WAIApp::saveGPSData(std::string videofile)
{
    std::string filename = Utils::getFileNameWOExt(videofile) + ".txt";
    std::string path     = videoDir + filename;
    _gpsDataStream.open(path);
}

void WAIApp::transformMapNode(SLTransformSpace tSpace,
                              SLVec3f          rotation,
                              SLVec3f          translation,
                              float            scale)
{
    _waiScene->mapNode->rotate(rotation.x, 1, 0, 0, tSpace);
    _waiScene->mapNode->rotate(rotation.y, 0, 1, 0, tSpace);
    _waiScene->mapNode->rotate(rotation.z, 0, 0, 1, tSpace);
    _waiScene->mapNode->translate(translation.x, 0, 0, tSpace);
    _waiScene->mapNode->translate(0, translation.y, 0, tSpace);
    _waiScene->mapNode->translate(0, 0, translation.z, tSpace);
    _waiScene->mapNode->scale(scale);
}

void WAIApp::handleEvents()
{
    while (!_eventQueue.empty())
    {
        WAIEvent* event = _eventQueue.front();
        _eventQueue.pop();

        switch (event->type)
        {
            case WAIEventType_StartOrbSlam: {
                WAIEventStartOrbSlam* startOrbSlamEvent = (WAIEventStartOrbSlam*)event;
                loadWAISceneView(SLApplication::scene, _sv, startOrbSlamEvent->params.location, startOrbSlamEvent->params.area);
                startOrbSlam(&startOrbSlamEvent->params);

                delete startOrbSlamEvent;
            }
            break;

            case WAIEventType_SaveMap: {
                WAIEventSaveMap* saveMapEvent = (WAIEventSaveMap*)event;
                saveMap(saveMapEvent->location, saveMapEvent->area, saveMapEvent->marker);

                delete saveMapEvent;
            }
            break;

            case WAIEventType_VideoControl: {
                WAIEventVideoControl* videoControlEvent = (WAIEventVideoControl*)event;
                _pauseVideo                             = videoControlEvent->pauseVideo;
                _videoCursorMoveIndex                   = videoControlEvent->videoCursorMoveIndex;

                delete videoControlEvent;
            }
            break;

            case WAIEventType_VideoRecording: {
                WAIEventVideoRecording* videoRecordingEvent = (WAIEventVideoRecording*)event;

                if (_videoWriter->isOpened())
                {
                    _videoWriter->release();
                    _gpsDataStream.close();
                }
                else
                {
                    saveVideo(videoRecordingEvent->filename);
                    saveGPSData(videoRecordingEvent->filename);
                }

                delete videoRecordingEvent;
            }
            break;

            case WAIEventType_MapNodeTransform: {
                WAIEventMapNodeTransform* mapNodeTransformEvent = (WAIEventMapNodeTransform*)event;

                transformMapNode(mapNodeTransformEvent->tSpace, mapNodeTransformEvent->rotation, mapNodeTransformEvent->translation, mapNodeTransformEvent->scale);

                delete mapNodeTransformEvent;
            }
            break;

            case WAIEventType_None:
            default: {
            }
            break;
        }
    }
}