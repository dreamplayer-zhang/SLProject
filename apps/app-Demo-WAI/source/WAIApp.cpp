#include <WAIApp.h>

void WAIApp::load(int scrWidth, int scrHeight, float scr2fbX, float scr2fbY, int dpi, AppDirectories directories)
{
    //create an event
    addEvent(new InitEvent());
}

void WAIApp::goBack()
{
    addEvent(new GoBackEvent());
}

//state update functions
void WAIApp::stateIdle(const sm::NoEventData* data)
{
    //wait for init
    Utils::log("WAIApp", "stateIdle");
}

void WAIApp::stateInit(const sm::NoEventData* data)
{
    Utils::log("WAIApp", "stateInit");

    //make internal event to proceed to state preocessXY
    //e.g.:
    _abcView = new ABCView(*this);
    _xyView  = new XYView(*this);
    addEvent(new StateDoneEvent());
}

void WAIApp::stateProcessXY(const ABCEventData* data)
  _initializationExtractor = _featureExtractorFactory.make(slamParams.extractorIds.initializationExtractorId, _videoFrameSize);
{
    Utils::log("WAIApp", "stateProcessXY");
    if (data)
    {
        Utils::log("WAIApp", "Message from ABC: %s", data->msg.c_str());
    }
    _initializationExtractor.get(),
}

void WAIApp::stateProcessABC(const sm::NoEventData* data)
{
    Utils::log("WAIApp", "stateProcessABC");
    _abcView->update();
}

void WAIApp::stateStop(const sm::NoEventData* data)
{
    Utils::log("WAIApp", "stateStop");
    addEvent(new StateDoneEvent());
}

//
//#include <SLKeyframeCamera.h>
//#include <Utils.h>
//#include <AverageTiming.h>
//
//#include <WAIMapStorage.h>
//#include <WAICalibration.h>
//#include <AppWAIScene.h>
//#include <SLGLProgramManager.h>
//
////#include <AppDemoGuiInfosDialog.h>
////#include <AppDemoGuiAbout.h>
////#include <AppDemoGuiInfosFrameworks.h>
////#include <AppDemoGuiInfosMapNodeTransform.h>
////#include <AppDemoGuiInfosScene.h>
////#include <AppDemoGuiInfosSensors.h>
////#include <AppDemoGuiInfosTracking.h>
////#include <AppDemoGuiProperties.h>
////#include <AppDemoGuiSceneGraph.h>
////#include <AppDemoGuiStatsDebugTiming.h>
////#include <AppDemoGuiStatsTiming.h>
////#include <AppDemoGuiStatsVideo.h>
////#include <AppDemoGuiTrackedMapping.h>
////#include <AppDemoGuiTransform.h>
////#include <AppDemoGuiUIPrefs.h>
////#include <AppDemoGuiVideoControls.h>
////#include <AppDemoGuiVideoStorage.h>
////#include <AppDemoGuiSlamLoad.h>
////#include <AppDemoGuiTestOpen.h>
////#include <AppDemoGuiTestWrite.h>
//#include <AppDemoGuiError.h>
//
//#include <DeviceData.h>
//#include <AppWAISlamParamHelper.h>
//#include <FtpUtils.h>
//#include <GUIPreferences.h>
//
//#include <GLSLextractor.h>
//#include <SLSceneView.h>
//#include <SLPoints.h>
//#include <SLQuat4.h>
//#include <SLPolyline.h>
//#include <opencv2/imgproc/imgproc.hpp>
//
////move
//#include <SLAssimpImporter.h>
//
////basic information
////-the sceen has a fixed size on android and also a fixed aspect ratio on desktop to simulate android behaviour on desktop
////-the viewport gets adjusted according to the target video aspect ratio (for live video WAIApp::liveVideoTargetWidth and WAIApp::_liveVideoTargetHeight are used and for video file the video frame size is used)
////-the live video gets cropped according to the viewport aspect ratio on android and according to WAIApp::videoFrameWdivH on desktop (which is also used to define the viewport aspect ratio)
////-the calibration gets adjusted according to the video (live and file)
////-the live video gets cropped to the aspect ratio that is defined by the transferred values in load(..) and assigned to liveVideoTargetWidth and liveVideoTargetHeight
//
////-----------------------------------------------------------------------------
//WAIApp::WAIApp()
//  : SLInputEventInterface(_inputManager),
//    _name("WAI Demo App")
//{
//}
////-----------------------------------------------------------------------------
//WAIApp::~WAIApp()
//{
//    close();
//}
//
////-----------------------------------------------------------------------------
//int WAIApp::load(int scrWidth, int scrHeight, float scr2fbX, float scr2fbY, int dpi, AppDirectories directories)
//{
//    _dirs       = directories;
//    _deviceData = std::make_unique<DeviceData>(scrWidth, scrHeight, scr2fbX, scr2fbY, dpi, directories);
//
//    // setup magic paths
//    SLGLProgram::defaultPath      = _dirs.slDataRoot + "/shaders/";
//    SLGLTexture::defaultPath      = _dirs.slDataRoot + "/images/textures/";
//    SLGLTexture::defaultPathFonts = _dirs.slDataRoot + "/images/fonts/";
//    SLAssimpImporter::defaultPath = _dirs.slDataRoot + "/models/";
//
//    _startFromIdle = true;
//    return 0;
//}
//
//void WAIApp::setCamera(SENSCamera* camera)
//{
//    std::lock_guard<std::mutex> lock(_cameraSetMutex);
//    _camera = camera;
//}
//
//SENSCamera* WAIApp::getCamera()
//{
//    std::lock_guard<std::mutex> lock(_cameraSetMutex);
//    return _camera;
//}
//
//void WAIApp::reset()
//{
//    _state            = State::IDLE;
//    _startFromIdle    = false;
//    _startFromStartUp = false;
//
//    _camera           = nullptr;
//    _appMode          = Selection::NONE;
//    _area             = Area::NONE;
//    _switchToTracking = false;
//    _goBack           = false;
//}
//
//void WAIApp::checkStateTransition()
//{
//    switch (_state)
//    {
//        case State::IDLE: {
//            if (_showSelectionState)
//            {
//                //start selection state
//                if (_selectionState && _selectionState->started())
//                    _state = State::SELECTION;
//            }
//            else
//            {
//                //directly go to start up
//                if (_startUpState && _startUpState->started())
//                {
//                    _state            = State::START_UP;
//                    _startFromStartUp = true;
//                }
//            }
//            break;
//        }
//        case State::SELECTION: {
//
//            if (_goBack)
//            {
//                _goBack = false;
//                if (_closeCB)
//                    _closeCB();
//            }
//            else if (_appMode != Selection::NONE && _startUpState->started())
//            {
//                _state            = State::START_UP;
//                _startFromStartUp = true;
//            }
//            break;
//        }
//        case State::START_UP: {
//            if (_startUpState && _startUpState->ready())
//            {
//                if (_appMode == Selection::CAMERA_TEST)
//                {
//                    if (_cameraTestState && _cameraTestState->started())
//                    {
//                        _state = State::CAMERA_TEST;
//                    }
//                }
//                else if (_appMode == Selection::TEST)
//                {
//                    if (_testState && _testState->started())
//                    {
//                        _state = State::TEST;
//                    }
//                }
//                else //(_appMode > Selection::TEST)
//                {
//                    if (_locationMapState && _areaTrackingState && _locationMapState->started() && _areaTrackingState->started())
//                    {
//                        _state = State::LOCATION_MAP;
//                    }
//                }
//            }
//            break;
//        }
//        case State::LOCATION_MAP: {
//            //if (_mapState.ready() && _slamState.started())
//            {
//                //  _state = State::MAP_SCENE
//            }
//            break;
//        }
//        case State::AREA_TRACKING: {
//            break;
//        }
//        case State::TEST: {
//            if (_goBack)
//            {
//                _goBack  = false;
//                _state   = State::SELECTION;
//                _appMode = Selection::NONE;
//                if (_selectionState)
//                    _selectionState->reset();
//            }
//            break;
//        }
//        case State::CAMERA_TEST: {
//            break;
//        }
//        case State::TUTORIAL: {
//            break;
//        }
//    }
//}
//
//bool WAIApp::updateState()
//{
//    bool doUpdate = false;
//    switch (_state)
//    {
//        case State::IDLE: {
//            if (_startFromIdle)
//            {
//                _startFromIdle = false;
//                if (_showSelectionState)
//                {
//                    //select Selection
//                    //(start up can be done as soon we have access to resouces)
//                    if (!_selectionState)
//                    {
//                        _selectionState = new SelectionState(_inputManager,
//                                                             _deviceData->scrWidth(),
//                                                             _deviceData->scrHeight(),
//                                                             _deviceData->dpi(),
//                                                             _deviceData->fontDir(),
//                                                             _deviceData->dirs().writableDir);
//                        _selectionState->start();
//                    }
//                }
//
//                _startUpState = new StartUpState(_inputManager,
//                                                 _deviceData->scrWidth(),
//                                                 _deviceData->scrHeight(),
//                                                 _deviceData->dpi(),
//                                                 _deviceData->dirs().writableDir);
//                _startUpState->start();
//            }
//            break;
//        }
//        case State::SELECTION: {
//            doUpdate = _selectionState->update();
//            if (_selectionState->ready())
//            {
//                _appMode = _selectionState->getSelection();
//            }
//            break;
//        }
//        case State::START_UP: {
//            //show loading screen
//            doUpdate = _startUpState->update();
//
//            if (_startFromStartUp && getCamera() && _camera->started())
//            {
//                _startFromStartUp = false;
//                if (_showSelectionState)
//                    _appMode = _selectionState->getSelection();
//
//                if (_appMode == Selection::NONE)
//                {
//                    Utils::warnMsg("WAIApp", "No Selection selected!", __LINE__, __FILE__);
//                }
//                else if (_appMode == Selection::TEST)
//                {
//                    if (!_testState)
//                    {
//                        _testState = new TestState(_inputManager,
//                                                   getCamera(),
//                                                   _deviceData->scrWidth(),
//                                                   _deviceData->scrHeight(),
//                                                   _deviceData->dpi(),
//                                                   _deviceData->fontDir(),
//                                                   _deviceData->dirs().writableDir,
//                                                   _deviceData->dirs().vocabularyDir,
//                                                   _deviceData->calibDir(),
//                                                   _deviceData->videoDir(),
//                                                   std::bind(&WAIApp::goBack, this));
//                        _testState->start();
//                    }
//                }
//                else if (_appMode == Selection::CAMERA_TEST)
//                {
//                    _cameraTestState = new CameraTestState;
//                    _cameraTestState->start();
//                }
//                else
//                {
//                    if (_appMode == Selection::AUGST)
//                        _locationMapState = new AugstMapState;
//                    else if (_appMode == Selection::AVANCHES)
//                        _locationMapState = new AvanchesMapState;
//                    else if (_appMode == Selection::BIEL)
//                        _locationMapState = new BielMapState;
//                    else if (_appMode == Selection::CHRISTOFFELTOWER)
//                        _locationMapState = new ChristoffelMapState;
//
//                    _locationMapState->start();
//                }
//            }
//            break;
//        }
//        case State::LOCATION_MAP: {
//
//            doUpdate = _locationMapState->update();
//            if (_locationMapState->ready())
//            {
//                _area             = _locationMapState->getTargetArea();
//                _switchToTracking = true;
//            }
//            break;
//        }
//        case State::AREA_TRACKING: {
//            doUpdate = _areaTrackingState->update();
//
//            if (_areaTrackingState->ready())
//            {
//                //todo:
//            }
//
//            break;
//        }
//        case State::TEST: {
//            doUpdate = _testState->update();
//            break;
//        }
//        case State::CAMERA_TEST: {
//            break;
//        }
//        case State::TUTORIAL: {
//            break;
//        }
//    }
//
//    return doUpdate;
//}
//
//bool WAIApp::update()
//{
//    bool doUpdate = false;
//
//    try
//    {
//        checkStateTransition();
//        doUpdate = updateState();
//    }
//    catch (std::exception& e)
//    {
//        Utils::log("WAIApp", "Std exception catched in update() %s", e.what());
//    }
//    catch (...)
//    {
//        Utils::log("WAIApp", "Unknown exception catched in update()");
//    }
//
//    return doUpdate;
//}
//
//void WAIApp::close()
//{
//    terminate();
//    reset();
//
//    //ATTENTION: if we dont do this we get problems when opening the app the second time
//    //(e.g. "The position attribute has no variable location." from SLGLVertexArray)
//    //We still cannot get rid of this stupid singleton instance..
//    SLGLProgramManager::deletePrograms();
//    SLMaterialDefaultGray::deleteInstance();
//    SLMaterialDiffuseAttribute::deleteInstance();
//}
//
//void WAIApp::terminate()
//{
//    //todo: terminate event with no event data
//    if (_selectionState)
//    {
//        delete _selectionState;
//        _selectionState = nullptr;
//    }
//    if (_startUpState)
//    {
//        delete _startUpState;
//        _startUpState = nullptr;
//    }
//    if (_areaTrackingState)
//    {
//        delete _areaTrackingState;
//        _areaTrackingState = nullptr;
//    }
//    if (_cameraTestState)
//    {
//        delete _cameraTestState;
//        _cameraTestState = nullptr;
//    }
//    if (_locationMapState)
//    {
//        delete _locationMapState;
//        _locationMapState = nullptr;
//    }
//    if (_testState)
//    {
//        delete _testState;
//        _testState = nullptr;
//    }
//    if (_tutorialState)
//    {
//        delete _tutorialState;
//        _tutorialState = nullptr;
//    }
//}
//
//std::string WAIApp::name()
//{
//    return _name;
//}

#if 0
void WAIApp::transformMapNode(SLTransformSpace tSpace,
                              SLVec3f          rotation,
                              SLVec3f          translation,
                              float            scale)
{
    _waiScene.mapNode->rotate(rotation.x, 1, 0, 0, tSpace);
    _waiScene.mapNode->rotate(rotation.y, 0, 1, 0, tSpace);
    _waiScene.mapNode->rotate(rotation.z, 0, 0, 1, tSpace);
    _waiScene.mapNode->translate(translation.x, 0, 0, tSpace);
    _waiScene.mapNode->translate(0, translation.y, 0, tSpace);
    _waiScene.mapNode->translate(0, 0, translation.z, tSpace);
    _waiScene.mapNode->scale(scale);
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
                startOrbSlam(startOrbSlamEvent->params);

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

                //if videoWriter is opened we assume that recording was started before
                if (_videoWriter && _videoWriter->isOpened() /*|| _gpsDataStream.is_open()*/)
                {
                    if (_videoWriter && _videoWriter->isOpened())
                        _videoWriter->release();
                    //if (_gpsDataStream.is_open())
                    //    _gpsDataStream.close();
                }
                else
                {
                    saveVideo(videoRecordingEvent->filename);
                    //saveGPSData(videoRecordingEvent->filename);
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

            case WAIEventType_DownloadCalibrationFiles: {
                WAIEventDownloadCalibrationFiles* downloadEvent = (WAIEventDownloadCalibrationFiles*)event;
                delete downloadEvent;
                downloadCalibrationFilesTo(_calibDir);
            }
            break;

            case WAIEventType_AdjustTransparency: {
                WAIEventAdjustTransparency* adjustTransparencyEvent = (WAIEventAdjustTransparency*)event;
                _waiScene.adjustAugmentationTransparency(adjustTransparencyEvent->kt);

                delete adjustTransparencyEvent;
            }
            break;

            case WAIEventType_EnterEditMode: {
                WAIEventEnterEditMode* enterEditModeEvent = (WAIEventEnterEditMode*)event;

                if (!_transformationNode)
                {
                    _transformationNode = new SLTransformNode(_sv->camera(), _sv, SLApplication::scene->root3D()->findChild<SLNode>("map"));
                    SLApplication::scene->eventHandlers().push_back(_transformationNode);
                    SLApplication::scene->root3D()->addChild(_transformationNode);
                }

                if (enterEditModeEvent->editMode == NodeEditMode_None)
                {
                    std::vector<SLEventHandler*>::iterator it = std::find(SLApplication::scene->eventHandlers().begin(),
                                                                          SLApplication::scene->eventHandlers().end(),
                                                                          _transformationNode);

                    if (it != SLApplication::scene->eventHandlers().end())
                    {
                        SLApplication::scene->eventHandlers().erase(it);
                    }

                    if (SLApplication::scene->root3D()->deleteChild(_transformationNode))
                    {
                        _transformationNode = nullptr;
                    }
                }
                else
                {
                    _transformationNode->toggleEditMode(enterEditModeEvent->editMode);
                }

                delete enterEditModeEvent;
            }
            break;

            case WAIEventType_None:
            default: {
            }
            break;
        }
    }
}
#endif