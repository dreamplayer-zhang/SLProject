#include <WAIHelper.h>
#include <WAIKeyFrameDB.h>
#include <WAIMap.h>
#include <WAIOrbVocabulary.h>
#include <OrbSlam/LocalMapping.h>
#include <OrbSlam/LoopClosing.h>
#include <OrbSlam/Initializer.h>
#include <OrbSlam/ORBmatcher.h>
#include <OrbSlam/Optimizer.h>
#include <OrbSlam/PnPsolver.h>
#include <opencv2/core.hpp>

enum TrackingState
{
    TrackingState_None,
    TrackingState_Idle,
    TrackingState_Initializing,
    TrackingState_TrackingOK,
    TrackingState_TrackingLost
};

struct LocalMap
{
    WAIKeyFrame*              refKF;
    std::vector<WAIKeyFrame*> keyFrames;
    std::vector<WAIMapPoint*> mapPoints;
};

struct InitializerData
{
    Initializer*             initializer;
    WAIFrame                 initialFrame;
    std::vector<cv::Point2f> prevMatched;
    std::vector<cv::Point3f> iniPoint3D;
    std::vector<int>         iniMatches;
};

class WAISlam
{
public:
    WAISlam(cv::Mat intrinsic, cv::Mat distortion, std::string orbVocFile, KPextractor* extractorp);

    void drawInfo(cv::Mat& imageRGB,
                  bool     showInitLine,
                  bool     showKeyPoints,
                  bool     showKeyPointsMatched);

    bool           hasStateIdle();
    void           requestStateIdle();
    WAIMap*        getMap();
    WAIKeyFrameDB* getKfDB();

    bool retainImage();
    void resume();
    void setInitialized(bool b);
    bool isInitialized();

    KPextractor*                                              getKPextractor();
    WAIFrame*                                                 getLastFrame();
    std::vector<WAIMapPoint*>                                 getMapPoints();
    std::vector<WAIKeyFrame*>                                 getKeyFrames();
    std::vector<WAIMapPoint*>                                 getLocalMapPoints();
    std::pair<std::vector<cv::Vec3f>, std::vector<cv::Vec2f>> getMatchedCorrespondances(WAIFrame& frame);
    std::string                                               getPrintableState();
    int                                                       getKeyPointCount();
    int                                                       getKeyFrameCount();
    int                                                       getMapPointCount();
    cv::Mat                                                   getPose();
    bool                                                      update(cv::Mat& imageGray, cv::Mat& imageRGB);
    cv::Mat                                                   getExtrinsic();
    void                                                      reset();

    static void                      drawKeyPointInfo(WAIFrame& frame, cv::Mat& image);
    static void                      drawKeyPointMatches(WAIFrame& frame, cv::Mat& image);
    static std::vector<WAIMapPoint*> getMatchedMapPoints(WAIFrame* frame);

    static void drawInitInfo(InitializerData& iniData, WAIFrame& newFrame, cv::Mat& imageRGB);

    static bool initialize(InitializerData& iniData,
                           cv::Mat&         camera,
                           cv::Mat&         distortion,
                           ORBVocabulary*   voc,
                           WAIMap*          map,
                           WAIKeyFrameDB*   keyFrameDatabase,
                           LocalMap&        lmap,
                           LocalMapping*    lmapper,
                           LoopClosing*     loopClosing,
                           WAIKeyFrame**    lastKeyFrame,
                           WAIFrame&        frame);

    static bool relocalization(WAIFrame&      currentFrame,
                               WAIMap*        waiMap,
                               WAIKeyFrameDB* keyFrameDatabase);

    static int tracking(WAIMap*        map,
                        WAIKeyFrameDB* keyFrameDatabase,
                        LocalMap&      localMap,
                        WAIFrame&      frame,
                        WAIFrame&      lastFrame,
                        int            lastRelocFrameId,
                        cv::Mat&       velocity);

    static void mapping(WAIMap*        map,
                        WAIKeyFrameDB* keyFrameDatabase,
                        LocalMap&      localMap,
                        LocalMapping*  localMapper,
                        WAIFrame&      frame,
                        WAIKeyFrame**  lastKf,
                        int            inliners);

    static void motionModel(WAIFrame& frame,
                            WAIFrame& lastFrame,
                            cv::Mat&  velocity,
                            cv::Mat&  pose);

    static bool trackReferenceKeyFrame(LocalMap& map, WAIFrame& lastFrame, WAIFrame& frame);

    static bool trackWithMotionModel(cv::Mat velocity, WAIFrame& previousFrame, WAIFrame& frame);

    static void updateLocalMap(WAIFrame& frame,
                               LocalMap& lmap);

    static int matchLocalMapPoints(LocalMap& lmap, int lastRelocFrameId, WAIFrame& frame);

    static bool needNewKeyFrame(WAIMap*       map,
                                LocalMap&     lmap,
                                LocalMapping* lmapper,
                                WAIKeyFrame*  lastKeyFrame,
                                WAIFrame&     frame,
                                int           nInliners);

    static bool createInitialMapMonocular(InitializerData& iniData,
                                          ORBVocabulary*   voc,
                                          WAIMap*          map,
                                          LocalMapping*    lmapper,
                                          LoopClosing*     loopCloser,
                                          LocalMap&        lmap,
                                          int              mapPointsNeeded,
                                          WAIKeyFrameDB*   keyFrameDatabase,
                                          WAIKeyFrame**    lastKeyFrame,
                                          WAIFrame&        frame);

    static WAIKeyFrame* createNewKeyFrame(LocalMapping*  localMapper,
                                          LocalMap&      lmap,
                                          WAIMap*        map,
                                          WAIKeyFrameDB* keyFrameDatabase,
                                          WAIFrame&      frame);

private:
    InitializerData _iniData;
    LocalMap        _localMap;
    WAIMap*         _globalMap;
    TrackingState   _state;
    LocalMapping*   _localMapping;
    LoopClosing*    _loopClosing;
    WAIKeyFrameDB*  _keyFrameDatabase;
    KPextractor*    _extractor;
    ORBVocabulary*  _voc;
    cv::Mat         _distortion;
    cv::Mat         _cameraIntrinsic;
    cv::Mat         _cameraExtrinsic;
    std::thread*    _localMappingThread = nullptr;
    std::thread*    _loopClosingThread  = nullptr;
    std::mutex      _mutexStates;
    WAIFrame        _lastFrame;
    WAIKeyFrame*    _lastKeyFrame;
    bool            _initialized;
    int             _lastRelocId;
    cv::Mat         _velocity;
};
