#include <iostream>

#include <Utils.h>
#include <CVCapture.h>

#include <WAICalibration.h>
#include <WAIMapStorage.h>

#include <WAIFrame.h>
#include <WAIOrbVocabulary.h>
#include <WAIModeOrbSlam2.h>
#include <WAIKeyFrameDB.h>

#include <OrbSlam/KPextractor.h>
#include <OrbSlam/SURFextractor.h>

struct RelocalizationTestResult
{
    bool  wasSuccessful;
    int   frameCount;
    int   relocalizationFrameCount;
    float ratio;
};

struct RelocalizationTestCase
{
    std::string videoFileName;
    std::string calibrationFileName;
};

struct RelocalizationTestBench
{
    std::string location;
    std::string weatherConditions;

    std::string mapFileName;
    std::string vocFileName;

    std::vector<RelocalizationTestCase>   testCases;
    std::vector<RelocalizationTestResult> testResults;
};

bool getCalibrationFileNameFromVideoFileName(std::string  videoFileName,
                                             std::string* calibrationFileName)
{
    bool result = false;

    std::vector<std::string> stringParts;
    Utils::splitString(Utils::getFileNameWOExt(videoFileName), '_', stringParts);

    WAICalibration wc;
    if (stringParts.size() >= 3)
    {
        std::string computerInfo = stringParts[1];
        *calibrationFileName     = "camCalib_" + computerInfo + "_main.xml";
        result                   = true;
    }

    return result;
}

RelocalizationTestResult runRelocalizationTest(std::string videoFile,
                                               std::string calibrationFile,
                                               std::string mapFile,
                                               std::string vocFile)
{
    RelocalizationTestResult result = {};

    // TODO(dgj1): this is kind of a hack... improve (maybe separate function call??)
    WAIFrame::mbInitialComputations = true;

    WAIOrbVocabulary::initialize(vocFile);
    ORB_SLAM2::ORBVocabulary* orbVoc     = WAIOrbVocabulary::get();
    ORB_SLAM2::KPextractor*   extractor  = new ORB_SLAM2::SURFextractor(800);
    WAIKeyFrameDB*            keyFrameDB = new WAIKeyFrameDB(*orbVoc);

    WAIMap* map = new WAIMap("map");
    WAIMapStorage::loadMap(map, keyFrameDB, nullptr, mapFile, false, true);

    CVCapture::instance()->videoType(VT_FILE);
    CVCapture::instance()->videoFilename = videoFile;
    CVCapture::instance()->videoLoops    = false;

    CVSize2i videoSize       = CVCapture::instance()->openFile();
    float    widthOverHeight = (float)videoSize.width / (float)videoSize.height;

    WAICalibration wc;
    wc.loadFromFile(calibrationFile);

    unsigned int lastRelocFrameId         = 0;
    int          frameCount               = 0;
    int          relocalizationFrameCount = 0;
    while (CVCapture::instance()->grabAndAdjustForSL(widthOverHeight))
    {
        WAIFrame currentFrame = WAIFrame(CVCapture::instance()->lastFrameGray,
                                         0.0f,
                                         extractor,
                                         wc.cameraMat(),
                                         wc.distortion(),
                                         orbVoc,
                                         false);

        if (WAI::ModeOrbSlam2::relocalization(currentFrame, keyFrameDB, &lastRelocFrameId, *map, false))
        {
            relocalizationFrameCount++;
        }

        frameCount++;
    }

    result.frameCount               = frameCount;
    result.relocalizationFrameCount = relocalizationFrameCount;
    result.ratio                    = ((float)relocalizationFrameCount / (float)frameCount);
    result.wasSuccessful            = true;

    return result;
}

void runRelocalizationTestBench(RelocalizationTestBench& b)
{
    b.testResults.resize(b.testCases.size());

    for (int i = 0; i < b.testCases.size(); i++)
    {
        RelocalizationTestCase   c = b.testCases[i];
        std::cout << "run test " << c.videoFileName << " " << c.calibrationFileName << " " << b.mapFileName << " " << b.vocFileName << std::endl;
        RelocalizationTestResult r = runRelocalizationTest(c.videoFileName, c.calibrationFileName, b.mapFileName, b.vocFileName);

        // scene;conditions;videoFileName;calibrationName;mapName;vocName;frameCount;relocalizationFrameCount;ratio
        printf("%s;%s;%s;%s;%s;%s;%i;%i;%.2f\n",
               b.location.c_str(),
               b.weatherConditions.c_str(),
               c.videoFileName.c_str(),
               c.calibrationFileName.c_str(),
               b.mapFileName.c_str(),
               b.vocFileName.c_str(),
               r.frameCount,
               r.relocalizationFrameCount,
               r.ratio);
    }
}

void addRelocalizationTestCase(std::vector<RelocalizationTestCase>& tests,
                               std::string                          videoFileName,
                               std::string                          calibrationFileName = "")
{
    RelocalizationTestCase test = {};

    test.videoFileName = videoFileName;

    if (calibrationFileName.empty())
    {
        if (getCalibrationFileNameFromVideoFileName(test.videoFileName, &test.calibrationFileName))
        {
            tests.push_back(test);
        }
        else
        {
            std::cerr << "Calibration file not found" << std::endl;
        }
    }
    else
    {
        test.calibrationFileName = calibrationFileName;
        tests.push_back(test);
    }
}

void addRelocalizationTestCase(RelocalizationTestBench& testBench,
                               std::string              videoFileName,
                               std::string              calibrationFileName = "")
{
    addRelocalizationTestCase(testBench.testCases, videoFileName, calibrationFileName);
}

RelocalizationTestBench createRelocalizationTestBench(std::string location,
                                                      std::string weatherConditions,
                                                      std::string mapFileName,
                                                      std::string vocFileName)
{
    RelocalizationTestBench result = {};

    result.location          = location;
    result.weatherConditions = weatherConditions;
    result.mapFileName       = mapFileName;
    result.vocFileName       = vocFileName;

    return result;
}

#define RELOC   0
#define TRACK   1
#define DEFAULT_ANALYZER RELOC
#define DEFAULT_SCENE "default"
#define DEFAULT_WEATHER "sunny"
#define DEFAULT_DIR "."
#define DEFAULT_MAP "map.json"
#define DEFAULT_CALIB ""
#define DEFAULT_VOC "voc.bin"


void parseArgs(int argc, char ** argv)
{
    int analyzer = DEFAULT_ANALYZER;
    std::string scene = DEFAULT_SCENE;
    std::string weather = DEFAULT_WEATHER;
    std::string calib = DEFAULT_CALIB;
    std::string map = DEFAULT_MAP;
    std::string voc = DEFAULT_VOC;
    std::string dir = DEFAULT_DIR;


    std::vector<std::string> args;
    for (int i = 1; i < argc; i++)
        args.push_back(std::string(argv[i]));

    for (int i = 0; i < args.size(); i++)
    {
        std::string s = args[i];
        if (s.compare("-t") == 0 && i < args.size()-1)
        {
            if (args[i+1].compare("reloc") == 0)
                analyzer = RELOC;
            else if (args[i+1].compare("tracking") == 0)
                analyzer = TRACK;
            i++;
        }
        else if (s.compare("-s") == 0 && i < args.size()-1)
        {
            scene = args[i+1];
            i++;
        }
        else if (s.compare("-w") == 0 && i < args.size()-1)
        {
            if (args[i+1].compare("sunny") == 0)
                weather = "sunny";
            else if (args[i+1].compare("shade") == 0)
                weather = "shade";
            else if (args[i+1].compare("cloudy") == 0)
                weather = "cloudy";
            i++;
        }
        else if (s.compare("-c") == 0 && i < args.size()-1)
        {
            calib = args[i+1];
            i++;
        }
        else if (s.compare("-m") == 0 && i < args.size()-1)
        {
            map = args[i+1];
            i++;
        }
        else if (s.compare("-v") == 0 && i < args.size()-1)
        {
            voc = args[i+1];
            i++;
        }
        else
        {
            dir = Utils::unifySlashes(args[i]);
        }
    }

    switch(analyzer)
    {
        case RELOC:
            std::cout << "Relocalization test" << std::endl;
            break;
        case TRACK:
            std::cout << "Tracking test" << std::endl;
            break;
    }
    std::cout << "scene " << scene << std::endl;
    std::cout << "weather " << weather << std::endl;
    if (calib.compare("") == 0)
        std::cout << "calibration file path is get from video name" << std::endl;
    else
        std::cout << "calibration : " << calib << std::endl;

    std::cout << "vocabulary " << voc  << std::endl;
    std::cout << "directory " << dir << std::endl;

    if (analyzer == RELOC)
    {
        RelocalizationTestBench testBench = createRelocalizationTestBench(scene, weather, map, voc);
        std::vector<std::string> files = Utils::getFileNamesInDir(dir);
        for (auto it = files.begin(); it != files.end(); it++)
        {
            std::string ext = Utils::getFileExt(*it);
            if (ext.compare("avi") == 0 ||  ext.compare("mp4") == 0)
            {
                addRelocalizationTestCase(testBench, *it, calib);
            }
        }
        runRelocalizationTestBench(testBench);
    }
    else if (analyzer == TRACK)
    {

    }
    return;
}

int main(int argc, char ** argv)
{
    DUtils::Random::SeedRandOnce(1337);

    parseArgs(argc, argv);

    return 0;
}
