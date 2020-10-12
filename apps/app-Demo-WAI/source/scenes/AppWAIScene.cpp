#include "AppWAIScene.h"
#include <SLBox.h>
#include <SLLightDirect.h>
#include <SLLightSpot.h>
#include <SLCoordAxis.h>
#include <SLPoints.h>
#include <SLAssimpImporter.h>
#include <SLVec4.h>
#include <SLKeyframeCamera.h>
#include <SLGLProgramManager.h>

AppWAIScene::AppWAIScene(SLstring name, std::string dataDir, std::string erlebARDir)
  : SLScene(name, nullptr),
    _dataDir(Utils::unifySlashes(dataDir)),
    _erlebARDir(Utils::unifySlashes(erlebARDir))
{
}

void AppWAIScene::loadMesh(std::string path)
{
    SLAssimpImporter importer;
    augmentationRoot = importer.load(_animManager,
                                     &assets,
                                     path,
                                     _dataDir + "images/textures/",
                                     true,
                                     nullptr,
                                     0.4f);

    // Set some ambient light
    for (auto child : augmentationRoot->children())
    {
        child->drawBits()->set(SL_DB_NOTSELECTABLE, true);
    }

    SLNode* n = augmentationRoot->findChild<SLNode>("TexturedMesh", true);
    if (n)
    {
        n->drawBits()->set(SL_DB_CULLOFF, true);
        n->drawBits()->set(SL_DB_NOTSELECTABLE, true);
    }

    augmentationRoot->drawBits()->set(SL_DB_NOTSELECTABLE, true);
;

    _root3D->addChild(augmentationRoot);
}

void AppWAIScene::hideNode(SLNode* node)
{
    if (node)
    {
        node->drawBits()->set(SL_DB_HIDDEN, true);
    }
}

void AppWAIScene::rebuild(std::string location, std::string area)
{
    Utils::log("AppWAIScene", "rebuild for location %s", location.c_str());
    //init(); //uninitializes everything
    //todo: is this necessary?
    assets.clear();

    // Set scene name and info string
    name("Track Keyframe based Features");
    info("Example for loading an existing pose graph with map points.");

    _root3D           = new SLNode("scene");
    cameraNode        = new SLCamera("Camera 1");
    mapNode           = new SLNode("map");
    mapPC             = new SLNode("MapPC");
    mapMatchedPC      = new SLNode("MapMatchedPC");
    mapLocalPC        = new SLNode("MapLocalPC");
    mapMarkerCornerPC = new SLNode("MapMarkerCornerPC");
    keyFrameNode      = new SLNode("KeyFrames");
    covisibilityGraph = new SLNode("CovisibilityGraph");
    spanningTree      = new SLNode("SpanningTree");
    loopEdges         = new SLNode("LoopEdges");

    redMat = new SLMaterial(&assets, SLGLProgramManager::get(SP_colorUniform), SLCol4f::RED, "Red");
    redMat->program(new SLGLGenericProgram(&assets, _dataDir + "shaders/ColorUniformPoint.vert", _dataDir + "shaders/Color.frag"));
    redMat->program()->addUniform1f(new SLGLUniform1f(UT_const, "u_pointSize", 3.0f));
    greenMat = new SLMaterial(&assets, SLGLProgramManager::get(SP_colorUniform), BFHColors::GreenLight, "Green");
    greenMat->program(new SLGLGenericProgram(&assets, _dataDir + "shaders/ColorUniformPoint.vert", _dataDir + "shaders/Color.frag"));
    greenMat->program()->addUniform1f(new SLGLUniform1f(UT_const, "u_pointSize", 5.0f));
    blueMat = new SLMaterial(&assets, SLGLProgramManager::get(SP_colorUniform), BFHColors::BlueLight, "Blue");
    blueMat->program(new SLGLGenericProgram(&assets, _dataDir + "shaders/ColorUniformPoint.vert", _dataDir + "shaders/Color.frag"));
    blueMat->program()->addUniform1f(new SLGLUniform1f(UT_const, "u_pointSize", 4.0f));


    covisibilityGraphMat = new SLMaterial(&assets, "covisibilityGraphMat", SLCol4f::YELLOW);
    covisibilityGraphMat->program(new SLGLGenericProgram(&assets, _dataDir + "shaders/ColorUniform.vert", _dataDir + "shaders/Color.frag"));
    spanningTreeMat = new SLMaterial(&assets, "spanningTreeMat", SLCol4f::GREEN);
    spanningTreeMat->program(new SLGLGenericProgram(&assets, _dataDir + "shaders/ColorUniform.vert", _dataDir + "shaders/Color.frag"));
    loopEdgesMat = new SLMaterial(&assets, "loopEdgesMat", SLCol4f::RED);
    loopEdgesMat->program(new SLGLGenericProgram(&assets, _dataDir + "shaders/ColorUniform.vert", _dataDir + "shaders/Color.frag"));

    _videoImage = new SLGLTexture(&assets, _dataDir + "images/textures/LiveVideoError.png", GL_LINEAR, GL_LINEAR);
    cameraNode->background().texture(_videoImage, false);

    SLGLProgram* spVideoBackground = new SLGLGenericProgram(&assets,
                                                            _dataDir + "shaders/PerVrtTextureBackground.vert",
                                                            _dataDir + "shaders/PerVrtTextureBackground.frag");

    matVideoBackground = new SLMaterial(&assets,
                                        "matVideoBackground",
                                        _videoImage,
                                        nullptr,
                                        nullptr,
                                        nullptr,
                                        spVideoBackground);

    // Create directional light for the sun light
    SLLightDirect* light1 = new SLLightDirect(&assets, this, 5.0f);
    light1->powers(1.0f, 1.0f, 1.0f);
    light1->attenuation(1, 0, 0);
    light1->translation(0, 10, 0);
    light1->lookAt(10, 0, 10);
    _root3D->addChild(light1);
    // Let the sun be rotated by time and location
    //SLApplication::devLoc.sunLightNode(light1);
    //light1->zenithPowerScaleEnabled(true);

    HighResTimer t;
    if (location == "avenches" || location == "Avenches")
    {
        std::string modelPath;
        if (area == "Amphitheater-Entrance" || area == "Amphitheater")
        {
            std::string      modelPath = _dataDir + "erleb-AR/models/avenches/Aventicum-Amphitheater1.gltf";
            loadMesh(modelPath);
            augmentationRoot->rotate(13.7f, 0, 1, 0, TS_parent);

            // Let the video shine through some objects
            augmentationRoot->findChild<SLNode>("Tht-Aussen-Untergrund")->setMeshMat(matVideoBackground, true);
            augmentationRoot->findChild<SLNode>("Tht-Eingang-Ost-Boden")->setMeshMat(matVideoBackground, true);
            augmentationRoot->findChild<SLNode>("Tht-Arenaboden")->setMeshMat(matVideoBackground, true);
            augmentationRoot->findChild<SLNode>("Tht-Aussen-Terrain")->setMeshMat(matVideoBackground, true);
        }
        else if (area == "Cigognier")
        {
            std::string      modelPath = _dataDir + "erleb-AR/models/avenches/Aventicum-Cigognier1.gltf";
            SLAssimpImporter importer;

            if (!Utils::fileExists(modelPath))
            {
                modelPath = _dataDir + "erleb-AR/models/avenches/Aventicum-Cigognier1.gltf";
            }

            loadMesh(modelPath);
        }
        else if (area == "Theater-marker" || area == "Theater")
        {
            std::string      modelPath = _dataDir + "erleb-AR/models/avenches/Aventicum-Theater1.gltf";
            SLAssimpImporter importer;

            loadMesh(modelPath);
        }
    }
    else if (location == "Augst")
    {
        std::string      modelPath = _erlebARDir + "models/augst/Tempel-Theater-02.gltf";
        SLAssimpImporter importer;

        if (!Utils::fileExists(modelPath))
        {
            modelPath = _dataDir + "models/Tempel-Theater-02.gltf";
        }

        loadMesh(modelPath);

        hideNode(augmentationRoot->findChild<SLNode>("Tmp-Portikus-Sockel", true));
        hideNode(augmentationRoot->findChild<SLNode>("Tmp-Boden", true));
        hideNode(augmentationRoot->findChild<SLNode>("Tht-Boden", true));
        hideNode(augmentationRoot->findChild<SLNode>("Tht-Boden-zw-Tht-Tmp", true));
    }
    else if (location == "Bern" || location == "bern")
    {
#if 1
        std::string modelPath = _dataDir + "erleb-AR/models/bern/Bern-Bahnhofsplatz.fbx";

        SLAssimpImporter importer;
        augmentationRoot = importer.load(_animManager,
                                         &assets,
                                         modelPath,
                                         _dataDir + "images/textures/");

        hideNode(augmentationRoot->findChild<SLNode>("Boden", true));
        hideNode(augmentationRoot->findChild<SLNode>("Baldachin-Stahl", true));
        hideNode(augmentationRoot->findChild<SLNode>("Baldachin-Glas", true));
        hideNode(augmentationRoot->findChild<SLNode>("Umgebung-Daecher", true));
        hideNode(augmentationRoot->findChild<SLNode>("Umgebung-Fassaden", true));

        hideNode(augmentationRoot->findChild<SLNode>("Mauer-Wand", true));
        hideNode(augmentationRoot->findChild<SLNode>("Mauer-Dach", true));
        hideNode(augmentationRoot->findChild<SLNode>("Mauer-Turm", true));
        hideNode(augmentationRoot->findChild<SLNode>("Mauer-Weg", true));
        hideNode(augmentationRoot->findChild<SLNode>("Graben-Mauern", true));
        hideNode(augmentationRoot->findChild<SLNode>("Graben-Bruecken", true));
        hideNode(augmentationRoot->findChild<SLNode>("Graben-Grass", true));
        hideNode(augmentationRoot->findChild<SLNode>("Graben-Turm-Dach", true));
        hideNode(augmentationRoot->findChild<SLNode>("Graben-Turm-Fahne", true));
        hideNode(augmentationRoot->findChild<SLNode>("Graben-Turm-Stein", true));

        /*
        mauer_wand          = bern->findChild<SLNode>("Mauer-Wand", true);
        mauer_dach          = bern->findChild<SLNode>("Mauer-Dach", true);
        mauer_turm          = bern->findChild<SLNode>("Mauer-Turm", true);
        mauer_weg           = bern->findChild<SLNode>("Mauer-Weg", true);
        grab_mauern         = bern->findChild<SLNode>("Graben-Mauern", true);
        grab_brueck         = bern->findChild<SLNode>("Graben-Bruecken", true);
        grab_grass          = bern->findChild<SLNode>("Graben-Grass", true);
        grab_t_dach         = bern->findChild<SLNode>("Graben-Turm-Dach", true);
        grab_t_fahn         = bern->findChild<SLNode>("Graben-Turm-Fahne", true);
        grab_t_stein        = bern->findChild<SLNode>("Graben-Turm-Stein", true);
        christ_aussen       = bern->findChild<SLNode>("Christoffel-Aussen", true);
        christ_innen        = bern->findChild<SLNode>("Christoffel-Innen", true);
        */

        // Create directional light for the sun light
        _root3D->addChild(augmentationRoot);

#endif
    }
    else if (location == "Biel" || location == "biel")
    {
        std::string modelPath = _dataDir + "erleb-AR/models/bern/Bern-Bahnhofsplatz.fbx";
        Utils::log("AppWAIScene", "loading model from path: %s", modelPath.c_str());
        SLAssimpImporter importer;
        augmentationRoot = importer.load(_animManager,
                                         &assets,
                                         modelPath,
                                         _dataDir + "images/textures/");

        //hideNode(augmentationRoot->findChild<SLNode>("Boden", true));
        //hideNode(augmentationRoot->findChild<SLNode>("Baldachin-Stahl", true));
        //hideNode(augmentationRoot->findChild<SLNode>("Baldachin-Glas", true));
        //hideNode(augmentationRoot->findChild<SLNode>("Umgebung-Daecher", true));
        //hideNode(augmentationRoot->findChild<SLNode>("Umgebung-Fassaden", true));

        hideNode(augmentationRoot->findChild<SLNode>("Mauer-Wand", true));
        hideNode(augmentationRoot->findChild<SLNode>("Mauer-Dach", true));
        hideNode(augmentationRoot->findChild<SLNode>("Mauer-Turm", true));
        hideNode(augmentationRoot->findChild<SLNode>("Mauer-Weg", true));
        hideNode(augmentationRoot->findChild<SLNode>("Graben-Mauern", true));
        hideNode(augmentationRoot->findChild<SLNode>("Graben-Bruecken", true));
        hideNode(augmentationRoot->findChild<SLNode>("Graben-Grass", true));
        hideNode(augmentationRoot->findChild<SLNode>("Graben-Turm-Dach", true));
        hideNode(augmentationRoot->findChild<SLNode>("Graben-Turm-Fahne", true));
        hideNode(augmentationRoot->findChild<SLNode>("Graben-Turm-Stein", true));

        _root3D->addChild(augmentationRoot);

        //adjust camera frustum
        cameraNode->clipNear(1.0f);
        cameraNode->clipFar(10.0f);
    }

#if 0 // office table boxes scene
    //SLBox*      box1     = new SLBox(0.0f, 0.0f, 0.0f, l, h, b, "Box 1", yellow);
    SLBox* box1 = new SLBox(0.0f, 0.0f, 0.0f, 0.355f, 0.2f, 0.1f, "Box 1", yellow);
    //SLBox*  box1     = new SLBox(0.0f, 0.0f, 0.0f, 10.0f, 5.0f, 3.0f, "Box 1", yellow);
    SLNode* boxNode1 = new SLNode(box1, "boxNode1");
    //boxNode1->rotate(-45.0f, 1.0f, 0.0f, 0.0f);
    //boxNode1->translate(10.0f, -5.0f, 15.0f);
    boxNode1->translate(0.316, -1.497f, -0.1f);
    SLBox*  box2     = new SLBox(0.0f, 0.0f, 0.0f, 0.355f, 0.2f, 0.1f, "Box 2", yellow);
    SLNode* boxNode2 = new SLNode(box2, "boxNode2");
    SLNode* axisNode = new SLNode(new SLCoordAxis(), "axis node");
    SLBox*  box3     = new SLBox(0.0f, 0.0f, 0.0f, 1.745f, 0.745, 0.81, "Box 3", yellow);
    SLNode* boxNode3 = new SLNode(box3, "boxNode3");
    boxNode3->translate(2.561f, -5.147f, -0.06f);

    _root3D->addChild(boxNode1);
    _root3D->addChild(axisNode);
    _root3D->addChild(boxNode2);
    _root3D->addChild(boxNode3);
#endif

#if 0 // locsim scene
    SLBox*  box2     = new SLBox(-0.5f, -0.5f, -0.5f, 0.5f, 0.5f, 0.5f, "Box 2", yellow);
    SLNode* boxNode2 = new SLNode(box2, "boxNode2");
    boxNode2->translation(79.7f, -3.26f, 2.88f);
    boxNode2->scale(1.0f, 8.95f, 1.0f);
    boxNode2->rotate(1.39f, SLVec3f(1.0f, 0.0f, 0.0f), TS_parent);
    boxNode2->rotate(3.88f, SLVec3f(0.0f, 1.0f, 0.0f), TS_parent);
    boxNode2->rotate(-0.1f, SLVec3f(0.0f, 0.0f, 1.0f), TS_parent);

    SLBox*  box3     = new SLBox(-0.5f, -0.5f, -0.5f, 0.5f, 0.5f, 0.5f, "Box 3", yellow);
    SLNode* boxNode3 = new SLNode(box3, "boxNode3");
    boxNode3->translation(83.54f, -3.26f, 23.64f);
    boxNode3->scale(1.0f, 8.95f, 1.0f);
    boxNode3->rotate(1.39f, SLVec3f(1.0f, 0.0f, 0.0f), TS_parent);
    boxNode3->rotate(3.88f, SLVec3f(0.0f, 1.0f, 0.0f), TS_parent);
    boxNode3->rotate(-0.1f, SLVec3f(0.0f, 0.0f, 1.0f), TS_parent);

    SLBox*  box4     = new SLBox(0.0f, 0.0f, 0.0f, 1.0f, 1.0f, 21.11f, "Box 4", yellow);
    SLNode* boxNode4 = new SLNode(box4, "boxNode4");
    boxNode4->translation(79.38f, 0.74f, 3.0f);
    boxNode4->rotate(-0.19f, SLVec3f(1.0f, 0.0f, 0.0f), TS_parent);
    boxNode4->rotate(9.91f, SLVec3f(0.0f, 1.0f, 0.0f), TS_parent);
    boxNode4->rotate(-0.95f, SLVec3f(0.0f, 0.0f, 1.0f), TS_parent);

    _root3D->addChild(boxNode1);
    _root3D->addChild(boxNode2);
    _root3D->addChild(boxNode3);
    _root3D->addChild(boxNode4);
#endif
    Utils::log("LoadingTime", "model loading time: %f ms", t.elapsedTimeInMilliSec());

    //boxNode->addChild(axisNode);

    cameraNode->translation(0, 0, 0.1f);
    cameraNode->lookAt(0, 0, 0);
    //for tracking we have to use the field of view from calibration
    cameraNode->clipNear(0.001f);
    cameraNode->clipFar(1000000.0f); // Increase to infinity?
    cameraNode->setInitialState();

    mapNode->addChild(mapPC);
    mapNode->addChild(mapMatchedPC);
    mapNode->addChild(mapLocalPC);
    mapNode->addChild(mapMarkerCornerPC);
    mapNode->addChild(keyFrameNode);
    mapNode->addChild(covisibilityGraph);
    mapNode->addChild(spanningTree);
    mapNode->addChild(loopEdges);
    mapNode->addChild(cameraNode);

    mapNode->rotate(180, 1, 0, 0);

    //setup scene
    _root3D->addChild(mapNode);
}

void AppWAIScene::adjustAugmentationTransparency(float kt)
{
    if (augmentationRoot)
    {
        for (SLNode* child : augmentationRoot->children())
        {
            child->mesh()->mat()->kt(kt);
            child->mesh()->mat()->ambient(SLCol4f(0.3f, 0.3f, 0.3f));
            child->mesh()->init(child);
        }
    }
}

void AppWAIScene::resetMapNode()
{
    mapNode->translation(0, 0, 0);
    mapNode->lookAt(0, 0, -1);
}

void AppWAIScene::updateCameraPose(const cv::Mat& pose)
{
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

    cameraNode->om(om);
}

void AppWAIScene::updateVideoImage(const cv::Mat& image)
{
    _videoImage->copyVideoImage(image.cols,
                                image.rows,
                                CVImage::cv2glPixelFormat(image.type()),
                                image.data,
                                image.isContinuous(),
                                true);
}

void AppWAIScene::renderMapPoints(const std::vector<WAIMapPoint*>& pts)
{
    renderMapPoints("MapPoints", pts, mapPC, mappointsMesh, redMat);
}

void AppWAIScene::renderMarkerCornerMapPoints(const std::vector<WAIMapPoint*>& pts)
{
    renderMapPoints("MarkerCornerMapPoints", pts, mapMarkerCornerPC, mappointsMarkerCornerMesh, blueMat);
}

void AppWAIScene::renderLocalMapPoints(const std::vector<WAIMapPoint*>& pts)
{
    renderMapPoints("LocalMapPoints", pts, mapLocalPC, mappointsLocalMesh, greenMat);
}

void AppWAIScene::renderMatchedMapPoints(const std::vector<WAIMapPoint*>& pts, float opacity)
{
    renderMapPoints("MatchedMapPoints",
                    pts,
                    mapMatchedPC,
                    mappointsMatchedMesh,
                    blueMat,
                    opacity);
}

void AppWAIScene::removeMapPoints()
{
    removeMesh(mapPC, mappointsMesh);
}

void AppWAIScene::removeMarkerCornerMapPoints()
{
    removeMesh(mapMarkerCornerPC, mappointsMarkerCornerMesh);
}

void AppWAIScene::removeLocalMapPoints()
{
    removeMesh(mapLocalPC, mappointsLocalMesh);
}

void AppWAIScene::removeMatchedMapPoints()
{
    removeMesh(mapMatchedPC, mappointsMatchedMesh);
}

void AppWAIScene::renderKeyframes(const std::vector<WAIKeyFrame*>& keyframes, const std::vector<WAIKeyFrame*>& candidates)
{
    keyFrameNode->deleteChildren();
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

        cam->setDrawColor();
        for (WAIKeyFrame * ckf : candidates)
        {
            if (kf->mnId == ckf->mnId)
                cam->setDrawColor(SLCol4f::BLUE);
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
        cam->clipFar(1.0f);

        keyFrameNode->addChild(cam);
    }
}

void AppWAIScene::removeKeyframes()
{
    keyFrameNode->deleteChildren();
}

void AppWAIScene::renderGraphs(const std::vector<WAIKeyFrame*>& kfs,
                               const int&                       minNumOfCovisibles,
                               const bool                       showCovisibilityGraph,
                               const bool                       showSpanningTree,
                               const bool                       showLoopEdges)
{
    SLVVec3f covisGraphPts;
    SLVVec3f spanningTreePts;
    SLVVec3f loopEdgesPts;

    for (auto* kf : kfs)
    {
        cv::Mat Ow = kf->GetCameraCenter();

        //covisibility graph
        const std::vector<WAIKeyFrame*> vCovKFs = kf->GetBestCovisibilityKeyFrames(minNumOfCovisibles);

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

    removeGraphs();

    if (covisGraphPts.size() && showCovisibilityGraph)
    {
        covisibilityGraphMesh = new SLPolyline(&assets, covisGraphPts, false, "CovisibilityGraph", covisibilityGraphMat);
        covisibilityGraph->addMesh(covisibilityGraphMesh);
        covisibilityGraph->updateAABBRec();
    }

    if (spanningTreePts.size() && showSpanningTree)
    {
        spanningTreeMesh = new SLPolyline(&assets, spanningTreePts, false, "SpanningTree", spanningTreeMat);
        spanningTree->addMesh(spanningTreeMesh);
        spanningTree->updateAABBRec();
    }

    if (loopEdgesPts.size() && showLoopEdges)
    {
        loopEdgesMesh = new SLPolyline(&assets, loopEdgesPts, false, "LoopEdges", loopEdgesMat);
        loopEdges->addMesh(loopEdgesMesh);
        loopEdges->updateAABBRec();
    }
}

void AppWAIScene::removeGraphs()
{
    if (covisibilityGraphMesh)
    {
        if (covisibilityGraph->removeMesh(covisibilityGraphMesh))
        {
            assets.removeMesh(covisibilityGraphMesh);
            delete covisibilityGraphMesh;
            covisibilityGraphMesh = nullptr;
        }
    }
    if (spanningTreeMesh)
    {
        if (spanningTree->removeMesh(spanningTreeMesh))
        {
            assets.removeMesh(spanningTreeMesh);
            delete spanningTreeMesh;
            spanningTreeMesh = nullptr;
        }
    }
    if (loopEdgesMesh)
    {
        if (loopEdges->removeMesh(loopEdgesMesh))
        {
            assets.removeMesh(loopEdgesMesh);
            delete loopEdgesMesh;
            loopEdgesMesh = nullptr;
        }
    }
}

void AppWAIScene::renderMapPoints(std::string                      name,
                                  const std::vector<WAIMapPoint*>& pts,
                                  SLNode*&                         node,
                                  SLPoints*&                       mesh,
                                  SLMaterial*&                     material,
                                  float                            opacity)
{
    //remove old mesh, if it exists
    if (mesh)
    {
        if (node->removeMesh(mesh))
        {
            assets.removeMesh(mesh);
            delete mesh;
            mesh = nullptr;
        }
    }

    //instantiate and add new mesh
    if (pts.size())
    {
        //get points as Vec3f
        std::vector<SLVec3f> points, normals;
        for (auto mapPt : pts)
        {
            if (mapPt->isBad())
                continue;
            WAI::V3 wP = mapPt->worldPosVec();
            WAI::V3 wN = mapPt->normalVec();
            points.push_back(SLVec3f(wP.x, wP.y, wP.z));
            normals.push_back(SLVec3f(wN.x, wN.y, wN.z));
        }
        
        material->kt(1.f - opacity);

        mesh = new SLPoints(&assets, points, normals, name, material);
        node->addMesh(mesh);
        node->updateAABBRec();
    }
}

void AppWAIScene::removeMesh(SLNode* node, SLMesh* mesh)
{
    if (mesh)
    {
        if (node->removeMesh(mesh))
        {
            assets.removeMesh(mesh);
            delete mesh;
            mesh = nullptr;
        }
    }
}
