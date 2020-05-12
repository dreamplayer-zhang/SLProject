//#############################################################################
//  File:      SLScene.cpp
//  Author:    Marcus Hudritsch
//  Date:      July 2014
//  Codestyle: https://github.com/cpvrlab/SLProject/wiki/SLProject-Coding-Style
//  Copyright: Marcus Hudritsch
//             This software is provide under the GNU General Public License
//             Please visit: http://opensource.org/licenses/GPL-3.0
//#############################################################################

#include <stdafx.h> // Must be the 1st include followed by  an empty line

#include <SLScene.h>
#include <Utils.h>
#include <SLKeyframeCamera.h>
#include <GlobalTimer.h>

//-----------------------------------------------------------------------------
SLMaterialDiffuseAttribute* SLMaterialDiffuseAttribute::_instance = nullptr;
SLMaterialDefaultGray*      SLMaterialDefaultGray::_instance      = nullptr;

//-----------------------------------------------------------------------------
/*! The constructor of the scene does all one time initialization such as
loading the standard shader programs from which the pointers are stored in
the vector _shaderProgs. Custom shader programs that are loaded in a
scene must be deleted when the scene changes.
The following standard shaders are preloaded:
  - ColorAttribute.vert, Color.frag
  - ColorUniform.vert, Color.frag
  - DiffuseAttribute.vert, Diffuse.frag
  - PerVrtBlinn.vert, PerVrtBlinn.frag
  - PerVrtBlinnTex.vert, PerVrtBlinnTex.frag
  - TextureOnly.vert, TextureOnly.frag
  - PerPixBlinn.vert, PerPixBlinn.frag
  - PerPixBlinnTex.vert, PerPixBlinnTex.frag
  - PerPixCookTorance.vert, PerPixCookTorance.frag
  - PerPixCookToranceTex.vert, PerPixCookToranceTex.frag
  - BumpNormal.vert, BumpNormal.frag
  - BumpNormal.vert, BumpNormalParallax.frag
  - FontTex.vert, FontTex.frag
  - StereoOculus.vert, StereoOculus.frag
  - StereoOculusDistortionMesh.vert, StereoOculusDistortionMesh.frag

There will be only one scene for an application and it gets constructed in
the C-interface function slCreateScene in SLInterface.cpp that is called by the
platform and UI-toolkit dependent window initialization.
As examples you can see it in:
  - app-Demo-SLProject/GLFW: glfwMain.cpp in function main()
  - app-Demo-SLProject/android: Java_ch_fhnw_comgRT_glES2Lib_onInit()
  - app-Demo-SLProject/iOS: ViewController.m in method viewDidLoad()
  - _old/app-Demo-Qt: qtGLWidget::initializeGL()
  - _old/app-Viewer-Qt: qtGLWidget::initializeGL()
*/
SLScene::SLScene(const SLstring& name,
                 cbOnSceneLoad   onSceneLoadCallback)
  : SLObject(name),
    _frameTimesMS(60, 0.0f),
    _updateTimesMS(60, 0.0f),
    _updateAABBTimesMS(60, 0.0f),
    _updateAnimTimesMS(60, 0.0f)
{
    onLoad = onSceneLoadCallback;

    _root3D           = nullptr;
    _root2D           = nullptr;
    _info             = "";
    _selectedMesh     = nullptr;
    _selectedNode     = nullptr;
    _stopAnimations   = false;
    _fps              = 0;
    _frameTimeMS      = 0;
    _lastUpdateTimeMS = 0;

    _oculus.init();
}
//-----------------------------------------------------------------------------
/*! The destructor does the final total deallocation of all global resources.
The destructor is called in slTerminate.
*/
SLScene::~SLScene()
{
    unInit();

    // delete global SLGLState instance
    SLGLState::deleteInstance();

    // clear light pointers
    _lights.clear();

    SL_LOG("Destructor      : ~SLScene");
    SL_LOG("------------------------------------------------------------------");
}
//-----------------------------------------------------------------------------
/*! The scene init is called before a new scene is assembled.
*/
void SLScene::init()
{
    unInit();

    // reset all states
    SLGLState::instance()->initAll();

    _globalAmbiLight.set(0.2f, 0.2f, 0.2f, 0.0f);
    _selectedNode = nullptr;

    // Reset timing variables
    _frameTimesMS.init(60, 0.0f);
    _updateTimesMS.init(60, 0.0f);
    _updateAnimTimesMS.init(60, 0.0f);
    _updateAABBTimesMS.init(60, 0.0f);
}
//-----------------------------------------------------------------------------
/*! The scene uninitializing clears the scenegraph (_root3D) and all global
global resources such as materials, textures & custom shaders loaded with the
scene. The standard shaders, the fonts and the 2D-GUI elements remain. They are
destructed at process end.
*/
void SLScene::unInit()
{
    _selectedMesh = nullptr;
    _selectedNode = nullptr;

    // delete entire scene graph
    delete _root3D;
    _root3D = nullptr;
    delete _root2D;
    _root2D = nullptr;

    // clear light pointers
    _lights.clear();

    _eventHandlers.clear();
    _animManager.clear();
}
//-----------------------------------------------------------------------------
//! Updates animations and AABBs
/*! Updates different updatables in the scene after all views got painted:
\n 1) Calculate frame time
\n 2) Update all animations
\n 3) Update AABBs
\n
\return true if really something got updated
*/
bool SLScene::onUpdate(bool renderTypeIsRT,
                       bool voxelsAreShown)
{
    /////////////////////////////
    // 1) Calculate frame time //
    /////////////////////////////

    // Calculate the elapsed time for the animation
    // todo: If slowdown on idle is enabled the delta time will be wrong!
    _frameTimeMS      = GlobalTimer::timeMS() - _lastUpdateTimeMS;
    _lastUpdateTimeMS = GlobalTimer::timeMS();

    // Calculate the frames per second metric
    _frameTimesMS.set(_frameTimeMS);
    SLfloat averagedFrameTimeMS = _frameTimesMS.average();
    if (averagedFrameTimeMS > 0.001f)
        _fps = 1 / _frameTimesMS.average() * 1000.0f;
    else
        _fps = 0.0f;

    SLfloat startUpdateMS = GlobalTimer::timeMS();

    SLbool sceneHasChanged = false;

    //////////////////////////////
    // 2) Update all animations //
    //////////////////////////////

    SLfloat startAnimUpdateMS = GlobalTimer::timeMS();

    if (_root3D)
        _root3D->updateRec();

    sceneHasChanged |= !_stopAnimations && _animManager.update(elapsedTimeSec());

    // Do software skinning on all changed skeletons. Update any out of date acceleration structure for RT or if they're being rendered.
    if (_root3D)
    {
        //we use a lambda to inform nodes that share a mesh that the mesh got updated (so we dont have to transfer the root node)
        sceneHasChanged |= _root3D->updateMeshSkins([&](SLMesh* mesh) {
            SLVNode nodes = _root3D->findChildren(mesh, true);
            for (auto* node : nodes)
                node->needAABBUpdate();
        });

        if (renderTypeIsRT || voxelsAreShown)
            _root3D->updateMeshAccelStructs();
    }

    _updateAnimTimesMS.set(GlobalTimer::timeMS() - startAnimUpdateMS);

    /////////////////////
    // 3) Update AABBs //
    /////////////////////

    // The updateAABBRec call won't generate any overhead if nothing changed
    SLfloat startAAABBUpdateMS = GlobalTimer::timeMS();
    SLNode::numWMUpdates       = 0;
    SLGLState::instance()->modelViewMatrix.identity();
    if (_root3D)
        _root3D->updateAABBRec();
    if (_root2D)
        _root2D->updateAABBRec();
    _updateAABBTimesMS.set(GlobalTimer::timeMS() - startAAABBUpdateMS);

    // Finish total updateRec time
    SLfloat updateTimeMS = GlobalTimer::timeMS() - startUpdateMS;
    _updateTimesMS.set(updateTimeMS);

    //SL_LOG("SLScene::onUpdate");
    return sceneHasChanged;
}
//-----------------------------------------------------------------------------
//! Sets the _selectedNode to the passed node and flags it as selected
/*! If one node is selected a rectangle selection is reset to zero.
The drawing of the selection is done in SLMesh::draw and SLAABBox::drawWS.
*/
void SLScene::selectNode(SLNode* nodeToSelect)
{
    if (_selectedNode)
        _selectedNode->drawBits()->off(SL_DB_SELECTED);

    if (nodeToSelect)
    {
        if (_selectedNode == nodeToSelect)
        {
            _selectedNode = nullptr;
        }
        else
        {
            _selectedNode = nodeToSelect;
            _selectedNode->drawBits()->on(SL_DB_SELECTED);
        }
    }
    else
        _selectedNode = nullptr;
    _selectedMesh = nullptr;
}
//-----------------------------------------------------------------------------
//! Sets the _selectedNode and _selectedMesh and flags it as selected
/*! If one node is selected a rectangle selection is reset to zero.
The drawing of the selection is done in SLMesh::draw and SLAABBox::drawWS.
*/
void SLScene::selectNodeMesh(SLNode* nodeToSelect,
                             SLMesh* meshToSelect)
{
    if (_selectedNode)
        _selectedNode->drawBits()->off(SL_DB_SELECTED);

    if (nodeToSelect)
    {
        if (_selectedNode == nodeToSelect && _selectedMesh == meshToSelect)
        {
            _selectedNode = nullptr;
            _selectedMesh = nullptr;
        }
        else
        {
            _selectedNode = nodeToSelect;
            _selectedMesh = meshToSelect;
            _selectedNode->drawBits()->on(SL_DB_SELECTED);
        }
    }
    else
    {
        _selectedNode = nullptr;
        _selectedMesh = nullptr;
    }
}
//-----------------------------------------------------------------------------
//! Returns the number of camera nodes in the scene
SLint SLScene::numSceneCameras()
{
    if (!_root3D) return 0;
    vector<SLCamera*> cams = _root3D->findChildren<SLCamera>();
    return (SLint)cams.size();
}
//-----------------------------------------------------------------------------
//! Returns the next camera in the scene if there is one
SLCamera* SLScene::nextCameraInScene(SLCamera* activeSVCam)
{
    if (!_root3D) return nullptr;

    vector<SLCamera*> cams = _root3D->findChildren<SLCamera>();

    if (cams.empty()) return nullptr;
    if (cams.size() == 1) return cams[0];

    SLint activeIndex = 0;
    for (SLulong i = 0; i < cams.size(); ++i)
    {
        if (cams[i] == activeSVCam)
        {
            activeIndex = (SLint)i;
            break;
        }
    }

    // find next camera, that is not of type SLKeyframeCamera
    // and if allowAsActiveCam is deactivated
    do
    {
        activeIndex = activeIndex > cams.size() - 2 ? 0 : ++activeIndex;
    } while (dynamic_cast<SLKeyframeCamera*>(cams[(uint)activeIndex]) &&
             !dynamic_cast<SLKeyframeCamera*>(cams[(uint)activeIndex])->allowAsActiveCam());

    return cams[(uint)activeIndex];
}
//-----------------------------------------------------------------------------
