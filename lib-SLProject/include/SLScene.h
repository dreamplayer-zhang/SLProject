//#############################################################################
//  File:      SLScene.h
//  Author:    Marcus Hudritsch
//  Date:      July 2014
//  Codestyle: https://github.com/cpvrlab/SLProject/wiki/SLProject-Coding-Style
//  Copyright: Marcus Hudritsch
//             This software is provide under the GNU General Public License
//             Please visit: http://opensource.org/licenses/GPL-3.0
//#############################################################################

#ifndef SLSCENE_H
#define SLSCENE_H

#include <SL.h>
#include <SLAnimManager.h>
#include <Averaged.h>
#include <SLEventHandler.h>
#include <SLGLOculus.h>
#include <SLLight.h>
#include <SLMaterial.h>
#include <SLMesh.h>
#include <SLRect.h>
#include <SLVec3.h>
#include <SLVec4.h>
#include <utility>
#include <vector>
#include <SLGLGenericProgram.h>
#include <SLAssetManager.h>

class SLSceneView;
class SLCamera;
class SLInputManager;

//-----------------------------------------------------------------------------
class SLGLDefaultPrograms
{
public:
private:
};
//-----------------------------------------------------------------------------

class SLGLColorUniformProgram : public SLGLGenericProgram
{
public:
    static SLGLColorUniformProgram* instance()
    {
        if (!_instance)
            _instance = new SLGLColorUniformProgram;
        return _instance;
    }
    static void deleteInstance()
    {
        if (_instance)
        {
            delete _instance;
            _instance = nullptr;
        }
    }

private:
    SLGLColorUniformProgram()
      : SLGLGenericProgram(nullptr, "ColorUniform.vert", "Color.frag")
    {
    }

    static SLGLColorUniformProgram* _instance;
};

//-----------------------------------------------------------------------------
//! Global default gray color material for meshes that don't define their own.
class SLMaterialDefaultGray : public SLMaterial
{
public:
    static SLMaterialDefaultGray* instance()
    {
        if (!_instance)
            _instance = new SLMaterialDefaultGray;
        return _instance;
    }
    static void deleteInstance()
    {
        if (_instance)
        {
            delete _instance;
            _instance = nullptr;
        }
    }

private:
    SLMaterialDefaultGray()
      : SLMaterial(nullptr, "default", SLVec4f::GRAY, SLVec4f::WHITE)
    {
        ambient({0.2f, 0.2f, 0.2f});
    }

    static SLMaterialDefaultGray* _instance;
};
//-----------------------------------------------------------------------------
//! Global diffuse reflection material for meshes with color vertex attributes.
class SLMaterialDiffuseAttribute : public SLMaterial
{
public:
    static SLMaterialDiffuseAttribute* instance()
    {
        if (!_instance)
            _instance = new SLMaterialDiffuseAttribute;
        return _instance;
    }
    static void deleteInstance()
    {
        if (_instance)
        {
            delete _instance;
            _instance = nullptr;
        }
    }

private:
    SLMaterialDiffuseAttribute()
      : SLMaterial(nullptr, "diffuseAttrib"),
        _program(nullptr, "PerVrtBlinnColorAttrib.vert", "PerVrtBlinn.frag")
    {
        specular(SLCol4f::BLACK);
        program(&_program);
    }

    SLGLGenericProgram                 _program;
    static SLMaterialDiffuseAttribute* _instance;
};

//-----------------------------------------------------------------------------
class SLGLTextureOnlyProgram : public SLGLGenericProgram
{
public:
    static SLGLTextureOnlyProgram* instance()
    {
        if (!_instance)
            _instance = new SLGLTextureOnlyProgram;
        return _instance;
    }
    static void deleteInstance()
    {
        if (_instance)
        {
            delete _instance;
            _instance = nullptr;
        }
    }

private:
    SLGLTextureOnlyProgram()
      : SLGLGenericProgram(nullptr, "TextureOnly.vert", "TextureOnly.frag")
    {
    }
    static SLGLTextureOnlyProgram* _instance;
};

//-----------------------------------------------------------------------------
typedef std::vector<SLSceneView*> SLVSceneView; //!< Vector of SceneView pointers
//-----------------------------------------------------------------------------
//! C-Callback function typedef for scene load function
typedef void(SL_STDCALL* cbOnSceneLoad)(SLScene* s, SLSceneView* sv, SLint sceneID);
//-----------------------------------------------------------------------------
//! The SLScene class represents the top level instance holding the scene structure
/*!      
 The SLScene class holds everything that is common for all scene views such as
 the root pointer (_root3D) to the scene, an array of lights as well as the
 global resources (_meshes (SLMesh), _materials (SLMaterial), _textures
 (SLGLTexture) and _shaderProgs (SLGLProgram)).
 All these resources and the scene with all nodes to which _root3D pointer points
 get deleted in the method unInit.\n
 A scene could have multiple scene views. A pointer of each is stored in the
 vector _sceneViews.\n
 The scene assembly takes place outside of the library in function of the application.
 A pointer for this function must be passed to the SLScene constructor. For the
 demo project this function is in AppDemoSceneLoad.cpp.
*/
class SLScene : public SLObject
{
    friend class SLNode;

public:
    SLScene(SLstring        name,
            cbOnSceneLoad   onSceneLoadCallback,
            SLInputManager& inputManager);
    ~SLScene();

    // Setters
    void root3D(SLNode* root3D) { _root3D = root3D; }
    void root2D(SLNode* root2D) { _root2D = root2D; }
    void globalAmbiLight(const SLCol4f& gloAmbi) { _globalAmbiLight = gloAmbi; }
    void stopAnimations(SLbool stop) { _stopAnimations = stop; }
    void info(SLstring i) { _info = std::move(i); }

    // Getters
    SLAnimManager&   animManager() { return _animManager; }
    SLSceneView*     sceneView(SLuint index) { return _sceneViews[index]; }
    SLVSceneView&    sceneViews() { return _sceneViews; }
    SLNode*          root3D() { return _root3D; }
    SLNode*          root2D() { return _root2D; }
    SLstring&        info() { return _info; }
    SLfloat          elapsedTimeMS() { return _frameTimeMS; }
    SLfloat          elapsedTimeSec() { return _frameTimeMS * 0.001f; }
    SLVEventHandler& eventHandlers() { return _eventHandlers; }

    SLCol4f   globalAmbiLight() const { return _globalAmbiLight; }
    SLVLight& lights() { return _lights; }
    SLfloat   fps() { return _fps; }
    AvgFloat& frameTimesMS() { return _frameTimesMS; }
    AvgFloat& updateTimesMS() { return _updateTimesMS; }
    AvgFloat& updateAnimTimesMS() { return _updateAnimTimesMS; }
    AvgFloat& updateAABBTimesMS() { return _updateAABBTimesMS; }
    AvgFloat& cullTimesMS() { return _cullTimesMS; }
    AvgFloat& draw2DTimesMS() { return _draw2DTimesMS; }
    AvgFloat& draw3DTimesMS() { return _draw3DTimesMS; }

    SLNode*   selectedNode() { return _selectedNode; }
    SLMesh*   selectedMesh() { return _selectedMesh; }
    SLRectf&  selectedRect() { return _selectedRect; }
    SLbool    stopAnimations() const { return _stopAnimations; }
    SLint     numSceneCameras();
    SLCamera* nextCameraInScene(SLSceneView* activeSV);

    cbOnSceneLoad onLoad; //!< C-Callback for scene load

    // Misc.
    bool         onUpdate();
    void         init();
    virtual void unInit();
    void         selectNode(SLNode* nodeToSelect);
    void         selectNodeMesh(SLNode* nodeToSelect, SLMesh* meshToSelect);

protected:
    SLVSceneView _sceneViews; //!< Vector of all sceneview pointers
    //SLVMesh      _meshes;     //!< Vector of all meshes

    SLVLight        _lights;        //!< Vector of all lights
    SLVEventHandler _eventHandlers; //!< Vector of all event handler
    SLAnimManager   _animManager;   //!< Animation manager instance

    SLNode*  _root3D;       //!< Root node for 3D scene
    SLNode*  _root2D;       //!< Root node for 2D scene displayed in ortho projection
    SLstring _info;         //!< scene info string
    SLNode*  _selectedNode; //!< Pointer to the selected node
    SLMesh*  _selectedMesh; //!< Pointer to the selected mesh
    SLRectf  _selectedRect; //!< Mouse selection rectangle

    SLCol4f _globalAmbiLight; //!< global ambient light intensity
    SLbool  _rootInitialized; //!< Flag if scene is initialized

    SLfloat _frameTimeMS;      //!< Last frame time in ms
    SLfloat _lastUpdateTimeMS; //!< Last time after update in ms
    SLfloat _fps;              //!< Averaged no. of frames per second

    // major part times
    AvgFloat _frameTimesMS;      //!< Averaged total time per frame in ms
    AvgFloat _updateTimesMS;     //!< Averaged time for update in ms
    AvgFloat _cullTimesMS;       //!< Averaged time for culling in ms
    AvgFloat _draw3DTimesMS;     //!< Averaged time for 3D drawing in ms
    AvgFloat _draw2DTimesMS;     //!< Averaged time for 2D drawing in ms
    AvgFloat _updateAABBTimesMS; //!< Averaged time for update the nodes AABB in ms
    AvgFloat _updateAnimTimesMS; //!< Averaged time for update the animations in ms

    SLbool _stopAnimations; //!< Global flag for stopping all animations

    SLInputManager& _inputManager;

    SLGLOculus _oculus; //!< Oculus Rift interface
};

//-----------------------------------------------------------------------------
class SLProjectScene : public SLScene
  , public SLAssetManager
{
public:
    SLProjectScene(SLstring name, cbOnSceneLoad onSceneLoadCallback, SLInputManager& inputManager);

    void        unInit() override;
    bool        deleteTexture(SLGLTexture* texture);
    SLGLOculus* oculus() { return &_oculus; }

    virtual void onLoadAsset(const SLstring& assetFile,
                             SLuint          processFlags);

private:
    SLint _numProgsPreload; //!< No. of preloaded shaderProgs
};
//-----------------------------------------------------------------------------

#endif
