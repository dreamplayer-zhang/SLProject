//#############################################################################
//  File:      AppNodeLoad.cpp
//  Author:    Marc Wacker
//  Date:      July 2015
//  Codestyle: https://github.com/cpvrlab/SLProject/wiki/SLProject-Coding-Style
//  Copyright: Marcus Hudritsch
//             This software is provide under the GNU General Public License
//             Please visit: http://opensource.org/licenses/GPL-3.0
//#############################################################################

#include <SLScene.h>
#include <SLSceneView.h>
#include <SLCamera.h>
#include <SLLightSpot.h>
#include <SLProjectScene.h>

//-----------------------------------------------------------------------------
//! appNodeLoadScene builds a scene from source code.
/*! appDemoLoadScene builds a scene from source code. Such a function must be
 passed as a void*-pointer to slCreateScene. It will be called from within
 slCreateSceneView as soon as the view is initialized. You could separate
 different scene by a different sceneID.<br>
 The purpose is to assemble a scene by creating scenegraph objects with nodes
 (SLNode) and meshes (SLMesh). See the scene with SID_Minimal for a minimal
 example of the different steps.
 */
void appNodeLoadScene(SLProjectScene* s, SLSceneView* sv, SLSceneID sid)
{
    s->init();

    SLCamera* cam1 = new SLCamera;
    cam1->translation(2, 3, 5);
    cam1->lookAt(-2, -1.0, 1);
    cam1->focalDist(6);
    cam1->background().colors(SLCol4f(0.8f, 0.8f, 0.8f));

    SLLightSpot* light1 = new SLLightSpot(s, s, 0.3f);
    light1->translation(10, 10, 10);

    SLNode* scene = new SLNode;
    scene->addChild(light1);
    scene->addChild(cam1);

    s->root3D(scene);

    sv->camera(cam1);
    sv->doWaitOnIdle(false);
    sv->onInitialize();
}
//-----------------------------------------------------------------------------
