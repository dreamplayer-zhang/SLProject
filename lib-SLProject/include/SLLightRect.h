//#############################################################################
//  File:      SLLightRect.h
//  Author:    Marcus Hudritsch
//  Date:      July 2014
//  Codestyle: https://github.com/cpvrlab/SLProject/wiki/SLProject-Coding-Style
//  Copyright: Marcus Hudritsch
//             This software is provide under the GNU General Public License
//             Please visit: http://opensource.org/licenses/GPL-3.0
//#############################################################################

#ifndef SLLIGHTRECT_H
#define SLLIGHTRECT_H

#include <SLLight.h>
#include <SLNode.h>

class SLSceneView;
class SLRay;
class SLAssetManager;
class SLScene;

//-----------------------------------------------------------------------------
//! Light node class for a rectangular light source
/*!
SLLightRect is a node that renders in OpenGL a light rectangle
object and applies the OpenGL light settings through the SLLight class.
The light rectangle is defined with its width and height and lies initially
centered in the x-y-plane. The light shines as a spotlight with 90 degrees
cutoff angle towards the negative z-axis.
If a light node is added to the scene it stays fix in the scene.\n
If a light node is added to the camera it moves with the camera.\n
See the scene examples for Per-Vertex-Blinn or Per-Pixel-Blinn lighting where
all light node types are used. \n
All light nodes inherited from SLLight work automatically together with the
following shaders: \n
  - PerVrtBlinn.vert, PerVrtBlinn.frag \n
  - PerVrtBlinnTex.vert, PerVrtBlinnTex.frag \n
  - PerPixBlinn.vert, PerPixBlinn.frag \n
  - PerPixBlinnTex.vert, PerPixBlinnTex.frag \n
*/
class SLLightRect : public SLNode
  , public SLLight
{
public:
    SLLightRect(SLAssetManager* assetMgr,
                SLScene*        s,
                SLfloat         width   = 1,
                SLfloat         height  = 1,
                SLbool          hasMesh = true);
    ~SLLightRect() override;

    void    init(SLScene* s);
    void    drawRec(SLSceneView* sv) override;
    bool    hitRec(SLRay* ray) override;
    void    statsRec(SLNodeStats& stats) override;
    void    drawMeshes(SLSceneView* sv) override;
    SLfloat shadowTest(SLRay*         ray,
                       const SLVec3f& L,
                       SLfloat        lightDist,
                       SLNode*        root3D) override;
    SLfloat shadowTestMC(SLRay*         ray,
                         const SLVec3f& L,
                         SLfloat        lightDist,
                         SLNode*        root3D) override;
    void    renderShadowMap(SLSceneView* sv, SLNode* root) override;

    // Setters
    void width(const SLfloat w)
    {
        _width     = w;
        _halfWidth = w * 0.5f;
    }
    void height(const SLfloat h)
    {
        _height     = h;
        _halfHeight = h * 0.5f;
    }
    void samples(SLVec2i samples);
    void samplesXY(SLint x, SLint y);

    // Getters
    SLfloat width() const { return _width; }
    SLfloat height() const { return _height; }
    SLVec4f positionWS() const override { return updateAndGetWM().translation(); }
    SLVec3f spotDirWS() override
    {
        return SLVec3f(_wm.m(8),
                       _wm.m(9),
                       _wm.m(10)) *
               -1.0;
    }

private:
    SLfloat _width;      //!< Width of square light in x direction
    SLfloat _height;     //!< Lenght of square light in y direction
    SLfloat _halfWidth;  //!< Half width of square light in x dir
    SLfloat _halfHeight; //!< Half height of square light in y dir
    SLVec2i _samples;    //!< Uneven NO. of samples in x and y dir
};
//-----------------------------------------------------------------------------
#endif
