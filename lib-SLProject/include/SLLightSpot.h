//#############################################################################
//  File:      SLLightSpot.h
//  Author:    Marcus Hudritsch
//  Date:      July 2014
//  Codestyle: https://github.com/cpvrlab/SLProject/wiki/SLProject-Coding-Style
//  Copyright: Marcus Hudritsch
//             This software is provide under the GNU General Public License
//             Please visit: http://opensource.org/licenses/GPL-3.0
//#############################################################################

#ifndef SLLIGHTSPHERE_H
#define SLLIGHTSPHERE_H

#include <SLLight.h>
#include <SLNode.h>
#include <SLSamples2D.h>

class SLSceneView;
class SLRay;
class SLScene;

//-----------------------------------------------------------------------------
//! SLLightSpot class for a spot light source
/*!      
SLLightSpot is a node and a light that can have a spot mesh for its 
representation.
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
class SLLightSpot : public SLNode
  , public SLLight
{
public:
    explicit SLLightSpot(SLAssetManager* assetMgr,
                         SLScene*        s,
                         SLfloat         radius       = 0.3f,
                         SLfloat         spotAngleDEG = 180.0f,
                         SLbool          hasMesh      = true);
    SLLightSpot(SLAssetManager* assetMgr,
                SLScene*        s,
                SLfloat         posx,
                SLfloat         posy,
                SLfloat         posz,
                SLfloat         radius       = 0.3f,
                SLfloat         spotAngleDEG = 180.0f,
                SLfloat         ambiPower    = 1.0f,
                SLfloat         diffPower    = 10.0f,
                SLfloat         specPower    = 10.0f,
                SLbool          hasMesh      = true);

    void    init(SLScene* s);
    bool    hitRec(SLRay* ray) override;
    void    statsRec(SLNodeStats& stats) override;
    void    drawMeshes(SLSceneView* sv) override;
    void    setState() override;
    SLfloat shadowTest(SLRay*         ray,
                       const SLVec3f& L,
                       SLfloat        lightDist,
                       SLNode*        root3D) override;
    SLfloat shadowTestMC(SLRay*         ray,
                         const SLVec3f& L,
                         SLfloat        lightDist,
                         SLNode*        root3D) override;

    // Setters
    void samples(SLuint x, SLuint y) { _samples.samples(x, y, false); }

    // Getters
    SLfloat radius() { return _radius; }
    SLuint  samples() { return _samples.samples(); }
    SLVec4f positionWS() const override { return translationWS(); }
    SLVec3f spotDirWS() override { return forwardWS(); }

#ifdef SL_HAS_OPTIX
    Light optixLight(bool doDistributed)
    {
        Samples loc_samples{};
        float   loc_radius;
        if (doDistributed)
        {
            loc_samples.samplesX = _samples.samplesX();
            loc_samples.samplesY = _samples.samplesY();
            loc_radius           = radius();
        }
        else
        {
            loc_samples = {
              1,
              1};
            loc_radius = 0.0f;
        }
        return {
          make_float4(diffuse()),
          make_float4(ambient()),
          make_float4(specular()),
          make_float3({positionWS().x, positionWS().y, positionWS().z}),
          spotCutOffDEG(),
          spotExponent(),
          spotCosCut(),
          make_float3(spotDirWS()),
          kc(),
          kl(),
          kq(),
          loc_samples,
          loc_radius};
    }
#endif

private:
    SLfloat     _radius;  //!< The sphere lights radius
    SLSamples2D _samples; //!< 2D samplepoints for soft shadows
};
//-----------------------------------------------------------------------------
#endif
