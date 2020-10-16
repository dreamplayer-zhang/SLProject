//#############################################################################
//  File:      SLLight.cpp
//  Author:    Marcus Hudritsch
//  Date:      July 2014
//  Codestyle: https://github.com/cpvrlab/SLProject/wiki/SLProject-Coding-Style
//  Copyright: Marcus Hudritsch
//             This software is provide under the GNU General Public License
//             Please visit: http://opensource.org/licenses/GPL-3.0
//#############################################################################

#include <stdafx.h> // Must be the 1st include followed by  an empty line

#include <SLLight.h>
#include <SLShadowMap.h>

//-----------------------------------------------------------------------------
SLCol4f SLLight::globalAmbient = SLCol4f(0.1f, 0.1f, 0.1f, 1.0f);
SLfloat SLLight::gamma         = 1.0f;
//-----------------------------------------------------------------------------
SLLight::SLLight(SLfloat ambiPower,
                 SLfloat diffPower,
                 SLfloat specPower,
                 SLint   id)
{
    // Set parameter of SLLight
    _id                = id;
    _isOn              = true;
    _spotCutOffDEG     = 180.0f;
    _spotCosCutOffRAD  = cos(Utils::DEG2RAD * _spotCutOffDEG);
    _spotExponent      = 1.0f;
    _createsShadows    = false;
    _shadowMap         = nullptr;
    _doSmoothShadows   = false;
    _smoothShadowLevel = 1;
    _shadowBias        = 0.005f;

    // Set parameters of inherited SLMaterial
    _ambientColor.set(1, 1, 1);
    _ambientPower = ambiPower;
    _diffuseColor.set(1, 1, 1);
    _diffusePower = diffPower;
    _specularColor.set(1, 1, 1);
    _specularPower = specPower;

    // By default there is no attenuation set. This is physically not correct
    // Default OpenGL:      kc=1, kl=0, kq=0
    // Physically correct:  kc=0, kl=0, kq=1
    // set quadratic attenuation with d = distance to light
    //                      1
    // attenuation = ------------------
    //               kc + kl*d + kq*d*d
    kc(1.0f);
    kl(0.0f);
    kq(0.0f);
}
//-----------------------------------------------------------------------------
void SLLight::kc(SLfloat kc)
{
    _kc           = kc;
    _isAttenuated = !(_kc == 1.0f && _kl == 0.0f && _kq == 0.0f);
}
//-----------------------------------------------------------------------------
void SLLight::kl(SLfloat kl)
{
    _kl           = kl;
    _isAttenuated = !(_kc == 1.0f && _kl == 0.0f && _kq == 0.0f);
}
//-----------------------------------------------------------------------------
void SLLight::kq(SLfloat kq)
{
    _kq           = kq;
    _isAttenuated = !(_kc == 1.0f && _kl == 0.0f && _kq == 0.0f);
}
//-----------------------------------------------------------------------------
void SLLight::spotCutOffDEG(const SLfloat cutOffAngleDEG)
{
    _spotCutOffDEG    = cutOffAngleDEG;
    _spotCosCutOffRAD = cos(Utils::DEG2RAD * _spotCutOffDEG);
}
//-----------------------------------------------------------------------------
void SLLight::createsShadows(SLbool createsShadows)
{
    _createsShadows = createsShadows;
    if (!createsShadows)
    {
        delete _shadowMap;
        _shadowMap = nullptr;
    }
}
//-----------------------------------------------------------------------------
//! SLLight::renderShadowMap renders the shadow map of the light
void SLLight::renderShadowMap(SLSceneView* sv, SLNode* root)
{
    assert(_shadowMap && "No shadow map was created!");

    _shadowMap->render(sv, root);
}
//-----------------------------------------------------------------------------
