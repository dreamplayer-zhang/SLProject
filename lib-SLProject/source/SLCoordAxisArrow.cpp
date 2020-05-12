//#############################################################################
//  File:      SLCoordAxisArrow.cpp
//  Author:    Jan Dellsperger
//  Date:      April 2020
//  Codestyle: https://github.com/cpvrlab/SLProject/wiki/SLProject-Coding-Style
//  Copyright: Jan Dellsperger, Marcus Hudritsch
//             This software is provide under the GNU General Public License
//             Please visit: http://opensource.org/licenses/GPL-3.0
//#############################################################################


#include <SLCoordAxisArrow.h>

//-----------------------------------------------------------------------------
SLCoordAxisArrow::SLCoordAxisArrow(SLAssetManager* assetMgr,
                                   SLMaterial*     material,
                                   SLfloat         arrowThickness,
                                   SLfloat         arrowHeadLenght,
                                   SLfloat         arrowHeadWidth)
  : SLMesh(assetMgr, "Coord-Axis-Arrow Mesh")
{
    _arrowThickness  = arrowThickness;
    _arrowHeadLength = arrowHeadLenght;
    _arrowHeadWidth  = arrowHeadWidth;
    buildMesh(material);
}
//-----------------------------------------------------------------------------
void SLCoordAxisArrow::buildMesh(SLMaterial* material)
{
    // allocate new vectors of SLMesh
    P.clear();
    N.clear();
    C.clear();
    Tc.clear();
    I16.clear();

    mat(material);

    SLushort i = 0, v1 = 0, v2 = 0, v3 = 0, v4 = 0;
    SLfloat  t = _arrowThickness * 0.5f;
    SLfloat  h = _arrowHeadLength;
    SLfloat  w = _arrowHeadWidth * 0.5f;

    // clang-format off
    P.push_back(SLVec3f( t,  t, t)); v1=i; i++;
    P.push_back(SLVec3f( t,1-h, t)); v2=i; i++;
    P.push_back(SLVec3f( t,1-h,-t)); v3=i; i++;
    P.push_back(SLVec3f( t,  t,-t)); v4=i; i++;
    I16.push_back(v1); I16.push_back(v3); I16.push_back(v2);
    I16.push_back(v1); I16.push_back(v4); I16.push_back(v3);

    P.push_back(SLVec3f(-t,  t, t)); v1=i; i++;
    P.push_back(SLVec3f(-t,1-h, t)); v2=i; i++;
    P.push_back(SLVec3f( t,1-h, t)); v3=i; i++;
    P.push_back(SLVec3f( t,  t, t)); v4=i; i++;
    I16.push_back(v1); I16.push_back(v3); I16.push_back(v2);
    I16.push_back(v1); I16.push_back(v4); I16.push_back(v3);

    P.push_back(SLVec3f(-t,  t,-t)); v1=i; i++;
    P.push_back(SLVec3f(-t,1-h,-t)); v2=i; i++;
    P.push_back(SLVec3f(-t,1-h, t)); v3=i; i++;
    P.push_back(SLVec3f(-t,  t, t)); v4=i; i++;
    I16.push_back(v1); I16.push_back(v3); I16.push_back(v2);
    I16.push_back(v1); I16.push_back(v4); I16.push_back(v3);

    P.push_back(SLVec3f( t,  t,-t)); v1=i; i++;
    P.push_back(SLVec3f( t,1-h,-t)); v2=i; i++;
    P.push_back(SLVec3f(-t,1-h,-t)); v3=i; i++;
    P.push_back(SLVec3f(-t,  t,-t)); v4=i; i++;
    I16.push_back(v1); I16.push_back(v3); I16.push_back(v2);
    I16.push_back(v1); I16.push_back(v4); I16.push_back(v3);

    P.push_back(SLVec3f( w,1-h, w)); v1=i; i++;
    P.push_back(SLVec3f( 0,  1, 0)); v2=i; i++;
    P.push_back(SLVec3f( w,1-h,-w)); v3=i; i++;
    I16.push_back(v1); I16.push_back(v3); I16.push_back(v2);

    P.push_back(SLVec3f( w,1-h, w)); v1=i; i++;
    P.push_back(SLVec3f(-w,1-h, w)); v2=i; i++;
    P.push_back(SLVec3f( 0,  1, 0)); v3=i; i++;
    I16.push_back(v1); I16.push_back(v3); I16.push_back(v2);
    P.push_back(SLVec3f(-w,1-h, w)); v1=i; i++;
    P.push_back(SLVec3f(-w,1-h,-w)); v2=i; i++;
    P.push_back(SLVec3f( 0,  1, 0)); v3=i; i++;
    I16.push_back(v1); I16.push_back(v3); I16.push_back(v2);

    P.push_back(SLVec3f(-w,1-h,-w)); v1=i; i++;
    P.push_back(SLVec3f( w,1-h,-w)); v2=i; i++;
    P.push_back(SLVec3f( 0,  1, 0)); v3=i; i++;
    I16.push_back(v1); I16.push_back(v3); I16.push_back(v2);

    P.push_back(SLVec3f( w,1-h, w)); v1=i; i++;
    P.push_back(SLVec3f( w,1-h,-w)); v2=i; i++;
    P.push_back(SLVec3f(-w,1-h,-w)); v3=i; i++;
    P.push_back(SLVec3f(-w,1-h, w)); v4=i; i++;
    I16.push_back(v1); I16.push_back(v3); I16.push_back(v2);
    I16.push_back(v1); I16.push_back(v4); I16.push_back(v3);

    P.push_back(SLVec3f( t, -t, t)); v1=i; i++;
    P.push_back(SLVec3f( t, -t,-t)); v2=i; i++;
    P.push_back(SLVec3f(-t, -t,-t)); v3=i; i++;
    P.push_back(SLVec3f(-t, -t, t)); v4=i; i++;
    I16.push_back(v1); I16.push_back(v3); I16.push_back(v2);
    I16.push_back(v1); I16.push_back(v4); I16.push_back(v3);
}
//-----------------------------------------------------------------------------
