//#############################################################################
//  File:      SLImGuiInfosTracking.cpp
//  Author:    Jan Dellsperger
//  Date:      Mai 2018
//  Codestyle: https://github.com/cpvrlab/SLProject/wiki/Coding-Style-Guidelines
//  Copyright: Marcus Hudritsch
//             This software is provide under the GNU General Public License
//             Please visit: http://opensource.org/licenses/GPL-3.0
//#############################################################################

#include <stdafx.h>
#include <imgui.h>
#include <imgui_internal.h>
#include <SLCVStateEstimator.h>
#include <SLImGuiInfosCameraMovement.h>
#include <SLImGuiInfosDialog.h>
#include <SLTrackingInfosInterface.h>

SLImGuiInfosCameraMovement::SLImGuiInfosCameraMovement(std::string name,
                                                       SLCVStateEstimator* stateEstimator)
    : SLImGuiInfosDialog(name),
      _stateEstimator(stateEstimator)
{
}

void SLImGuiInfosCameraMovement::buildInfos()
{
    SLMat4f pose = _stateEstimator->getPose();
    SLVec3f t = pose.translation();
    SLVec3f r;
    pose.toEulerAnglesZYX(r.z, r.y, r.x);

    tX[recordIndex] = t.x;
    tY[recordIndex] = t.y;
    tZ[recordIndex] = t.z;
    rX[recordIndex] = r.x;
    rY[recordIndex] = r.y;
    rZ[recordIndex] = r.z;
    recordIndex = (recordIndex + 1) % MAX_CAM_MOVEMENT_RECORD_COUNT;

    SLVec3f dP = _stateEstimator->dP();
    SLVec3f dR = _stateEstimator->dR();
    SLint64 dT = _stateEstimator->dT() / 1000;
    SLint64 dTc = _stateEstimator->dTc() / 1000;
    
    ImGui::Text("translation : x - %.2f, y - %.2f, z - %.2f", t.x, t.y, t.z);
    ImGui::Text("rotation : x - %.2f, y - %.2f, z - %.2f", r.x, r.y, r.z);
    ImGui::Text("dP : x - %.8f, y - %.8f, z - %.8f", dP.x, dP.y, dP.z);
    ImGui::Text("dR : x - %.8f, y - %.8f, z - %.8f", dR.x, dR.y, dR.z);
    ImGui::Text("dT : %i, dTc: %i", dT, dTc);
    ImGui::PlotLines("translation x", tX, IM_ARRAYSIZE(tX));
    ImGui::PlotLines("translation y", tY, IM_ARRAYSIZE(tY));
    ImGui::PlotLines("translation z", tZ, IM_ARRAYSIZE(tZ));
    ImGui::PlotLines("rotation x", rX, IM_ARRAYSIZE(rX));
    ImGui::PlotLines("rotation y", rY, IM_ARRAYSIZE(rY));
    ImGui::PlotLines("rotation z", rZ, IM_ARRAYSIZE(rZ));
}
