#include <AppDemoGuiTrackedMapping.h>
#include <WAISlam.h>

//-----------------------------------------------------------------------------
const char* AppDemoGuiTrackedMapping::_currItem = NULL;
int         AppDemoGuiTrackedMapping::_currN    = -1;
//-----------------------------------------------------------------------------
AppDemoGuiTrackedMapping::AppDemoGuiTrackedMapping(string                        name,
                                                   bool*                         activator,
                                                   ImFont*                       font,
                                                   std::function<WAISlam*(void)> getModeCB)
  : AppDemoGuiInfosDialog(name, activator, font),
    _getMode(getModeCB)
{
}
//-----------------------------------------------------------------------------
void AppDemoGuiTrackedMapping::buildInfos(SLScene* s, SLSceneView* sv)
{
    ImGui::PushFont(_font);
    ImGui::Begin("Tracked Mapping", _activator /*, ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoResize*/);

    WAISlam* mode = _getMode();
    if (!mode)
    {
        ImGui::Text("SLAM not running.");
    }
    else
    {
        if (!mode)
            return;

        if (ImGui::Button("Reset", ImVec2(ImGui::GetContentRegionAvailWidth(), 0.0f)))
        {
            mode->reset();
        }

        /*
        if (ImGui::Button("Disable Mapping", ImVec2(ImGui::GetContentRegionAvailWidth(), 0.0f)))
        {
            mode->disableMapping();
        }
        if (ImGui::Button("Enable Mapping", ImVec2(ImGui::GetContentRegionAvailWidth(), 0.0f)))
        {
            mode->enableMapping();
        }
        */

        //add tracking state
        ImGui::Text("Tracking State : %s ", mode->getPrintableState().c_str());
        //add number of matches map points in current frame
        ImGui::Text("Num Map Matches: %d ", mode->getMapPointMatchesCount());
        //number of map points
        ImGui::Text("Num Map Pts: %d ", mode->getMapPointCount());
        //add number of keyframes
        ImGui::Text("Number of Keyframes : %d ", mode->getKeyFrameCount());
        //ImGui::InputInt("Min. frames before next KF", &mode->mMinFrames, 5, 0);

        //add loop closings counter
        ImGui::Text("Number of LoopClosings : %d ", mode->getLoopCloseCount());
        //ImGui::Text("Number of LoopClosings : %d ", _wai->mpLoopCloser->numOfLoopClosings());
        ImGui::Text("Loop closing status : %s ", mode->getLoopCloseStatus().c_str());
        ImGui::Text("Keyframes in Loop closing queue : %d", mode->getKeyFramesInLoopCloseQueueCount());

        /*
        //show 2D key points in video image
        bool b = mode->getTrackOptFlow();
        if (ImGui::Checkbox("Track optical flow", &b))
        {
            mode->setTrackOptFlow(b);
        }
        */
    }

    ImGui::End();
    ImGui::PopFont();
}
