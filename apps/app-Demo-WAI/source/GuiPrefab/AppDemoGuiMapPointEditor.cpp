//#############################################################################
//  File:      AppDemoGuiInfosMapNodeTransform.cpp
//  Author:    Michael Goettlicher, Jan Dellsperger
//  Date:      September 2018
//  Codestyle: https://github.com/cpvrlab/SLProject/wiki/Coding-Style-Guidelines
//  Copyright: Marcus Hudritsch
//             This software is provide under the GNU General Public License
//             Please visit: http://opensource.org/licenses/GPL-3.0
//#############################################################################

#include <stdafx.h>
#include <AppDemoGuiMapPointEditor.h>

#include <imgui.h>
#include <imgui_internal.h>

#include <SLScene.h>
#include <SLSceneView.h>
#include <WAIEvent.h>
//-----------------------------------------------------------------------------
AppDemoGuiMapPointEditor::AppDemoGuiMapPointEditor(std::string            name,
                                                   bool*                  activator,
                                                   std::queue<WAIEvent*>* eventQueue,
                                                   ImFont*                font)
  : AppDemoGuiInfosDialog(name, activator, font),
    _eventQueue(eventQueue)
{
}

//-----------------------------------------------------------------------------
void AppDemoGuiMapPointEditor::buildInfos(SLScene* s, SLSceneView* sv)
{
    ImGui::Begin("Map Point editor");
    ImGui::PushFont(_font);

    if (ImGui::Button("Enter edit mode"))
    {
        WAIEventEnterEditMapPointMode* event = new WAIEventEnterEditMapPointMode();
        event->action = MapPointEditor_EnterEditMode;

        _eventQueue->push(event);
    }

    if (ImGui::Button("Exit edit mode"))
    {
        WAIEventEnterEditMapPointMode* event = new WAIEventEnterEditMapPointMode();
        event->action = MapPointEditor_Quit;

        _eventQueue->push(event);
    }

    if (ImGui::Button("Save map"))
    {
        WAIEventEnterEditMapPointMode* event = new WAIEventEnterEditMapPointMode();
        event->action = MapPointEditor_SaveInMap;

        _eventQueue->push(event);
    }

    if (ImGui::Button("Select single video matched"))
    {
        WAIEventEnterEditMapPointMode* event = new WAIEventEnterEditMapPointMode();
        event->action = MapPointEditor_SelectSingleVideo;

        _eventQueue->push(event);
    }


    ImGui::PopFont();
    ImGui::End();
}
