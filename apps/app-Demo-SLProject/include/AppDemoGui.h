//#############################################################################
//  File:      AppDemoGui.h
//  Author:    Marcus Hudritsch
//  Date:      Summer 2017
//  Codestyle: https://github.com/cpvrlab/SLProject/wiki/SLProject-Coding-Style
//  Copyright: Marcus Hudritsch
//             This software is provide under the GNU General Public License
//             Please visit: http://opensource.org/licenses/GPL-3.0
//#############################################################################

#ifndef SLGUIDEMO_H
#define SLGUIDEMO_H

#include <SL.h>
#include <SLTransformNode.h>

class SLScene;
class SLSceneView;
class SLNode;
class SLGLTexture;
class SLProjectScene;

//-----------------------------------------------------------------------------
//! ImGui UI class for the UI of the demo applications
/* The UI is completely built within this class by calling build function
AppDemoGui::build. This build function is passed in the slCreateSceneView and
it is called in SLSceneView::onPaint in every frame.<br>
The entire UI is configured and built on every frame. That is why it is called
"Im" for immediate. See also the SLGLImGui class to see how it minimaly
integrated in the SLProject.<br>
*/
class AppDemoGui
{
public:
    static void build(SLProjectScene* s, SLSceneView* sv);
    static void buildMenuBar(SLProjectScene* s, SLSceneView* sv);
    static void buildSceneGraph(SLScene* s);
    static void addSceneGraphNode(SLScene* s, SLNode* node);
    static void buildProperties(SLScene* s, SLSceneView* sv);
    static void loadConfig(SLint dotsPerInch);
    static void saveConfig();
    static void showLUTColors(SLColorLUT* lut);

    static SLstring    configTime;          //!< Time of stored configuration
    static SLstring    infoAbout;           //!< About info string
    static SLstring    infoCredits;         //!< Credits info string
    static SLstring    infoHelp;            //!< Help info string
    static SLstring    infoCalibrate;       //!< Calibration info string
    static SLbool      showProgress;        //!< Flag if about info should be shown
    static SLbool      showDockSpace;       //!< Flag if dock space should be enabled
    static SLbool      showAbout;           //!< Flag if about info should be shown
    static SLbool      showHelp;            //!< Flag if help info should be shown
    static SLbool      showHelpCalibration; //!< Flag if calibration info should be shown
    static SLbool      showCredits;         //!< Flag if credits info should be shown
    static SLbool      showStatsTiming;     //!< Flag if timing info should be shown
    static SLbool      showStatsScene;      //!< Flag if scene info should be shown
    static SLbool      showStatsVideo;      //!< Flag if video info should be shown
    static SLbool      showStatsWAI;        //!< Flag if WAI info should be shown
    static SLbool      showImGuiMetrics;    //!< Flag if imgui metrics infor should be shown
    static SLbool      showInfosSensors;    //!< Flag if device sensors info should be shown
    static SLbool      showInfosDevice;     //!< Flag if device info should be shown
    static SLbool      showInfosScene;      //!< Flag if scene info should be shown
    static SLbool      showSceneGraph;      //!< Flag if scene graph should be shown
    static SLbool      showProperties;      //!< Flag if properties should be shown
    static SLbool      showChristoffel;     //!< Flag if Christoffel infos should be shown
    static SLbool      showUIPrefs;         //!< Flag if UI preferences
    static SLbool      showTransform;       //!< Flag if tranform dialog should be shown
    static std::time_t adjustedTime;        //!< Adjusted GUI time for sun setting (default 0)

private:
    static void setTransformEditMode(SLProjectScene* s,
                                     SLSceneView*    sv,
                                     SLNodeEditMode  editMode);
    static void removeTransformNode(SLProjectScene* s);
};
//-----------------------------------------------------------------------------
#endif
