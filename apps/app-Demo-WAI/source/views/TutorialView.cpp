#include <views/TutorialView.h>

TutorialView::TutorialView(sm::EventHandler&   eventHandler,
                           SLInputManager&     inputManager,
                           const ImGuiEngine&  imGuiEngine,
                           ErlebAR::Resources& resources,
                           int                 screenWidth,
                           int                 screenHeight,
                           int                 dotsPerInch,
                           std::string         fontPath,
                           std::string         imguiIniPath,
                           std::string         texturePath)
  : SLSceneView(nullptr, dotsPerInch, inputManager),
    _gui(imGuiEngine, eventHandler, resources, dotsPerInch, screenWidth, screenHeight, fontPath, texturePath)
{
    init("TutorialView", screenWidth, screenHeight, nullptr, nullptr, &_gui, imguiIniPath);
    onInitialize();
}

bool TutorialView::update()
{
    return onPaint();
}
