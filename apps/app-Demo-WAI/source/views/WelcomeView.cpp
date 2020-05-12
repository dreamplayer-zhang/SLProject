#include <views/WelcomeView.h>

WelcomeView::WelcomeView(SLInputManager&     inputManager,
                         ErlebAR::Resources& resources,
                         const ImGuiEngine&  imGuiEngine,
                         int                 screenWidth,
                         int                 screenHeight,
                         int                 dotsPerInch,
                         std::string         fontPath,
                         std::string         texturePath,
                         std::string         imguiIniPath,
                         std::string         version)
  : SLSceneView(nullptr, dotsPerInch, inputManager),
    _gui(imGuiEngine, resources, dotsPerInch, screenWidth, screenHeight, fontPath, texturePath, version)
{
    init("WelcomeView", screenWidth, screenHeight, nullptr, nullptr, &_gui, imguiIniPath);
    onInitialize();
}

bool WelcomeView::update()
{
    return onPaint();
}
