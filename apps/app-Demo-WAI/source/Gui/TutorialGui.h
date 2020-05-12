#ifndef TUTORIAL_GUI_H
#define TUTORIAL_GUI_H

#include <string>

#include <ImGuiWrapper.h>
#include <sm/EventSender.h>
#include <ErlebAR.h>
#include <Resources.h>

class SLScene;
class SLSceneView;
struct ImFont;

class TutorialGui : public ImGuiWrapper
  , private sm::EventSender
{
public:
    TutorialGui(const ImGuiEngine&  imGuiEngine,
                sm::EventHandler&   eventHandler,
                ErlebAR::Resources& resources,
                int                 dotsPerInch,
                int                 screenWidthPix,
                int                 screenHeightPix,
                std::string         fontPath,
                std::string         texturePath);
    ~TutorialGui();

    void build(SLScene* s, SLSceneView* sv) override;
    void onResize(SLint scrW, SLint scrH, SLfloat scr2fbX, SLfloat scr2fbY) override;
    void onShow(); //call when gui becomes visible

private:
    void resize(int scrW, int scrH);

    float _screenW;
    float _screenH;
    float _headerBarH;
    float _contentH;
    float _contentStartY;
    float _spacingBackButtonToText;
    float _buttonRounding;
    float _textWrapW;
    float _windowPaddingContent;
    float _itemSpacingContent;

    ErlebAR::Resources& _resources;

    GLuint _textureBackgroundId1 = 0;
    GLuint _textureBackgroundId2 = 0;
    GLuint _currentBackgroundId  = 0;
    GLuint _textureIconLeftId    = 0;
    GLuint _textureIconRightId   = 0;
};

#endif //TUTORIAL_GUI_H
