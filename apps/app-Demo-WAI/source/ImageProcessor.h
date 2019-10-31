#ifndef IMAGE_PROCESSOR
#define IMAGE_PROCESSOR

#include <SLSceneView.h>
#include <SLPoints.h>
#include <SLPolyline.h>
#include <CVCalibration.h>

class ImageProcessor
{
public:

    GLuint yuv422Converter;
    GLuint RGBTexture;

    GLuint d2Gdx2;
    GLuint d2Gdy2;
    GLuint dGdx;
    GLuint dGdy;
    GLuint Gx;
    GLuint Gy;
    GLuint detH;
    GLuint nmsx;
    GLuint nmsy;
    GLuint nmsz;

    GLuint d2Gdx2WLoc;
    GLuint d2Gdy2WLoc;
    GLuint dGdxWLoc;
    GLuint dGdyWLoc;
    GLuint GxWLoc;
    GLuint GyWLoc;
    GLuint nmsxWLoc;
    GLuint nmsyWLoc;
    GLuint d2Gdx2TexLoc;
    GLuint d2Gdy2TexLoc;
    GLuint dGdxTexLoc;
    GLuint dGdyTexLoc;
    GLuint GxTexLoc;
    GLuint GyTexLoc;
    GLuint Gx1chTexLoc;
    GLuint Gy1chTexLoc;
    GLuint detHGxxLoc;
    GLuint detHGyyLoc;
    GLuint detHGxyLoc;
    GLuint nmsxTexLoc;
    GLuint nmsyTexLoc;
    GLuint nmszTexLoc;
    GLuint nmszGxxLoc;
    GLuint nmszGyyLoc;
    GLuint nmszGxyLoc;

    GLuint renderTextures[10];
    GLuint renderFBO[10];

    GLuint vao;
    GLuint vbo;
    GLuint vboi;
    GLuint KpSSBO;
    GLuint patternSSBO;
    GLuint outFBO;
    GLuint outTextures;
    GLuint pbo[2];
    int curr;
    int ready;
    int m_w, m_h;

    ~ImageProcessor();
    ImageProcessor();
    ImageProcessor(int w, int h);
    void init(int w, int h);
    void initShaders();
    void initVBO();
    void initTextureBuffers(int width, int height);
    void initFBO();
    void setTextureParameters();
    void textureRGBF(int w, int h);
    void textureRF(int w, int h);
    void textureRB(int w, int h);
    GLuint buildShaderFromSource(string source, GLenum shaderType);

    void gxx(int w, int h);
    void gyy(int w, int h);
    void gxy(int w, int h);
    void det(int w, int h);
    void nms(int w, int h);
    void gpu_kp();
    void readResult(SLGLTexture * tex);
    void readResult(cv::Mat * tex);

    void initComputeShader();
    void computeBRIEF();
};

#endif
