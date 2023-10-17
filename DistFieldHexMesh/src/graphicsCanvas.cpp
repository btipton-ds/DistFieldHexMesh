#include <memory>
#include <GraphicsCanvas.h>

#ifdef __WXMAC__
#include <GLUT/glut.h>
#else
#include <GL/glu.h>
#endif

#ifndef WIN32
#include <unistd.h> // FIXME: ¿This work/necessary in Windows?
//Not necessary, but if it was, it needs to be replaced by process.h AND io.h
#endif

#include <OGLMultiVboHandlerTempl.h>
#include <OglShader.h>
#include <OglMath.h>
#include <cmath>

using namespace std;
using namespace DFHM;

BEGIN_EVENT_TABLE(GraphicsCanvas, wxGLCanvas)
EVT_PAINT(GraphicsCanvas::doPaint)
EVT_SIZE(GraphicsCanvas::onSize)
END_EVENT_TABLE()

namespace
{
    shared_ptr<wxGLContext> g_pContext;

    inline float toRad(float v)
    {
        return v * M_PI / 180.0f;
    }

#define OVER_SAMPLING 2

    int attribs[] = {
        WX_GL_DEPTH_SIZE, 16,
#if OVER_SAMPLING > 1
        WX_GL_SAMPLES, OVER_SAMPLING * OVER_SAMPLING,
#endif
        0
    };
}

GraphicsCanvas::GraphicsCanvas(wxFrame* parent)
    : wxGLCanvas(parent, wxID_ANY, attribs, wxDefaultPosition, wxDefaultSize, 0, wxT("GLCanvas"))
    , _faceVBO(GL_TRIANGLES, 10)
    , _edgeVBO(GL_LINES, 10)
{
    if (!g_pContext)
        g_pContext = make_shared<wxGLContext>(this);

    float lightAz[] = { 
        toRad(0.0f), 
        toRad(90 + 30.0f), 
    };
    float lightEl[] = { 
        toRad(0.0f), 
        toRad(30.0f),
    };
    
    _graphicsUBO.defColor = p3f(1.0f, 1.0f, 1);
    _graphicsUBO.ambient = 0.15f;
    _graphicsUBO.numLights = 1;
    _graphicsUBO.modelView = m44f().identity();
    _graphicsUBO.proj = m44f().identity();
    for (int i = 0; i < _graphicsUBO.numLights; i++) {
        float sinAz = sinf(lightAz[i]);
        float cosAz = cosf(lightAz[i]);
        float sinEl = sinf(lightEl[i]);
        float cosEl = cosf(lightEl[i]);

        _graphicsUBO.lightDir[i] = p3f(cosEl * sinAz, sinEl, cosEl * cosAz);
    }

    loadShaders();

    Bind(wxEVT_LEFT_DOWN, &GraphicsCanvas::onMouseLeftDown, this);
    Bind(wxEVT_LEFT_UP, &GraphicsCanvas::onMouseLeftUp, this);
    Bind(wxEVT_MIDDLE_DOWN, &GraphicsCanvas::onMouseMiddleDown, this);
    Bind(wxEVT_MIDDLE_UP, &GraphicsCanvas::onMouseMiddleUp, this);
    Bind(wxEVT_RIGHT_DOWN, &GraphicsCanvas::onMouseRightDown, this);
    Bind(wxEVT_RIGHT_UP, &GraphicsCanvas::onMouseRightUp, this);
    Bind(wxEVT_MOTION, &GraphicsCanvas::onMouseMove, this);
}

GraphicsCanvas::~GraphicsCanvas()
{
}

bool GraphicsCanvas::toggleShowSharpEdges()
{
    _showSharpEdges = !_showSharpEdges;

    return _showSharpEdges;
}

bool GraphicsCanvas::toggleShowTriNormals()
{
    _showTriNormals = !_showTriNormals;

    return _showTriNormals;
}

void GraphicsCanvas::onMouseLeftDown(wxMouseEvent& event)
{
    _mouseStartLoc = calMouseLoc(event.GetPosition());
    getViewEulerAnglesRad(_initAzRad, _initElRad);
    _leftDown = true;
}

void GraphicsCanvas::onMouseLeftUp(wxMouseEvent& event)
{
    _leftDown = false;
}

void GraphicsCanvas::onMouseMiddleDown(wxMouseEvent& event)
{
    _mouseStartLoc = calMouseLoc(event.GetPosition());
    getViewEulerAnglesRad(_initAzRad, _initElRad);
    _middleDown = true;
}

void GraphicsCanvas::onMouseMiddleUp(wxMouseEvent& event)
{
    _middleDown = false;
}

void GraphicsCanvas::onMouseRightDown(wxMouseEvent& event)
{
    _mouseStartLoc = calMouseLoc(event.GetPosition());
    getViewEulerAnglesRad(_initAzRad, _initElRad);
    _rightDown = true;
}

void GraphicsCanvas::onMouseRightUp(wxMouseEvent& event)
{
    _rightDown = false;
}

Eigen::Vector2d GraphicsCanvas::calMouseLoc(const wxPoint& pt)
{
    wxSize frameSize = GetSize();
    double x = pt.x / (double)frameSize.x;
    double y = pt.y / (double)frameSize.y;
    return Eigen::Vector2d(x, y);
}

void GraphicsCanvas::onMouseMove(wxMouseEvent& event)
{
    Eigen::Vector2d pos = calMouseLoc(event.GetPosition());
    Eigen::Vector2d delta;
    if (_leftDown) {
        delta = pos - _mouseStartLoc;
        double az = _initAzRad - delta[0] * M_PI / 2;
        double el = _initElRad - delta[1] * M_PI / 2;
        setViewEulerAnglesRad(az, el);
    } else if (_middleDown) {
        delta = pos - _mouseStartLoc;
        cout << "Mouse middle delta: " << delta[0] << ", " << delta[1] << "\n";
    } else if (_rightDown) {
        delta = pos - _mouseStartLoc;
        cout << "Mouse right delta: " << delta[0] << ", " << delta[1] << "\n";
    }
}

void GraphicsCanvas::doPaint(wxPaintEvent& WXUNUSED(event)) {
    render();
}

void GraphicsCanvas::onSize(wxSizeEvent& event)
{
    // Adjust the projection matrix to keep the pixel aspect ratio constant
    float ratio;
    Eigen::Matrix4f proj(Eigen::Matrix4f::Identity());
    if (event.m_size.x > event.m_size.y) {
        ratio = event.m_size.y / (float)event.m_size.x;
        proj(0, 0) = ratio;
        proj(1, 1) = 1;
    } else {
        ratio = event.m_size.x / (float)event.m_size.y;
        proj(0, 0) = 1;
        proj(1, 1) = ratio;

    }

    float* pf = _graphicsUBO.proj;
    for (int i = 0; i < 16; i++) {
        pf[i] = (float)proj(i);
    }
}


void GraphicsCanvas::glClearColor(GLclampf red, GLclampf green, GLclampf blue, GLclampf alpha)
{
    ::glClearColor(red, green, blue, alpha);
}

void GraphicsCanvas::loadShaders()
{
    SetCurrent(*g_pContext);
    string path = "shaders/";
    _phongShader = make_shared<COglShader>();

    _phongShader->setShaderVertexAttribName("inPosition");
    _phongShader->setShaderNormalAttribName("inNormal");
    _phongShader->setShaderColorAttribName("inColor");
//    _phongShader->setShaderTexParamAttribName("inPosition");

    _phongShader->setVertexSrcFile(path + "phong.vert");
    _phongShader->setFragmentSrcFile(path + "phong.frag");
    _phongShader->load();
    _faceVBO.setShader(_phongShader.get());
    _edgeVBO.setShader(_phongShader.get());
}

void GraphicsCanvas::glClearColor(const rgbaColor& color)
{
    ::glClearColor(
        color._rgba[0] / 255.0f,
        color._rgba[1] / 255.0f,
        color._rgba[2] / 255.0f,
        color._rgba[3] / 255.0f
    );
}

void GraphicsCanvas::render()
{
    SetCurrent(*g_pContext);

    updateView();
    _phongShader->bind();

    wxPaintDC(this);
    /*
layout(binding = 0) uniform UniformBufferObject {
    mat4 modelView;
    mat4 proj;
    vec3 lightDir[2];
    int numLights;
    float ambient;

} ubo;

    */
    const GLuint bindingPoint = 1;
    static GLuint vertUboIdx = -1;
    static GLint blockSize = -1;
    static GLuint buffer = -1;
    if (vertUboIdx == -1) {
        vertUboIdx = glGetUniformBlockIndex(_phongShader->programID(), "UniformBufferObject");
        glGetActiveUniformBlockiv(_phongShader->programID(), vertUboIdx, GL_UNIFORM_BLOCK_DATA_SIZE, &blockSize); COglShader::dumpGlErrors();
        glGenBuffers(1, &buffer);
    }

#if 0 && defined(_DEBUG)

    size_t cBlockSize = sizeof(GraphicsUBO);
    const GLchar* names[] = { "modelView", "proj", "defColor", "ambient", "numLights", "lightDir" };
    GLuint indices[6] = { 0, 0, 0, 0, 0 };
    glGetUniformIndices(_phongShader->programID(), 6, names, indices); COglShader::dumpGlErrors();

    GLint offset0[6];
    glGetActiveUniformsiv(_phongShader->programID(), 6, indices, GL_UNIFORM_OFFSET, offset0); COglShader::dumpGlErrors();

    GraphicsUBO testSize;
    size_t addr0 = (size_t) &testSize;
    GLint offset1[] = { 
        (GLint)(((size_t)&testSize.modelView) - addr0),
        (GLint)(((size_t)&testSize.proj) - addr0),
        (GLint)((GLint)((size_t)&testSize.defColor) - addr0),
        (GLint)(((size_t)&testSize.ambient) - addr0),
        (GLint)(((size_t)&testSize.numLights) - addr0),
        (GLint)((GLint)((size_t)&testSize.lightDir) - addr0),
    };
#endif

    glUniformBlockBinding(_phongShader->programID(), vertUboIdx, bindingPoint);

    glBindBuffer(GL_UNIFORM_BUFFER, buffer);
    glBufferData(GL_UNIFORM_BUFFER, sizeof(_graphicsUBO), &_graphicsUBO, GL_DYNAMIC_DRAW);
    glBindBufferBase(GL_UNIFORM_BUFFER, bindingPoint, buffer);

    glClearColor(_backColor);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

//    glEnable(GL_CULL_FACE);
//    glCullFace(GL_BACK);
    glViewport(0, 0, (GLint)GetSize().x, (GLint)GetSize().y);

    drawFaces();
    drawEdges();

    SwapBuffers();
    _phongShader->unBind();
}

void GraphicsCanvas::drawFaces()
{
    auto preDraw = [this](int key) -> COglMultiVBO::DrawVertexColorMode {
        _graphicsUBO.ambient = 0.2f;
        switch (key) {
            default:
            case 0:
                _graphicsUBO.defColor = p3f(1.0f, 1.0f, 1.0f);
                break;
            case 1:
                _graphicsUBO.defColor = p3f(0.0f, 1.0f, 0);
                break;
        }
        glBufferData(GL_UNIFORM_BUFFER, sizeof(_graphicsUBO), &_graphicsUBO, GL_DYNAMIC_DRAW);
        glEnable(GL_DEPTH_TEST);
        glDepthFunc(GL_LESS);

        if (_showSharpEdges) {
            glEnable(GL_POLYGON_OFFSET_FILL);
            glPolygonOffset(1.0f, 2.0f);
        }

        return COglMultiVBO::DrawVertexColorMode::DRAW_COLOR_NONE;
        };

    auto postDraw = [this]() {
        glDisable(GL_POLYGON_OFFSET_FILL);
        };

    auto preTexDraw = [this](int key) {
        };

    auto postTexDraw = [this]() {
        };

    _faceVBO.drawAllKeys(preDraw, postDraw, preTexDraw, postTexDraw);
}

void GraphicsCanvas::drawEdges()
{
    auto preDraw = [this](int key) -> COglMultiVBO::DrawVertexColorMode {
        switch (key) {
            default:
            case 0:
                if (!_showSharpEdges)
                    return COglMultiVBO::DrawVertexColorMode::DRAW_COLOR_SKIP;

                glLineWidth(2.0f);
                _graphicsUBO.defColor = p3f(1.0f, 0.0f, 0.0f);
                break;
            case 1:
                _graphicsUBO.defColor = p3f(0.0f, 1.0f, 0);
                break;
            case 2:
                if (!_showTriNormals)
                    return COglMultiVBO::DrawVertexColorMode::DRAW_COLOR_SKIP;

                glLineWidth(1.0f);
                _graphicsUBO.defColor = p3f(0.0f, 0.0f, 1.0f);
                break;
        }
        _graphicsUBO.ambient = 1.0f;
        glBufferData(GL_UNIFORM_BUFFER, sizeof(_graphicsUBO), &_graphicsUBO, GL_DYNAMIC_DRAW);
        glEnable(GL_DEPTH_TEST);
        glDepthFunc(GL_LESS);
        return COglMultiVBO::DrawVertexColorMode::DRAW_COLOR_NONE;
        };

    auto postDraw = [this]() {
        };

    auto preTexDraw = [this](int key) {
        };

    auto postTexDraw = [this]() {
        };

    _edgeVBO.drawAllKeys(preDraw, postDraw, preTexDraw, postTexDraw);
}

void GraphicsCanvas::updateView()
{
    Eigen::Matrix4d rot, rotZ, rotX, rotToGl, scale, pan, trans;
    
    rot.setIdentity();
    rotZ.setIdentity();
    rotX.setIdentity();
    rotToGl.setIdentity();
    scale.setIdentity();
    pan.setIdentity();
    trans.setIdentity();
    for (int i = 0; i < 3; i++) {
        scale(i, i) = _viewScale;
        pan(3, i) = _viewOrigin[i];
    }
    Eigen::Matrix4d panInv(pan.inverse());

    rotZ(0, 0) = cos(_viewAzRad);
    rotZ(0, 1) = -sin(_viewAzRad);
    rotZ(1, 0) = sin(_viewAzRad);
    rotZ(1, 1) = cos(_viewAzRad);

    rotX(1, 1) = cos(-_viewElRad);
    rotX(1, 2) = sin(-_viewElRad);
    rotX(2, 1) = -sin(-_viewElRad);
    rotX(2, 2) = cos(-_viewElRad);

    double xRotAngleRad = -90 * M_PI / 180;
    rotToGl(1, 1) = cos(xRotAngleRad);
    rotToGl(1, 2) = -sin(xRotAngleRad);
    rotToGl(2, 1) = sin(xRotAngleRad);
    rotToGl(2, 2) = cos(xRotAngleRad);

    trans *= rotToGl;
    trans *= panInv;
    trans *= rotX;
    trans *= rotZ;
    trans *= pan;
    trans *= pan;
    trans *= scale;

    float* pf = _graphicsUBO.modelView;
    for (int i = 0; i < 16; i++) {
        pf[i] = (float)trans(i);
    }
}
