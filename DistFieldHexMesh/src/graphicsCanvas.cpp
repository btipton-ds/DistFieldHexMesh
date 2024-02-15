#include <memory>
#include <graphicsCanvas.h>

#ifdef WIN32
#include <Windows.h>
#include <GL/gl.h>
#include <GL/glext.h>
#include <GL/wglext.h>
#else
#include "/usr/include/GL/gl.h"
#include "/usr/include/GL/glext.h"
#endif

#ifdef __WXMAC__
#include <GLUT/glut.h>
#else
#include <GL/glu.h>
#endif

#ifndef WIN32
#include <unistd.h> // FIXME: �This work/necessary in Windows?
//Not necessary, but if it was, it needs to be replaced by process.h AND io.h
#endif

#include <cmath>
#include <Eigen/Geometry>
#include <Eigen/src/Core/Matrix.h>
#include <OGLMultiVboHandlerTempl.h>
#include <OGLShader.h>
#include <OGLMath.h>
#include <triMesh.h>
#include <volume.h>
#include <appData.h>

using namespace std;
using namespace DFHM;

BEGIN_EVENT_TABLE(GraphicsCanvas, wxGLCanvas)
EVT_PAINT(GraphicsCanvas::doPaint)
EVT_SIZE(GraphicsCanvas::onSize)
END_EVENT_TABLE()

namespace
{
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
#if 0
    template<class T, int n, int m>
    Eigen::Matrix<T, n, n> changeMatrixSize(const Eigen::Matrix<T, m, m>& src)
    {
        Eigen::Matrix<T, n, n> result;
        int l = min(n, m);
        for (int i = 0; i < l; i++) {
            for (int j = 0; j < l; j++) {
                result(i, j) = src(i, j);
            }
        }
        return result;
    }
#else
    template<class A, class B>
    A changeSize(const B& src)
    {
        A result;
        result.setZero();
        if (src.cols() == src.rows()) {
            int l = min(src.rows(), result.rows());
            for (int i = 0; i < l; i++) {
                for (int j = 0; j < l; j++) {
                    result(i, j) = src(i, j);
                }
            }
        } else {
            int l = min(src.rows(), result.rows());
            for (int i = 0; i < l; i++) {
                result(i, 0) = src(i, 0);
            }
        }
        return result;
    }
#endif

    template<class T, int n, int m>
    Eigen::Matrix<T, n, 1> changeVectorSize(const Eigen::Matrix<T, m, 1>& src)
    {
        Eigen::Matrix<T, n, 1> result;
        int l = min(n, m);
        for (int i = 0; i < l; i++) {
            result[i] = src[i];
        }
        return result;
    }
}

GraphicsCanvas::GraphicsCanvas(wxFrame* parent, const AppDataPtr& pAppData)
    : wxGLCanvas(parent, wxID_ANY, attribs, wxDefaultPosition, wxDefaultSize, 0, wxT("GLCanvas"))
    , _pAppData(pAppData)
    , _faceVBO(GL_TRIANGLES, 20)
    , _edgeVBO(GL_LINES, 20)
    , _origin(0, 0, 0)
{
    _pContext = make_shared<wxGLContext>(this);

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

    _trans.setIdentity();
    _intitialTrans = _trans;
    _proj.setIdentity();

    double xRotAngleRad = -90 * M_PI / 180;
    _rotToGl.setIdentity();
    _rotToGl(1, 1) = cos(xRotAngleRad);
    _rotToGl(1, 2) = -sin(xRotAngleRad);
    _rotToGl(2, 1) = sin(xRotAngleRad);
    _rotToGl(2, 2) = cos(xRotAngleRad);

    Bind(wxEVT_LEFT_DOWN, &GraphicsCanvas::onMouseLeftDown, this);
    Bind(wxEVT_LEFT_UP, &GraphicsCanvas::onMouseLeftUp, this);
    Bind(wxEVT_MIDDLE_DOWN, &GraphicsCanvas::onMouseMiddleDown, this);
    Bind(wxEVT_MIDDLE_UP, &GraphicsCanvas::onMouseMiddleUp, this);
    Bind(wxEVT_RIGHT_DOWN, &GraphicsCanvas::onMouseRightDown, this);
    Bind(wxEVT_RIGHT_UP, &GraphicsCanvas::onMouseRightUp, this);
    Bind(wxEVT_MOTION, &GraphicsCanvas::onMouseMove, this);
    Bind(wxEVT_MOUSEWHEEL, &GraphicsCanvas::onMouseWheel, this);
}

GraphicsCanvas::~GraphicsCanvas()
{
}

bool GraphicsCanvas::toggleShowSharpEdges()
{
    _showSharpEdges = !_showSharpEdges;
    changeEdgeViewElements();

    return _showSharpEdges;
}

bool GraphicsCanvas::toggleShowSharpVerts()
{
    _showSharpVerts = !_showSharpVerts;
    changeEdgeViewElements();

    return _showSharpVerts;
}

bool GraphicsCanvas::toggleShowTriNormals()
{
    _showTriNormals = !_showTriNormals;
    changeEdgeViewElements();

    return _showTriNormals;
}

bool GraphicsCanvas::toggleShowFaces()
{
    _showFaces = !_showFaces;
    changeFaceViewElements();

    return _showFaces;
}

bool GraphicsCanvas::toggleShowEdges()
{
    _showEdges = !_showEdges;
    changeFaceViewElements();
    changeEdgeViewElements();

    return _showEdges;
}

bool GraphicsCanvas::toggleShowOuter()
{
    _showOuter = !_showOuter;
    changeFaceViewElements();
    changeEdgeViewElements();

    return _showOuter;
}

void GraphicsCanvas::onMouseLeftDown(wxMouseEvent& event)
{
    _mouseStartLoc2D = calMouseLoc(event.GetPosition());
    _mouseStartLoc3D = screenPointToModel(_mouseStartLoc2D);
    CMeshPtr pMesh = _pAppData ? _pAppData->getMesh() : nullptr;
    if (pMesh) {
        Vector3d dir(screenVectorToModel(Vector3d(0, 0, 1)));
        dir.normalize();
        Ray ray(_mouseStartLoc3D, dir);
        vector<RayHit> hits;
        if (pMesh->rayCast(ray, hits)) {
            cout << "Hit model\n";
            _mouseStartLoc3D = hits.front().hitPt;
        }
        else {
            cout << "Missed model\n";
            auto bbox = pMesh->getBBox();
            Vector3d ctr = (bbox.getMin() + bbox.getMax()) * 0.5;
            Vector3d v = ctr - _mouseStartLoc3D;
            double dp = v.dot(dir);
            _mouseStartLoc3D += dp * dir;
        }
    }

    _intitialTrans = _trans;
    _leftDown = true;
}

void GraphicsCanvas::onMouseLeftUp(wxMouseEvent& event)
{
    _leftDown = false;
}

void GraphicsCanvas::onMouseMiddleDown(wxMouseEvent& event)
{
    _mouseStartLoc2D = calMouseLoc(event.GetPosition());
    _intitialTrans = _trans;
    _middleDown = true;
}

void GraphicsCanvas::onMouseMiddleUp(wxMouseEvent& event)
{
    _middleDown = false;

    Eigen::Vector2d pos = calMouseLoc(event.GetPosition());
    Vector3d sp = screenPointToModel(_mouseStartLoc2D);
    Vector3d ep = screenPointToModel(pos);
    Vector3d delta = ep - sp;
    delta *= 2;
    _origin += delta;
}

void GraphicsCanvas::onMouseRightDown(wxMouseEvent& event)
{
    _mouseStartLoc2D = calMouseLoc(event.GetPosition());
    _intitialTrans = _trans;
    _rightDown = true;
}

void GraphicsCanvas::onMouseRightUp(wxMouseEvent& event)
{
    _rightDown = false;
}

Eigen::Vector2d GraphicsCanvas::calMouseLoc(const wxPoint& pt)
{
    wxSize frameSize = GetSize();
    double x = -1 + 2 * pt.x / (double)frameSize.x;
    double y = -1 + 2 * pt.y / (double)frameSize.y;
    return Eigen::Vector2d(x, y);
}

void GraphicsCanvas::onMouseMove(wxMouseEvent& event)
{
    Eigen::Vector2d pos = calMouseLoc(event.GetPosition());
    {
        Vector3d temp = screenPointToModel(pos);
        _mouseLoc3D = Vector3f((float)temp[0], (float)temp[1], (float)temp[2]);
    }
    if (_leftDown) {
        Eigen::Vector2d delta = pos - _mouseStartLoc2D;
        Eigen::Vector2d orth(-delta[1], delta[0]);
        Vector3d axis(screenVectorToModel(orth, 0));
        axis.normalize();
        double angle = delta.norm() * M_PI / 2;
        applyRotation(angle, _mouseStartLoc3D, axis);

    } else if (_middleDown) {
        Vector3d sp = screenPointToModel(_mouseStartLoc2D);
        Vector3d ep = screenPointToModel(pos);
        Vector3d delta = ep - sp;

        moveOrigin(delta);
    } else if (_rightDown) {
    }
}

void GraphicsCanvas::onMouseWheel(wxMouseEvent& event)
{
    double t = fabs(event.m_wheelRotation / (double)event.m_wheelDelta);
    double scaleMult = 1 + t * 0.05;
    double scale = 1.0;

    if (event.m_wheelRotation > 0) {
        scale *= scaleMult;
    } else if (event.m_wheelRotation < 0) {
        scale /= scaleMult;
    } else
        return;
    applyScale(scale);
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

void GraphicsCanvas::initialize() 
{
	if (_initialized)
		return;

	loadShaders();
	_initialized = true;
	return;
}

void GraphicsCanvas::loadShaders()
{
    SetCurrent(*_pContext);
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
    SetCurrent(*_pContext);
	initialize();

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
        glGetActiveUniformBlockiv(_phongShader->programID(), vertUboIdx, GL_UNIFORM_BLOCK_DATA_SIZE, &blockSize);GL_ASSERT
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

    drawMousePos3D();
    drawFaces();
    drawEdges();

    SwapBuffers();
    _phongShader->unBind();
}

void GraphicsCanvas::drawMousePos3D()
{
    float len = 0.1f;
    const Vector3f x(len, 0, 0), y(0, len, 0), z(0, 0, len);

    Vector3f x0 = _mouseLoc3D - x;
    Vector3f x1 = _mouseLoc3D + x;

    Vector3f y0 = _mouseLoc3D - y;
    Vector3f y1 = _mouseLoc3D + y;

    Vector3f z0 = _mouseLoc3D - z;
    Vector3f z1 = _mouseLoc3D + z;

    glBegin(GL_LINES);
    glVertex3f(x0[0], x0[1], x0[2]);
    glVertex3f(x1[0], x1[1], x1[2]);

    glVertex3f(y0[0], y0[1], y0[2]);
    glVertex3f(y1[0], y1[1], y1[2]);

    glVertex3f(z0[0], z0[1], z0[2]);
    glVertex3f(z1[0], z1[1], z1[2]);
    glEnd();
}

void GraphicsCanvas::drawFaces()
{
    auto preDraw = [this](int key) -> COglMultiVBO::DrawVertexColorMode {
        _graphicsUBO.ambient = 0.2f;
        switch (key) {
            default:
            case DS_MODEL:
                _graphicsUBO.defColor = p3f(1.0f, 1.0f, 1.0f);
                break;
            case DS_MODEL_SHARP_VERTS:
                _graphicsUBO.defColor = p3f(1.0f, 1.0f, 0);
                break;
            case DS_BLOCK_MESH + DSS_OUTER:
                _graphicsUBO.defColor = p3f(0.0f, 0.8f, 0);
                break;
            case DS_BLOCK_MESH + DSS_INNER:
                _graphicsUBO.defColor = p3f(0.75f, 1, 1);
                break;
            case DS_BLOCK_MESH + DSS_BLOCK_BOUNDARY:
                _graphicsUBO.defColor = p3f(1.0f, 0.5f, 0.5f);
                break;
            case DS_BLOCK_MESH + DSS_LAYER_BOUNDARY:
                _graphicsUBO.defColor = p3f(0.0f, 0.6f, 0);
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
            case DS_MODEL:
                glLineWidth(2.0f);
                _graphicsUBO.defColor = p3f(1.0f, 0.0f, 0.0f);
                break;
            case DS_MODEL_SHARP_EDGES:
                glLineWidth(2.0f);
                _graphicsUBO.defColor = p3f(1.0f, 0.0f, 0);
                break;
            case DS_MODEL_NORMALS:
                glLineWidth(1.0f);
                _graphicsUBO.defColor = p3f(0.0f, 0.0f, 1.0f);
                break;
            case DS_BLOCK_MESH + DSS_OUTER:
            case DS_BLOCK_MESH + DSS_INNER:
            case DS_BLOCK_MESH + DSS_LAYER_BOUNDARY:
                glLineWidth(1.0f);
                _graphicsUBO.defColor = p3f(0.0f, 0.0f, 0.50f);
                break;
            case DS_BLOCK_MESH + DSS_BLOCK_BOUNDARY:
                glLineWidth(1.0f);
                _graphicsUBO.defColor = p3f(0.75f, 0, 0);
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

Vector3d GraphicsCanvas::screenPointToModel(const Eigen::Vector2d& pt2d) const
{
    Eigen::Vector4d pt3d(pt2d[0], -pt2d[1], 0, 1);
    Eigen::Vector4d r = cumTransform().inverse() * pt3d;
    return changeSize<Vector3d, Eigen::Vector4d>(r);
}
Vector3d GraphicsCanvas::screenVectorToModel(const Eigen::Vector2d& v, double z) const
{
    return screenVectorToModel(Vector3d(v[0], v[1], z));
}
Vector3d GraphicsCanvas::screenVectorToModel(const Eigen::Vector3d& v) const
{
    Eigen::Vector4d pt3d(v[0], -v[1], v[2], 0);
    Eigen::Vector4d r = cumTransform().inverse() * pt3d;
    return changeSize<Vector3d, Eigen::Vector4d>(r);
}

Eigen::Matrix4d GraphicsCanvas::createTranslation(const Vector3d& delta)
{
    Eigen::Transform<double, 3, Eigen::Affine> t = Eigen::Transform<double, 3, Eigen::Affine>::Identity();
    Eigen::Vector3d x(delta);
    t.translate(x);
    Eigen::Matrix4d result(t.matrix());
    return result;
}

void GraphicsCanvas::moveOrigin(const Vector3d& delta)
{
    Eigen::Matrix4d pan(createTranslation(delta));

    _trans = _intitialTrans;
    _trans *= pan;
}

inline void GraphicsCanvas::applyScale(double scaleFact)
{
    Eigen::Matrix4d pan, scale;
    scale.setIdentity();
    pan = createTranslation(_origin);

    for (int i = 0; i < 3; i++) {
        scale(i, i) = scaleFact;
    }
    Eigen::Matrix4d panInv(pan.inverse());

    _trans *= panInv;
    _trans *= scale;
    _trans *= pan;

    _proj *= scaleFact;
    _proj(2, 2) = 0.1;
}

inline void GraphicsCanvas::applyRotation(double angle, const Vector3d& rotationCenter, const Vector3d& rotationAxis)
{
    Eigen::Matrix3d rot3 = Eigen::AngleAxisd(angle, rotationAxis).toRotationMatrix();
    Eigen::Matrix4d rot(changeSize<Eigen::Matrix4d>(rot3));
    rot(3, 3) = 1;

    _trans = _intitialTrans;
    _trans *= createTranslation(-rotationCenter);
    _trans *= rot;
    _trans *= createTranslation(rotationCenter);
}

inline Eigen::Matrix4d GraphicsCanvas::cumTransform() const
{
    return _rotToGl * _trans;
}

void GraphicsCanvas::updateView()
{
    Eigen::Matrix4d m = cumTransform();

    float* pMV = _graphicsUBO.modelView;
    float* pPr = _graphicsUBO.proj;
    for (int i = 0; i < 16; i++) {
        pMV[i] = (float)m(i);
        pPr[i] = (float)_proj(i);
    }
}

// vertiIndices is index pairs into points, normals and parameters to form triangles. It's the standard OGL element index structure
const GraphicsCanvas::OGLIndices* GraphicsCanvas::setFaceTessellation(const CMeshPtr& pMesh)
{
    const auto& points = pMesh->getGlPoints();
    const auto& normals = pMesh->getGlNormals(false);
    const auto& parameters = pMesh->getGlParams();
    const auto& vertIndices = pMesh->getGlFaceIndices();
    return _faceVBO.setFaceTessellation(pMesh->getId(), pMesh->getChangeNumber(), points, normals, parameters, vertIndices);
}


void GraphicsCanvas::changeFaceViewElements()
{
    _faceVBO.beginSettingElementIndices(0xffffffffffffffff);
    if (_pTriTess)
        _faceVBO.includeElementIndices(DS_MODEL, *_pTriTess);

    if (_showSharpVerts && _pSharpVertTess)
        _faceVBO.includeElementIndices(DS_MODEL_SHARP_VERTS, *_pSharpVertTess);

    if (_showFaces && !_faceTessellations.empty()) {
        if (_showOuter && DSS_OUTER < _faceTessellations.size()) {
            for (auto pBlockTess : _faceTessellations[DSS_OUTER]) {
                if (pBlockTess)
                    _faceVBO.includeElementIndices(DS_BLOCK_MESH + DSS_OUTER, *pBlockTess);
            }
        } 
        
        if (_showOuter && DSS_LAYER_BOUNDARY < _faceTessellations.size()) {
            for (auto pBlockTess : _faceTessellations[DSS_LAYER_BOUNDARY]) {
                if (pBlockTess)
                    _faceVBO.includeElementIndices(DS_BLOCK_MESH + DSS_LAYER_BOUNDARY, *pBlockTess);
            }
        }

        if (!_showOuter && DSS_INNER < _faceTessellations.size()) {
            for (auto pBlockTess : _faceTessellations[DSS_INNER]) {
                if (pBlockTess)
                    _faceVBO.includeElementIndices(DS_BLOCK_MESH + DSS_INNER, *pBlockTess);
            }
        } 
        
        if (!_showOuter && DSS_BLOCK_BOUNDARY < _faceTessellations.size()) {
            for (auto pBlockTess : _faceTessellations[DSS_BLOCK_BOUNDARY]) {
                if (pBlockTess)
                    _faceVBO.includeElementIndices(DS_BLOCK_MESH + DSS_BLOCK_BOUNDARY, *pBlockTess);
            }
        }
    }
    _faceVBO.endSettingElementIndices();
}

void GraphicsCanvas::changeEdgeViewElements()
{
    _edgeVBO.beginSettingElementIndices(0xffffffffffffffff);

    if (_showSharpEdges && _pSharpEdgeTess) {
        _edgeVBO.includeElementIndices(DS_MODEL_SHARP_EDGES, *_pSharpEdgeTess);
    }
    if (_showTriNormals && _pNormalTess) {
        _edgeVBO.includeElementIndices(DS_MODEL_NORMALS, *_pNormalTess);
    }
    if (_showEdges && !_edgeTessellations.empty()) {
        if (_showOuter && DSS_OUTER < _edgeTessellations.size()) {
            for (auto pBlockTess : _edgeTessellations[DSS_OUTER]) {
                if (pBlockTess)
                    _edgeVBO.includeElementIndices(DS_BLOCK_MESH + DSS_OUTER, *pBlockTess);
            }
        } 
        
        if (_showOuter && DSS_LAYER_BOUNDARY < _edgeTessellations.size()) {
            for (auto pBlockTess : _edgeTessellations[DSS_LAYER_BOUNDARY]) {
                if (pBlockTess)
                    _edgeVBO.includeElementIndices(DS_BLOCK_MESH + DSS_LAYER_BOUNDARY, *pBlockTess);
            }
        }

        if (!_showOuter && DSS_INNER < _edgeTessellations.size()) {
            for (auto pBlockTess : _edgeTessellations[DSS_INNER]) {
                if (pBlockTess)
                    _edgeVBO.includeElementIndices(DS_BLOCK_MESH + DSS_INNER, *pBlockTess);
            }
        }
        
        if (!_showOuter && DSS_BLOCK_BOUNDARY < _edgeTessellations.size()) {
            for (auto pBlockTess : _edgeTessellations[DSS_BLOCK_BOUNDARY]) {
                if (pBlockTess)
                    _edgeVBO.includeElementIndices(DS_BLOCK_MESH + DSS_BLOCK_BOUNDARY, *pBlockTess);
            }
        }
    }
    _edgeVBO.endSettingElementIndices();

}