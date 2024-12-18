/*
This file is part of the DistFieldHexMesh application/library.

    The DistFieldHexMesh application/library is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    The DistFieldHexMesh application/library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    This link provides the exact terms of the GPL license <https://www.gnu.org/licenses/>.

    The author's interpretation of GPL 3 is that if you receive money for the use or distribution of the DistFieldHexMesh application/library or a derivative product, GPL 3 no longer applies.

    Under those circumstances, the author expects and may legally pursue a reasoble share of the income. To avoid the complexity of agreements and negotiation, the author makes
    no specific demands in this regard. Compensation of roughly 1% of net or $5 per user license seems appropriate, but is not legally binding.

    In lay terms, if you make a profit by using the DistFieldHexMesh application/library (violating the spirit of Open Source Software), I expect a reasonable share for my efforts.

    Robert R Tipton - Author

    Dark Sky Innovative Solutions http://darkskyinnovation.com/
*/

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
//#include <GL/glu.h>
#endif

#ifndef WIN32
#include <unistd.h> // FIXME: �This work/necessary in Windows?
//Not necessary, but if it was, it needs to be replaced by process.h AND io.h
#endif

#include <defines.h>
#include <cmath>
#include <Eigen/Geometry>
#include <Eigen/src/Core/Matrix.h>
#include <OGLMultiVboHandlerTempl.h>
#include <OGLShader.h>
#include <OGLMath.h>
#include <tm_vector3.h>
#include <tm_ray.h>
#include <triMesh.h>
#include <volume.h>
#include <appData.h>
#include <meshData.h>

using namespace std;
using namespace DFHM;

BEGIN_EVENT_TABLE(GraphicsCanvas, wxGLCanvas)
EVT_PAINT(GraphicsCanvas::doPaint)
EVT_SIZE(GraphicsCanvas::onSizeChange)
END_EVENT_TABLE()

#define DRAW_MOUSE_POSITION 0

namespace
{
    inline float toRad(float v)
    {
        return v * M_PI / 180.0f;
    }


    int attribs[] = {
        WX_GL_DEPTH_SIZE, 16,
#if GRAPHICS_OVER_SAMPLING > 1
        WX_GL_SAMPLES, GRAPHICS_OVER_SAMPLING * GRAPHICS_OVER_SAMPLING,
#endif
        0
    };

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

    template<class A, class B>
    A rot3ToRot4(const B& src)
    {
        A result = changeSize<A,B>(src);
        result(3, 3) = 1;
        return result;
    }

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
    : wxGLCanvas(parent, wxID_ANY, attribs, wxPoint(200, 0), wxDefaultSize, 0, wxT("GLCanvas"))
    , _pAppData(pAppData)
{
    _modelVBOs = make_shared<VBORec>();
    _meshVBOs = make_shared<VBORec>();

    _pContext = make_shared<wxGLContext>(this);

    initProjection();
    setLights();
    setView(GraphicsCanvas::VIEW_FRONT);

//    Bind(wxEVT_SIZE, &GraphicsCanvas::onSizeChange, this);

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

namespace {
    void fromMatrixToGl(const Eigen::Matrix4d& src, m44f& dst)
    {
        float* pDst = (float*) & dst;
        for (int i = 0; i < 16; i++)
            pDst[i] = src(i);
    }
}

void GraphicsCanvas::setView(Vector3d viewVec)
{
    _viewBounds = _pAppData->getBoundingBox();
    _modelView.setIdentity();
    _intitialModelView.setIdentity();

    viewVec.normalize();
    double alpha = atan2(viewVec[1], viewVec[0]);
    double r = sqrt(viewVec[0] * viewVec[0] + viewVec[1] * viewVec[1]);

    double phi = atan2(viewVec[2], r);

    Vector3d zAxis(0, 0, 1);
    Eigen::Vector3d rotatedXAxis(1, 0, 0);
    rotatedXAxis.normalize();

    Eigen::Matrix3d rotation, tmpRotation;
    Vector3d xAxis(1, 0, 0);
    rotation.setIdentity();

    tmpRotation = Eigen::AngleAxisd(alpha, (Eigen::Matrix<double, 3, 1>)zAxis).toRotationMatrix();
    rotation = tmpRotation * rotation;
    rotatedXAxis = rotation * rotatedXAxis;

    tmpRotation = Eigen::AngleAxisd(phi, (Eigen::Matrix<double, 3, 1>)rotatedXAxis).toRotationMatrix();
    rotation = tmpRotation * rotation;

    _modelView = rot3ToRot4<Eigen::Matrix4d>(rotation);

}

void GraphicsCanvas::setView(View v)
{
    switch (v) {
    case VIEW_RIGHT:
        setView(Vector3d(1, 0, 0));
        break;
    case VIEW_LEFT:
        setView(Vector3d(-1, 0, 0));
        break;

    case VIEW_FRONT:
        setView(Vector3d(0, 1, 0));
        break;
    case VIEW_BACK:
        setView(Vector3d(0, -1, 0));
        break;

    case VIEW_TOP:
        setView(Vector3d(0, 0, -1));
        break;
    case VIEW_BOTTOM:
        setView(Vector3d(0, 0, 1));
        break;

    }
}

void GraphicsCanvas::resetView()
{
    _viewScale = INIT_VIEW_SCALE;
    initProjection();
    setView(GraphicsCanvas::VIEW_FRONT);
}

void GraphicsCanvas::setLights()
{
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
    for (int i = 0; i < _graphicsUBO.numLights; i++) {
        float sinAz = sinf(lightAz[i]);
        float cosAz = cosf(lightAz[i]);
        float sinEl = sinf(lightEl[i]);
        float cosEl = cosf(lightEl[i]);

        _graphicsUBO.lightDir[i] = p3f(cosEl * sinAz, sinEl, cosEl * cosAz);
    }
}

bool GraphicsCanvas::toggleShowSharpEdges()
{
    _viewOptions.showSharpEdges = !_viewOptions.showSharpEdges;
    changeEdgeViewElements();

    return _viewOptions.showSharpEdges;
}

bool GraphicsCanvas::toggleShowSharpVerts()
{
    _viewOptions.showSharpVerts = !_viewOptions.showSharpVerts;
    changeFaceViewElements();

    return _viewOptions.showSharpVerts;
}

bool GraphicsCanvas::toggleShowTriNormals()
{
    _viewOptions.showTriNormals = !_viewOptions.showTriNormals;
    changeEdgeViewElements();

    return _viewOptions.showTriNormals;
}

bool GraphicsCanvas::toggleShowFaces()
{
    _viewOptions.showFaces = !_viewOptions.showFaces;
    changeFaceViewElements();

    return _viewOptions.showFaces;
}

bool GraphicsCanvas::toggleShowCurvature()
{
    _viewOptions.showCurvature = !_viewOptions.showCurvature;
    changeFaceViewElements();
    return _viewOptions.showCurvature;
}

bool GraphicsCanvas::toggleShowEdges()
{
    _viewOptions.showEdges = !_viewOptions.showEdges;
    changeFaceViewElements();
    changeEdgeViewElements();

    return _viewOptions.showEdges;
}

bool GraphicsCanvas::toggleShowOuter()
{
    _viewOptions.showOuter = !_viewOptions.showOuter;
    changeFaceViewElements();
    changeEdgeViewElements();

    return _viewOptions.showOuter;
}

bool GraphicsCanvas::toggleShowModelBoundary()
{
    _viewOptions.showModelBoundary = !_viewOptions.showModelBoundary;
    changeFaceViewElements();
    changeEdgeViewElements();

    return _viewOptions.showOuter;
}

void GraphicsCanvas::onMouseLeftDown(wxMouseEvent& event)
{
    _mouseStartLocNDC_2D = screenToNDC(event.GetPosition());
    vector<CMeshPtr> meshes;
    for (const auto& md : _pAppData->getMeshObjects()) {
        auto pMeshData = md.second;
        if (pMeshData->isActive() && !pMeshData->isReference()) {
            auto pMesh = pMeshData->getMesh();
            if (pMesh)
                meshes.push_back(pMesh);
        }
    }
    Vector3d hitModel;
    double minDist = DBL_MAX;
    bool hadHit = false;
    CBoundingBox3Dd bbox;
    Vector3d startPt = NDCPointToModel(_mouseStartLocNDC_2D);
    if (meshes.empty()) {
        // Rotate about point hit at arbitrary depth
        hitModel = startPt;
    } else {
        Vector3d dir(screenVectorToModel(Vector3d(0, 0, 1)));
        dir.normalize();
        Rayd ray(startPt, dir);
        for (const auto pMesh : meshes) {
            bbox.merge(pMesh->getBBox());
            vector<RayHitd> hits;
            if (pMesh->rayCast(ray, hits)) {
                hadHit = true;
                // Rotate about hit point
                for (const auto& hit : hits) {
                    if (hit.dist < minDist) {
                        minDist = hit.dist;
                        hitModel = hit.hitPt;
                    }
                }
            }
        }
        if (!hadHit) {
            hitModel = bbox.getMin() + bbox.range() * 0.5;
        }
    }

    _mouseStartLocal = pointToLocal(hitModel);

    _intitialModelView = _modelView;
    _leftDown = true;
}

void GraphicsCanvas::onMouseLeftUp(wxMouseEvent& event)
{
    _leftDown = false;
}

void GraphicsCanvas::onMouseMiddleDown(wxMouseEvent& event)
{
    _mouseStartLocNDC_2D = screenToNDC(event.GetPosition());
    _initialProjection = _projection;
    _middleDown = true;
}

void GraphicsCanvas::onMouseMiddleUp(wxMouseEvent& event)
{
    _middleDown = false;
}

void GraphicsCanvas::onMouseRightDown(wxMouseEvent& event)
{
    _mouseStartLocNDC_2D = screenToNDC(event.GetPosition());
    _intitialModelView = _modelView;
    _rightDown = true;
}

void GraphicsCanvas::onMouseRightUp(wxMouseEvent& event)
{
    _rightDown = false;
}

Eigen::Vector2d GraphicsCanvas::screenToNDC(const wxPoint& pt)
{
    // Screen coordinates are left to right TOP to BOTTOM - like text
    // NDC are (-1,-1,-1) to (1,1,1) with the origin at the center.
    wxSize frameSize = GetSize();
    double x = -1 + 2 * pt.x / (double)frameSize.x;
    double y = -1 + 2 * pt.y / (double)frameSize.y;
    return Eigen::Vector2d(x, -y); // Invert the y axis
}

void GraphicsCanvas::onMouseMove(wxMouseEvent& event)
{
    Eigen::Vector2d pos = screenToNDC(event.GetPosition());
    {
        Vector3d temp = NDCPointToModel(pos);
        _mouseLoc3D = Vector3f((float)temp[0], (float)temp[1], (float)temp[2]);
    }
    if (_leftDown) {
        wxSize frameSize = GetSize();
        Eigen::Vector2d delta = pos - _mouseStartLocNDC_2D;
        double angleSpin = delta[0] * M_PI / 2;
        double anglePitch = delta[1] * M_PI / 2;
        applyRotation(angleSpin, anglePitch, _mouseStartLocal);

    } else if (_middleDown) {
        Eigen::Vector2d delta2D = pos - _mouseStartLocNDC_2D;

        moveOrigin(delta2D);
    } else if (_rightDown) {
    }
}

void GraphicsCanvas::onMouseWheel(wxMouseEvent& event)
{
    Eigen::Vector2d pos = screenToNDC(event.GetPosition());

    double t = fabs(event.m_wheelRotation / (double)event.m_wheelDelta);
    double scaleMult = 1 + t * 0.1;

    if (event.m_wheelRotation > 0) {
        scaleMult = scaleMult;
    } else if (event.m_wheelRotation < 0) {
        scaleMult = 1 / scaleMult;
    } else
        return;

    applyScaleFactor(scaleMult, pos);
}

void GraphicsCanvas::onSizeChange(wxSizeEvent& event)
{
    updateProjectionAspect();
    event.Skip();
}

void GraphicsCanvas::clearMesh3D()
{
    _meshVBOs->_edgeVBO.clear();
    _meshVBOs->_faceVBO.clear();
    _meshVBOs->_edgeTessellations.clear();
    _meshVBOs->_faceTessellations.clear();
}

void GraphicsCanvas::registerMeshData(MeshDataPtr& pMeshData)
{
    pMeshData->setShader(_phongShader);
}

void GraphicsCanvas::doPaint(wxPaintEvent& WXUNUSED(event)) {
    render();
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

    _modelVBOs->_faceVBO.setShader(_phongShader.get());
    _modelVBOs->_edgeVBO.setShader(_phongShader.get());

    _meshVBOs->_faceVBO.setShader(_phongShader.get());
    _meshVBOs->_edgeVBO.setShader(_phongShader.get());
}

Vector3d GraphicsCanvas::pointToLocal(const Vector3d& pointMC) const
{
    Eigen::Vector4d pointMC4(pointMC[0], pointMC[1], pointMC[2], 1);
    Eigen::Vector4d pointLC4 = cumTransform(false) * pointMC4;
    Vector3d result(pointLC4[0], pointLC4[1], pointLC4[2]);
    return result;
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

    drawMousePos3D(); // Available for mouse position testing
    drawFaces();
    drawEdges();

    SwapBuffers();
    _phongShader->unBind();
}

void GraphicsCanvas::drawMousePos3D()
{
#if  DRAW_MOUSE_POSITION
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

#endif //  DRAW_MOUSE_POSITION
}

void GraphicsCanvas::drawFaces()
{
    auto preDraw = [this](int key) -> COglMultiVBO::DrawVertexColorMode {
        COglMultiVBO::DrawVertexColorMode result = COglMultiVBO::DrawVertexColorMode::DRAW_COLOR_NONE;
        _graphicsUBO.ambient = 0.2f;
        switch (key) {
            default:
            case DS_MODEL:
                _graphicsUBO.defColor = p3f(0.9f, 0.9f, 1.0f);
                break;
            case DS_MODEL_CURVATURE:
                result = COglMultiVBO::DrawVertexColorMode::DRAW_COLOR;
                _graphicsUBO.defColor = p3f(0.0f, 0.0f, 0.0f); // Must be all 0 to turn on vertex color drawing
                break;
            case DS_MODEL_SHARP_VERTS:
                _graphicsUBO.defColor = p3f(1.0f, 1.0f, 0);
                break;
            case DS_BLOCK_ALL:
            case DS_BLOCK_OUTER:
                _graphicsUBO.defColor = p3f(0.0f, 0.8f, 0);
                break;
            case DS_BLOCK_INNER:
                _graphicsUBO.defColor = p3f(0.75f, 1, 1);
                break;
            case DS_BLOCK_BOUNDARY:
                _graphicsUBO.defColor = p3f(1.0f, 0.5f, 0.5f);
                break;
        }
        glBufferData(GL_UNIFORM_BUFFER, sizeof(_graphicsUBO), &_graphicsUBO, GL_DYNAMIC_DRAW);
        glEnable(GL_DEPTH_TEST);
        glDepthFunc(GL_LESS);

        if (_viewOptions.showSharpEdges) {
            glEnable(GL_POLYGON_OFFSET_FILL);
            glPolygonOffset(1.0f, 2.0f);
        }

        return result;
    };

    auto postDraw = [this]() {
        glDisable(GL_POLYGON_OFFSET_FILL);
        };

    auto preTexDraw = [this](int key) {
        };

    auto postTexDraw = [this]() {
        };

//    _modelVBOs->_faceVBO.drawAllKeys(preDraw, postDraw, preTexDraw, postTexDraw);
    _meshVBOs->_faceVBO.drawAllKeys(preDraw, postDraw, preTexDraw, postTexDraw);

    auto& meshObjects = _pAppData->getMeshObjects();
    for (auto& d : meshObjects)
    {
        auto& VBO = d.second->getFaceVBO();
        VBO.drawAllKeys(preDraw, postDraw, preTexDraw, postTexDraw);
    }
}

void GraphicsCanvas::drawEdges()
{
    auto preDraw = [this](int key) -> COglMultiVBO::DrawVertexColorMode {
        COglMultiVBO::DrawVertexColorMode result = COglMultiVBO::DrawVertexColorMode::DRAW_COLOR_NONE;
        switch (key) {
            default:
            case DS_MODEL:
                glLineWidth(2.0f);
                _graphicsUBO.defColor = p3f(1.0f, 0.0f, 0.0f);
                break;
            case DS_MODEL_SHARP_EDGES:
                glLineWidth(1.0f);
                _graphicsUBO.defColor = p3f(0.0f, 0.0f, 0);
                result = COglMultiVBO::DrawVertexColorMode::DRAW_COLOR;
                break;
            case DS_MODEL_NORMALS:
                glLineWidth(1.0f);
                _graphicsUBO.defColor = p3f(0.0f, 0.0f, 1.0f);
                break;
            case DS_BLOCK_ALL:
            case DS_BLOCK_OUTER:
            case DS_BLOCK_INNER:
                glLineWidth(1.0f);
                _graphicsUBO.defColor = p3f(0.0f, 0.0f, 0.50f);
                break;
            case DS_BLOCK_BOUNDARY:
                glLineWidth(1.0f);
                _graphicsUBO.defColor = p3f(0.75f, 0, 0);
                break;
            case DS_MODEL_REF_EDGES:
                glLineWidth(0.5f);
                _graphicsUBO.defColor = p3f(1.0f, 1.0f, 0);
                break;
        }
        _graphicsUBO.ambient = 1.0f;
        glBufferData(GL_UNIFORM_BUFFER, sizeof(_graphicsUBO), &_graphicsUBO, GL_DYNAMIC_DRAW);
        glEnable(GL_DEPTH_TEST);
        glDepthFunc(GL_LESS);

        return result;
    };

    auto postDraw = [this]() {
        };

    auto preTexDraw = [this](int key) {
        };

    auto postTexDraw = [this]() {
        };

//    _modelVBOs->_edgeVBO.drawAllKeys(preDraw, postDraw, preTexDraw, postTexDraw);
    _meshVBOs->_edgeVBO.drawAllKeys(preDraw, postDraw, preTexDraw, postTexDraw);

    auto& meshObjects = _pAppData->getMeshObjects();
    for (auto& d : meshObjects)
    {
        auto& VBO = d.second->getEdgeVBO();
        VBO.drawAllKeys(preDraw, postDraw, preTexDraw, postTexDraw);
    }
}

Vector3d GraphicsCanvas::NDCPointToModel(const Eigen::Vector2d& pt2d) const
{
    auto xform = cumTransform(true);
    auto invXform = xform.inverse();

    Eigen::Vector4d pt4d(pt2d[0], pt2d[1], 0, 1);
    Eigen::Vector4d r = invXform * pt4d;
    Eigen::Vector3d t = changeSize<Eigen::Vector3d>(r);
    return Vector3d(t[0], t[1], t[2]);
}

Vector3d GraphicsCanvas::screenVectorToModel(const Eigen::Vector2d& v, double z) const
{
    return screenVectorToModel(Vector3d(v[0], v[1], z));
}

Vector3d GraphicsCanvas::screenVectorToModel(const Eigen::Vector3d& v) const
{
    auto xform = cumTransform(true);
    auto invXform = xform.inverse();

    Eigen::Vector4d v4(v[0], v[1], v[2], 0);
    Eigen::Vector4d r = invXform * v4;
    Eigen::Vector3d t = changeSize<Eigen::Vector3d>(r);
    return Vector3d(t[0], t[1], t[2]);
}

Eigen::Matrix4d GraphicsCanvas::createTranslation(const Vector3d& delta)
{
    Eigen::Transform<double, 3, Eigen::Affine> t = Eigen::Transform<double, 3, Eigen::Affine>::Identity();
    Eigen::Vector3d x(delta);
    t.translate(x);
    Eigen::Matrix4d result(t.matrix());
    return result;
}

Eigen::Matrix4d GraphicsCanvas::createRotation(const Vector3d& axis, double angle)
{
    Eigen::Vector3d eAxis(axis[0], axis[1], axis[2]);
    Eigen::Matrix4d result = rot3ToRot4<Eigen::Matrix4d>(Eigen::AngleAxisd(angle, eAxis).toRotationMatrix());
    return result;
}

void GraphicsCanvas::moveOrigin(const Eigen::Vector2d& delta)
{
    Eigen::Matrix4d pan(createTranslation(Vector3d(delta[0], delta[1], 0)));
    _projection = _initialProjection;
    _projection = pan * _projection;
}

namespace
{
    Vector3d transformVector(const Vector3d& v, const Eigen::Matrix4d& mat)
    {
        Eigen::Vector4d v4(v[0], v[1], v[2], 0);
        v4 = mat * v4;
        Vector3d result(v4[0], v4[1], v4[2]);
        return result;
    }
}
inline void GraphicsCanvas::applyRotation(double angleSpin, double anglePitch, const Vector3d& rotationCenterLC)
{
    Eigen::Vector3d xAxis(1, 0, 0);
    Eigen::Vector3d zAxis(0, 0, 1);
    Eigen::Matrix4d translate(createTranslation(-rotationCenterLC));
    Eigen::Matrix4d unTranslate(createTranslation(rotationCenterLC));
    Eigen::Matrix4d rotSpin = rot3ToRot4<Eigen::Matrix4d>(Eigen::AngleAxisd(angleSpin, zAxis).toRotationMatrix());
    Eigen::Matrix4d rotPitch = rot3ToRot4<Eigen::Matrix4d>(Eigen::AngleAxisd(anglePitch, xAxis).toRotationMatrix());

    _modelView = _intitialModelView;
    _modelView = translate * _modelView;
    _modelView = rotSpin * _modelView;
    _modelView = rotPitch * _modelView;
    _modelView = unTranslate * _modelView;

    updateView();
}

void GraphicsCanvas::applyScaleFactor(double scaleMult, const Eigen::Vector2d& center)
{
    Vector3d center3d(center[0], center[1], 0);
    Eigen::Matrix4d translate(createTranslation(-center3d)), untranslate(createTranslation(center3d));
    Eigen::Matrix4d scale;
    scale.setIdentity();
    for (int i = 0; i < 2; i++)
        scale(i, i) = scaleMult;

    _projection = translate * _projection;
    _projection = scale * _projection;
    _projection = untranslate * _projection;

}

inline Eigen::Matrix4d GraphicsCanvas::cumTransform(bool withProjection) const
{
    Vector3d span = _viewBounds.range();
    Vector3d viewOrigin = _viewBounds.getMin() + span * 0.5;
    Eigen::Matrix4d view, result, scale;
    view.setIdentity();
    result.setIdentity();
    scale.setIdentity();

    view = createTranslation(-viewOrigin) * view;
    result *= view;
    double maxDim = 0;
    for (int i = 0; i < 3; i++) {
        if (span[i] > maxDim)
            maxDim = span[i];
    }
    double sf = maxDim > 0 ? 1.0 / maxDim : 1;
    sf *= 1 / _viewScale;
    scale(0) = sf;
    scale(5) = sf;
    scale(10) = -sf; // Open gl'z NDC xAxis is left to right, yAxis i bottom to top and zAxis is INTO the screen (depth) when it should be out of the screen - it's left handed.
                     // Screen coordinates are x left to right and Y TOP to BOTTOM - it's right handed.
    result = scale * result;

    result = _modelView * result;
    if (withProjection)
        result = _projAspect * _projection * result;

    return result;
}

void GraphicsCanvas::updateProjectionAspect()
{
    _projAspect.setIdentity();

    wxSize frameSize = GetSize();
    double ratio = frameSize.y / (double)frameSize.x;
    if (ratio >= 1)
        _projAspect(1, 1) = 1 / ratio;
    else
        _projAspect(0, 0) = ratio;
}

void GraphicsCanvas::initProjection()
{
    updateProjectionAspect();
    _projection.setIdentity();

    Eigen::Matrix<double, 3, 1> xAxis(1, 0, 0);
    double makeZUpAngle = 90.0 * M_PI / 180.0;
    Eigen::Matrix3d tmpRotation = Eigen::AngleAxisd(makeZUpAngle, xAxis).toRotationMatrix();
    Eigen::Matrix4d rotZUp = rot3ToRot4<Eigen::Matrix4d>(tmpRotation);

    _projection = rotZUp * _projection;

    Eigen::Matrix4d scale;
    scale.setIdentity();
    for (int i = 0; i < 2; i++)
        scale(i, i) *= _viewScale;

    _projection = scale * _projection;
}

void GraphicsCanvas::updateView()
{
    Eigen::Matrix4d m(cumTransform(false));
    Eigen::Matrix4d proj;

    proj = _projAspect * _projection;

    float* pMV = _graphicsUBO.modelViewX;
    float* pPr = _graphicsUBO.projX;
    for (int i = 0; i < 16; i++) {
        pMV[i] = (float)m(i);
        pPr[i] = (float)proj(i);
    }
}

void GraphicsCanvas::beginFaceTesselation(bool useModel)
{
    _activeVBOs = useModel ? _modelVBOs : _meshVBOs;
    _activeVBOs->_faceVBO.beginFaceTesselation();
}

void GraphicsCanvas::endFaceTesselation(const OGLIndices* pTriTess, const OGLIndices* pSharpVertTess, bool smoothNormals)
{
    _activeVBOs->_faceVBO.endFaceTesselation(smoothNormals);
    _activeVBOs->_pTriTess = pTriTess;
    _activeVBOs->_pSharpVertTess = pSharpVertTess;
    _activeVBOs->_faceTessellations.clear();

    changeFaceViewElements();
}

void GraphicsCanvas::endFaceTesselation(const std::vector<std::vector<const OGLIndices*>>& faceTess)
{
    _activeVBOs->_faceVBO.endFaceTesselation(false);
    _activeVBOs->_faceTessellations = faceTess;

    changeFaceViewElements();
}

void GraphicsCanvas::beginEdgeTesselation(bool useModel)
{
    _activeVBOs = useModel ? _modelVBOs : _meshVBOs;
    _activeVBOs->_edgeVBO.beginEdgeTesselation();
}

void GraphicsCanvas::endEdgeTesselation(const OGLIndices* pSharpEdgeTess, const OGLIndices* pNormalTess)
{
    _activeVBOs->_edgeVBO.endEdgeTesselation();

    _activeVBOs->_pSharpEdgeTess = pSharpEdgeTess;
    _activeVBOs->_pNormalTess = pNormalTess;
    _activeVBOs->_edgeTessellations.clear();

    changeEdgeViewElements();
}

void GraphicsCanvas::endEdgeTesselation(const std::vector<std::vector<const OGLIndices*>>& edgeTess)
{
    _activeVBOs->_edgeVBO.endEdgeTesselation();

    _activeVBOs->_edgeTessellations = edgeTess;

    changeEdgeViewElements();
}


// vertiIndices is index pairs into points, normals and parameters to form triangles. It's the standard OGL element index structure
const GraphicsCanvas::OGLIndices* GraphicsCanvas::setFaceTessellation(const CMeshPtr& pMesh)
{
    const auto& points = pMesh->getGlTriPoints();
    const auto& normals = pMesh->getGlTriNormals(false);
    const auto& parameters = pMesh->getGlTriParams();
    const auto& vertIndices = pMesh->getGlTriIndices();

    auto colorFunc = [](float curvature, float rgb[3])->bool {
        rgbaColor c = curvatureToColor(curvature);
        for (int i = 0; i < 3; i++)
            rgb[i] = c._rgba[i] / 255.0f;
        return true;
    };

    const auto& colors = pMesh->getGlTriCurvatureColors(colorFunc);
    return _activeVBOs->_faceVBO.setFaceTessellation(pMesh->getId(), pMesh->getChangeNumber(), points, normals, parameters, colors, vertIndices);
}

// vertiIndices is index pairs into points, normals and parameters to form triangles. It's the standard OGL element index structure
const GraphicsCanvas::OGLIndices* GraphicsCanvas::setEdgeSegTessellation(long entityKey, int changeNumber, const std::vector<float>& points, 
    const std::vector<unsigned int>& indices)
{
    return _activeVBOs->_edgeVBO.setEdgeSegTessellation(entityKey, changeNumber, points, indices);
}

const GraphicsCanvas::OGLIndices* GraphicsCanvas::setEdgeSegTessellation(const CMeshPtr& pMesh)
{
    vector<float> points, colors;
    vector<unsigned int> indices;
    auto colorFunc = [](float curvature, float rgb[3])->bool {
        rgbaColor c;
        if (curvature < 0)
            c = rgbaColor(1, 0, 0);
        else if (curvature < 1.0e-6)
            c = rgbaColor(0, 0, 0);
        else
            c = curvatureToColor(curvature);
        for (int i = 0; i < 3; i++)
            rgb[i] = c._rgba[i] / 255.0f;
        return true;
    };

    pMesh->getGlEdges(colorFunc, false, points, colors, indices);

    return _activeVBOs->_edgeVBO.setEdgeSegTessellation(pMesh->getId(), pMesh->getChangeNumber(), points, colors, indices);
}

void GraphicsCanvas::changeFaceViewElements()
{
    _modelVBOs->changeFaceViewElements(true, false, _viewOptions);
    auto& objs = _pAppData->getMeshObjects();
    for (auto& pair : objs) {
        auto& obj = pair.second;
        obj->changeFaceViewElements(_viewOptions);
    }
}

void GraphicsCanvas::changeEdgeViewElements()
{
    _modelVBOs->changeEdgeViewElements(true, false, _viewOptions);
    auto& objs = _pAppData->getMeshObjects();
    for (auto& pair : objs) {
        auto& obj = pair.second;
        obj->changeEdgeViewElements(_viewOptions);
    }
}

