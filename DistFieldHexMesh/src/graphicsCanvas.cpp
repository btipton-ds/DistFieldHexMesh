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
#include <drawHexMesh.h>
#include <drawModelMesh.h>

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

    void fromMatrixToGl(const Eigen::Matrix4d& src, m44f& dst)
    {
        float* pDst = (float*)&dst;
        for (int i = 0; i < 16; i++)
            pDst[i] = src(i);
    }
}


GraphicsCanvas::GraphicsCanvas(wxFrame* parent, const AppDataPtr& pAppData)
    : wxGLCanvas(parent, wxID_ANY, attribs, wxPoint(200, 0), wxDefaultSize, 0, wxT("GLCanvas"))
    , _pAppData(pAppData)
{
    _pDrawHexMesh = make_shared<DrawHexMesh>(this);
    _pDrawModelMesh = make_shared<DrawModelMesh>(this);

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

size_t GraphicsCanvas::numBytes() const
{
    size_t result = sizeof(GraphicsCanvas);

    if (_pDrawHexMesh)
        result += _pDrawHexMesh->numBytes();
    if (_pDrawModelMesh)
        result += _pDrawModelMesh->numBytes();
    std::shared_ptr<DrawHexMesh> _pDrawHexMesh;
    std::shared_ptr<DrawModelMesh> _pDrawModelMesh;

    return result;
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

void GraphicsCanvas::toggleShowFace(View v)
{
    switch (v) {
    case VIEW_RIGHT:
        _pDrawHexMesh->toggleShowRight();
        break;
    case VIEW_LEFT:
        _pDrawHexMesh->toggleShowLeft();
        break;

    case VIEW_FRONT:
        _pDrawHexMesh->toggleShowFront();
        break;
    case VIEW_BACK:
        _pDrawHexMesh->toggleShowBack();
        break;

    case VIEW_TOP:
        _pDrawHexMesh->toggleShowTop();
        break;
    case VIEW_BOTTOM:
        _pDrawHexMesh->toggleShowBottom();
        break;
    }
    _pDrawHexMesh->changeViewElements();
}

bool GraphicsCanvas::showFace(View v) const
{
    switch (v) {
    case VIEW_RIGHT:
        return _pDrawHexMesh->showRight();
    case VIEW_LEFT:
        return _pDrawHexMesh->showLeft();

    case VIEW_FRONT:
        return _pDrawHexMesh->showFront();
    case VIEW_BACK:
        return _pDrawHexMesh->showBack();

    case VIEW_TOP:
        return _pDrawHexMesh->showTop();
    case VIEW_BOTTOM:
        return _pDrawHexMesh->showBottom();
    default:
        break;
    }
    return false;
}

void GraphicsCanvas::resetView()
{
    _viewScale = INIT_VIEW_SCALE;
    initProjection();
    setView(GraphicsCanvas::VIEW_FRONT);
}

void GraphicsCanvas::setLights()
{
    const float elOffset = 0.0f;
    float lightAz[] = {
        0.0f,
        0.0f,
        0.0f,
        30.0f,
        60.0f,
    };

    float lightEl[] = {
        0.0f,
        0.0f,
        0.0f,
        0.0f,
        0.0f,
    };

    _graphicsUBO.defColor = p3f(1.0f, 1.0f, 1);
    _graphicsUBO.ambient = 0.3f;
    _graphicsUBO.numLights = 2;
    for (int i = 0; i < _graphicsUBO.numLights; i++) {
        float sinAz = sinf(toRad(lightAz[i]));
        float cosAz = cosf(toRad(lightAz[i]));
        float sinEl = sinf(toRad(lightEl[i] + elOffset));
        float cosEl = cosf(toRad(lightEl[i] + elOffset));

        _graphicsUBO.lightDir[i] = -p3f(cosEl * sinAz, cosEl * cosAz, sinEl);
    }
}

bool GraphicsCanvas::toggleShowModelSharpEdges()
{
    bool result = _pDrawModelMesh->toggleShowSharpEdges();
    changeViewElements();

    return result;
}

bool GraphicsCanvas::toggleShowSharpVerts()
{
    bool result = _pDrawModelMesh->toggleShowSharpVerts();
    changeViewElements();

    return result;
}

bool GraphicsCanvas::toggleShowTriNormals()
{
    bool result = _pDrawModelMesh->showTriNormals();
    changeViewElements();

    return result;
}

bool GraphicsCanvas::toggleShowModelFaces()
{
    bool result = _pDrawModelMesh->toggleShowFaces();
    changeViewElements();

    return result;
}

bool GraphicsCanvas::showCurvature() const
{
    return _pDrawModelMesh->showCurvature();
}

bool GraphicsCanvas::toggleShowCurvature()
{
    bool result = _pDrawModelMesh->toggleShowCurvature();
    changeViewElements();
    return result;
}

bool GraphicsCanvas::toggleShowModelEdges()
{
    bool result = _pDrawModelMesh->toggleShowEdges();
    changeViewElements();

    return result;
}

bool GraphicsCanvas::toggleShowMeshEdges()
{
    bool result = _pDrawHexMesh->toggleShowEdges();
    changeViewElements();

    return result;
}

bool GraphicsCanvas::toggleShowMeshFaces()
{
    bool result = _pDrawHexMesh->toggleShowFaces();
    changeViewElements();

    return result;
}

bool GraphicsCanvas::toggleShowMeshWalls()
{
    bool result = _pDrawHexMesh->toggleShowWalls();
    changeViewElements();

    return result;
}

bool GraphicsCanvas::toggleShowMeshAll()
{
    bool result = _pDrawHexMesh->toggleShowAll();
    changeViewElements();

    return result;
}

void GraphicsCanvas::setShowLayer(int32_t layerNum)
{
    _pDrawHexMesh->setShowLayer(layerNum);
    changeViewElements();
}

void GraphicsCanvas::onMouseLeftDown(wxMouseEvent& event)
{
    _mouseStartLocNDC_2D = screenToNDC(event.GetPosition());
    vector<CMeshPtr> meshes;
    for (const auto& md : *_pAppData->getMeshData()) {
        auto pMeshData = md.second;
        if (pMeshData->isActive()) {
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
    auto& VBOs = _pDrawHexMesh->getVBOs();
    VBOs->clear();
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
    
    _phongShader = make_shared<OGL::Shader>();

    _phongShader->setShaderVertexAttribName("inPosition");
    _phongShader->setShaderNormalAttribName("inNormal");
    _phongShader->setShaderColorAttribName("inColor");
//    _phongShader->setShaderTexParamAttribName("inPosition");

    _phongShader->setVertexSrcFile(path + "phong.vert");
    _phongShader->setFragmentSrcFile(path + "phong.frag");
    _phongShader->load();

    _pDrawModelMesh->setShader(_phongShader);
    _pDrawHexMesh->setShader(_phongShader);
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

    const GLuint bindingPoint = 1;
    static GLuint vertUboIdx = -1;
    static GLint blockSize = -1;
    static GLuint uniformBuffer = -1;
    if (vertUboIdx == -1) {
        vertUboIdx = glGetUniformBlockIndex(_phongShader->programID(), "UniformBufferObject");
        glGetActiveUniformBlockiv(_phongShader->programID(), vertUboIdx, GL_UNIFORM_BLOCK_DATA_SIZE, &blockSize); GL_ASSERT
        glGenBuffers(1, &uniformBuffer);
    }

    glUniformBlockBinding(_phongShader->programID(), vertUboIdx, bindingPoint);

    glBindBuffer(GL_UNIFORM_BUFFER, uniformBuffer);
    glBufferData(GL_UNIFORM_BUFFER, sizeof(_graphicsUBO), &_graphicsUBO, GL_DYNAMIC_DRAW);
    glBindBufferBase(GL_UNIFORM_BUFFER, bindingPoint, uniformBuffer);

    glClearColor(_backColor);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glViewport(0, 0, (GLint)GetSize().x, (GLint)GetSize().y);

    _pDrawModelMesh->render();
    _pDrawHexMesh->render();

#if  DRAW_MOUSE_POSITION
    drawMousePos3D(); // Available for mouse position testing
#endif //  DRAW_MOUSE_POSITION

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

void GraphicsCanvas::buildHexFaceTables(const VolumePtr& pVolume, const Index3D& min, const Index3D& max, bool multiCore)
{
    _pDrawHexMesh->buildHexFaceTables(pVolume, min, max, RUN_MULTI_THREAD);
}

void GraphicsCanvas::copyHexFaceTablesToVBOs()
{
    _pDrawHexMesh->copyHexFaceTablesToVBOs();
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

    float* pMV = _graphicsUBO.modelView;
    float* pPr = _graphicsUBO.proj;
    for (int i = 0; i < 16; i++) {
        pMV[i] = (float)m(i);
        pPr[i] = (float)proj(i);
    }
}

void GraphicsCanvas::changeViewElements()
{
    const auto& meshData = *_pAppData->getMeshData();
    _pDrawModelMesh->changeViewElements(meshData);
    _pDrawHexMesh->changeViewElements();
}
