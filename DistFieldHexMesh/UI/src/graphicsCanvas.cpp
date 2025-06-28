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

    Copyright Robert R Tipton, 2022, all rights reserved except those granted in prior license statement.

    Dark Sky Innovative Solutions http://darkskyinnovation.com/
*/

#include <memory>

#include <graphicsCanvas.h>
#include <mainFrame.h>

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

#if INCLUDE_DEBUG_WX_FRAME
#include <graphicsDebugCanvas.h>
#endif

using namespace std;
using namespace DFHM;

BEGIN_EVENT_TABLE(GraphicsCanvas, wxGLCanvas)
EVT_PAINT(GraphicsCanvas::doPaint)
EVT_SIZE(GraphicsCanvas::onSizeChange)
END_EVENT_TABLE()

#define DRAW_MOUSE_POSITION 0

namespace
{
    const float MAX_DEPTH = 1.0f;

    GLenum g_drawBuffers[] = {
        GL_COLOR_ATTACHMENT0,
        GL_COLOR_ATTACHMENT1,
        GL_COLOR_ATTACHMENT2,
        GL_COLOR_ATTACHMENT3,
        GL_COLOR_ATTACHMENT4,
        GL_COLOR_ATTACHMENT5,
        GL_COLOR_ATTACHMENT6,
    };

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
    : wxGLCanvas(parent, wxID_ANY, attribs, wxDefaultPosition, wxDefaultSize, 0, wxT("GLCanvas"))
    , _pAppData(pAppData)
{
    _pDrawHexMesh = make_shared<DrawHexMesh>(this);
    _pDrawModelMesh = make_shared<DrawModelMesh>(this);

    initProjection();
    setLights();
    setView(GraphicsCanvas::VIEW_FRONT);

    Bind(wxEVT_LEFT_DOWN, &GraphicsCanvas::onMouseLeftDown, this);
    Bind(wxEVT_LEFT_UP, &GraphicsCanvas::onMouseLeftUp, this);
    Bind(wxEVT_MIDDLE_DOWN, &GraphicsCanvas::onMouseMiddleDown, this);
    Bind(wxEVT_MIDDLE_UP, &GraphicsCanvas::onMouseMiddleUp, this);
    Bind(wxEVT_RIGHT_DOWN, &GraphicsCanvas::onMouseRightDown, this);
    Bind(wxEVT_RIGHT_UP, &GraphicsCanvas::onMouseRightUp, this);
    Bind(wxEVT_MOTION, &GraphicsCanvas::onMouseMove, this);
    Bind(wxEVT_MOUSEWHEEL, &GraphicsCanvas::onMouseWheel, this);

    Planed clipPlane0(Vector3d(8, 4, 0), Vector3d(1, 0, 1));
    Planed clipPlane1(Vector3d(8.25, 4, 0), -Vector3d(1, 0, 1));

    setClipplingPlane(0, clipPlane0);
    setClippingPlaneEnabled(0, false);

    setClipplingPlane(1, clipPlane1);
    setClippingPlaneEnabled(1, false);

    dumpUniformOffset();
}

GraphicsCanvas::~GraphicsCanvas()
{
    releaseDepthPeeling();
}

void GraphicsCanvas::dumpUniformOffset() const
{
    size_t uboBase = (size_t)&_graphicsUBO;
    size_t numLightsOff = (size_t)&_graphicsUBO.numLights - uboBase;
    size_t useDefColorOff = (size_t)&_graphicsUBO.useDefColor - uboBase;
    size_t normalShadingOnOff = (size_t)&_graphicsUBO.normalShadingOn - uboBase;
    size_t twoSideLightingOff = (size_t)&_graphicsUBO.twoSideLighting - uboBase;
    size_t ambientOff = (size_t)&_graphicsUBO.ambient - uboBase;

    size_t modelViewOff = (size_t)&_graphicsUBO.modelView - uboBase;
    size_t projOff = (size_t)&_graphicsUBO.proj - uboBase;
    size_t defColorOff = (size_t)&_graphicsUBO.defColor - uboBase;
    size_t lightDir0Off = (size_t)&_graphicsUBO.lightDir0 - uboBase;
    size_t lightDir1Off = (size_t)&_graphicsUBO.lightDir1 - uboBase;

    cout << "GraphicsUBO::modelView       : " << modelViewOff << "\n";
    cout << "GraphicsUBO::proj            : " << projOff << "\n";
    cout << "GraphicsUBO::defColor        : " << defColorOff << "\n";
    cout << "GraphicsUBO::lightDir0       : " << lightDir0Off << "\n";
    cout << "GraphicsUBO::lightDir1       : " << lightDir1Off << "\n";
    cout << "GraphicsUBO::ambient         : " << ambientOff << "\n";
    cout << "GraphicsUBO::useDefColor     : " << useDefColorOff << "\n";
    cout << "GraphicsUBO::normalShadingOn : " << normalShadingOnOff << "\n";
    cout << "GraphicsUBO::twoSideLighting : " << twoSideLightingOff << "\n";

    cout << "GraphicsUBO::numLightsOff    : " << numLightsOff << "\n";
}

std::shared_ptr<DrawCrossSectionEdges> GraphicsCanvas::getDrawCrossSectionEdges()
{
    if (!_pDrawCrossSections)
        _pDrawCrossSections = make_shared<DrawCrossSectionEdges>(this);
    return _pDrawCrossSections;
}


size_t GraphicsCanvas::numBytes() const
{
    size_t result = sizeof(GraphicsCanvas);

    if (_pDrawHexMesh)
        result += _pDrawHexMesh->numBytes();
    if (_pDrawModelMesh)
        result += _pDrawModelMesh->numBytes();
    if (_pDrawCrossSections)
        result += _pDrawCrossSections->numBytes();


    return result;
}

void GraphicsCanvas::setView(Vector3d viewVec)
{
    if (!_pAppData)
        return;
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

#if INCLUDE_DEBUG_WX_FRAME
void GraphicsCanvas::setDebugCanvas(GraphicsDebugCanvas* pCanvas)
{
    _pDebugCanvas = pCanvas;
}
#endif

void GraphicsCanvas::setLights()
{
    const float elOffset = 0.0f;
    float lightAz[] = {
        -25.0f,
        25.0f,
        0.0f,
        30.0f,
        60.0f,
    };

    float lightEl[] = {
        60.0f,
        60.0f,
        -60.0f,
        -60.0f,
        0.0f,
        0.0f,
        0.0f,
    };

    _graphicsUBO.defColor = p3f(1.0f, 1.0f, 1);
    _graphicsUBO.ambient = 0.3f;
    _graphicsUBO.numLights = 4;
    for (int i = 0; i < _graphicsUBO.numLights; i++) {
        float sinAz = sinf(toRad(lightAz[i]));
        float cosAz = cosf(toRad(lightAz[i]));
        float sinEl = sinf(toRad(lightEl[i] + elOffset));
        float cosEl = cosf(toRad(lightEl[i] + elOffset));
        p4f lv(cosEl * sinAz, sinEl, cosEl * cosAz, 0);
        switch (i){
        default:
        case 0:
            _graphicsUBO.lightDir0 = lv;;
            break;
        case 1:
            _graphicsUBO.lightDir1 = lv;;
            break;
        case 2:
            _graphicsUBO.lightDir2 = lv;;
            break;
        case 3:
            _graphicsUBO.lightDir3 = lv;;
            break;
        case 4:
            _graphicsUBO.lightDir4 = lv;;
            break;
        case 5:
            _graphicsUBO.lightDir5 = lv;;
            break;
        case 6:
            _graphicsUBO.lightDir6 = lv;;
            break;
        case 7:
            _graphicsUBO.lightDir7 = lv;;
            break;
        }
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
    bool result = _pDrawHexMesh->toggleShowErrorWalls();
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
    if (!_pAppData)
        return;
    _mouseStartLocNDC_2D = screenToNDC(event.GetPosition());

    Vector3d startPt = NDCPointToModel(_mouseStartLocNDC_2D);
    if (_meshSelection && event.ControlDown()) {
        onMouseLeftDownMesh(event, startPt);
    } else if ((_clippingMove || _clippingRotate) && event.ControlDown()) {
        onMouseLeftDownClipping(event);
    } else {
        onMouseLeftDownModel(event, startPt);
    }
}

void GraphicsCanvas::onMouseLeftDownModel(wxMouseEvent& event, const Vector3d& startPt)
{
    const auto& model = _pAppData->getModel();

    Vector3d hitModel;

    if (model.empty()) {
        // Rotate about point hit at arbitrary depth
        hitModel = startPt;
    }
    else {
        Vector3d dir(screenVectorToModel(Vector3d(0, 0, 1)));
        dir.normalize();
        Rayd ray(startPt, dir);
        MultiPolyMeshRayHit hit;
        if (model.rayCast(ray, hit)) {
            hitModel = hit.getPoint();
        }
        else {
            CBoundingBox3Dd bbox = _pAppData->getBoundingBox();
            hitModel = bbox.getMin() + bbox.range() * 0.5;
        }
    }

    _mouseStartLocal = pointToLocal(hitModel);

    _intitialModelView = _modelView;
    _leftDown = true;
}

void GraphicsCanvas::onMouseLeftDownMesh(wxMouseEvent& event, const Vector3d& startPt)
{

    Vector3d hitPt;

    Vector3d dir(screenVectorToModel(Vector3d(0, 0, 1)));
    dir.normalize();
    Rayd ray(startPt, dir);
    _pAppData->handleMeshRayCast(event, ray);

    _leftDown = false;
}

void GraphicsCanvas::onMouseLeftDownClipping(wxMouseEvent& event)
{
    _startPlane0 = getClipplingPlane(0);
    _startPlane1 = getClipplingPlane(1);
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

        if (_clippingMove && event.ControlDown()) {
            double dx = delta[0];
            if (!event.ShiftDown())
                dx *= 10;
            Vector3d origin0 = _startPlane0.getOrigin() + dx * _startPlane0.getNormal();
            Vector3d origin1 = _startPlane1.getOrigin() + dx * _startPlane0.getNormal();
            setClipplingPlane(0, Planed(origin0, _startPlane0.getNormal()));
            setClipplingPlane(1, Planed(origin1, _startPlane0.getNormal()));
            updateUniformBlock();
        } else if (_clippingRotate && event.ControlDown()) {
            double angleAz = delta[0] * M_PI / 2;
            double angleEl = delta[1] * M_PI / 2;
            auto& n = _startPlane0.getNormal();
            auto& x = _startPlane0.getXRef();
            Eigen::Vector3d zAxis (n[0], n[1], n[2]);
            Eigen::Vector3d xAxis (x[0], x[1], x[2]);
            auto yAxis = zAxis.cross(xAxis).normalized();
            Eigen::Matrix3d rotSpin = Eigen::AngleAxisd(angleAz, yAxis).toRotationMatrix();
            Eigen::Matrix3d rotPitch = Eigen::AngleAxisd(angleEl, xAxis).toRotationMatrix();
            Eigen::Matrix3d rot = rotSpin * rotPitch;
            Eigen::Vector3d n2 = rot * zAxis;
            setClipplingPlane(0, Planed(_startPlane0.getOrigin(), Vector3d(n2[0], n2[1], n2[2])));
        } else {
            double angleSpin = delta[0] * M_PI / 2;
            double anglePitch = delta[1] * M_PI / 2;
            applyRotation(angleSpin, anglePitch, _mouseStartLocal);
        }
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
    auto rect = GetRect();
    _width = rect.width;
    _height = rect.height;

    glViewport(0, 0, _width, _height);

//    initializeDepthPeeling();

    resizeDepthPeelingTextures();

    updateProjectionAspect();
    event.Skip();
}

void GraphicsCanvas::clearMesh3D()
{
    auto& meshVBOs = _pDrawHexMesh->getVBOs();
    meshVBOs->clear();
    if (_pDrawCrossSections) {
        auto& sectionVBOs = _pDrawCrossSections->getVBOs();
        sectionVBOs->clear();
    }
}

void GraphicsCanvas::clearModel()
{
    auto& VBOs = _pDrawModelMesh->getVBOs();
    VBOs->clear();
}

void GraphicsCanvas::initialize() 
{
	if (_initialized)
		return;

	loadShaders();
    initializeDepthPeeling();
    _initialized = true;
	return;
}

void GraphicsCanvas::initializeDepthPeeling()
{
    SetCurrent(*MainFrame::getGLContext(this));
    bool hasContext = wglGetCurrentContext();
    if (hasContext && hasVBOSupport() && _ddp_FBO_id == UINT_MAX) {
        createScreenRectPoints();

        auto rect = GetRect();
        _width = rect.width;
        _height = rect.height;

        glGenQueries(1, &_queryId);

        glGenTextures(2, _ddp_depthBufferTexId);
        glGenTextures(2, _ddp_frontColorTexId);
        glGenTextures(2, _ddp_backColorTexId);
        glGenFramebuffers(1, &_ddp_FBO_id);

        int idx = 0;
        _ddp_depthBufferTexId[idx] = createDepthBuffer(GL_TEXTURE0);
        _ddp_frontColorTexId[idx] = createColorBuffer(GL_TEXTURE1, GL_RGBA);
        _ddp_backColorTexId[idx] = createColorBuffer(GL_TEXTURE2, GL_RGBA);

        idx = 1;
        _ddp_depthBufferTexId[idx] = createDepthBuffer(GL_TEXTURE3);
        _ddp_frontColorTexId[idx] = createColorBuffer(GL_TEXTURE4, GL_RGBA);
        _ddp_backColorTexId[idx] = createColorBuffer(GL_TEXTURE5, GL_RGBA);

        _ddp_blend_backColorTexId = createColorBuffer(GL_TEXTURE6, GL_RGB);

        int level = 0;
        glBindFramebuffer(GL_FRAMEBUFFER, _ddp_FBO_id);

        int j = 0;
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_RECTANGLE, _ddp_depthBufferTexId[j], level);
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT1, GL_TEXTURE_RECTANGLE, _ddp_frontColorTexId[j], level);
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT2, GL_TEXTURE_RECTANGLE, _ddp_backColorTexId[j], level); GL_ASSERT;

        j = 1;
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT3, GL_TEXTURE_RECTANGLE, _ddp_depthBufferTexId[j], level);
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT4, GL_TEXTURE_RECTANGLE, _ddp_frontColorTexId[j], level);
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT5, GL_TEXTURE_RECTANGLE, _ddp_backColorTexId[j], level); GL_ASSERT;

        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT6, GL_TEXTURE_RECTANGLE, _ddp_blend_backColorTexId, level); GL_ASSERT;
        checkBoundFrameBuffer();
    }
}

void GraphicsCanvas::resizeDepthPeelingTextures()
{
    if (_ddp_FBO_id == UINT_MAX)
        return;

    auto rect = GetRect();
    _width = rect.width;
    _height = rect.height;

    int idx = 0;
    resizeDepthBuffer(_ddp_depthBufferTexId[idx], GL_TEXTURE0);
    resizeColorBuffer(_ddp_frontColorTexId[idx], GL_TEXTURE1, GL_RGBA);
    resizeColorBuffer(_ddp_backColorTexId[idx], GL_TEXTURE2, GL_RGBA);

    idx = 1;
    resizeDepthBuffer(_ddp_depthBufferTexId[idx], GL_TEXTURE3);
    resizeColorBuffer(_ddp_frontColorTexId[idx], GL_TEXTURE4, GL_RGBA);
    resizeColorBuffer(_ddp_backColorTexId[idx], GL_TEXTURE5, GL_RGBA);

    resizeColorBuffer(_ddp_blend_backColorTexId, GL_TEXTURE6, GL_RGB);
}

GLuint GraphicsCanvas::createColorBuffer(GLenum textureUnit, GLenum storageType)
{
    GLuint result;
    glCreateTextures(GL_TEXTURE_RECTANGLE, 1, &result); GL_ASSERT;

    resizeColorBuffer(result, textureUnit, storageType);

    return result;
}

void GraphicsCanvas::resizeColorBuffer(GLuint texId, GLenum textureUnit, GLenum storageType)
{
    glActiveTexture(textureUnit); GL_ASSERT;
    glBindTexture(GL_TEXTURE_RECTANGLE, texId); GL_ASSERT;
    glTexParameteri(GL_TEXTURE_RECTANGLE, GL_TEXTURE_MAG_FILTER, GL_NEAREST); GL_ASSERT;
    glTexParameteri(GL_TEXTURE_RECTANGLE, GL_TEXTURE_MIN_FILTER, GL_NEAREST); GL_ASSERT;
    glTexParameteri(GL_TEXTURE_RECTANGLE, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE); GL_ASSERT;
    glTexParameteri(GL_TEXTURE_RECTANGLE, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE); GL_ASSERT;
    glTexImage2D(GL_TEXTURE_RECTANGLE, 0, GL_RGBA8, _width, _height, 0, GL_RGBA, GL_FLOAT, nullptr); GL_ASSERT;
}

GLuint GraphicsCanvas::createDepthBuffer(GLenum textureUnit)
{
    GLuint result;
    glCreateTextures(GL_TEXTURE_RECTANGLE, 1, &result); GL_ASSERT;

    resizeDepthBuffer(result, textureUnit);

    return result;
}

void GraphicsCanvas::getGlDims(int& width, int& height)
{
    GLint dims[4] = { 0 };
    glGetIntegerv(GL_VIEWPORT, dims);
    width = dims[2];
    height = dims[3];
}

void GraphicsCanvas::resizeDepthBuffer(GLuint texId, GLenum textureUnit)
{
    glActiveTexture(textureUnit); GL_ASSERT;
    glBindTexture(GL_TEXTURE_RECTANGLE, texId); GL_ASSERT;
    glTexParameteri(GL_TEXTURE_RECTANGLE, GL_TEXTURE_MAG_FILTER, GL_NEAREST); GL_ASSERT;
    glTexParameteri(GL_TEXTURE_RECTANGLE, GL_TEXTURE_MIN_FILTER, GL_NEAREST); GL_ASSERT;
    glTexParameteri(GL_TEXTURE_RECTANGLE, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE); GL_ASSERT;
    glTexParameteri(GL_TEXTURE_RECTANGLE, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE); GL_ASSERT;
    glTexImage2D(GL_TEXTURE_RECTANGLE, 0, GL_RGBA32F, _width, _height, 0, GL_RGBA, GL_FLOAT, nullptr); GL_ASSERT;

}

void GraphicsCanvas::createScreenRectPoints()
{
    float min = -1.0f, max = 1.0f;
    Vector3f screenPts[] = {
        Vector3f(min, min, 0),
        Vector3f(max, min, 0),
        Vector3f(max, max, 0),

        Vector3f(min, min, 0),
        Vector3f(max, max, 0),
        Vector3f(min, max, 0),
    };

    float screenRectPts[3 * 6];
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 3; j++) {
            screenRectPts[3 * i + j] = screenPts[i][j];
        }
    }

    if (_screenVertexVboID == -1) {
        glGenBuffers(1, &_screenVertexVboID);
    }

    glBindBuffer(GL_ARRAY_BUFFER, _screenVertexVboID);
    glBufferData(GL_ARRAY_BUFFER, 3 * 6 * sizeof(float), screenRectPts, GL_STATIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
}

void GraphicsCanvas::checkBoundFrameBuffer() const
{
    GLenum result = glCheckFramebufferStatus(GL_FRAMEBUFFER);
    switch (result) {
    case GL_FRAMEBUFFER_COMPLETE:
        break;
    case GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT:
        cout << "GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT\n";
        break;
    case GL_FRAMEBUFFER_INCOMPLETE_MISSING_ATTACHMENT:
        cout << "GL_FRAMEBUFFER_INCOMPLETE_MISSING_ATTACHMENT\n";
        break;
    case GL_FRAMEBUFFER_INCOMPLETE_DRAW_BUFFER:
        cout << "GL_FRAMEBUFFER_INCOMPLETE_DRAW_BUFFER\n";
        break;
    case GL_FRAMEBUFFER_INCOMPLETE_READ_BUFFER:
        cout << "GL_FRAMEBUFFER_INCOMPLETE_READ_BUFFER\n";
        break;
    default:
        cout << "Unknown frame buffer error\n";
        break;
    }
}

void GraphicsCanvas::bindTextureRect(GLint texLoc, GLuint texId, int textureUnit)
{
    if (texLoc != -1 && textureUnit >= 0) {
        glUniform1i(texLoc, textureUnit); GL_ASSERT;
        glBindTextureUnit (textureUnit, texId);
    } else {
        assert(!"uniform location is not set");
    }
}

void GraphicsCanvas::releaseDepthPeeling()
{
    if (hasVBOSupport() && _ddp_FBO_id != UINT_MAX) {
        glDeleteFramebuffers(1, &_ddp_FBO_id);
        glDeleteTextures(2, _ddp_depthBufferTexId);
        glDeleteTextures(2, _ddp_frontColorTexId);
        glDeleteTextures(2, _ddp_backColorTexId);
        glDeleteTextures(1, &_ddp_blend_backColorTexId);

        glDeleteQueries(1, &_queryId);
        _queryId = UINT_MAX;

        _ddp_FBO_id = UINT_MAX;
        _ddp_blend_backColorTexId = UINT_MAX;
        for (int i = 0; i < 2; i++) {
            _ddp_depthBufferTexId[i] = UINT_MAX;
            _ddp_frontColorTexId[i] = UINT_MAX;
            _ddp_backColorTexId[i] = UINT_MAX;
        }
    }
}

void GraphicsCanvas::loadShaders()
{
    SetCurrent(*MainFrame::getGLContext(this));
    string path = "shaders/";
    
    /*********************************************************************************************************/
    _ddp_init_shader = createShader(path, "dual_peeling_init"); GL_ASSERT;
    _ddp_init_shader->setShaderVertexAttribName("inPosition");
    finishCreateShader(_ddp_init_shader);
    _ddp_uboIdx = glGetUniformBlockIndex(_ddp_init_shader->programID(), "UniformBufferObject");

    /*********************************************************************************************************/
    _ddp_peel_shader = createShader(path, "dual_peeling_peel"); GL_ASSERT;
    _ddp_peel_shader->setShaderVertexAttribName("inPosition");
    _ddp_peel_shader->setShaderNormalAttribName("inNormal");
    _ddp_peel_shader->setShaderColorAttribName("inColor");
    finishCreateShader(_ddp_peel_shader);
    _uboIdx = glGetUniformBlockIndex(_ddp_peel_shader->programID(), "UniformBufferObject");

    _ddp_peel_DepthBlenderSamplerLoc = glGetUniformLocation(_ddp_peel_shader->programID(), "depthBlenderSampler"); GL_ASSERT;
    _ddp_peel_FrontBlenderSamplerLoc = glGetUniformLocation(_ddp_peel_shader->programID(), "frontBlenderSampler"); GL_ASSERT;

    /*********************************************************************************************************/
    _ddp_blend_shader = createRectShader(path, "dual_peeling_blend"); GL_ASSERT;
    _ddp_blend_shader->setShaderVertexAttribName("inPosition");
    finishCreateShader(_ddp_blend_shader);

    _ddp_blend_BackColorTexLoc = glGetUniformLocation(_ddp_blend_shader->programID(), "backColorSampler"); GL_ASSERT;

    /*********************************************************************************************************/
    _ddp_final_shader = createRectShader(path, "dual_peeling_final"); GL_ASSERT;
    _ddp_final_shader->setShaderVertexAttribName("inPosition");
    finishCreateShader(_ddp_final_shader);

    _ddp_final_FrontColorTexLoc = glGetUniformLocation(_ddp_final_shader->programID(), "frontBlenderSampler"); GL_ASSERT;
    _ddp_final_BackColorTexLoc = glGetUniformLocation(_ddp_final_shader->programID(), "backBlenderSampler"); GL_ASSERT;

    /*********************************************************************************************************/
    _pDrawModelMesh->setShader(_ddp_peel_shader);
    _pDrawHexMesh->setShader(_ddp_peel_shader);

    glUseProgram(0);

    glGenBuffers(1, &_uboBufferId);
}

void GraphicsCanvas::readShader(const std::string& path, const std::string& filename, std::string& contents) const
{
    contents.clear();
    {
        ifstream in(path + filename);
        char buf[2048];
        while (in.good() && !in.eof()) {
            in.getline(buf, 2048);
            contents += string(buf) + "\n";
        }
    }

    string search("_COMMON_UBOS_");
    auto pos = contents.find(search);
    if (pos < contents.size()) {
        string replace;
        {
            string incFilename = path + "common_ubos.inc";
            ifstream in(incFilename);
            char buf[2048];
            while (in.good() && !in.eof()) {
                in.getline(buf, 2048);
                replace += string(buf) + "\n";
            }
        }
        contents = contents.replace(pos, search.length(), replace);
    }
}

shared_ptr<OGL::Shader> GraphicsCanvas::createShader(const std::string& path, const std::string& filename)
{
    shared_ptr<OGL::Shader> pResult = make_shared<OGL::Shader>();

    string srcVerts;
    readShader(path, filename + ".vert", srcVerts);
    pResult->setVertexSrc(srcVerts);

    string srcFrag;
    readShader(path, filename + ".frag", srcFrag);
    pResult->setFragmentSrc(srcFrag);

    dumpShaders(pResult, filename);
    return pResult;
}

shared_ptr<OGL::Shader> GraphicsCanvas::createRectShader(const std::string& path, const std::string& filename)
{
    shared_ptr<OGL::Shader> pResult = make_shared<OGL::Shader>();

    string srcVerts;
    readShader(path, "dual_peeling_screenRect.vert", srcVerts);
    pResult->setVertexSrc(srcVerts);

    string srcFrag;
    readShader(path, filename + ".frag", srcFrag);
    pResult->setFragmentSrc(srcFrag);

    dumpShaders(pResult, filename);
    return pResult;
}

void GraphicsCanvas::dumpShaders(const std::shared_ptr<OGL::Shader>& pShader, const std::string& filename)
{
#if 0
    auto vSrc = pShader->getVertexShaderSource();
    cout << (filename + ".vert") << "\n";
    cout << "*************************************************************\n";
    cout << vSrc;
    cout << "*************************************************************\n";

    auto fSrc = pShader->getFragmentShaderSource();
    cout << (filename + ".frag") << "\n";
    cout << "*************************************************************\n";
    cout << fSrc;
    cout << "*************************************************************\n";
#endif
}

void GraphicsCanvas::finishCreateShader(const std::shared_ptr<OGL::Shader>& pShader)
{
    pShader->load();
    pShader->bind();
    pShader->unBind();
}

void GraphicsCanvas::updateUniformBlock()
{
    glBindBuffer(GL_UNIFORM_BUFFER, _uboBufferId);
    glBufferData(GL_UNIFORM_BUFFER, sizeof(_graphicsUBO), &_graphicsUBO, GL_DYNAMIC_DRAW);
    glBindBufferBase(GL_UNIFORM_BUFFER, _uboBindingPoint, _uboBufferId);
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

void GraphicsCanvas::doPaint(wxPaintEvent& WXUNUSED(event)) {
    //    wxPaintDC dc(this); // Despite the documentation, wxPaintDC BREAKS many things and the destructor DOES NOT roll up the stack with multiple GlCanvases.
    render();
}

void GraphicsCanvas::render()
{
//    wxPaintDC dc(this);
    if (!IsShown())
        return;
    SetCurrent(*MainFrame::getGLContext(this));
 	initialize();
    auto portRect = GetSize();
    int width = portRect.GetWidth(), height = portRect.GetHeight();

    updateView();

    _ddp_peel_shader->bind();

    glUniformBlockBinding(_ddp_peel_shader->programID(), _uboIdx, _uboBindingPoint);

    updateUniformBlock();

    _ddp_peel_shader->unBind();
    subRenderOIT();

#if  DRAW_MOUSE_POSITION
    drawMousePos3D(); // Available for mouse position testing
#endif //  DRAW_MOUSE_POSITION

    SwapBuffers(); GL_ASSERT
}

void GraphicsCanvas::subRender(const std::shared_ptr<OGL::Shader>& pShader)
{
    // We're messing with the draw state in these calls and breaking renderScreenRect
    // TODO find where we're breaking it and FIX IT, then remove this hack
    glPushAttrib(0xffffffff);

    if (_pDrawCrossSections) {
        _pDrawModelMesh->setShader(pShader);
        _pDrawHexMesh->setShader(pShader);
        _pDrawCrossSections->setShader(pShader);

        _pDrawModelMesh->render();
        _pDrawCrossSections->render();

        bool twoSided = _graphicsUBO.twoSideLighting;

        _graphicsUBO.twoSideLighting = true;
        updateUniformBlock();

        _pDrawHexMesh->render();

        _graphicsUBO.twoSideLighting = twoSided;
        updateUniformBlock();
    } else {
        _pDrawModelMesh->setShader(pShader);
        _pDrawHexMesh->setShader(pShader);

        _pDrawModelMesh->render();

        bool twoSided = _graphicsUBO.twoSideLighting;

        _graphicsUBO.twoSideLighting = true;
        updateUniformBlock();

        _pDrawHexMesh->render();

        _graphicsUBO.twoSideLighting = twoSided;
        updateUniformBlock();
    }
    glPopAttrib();
}

void GraphicsCanvas::subRenderOIT()
{
    const int numPeelingPasses = 15;

    glDisable(GL_DEPTH_TEST); GL_ASSERT;
    glEnable(GL_BLEND); GL_ASSERT;

    // ---------------------------------------------------------------------
    // 1. Initialize Min-Max Depth Buffer
    // ---------------------------------------------------------------------

    // This seems to be setting every model pixel's depth to the depth of the model pixel.
    glBindFramebuffer(GL_FRAMEBUFFER, _ddp_FBO_id); GL_ASSERT;

    static bool dump1 = false;
    static bool dump2 = false;
    static bool dump3 = false;

#define REPORT_PASS_COUNT 0
#if REPORT_PASS_COUNT
    const static size_t sampleRingBufSize = 120;
    static size_t sampleRingIndex = 0;
    static size_t sampleRingBuf[sampleRingBufSize] = {};
#endif

    subRenderOITInit(dump1);

    int currId = 0, pass;
    GLuint sample_count;

    for (pass = 1; pass < numPeelingPasses; pass++) {
        currId = pass % 2;
        int prevId = 1 - currId;
        int bufOffset = currId * 3;

        glDrawBuffers(2, &g_drawBuffers[bufOffset + 1]);
        ::glClearColor(0, 0, 0, 0); // Verified with displayable color
        glClear(GL_COLOR_BUFFER_BIT);

        glDrawBuffers(1, &g_drawBuffers[bufOffset + 0]);
        ::glClearColor(-MAX_DEPTH, -MAX_DEPTH, 0, 0); // Verified with displayable color
        glClear(GL_COLOR_BUFFER_BIT); GL_ASSERT;

        // Render target 0: RG32F MAX blending
        // Render target 1: RGBA MAX blending
        // Render target 2: RGBA MAX blending
        glDrawBuffers(3, &g_drawBuffers[bufOffset + 0]);
        glEnable(GL_BLEND);
        glBlendEquation(GL_MAX);

        _ddp_peel_shader->bind();
        glUniformBlockBinding(_ddp_init_shader->programID(), _ddp_uboIdx, _uboBindingPoint);
        bindTextureRect(_ddp_peel_DepthBlenderSamplerLoc, _ddp_depthBufferTexId[prevId], 0);
        bindTextureRect(_ddp_peel_FrontBlenderSamplerLoc, _ddp_frontColorTexId[prevId], 1); GL_ASSERT;

        subRender(_ddp_peel_shader);
        _ddp_peel_shader->unBind(); GL_ASSERT;

        // Full screen pass to alpha-blend the back color
        glDrawBuffers(1, &g_drawBuffers[6]); // Bind to attachment 6 to texture unit 0
        glBlendEquation(GL_FUNC_ADD);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        GL_ASSERT;

        glBeginQuery(GL_SAMPLES_PASSED, _queryId);

        _ddp_blend_shader->bind();
        bindTextureRect(_ddp_blend_BackColorTexLoc, _ddp_backColorTexId[currId], 0);

        // This is having no effect. 
        // Clearing the buffer works. 
        // The rectangle shader has been confirmed with _ddp_blend_final - and they are using the same source file.
        // Something is wrong with the shader.
        // Looks like it's the coordinates from the vertex shader
        // The problem is that multivbo is leaving the vertex VBO state enabled, and the screen rect shader is NOT using a VBO.
        // This is causing screen rect to not draw the rectangle AFTER rendering a model.
        // It took some creative clearing and dynamic redrawing to spot this because the old image was left on the screen.
        // make screenRect verts a VBO did not fix the problem. Rect renders until we draw a model.
        drawScreenRect(_ddp_blend_shader);
        _ddp_blend_shader->unBind(); GL_ASSERT;

        glEndQuery(GL_SAMPLES_PASSED);
        glGetQueryObjectuiv(_queryId, GL_QUERY_RESULT, &sample_count);

        if (dump2) { // check buffers
            stringstream ss;
            ss << "_" << pass << "_";
            string prefix = ss.str();

            writeTexture(prefix + "front", GL_TEXTURE_RECTANGLE, _ddp_frontColorTexId[currId]);
            writeTexture(prefix + "back", GL_TEXTURE_RECTANGLE, _ddp_backColorTexId[currId]);
            writeDepthTexture(prefix + "depth", GL_TEXTURE_RECTANGLE, _ddp_depthBufferTexId[currId]);
            writeTexture(prefix + "blendBack", GL_TEXTURE_RECTANGLE, _ddp_blend_backColorTexId);
        }

        if (sample_count == 0) {
            break;
        }
        GL_ASSERT;
    }

    glDisable(GL_BLEND); GL_ASSERT;

    // ---------------------------------------------------------------------
    // 3. Final Pass
    // ---------------------------------------------------------------------
    subRenderOITFinal(currId, dump3);

#define REPORT_PASS_COUNT 0
#if REPORT_PASS_COUNT
    sampleRingBuf[sampleRingIndex++] = pass;
    sampleRingIndex = sampleRingIndex % sampleRingBufSize;

    static size_t count = 0;
    if (count++ > 1000) {
        size_t sum = 0;
        for (size_t i = 0; i < sampleRingBufSize; i++) {
            sum += sampleRingBuf[i];
        }
        double avgPasses = sum / (double)sampleRingBufSize;
        cout << "Average DDP passes: " << avgPasses << "\n";
    }
#endif
}

void GraphicsCanvas::subRenderOITInit(bool& dump1)
{
    // Prime the color buffers with transparent black - nothing
     // Render targets 1 and 2 store the front and back colors
     // Clear to 0.0 and use MAX blending to filter written color
     // At most one front color and one back color can be written every pass
    glDrawBuffers(2, &g_drawBuffers[1]); GL_ASSERT;
    ::glClearColor(0, 0, 0, 0); GL_ASSERT; // Checked with a displayable color
    glClear(GL_COLOR_BUFFER_BIT); GL_ASSERT;

    // Prime the depth buffers using the image. This sets the min/max depths for the entire model
    // Render target 0 stores (-minDepth, maxDepth, alphaMultiplier)
    glDrawBuffer(g_drawBuffers[0]); GL_ASSERT;
    ::glClearColor(-MAX_DEPTH, -MAX_DEPTH, 0, 0); GL_ASSERT; // Use the global version to store the full float. Checked with a displayable color
    glClear(GL_COLOR_BUFFER_BIT); GL_ASSERT;

    glBlendEquation(GL_MAX); GL_ASSERT;

    // This draws the model to set the depth buffers
    _ddp_init_shader->bind();
    glUniformBlockBinding(_ddp_init_shader->programID(), _ddp_uboIdx, _uboBindingPoint);
    subRender(_ddp_init_shader);
    _ddp_init_shader->unBind();

    GL_ASSERT;

    // ---------------------------------------------------------------------
    // 2. Dual Depth Peeling + Blending
    // ---------------------------------------------------------------------

    // Since we cannot blend the back colors in the geometry passes,
    // we use another render target to do the alpha blending
    glDrawBuffer(g_drawBuffers[6]); // According to docs this binds GL_COLOR_ATTACHMENT6 to location 0 
    glClearColor(_backColor);
//    ::glClearColor(0, 0, 0, 0);
    glClear(GL_COLOR_BUFFER_BIT); GL_ASSERT;

    if (dump1) {// check buffers
        writeTexture("_0_front", GL_TEXTURE_RECTANGLE, _ddp_frontColorTexId[0]);
        writeTexture("_0_back", GL_TEXTURE_RECTANGLE, _ddp_backColorTexId[0]);
        writeDepthTexture("_0_depth", GL_TEXTURE_RECTANGLE, _ddp_depthBufferTexId[0]);
        writeTexture("_0_blendBack", GL_TEXTURE_RECTANGLE, _ddp_blend_backColorTexId); // Visibly checked _backColor
    }

}

void GraphicsCanvas::subRenderOITFinal(int currId, bool& dump3)
{
    if (dump3) {// check buffers
        writeTexture("_last_front", GL_TEXTURE_RECTANGLE, _ddp_frontColorTexId[currId]);
        writeTexture("_last_back", GL_TEXTURE_RECTANGLE, _ddp_backColorTexId[currId]);
        writeTexture("_last_blendBack", GL_TEXTURE_RECTANGLE, _ddp_blend_backColorTexId);
    }
    glBindFramebuffer(GL_FRAMEBUFFER, 0); GL_ASSERT;
//    glDrawBuffer(GL_BACK); GL_ASSERT; // Not needed with wxWidgets, not sure why.

    _ddp_final_shader->bind(); GL_ASSERT;
    bindTextureRect(_ddp_final_FrontColorTexLoc, _ddp_frontColorTexId[currId], 0);
    bindTextureRect(_ddp_final_BackColorTexLoc, _ddp_blend_backColorTexId, 1);
    drawScreenRect(_ddp_final_shader); GL_ASSERT;
    _ddp_final_shader->unBind(); GL_ASSERT;

}

void GraphicsCanvas::drawScreenRect(const std::shared_ptr<OGL::Shader>& pShader)
{
    int vertSize = 3, stride = 0;

    glEnableClientState(GL_VERTEX_ARRAY); GL_ASSERT;
    glBindBuffer(GL_ARRAY_BUFFER, _screenVertexVboID);       GL_ASSERT;
    assert(pShader->getVertexLoc() != -1);
    glEnableVertexAttribArray(pShader->getVertexLoc()); GL_ASSERT;
    glVertexAttribPointer(0, 3, GL_FLOAT, 0, 0, 0); GL_ASSERT;
    glDrawArrays(GL_TRIANGLES, 0, 6); GL_ASSERT

#if 0 // This used to work
    glVertexPointer(vertSize, GL_FLOAT, stride, _screenRectPts); GL_ASSERT
    glDrawArrays(GL_TRIANGLES, 0, 6); GL_ASSERT
    glDisableClientState(GL_VERTEX_ARRAY); GL_ASSERT;
#endif
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

bool GraphicsCanvas::isClippingPlaneEnabled(int num) const
{
    switch (num) {
    case 0:
        return _graphicsUBO.clippingPlane0On == 1;
    case 1:
        return _graphicsUBO.clippingPlane1On == 1;
    default:
        return false;
    }
}

void GraphicsCanvas::setClippingPlaneEnabled(int num, bool val)
{
    switch (num) {
    case 0:
        _graphicsUBO.clippingPlane0On = val ? 1 : 0;
        break;
    case 1:
        _graphicsUBO.clippingPlane1On = val ? 1 : 0;
        break;
    default:
        break;
    }
}

void GraphicsCanvas::setClipplingPlane(int num, const Planed& pl)
{
    const auto& pt = pl.getOrigin();
    const auto& n = pl.getNormal();
    p4f origin((float)pt[0], (float)pt[1], (float)pt[2]);
    p4f normal((float)n[0], (float)n[1], (float)n[2]);
    switch (num) {
    case 0:
        _graphicsUBO.clippingPlane0Origin = origin;
        _graphicsUBO.clippingPlane0Normal = normal;
        break;
    case 1:
        _graphicsUBO.clippingPlane1Origin = origin;
        _graphicsUBO.clippingPlane1Normal = normal;
        break;
    default:
        break;
    }
}

const Planed GraphicsCanvas::getClipplingPlane(int num) const
{
    p4f o, n;
    switch (num) {
    case 0:
        o = _graphicsUBO.clippingPlane0Origin;
        n = _graphicsUBO.clippingPlane0Normal;
        break;
    case 1:
        o = _graphicsUBO.clippingPlane1Origin;
        n = _graphicsUBO.clippingPlane1Normal;
        break;
    default:
        break;
    }

    Vector3d origin(o[0], o[1], o[2]);
    Vector3d normal(n[0], n[1], n[2]);
    return Planed(origin, normal);
}

void GraphicsCanvas::setClippingMoveEnabled(bool val)
{
    _clippingMove = val;
}

void GraphicsCanvas::setClippingRotateEnabled(bool val)
{
    _clippingRotate = val;
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
    if (!_pAppData)
        return;
    const auto& model = _pAppData->getModel();
    _pDrawModelMesh->changeViewElements(model);
    _pDrawHexMesh->changeViewElements();
    if (_pDrawCrossSections)
        _pDrawCrossSections->changeViewElements();
}

void GraphicsCanvas::writeTexture(const std::string& filename, GLenum target, GLuint texId)
{
    string path = "D:/DarkSky/Projects/DistFieldHexMesh/out/";
    string fn = path + filename + ".ppm";

    int dimx = 800, dimy = 800, storageFormat;
    int miplevel = 0;

    glActiveTexture(GL_TEXTURE12); GL_ASSERT;
    glBindTexture(target, texId); GL_ASSERT;
    glGetTexLevelParameteriv(target, miplevel, GL_TEXTURE_WIDTH, &dimx); GL_ASSERT;
    glGetTexLevelParameteriv(target, miplevel, GL_TEXTURE_HEIGHT, &dimy); GL_ASSERT;
    glGetTexLevelParameteriv(target, miplevel, GL_TEXTURE_INTERNAL_FORMAT, &storageFormat); GL_ASSERT;

    size_t numElements = 0;
    int format;
    switch (storageFormat) {
    case GL_RGBA8_EXT:
    case GL_RGBA16F:
    case GL_RGBA32F:
        numElements = 4;
        format = GL_RGBA;
        break;
    case GL_RGB8:
        numElements = 3;
        format = GL_RGB;
        break;
    case GL_RG32F:
        numElements = 2;
        format = GL_RG;
        break;
    default:
        assert(!"Unexpected storage format");
        break;
    }
    size_t size = numElements * dimx * dimy;
    vector<float> buf;
    buf.resize(size);
    glGetTexImage(target, miplevel, format, GL_FLOAT, buf.data()); GL_ASSERT;

    int i, j;
    FILE* fp = fopen(fn.c_str(), "wb"); /* b - binary mode */
    (void)fprintf(fp, "P6\n%d %d\n255\n", dimx, dimy);
    for (j = dimy - 1; j >= 0; j--)
    {
        for (i = 0; i < dimx; ++i)
        {
            static unsigned char color[3];
            size_t idx = numElements * (j * dimx + i);
            int max = numElements > 3 ? 3 : numElements;
            float alpha = numElements == 4 ? buf[idx + 3] : 1.0f;
            alpha = 1;
            float alphaConv = 1 - alpha;
            color[0] = (unsigned char)((alphaConv + alpha * buf[idx + 0]) * 255);
            color[1] = (unsigned char)((alphaConv + alpha * buf[idx + 1]) * 255);
            color[2] = (unsigned char)((alphaConv + alpha * buf[idx + 2]) * 255);

            (void)fwrite(color, 1, 3, fp);
        }
    }
    (void)fclose(fp);
}

void GraphicsCanvas::writeDepthTexture(const std::string& filename, GLenum target, GLuint texId)
{
    string path = "D:/DarkSky/Projects/DistFieldHexMesh/out/";

    int dimx = 800, dimy = 800, storageFormat;
    int miplevel = 0;

    glActiveTexture(GL_TEXTURE12); GL_ASSERT;
    glBindTexture(target, texId); GL_ASSERT;
    glGetTexLevelParameteriv(target, miplevel, GL_TEXTURE_WIDTH, &dimx); GL_ASSERT;
    glGetTexLevelParameteriv(target, miplevel, GL_TEXTURE_HEIGHT, &dimy); GL_ASSERT;
    glGetTexLevelParameteriv(target, miplevel, GL_TEXTURE_INTERNAL_FORMAT, &storageFormat); GL_ASSERT;

    size_t numElements = 0;
    int format;
    switch (storageFormat) {
    case GL_RGBA16F:
    case GL_RGBA32F:
        numElements = 4;
        format = GL_RGBA;
        break;
    case GL_RGB8:
        numElements = 3;
        format = GL_RGB;
        break;
    case GL_RG32F:
        numElements = 2;
        format = GL_RG;
        break;
    default:
        assert(!"Unexpected storage format");
        break;
    }
    size_t size = numElements * dimx * dimy;
    vector<float> buf;
    buf.resize(size);
    glGetTexImage(target, miplevel, format, GL_FLOAT, buf.data()); GL_ASSERT;

    {
        string fn = path + filename + "_near.ppm";
        int i, j;
        FILE* fp = fopen(fn.c_str(), "wb"); /* b - binary mode */
        (void)fprintf(fp, "P6\n%d %d\n255\n", dimx, dimy);
        for (j = dimy - 1; j >= 0; j--)
        {
            for (i = 0; i < dimx; ++i)
            {
                static unsigned char color[3];
                size_t idx = numElements * (j * dimx + i);
                color[0] = color[1] = color[2] = (unsigned char)(fabs(buf[idx + 0]) * 255);

                (void)fwrite(color, 1, 3, fp);
            }
        }
        (void)fclose(fp);
    }

    {
        string fn = path + filename + "_far.ppm";
        int i, j;
        FILE* fp = fopen(fn.c_str(), "wb"); /* b - binary mode */
        (void)fprintf(fp, "P6\n%d %d\n255\n", dimx, dimy);
        for (j = dimy - 1; j >= 0; j--)
        {
            for (i = 0; i < dimx; ++i)
            {
                static unsigned char color[3] = { 0,0,0 };
                size_t idx = numElements * (j * dimx + i);
                color[0] = color[1] = color[2] = (unsigned char)(fabs(buf[idx + 1]) * 255);

                (void)fwrite(color, 1, 3, fp);
            }
        }
        (void)fclose(fp);
    }
}

bool GraphicsCanvas::drawSectionsEnabled() const
{
    return _drawSectionsEnabled;
}

void GraphicsCanvas::toggleDrawSections(const VolumePtr& pVolume)
{

    if (_drawSectionsEnabled) {
        if (_pDrawCrossSections)
            _pDrawCrossSections->getVBOs()->clear();
        _pDrawCrossSections = nullptr;
        _drawSectionsEnabled = false;
    } else {
        if (pVolume && pVolume->hasCrossSections()) {
            if (!_pDrawCrossSections)
                _pDrawCrossSections = make_shared<DrawCrossSectionEdges>(this);

            _drawSectionsEnabled = true;
            _pAppData->updateHexTess();
            _pDrawCrossSections->changeViewElements();
        }
    }
}

bool GraphicsCanvas::hasSections() const
{
    return _pDrawCrossSections != nullptr;
}

bool GraphicsCanvas::showSections(int axis) const
{
    if (_pDrawCrossSections) {
        switch (axis) {
        default:
            break;
        case 0:
            return _pDrawCrossSections->showX();
        case 1:
            return _pDrawCrossSections->showY();
        case 2:
            return _pDrawCrossSections->showZ();
        }
    }
    return false;
}

void GraphicsCanvas::toggleShowSections(int axis)
{
    if (_pDrawCrossSections) {
        switch (axis) {
        default:
            return;
        case 0:
            _pDrawCrossSections->toggleShowX();
            break;
        case 1:
            _pDrawCrossSections->toggleShowY();
            break;
        case 2:
            _pDrawCrossSections->toggleShowZ();
            break;
        }
        changeViewElements();
    }
}