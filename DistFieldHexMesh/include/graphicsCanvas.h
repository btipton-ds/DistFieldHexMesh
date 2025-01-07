#pragma once

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
#define GL_GLEXT_PROTOTYPES
#include <wx/wx.h>
#include <wx/glcanvas.h>
#include <rgbaColor.h>
#include <triMesh.h>
#include <meshData.h>
#include <graphicsVBORec.h>
#include <drawHexMesh.h>
#include <drawModelMesh.h>
#include <OGLMath.h>
#include <OGLMultiVboHandler.h>
#include <OGLExtensions.h>

#include <tm_vector3.h>
#include <enums.h>

namespace OGL {
class Shader;
}

namespace DFHM {

using CMesh = TriMesh::CMesh;
using CMeshPtr = TriMesh::CMeshPtr;

class AppData;
using AppDataPtr = std::shared_ptr<AppData>;

class Volume;
using VolumePtr = std::shared_ptr<Volume>;

#define USING_VULKAN 0
#if USING_VULKAN
#else
using GraphicsCanvasBase = wxGLCanvas;
#endif

#ifdef WIN32
class GraphicsCanvas : public GraphicsCanvasBase, public OGL::Extensions
#else
class GraphicsCanvas : public GraphicsCanvasBase 
#endif
{
public:
    enum View {
        VIEW_BOTTOM,
        VIEW_TOP,
        VIEW_LEFT,
        VIEW_RIGHT,
        VIEW_FRONT,
        VIEW_BACK,

    };

    struct GraphicsUBO {
        m44f modelViewX; // Model matrix is always identity, so this is the view matrix
        m44f projX;
        p3f defColor;
        float ambient = 0;
        int numLights = 0;
        p3f lightDir[8];
    };

    GraphicsCanvas(wxFrame* parent, const AppDataPtr& pAppData);
    ~GraphicsCanvas();

    void clearMesh3D();

    GraphicsUBO& getUBO();
    const GraphicsUBO& getUBO() const;

    std::shared_ptr<DrawModelMesh> getDrawModelMesh();
    const std::shared_ptr<DrawModelMesh> getDrawModelMesh() const;

    std::shared_ptr<DrawHexMesh> getDrawHexMesh();
    const std::shared_ptr<DrawHexMesh> getDrawHexMesh() const;


    void doPaint(wxPaintEvent& event);
    void setBackColor(const rgbaColor& color);

    void setView(Vector3d viewVec);
    void setLights();

    void changeViewElements();

    Vector3d NDCPointToModel(const Eigen::Vector2d& pt2d) const;
    Vector3d screenVectorToModel(const Eigen::Vector2d& v, double z) const;
    Vector3d screenVectorToModel(const Eigen::Vector3d& v) const;
    void moveOrigin(const Eigen::Vector2d& delta);
    void applyRotation(double angleSpin, double anglePitch, const Vector3d& rotationCenterLC);
    void applyScaleFactor(double scaleMult, const Eigen::Vector2d& center);

    bool showModelSharpEdges() const;
    bool toggleShowModelSharpEdges();

    bool showSharpVerts() const;
    bool toggleShowSharpVerts();

    bool showTriNormals() const;
    bool toggleShowTriNormals();

    bool showModelFaces() const;
    bool toggleShowModelFaces();

    bool showCurvature() const;
    bool toggleShowCurvature();

    bool showModelEdges() const;
    bool toggleShowModelEdges();

    bool showMeshEdges() const;
    bool toggleShowMeshEdges();

    bool showMeshFaces() const;
    bool toggleShowMeshFaces();

    bool showMeshWalls() const;
    bool toggleShowMeshWalls();

    bool showMeshBoundary() const;
    bool toggleShowMeshBoundary();

    void setShowMeshSelectedBlocks(bool val);

    void setView(View v);
    void toggleShowFace(View v);
    bool showFace(View v) const;
    void resetView();

    void onMouseLeftDown(wxMouseEvent& event);
    void onMouseLeftUp(wxMouseEvent& event);
    void onMouseMiddleDown(wxMouseEvent& event);
    void onMouseMiddleUp(wxMouseEvent& event);
    void onMouseRightDown(wxMouseEvent& event);
    void onMouseRightUp(wxMouseEvent& event);
    void onMouseMove(wxMouseEvent& event);
    void onMouseWheel(wxMouseEvent& event);
    void onSizeChange(wxSizeEvent& event);

    static Eigen::Matrix4d createTranslation(const Vector3d& delta);
    static Eigen::Matrix4d createRotation(const Vector3d& axis, double angle);
private:
    Eigen::Matrix4d cumTransform(bool withProjection) const;
    void initProjection();
    void updateProjectionAspect();
    Vector3d pointToLocal(const Vector3d& pointMC) const;

    void glClearColor(const rgbaColor& color);
    void glClearColor(GLclampf red, GLclampf green, GLclampf blue, GLclampf alpha);
    void render();
    void updateView();
    void drawMousePos3D();

    void initialize();
    void loadShaders();

    Eigen::Vector2d screenToNDC(const wxPoint& pt);
    std::shared_ptr<wxGLContext> _pContext;
    
    bool _initialized = false;
    bool _leftDown = false, _middleDown = false, _rightDown = false;
    
#define INIT_VIEW_SCALE 10
    double _viewScale = INIT_VIEW_SCALE;

    AppDataPtr _pAppData;
    std::shared_ptr<DrawHexMesh> _pDrawHexMesh;
    std::shared_ptr<DrawModelMesh> _pDrawModelMesh;
    CBoundingBox3Dd _viewBounds;
    Eigen::Vector2d _mouseStartLocNDC_2D;
    Vector3d _mouseStartLocal;
    Vector3f _mouseLoc3D;
    Eigen::Matrix4d _projAspect;
    Eigen::Matrix4d _modelView, _projection, _intitialModelView, _initialProjection;

    GraphicsUBO _graphicsUBO;
    std::shared_ptr<OGL::Shader> _phongShader;
    rgbaColor _backColor = rgbaColor(0.0f, 0.0f, 0.0f);

protected:
    wxDECLARE_EVENT_TABLE(); 
};

inline GraphicsCanvas::GraphicsUBO& GraphicsCanvas::getUBO()
{
    return _graphicsUBO;
}

inline const GraphicsCanvas::GraphicsUBO& GraphicsCanvas::getUBO() const
{
    return _graphicsUBO;
}

inline std::shared_ptr<DrawModelMesh> GraphicsCanvas::getDrawModelMesh()
{
    return _pDrawModelMesh;
}

inline const std::shared_ptr<DrawModelMesh> GraphicsCanvas::getDrawModelMesh() const
{
    return _pDrawModelMesh;
}

inline std::shared_ptr<DrawHexMesh> GraphicsCanvas::getDrawHexMesh()
{
    return _pDrawHexMesh;
}

inline const std::shared_ptr<DrawHexMesh> GraphicsCanvas::getDrawHexMesh() const
{
    return _pDrawHexMesh;
}

inline void GraphicsCanvas::setBackColor(const rgbaColor& color)
{
    _backColor = color;
}

inline bool GraphicsCanvas::showModelSharpEdges() const
{
    return _pDrawModelMesh->showSharpEdges();
}

inline bool GraphicsCanvas::showSharpVerts() const
{
    return _pDrawModelMesh->showSharpVerts();
}

inline bool GraphicsCanvas::showTriNormals() const
{
    return _pDrawModelMesh->showTriNormals();
}

inline bool GraphicsCanvas::showModelFaces() const
{
    return _pDrawModelMesh->showFaces();
}

inline bool GraphicsCanvas::showModelEdges() const
{
    return _pDrawModelMesh->showEdges();
}

inline bool GraphicsCanvas::showMeshEdges() const
{
    return _pDrawHexMesh->showEdges();
}

inline bool GraphicsCanvas::showMeshFaces() const
{
    return _pDrawHexMesh->showFaces();
}

inline bool GraphicsCanvas::showMeshWalls() const
{
    return _pDrawHexMesh->showWalls();
}

inline bool GraphicsCanvas::showMeshBoundary() const
{
    return _pDrawHexMesh->showBoundary();
}

inline void GraphicsCanvas::setShowMeshSelectedBlocks(bool val)
{
    _pDrawHexMesh->setShowSelectedBlocks(val);
}

}
