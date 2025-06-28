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

    Copyright Robert R Tipton, 2022, all rights reserved except those granted in prior license statement.

    Dark Sky Innovative Solutions http://darkskyinnovation.com/
*/

#include <memory>
#include <mutex>
#define GL_GLEXT_PROTOTYPES
#include <wx/wx.h>
#include <wx/glcanvas.h>
#include <rgbaColor.h>
#include <triMesh.h>
#include <meshData.h>
#include <graphicsVBORec.h>
#include <drawHexMesh.h>
#include <drawModelMesh.h>
#include <drawCrossSectionEdges.h>
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

#if INCLUDE_DEBUG_WX_FRAME
class GraphicsDebugCanvas;
#endif

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
        VIEW_BACK,
        VIEW_FRONT,
    };

    struct GraphicsUBO {
        m44f modelView; // Model matrix is always identity, so this is the view matrix
        m44f proj;
        p4f defColor;
        p4f lightDir0;
        p4f lightDir1;
        p4f lightDir2;
        p4f lightDir3;
        p4f lightDir4;
        p4f lightDir5;
        p4f lightDir6;
        p4f lightDir7;

        p4f clippingPlane0Origin;
        p4f clippingPlane0Normal;

        p4f clippingPlane1Origin;
        p4f clippingPlane1Normal;

        float ambient = 0;
        int useDefColor = 1;
        int normalShadingOn = 1;
        int twoSideLighting = 1;
        int numLights = 0;

        int clippingPlane0On = 0;
        int clippingPlane1On = 0;
    };

    GraphicsCanvas(wxFrame* parent, const AppDataPtr& pAppData);
    ~GraphicsCanvas();
    void reset();

    void clearMesh3D();
    void clearModel();

    size_t numBytes() const;

    GraphicsUBO& getUBO();
    const GraphicsUBO& getUBO() const;

    std::shared_ptr<DrawModelMesh> getDrawModelMesh();
    const std::shared_ptr<DrawModelMesh> getDrawModelMesh() const;

    std::shared_ptr<DrawHexMesh> getDrawHexMesh();
    const std::shared_ptr<DrawHexMesh> getDrawHexMesh() const;

    std::shared_ptr<DrawCrossSectionEdges> getDrawCrossSectionEdges();
    const std::shared_ptr<DrawCrossSectionEdges> getDrawCrossSectionEdges() const;

    void doPaint(wxPaintEvent& event);
    void render();
    void setBackColor(const rgbaColor& color);

    void setView(Vector3d viewVec);
    void setLights();

    bool meshSelectionEnabled() const;
    bool toggleMeshSelection();

    void changeViewElements();
    void buildHexFaceTables(const VolumePtr& pVolume, const Index3D& min, const Index3D& max, bool multiCore);
    void copyHexFaceTablesToVBOs();

    Vector3d NDCPointToModel(const Eigen::Vector2d& pt2d) const;
    Vector3d screenVectorToModel(const Eigen::Vector2d& v, double z) const;
    Vector3d screenVectorToModel(const Eigen::Vector3d& v) const;
    void moveOrigin(const Eigen::Vector2d& delta);
    void applyRotation(double angleSpin, double anglePitch, const Vector3d& rotationCenterLC);
    void applyScaleFactor(double scaleMult, const Eigen::Vector2d& center);

    bool isClippingPlaneEnabled(int num) const;
    void setClippingPlaneEnabled(int num, bool val);
    void setClipplingPlane(int num, const Planed& pl);
    const Planed getClipplingPlane(int num) const;
    void setClippingMoveEnabled(bool val);
    void setClippingRotateEnabled(bool val);

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

    bool showMeshAll() const;
    bool toggleShowMeshAll();

    bool showLayersOn() const;
    bool showLayer(int32_t layerNum) const;
    void setShowLayer(int32_t layerNum);

    bool drawSectionsEnabled() const;
    void toggleDrawSections(const VolumePtr& pVolume);

    bool hasSections() const;
    bool showSections(int axis) const;
    void toggleShowSections(int axis);

    void setShowMeshSelectedBlocks(bool val);

    void setView(View v);
    void toggleShowFace(View v);
    bool showFace(View v) const;
    void resetView();

#if INCLUDE_DEBUG_WX_FRAME
    void setDebugCanvas(GraphicsDebugCanvas* pCanvas);
#endif

    void onMouseLeftDown(wxMouseEvent& event);
    void onMouseLeftDownModel(wxMouseEvent& event, const Vector3d& startPt);
    void onMouseLeftDownMesh(wxMouseEvent& event, const Vector3d& startPt);
    void onMouseLeftDownClipping(wxMouseEvent& event);
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
    void subRender(const std::shared_ptr<OGL::Shader>& pShader);
    void updateView();
    void drawMousePos3D();

    void initialize();
    void getGlDims(int& width, int& height);

    void readShader(const std::string& path, const std::string& filename, std::string& contents) const;
    std::shared_ptr<OGL::Shader> createShader(const std::string& path, const std::string& filename);
    std::shared_ptr<OGL::Shader> createRectShader(const std::string& path, const std::string& filename);
    void dumpShaders(const std::shared_ptr<OGL::Shader>& pShader, const std::string& filename);
    void finishCreateShader(const std::shared_ptr<OGL::Shader>& pShader);

    void updateUniformBlock();

    void subRenderOIT();
    void subRenderOITInit(bool& dump1);
    void subRenderOITFinal(int currId, bool& dump3);

    void syncClippingPlane(int num);
    void initializeDepthPeeling();
    void resizeDepthPeelingTextures();
    void releaseDepthPeeling();

    GLuint createColorBuffer(GLenum textureUnit, GLenum storageType);
    void resizeColorBuffer(GLuint texId, GLenum textureUnit, GLenum storageType);
    GLuint createDepthBuffer(GLenum textureUnit);
    void resizeDepthBuffer(GLuint texId, GLenum textureUnit);

    void createScreenRectPoints();
    void drawScreenRect(const std::shared_ptr<OGL::Shader>& pShader);
    void checkBoundFrameBuffer() const;
    static void bindTextureRect(GLint texLoc, GLuint texId, int textureUnit);

    void loadShaders();
    void dumpUniformOffset() const;

    Eigen::Vector2d screenToNDC(const wxPoint& pt);

    void writeTexture(const std::string& filename, GLenum target, GLuint texId);
    void writeDepthTexture(const std::string& filename, GLenum target, GLuint texId);

    bool _initialized = false;
    bool _renderRunning = true;
    bool _meshSelection = false;
    bool _clippingMove = false;
    bool _clippingRotate = false;
    bool _leftDown = false, _middleDown = false, _rightDown = false;
    Planed _startPlane0, _startPlane1;
    
#define INIT_VIEW_SCALE 10
    double _viewScale = INIT_VIEW_SCALE;

    AppDataPtr _pAppData;
#if INCLUDE_DEBUG_WX_FRAME
    GraphicsDebugCanvas* _pDebugCanvas = nullptr;
#endif
    GLuint _width, _height;
    std::shared_ptr<DrawHexMesh> _pDrawHexMesh;
    std::shared_ptr<DrawModelMesh> _pDrawModelMesh;
    bool _drawSectionsEnabled = false;
    std::shared_ptr<DrawCrossSectionEdges> _pDrawCrossSections;
    CBoundingBox3Dd _viewBounds;
    Eigen::Vector2d _mouseStartLocNDC_2D;
    Vector3d _mouseStartLocal;
    Vector3f _mouseLoc3D;
    Eigen::Matrix4d _projAspect;
    Eigen::Matrix4d _modelView, _projection, _intitialModelView, _initialProjection;

    GraphicsUBO _graphicsUBO;
    GLuint _uboBindingPoint = 1;
    GLuint _uboBufferId = -1;
    GLuint _uboIdx = -1;
    rgbaColor _backColor;

    GLuint _queryId = UINT_MAX;
    GLuint _ddp_FBO_id = UINT_MAX;

    GLuint _ddp_uboIdx = -1;

    GLint _ddp_peel_DepthBlenderSamplerLoc = -1;
    GLint _ddp_peel_FrontBlenderSamplerLoc = -1;

    GLint _ddp_blend_BackColorTexLoc = -1;

    GLint _ddp_final_FrontColorTexLoc = -1;
    GLint _ddp_final_BackColorTexLoc = -1;

    std::shared_ptr<OGL::Shader> _ddp_init_shader;
    std::shared_ptr<OGL::Shader> _ddp_peel_shader;
    std::shared_ptr<OGL::Shader> _ddp_blend_shader;
    std::shared_ptr<OGL::Shader> _ddp_final_shader;

    GLuint _ddp_depthBufferTexId[2] = { UINT_MAX, UINT_MAX };
    GLuint _ddp_frontColorTexId[2] = { UINT_MAX, UINT_MAX };
    GLuint _ddp_backColorTexId[2] = { UINT_MAX, UINT_MAX };
    GLuint _ddp_blend_backColorTexId = UINT_MAX;

    GLuint _screenVertexVboID = -1;

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

inline bool GraphicsCanvas::meshSelectionEnabled() const
{
    return _meshSelection;
}

inline bool GraphicsCanvas::toggleMeshSelection()
{
    _meshSelection = !_meshSelection;
    return _meshSelection;
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

inline const std::shared_ptr<DrawCrossSectionEdges> GraphicsCanvas::getDrawCrossSectionEdges() const
{
    return _pDrawCrossSections;
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

inline bool GraphicsCanvas::showMeshAll() const
{
    return _pDrawHexMesh->showAll();
}


inline bool GraphicsCanvas::showMeshWalls() const
{
    return _pDrawHexMesh->showErrorWalls();
}

inline bool GraphicsCanvas::showLayersOn() const
{
    return _pDrawHexMesh->showLayersOn();
}

inline bool GraphicsCanvas::showLayer(int32_t layerNum) const
{
    return _pDrawHexMesh->showLayer(layerNum);
}

inline void GraphicsCanvas::setShowMeshSelectedBlocks(bool val)
{
    _pDrawHexMesh->setShowSelectedBlocks(val);
}

}
