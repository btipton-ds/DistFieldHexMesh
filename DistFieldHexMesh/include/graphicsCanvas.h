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
#include <OGLMath.h>
#include <OGLMultiVboHandler.h>
#include <OGLExtensions.h>

#include <tm_vector3.h>
#include <enums.h>

class COglShader;

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
class GraphicsCanvas : public GraphicsCanvasBase, public COglExtensions 
#else
class GraphicsCanvas : public GraphicsCanvasBase 
#endif
{
public:
    using OGLIndices = COglMultiVboHandler::OGLIndices;

    enum View {
        VIEW_FRONT,
        VIEW_BACK,
        VIEW_TOP,
        VIEW_BOTTOM,
        VIEW_LEFT,
        VIEW_RIGHT,
    };

    GraphicsCanvas(wxFrame* parent, const AppDataPtr& pAppData);
    ~GraphicsCanvas();

    void preDestroy();
    void clearMesh3D();
    void registerMeshData(MeshDataPtr& pMeshData);
    const VBORec::ChangeElementsOptions& getViewOptions() const;

    void doPaint(wxPaintEvent& event);
    void setBackColor(const rgbaColor& color);

    void setView(Vector3d viewVec);
    void setLights();

    void beginFaceTesselation(bool useModel);
    // vertiIndices is index pairs into points, normals and parameters to form triangles. It's the standard OGL element index structure
    const OGLIndices* setFaceTessellation(const CMeshPtr& pMesh);
    void endFaceTesselation(const OGLIndices* pTriTess, const OGLIndices* pSharpVertTess, bool smoothNormals);
    void endFaceTesselation(const std::vector<std::vector<const OGLIndices*>>& faceTess);

    void beginEdgeTesselation(bool useModel);
    // vertiIndices is index pairs into points, normals and parameters to form triangles. It's the standard OGL element index structure
    const OGLIndices* setEdgeSegTessellation(long entityKey, int changeNumber, const std::vector<float>& points, const std::vector<unsigned int>& indices);
    const OGLIndices* setEdgeSegTessellation(const CMeshPtr& pMesh);
    void endEdgeTesselation(const OGLIndices* pSharpEdgeTess, const OGLIndices* pNormalTess);
    void endEdgeTesselation(const std::vector<std::vector<const OGLIndices*>>& edgeTess);

    void changeFaceViewElements();
    void changeEdgeViewElements();

    Vector3d NDCPointToModel(const Eigen::Vector2d& pt2d) const;
    Vector3d screenVectorToModel(const Eigen::Vector2d& v, double z) const;
    Vector3d screenVectorToModel(const Eigen::Vector3d& v) const;
    void moveOrigin(const Eigen::Vector2d& delta);
    void applyRotation(double angleSpin, double anglePitch, const Vector3d& rotationCenterLC);
    void applyScaleFactor(double scaleMult, const Eigen::Vector2d& center);

    bool showSharpEdges() const;
    bool toggleShowSharpEdges();

    bool showSharpVerts() const;
    bool toggleShowSharpVerts();

    bool showTriNormals() const;
    bool toggleShowTriNormals();

    bool showFaces() const;
    bool toggleShowFaces();

    bool showCurvature() const;
    bool toggleShowCurvature();

    bool showEdges() const;
    bool toggleShowEdges();

    bool showOuter() const;
    bool toggleShowOuter();

    bool showModelBoundary() const;
    bool toggleShowModelBoundary();

    void setShowSelectedBlocks(bool val);

    void setView(View v);
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

private:
    struct GraphicsUBO {
        m44f modelViewX; // Model matrix is always identity, so this is the view matrix
        m44f projX;
        p3f defColor;
        float ambient = 0;
        int numLights = 0;
        p3f lightDir[8];
    };

    Eigen::Matrix4d cumTransform(bool withProjection) const;
    void initProjection();
    void updateProjectionAspect();
    Vector3d pointToLocal(const Vector3d& pointMC) const;

    void glClearColor(const rgbaColor& color);
    void glClearColor(GLclampf red, GLclampf green, GLclampf blue, GLclampf alpha);
    void render();
    void updateView();
    void drawMousePos3D();
    void drawFaces();
    void drawEdges();

    void initialize();
    void loadShaders();
    static Eigen::Matrix4d createTranslation(const Vector3d& delta);

    Eigen::Vector2d screenToNDC(const wxPoint& pt);
    std::shared_ptr<wxGLContext> _pContext;
    
    bool _initialized = false;
    VBORec::ChangeElementsOptions _viewOptions;
    bool _leftDown = false, _middleDown = false, _rightDown = false;
    
    double _viewScale = 2;

    AppDataPtr _pAppData;
    CBoundingBox3Dd _viewBounds;
    Eigen::Vector2d _mouseStartLocNDC_2D;
    Vector3d _mouseStartModel;
    Vector3f _mouseLoc3D;
    Eigen::Matrix4d _projAspect;
    Eigen::Matrix4d _modelView, _projection, _intitialModelView, _initialProjection;

    GraphicsUBO _graphicsUBO;
    std::shared_ptr<COglShader> _phongShader;
    rgbaColor _backColor = rgbaColor(0.0f, 0.0f, 0.0f);

    std::shared_ptr<VBORec> _modelVBOs, _meshVBOs, _activeVBOs;

protected:
    wxDECLARE_EVENT_TABLE(); 
};

inline void GraphicsCanvas::preDestroy()
{
    _pAppData = nullptr;
}

inline void GraphicsCanvas::setBackColor(const rgbaColor& color)
{
    _backColor = color;
}

inline const VBORec::ChangeElementsOptions& GraphicsCanvas::getViewOptions() const
{
    return _viewOptions;
}

inline bool GraphicsCanvas::showSharpEdges() const
{
    return _viewOptions.showSharpEdges;
}

inline bool GraphicsCanvas::showSharpVerts() const
{
    return _viewOptions.showSharpVerts;
}

inline bool GraphicsCanvas::showTriNormals() const
{
    return _viewOptions.showTriNormals;
}

inline bool GraphicsCanvas::showFaces() const
{
    return _viewOptions.showFaces;
}

inline bool GraphicsCanvas::showCurvature() const
{
    return _viewOptions.showCurvature;
}

inline bool GraphicsCanvas::showEdges() const
{
    return _viewOptions.showEdges;
}

inline bool GraphicsCanvas::showOuter() const
{
    return _viewOptions.showOuter;
}

inline bool GraphicsCanvas::showModelBoundary() const
{
    return _viewOptions.showModelBoundary;
}

inline void GraphicsCanvas::setShowSelectedBlocks(bool val)
{
    _viewOptions.showSelectedBlocks = val;
}

}
