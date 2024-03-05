#pragma once

#include <memory>
#define GL_GLEXT_PROTOTYPES
#include <wx/wx.h>
#include <wx/glcanvas.h>
#include <rgbaColor.h>
#include <triMesh.h>
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

    GraphicsCanvas(wxFrame* parent, const AppDataPtr& pAppData);
    ~GraphicsCanvas();

    void doPaint(wxPaintEvent& event);
    void setBackColor(const rgbaColor& color);

    void beginFaceTesselation(bool useModel);
    // vertiIndices is index pairs into points, normals and parameters to form triangles. It's the standard OGL element index structure
    const OGLIndices* setFaceTessellation(const CMeshPtr& pMesh);
    void endFaceTesselation(const OGLIndices* pTriTess, bool smoothNormals);
    void endFaceTesselation(const std::vector<std::vector<const OGLIndices*>>& faceTess);

    void beginEdgeTesselation(bool useModel);
    // vertiIndices is index pairs into points, normals and parameters to form triangles. It's the standard OGL element index structure
    const OGLIndices* setEdgeSegTessellation(long entityKey, int changeNumber, const std::vector<float>& points, const std::vector<unsigned int>& indices);
    const OGLIndices* setEdgeSegTessellation(const CMeshPtr& pMesh);
    void endEdgeTesselation(const OGLIndices* pSharpEdgeTess, const OGLIndices* pNormalTess);
    void endEdgeTesselation(const std::vector<std::vector<const OGLIndices*>>& edgeTess);

    Vector3d screenPointToModel(const Eigen::Vector2d& pt2d) const;
    Vector3d screenVectorToModel(const Eigen::Vector2d& v, double z) const;
    Vector3d screenVectorToModel(const Eigen::Vector3d& v) const;
    void moveOrigin(const Vector3d& delta);
    void applyRotation(double angle, const Vector3d& rotationCenter, const Vector3d& rotationAxis);

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

    void onMouseLeftDown(wxMouseEvent& event);
    void onMouseLeftUp(wxMouseEvent& event);
    void onMouseMiddleDown(wxMouseEvent& event);
    void onMouseMiddleUp(wxMouseEvent& event);
    void onMouseRightDown(wxMouseEvent& event);
    void onMouseRightUp(wxMouseEvent& event);
    void onMouseMove(wxMouseEvent& event);
    void onMouseWheel(wxMouseEvent& event);

private:
    struct GraphicsUBO {
        m44f modelView;
        m44f proj;
        p3f defColor;
        float ambient = 0;
        int numLights = 0;
        p3f lightDir[8];
    };

    Eigen::Matrix4d cumTransform(bool withProjection) const;
    Eigen::Matrix4d getProjection() const;
    void glClearColor(const rgbaColor& color);
    void glClearColor(GLclampf red, GLclampf green, GLclampf blue, GLclampf alpha);
    void render();
    void updateView();
    void changeFaceViewElements();
    void changeEdgeViewElements();
    void changeFaceViewElements(bool isModel);
    void changeEdgeViewElements(bool isModel);
    void drawMousePos3D();
    void drawFaces();
    void drawEdges();

    std::shared_ptr<wxGLContext> _pContext;
    
    const AppDataPtr _pAppData;
    bool _initialized = false;

    bool 
        _showSharpEdges = false, 
        _showSharpVerts = false, 
        _showTriNormals = false, 
        _showEdges = true, 
        _showFaces = true, 
        _showOuter = true,
        _showCurvature = false;
    bool _leftDown = false, _middleDown = false, _rightDown = false;
    void initialize();
    void loadShaders();
    static Eigen::Matrix4d createTranslation(const Vector3d& delta);

    Eigen::Vector2d calMouseLoc(const wxPoint& pt);
    
    double _viewScale = 1;
    Eigen::Vector2d _mouseStartLoc2D;
    Vector3d _origin, _mouseStartLoc3D;
    Vector3f _mouseLoc3D;
    Eigen::Matrix4d _rotToGl, _trans, _intitialTrans;

    GraphicsUBO _graphicsUBO;
    std::shared_ptr<COglShader> _phongShader;
    rgbaColor _backColor = rgbaColor(0.0f, 0.0f, 0.0f);

    struct VBORec {
        VBORec();
        const OGLIndices
            * _pTriTess = nullptr,
            * _pSharpVertTess = nullptr,
            * _pSharpEdgeTess = nullptr,
            * _pNormalTess = nullptr;
        std::vector<std::vector<const OGLIndices*>> _faceTessellations, _edgeTessellations;

        COglMultiVboHandler _faceVBO, _edgeVBO;
    };

    std::shared_ptr<VBORec> _modelVBOs, _meshVBOs, _activeVBOs;

protected:
    DECLARE_EVENT_TABLE()
};

inline void GraphicsCanvas::setBackColor(const rgbaColor& color)
{
    _backColor = color;
}

inline void GraphicsCanvas::beginFaceTesselation(bool useModel)
{
    _activeVBOs = useModel ? _modelVBOs : _meshVBOs;
    _activeVBOs->_faceVBO.beginFaceTesselation();
}

inline void GraphicsCanvas::endFaceTesselation(const OGLIndices* pTriTess, bool smoothNormals)
{
    _activeVBOs->_faceVBO.endFaceTesselation(smoothNormals);
    _activeVBOs->_pTriTess = pTriTess;
    _activeVBOs->_faceTessellations.clear();

    changeFaceViewElements();
}

inline void GraphicsCanvas::endFaceTesselation(const std::vector<std::vector<const OGLIndices*>>& faceTess)
{
    _activeVBOs->_faceVBO.endFaceTesselation(false);
    _activeVBOs->_faceTessellations = faceTess;

    changeFaceViewElements();
}

inline void GraphicsCanvas::beginEdgeTesselation(bool useModel)
{
    _activeVBOs = useModel ? _modelVBOs : _meshVBOs;
    _activeVBOs->_edgeVBO.beginEdgeTesselation();
}

inline void GraphicsCanvas::endEdgeTesselation(const OGLIndices* pSharpEdgeTess, const OGLIndices* pNormalTess)
{
    _activeVBOs->_edgeVBO.endEdgeTesselation();

    _activeVBOs->_pSharpEdgeTess = pSharpEdgeTess;
    _activeVBOs->_pNormalTess = pNormalTess;
    _activeVBOs->_edgeTessellations.clear();

    changeEdgeViewElements();
}

inline void GraphicsCanvas::endEdgeTesselation(const std::vector<std::vector<const OGLIndices*>>& edgeTess)
{
    _activeVBOs->_edgeVBO.endEdgeTesselation();

    _activeVBOs->_edgeTessellations = edgeTess;

    changeEdgeViewElements();
}

inline bool GraphicsCanvas::showSharpEdges() const
{
    return _showSharpEdges;
}

inline bool GraphicsCanvas::showSharpVerts() const
{
    return _showSharpVerts;
}

inline bool GraphicsCanvas::showTriNormals() const
{
    return _showTriNormals;
}

inline bool GraphicsCanvas::showFaces() const
{
    return _showFaces;
}

inline bool GraphicsCanvas::showCurvature() const
{
    return _showCurvature;
}

inline bool GraphicsCanvas::showEdges() const
{
    return _showEdges;
}

inline bool GraphicsCanvas::showOuter() const
{
    return _showOuter;
}

}
