#pragma once

#include <memory>
#define GL_GLEXT_PROTOTYPES
#include <wx/wx.h>
#include <wx/glcanvas.h>
#include <rgbaColor.h>
#include <OGLMath.h>
#include <OGLMultiVboHandler.h>
#include <OGLExtensions.h>

#include <tm_vector3.h>

class COglShader;

namespace DFHM {

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
    enum DrawStates {
        DS_MODEL = 0,
        DS_BLOCK_MESH = 10,
    };
    enum DrawSubStates {
        DSS_OUTER = 0,
        DSS_INNER = 1,
    };
    GraphicsCanvas(wxFrame* parent);
    ~GraphicsCanvas();

    void doPaint(wxPaintEvent& event);
    void onSize(wxSizeEvent& event);
    void setBackColor(const rgbaColor& color);

    void beginFaceTesselation();
    // vertiIndices is index pairs into points, normals and parameters to form triangles. It's the standard OGL element index structure
    const COglMultiVboHandler::OGLIndices* setFaceTessellation(long entityKey, int changeNumber, const std::vector<float>& points, const std::vector<float>& normals, const std::vector<float>& parameters, const std::vector<unsigned int>& vertiIndices);
    void endFaceTesselation(const COglMultiVboHandler::OGLIndices* pTriTess, bool smoothNormals);
    void endFaceTesselation(const COglMultiVboHandler::OGLIndices* pTriTess, const std::vector<std::vector<const COglMultiVboHandler::OGLIndices*>>& faceTess, bool smoothNormals);

    void beginSettingFaceElementIndices(size_t layerBitMask);
    void includeFaceElementIndices(int key, const COglMultiVboHandler::OGLIndices& batchIndices, GLuint texId = 0);
    void endSettingFaceElementIndices();

    void beginEdgeTesselation();
    // vertiIndices is index pairs into points, normals and parameters to form triangles. It's the standard OGL element index structure
    const COglMultiVboHandler::OGLIndices* setEdgeSegTessellation(long entityKey, int changeNumber, const std::vector<float>& points, const std::vector<int>& indices);
    void endEdgeTesselation(const COglMultiVboHandler::OGLIndices* pSharpEdgeTess, const COglMultiVboHandler::OGLIndices* pNormalTess);
    void endEdgeTesselation(const COglMultiVboHandler::OGLIndices* pSharpEdgeTess, const COglMultiVboHandler::OGLIndices* pNormalTess,
        const std::vector<std::vector<const COglMultiVboHandler::OGLIndices*>>& edgeTess);

    void setViewOrigin(const Vector3d& origin);
    void setViewScale(double scale);
    void setViewEulerAnglesRad(double az, double el);
    void getViewOrigin(Vector3d& origin);
    void getViewScale(double& scale);
    void getViewEulerAnglesRad(double& az, double& el);

    bool showSharpEdges() const;
    bool toggleShowSharpEdges();

    bool showTriNormals() const;
    bool toggleShowTriNormals();

    bool showFaces() const;
    bool toggleShowFaces();

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

private:
    struct GraphicsUBO {
        m44f modelView;
        m44f proj;
        p3f defColor;
        float ambient = 0;
        int numLights = 0;
        p3f lightDir[8];
    };

    void glClearColor(const rgbaColor& color);
    void glClearColor(GLclampf red, GLclampf green, GLclampf blue, GLclampf alpha);
    void render();
    void updateView();
    void changeFaceViewElements();
    void changeEdgeViewElements();
    void drawFaces();
    void drawEdges();

    std::shared_ptr<wxGLContext> _pContext;
    
    bool _initialized = false;

    bool _showSharpEdges = false, _showTriNormals = false, _showEdges = true, _showFaces = true, _showOuter = true;
    bool _leftDown = false, _middleDown = false, _rightDown = false;
    double _initAzRad, _initElRad;
    void initialize();
    void loadShaders();

    Eigen::Vector2d _mouseStartLoc;
    Eigen::Vector2d calMouseLoc(const wxPoint& pt);
    
    Vector3d _viewOrigin = Vector3d(0, 0, 0);
    double _viewScale = 1, _viewAzRad = 0, _viewElRad = 0;

    GraphicsUBO _graphicsUBO;
    std::shared_ptr<COglShader> _phongShader;
    rgbaColor _backColor = rgbaColor(0.0f, 0.0f, 0.0f);

    const COglMultiVboHandler::OGLIndices
        *_pTriTess = nullptr,
        *_pSharpEdgeTess = nullptr,
        *_pNormalTess = nullptr;
    std::vector<std::vector<const COglMultiVboHandler::OGLIndices*>> _faceTessellations, _edgeTessellations;

    COglMultiVboHandler _faceVBO, _edgeVBO;
protected:
    DECLARE_EVENT_TABLE()
};

inline void GraphicsCanvas::setBackColor(const rgbaColor& color)
{
    _backColor = color;
}

inline void GraphicsCanvas::beginFaceTesselation()
{
    _faceVBO.beginFaceTesselation();
}

// vertiIndices is index pairs into points, normals and parameters to form triangles. It's the standard OGL element index structure
inline const COglMultiVboHandler::OGLIndices* GraphicsCanvas::setFaceTessellation(long entityKey, int changeNumber, const std::vector<float>& points, const std::vector<float>& normals, const std::vector<float>& parameters, const std::vector<unsigned int>& vertiIndices)
{
    return _faceVBO.setFaceTessellation(entityKey, changeNumber, points, normals, parameters, vertiIndices);
}

inline void GraphicsCanvas::endFaceTesselation(const COglMultiVboHandler::OGLIndices* pTriTess, bool smoothNormals)
{
    _faceVBO.endFaceTesselation(smoothNormals);
    _pTriTess = pTriTess;
    _faceTessellations.clear();

    changeFaceViewElements();
}

inline void GraphicsCanvas::endFaceTesselation(const COglMultiVboHandler::OGLIndices* pTriTess, const std::vector<std::vector<const COglMultiVboHandler::OGLIndices*>>& faceTess, bool smoothNormals)
{
    _faceVBO.endFaceTesselation(smoothNormals);
    _pTriTess = pTriTess;
    _faceTessellations = faceTess;

    changeFaceViewElements();
}

inline void GraphicsCanvas::beginSettingFaceElementIndices(size_t layerBitMask)
{
    _faceVBO.beginSettingElementIndices(layerBitMask);
}

inline void GraphicsCanvas::includeFaceElementIndices(int key, const COglMultiVboHandler::OGLIndices& batchIndices, GLuint texId)
{
    _faceVBO.includeElementIndices(key, batchIndices, texId);
}

inline void GraphicsCanvas::endSettingFaceElementIndices()
{
    _faceVBO.endSettingElementIndices();
}

inline void GraphicsCanvas::beginEdgeTesselation()
{
    _edgeVBO.beginEdgeTesselation();
}

// vertiIndices is index pairs into points, normals and parameters to form triangles. It's the standard OGL element index structure
inline const COglMultiVboHandler::OGLIndices* GraphicsCanvas::setEdgeSegTessellation(long entityKey, int changeNumber, const std::vector<float>& points, const std::vector<int>& indices)
{
    return _edgeVBO.setEdgeSegTessellation(entityKey, changeNumber, points, indices);
}

inline void GraphicsCanvas::endEdgeTesselation(const COglMultiVboHandler::OGLIndices* pSharpEdgeTess, const COglMultiVboHandler::OGLIndices* pNormalTess)
{
    _edgeVBO.endEdgeTesselation();

    _pSharpEdgeTess = pSharpEdgeTess;
    _pNormalTess = pNormalTess;
    _edgeTessellations.clear();

    changeEdgeViewElements();
}

inline void GraphicsCanvas::endEdgeTesselation(const COglMultiVboHandler::OGLIndices* pSharpEdgeTess, const COglMultiVboHandler::OGLIndices* pNormalTess,
    const std::vector<std::vector<const COglMultiVboHandler::OGLIndices*>>& edgeTess)
{
    _edgeVBO.endEdgeTesselation();

    _pSharpEdgeTess = pSharpEdgeTess;
    _pNormalTess = pNormalTess;
    _edgeTessellations = edgeTess;

    changeEdgeViewElements();
}

inline void GraphicsCanvas::setViewOrigin(const Vector3d& origin)
{
    _viewOrigin = origin;
}

inline void GraphicsCanvas::setViewScale(double scale)
{
    _viewScale = scale;
}

inline void GraphicsCanvas::setViewEulerAnglesRad(double az, double el)
{
    _viewAzRad = az;
    _viewElRad = el;
}

inline void GraphicsCanvas::getViewOrigin(Vector3d& origin)
{
    origin = _viewOrigin;
}

inline void GraphicsCanvas::getViewScale(double& scale)
{
    scale = _viewScale;
}

inline void GraphicsCanvas::getViewEulerAnglesRad(double& az, double& el)
{
    az = _viewAzRad;
    el = _viewElRad;
}

inline bool GraphicsCanvas::showSharpEdges() const
{
    return _showSharpEdges;
}

inline bool GraphicsCanvas::showTriNormals() const
{
    return _showTriNormals;
}

inline bool GraphicsCanvas::showFaces() const
{
    return _showFaces;
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
