#pragma once

#include <wx/wx.h>
#include <wx/glcanvas.h>
#include <rgbaColor.h>
#include <OGLMultiVboHandler.h>

namespace DFHM {

#define USING_VULKAN 0
#if USING_VULKAN
#else
using GraphicsCanvasBase = wxGLCanvas;
#endif

class GraphicsCanvas : public GraphicsCanvasBase {
public:

    GraphicsCanvas(wxFrame* parent);
    ~GraphicsCanvas();

    void doPaint(wxPaintEvent& event);
    void setBackColor(const rgbaColor& color);

    void beginFaceTesselation();
    // vertiIndices is index pairs into points, normals and parameters to form triangles. It's the standard OGL element index structure
    void setFaceTessellation(long entityKey, int changeNumber, const std::vector<float>& points, const std::vector<float>& normals, const std::vector<float>& parameters, const std::vector<unsigned int>& vertiIndices);
    void endFaceTesselation(bool smoothNormals);

private:
    void glClearColor(const rgbaColor& color);
    void glClearColor(GLclampf red, GLclampf green, GLclampf blue, GLclampf alpha);

    rgbaColor _backColor = rgbaColor(0.0f, 0.0f, 0.0f);
    void render();

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
inline void GraphicsCanvas::setFaceTessellation(long entityKey, int changeNumber, const std::vector<float>& points, const std::vector<float>& normals, const std::vector<float>& parameters, const std::vector<unsigned int>& vertiIndices)
{
    _faceVBO.setFaceTessellation(entityKey, changeNumber, points, normals, parameters, vertiIndices);
}

inline void GraphicsCanvas::endFaceTesselation(bool smoothNormals)
{
    _faceVBO.endFaceTesselation(smoothNormals);

}

}