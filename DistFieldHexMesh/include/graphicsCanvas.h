#pragma once

#include <memory>
#include <wx/wx.h>
#include <wx/glcanvas.h>
#include <rgbaColor.h>
#include <OglMath.h>
#include <OGLMultiVboHandler.h>
#include <OGLExtensions.h>

class COglShader;

namespace DFHM {

#define USING_VULKAN 0
#if USING_VULKAN
#else
using GraphicsCanvasBase = wxGLCanvas;
#endif

class GraphicsCanvas : public GraphicsCanvasBase, public COglExtensions 
{
public:

    GraphicsCanvas(wxFrame* parent);
    ~GraphicsCanvas();

    void doPaint(wxPaintEvent& event);
    void setBackColor(const rgbaColor& color);

    void beginFaceTesselation();
    // vertiIndices is index pairs into points, normals and parameters to form triangles. It's the standard OGL element index structure
    const COglMultiVboHandler::OGLIndices* setFaceTessellation(long entityKey, int changeNumber, const std::vector<float>& points, const std::vector<float>& normals, const std::vector<float>& parameters, const std::vector<unsigned int>& vertiIndices);
    void endFaceTesselation(bool smoothNormals);

    void beginSettingFaceElementIndices(size_t layerBitMask);
    void includeFaceElementIndices(int key, const COglMultiVboHandler::OGLIndices& batchIndices, GLuint texId = 0);
    void endSettingFaceElementIndices();

private:
    struct GraphicsUBO {
        m44f modelView;
        m44f proj;
        p3f lightDir[2];
        size_t __PAD0 = -1;
        p3f defColor;
        int numLights = 0;
        float ambient = 0;
        size_t __PAD1 = -1;
    };

    void loadShaders();
    
    void glClearColor(const rgbaColor& color);
    void glClearColor(GLclampf red, GLclampf green, GLclampf blue, GLclampf alpha);

    GraphicsUBO _graphicsUBO;
    std::shared_ptr<COglShader> _phongShader;
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
inline const COglMultiVboHandler::OGLIndices* GraphicsCanvas::setFaceTessellation(long entityKey, int changeNumber, const std::vector<float>& points, const std::vector<float>& normals, const std::vector<float>& parameters, const std::vector<unsigned int>& vertiIndices)
{
    return _faceVBO.setFaceTessellation(entityKey, changeNumber, points, normals, parameters, vertiIndices);
}

inline void GraphicsCanvas::endFaceTesselation(bool smoothNormals)
{
    _faceVBO.endFaceTesselation(smoothNormals);

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
    _faceVBO.endSettingElementVBOs();
}


}