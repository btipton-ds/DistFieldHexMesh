#pragma once

#include <wx/wx.h>
#include <wx/glcanvas.h>
#include <rgbaColor.h>
#include <OGLMultiVboHandler.h>

namespace DFHM {

#define USING_VULKAN 0
#if USING_VULKAN
#else
using graphicsCanvasBase = wxGLCanvas;
#endif

class graphicsCanvas : public graphicsCanvasBase {
public:

    graphicsCanvas(wxFrame* parent);
    ~graphicsCanvas();

    void doPaint(wxPaintEvent& event);
    void setBackColor(const rgbaColor& color);
private:
    void glClearColor(const rgbaColor& color);
    void glClearColor(GLclampf red, GLclampf green, GLclampf blue, GLclampf alpha);

    rgbaColor _backColor = rgbaColor(0.0f, 0.0f, 0.0f);
    void render();

    COglMultiVboHandler _faceVBO, _edgeVBO;
protected:
    DECLARE_EVENT_TABLE()
};

inline void graphicsCanvas::setBackColor(const rgbaColor& color)
{
    _backColor = color;
}


}