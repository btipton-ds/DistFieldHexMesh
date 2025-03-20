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
#include <mutex>
#define GL_GLEXT_PROTOTYPES
#include <wx/wx.h>
#include <wx/glcanvas.h>
#include <rgbaColor.h>
#include <OGLMath.h>
#include <OGLMultiVboHandler.h>
#include <OGLExtensions.h>

#include <tm_vector3.h>
#include <enums.h>

namespace OGL {
    class Shader;
}

namespace DFHM {

#define USING_VULKAN 0
#if USING_VULKAN
#else
using GraphicsCanvasBase = wxGLCanvas;
#endif

#ifdef WIN32
class GraphicsDebugCanvas : public GraphicsCanvasBase, public OGL::Extensions
#else
class GraphicsCanvas : public GraphicsCanvasBase
#endif
{
public:
    GraphicsDebugCanvas(wxFrame* parent);
    ~GraphicsDebugCanvas();

    void setSourceTextureId(GLuint texId);
    void doPaint(wxPaintEvent& event);
    void render();
    void onSizeChange(wxSizeEvent& event);
private:
    friend class GraphicsCanvas;

    void initialize();
    void glClearColor(const rgbaColor& color);
    void drawScreenRect();
    void regenDefImage();
    static void getGlDims(int& width, int& height);

    std::shared_ptr<OGL::Shader> _pShader;

    bool _initialized = false;
    bool _ready = false;
    GLuint _texId = -1;
    GLuint _sourceTexId = -1;

    GLint _sourceLoc = -1;

    GLfloat _screenRectPts[6 * 3];

    std::vector<GLubyte> _pixels;

protected:
    wxDECLARE_EVENT_TABLE();
};
}
