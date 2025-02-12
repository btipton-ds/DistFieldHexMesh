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
#include <ctime>
#include <wx/filedlg.h>
#include <wx/string.h>
#include <wx/wfstream.h>
#include <filesystem>

#ifdef WIN32
#include "windows.h"
#endif // WIN32

#include <OGLShader.h>
#include <graphicsDebugCanvas.h>

using namespace std;
using namespace DFHM;

BEGIN_EVENT_TABLE(GraphicsDebugCanvas, wxGLCanvas)
EVT_PAINT(GraphicsDebugCanvas::doPaint)
END_EVENT_TABLE()

namespace
{
    int attribs[] = {
        WX_GL_DEPTH_SIZE, 16,
#if GRAPHICS_OVER_SAMPLING > 1
        WX_GL_SAMPLES, GRAPHICS_OVER_SAMPLING * GRAPHICS_OVER_SAMPLING,
#endif
        0
    };
}

GraphicsDebugCanvas::GraphicsDebugCanvas(wxFrame* parent)
    : wxGLCanvas(parent, wxID_ANY, attribs, wxDefaultPosition, wxDefaultSize, 0, wxT("GLDebugCanvas"))
    , Extensions()
{
    initialize();
}

GraphicsDebugCanvas::~GraphicsDebugCanvas()
{
    if (_texId != -1) {
        glDeleteTextures(1, &_texId);
    }
}

void GraphicsDebugCanvas::initialize()
{
    string path = "shaders/";

    _pContext = make_shared<wxGLContext>(this);
    SetCurrent(*_pContext);
    auto p = wglGetCurrentContext();
    _pShader = make_shared<OGL::Shader>();
    _pShader->setVertexSrcFile(path + "debug.vert");
    _pShader->setFragmentSrcFile(path + "debug.frag");

    _pShader->load();
    _pShader->bind();
    _pShader->unBind();

    Vector3f screenPts[] = {
        Vector3f(-1, -1, 0),
        Vector3f(1, -1, 0),
        Vector3f(1,  1, 0),

        Vector3f(-1, -1, 0),
        Vector3f(1,  1, 0),
        Vector3f(-1,  1, 0),
    };
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 3; j++) {
            _screenRectPts[3 * i + j] = screenPts[i][j];
        }
    }
}

void GraphicsDebugCanvas::glClearColor(const rgbaColor& color)
{
    ::glClearColor(
        color._rgba[0] / 255.0f,
        color._rgba[1] / 255.0f,
        color._rgba[2] / 255.0f,
        color._rgba[3] / 255.0f
    );

}

void GraphicsDebugCanvas::doPaint(wxPaintEvent& WXUNUSED(event)) {
    render();
}

void GraphicsDebugCanvas::render()
{
    cout << "Render debug\n";
    SetCurrent(*_pContext);
    initialize();

    wxPaintDC(this);

    auto portRect = GetSize();
    int width = portRect.GetWidth(), height = portRect.GetHeight();

    glClearColor(rgbaColor(0, 0, 0, 1));
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    if (_sourceTexId != -1) {
        _pShader->bind();

        auto loc = glGetUniformLocation(_pShader->programID(), "source"); GL_ASSERT;

        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, _sourceTexId);
        glUniform1i(loc, _sourceTexId);

        drawScreenRect();

        _pShader->unBind();
    }
    SwapBuffers(); GL_ASSERT

}

void GraphicsDebugCanvas::setSourceTextureId(GLuint texId)
{
    _sourceTexId = texId;
}

void GraphicsDebugCanvas::drawScreenRect()
{
    int vertSize = 3, stride = 0;

    glEnableClientState(GL_VERTEX_ARRAY); GL_ASSERT;

    glVertexPointer(vertSize, GL_FLOAT, stride, _screenRectPts); GL_ASSERT;
    glDrawArrays(GL_TRIANGLES, 0, 6); GL_ASSERT;
    glDisableClientState(GL_VERTEX_ARRAY); GL_ASSERT;
}

