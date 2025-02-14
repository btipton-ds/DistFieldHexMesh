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
#include <mainFrame.h>
#include <graphicsCanvas.h>
#include <graphicsDebugCanvas.h>

using namespace std;
using namespace DFHM;

BEGIN_EVENT_TABLE(GraphicsDebugCanvas, wxGLCanvas)
EVT_PAINT(GraphicsDebugCanvas::doPaint)
EVT_SIZE(GraphicsDebugCanvas::onSizeChange)
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
    if (_initialized)
        return;

    string path = "shaders/";

    SetCurrent(*MainFrame::getGLContext(this));

    _pShader = make_shared<OGL::Shader>();
    _pShader->setVertexSrcFile(path + "debug.vert");
    _pShader->setFragmentSrcFile(path + "debug.frag");

    _pShader->load();
    _pShader->bind();

    _sourceLoc = glGetUniformLocation(_pShader->programID(), "source"); GL_ASSERT;

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

    glCreateTextures(GL_TEXTURE_RECTANGLE, 1, &_texId);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_RECTANGLE, _texId);

    glTexParameteri(GL_TEXTURE_RECTANGLE, GL_TEXTURE_MAG_FILTER, GL_LINEAR); GL_ASSERT;
    glTexParameteri(GL_TEXTURE_RECTANGLE, GL_TEXTURE_MIN_FILTER, GL_LINEAR); GL_ASSERT;
    glTexParameteri(GL_TEXTURE_RECTANGLE, GL_TEXTURE_WRAP_S, GL_CLAMP); GL_ASSERT;
    glTexParameteri(GL_TEXTURE_RECTANGLE, GL_TEXTURE_WRAP_T, GL_CLAMP); GL_ASSERT;

    regenDefImage();
}

void GraphicsDebugCanvas::regenDefImage()
{
    int width, height;
    getGlDims(width, height);
    size_t dim = 60;
    size_t size = 4 * width * height;
    if (size != _pixels.size()) {
        _pixels.resize(size);

        const rgbaColor colorA = rgbaColor(0.5, 0.25, 0.25, 1);
        const rgbaColor colorB = rgbaColor(0.25, 0.5, 0.25, 1);
        rgbaColor color;
        size_t idx = 0;
        for (size_t v = 0; v < height; v++) {
            size_t row = v / dim;
            for (size_t h = 0; h < width; h++) {
                size_t col = h / dim;
                if (row % 2 == 0) {
                    color = (col % 2 == 0) ? colorA : colorB;
                }
                else {
                    color = (col % 2 == 1) ? colorA : colorB;
                }
                _pixels[idx++] = color._rgba[0];
                _pixels[idx++] = color._rgba[1];
                _pixels[idx++] = color._rgba[2];
                _pixels[idx++] = color._rgba[3];
            }
        }

        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_RECTANGLE, _texId);

        const GLint level = 0, border = 0;
        glTexImage2D(GL_TEXTURE_RECTANGLE, level, GL_RGBA8, width, height, border, GL_RGBA, GL_UNSIGNED_BYTE, _pixels.data()); GL_ASSERT;
    }

}

void GraphicsDebugCanvas::onSizeChange(wxSizeEvent& event)
{
    regenDefImage();
    event.Skip();
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

void GraphicsDebugCanvas::getGlDims(int& width, int& height)
{
    GLint dims[4] = { 0 };
    glGetIntegerv(GL_VIEWPORT, dims);
    width = dims[2];
    height = dims[3];
}

void GraphicsDebugCanvas::doPaint(wxPaintEvent& WXUNUSED(event)) {
    //    wxPaintDC dc(this); // Despite the documentation, wxPaintDC BREAKS many things and the destructor DOES NOT roll up the stack with multiple GlCanvases.
    render();
}

void GraphicsDebugCanvas::render()
{
//    wxPaintDC dc(this);
    if (!IsShown())
        return;

    SetCurrent(*MainFrame::getGLContext(this));
    initialize();

    int width, height;
    getGlDims(width, height);

    glViewport(0, 0, width, height);
    glClearColor(rgbaColor(0, 0, 0.2f, 0));
    glDisable(GL_DEPTH_TEST);
    glClear(GL_COLOR_BUFFER_BIT);

    GLuint texId = (_sourceTexId != -1) ? _sourceTexId : _texId;

    _pShader->bind();

    glActiveTexture(GL_TEXTURE0); GL_ASSERT;
    glBindTexture(GL_TEXTURE_RECTANGLE, _texId); GL_ASSERT;
    glUniform1i(_sourceLoc, 0); GL_ASSERT;

    drawScreenRect();

    _pShader->unBind(); GL_ASSERT;

    SwapBuffers(); GL_ASSERT;
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

