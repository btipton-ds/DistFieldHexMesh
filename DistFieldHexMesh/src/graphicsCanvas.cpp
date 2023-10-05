#include <memory>
#include <graphicsCanvas.h>

#ifdef __WXMAC__
#include <GLUT/glut.h>
#else
#include <GL/glu.h>
#endif

#ifndef WIN32
#include <unistd.h> // FIXME: ¿This work/necessary in Windows?
//Not necessary, but if it was, it needs to be replaced by process.h AND io.h
#endif

using namespace std;
using namespace DFHM;

BEGIN_EVENT_TABLE(graphicsCanvas, wxGLCanvas)
EVT_PAINT(graphicsCanvas::doPaint)
END_EVENT_TABLE()

namespace
{
    shared_ptr<wxGLContext> g_pContext;
}

graphicsCanvas::graphicsCanvas(wxFrame* parent)
    : wxGLCanvas(parent, wxID_ANY, nullptr, wxDefaultPosition, wxDefaultSize, 0, wxT("GLCanvas"))
    , _faceVBO(GL_TRIANGLES, 10)
    , _edgeVBO(GL_LINES, 10)
{
    if (!g_pContext)
        g_pContext = make_shared<wxGLContext>(this);
}

graphicsCanvas::~graphicsCanvas()
{
}

void graphicsCanvas::doPaint(wxPaintEvent& WXUNUSED(event)) {
    render();
}

void graphicsCanvas::glClearColor(GLclampf red, GLclampf green, GLclampf blue, GLclampf alpha)
{
    ::glClearColor(red, green, blue, alpha);
}

void graphicsCanvas::glClearColor(const rgbaColor& color)
{
    ::glClearColor(
        color._rgba[0] / 255.0f,
        color._rgba[1] / 255.0f,
        color._rgba[2] / 255.0f,
        color._rgba[3] / 255.0f
    );
}

void graphicsCanvas::render()
{
    SetCurrent(*g_pContext);
    wxPaintDC(this);
    
    glClearColor(_backColor);
    glClear(GL_COLOR_BUFFER_BIT);
    glViewport(0, 0, (GLint)GetSize().x, (GLint)GetSize().y);


    glFlush();
    SwapBuffers();
}

