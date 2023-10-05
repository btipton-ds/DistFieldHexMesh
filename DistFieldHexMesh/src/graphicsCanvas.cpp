#include <memory>
#include <GraphicsCanvas.h>
#include <OGLMultiVboHandlerTempl.h>

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

BEGIN_EVENT_TABLE(GraphicsCanvas, wxGLCanvas)
EVT_PAINT(GraphicsCanvas::doPaint)
END_EVENT_TABLE()

namespace
{
    shared_ptr<wxGLContext> g_pContext;
}

GraphicsCanvas::GraphicsCanvas(wxFrame* parent)
    : wxGLCanvas(parent, wxID_ANY, nullptr, wxDefaultPosition, wxDefaultSize, 0, wxT("GLCanvas"))
    , _faceVBO(GL_TRIANGLES, 10)
    , _edgeVBO(GL_LINES, 10)
{
    if (!g_pContext)
        g_pContext = make_shared<wxGLContext>(this);
}

GraphicsCanvas::~GraphicsCanvas()
{
}

void GraphicsCanvas::doPaint(wxPaintEvent& WXUNUSED(event)) {
    render();
}

void GraphicsCanvas::glClearColor(GLclampf red, GLclampf green, GLclampf blue, GLclampf alpha)
{
    ::glClearColor(red, green, blue, alpha);
}

void GraphicsCanvas::glClearColor(const rgbaColor& color)
{
    ::glClearColor(
        color._rgba[0] / 255.0f,
        color._rgba[1] / 255.0f,
        color._rgba[2] / 255.0f,
        color._rgba[3] / 255.0f
    );
}

void GraphicsCanvas::render()
{
    SetCurrent(*g_pContext);
    wxPaintDC(this);
    
    glClearColor(_backColor);
    glClear(GL_COLOR_BUFFER_BIT);
    glViewport(0, 0, (GLint)GetSize().x, (GLint)GetSize().y);

    auto preDraw = [](int key) -> COglMultiVBO::DrawVertexColorMode {
        return COglMultiVBO::DRAW_COLOR;
    };
    auto postDraw = []() {
    };
    auto preDrawTex = [](GLuint texId) {
    };
    auto postDrawTex = []() {
    };

    _faceVBO.drawAllKeys(preDraw, postDraw, preDrawTex, postDrawTex);
//    _edgeVBO.draw(0);

#if 0
	{
		const int POLY_SMOOTH = 0;
		const int POLY_FLAT = 1;
		const int POLY_WIRE = 2;

		if (BrepModel::usingBatchedVBOs()) {
			COglShader* shader = shaderBound ? &mat->getShader() : 0;
			bool isMetal = shader && shader->getName() == _T("metal");
			DrawPass currentDrawPass = dp.getDrawPass();

			auto preDrawFaceLambda = [this, &dp](int key /*OGL works with ints. C cast of int to DrawPhase is simplest.*/) {
				return preDrawVBO(dp, (DrawPhase)key);
				};

			auto postDrawFaceLambda = [this, &dp]() {
				this->postDrawVBO();
				};

			auto preDrawTexFaceLambda = [this, isMetal, shader](GLuint texId) {
				GLfloat pFrontColor[] = { 1.0f, 1.0f, 1.0f, 1.0f };
				GLfloat pBackColor[] = { 1.0f, 1.0f, 1.0f, 1.0f };

				if (shader)
				{
					shader->setVariable(_T("useBackFaceColor"), 1.0f); // LIMBO add int based param support
					shader->setVariable(_T("BackFaceColor"), col4f(pBackColor));
				}


				if (isMetal) {
					COglShader::glActiveTexture(GL_TEXTURE0);
					glEnable(GL_TEXTURE_2D);
					glBindTexture(GL_TEXTURE_2D, texId);
					shader->setVariablei(_T("tex0"), 0);
					shader->setVariablei(_T("useTex0"), 1);
				}


				glColor4fv(pFrontColor);
				glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, pFrontColor);
				glMaterialfv(GL_BACK, GL_AMBIENT_AND_DIFFUSE, pBackColor);
				};

			auto postDrawTexFaceLambda = [this, isMetal, shader]() {
				if (isMetal) {
					shader->setVariablei(_T("tex0"), 0);
					shader->setVariablei(_T("useTex0"), 0);
				}

				glDisable(GL_TEXTURE_2D);
				glBindTexture(GL_TEXTURE_2D, 0);
				};

			bool pushedPolyMode = false;
			int polyMode = prefs._BREPShading;

			if (polyMode == POLY_WIRE)
			{
				glPushAttrib(GL_POLYGON_BIT);
				pushedPolyMode = true;
				glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
				glLineWidth(1.0);
			}
			if (currentDrawPass == Draw_Foreground)
			{
			}
			else {
				m_multiVBO.drawAllKeys(preDrawFaceLambda, postDrawFaceLambda, preDrawTexFaceLambda, postDrawTexFaceLambda);
			}

			if (pushedPolyMode) {
				glPopAttrib();
			}

			for (auto faceId : m_facesWithExtras) {
				Face* pFace = oid_cast<Face*> (faceId);
				pFace->drawExtras(dp, prefs);
			}

		}
    }
#endif

	glFlush();
    SwapBuffers();
}

