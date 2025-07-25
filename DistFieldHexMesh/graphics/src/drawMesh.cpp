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

    Copyright Robert R Tipton, 2022, all rights reserved except those granted in prior license statement.

    Dark Sky Innovative Solutions http://darkskyinnovation.com/
*/

#include<drawMesh.h>
#include <OGLShader.h>
#include <OGLMultiVboHandlerTempl.h>

using namespace std;
using namespace DFHM;


DrawMesh::DrawMesh(GraphicsCanvas* pCanvas, int numLayers)
	: _pCanvas(pCanvas)
{
	_VBOs = std::make_shared<VBORec>(numLayers);
}

DrawMesh::~DrawMesh()
{

}

size_t DrawMesh::numBytes() const
{
    size_t result = sizeof(DrawMesh);

    if (_VBOs) {
        result += sizeof(VBORec);
        result += _VBOs->_edgeVBO.numBytes();
        result += _VBOs->_faceVBO.numBytes();
    }

    return result;
}

void DrawMesh::setShader(const shared_ptr<OGL::Shader>& pShader)
{
    _VBOs->_faceVBO.setShader(pShader.get());
    _VBOs->_edgeVBO.setShader(pShader.get());
}

void DrawMesh::render()
{
    if (!_readyToDraw)
        return;

    drawEdges();
    drawFaces();
}

void DrawMesh::drawEdges()
{
    auto preDrawFunc = [this](int key) -> OGL::MultiVBO::DrawVertexColorMode {
        return preDrawEdges(key);
    };
    auto postDrawFunc = [this]() {
        postDrawEdges();
    };

    auto preTexDrawFunc = [this](unsigned int texId) -> OGL::MultiVBO::DrawVertexColorMode {
        return preTexDraw(texId);
    };

    auto postTexDrawFunc = [this]() {
        postTexDraw();
    };

    _VBOs->_edgeVBO.drawAllKeys(preDrawFunc, postDrawFunc, preTexDrawFunc, postTexDrawFunc);
}

void DrawMesh::drawFaces()
{
    _VBOs->_faceVBO.drawAllKeys(
        [this](int key) -> OGL::MultiVBO::DrawVertexColorMode {
            return preDrawFaces(key);
        },
        [this]() {
            postDrawFaces();
        },
        [this](unsigned int texId) -> OGL::MultiVBO::DrawVertexColorMode {
            return preTexDraw(texId);
        },
        [this]() {
            postTexDraw();
        }
    );
}

OGL::MultiVBO::DrawVertexColorMode DrawMesh::preDrawEdges(int key)
{
    return OGL::MultiVBO::DrawVertexColorMode::DRAW_COLOR_NONE;
}

void DrawMesh::postDrawEdges()
{

}

OGL::MultiVBO::DrawVertexColorMode DrawMesh::preDrawFaces(int key)
{
    return OGL::MultiVBO::DrawVertexColorMode::DRAW_COLOR_NONE;
}

void DrawMesh::postDrawFaces()
{

}

OGL::MultiVBO::DrawVertexColorMode DrawMesh::preTexDraw(int key)
{
    return OGL::MultiVBO::DrawVertexColorMode::DRAW_COLOR_NONE;
}

void DrawMesh::postTexDraw()
{

}
