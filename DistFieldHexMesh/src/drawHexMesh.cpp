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

#include<drawHexMesh.h>
#include <enums.h>
#include <graphicsCanvas.h>

using namespace DFHM;

DrawHexMesh::DrawHexMesh(GraphicsCanvas* pCanvas)
    : DrawMesh(pCanvas)
{
}

DrawHexMesh::~DrawHexMesh()
{

}

void DrawHexMesh::changeViewElements(const VBORec::ChangeElementsOptions& opts)
{
    auto& faceVBO = _VBOs->_faceVBO;
    auto& edgeVBO = _VBOs->_edgeVBO;

    edgeVBO.beginSettingElementIndices(0xffffffffffffffff);
    faceVBO.beginSettingElementIndices(0xffffffffffffffff);

    if (opts.showFaces && !_faceTessellations.empty()) {
        if (opts.showSelectedBlocks) {
            if (FT_ALL < _faceTessellations.size()) {
                for (auto pBlockTess : _faceTessellations[FT_ALL]) {
                    if (pBlockTess)
                        faceVBO.includeElementIndices(DS_BLOCK_ALL, pBlockTess);
                }
            }
        } else if (opts.showOuter) {
            if (FT_OUTER < _faceTessellations.size()) {
                for (auto pBlockTess : _faceTessellations[FT_OUTER]) {
                    if (pBlockTess)
                        faceVBO.includeElementIndices(DS_BLOCK_OUTER, pBlockTess);
                }
            }
        } else {
            bool blocksAdded = false;
            if (FT_MODEL_BOUNDARY < _faceTessellations.size()) {
                for (auto pBlockTess : _faceTessellations[FT_MODEL_BOUNDARY]) {
                    if (pBlockTess) {
                        faceVBO.includeElementIndices(DS_BLOCK_INNER, pBlockTess);
                        blocksAdded = true;
                    }
                }
            }

            if (FT_BLOCK_BOUNDARY < _faceTessellations.size()) {
                for (auto pBlockTess : _faceTessellations[FT_BLOCK_BOUNDARY]) {
                    if (pBlockTess) {
                        blocksAdded = true;
                        faceVBO.includeElementIndices(DS_BLOCK_BOUNDARY, pBlockTess);
                    }
                }
            }

            if (!blocksAdded) {
                if (FT_ALL < _faceTessellations.size()) {
                    for (auto pBlockTess : _faceTessellations[FT_ALL]) {
                        if (pBlockTess) {
                            blocksAdded = true;
                            faceVBO.includeElementIndices(DS_BLOCK_ALL, pBlockTess);
                        }
                    }
                }
            }
        }
    }

    if (opts.showEdges && !_edgeTessellations.empty()) {
        if (opts.showOuter && FT_OUTER < _edgeTessellations.size()) {
            for (auto pBlockTess : _edgeTessellations[FT_OUTER]) {
                if (pBlockTess)
                    edgeVBO.includeElementIndices(DS_BLOCK_OUTER, pBlockTess);
            }
        }

        if (!opts.showOuter && FT_MODEL_BOUNDARY < _edgeTessellations.size()) {
            for (auto pBlockTess : _edgeTessellations[FT_MODEL_BOUNDARY]) {
                if (pBlockTess)
                    edgeVBO.includeElementIndices(DS_BLOCK_INNER, pBlockTess);
            }
        }

        if (!opts.showOuter && FT_BLOCK_BOUNDARY < _edgeTessellations.size()) {
            for (auto pBlockTess : _edgeTessellations[FT_BLOCK_BOUNDARY]) {
                if (pBlockTess)
                    edgeVBO.includeElementIndices(DS_BLOCK_BOUNDARY, pBlockTess);
            }
        }

        if (!opts.showSelectedBlocks && FT_ALL < _edgeTessellations.size()) {
            for (auto pBlockTess : _edgeTessellations[FT_ALL]) {
                if (pBlockTess)
                    edgeVBO.includeElementIndices(DS_BLOCK_ALL, pBlockTess);
            }
        }
    }

    edgeVBO.endSettingElementIndices();
    faceVBO.endSettingElementIndices();
}

OGL::MultiVBO::DrawVertexColorMode DrawHexMesh::preDrawEdges(int key)
{
    OGL::MultiVBO::DrawVertexColorMode result = OGL::MultiVBO::DrawVertexColorMode::DRAW_COLOR_NONE;
    auto& UBO = _pCanvas->getUBO();
    const auto& opts = _pCanvas->getViewOptions();

    switch (key) {
    default:
    case DS_MODEL_EDGES:
        glLineWidth(2.0f);
        UBO.defColor = p3f(1.0f, 0.0f, 0.0f);
        break;
    case DS_MODEL_SHARP_EDGES:
        glLineWidth(1.0f);
        UBO.defColor = p3f(0.0f, 0.0f, 0);
        result = OGL::MultiVBO::DrawVertexColorMode::DRAW_COLOR;
        break;
    case DS_MODEL_NORMALS:
        glLineWidth(1.0f);
        UBO.defColor = p3f(0.0f, 0.0f, 1.0f);
        break;
    case DS_BLOCK_ALL:
    case DS_BLOCK_OUTER:
    case DS_BLOCK_INNER:
        glLineWidth(1.0f);
        UBO.defColor = p3f(0.0f, 0.0f, 0.50f);
        break;
    case DS_BLOCK_BOUNDARY:
        glLineWidth(1.0f);
        UBO.defColor = p3f(0.75f, 0, 0);
        break;
    case DS_MODEL_REF_EDGES:
        glLineWidth(0.5f);
        UBO.defColor = p3f(1.0f, 1.0f, 0);
        break;
    }
    UBO.ambient = 1.0f;
    glBufferData(GL_UNIFORM_BUFFER, sizeof(UBO), &UBO, GL_DYNAMIC_DRAW);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);

    return result;
}

void DrawHexMesh::postDrawEdges()
{

}

OGL::MultiVBO::DrawVertexColorMode DrawHexMesh::preDrawFaces(int key)
{
    OGL::MultiVBO::DrawVertexColorMode result = OGL::MultiVBO::DrawVertexColorMode::DRAW_COLOR_NONE;
    auto& UBO = _pCanvas->getUBO();
    const auto& opts = _pCanvas->getViewOptions();
    UBO.ambient = 0.2f;
    switch (key) {
    default:
    case DS_MODEL_FACES:
        UBO.defColor = p3f(0.9f, 0.9f, 1.0f);
        break;
    case DS_MODEL_CURVATURE:
        result = OGL::MultiVBO::DrawVertexColorMode::DRAW_COLOR;
        UBO.defColor = p3f(0.0f, 0.0f, 0.0f); // Must be all 0 to turn on vertex color drawing
        break;
    case DS_MODEL_SHARP_VERTS:
        UBO.defColor = p3f(1.0f, 1.0f, 0);
        break;
    case DS_BLOCK_ALL:
    case DS_BLOCK_OUTER:
        UBO.defColor = p3f(0.0f, 0.8f, 0);
        break;
    case DS_BLOCK_INNER:
        UBO.defColor = p3f(0.75f, 1, 1);
        break;
    case DS_BLOCK_BOUNDARY:
        UBO.defColor = p3f(1.0f, 0.5f, 0.5f);
        break;
    }
    glBufferData(GL_UNIFORM_BUFFER, sizeof(UBO), &UBO, GL_DYNAMIC_DRAW);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);

    return result;
}

void DrawHexMesh::postDrawFaces()
{

}

