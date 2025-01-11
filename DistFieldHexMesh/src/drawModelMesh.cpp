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

#include <drawModelMesh.h>
#include <OGLMultiVboHandlerTempl.h>
#include <OGLShader.h>
#include <graphicsCanvas.h>
#include <meshData.h>

using namespace DFHM;

namespace {
    enum DrawStates {
        DS_MODEL,
        DS_MODEL_CURVATURE,
        DS_MODEL_SHARP_EDGES,
        DS_MODEL_SHARP_VERTS,
        DS_MODEL_NORMALS,
        DS_BLOCK_OUTER,
        DS_MESH_INNER,
        DS_MESH_BOUNDARY,
        DS_MODEL_REF_EDGES,
        DS_MESH_ALL,
    };

}

DrawModelMesh::DrawModelMesh(GraphicsCanvas* pCanvas)
    : DrawMesh(pCanvas)
{
}

DrawModelMesh::~DrawModelMesh()
{

}

void DrawModelMesh::changeViewElements(MeshDataPtr& pData)
{
    if (!pData->isActive())
        return;

    auto& faceVBO = _VBOs->_faceVBO;
    auto& edgeVBO = _VBOs->_edgeVBO;

    if (pData->isReference()) {
        edgeVBO.includeElementIndices(DS_MODEL_REF_EDGES, pData->getAllEdgeTess());
    }
    else {
        if (_options.showFaces) {
            faceVBO.includeElementIndices(DS_MODEL_FACES, pData->getFaceTess());
            if (_options.showTriNormals)
                edgeVBO.includeElementIndices(DS_MODEL_NORMALS, pData->getNormalTess());
        }

        if (_options.showEdges) {
            if (_options.showSharpEdges) {
                edgeVBO.includeElementIndices(DS_MODEL_SHARP_EDGES, pData->getSharpEdgeTess());
                edgeVBO.includeElementIndices(DS_MODEL_SMOOTH_EDGES, pData->getSmoothEdgeTess());
            }
            else {
                edgeVBO.includeElementIndices(DS_MODEL_EDGES, pData->getAllEdgeTess());
            }
        } else if (_options.showSharpEdges) {
            edgeVBO.includeElementIndices(DS_MODEL_SHARP_EDGES, pData->getSharpEdgeTess());
        }
    }
}

OGL::MultiVBO::DrawVertexColorMode DrawModelMesh::preDrawEdges(int key)
{
    OGL::MultiVBO::DrawVertexColorMode result = OGL::MultiVBO::DrawVertexColorMode::DRAW_COLOR_NONE;
    auto& UBO = _pCanvas->getUBO();

    switch (key) {
    default:
        break;
    case DS_MODEL_EDGES:
    case DS_MODEL_SMOOTH_EDGES:
        glLineWidth(1.0f);
        UBO.defColor = p3f(0.0f, 0.0f, 0.5f);
        break;
    case DS_MODEL_SHARP_EDGES:
        glLineWidth(1.0f);
        UBO.defColor = p3f(1.0f, 0.0f, 0);
        break;
    case DS_MODEL_NORMALS:
        glLineWidth(1.0f);
        UBO.defColor = p3f(0.0f, 0.0f, 1.0f);
        break;
    case DS_MODEL_REF_EDGES:
        glLineWidth(0.5f);
        UBO.defColor = p3f(1.0f, 1.0f, 0);
        break;
    }

    if (_options.showCurvature) {
        result = OGL::MultiVBO::DrawVertexColorMode::DRAW_COLOR;
        UBO.defColor = p3f(0.0f, 0.0f, 0.0f); // Must be all 0 to turn on vertex color drawing
    }

    glBufferData(GL_UNIFORM_BUFFER, sizeof(UBO), &UBO, GL_DYNAMIC_DRAW);

    glDisable(GL_LIGHTING);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);

    return result;

}

void DrawModelMesh::postDrawEdges()
{
    glEnable(GL_LIGHTING);
}

OGL::MultiVBO::DrawVertexColorMode DrawModelMesh::preDrawFaces(int key)
{
    OGL::MultiVBO::DrawVertexColorMode result = OGL::MultiVBO::DrawVertexColorMode::DRAW_COLOR_NONE;
    auto& UBO = _pCanvas->getUBO();

    switch (key) {
    default:
    case DS_MODEL:
        UBO.defColor = p3f(0.9f, 0.9f, 1.0f);
        break;
    case DS_MODEL_SHARP_VERTS:
        UBO.defColor = p3f(1.0f, 1.0f, 0);
        break;
    }

    if (_options.showCurvature) {
        result = OGL::MultiVBO::DrawVertexColorMode::DRAW_COLOR;
        UBO.defColor = p3f(0.0f, 0.0f, 0.0f); // Must be all 0 to turn on vertex color drawing
    }

    glBufferData(GL_UNIFORM_BUFFER, sizeof(UBO), &UBO, GL_DYNAMIC_DRAW);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);

    if (_options.showSharpEdges || _options.showEdges) {
        glEnable(GL_POLYGON_OFFSET_FILL);
        glPolygonOffset(1.0f, 2.0f);
    }

    return result;
}

void DrawModelMesh::postDrawFaces()
{
    glDisable(GL_DEPTH_TEST);
    if (_options.showSharpEdges || _options.showEdges) {
        glDisable(GL_POLYGON_OFFSET_FILL);
    }
}

