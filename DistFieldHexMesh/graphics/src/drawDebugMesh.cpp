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

#include <triMesh.hpp>
#include <drawDebugMesh.h>
#include <OGLMultiVboHandlerTempl.h>
#include <OGLShader.h>
#include <graphicsCanvas.h>
#include <debugMeshData.h>

using namespace std;
using namespace DFHM;

DrawDebugMesh::DrawDebugMesh(GraphicsCanvas* pCanvas)
    : DrawMesh(pCanvas, 20)
{
}

DrawDebugMesh::~DrawDebugMesh()
{

}

void DrawDebugMesh::createTessellation(DebugMeshData& data)
{
    auto& eVBO = getVBOs(0)->_edgeVBO;
    auto& fVBO = getVBOs(0)->_faceVBO;

    data.setEdgeTess(nullptr);
    eVBO.clear();
    fVBO.clear();

    eVBO.beginEdgeTesselation();
    fVBO.beginFaceTesselation();

    vector<float> edgePts;
    vector<unsigned int> edgeIndices;
    data.getGLEdges(edgePts, edgeIndices);
    if (!edgePts.empty() && !edgeIndices.empty()) {

        auto tess = eVBO.setEdgeSegTessellation(DS_DEBUG_MESH_EDGES, 0, 0, edgePts, edgeIndices);
        data.setEdgeTess(tess);
    }
    eVBO.endEdgeTesselation();
    fVBO.endFaceTesselation(false);
}


void DrawDebugMesh::changeViewElements(const DebugMeshData& data)
{
    auto& faceVBO = getVBOs(0)->_faceVBO;
    auto& edgeVBO = getVBOs(0)->_edgeVBO;

    faceVBO.beginSettingElementIndices(0xffffffffffffffff);
    edgeVBO.beginSettingElementIndices(0xffffffffffffffff);

    edgeVBO.includeElementIndices(DS_DEBUG_MESH_EDGES, data.getEdgeTess());

    faceVBO.endSettingElementIndices();
    edgeVBO.endSettingElementIndices();
}

OGL::MultiVBO::DrawVertexColorMode DrawDebugMesh::preDrawEdges(int key)
{
    OGL::MultiVBO::DrawVertexColorMode result = OGL::MultiVBO::DrawVertexColorMode::DRAW_COLOR_NONE;
    auto& UBO = _pCanvas->getUBO();
    UBO.useDefColor = 1;
    _priorNormalShadingOn = UBO.normalShadingOn;
    UBO.normalShadingOn = 0;

    switch (key) {
    default:
        break;
    case DS_DEBUG_MESH_EDGES:
        glLineWidth(1.0f);
        UBO.defColor = p3f(1.0f, 0.0f, 0.0f);
        break;
    }


    glBufferData(GL_UNIFORM_BUFFER, sizeof(UBO), &UBO, GL_DYNAMIC_DRAW);

    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);

    return result;

}

void DrawDebugMesh::postDrawEdges()
{
    auto& UBO = _pCanvas->getUBO();
    UBO.normalShadingOn = _priorNormalShadingOn ? 1 : 0;
}

OGL::MultiVBO::DrawVertexColorMode DrawDebugMesh::preDrawFaces(int key)
{
    OGL::MultiVBO::DrawVertexColorMode result = OGL::MultiVBO::DrawVertexColorMode::DRAW_COLOR_NONE;
    return result;
}

void DrawDebugMesh::postDrawFaces()
{
}

