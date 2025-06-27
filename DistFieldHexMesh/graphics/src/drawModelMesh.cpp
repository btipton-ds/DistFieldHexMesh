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
#include <drawModelMesh.h>
#include <OGLMultiVboHandlerTempl.h>
#include <OGLShader.h>
#include <graphicsCanvas.h>
#include <meshData.h>
#include <model.h>
#include <splitParams.h>

using namespace std;
using namespace DFHM;

DrawModelMesh::DrawModelMesh(GraphicsCanvas* pCanvas)
    : DrawMesh(pCanvas, 20)
{
}

DrawModelMesh::~DrawModelMesh()
{

}

void DrawModelMesh::createFaceTessellation(const MeshDataPtr& pData)
{
    auto pMesh = pData->getPolyMesh();
    auto tess = createFaceTessellation(pMesh);
    pData->setFaceTess(tess);
}

OGL::IndicesPtr DrawModelMesh::createFaceTessellation(const PolyMeshPtr& pMesh)
{
    vector<float> points, normals;
    vector<unsigned int> vertIndices;
    pMesh->getGlTriPoints(points);
    pMesh->getGlTriNormals(normals);
    pMesh->getGlTriIndices(vertIndices);

    vector<float> parameters;
    parameters.resize((points.size() * 3) / 2, 0); // Must match size of points with dim 2 instead of 3, but not used

    vector<float> colors;
    auto meshId = 1;// pMesh->getId();
    auto changeNumber = 0; // pMesh->getChangeNumber();

    auto& VBOs = getVBOs();
    auto& faceVBO = VBOs->_faceVBO;
    auto tess = faceVBO.setFaceTessellation(meshId, changeNumber, points, normals, parameters, colors, vertIndices);
    return tess;
}

void DrawModelMesh::createEdgeTessellation(const SplittingParams& params, const MeshDataPtr& pData)
{
    const auto sinSharpAngle = params.getSinSharpAngle();
    auto colorFunc = [](float curvature, float rgb[3])->bool {
        rgbaColor c = curvatureToColor(curvature);
        for (int i = 0; i < 3; i++)
            rgb[i] = c._rgba[i] / 255.0f;
        return true;
    };

    vector<float> points, colors;
    vector<unsigned int> indices, sharpIndices, smoothIndices;


    bool includeSmooth = true;
    auto pPolyMesh = pData->getPolyMesh();
    pPolyMesh->getGlEdges(colorFunc, includeSmooth, points, colors, sinSharpAngle, sharpIndices, smoothIndices);

    indices = smoothIndices;
    indices.insert(indices.end(), sharpIndices.begin(), sharpIndices.end());

    auto& VBOs = getVBOs();

    auto meshId = pData->getId();
    auto changeNumber = 1;
    auto& edgeVBO = VBOs->_edgeVBO;
    auto allEdgeTess = edgeVBO.setEdgeSegTessellation(meshId, 0, changeNumber, points, colors, indices);
    auto sharpEdgeTess = edgeVBO.setEdgeSegTessellation(meshId, 1, changeNumber, points, colors, sharpIndices);
    auto smoothEdgeTess = edgeVBO.setEdgeSegTessellation(meshId, 2, changeNumber, points, colors, smoothIndices);

    pData->setTessEdges(allEdgeTess, sharpEdgeTess, smoothEdgeTess);
    vector<float> normPts;
    vector<unsigned int> normIndices;
    pData->getEdgeData(normPts, normIndices);

    if (!normPts.empty()) {
        auto normalTess = edgeVBO.setEdgeSegTessellation(meshId, 3, changeNumber, normPts, normIndices);
        pData->setTessNormals(normalTess);
    }
}

void DrawModelMesh::changeViewElements(const Model& meshData)
{
    auto& faceVBO = _VBOs->_faceVBO;
    auto& edgeVBO = _VBOs->_edgeVBO;

    faceVBO.beginSettingElementIndices(0xffffffffffffffff);
    edgeVBO.beginSettingElementIndices(0xffffffffffffffff);

    for (auto& pData : meshData) {

        if (_options.showFaces) {
            if (pData->isSolid())
                faceVBO.includeElementIndices(DS_MODEL_FACES_SOLID, pData->getFaceTess());
            else
                faceVBO.includeElementIndices(DS_MODEL_FACES_SURFACE, pData->getFaceTess());
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
        }
        else if (_options.showSharpEdges) {
            edgeVBO.includeElementIndices(DS_MODEL_SHARP_EDGES, pData->getSharpEdgeTess());
        }
    }

    faceVBO.endSettingElementIndices();
    edgeVBO.endSettingElementIndices();
}

OGL::MultiVBO::DrawVertexColorMode DrawModelMesh::preDrawEdges(int key)
{
    OGL::MultiVBO::DrawVertexColorMode result = OGL::MultiVBO::DrawVertexColorMode::DRAW_COLOR_NONE;
    auto& UBO = _pCanvas->getUBO();
    UBO.useDefColor = 1;
    _priorNormalShadingOn = UBO.normalShadingOn;
    UBO.normalShadingOn = 0;

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
    }

    if (_options.showCurvature) {
        result = OGL::MultiVBO::DrawVertexColorMode::DRAW_COLOR;
        UBO.useDefColor = 0;
    }

    glBufferData(GL_UNIFORM_BUFFER, sizeof(UBO), &UBO, GL_DYNAMIC_DRAW);

    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);

    return result;

}

void DrawModelMesh::postDrawEdges()
{
    auto& UBO = _pCanvas->getUBO();
    UBO.normalShadingOn = _priorNormalShadingOn ? 1 : 0;
}

OGL::MultiVBO::DrawVertexColorMode DrawModelMesh::preDrawFaces(int key)
{
    OGL::MultiVBO::DrawVertexColorMode result = OGL::MultiVBO::DrawVertexColorMode::DRAW_COLOR_NONE;
    auto& UBO = _pCanvas->getUBO();
    UBO.useDefColor = 1;

    switch (key) {
    default:
    case DS_MODEL_FACES_SOLID:
        UBO.defColor = p4f(0.4f, 0.4f, 1.0f, 1);
        break;
    case DS_MODEL_FACES_SURFACE:
        UBO.defColor = p4f(0.6f, 1.0f, 0.6f, 0.6f);
        break;
    case DS_MODEL_SHARP_VERTS:
        UBO.defColor = p3f(1.0f, 1.0f, 0);
        break;
    }

#if 0
    if (_options.showCurvature) {
        UBO.useDefColor = 0;
        result = OGL::MultiVBO::DrawVertexColorMode::DRAW_COLOR;
    }
#endif

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

