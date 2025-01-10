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
#include <volume.h>
#include <graphicsCanvas.h>

using namespace std;
using namespace DFHM;

DrawHexMesh::DrawHexMesh(GraphicsCanvas* pCanvas)
    : DrawMesh(pCanvas)
{
}

DrawHexMesh::~DrawHexMesh()
{

}

void DrawHexMesh::addHexFacesToScene(const VolumePtr& pVolume, const Index3D& min, const Index3D& max, bool multiCore)
{
    Block::GlHexMeshGroup blockMeshes;
    pVolume->createHexFaceTris(blockMeshes, min, max, multiCore);

    auto& faceVBO = getVBOs()->_faceVBO;
    auto& edgeVBO = getVBOs()->_edgeVBO;

    faceVBO.beginFaceTesselation();
    edgeVBO.beginEdgeTesselation();

    vector<OGL::IndicesPtr> faceTesselations, edgeTesselations;
    for (size_t mode = 0; mode < blockMeshes.size(); mode++) {
        FaceDrawType faceType = (FaceDrawType)mode;
        auto& thisGroup = blockMeshes[mode];

        size_t changeNumber = 0;
        vector<float> triPoints, triNormals, triParameters, edgePoints;
        vector<unsigned int> vertIndices, edgeIndices;
        size_t triIdx = 0, edgeIdx = 0;
        for (const auto& pBlockMesh : thisGroup) {
            if (pBlockMesh) {
                size_t numTriVerts = pBlockMesh->numTriVertices();
                if (numTriVerts > 0) {
                    const auto& tmpTriPoints = pBlockMesh->_glTriPoints;
                    const auto& tmpTriNormals = pBlockMesh->_glTriNormals;
                    vector<float> tmpTriParameters;
                    tmpTriParameters.resize(3 * numTriVerts);


                    triPoints.insert(triPoints.end(), tmpTriPoints.begin(), tmpTriPoints.end());
                    triNormals.insert(triNormals.end(), tmpTriNormals.begin(), tmpTriNormals.end());
                    triParameters.insert(triParameters.end(), tmpTriParameters.begin(), tmpTriParameters.end());

                    vertIndices.reserve(vertIndices.size() + numTriVerts);
                    for (size_t i = 0; i < numTriVerts; i++)
                        vertIndices.push_back(triIdx++);
                }

                size_t numEdgeVerts = pBlockMesh->numEdgeVertices();
                if (numEdgeVerts > 0) {
                    const auto& tmpEdgePoints = pBlockMesh->_glEdgePoints;
                    edgePoints.insert(edgePoints.end(), tmpEdgePoints.begin(), tmpEdgePoints.end());
                    edgeIndices.reserve(edgeIndices.size() + numEdgeVerts);
                    for (size_t i = 0; i < numEdgeVerts; i++)
                        edgeIndices.push_back(edgeIdx++);
                }
            }
        }

        auto pFaceTess = faceVBO.setFaceTessellation(faceType, changeNumber, triPoints, triNormals, triParameters, vertIndices);
        faceTesselations.push_back(pFaceTess);

        auto pEdgeTess = edgeVBO.setEdgeSegTessellation(faceType, changeNumber, edgePoints, edgeIndices);
        edgeTesselations.push_back(pFaceTess);
    }

    setFaceTessellations(faceTesselations);
    setEdgeTessellations(edgeTesselations);

    faceVBO.endFaceTesselation(false);
    edgeVBO.endEdgeTesselation();
}

void DrawHexMesh::changeViewElements()
{
    auto& faceVBO = _VBOs->_faceVBO;
    auto& edgeVBO = _VBOs->_edgeVBO;

    edgeVBO.beginSettingElementIndices(0xffffffffffffffff);
    faceVBO.beginSettingElementIndices(0xffffffffffffffff);

    if (_options.showFaces && !_faceTessellations.empty()) {
        bool drewSomething = false;
        if (_options.showSelectedBlocks) {
            if (FT_ALL < _faceTessellations.size()) {
                if (_faceTessellations[FT_ALL]) {
                    drewSomething = true;
                    faceVBO.includeElementIndices(DS_MESH_ALL, _faceTessellations[FT_ALL]);
                }
            }
        } else if (_options.showWalls) {
            if (FT_WALL < _faceTessellations.size()) {
                if (_faceTessellations[FT_WALL]) {
                    drewSomething = true;
                    faceVBO.includeElementIndices(DS_MESH_WALL, _faceTessellations[FT_WALL]);
                }
            }
        } else {
            bool blocksAdded = false;
            if (_options.showBack && FT_BACK < _faceTessellations.size()) {
                if (_faceTessellations[FT_BACK]) {
                    drewSomething = true;
                    blocksAdded = true;
                    faceVBO.includeElementIndices(DS_MESH_BACK, _faceTessellations[FT_BACK]);
                }
            }

            if (_options.showFront && FT_FRONT < _faceTessellations.size()) {
                if (_faceTessellations[FT_FRONT]) {
                    drewSomething = true;
                    blocksAdded = true;
                    faceVBO.includeElementIndices(DS_MESH_FRONT, _faceTessellations[FT_FRONT]);
                }
            }

            if (_options.showLeft && FT_LEFT < _faceTessellations.size()) {
                if (_faceTessellations[FT_LEFT]) {
                    drewSomething = true;
                    blocksAdded = true;
                    faceVBO.includeElementIndices(DS_MESH_LEFT, _faceTessellations[FT_LEFT]);
                }
            }

            if (_options.showRight && FT_RIGHT < _faceTessellations.size()) {
                if (_faceTessellations[FT_RIGHT]) {
                    drewSomething = true;
                    blocksAdded = true;
                    faceVBO.includeElementIndices(DS_MESH_RIGHT, _faceTessellations[FT_RIGHT]);
                }
            }

            if (_options.showBottom && FT_BOTTOM < _faceTessellations.size()) {
                if (_faceTessellations[FT_BOTTOM]) {
                    drewSomething = true;
                    blocksAdded = true;
                    faceVBO.includeElementIndices(DS_MESH_BOTTOM, _faceTessellations[FT_BOTTOM]);
                }
            }

            if (_options.showTop && FT_TOP < _faceTessellations.size()) {
                if (_faceTessellations[FT_TOP]) {
                    drewSomething = true;
                    blocksAdded = true;
                    faceVBO.includeElementIndices(DS_MESH_TOP, _faceTessellations[FT_TOP]);
                }
            }

            if (!blocksAdded || !drewSomething) {
                if (FT_ALL < _faceTessellations.size()) {
                    if (_faceTessellations[FT_ALL]) {
                        blocksAdded = true;
                        faceVBO.includeElementIndices(DS_MESH_ALL, _faceTessellations[FT_ALL]);
                    }
                }
            }
        }
    }

    if (_options.showEdges && !_edgeTessellations.empty()) {
        bool drewSomething = false;
        if (_options.showWalls && FT_WALL < _edgeTessellations.size()) {
            if (_edgeTessellations[FT_WALL]) {
                drewSomething = true;
                edgeVBO.includeElementIndices(DS_MESH_WALL, _edgeTessellations[FT_WALL]);
            }
        }

        if (_options.showBack && FT_BACK < _edgeTessellations.size()) {
            if (_edgeTessellations[FT_BACK]) {
                drewSomething = true;
                edgeVBO.includeElementIndices(DS_MESH_BACK, _edgeTessellations[FT_BACK]);
            }
        }

        if (_options.showFront && FT_FRONT < _edgeTessellations.size()) {
            if (_edgeTessellations[FT_FRONT]) {
                drewSomething = true;
                edgeVBO.includeElementIndices(DS_MESH_FRONT, _edgeTessellations[FT_FRONT]);
            }
        }

        if (_options.showLeft && FT_LEFT < _edgeTessellations.size()) {
            if (_edgeTessellations[FT_LEFT]) {
                drewSomething = true;
                edgeVBO.includeElementIndices(DS_MESH_LEFT, _edgeTessellations[FT_LEFT]);
            }
        }

        if (_options.showRight && FT_RIGHT < _edgeTessellations.size()) {
            if (_edgeTessellations[FT_RIGHT]) {
                drewSomething = true;
                edgeVBO.includeElementIndices(DS_MESH_RIGHT, _edgeTessellations[FT_RIGHT]);
            }
        }

        if (_options.showBottom && FT_BOTTOM < _edgeTessellations.size()) {
            if (_edgeTessellations[FT_BOTTOM]) {
                drewSomething = true;
                edgeVBO.includeElementIndices(DS_MESH_BOTTOM, _edgeTessellations[FT_BOTTOM]);
            }
        }

        if (_options.showTop && FT_TOP < _edgeTessellations.size()) {
            if (_edgeTessellations[FT_TOP]) {
                drewSomething = true;
                edgeVBO.includeElementIndices(DS_MESH_TOP, _edgeTessellations[FT_TOP]);
            }
        }

        if (!drewSomething && FT_ALL < _edgeTessellations.size()) {
            if (_edgeTessellations[FT_ALL])
                edgeVBO.includeElementIndices(DS_MESH_ALL, _edgeTessellations[FT_ALL]);
        }

    }

    edgeVBO.endSettingElementIndices();
    faceVBO.endSettingElementIndices();
}

OGL::MultiVBO::DrawVertexColorMode DrawHexMesh::preDrawEdges(int key)
{
    OGL::MultiVBO::DrawVertexColorMode result = OGL::MultiVBO::DrawVertexColorMode::DRAW_COLOR_NONE;
    auto& UBO = _pCanvas->getUBO();
    glLineWidth(0.25f);
    if (_options.showFaces) {
        UBO.defColor = p3f(0.0f, 0.0f, 0.0f);
    } else {
        switch (key) {
        default:
        case DS_MESH_ALL:
            UBO.defColor = p3f(0.0f, 0.0f, 0.0f);
            break;
        case DS_MESH_INNER:
            UBO.defColor = p3f(1.0f, 0.5f, 0.50f);
            break;
        case DS_MESH_WALL:
            UBO.defColor = p3f(0.25f, 0.5f, 0.250f);
            break;

        case DS_MESH_BACK:
            UBO.defColor = p3f(1.0f, 0.0f, 0.0f);
            break;

        case DS_MESH_FRONT:
            UBO.defColor = p3f(0.0f, 1.0f, 0.0f);
            break;

        case DS_MESH_BOTTOM:
            UBO.defColor = p3f(0.0f, 0.0f, 1.0f);
            break;

        case DS_MESH_TOP:
            UBO.defColor = p3f(0.0f, 1.0f, 1.0f);
            break;

        case DS_MESH_LEFT:
            UBO.defColor = p3f(1.0f, 0.0f, 1.0f);
            break;

        case DS_MESH_RIGHT:
            UBO.defColor = p3f(1.0f, 1.0f, 0.0f);
            break;

        }
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
    UBO.ambient = 0.2f;
    switch (key) {
    default:
    case DS_MESH_WALL:
        UBO.defColor = p3f(0.5f, 1.f, 0.5f);
        break;
    case DS_MESH_ALL:
        UBO.defColor = p3f(0.0f, 0.8f, 0);
        break;
    case DS_MESH_INNER:
        UBO.defColor = p3f(0.75f, 1, 1);
        break;

    case DS_MESH_BACK:
        UBO.defColor = p3f(1.0f, 0.0f, 0.0f);
        break;

    case DS_MESH_FRONT:
        UBO.defColor = p3f(0.0f, 1.0f, 0.0f);
        break;

    case DS_MESH_BOTTOM:
        UBO.defColor = p3f(0.0f, 0.0f, 1.0f);
        break;

    case DS_MESH_TOP:
        UBO.defColor = p3f(0.0f, 1.0f, 1.0f);
        break;

    case DS_MESH_LEFT:
        UBO.defColor = p3f(1.0f, 0.0f, 1.0f);
        break;

    case DS_MESH_RIGHT:
        UBO.defColor = p3f(1.0f, 1.0f, 0.0f);
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

