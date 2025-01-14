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

#include <defines.h>
#include<drawHexMesh.h>

#include <enums.h>
#include <volume.h>
#include <graphicsCanvas.h>
#include <utils.h>

using namespace std;
using namespace DFHM;

namespace
{
    const float satVal = 0.5f;

}

DrawHexMesh::DrawHexMesh(GraphicsCanvas* pCanvas)
    : DrawMesh(pCanvas)
{
}

DrawHexMesh::~DrawHexMesh()
{

}

void DrawHexMesh::clearPrior()
{
    _faceTessellations.clear();
    _edgeTessellations.clear();

    _VBOs->_edgeVBO.clear();
    _VBOs->_faceVBO.clear();
}

void DrawHexMesh::clearPost()
{
    _triVertexToIndexMap.clear();
    _edgeVertexToIndexMap.clear();
    _edgeMap.clear();
    _triPoints.clear();
    _triNormals.clear();
    _edgePoints.clear();
    _triIndices.clear();
    _edgeIndices.clear();

}

void DrawHexMesh::addHexFacesToScene(const VolumePtr& pVolume, const Index3D& min, const Index3D& max, bool multiCore)
{
    Utils::ScopedRestore sr(_readyToDraw);
    _readyToDraw = false;

    clearPrior();

    Block::GlHexMeshGroup blockMeshes;
    pVolume->createHexFaceTris(blockMeshes, min, max, multiCore);

    auto& faceVBO = getVBOs()->_faceVBO;
    auto& edgeVBO = getVBOs()->_edgeVBO;
    faceVBO.clear();
    edgeVBO.clear();

    faceVBO.beginFaceTesselation();
    edgeVBO.beginEdgeTesselation();

    createVertexBuffers(blockMeshes[FT_ALL]);

    for (size_t mode = 0; mode < FT_ALL; mode++) {
        FaceDrawType faceType = (FaceDrawType)mode;
        vector<unsigned int> triIndices, edgeIndices;
        auto& thisGroup = blockMeshes[mode];

        std::map<GLEdge, size_t> localEdgeMap;
        for (const auto& pBlockMesh : thisGroup) {
            if (pBlockMesh) {
                size_t numTriVerts = pBlockMesh->numTriVertices();
                if (numTriVerts > 0) {
                    const auto& tmpTriPoints = pBlockMesh->_glTriPoints;
                    const auto& tmpTriNormals = pBlockMesh->_glTriNormals;
                    for (size_t i = 0; i < numTriVerts; i++) {
                        Vector3f ptf((float)tmpTriPoints[3 * i + 0], (float)tmpTriPoints[3 * i + 1], (float)tmpTriPoints[3 * i + 2]);
                        Vector3f normf((float)tmpTriNormals[3 * i + 0], (float)tmpTriNormals[3 * i + 1], (float)tmpTriNormals[3 * i + 2]);
                        size_t idx = getVertexIdx(ptf, normf);
                        triIndices.push_back(idx);
                    }
                }

                const auto& tmpEdgePoints = pBlockMesh->_glEdgePoints;
                size_t numEdges = tmpEdgePoints.size() / (2 * 3);
                if (numEdges > 0) {
                    for (size_t i = 0; i < numEdges; i++) {
                        int vIdx = 2 * i;

                        Vector3f ptf0((float)tmpEdgePoints[3 * vIdx + 0], (float)tmpEdgePoints[3 * vIdx + 1], (float)tmpEdgePoints[3 * vIdx + 2]);
                        size_t idx0 = getVertexIdx(ptf0);

                        vIdx++;
                        Vector3f ptf1((float)tmpEdgePoints[3 * vIdx + 0], (float)tmpEdgePoints[3 * vIdx + 1], (float)tmpEdgePoints[3 * vIdx + 2]);
                        size_t idx1 = getVertexIdx(ptf1);

                        GLEdge e(idx0, idx1);
                        if (localEdgeMap.find(e) == localEdgeMap.end()) {
                            size_t eIdx = edgeIndices.size() / 2;
                            edgeIndices.push_back(idx0);
                            edgeIndices.push_back(idx1);
                            localEdgeMap.insert(make_pair(e, eIdx));
                        }
                    }
                }
            }
        }

        auto pFaceTess = faceVBO.setFaceTessellation(faceType, _faceTessellations[FT_ALL], triIndices);
        _faceTessellations[faceType] = pFaceTess;

        auto pEdgeTess = edgeVBO.setEdgeSegTessellation(faceType, _edgeTessellations[FT_ALL], edgeIndices);
        _edgeTessellations[faceType] = pEdgeTess;
    }

    faceVBO.endFaceTesselation(false);
    edgeVBO.endEdgeTesselation();

    clearPost();
}

void DrawHexMesh::changeViewElements()
{
    auto& faceVBO = _VBOs->_faceVBO;
    auto& edgeVBO = _VBOs->_edgeVBO;

    edgeVBO.beginSettingElementIndices(0xffffffffffffffff);
    faceVBO.beginSettingElementIndices(0xffffffffffffffff);

    bool blocksAdded = false;

    if (_options.showFaces && !_faceTessellations.empty()) {
        blocksAdded = false;
        if (_options.showSelectedBlocks) {
            if (FT_ALL < _faceTessellations.size()) {
                if (_faceTessellations[FT_ALL]) {
                    blocksAdded = true;
                    faceVBO.includeElementIndices(DS_MESH_ALL, _faceTessellations[FT_ALL]);
                }
            }
        } else {
            if (_options.showWalls) {
                if (FT_WALL < _faceTessellations.size()) {
                    if (_faceTessellations[FT_WALL]) {
                        blocksAdded = true;
                        faceVBO.includeElementIndices(DS_MESH_WALL, _faceTessellations[FT_WALL]);
                    }
                }
            }

            if (_options.showBack && FT_BACK < _faceTessellations.size()) {
                if (_faceTessellations[FT_BACK]) {
                    blocksAdded = true;
                    faceVBO.includeElementIndices(DS_MESH_BACK, _faceTessellations[FT_BACK]);
                }
            }

            if (_options.showFront && FT_FRONT < _faceTessellations.size()) {
                if (_faceTessellations[FT_FRONT]) {
                    blocksAdded = true;
                    faceVBO.includeElementIndices(DS_MESH_FRONT, _faceTessellations[FT_FRONT]);
                }
            }

            if (_options.showLeft && FT_LEFT < _faceTessellations.size()) {
                if (_faceTessellations[FT_LEFT]) {
                    blocksAdded = true;
                    faceVBO.includeElementIndices(DS_MESH_LEFT, _faceTessellations[FT_LEFT]);
                }
            }

            if (_options.showRight && FT_RIGHT < _faceTessellations.size()) {
                if (_faceTessellations[FT_RIGHT]) {
                    blocksAdded = true;
                    faceVBO.includeElementIndices(DS_MESH_RIGHT, _faceTessellations[FT_RIGHT]);
                }
            }

            if (_options.showBottom && FT_BOTTOM < _faceTessellations.size()) {
                if (_faceTessellations[FT_BOTTOM]) {
                    blocksAdded = true;
                    faceVBO.includeElementIndices(DS_MESH_BOTTOM, _faceTessellations[FT_BOTTOM]);
                }
            }

            if (_options.showTop && FT_TOP < _faceTessellations.size()) {
                if (_faceTessellations[FT_TOP]) {
                    blocksAdded = true;
                    faceVBO.includeElementIndices(DS_MESH_TOP, _faceTessellations[FT_TOP]);
                }
            }

            if (!blocksAdded) {
                if (FT_ALL < _faceTessellations.size()) {
                    if (_faceTessellations[FT_ALL]) {
                        blocksAdded = true;
                        faceVBO.includeElementIndices(DS_MESH_ALL, _faceTessellations[FT_ALL]);
                    }
                }
            }
        }
    }

    if (_options.showEdges && FT_ALL < _edgeTessellations.size()) {
        blocksAdded = false;
        if (_options.showWalls && FT_WALL < _edgeTessellations.size()) {
            if (_edgeTessellations[FT_WALL]) {
                blocksAdded = true;
                edgeVBO.includeElementIndices(DS_MESH_WALL, _edgeTessellations[FT_WALL]);
            }
        }

        if (_options.showBack && FT_BACK < _edgeTessellations.size()) {
            if (_edgeTessellations[FT_BACK]) {
                blocksAdded = true;
                edgeVBO.includeElementIndices(DS_MESH_BACK, _edgeTessellations[FT_BACK]);
            }
        }

        if (_options.showFront && FT_FRONT < _edgeTessellations.size()) {
            if (_edgeTessellations[FT_FRONT]) {
                blocksAdded = true;
                edgeVBO.includeElementIndices(DS_MESH_FRONT, _edgeTessellations[FT_FRONT]);
            }
        }

        if (_options.showLeft && FT_LEFT < _edgeTessellations.size()) {
            if (_edgeTessellations[FT_LEFT]) {
                blocksAdded = true;
                edgeVBO.includeElementIndices(DS_MESH_LEFT, _edgeTessellations[FT_LEFT]);
            }
        }

        if (_options.showRight && FT_RIGHT < _edgeTessellations.size()) {
            if (_edgeTessellations[FT_RIGHT]) {
                blocksAdded = true;
                edgeVBO.includeElementIndices(DS_MESH_RIGHT, _edgeTessellations[FT_RIGHT]);
            }
        }

        if (_options.showBottom && FT_BOTTOM < _edgeTessellations.size()) {
            if (_edgeTessellations[FT_BOTTOM]) {
                blocksAdded = true;
                edgeVBO.includeElementIndices(DS_MESH_BOTTOM, _edgeTessellations[FT_BOTTOM]);
            }
        }

        if (_options.showTop && FT_TOP < _edgeTessellations.size()) {
            if (_edgeTessellations[FT_TOP]) {
                blocksAdded = true;
                edgeVBO.includeElementIndices(DS_MESH_TOP, _edgeTessellations[FT_TOP]);
            }
        }

        if (!blocksAdded && FT_ALL < _edgeTessellations.size()) {
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
    UBO.useDefColor = 1;
    _priorNormalShadingOn = UBO.normalShadingOn;
    UBO.normalShadingOn = 0;
    glLineWidth(0.25f);

    const auto& alpha = _options.alpha;

    if (_options.showFaces) {
        glLineWidth(0.5f);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        UBO.defColor = p4f(0.0f, 0.0f, 0.0f, alpha);
    } else {
        switch (key) {
        default:
        case DS_MESH_ALL:
            UBO.defColor = p3f(0.0f, 0.0f, 0.5f);
            break;
        case DS_MESH_INNER:
            UBO.defColor = p3f(1.0f, 0.5f, 0.50f);
            break;
        case DS_MESH_WALL:
            UBO.defColor = p3f(0.25f, 0.5f, 0.250f);
            break;

        case DS_MESH_BACK:
            UBO.defColor = p3f(1.0f, satVal, satVal);
            break;

        case DS_MESH_FRONT:
            UBO.defColor = p3f(satVal, 1.0f, satVal);
            break;

        case DS_MESH_BOTTOM:
            UBO.defColor = p3f(satVal, satVal, 1.0f);
            break;

        case DS_MESH_TOP:
            UBO.defColor = p3f(satVal, 1.0f, 1.0f);
            break;

        case DS_MESH_LEFT:
            UBO.defColor = p3f(1.0f, satVal, 1.0f);
            break;

        case DS_MESH_RIGHT:
            UBO.defColor = p3f(1.0f, 1.0f, satVal);
            break;

        }
    }

    glBufferData(GL_UNIFORM_BUFFER, sizeof(UBO), &UBO, GL_DYNAMIC_DRAW);

    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);

    return result;
}

void DrawHexMesh::postDrawEdges()
{
    auto& UBO = _pCanvas->getUBO();
    UBO.normalShadingOn = _priorNormalShadingOn ? 1 : 0;
}

OGL::MultiVBO::DrawVertexColorMode DrawHexMesh::preDrawFaces(int key)
{
    OGL::MultiVBO::DrawVertexColorMode result = OGL::MultiVBO::DrawVertexColorMode::DRAW_COLOR_NONE;
    auto& UBO = _pCanvas->getUBO();
    _priorDrawTwoSided = UBO.twoSideLighting;
    UBO.twoSideLighting = 1;

    bool blend = false;
    const auto& alpha = _options.alpha;
    const float den = 255.0f;

    switch (key) {
    default:
    case DS_MESH_WALL:
        blend = true;
        UBO.defColor = p4f(0.5f, 1.f, 0.5f, alpha);
        break;
    case DS_MESH_ALL:
        blend = true;
        UBO.defColor = p4f(210 / den, 180 / den, 140 / den, alpha);
        break;
    case DS_MESH_INNER:
        blend = true;
        UBO.defColor = p4f(0.75f, 1, 1, alpha);
        break;

    case DS_MESH_BACK:
        blend = true;
        UBO.defColor = p4f(1.0f, satVal, satVal, alpha);
        break;

    case DS_MESH_FRONT:
        blend = true;
        UBO.defColor = p4f(satVal, 1.0f, satVal, alpha);
        break;

    case DS_MESH_BOTTOM:
        blend = true;
        UBO.defColor = p4f(satVal, satVal, 1.0f, alpha);
        break;

    case DS_MESH_TOP:
        blend = true;
        UBO.defColor = p4f(satVal, 1.0f, 1.0f, alpha);
        break;

    case DS_MESH_LEFT:
        blend = true;
        UBO.defColor = p4f(1.0f, satVal, 1.0f, alpha);
        break;

    case DS_MESH_RIGHT:
        blend = true;
        UBO.defColor = p4f(1.0f, 1.0f, satVal, alpha);
        break;
    }

    glBufferData(GL_UNIFORM_BUFFER, sizeof(UBO), &UBO, GL_DYNAMIC_DRAW);

    if (blend) {
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    }

    if (_options.showEdges) {
        glEnable(GL_POLYGON_OFFSET_FILL);
        glPolygonOffset(1.0f, 2.0f);
    }

    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);

    return result;
}

void DrawHexMesh::postDrawFaces()
{
    auto& UBO = _pCanvas->getUBO();
    UBO.twoSideLighting = _priorDrawTwoSided;

    glBufferData(GL_UNIFORM_BUFFER, sizeof(UBO), &UBO, GL_DYNAMIC_DRAW);

    glDisable(GL_BLEND);
}

void DrawHexMesh::createVertexBuffers(const Block::GlHexFacesVector& faces)
{
    _triIndices.resize(FT_ALL + 1);
    _edgeIndices.resize(FT_ALL + 1);
    _faceTessellations.resize(FT_ALL + 1);
    _edgeTessellations.resize(FT_ALL + 1);

    size_t triIdx = 0, edgeIdx = 0;
    for (const auto& pBlockMesh : faces) {
        if (!pBlockMesh)
            continue;

        size_t numTriVerts = pBlockMesh->numTriVertices();

        if (numTriVerts > 0) {
            const auto& tmpTriPoints = pBlockMesh->_glTriPoints;
            const auto& tmpTriNormals = pBlockMesh->_glTriNormals;

            for (size_t i = 0; i < numTriVerts; i++) {
                Vector3f ptf((float)tmpTriPoints[3 * i + 0], (float)tmpTriPoints[3 * i + 1], (float)tmpTriPoints[3 * i + 2]);
                Vector3f normf((float)tmpTriNormals[3 * i + 0], (float)tmpTriNormals[3 * i + 1], (float)tmpTriNormals[3 * i + 2]);
                size_t idx = getVertexIdx(ptf, normf);
                _triIndices[FT_ALL].push_back(idx);
            }
        }

        const auto& tmpEdgePoints = pBlockMesh->_glEdgePoints;
        size_t numEdges = tmpEdgePoints.size() / (2 * 3);
        if (numEdges > 0) {
            for (size_t i = 0; i < numEdges; i++) {
                int vIdx = 2 * i;

                Vector3f ptf0((float)tmpEdgePoints[3 * vIdx + 0], (float)tmpEdgePoints[3 * vIdx + 1], (float)tmpEdgePoints[3 * vIdx + 2]);
                size_t idx0 = getVertexIdx(ptf0);

                vIdx++;
                Vector3f ptf1((float)tmpEdgePoints[3 * vIdx + 0], (float)tmpEdgePoints[3 * vIdx + 1], (float)tmpEdgePoints[3 * vIdx + 2]);
                size_t idx1 = getVertexIdx(ptf1);

                GLEdge e(idx0, idx1);
                if (_edgeMap.find(e) == _edgeMap.end()) {
                    size_t eIdx = _edgeIndices.size() / 2;
                    _edgeIndices[FT_ALL].push_back(idx0);
                    _edgeIndices[FT_ALL].push_back(idx1);
                    _edgeMap.insert(make_pair(e, eIdx));
                }
            }
        }
    }

    vector<float> triParameters;
    triParameters.resize(_triPoints.size(), 0);
    auto pFaceTess = _VBOs->_faceVBO.setFaceTessellation(FT_ALL, 0, _triPoints, _triNormals, triParameters, _triIndices[FT_ALL]);
    _faceTessellations[FT_ALL] = pFaceTess;

    auto pEdgeTess = _VBOs->_edgeVBO.setEdgeSegTessellation(FT_ALL, 0, _edgePoints, _edgeIndices[FT_ALL]);
    _edgeTessellations[FT_ALL] = pEdgeTess;

}

DrawHexMesh::VertexPointAndNormal::VertexPointAndNormal(const Vector3f& pt, const Vector3f& normal)
{
    for (int i = 0; i < 3; i++) {
        _iPoint[i] = (int)(pt[i] * 100000);
        _iNormal[i] = (int)(normal[i] * 100000);
    }
}

bool DrawHexMesh::VertexPointAndNormal::operator < (const VertexPointAndNormal& rhs) const
{
    if (_iPoint < rhs._iPoint)
        return true;
    else if (rhs._iPoint < _iPoint)
        return false;

    return _iNormal < rhs._iNormal;
}

size_t DrawHexMesh::getVertexIdx(const Vector3f& pt, const Vector3f& normal)
{
    VertexPointAndNormal val(pt, normal);
    auto iter = _triVertexToIndexMap.find(val);
    if (iter == _triVertexToIndexMap.end()) {
        size_t idx = _triPoints.size() / 3;

        for (int i = 0; i < 3; i++) {
            _triPoints.push_back(pt[i]);
            _triNormals.push_back(normal[i]);
        }

        iter = _triVertexToIndexMap.insert(make_pair(val, idx)).first;
    }

    return iter->second;
}

size_t DrawHexMesh::getVertexIdx(const Vector3f& pt)
{
    VertexPointAndNormal val(pt);
    auto iter = _edgeVertexToIndexMap.find(val);
    if (iter == _edgeVertexToIndexMap.end()) {
        size_t idx = _edgePoints.size() / 3;
        _edgePoints.push_back(pt[0]);
        _edgePoints.push_back(pt[1]);
        _edgePoints.push_back(pt[2]);

        iter = _edgeVertexToIndexMap.insert(make_pair(val, idx)).first;
    }

    return iter->second;
}

DrawHexMesh::GLEdge::GLEdge(unsigned int idx0, unsigned int idx1)
    : _idx0(idx0 < idx1 ? idx0 : idx1)
    , _idx1(idx0 < idx1 ? idx1 : idx0)
{
}

bool DrawHexMesh::GLEdge::operator < (const GLEdge& rhs) const
{
    if (_idx0 < rhs._idx0)
        return true;
    else if (_idx0 > rhs._idx0)
        return false;

    return _idx1 < rhs._idx1;
}
