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
#include <enums.h>
#include <graphicsVBORec.h>

using namespace DFHM;

VBORec::VBORec()
    : _faceVBO(GL_TRIANGLES, 20)
    , _edgeVBO(GL_LINES, 20)
{
}

void VBORec::changeFaceViewElements(bool visible, const ChangeElementsOptions& opts)
{
    _faceVBO.beginSettingElementIndices(0xffffffffffffffff);

    if (visible) {
        if (_pTriTess) {
            if (opts.showCurvature)
                _faceVBO.includeElementIndices(DS_MODEL_CURVATURE, *_pTriTess);
            else
                _faceVBO.includeElementIndices(DS_MODEL, *_pTriTess);
        }

        if (opts.showSharpVerts && _pSharpVertTess)
            _faceVBO.includeElementIndices(DS_MODEL_SHARP_VERTS, *_pSharpVertTess);

        if (opts.showFaces) {
            if (opts.showSelectedBlocks) {
                if (FT_ALL < _faceTessellations.size()) {
                    for (auto pBlockTess : _faceTessellations[FT_ALL]) {
                        if (pBlockTess)
                            _faceVBO.includeElementIndices(DS_BLOCK_ALL, *pBlockTess);
                    }
                }
            }
            else if (opts.showOuter) {
                if (FT_OUTER < _faceTessellations.size()) {
                    for (auto pBlockTess : _faceTessellations[FT_OUTER]) {
                        if (pBlockTess)
                            _faceVBO.includeElementIndices(DS_BLOCK_OUTER, *pBlockTess);
                    }
                }
            }
            else {
                if (FT_MODEL_BOUNDARY < _faceTessellations.size()) {
                    for (auto pBlockTess : _faceTessellations[FT_MODEL_BOUNDARY]) {
                        if (pBlockTess)
                            _faceVBO.includeElementIndices(DS_BLOCK_INNER, *pBlockTess);
                    }
                }

                if (FT_BLOCK_BOUNDARY < _faceTessellations.size()) {
                    for (auto pBlockTess : _faceTessellations[FT_BLOCK_BOUNDARY]) {
                        if (pBlockTess)
                            _faceVBO.includeElementIndices(DS_BLOCK_BOUNDARY, *pBlockTess);
                    }
                }
            }
        }
    }

    _faceVBO.endSettingElementIndices();
}

void VBORec::changeEdgeViewElements(bool visible, const ChangeElementsOptions& opts)
{
    _edgeVBO.beginSettingElementIndices(0xffffffffffffffff);

    if (visible) {
        if (opts.showSharpEdges && _pSharpEdgeTess) {
            _edgeVBO.includeElementIndices(DS_MODEL_SHARP_EDGES, *_pSharpEdgeTess);
        }
        if (opts.showTriNormals && _pNormalTess) {
            _edgeVBO.includeElementIndices(DS_MODEL_NORMALS, *_pNormalTess);
        }
        if (opts.showEdges && !_edgeTessellations.empty()) {
            if (opts.showOuter && FT_OUTER < _edgeTessellations.size()) {
                for (auto pBlockTess : _edgeTessellations[FT_OUTER]) {
                    if (pBlockTess)
                        _edgeVBO.includeElementIndices(DS_BLOCK_OUTER, *pBlockTess);
                }
            }

            if (!opts.showOuter && FT_MODEL_BOUNDARY < _edgeTessellations.size()) {
                for (auto pBlockTess : _edgeTessellations[FT_MODEL_BOUNDARY]) {
                    if (pBlockTess)
                        _edgeVBO.includeElementIndices(DS_BLOCK_INNER, *pBlockTess);
                }
            }

            if (!opts.showOuter && FT_BLOCK_BOUNDARY < _edgeTessellations.size()) {
                for (auto pBlockTess : _edgeTessellations[FT_BLOCK_BOUNDARY]) {
                    if (pBlockTess)
                        _edgeVBO.includeElementIndices(DS_BLOCK_BOUNDARY, *pBlockTess);
                }
            }

            if (!opts.showSelectedBlocks && FT_ALL < _edgeTessellations.size()) {
                for (auto pBlockTess : _edgeTessellations[FT_ALL]) {
                    if (pBlockTess)
                        _edgeVBO.includeElementIndices(DS_BLOCK_ALL, *pBlockTess);
                }
            }
        }
    }

    _edgeVBO.endSettingElementIndices();

}