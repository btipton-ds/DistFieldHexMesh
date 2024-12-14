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

#include <OGLShader.h>
#include <rgbaColor.h>
#include <index3D.h>
#include <meshData.h>
//#include <graphicsCanvas.h>

using namespace std;
using namespace DFHM;

MeshData::MeshData(const TriMesh::CMeshPtr& pMesh, const std::wstring& name, const VBORec::ChangeElementsOptions& options)
	: _name(name)
	, _pMesh(pMesh)
	, _options(options)
{
	_VBOs = make_shared<VBORec>();
}

void MeshData::beginFaceTesselation()
{
	_VBOs->_faceVBO.beginFaceTesselation();
}

// vertiIndices is index pairs into points, normals and parameters to form triangles. It's the standard OGL element index structure
const COglMultiVboHandler::OGLIndices* MeshData::createFaceTessellation(const TriMesh::CMeshPtr& pMesh)
{
	const auto& points = pMesh->getGlTriPoints();
	const auto& normals = pMesh->getGlTriNormals(false);
	const auto& parameters = pMesh->getGlTriParams();
	const auto& vertIndices = pMesh->getGlTriIndices();

	auto colorFunc = [](double curvature, float rgb[3])->bool {
		rgbaColor c = curvatureToColor(curvature);
		for (int i = 0; i < 3; i++)
			rgb[i] = c._rgba[i] / 255.0f;
		return true;
		};

	const auto& colors = pMesh->getGlTriCurvatureColors(colorFunc);
	return _VBOs->_faceVBO.setFaceTessellation(pMesh->getId(), pMesh->getChangeNumber(), points, normals, parameters, colors, vertIndices);
}

void MeshData::endFaceTesselation(const OGLIndices* pTriTess, const OGLIndices* pSharpVertTess, bool smoothNormals)
{
	_VBOs->_faceVBO.endFaceTesselation(smoothNormals);
	_VBOs->_pTriTess = pTriTess;
	_VBOs->_pSharpVertTess = pSharpVertTess;
	_VBOs->_faceTessellations.clear();

	changeFaceViewElements(_options);
}

void MeshData::endFaceTesselation(const std::vector<std::vector<const OGLIndices*>>& faceTess)
{
	_VBOs->_faceVBO.endFaceTesselation(false);
	_VBOs->_faceTessellations = faceTess;

	changeFaceViewElements(_options);
}


void MeshData::beginEdgeTesselation()
{
	_VBOs->_edgeVBO.beginEdgeTesselation();
}

// vertiIndices is index pairs into points, normals and parameters to form triangles. It's the standard OGL element index structure
const OGLIndices* MeshData::setEdgeSegTessellation(size_t entityKey, size_t changeNumber, const std::vector<float>& points, const std::vector<unsigned int>& indices)
{
	return _VBOs->_edgeVBO.setEdgeSegTessellation(entityKey, changeNumber, points, indices);
}

const OGLIndices* MeshData::setEdgeSegTessellation(const CMeshPtr& pMesh)
{
	vector<float> points, colors;
	vector<unsigned int> indices;
	auto colorFunc = [](float curvature, float rgb[3])->bool {
		rgbaColor c;
		if (curvature < 0)
			c = rgbaColor(1, 0, 0);
		else if (curvature < 1.0e-6)
			c = rgbaColor(0, 0, 0);
		else
			c = curvatureToColor(curvature);
		for (int i = 0; i < 3; i++)
			rgb[i] = c._rgba[i] / 255.0f;
		return true;
		};

	pMesh->getGlEdges(colorFunc, false, points, colors, indices);

	return _VBOs->_edgeVBO.setEdgeSegTessellation(pMesh->getId(), pMesh->getChangeNumber(), points, colors, indices);
}

void MeshData::endEdgeTesselation(const OGLIndices* pSharpEdgeTess, const OGLIndices* pNormalTess)
{
	_VBOs->_edgeVBO.endEdgeTesselation();

	_VBOs->_pSharpEdgeTess = pSharpEdgeTess;
	_VBOs->_pNormalTess = pNormalTess;
	_VBOs->_edgeTessellations.clear();

	changeEdgeViewElements(_options);
}

void MeshData::endEdgeTesselation(const std::vector<std::vector<const OGLIndices*>>& edgeTess)
{
	_VBOs->_edgeVBO.endEdgeTesselation();

	_VBOs->_edgeTessellations = edgeTess;

	changeEdgeViewElements(_options);
}


void MeshData::makeOGLTess()
{
	//    _pMesh->squeezeSkinnyTriangles(0.025); TODO This helps curvature calculations, but should be removed
	_pMesh->buildCentroids();
	_pMesh->calCurvatures(SHARP_EDGE_ANGLE_RADIANS, false);
	_pMesh->calGaps();

	beginFaceTesselation();
	CMeshPtr pSharpVertMesh; // = getSharpVertMesh(); // TODO This needs to be instanced and much faster.
	_faceTess = createFaceTessellation(_pMesh);
	if (pSharpVertMesh)
		_sharpPointTess = createFaceTessellation(pSharpVertMesh);
	endFaceTesselation(_faceTess, _sharpPointTess, false);

	vector<float> normPts;
	vector<unsigned int> normIndices;
	getEdgeData(normPts, normIndices);

	beginEdgeTesselation();

	_edgeTess = setEdgeSegTessellation(_pMesh);

	if (!normPts.empty())
		_normalTess = setEdgeSegTessellation(_pMesh->getId() + 10000ull, _pMesh->getChangeNumber(), normPts, normIndices);

	endEdgeTesselation(_edgeTess, _normalTess);

}

void MeshData::getEdgeData(std::vector<float>& normPts, std::vector<unsigned int>& normIndices) const
{
	normPts.clear();
	normIndices.clear();

	for (size_t triIdx = 0; triIdx < _pMesh->numTris(); triIdx++) {
		const Index3D& triIndices = _pMesh->getTri(triIdx);
		const auto pt0 = _pMesh->getVert(triIndices[0])._pt;
		const auto pt1 = _pMesh->getVert(triIndices[1])._pt;
		const auto pt2 = _pMesh->getVert(triIndices[2])._pt;

		Vector3d ctr = (pt0 + pt1 + pt2) / 3.0;
		Vector3d v0 = pt0 - pt1;
		Vector3d v1 = pt2 - pt1;
		Vector3d n = v1.cross(v0);
		double area = n.norm() / 2;
		double charLen = sqrt(area);
		Vector3d ptEnd = ctr + n.normalized() * 0.01;// *charLen;

		for (int j = 0; j < 3; j++)
			normPts.push_back((float)ctr[j]);

		for (int j = 0; j < 3; j++)
			normPts.push_back((float)ptEnd[j]);

		normIndices.push_back((int)normIndices.size());
		normIndices.push_back((int)normIndices.size());
	}
}

void MeshData::changeFaceViewElements(const VBORec::ChangeElementsOptions& opts)
{
	_VBOs->changeFaceViewElements(opts);
}

void MeshData::changeEdgeViewElements(const VBORec::ChangeElementsOptions& opts)
{
	_VBOs->changeEdgeViewElements(opts);
}

void MeshData::setShader(std::shared_ptr<COglShader> pShader)
{
	_VBOs->_faceVBO.setShader(pShader.get());
}

