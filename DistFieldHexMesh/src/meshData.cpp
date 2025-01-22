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
#include <triMesh.hpp>
#include <appData.h>
#include <OGLShader.h>
#include <rgbaColor.h>
#include <index3D.h>
#include <meshData.h>
#include <volume.h>

using namespace std;
using namespace DFHM;

MeshData::MeshData(const AppData* pAppData, const TriMesh::CMeshRepoPtr& pRepo)
: _pAppData(pAppData)
, _pRepo(pRepo)
{
}

MeshData::MeshData(const AppData* pAppData, const TriMesh::CMeshPtr& pMesh, const std::wstring& name)
	: _pAppData(pAppData)
	, _name(name)
	, _pMesh(pMesh)
	, _pRepo(pMesh->getRepo())
{
}

MeshData::~MeshData()
{
}

void MeshData::clear()
{
	_faceTess = nullptr;
	_allEdgeTess = nullptr;
	_smoothEdgeTess = nullptr;
	_sharpEdgeTess = nullptr;
	_normalTess = nullptr;
	_sharpPointTess = nullptr;
}

size_t MeshData::numBytes() const
{
	size_t result = sizeof(MeshData);
	result += sizeof(_name) + _name.length() * sizeof(wchar_t);
	result += _pRepo->numBytes();
	result += _pMesh->numBytes();


	OGL::IndicesPtr
		_faceTess,
		_allEdgeTess,
		_smoothEdgeTess,
		_sharpEdgeTess,
		_normalTess,
		_sharpPointTess;

	return result;
}

void MeshData::write(std::ostream& out) const
{
	uint8_t version = 2;
	out.write((char*)&version, sizeof(version));

	out.write((char*)&_active, sizeof(_active));

	size_t numChars = _name.size();
	out.write((char*)&numChars, sizeof(numChars));
	out.write((char*)_name.c_str(), numChars * sizeof(wchar_t));
//	assert(_pMesh->verifyTopology(false));
	_pMesh->write(out);
}

void MeshData::read(std::istream& in)
{
	uint8_t version = 0;
	in.read((char*)&version, sizeof(version));

	in.read((char*)&_active, sizeof(_active));
	if (version == 1) {
		bool deprecatedReference;
		in.read((char*)&deprecatedReference, sizeof(deprecatedReference));
	}

	size_t numChars;
	in.read((char*)&numChars, sizeof(numChars));
	wchar_t buf[1024];
	in.read((char*)buf, numChars * sizeof(wchar_t));
	buf[numChars] = (wchar_t)0;
	_name = wstring(buf);

	_pMesh = make_shared<CMesh>(_pRepo);
	_pMesh->read(in);
}

// vertiIndices is index pairs into points, normals and parameters to form triangles. It's the standard OGL element index structure
const OGL::IndicesPtr MeshData::createFaceTessellation(const TriMesh::CMeshPtr& pMesh, shared_ptr<DrawModelMesh>& pDrawModelMesh)
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
	auto meshId = pMesh->getId();
	auto changeNumber = pMesh->getChangeNumber();

	auto& faceVBO = pDrawModelMesh->getVBOs()->_faceVBO;
	return faceVBO.setFaceTessellation(meshId, changeNumber, points, normals, parameters, colors, vertIndices);
}

void MeshData::setEdgeSegTessellation(const TriMesh::CMeshPtr& pMesh, std::shared_ptr<DrawModelMesh>& pDrawModelMesh)
{
	const auto& params = _pAppData->getParams();
	const auto sinSharpAngle = params.getSinSharpAngle();

	vector<float> points, colors;
	vector<unsigned int> indices, sharpIndices, smoothIndices;
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

	bool includeSmooth = true;
	pMesh->getGlEdges(colorFunc, includeSmooth, points, colors, sinSharpAngle, sharpIndices, smoothIndices);
	indices = smoothIndices;
	indices.insert(indices.end(), sharpIndices.begin(), sharpIndices.end());

	auto meshId = pMesh->getId();
	auto changeNumber = pMesh->getChangeNumber();
	auto& edgeVBO = pDrawModelMesh->getVBOs()->_edgeVBO;
	_allEdgeTess = edgeVBO.setEdgeSegTessellation(meshId, 0, changeNumber, points, colors, indices);
	_sharpEdgeTess = edgeVBO.setEdgeSegTessellation(meshId, 1, changeNumber, points, colors, sharpIndices);
	_smoothEdgeTess = edgeVBO.setEdgeSegTessellation(meshId, 2, changeNumber, points, colors, smoothIndices);

	vector<float> normPts;
	vector<unsigned int> normIndices;
	getEdgeData(normPts, normIndices);

	if (!normPts.empty()) {
		_normalTess = edgeVBO.setEdgeSegTessellation(meshId, 3, changeNumber, normPts, normIndices);

	}
}

void MeshData::makeOGLTess(shared_ptr<DrawModelMesh>& pDrawModelMesh)
{
	//    _pMesh->squeezeSkinnyTriangles(0.025); TODO This helps curvature calculations, but should be removed
	_pMesh->buildCentroids();
	_pMesh->calCurvatures(SHARP_EDGE_ANGLE_RADIANS, false);
	_pMesh->calGaps();

	CMeshPtr pSharpVertMesh; // = getSharpVertMesh(); // TODO This needs to be instanced and much faster.
	_faceTess = createFaceTessellation(_pMesh, pDrawModelMesh);
	if (pSharpVertMesh)
		_sharpPointTess = createFaceTessellation(pSharpVertMesh, pDrawModelMesh);

	setEdgeSegTessellation(_pMesh, pDrawModelMesh);
}

void MeshData::changeViewElements(std::shared_ptr<DrawModelMesh>& pDraw)
{
	if (!isActive())
		return;

	auto& faceVBO = pDraw->getVBOs()->_faceVBO;
	auto& edgeVBO = pDraw->getVBOs()->_edgeVBO;

	if (pDraw->showFaces()) {
		faceVBO.includeElementIndices(DS_MESH_FACES, getFaceTess());
	}
	if (pDraw->showEdges()) {
		edgeVBO.includeElementIndices(DS_MESH_EDGES, getAllEdgeTess());
	}
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

void MeshData::addPointMarker(CMeshPtr& pMesh, const Vector3d& origin, double radius) const
{
	Vector3d xAxis(radius, 0, 0), yAxis(0, radius, 0), zAxis(0, 0, radius);
	size_t stepsI = 72;
	size_t stepsJ = stepsI / 2;
	for (size_t i = 0; i < stepsI; i++) {
		double alpha0 = 2 * M_PI * i / (double)stepsI;
		double alpha1 = 2 * M_PI * (i + 1) / (double)stepsI;

		for (size_t j = 0; j < stepsJ; j++) {
			double phi0 = M_PI * (-0.5 + j / (double)stepsJ);
			double phi1 = M_PI * (-0.5 + (j + 1) / (double)stepsJ);

			Vector3d pt00 = origin + cos(phi0) * (cos(alpha0) * xAxis + sin(alpha0) * yAxis) + sin(phi0) * zAxis;
			Vector3d pt01 = origin + cos(phi0) * (cos(alpha1) * xAxis + sin(alpha1) * yAxis) + sin(phi0) * zAxis;
			Vector3d pt10 = origin + cos(phi1) * (cos(alpha0) * xAxis + sin(alpha0) * yAxis) + sin(phi1) * zAxis;
			Vector3d pt11 = origin + cos(phi1) * (cos(alpha1) * xAxis + sin(alpha1) * yAxis) + sin(phi1) * zAxis;
			if (j == 0) {
				pMesh->addTriangle(pt00, pt11, pt10);
			}
			else if (j == stepsJ - 1) {
				pMesh->addTriangle(pt00, pt01, pt11);
			}
			else {
				pMesh->addTriangle(pt00, pt01, pt11);
				pMesh->addTriangle(pt00, pt11, pt10);
			}
		}
	}
}

CMeshPtr MeshData::getSharpVertMesh() const
{
	auto bBox = _pMesh->getBBox();
	double span = bBox.range().norm();
	double radius = span / 500;
	bBox.grow(2 * radius);

	vector<size_t> sVerts;
	Volume::findSharpVertices(_pMesh, SHARP_EDGE_ANGLE_RADIANS, sVerts);
	if (!sVerts.empty()) {
		CMeshPtr pMesh = make_shared<CMesh>(bBox, _pRepo);
		for (size_t vertIdx : sVerts) {
			auto pt = _pMesh->getVert(vertIdx)._pt;
			addPointMarker(pMesh, pt, radius);
		}

		return pMesh;
	}

	return nullptr;
}

void MeshData::setActive(bool val)
{
	if (_active == val)
		return;

	_active = val;
	
}

