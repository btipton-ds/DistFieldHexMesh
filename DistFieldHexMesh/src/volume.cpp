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

#include <sstream>
#include <fstream>

#define _USE_MATH_DEFINES
#include <cmath>

#include <triMesh.h>

#include <block.h>
#include <volume.h>
#include <MultiCoreUtil.h>
#include <vertex.h>
#include <filesystem>
#include <stdexcept>

using namespace std;
using namespace DFHM;
using namespace TriMesh;

#define RUN_MULTI_THREAD true

Index3D Volume::s_volDim;

Volume::Volume()
{
	_sharpAngleRad = 30.0 / 180.0 * M_PI;
}

Volume::Volume(const Volume& src)
	: _originMeters(src._originMeters) 
	, _spanMeters(src._spanMeters)
	, _blocks(src._blocks)
{
}

Volume::~Volume()
{
}

void Volume::startOperation()
{
}

void Volume::endOperation()
{
}

void Volume::setVolDim(const Index3D& blockSize)
{
	s_volDim = blockSize;
}

const Index3D& Volume::volDim()
{
	return s_volDim;
}

Index3D Volume::calBlockIndexFromLinearIndex(size_t linearIdx) const
{
	Index3D result;
	size_t temp = linearIdx;
	const auto& dim = volDim();

	size_t denom = dim[0] * dim[1];
	result[2] = (Index3DBaseType)(temp / denom);
	temp = temp % denom;

	denom = dim[0];

	result[1] = (Index3DBaseType)(temp / denom);
	temp = temp % denom;
	result[0] = (Index3DBaseType)temp;

	if (calLinearBlockIndex(result) != linearIdx) {
		throw runtime_error("calBlockIndexFromLinearIndex failed");
	}

	return result;
}

void Volume::findFeatures()
{
	// This function is already running on multiple cores, DO NOT calculate centroids or normals using muliple cores.
	_pModelTriMesh->buildCentroids(false);
	_pModelTriMesh->buildNormals(false);

	findSharpVertices();
	findSharpEdgeGroups();
}

void Volume::findSharpEdgeGroups()
{
	const double sinSharpAngle = sin(getSharpAngleRad());
	for (size_t edgeIdx = 0; edgeIdx < _pModelTriMesh->numEdges(); edgeIdx++) {
		const auto& edge = _pModelTriMesh->getEdge(edgeIdx);
		if (edge._numFaces == 2 && _pModelTriMesh->isEdgeSharp(edgeIdx, sinSharpAngle)) {
			_sharpEdgeIndices.insert(edgeIdx);
		}
	}
}

void Volume::findSharpVertices()
{
	const double cosSharpAngle = cos(M_PI - getSharpAngleRad());

	size_t numVerts = _pModelTriMesh->numVertices();
	for (size_t vIdx = 0; vIdx < numVerts; vIdx++) {
		const auto& vert = _pModelTriMesh->getVert(vIdx);
		double maxDp = 1;
		const auto& edgeIndices = vert._edgeIndices;
		for (size_t i = 0; i < edgeIndices.size(); i++) {
			auto edge0 = _pModelTriMesh->getEdge(edgeIndices[i]);
			size_t opIdx0 = edge0._vertIndex[0] == vIdx ? edge0._vertIndex[1] : edge0._vertIndex[0];
			const auto& vert0 = _pModelTriMesh->getVert(opIdx0);
			Vector3d v0 = (vert0._pt - vert._pt).normalized();

			for (size_t j = i + 1; j < edgeIndices.size(); j++) {
				auto edge1 = _pModelTriMesh->getEdge(edgeIndices[j]);
				size_t opIdx1 = edge1._vertIndex[0] == vIdx ? edge1._vertIndex[1] : edge1._vertIndex[0];
				const auto& vert1 = _pModelTriMesh->getVert(opIdx1);
				Vector3d v1 = (vert1._pt - vert._pt).normalized();

				double dp = v0.dot(v1);
				if (dp < maxDp) {
					maxDp = dp;
					if (maxDp < cosSharpAngle)
						break;
				}
			}
			if (maxDp < cosSharpAngle)
				break;
		}
		if (maxDp > cosSharpAngle) {
			_sharpVertIndices.insert(vIdx);
		}
	}
}

shared_ptr<Block> Volume::createBlock(const Index3D& blockIdx)
{
	const auto& dim = volDim();
	const Vector3d xAxis(1, 0, 0);
	const Vector3d yAxis(0, 1, 0);
	const Vector3d zAxis(0, 0, 1);

	size_t idx = calLinearBlockIndex(blockIdx);
	auto pBlock = _blocks[idx];
	if (pBlock)
		return pBlock;

	Vector3d origin, span;
	for (int i = 0; i < 3; i++) {
		span[i] = _spanMeters[i] / dim[i];
	}

	origin[0] = _originMeters[0] + blockIdx[0] * span[0];
	origin[1] = _originMeters[1] + blockIdx[1] * span[1];
	origin[2] = _originMeters[2] + blockIdx[2] * span[2];
	vector<Vector3d> pts = {
		origin,
		origin + xAxis * span[0],
		origin + xAxis * span[0] + yAxis * span[1],
		origin + yAxis * span[1],

		origin + zAxis * span[2],
		origin + zAxis * span[2] + xAxis * span[0],
		origin + zAxis * span[2] + xAxis * span[0] + yAxis * span[1],
		origin + zAxis * span[2] + yAxis * span[1],
	};

	return make_shared<Block>(this, blockIdx, pts);
}

void Volume::addAllBlocks(Block::TriMeshGroup& triMeshes, Block::glPointsGroup& faceEdges)
{
	const auto& dim = volDim();
	Vector3d origin, blockSpan;
	for (int i = 0; i < 3; i++) {
		blockSpan[i] = _spanMeters[i] / dim[i];
	}

	Index3D blkIdx;
	for (blkIdx[0] = 0; blkIdx[0] < dim[0]; blkIdx[0]++) {
		origin[1] = _originMeters[1] + blkIdx[0] * blockSpan[1];
		for (blkIdx[1] = 0; blkIdx[1] < dim[1]; blkIdx[1]++) {
			origin[1] = _originMeters[1] + blkIdx[1] * blockSpan[1];

			for (blkIdx[2] = 0; blkIdx[2] < dim[2]; blkIdx[2]++) {
				origin[2] = _originMeters[2] + blkIdx[2] * blockSpan[2];
				auto linearIdx = calLinearBlockIndex(blkIdx);
				_blocks[linearIdx] = createBlock(blkIdx);
			}
		}
	}

	makeFaceTris(triMeshes, true);
}

bool Volume::blockExists(const Index3D& blockIdx) const
{
	size_t idx = calLinearBlockIndex(blockIdx);
	if (idx >= _blocks.size())
		return false;
	return _blocks[idx] != nullptr;
}

void Volume::buildCFDHexes(const CMeshPtr& pTriMesh, const BuildCFDParams& params)
{
	_pModelTriMesh = pTriMesh;
	CMesh::BoundingBox bb = pTriMesh->getBBox();
	bb.growPercent(0.0125);
	_originMeters = bb.getMin();
	_spanMeters = bb.range();

	double minSpan = DBL_MAX;
	for (int i = 0; i < 3; i++) {
		if (_spanMeters[i] < minSpan) {
			minSpan = _spanMeters[i];
		}
	}
	double targetBlockSize = minSpan / params.minBlocksPerSide;
	size_t blockDim = 1;
	Index3D::setBlockDim(blockDim);

	Index3D blockSize(
		(size_t)(_spanMeters[0] / targetBlockSize + 0.5),
		(size_t)(_spanMeters[1] / targetBlockSize + 0.5),
		(size_t)(_spanMeters[2] / targetBlockSize + 0.5)
	);

	setVolDim(blockSize);

	const auto& dim = volDim();
	Vector3d blockSpan(_spanMeters[0] / dim[0], _spanMeters[1] / dim[1], _spanMeters[2] / dim[2]);

	size_t numBlocks = dim[0] * dim[1] * dim[2];
	_blocks.resize(numBlocks);

	double sharpAngleRadians = params.sharpAngleDegrees / 180.0 * M_PI;
	double sinEdgeAngle = sin(sharpAngleRadians);
	_pModelTriMesh->buildCentroids(true);
	_pModelTriMesh->buildNormals(true);
	_pModelTriMesh->calCurvatures(sharpAngleRadians, true);
	const auto& sharpEdges = _pModelTriMesh->getSharpEdgeIndices(sharpAngleRadians);
	findFeatures();

	MultiCore::runLambda([this, &blockSpan](size_t linearIdx)->bool {
		auto blockIdx = calBlockIndexFromLinearIndex(linearIdx);
		_blocks[linearIdx] = createBlock(blockIdx);
		return true;
	}, _blocks.size(), RUN_MULTI_THREAD);

	// Cannot create subBlocks until all blocks are created
	runLambda([this, &blockSpan](size_t linearIdx)->bool {
		if (_blocks[linearIdx])
			_blocks[linearIdx]->createSubBlocks();
		return true;
	}, RUN_MULTI_THREAD);

//	assert(verifyTopology());

#if 1
	for (size_t i = 0; i < params.numSimpleDivs; i++) {
		runLambda([this](size_t linearIdx)->bool {
			if (_blocks[linearIdx]) {
				_blocks[linearIdx]->splitAllCellsAtCentroid();
			}
			return true;
		}, RUN_MULTI_THREAD);
	}
//	assert(verifyTopology());

	for (size_t i = 0; i < params.numCurvatureDivs; i++) {
		runLambda([this, &params, sinEdgeAngle](size_t linearIdx)->bool {
			if (_blocks[linearIdx]) {
				_blocks[linearIdx]->splitAllCellsByCurvature(params.divsPerRadius, params.maxCurvatureRadius, sinEdgeAngle);
			}
			return true;
		}, RUN_MULTI_THREAD);
	}
	assert(verifyTopology());
#endif

	cout << "Num polyhedra: " << numPolyhedra() << "\n";
	cout << "Num faces. All: " << numFaces(true) << ", outer: " << numFaces(false) << "\n";
}

void Volume::makeFaceTris(Block::TriMeshGroup& triMeshes, bool multiCore) const
{
	CBoundingBox3Dd bbox;
	bbox.merge(_originMeters);
	bbox.merge(_originMeters + _spanMeters);
	auto diagDist = bbox.range().norm();
	bbox.grow(diagDist * 0.05);


	triMeshes.resize(4);
	triMeshes[FT_OUTER].resize(_blocks.size());
	triMeshes[FT_INNER].resize(_blocks.size());
	triMeshes[FT_LAYER_BOUNDARY].resize(_blocks.size());
	triMeshes[FT_BLOCK_BOUNDARY].resize(_blocks.size());
	MultiCore::runLambda([this, &triMeshes](size_t index)->bool {
		const auto& blockPtr = _blocks[index];
		if (blockPtr) {

			triMeshes[FT_OUTER][index] = blockPtr->getBlockTriMesh(FT_OUTER);
			triMeshes[FT_INNER][index] = blockPtr->getBlockTriMesh(FT_INNER);
			triMeshes[FT_LAYER_BOUNDARY][index] = blockPtr->getBlockTriMesh(FT_LAYER_BOUNDARY);
			triMeshes[FT_BLOCK_BOUNDARY][index] = blockPtr->getBlockTriMesh(FT_BLOCK_BOUNDARY);
		}
		return true;
	}, _blocks.size(), multiCore && RUN_MULTI_THREAD);
}

void Volume::makeEdgeSets(Block::glPointsGroup& faceEdges, bool multiCore) const
{
	faceEdges.resize(4);
	faceEdges[FT_OUTER].resize(_blocks.size());
	faceEdges[FT_INNER].resize(_blocks.size());
	faceEdges[FT_LAYER_BOUNDARY].resize(_blocks.size());
	faceEdges[FT_BLOCK_BOUNDARY].resize(_blocks.size());
	MultiCore::runLambda([this, &faceEdges](size_t index)->bool {
		const auto& blockPtr = _blocks[index];
		if (blockPtr) {
			faceEdges[FT_OUTER][index] = blockPtr->makeEdgeSets(FT_OUTER);
			faceEdges[FT_INNER][index] = blockPtr->makeEdgeSets(FT_INNER);
			faceEdges[FT_LAYER_BOUNDARY][index] = blockPtr->makeEdgeSets(FT_BLOCK_BOUNDARY);
			faceEdges[FT_BLOCK_BOUNDARY][index] = blockPtr->makeEdgeSets(FT_BLOCK_BOUNDARY);
		}
		return true;
	}, _blocks.size(), multiCore && RUN_MULTI_THREAD);
}

size_t Volume::numFaces(bool includeInner) const
{
	size_t result = 0;
	for (auto pBlock : _blocks) {
		if (pBlock)
			result += pBlock->numFaces(includeInner);
	}

	return result;
}

size_t Volume::numPolyhedra() const
{
	size_t result = 0;
	for (auto pBlock : _blocks) {
		if (pBlock)
			result += pBlock->numPolyhedra();
	}

	return result;
}

namespace {
	void appendDir(string& dirName, const string& str)
	{
		if (dirName.find_last_of("/") != dirName.length() - 1)
			dirName += "/";
		dirName += str;
	}

	void unifyDirectorySeparator(string& dirName)
	{
		auto pos = dirName.find("\\");
		while (pos != string::npos) {
			dirName.replace(pos, 1, "/");
			pos = dirName.find("\\");
		}
	}
}

void Volume::consolidateBlocks()
{
	Index3D blkIdx;
#if 0 && defined(_DEBUG)
	set<FixedPt> pts;

	for (blkIdx[0] = 0; blkIdx[0] < s_volDim[0]; blkIdx[0]++) {
		for (blkIdx[1] = 0; blkIdx[1] < s_volDim[1]; blkIdx[1]++) {
			for (blkIdx[2] = 0; blkIdx[2] < s_volDim[2]; blkIdx[2]++) {
				auto pBlk = _blocks[calLinearBlockIndex(blkIdx)];
				if (pBlk) {
					pBlk->_vertices.iterateInOrder([&pts](const Vertex& vert) {
						auto fpt = vert.getFixedPt();
						assert(pts.count(fpt) == 0);
						pts.insert(fpt);
					});
				}
			}
		}
	}
#endif
	assert(verifyTopology());
	// Set block baseIndices. This orders them, does not ignore "dead" ones and does not pack them
	size_t vertIdx = 0, polygonIdx = 0, polyhedronIdx = 0;
	for (blkIdx[0] = 0; blkIdx[0] < s_volDim[0]; blkIdx[0]++) {
		for (blkIdx[1] = 0; blkIdx[1] < s_volDim[1]; blkIdx[1]++) {
			for (blkIdx[2] = 0; blkIdx[2] < s_volDim[2]; blkIdx[2]++) {
				size_t linIdx = calLinearBlockIndex(blkIdx);
				auto pBlk = _blocks[linIdx];
				if (pBlk) {
					pBlk->_baseIdxVerts = vertIdx;
					pBlk->_baseIdxPolygons = polygonIdx;
					pBlk->_baseIdxPolyhedra = polyhedronIdx;

					pBlk->_vertices.iterateInOrder([&vertIdx](const Vertex& v) {
						vertIdx++;
					});
					pBlk->_polygons.iterateInOrder([&polygonIdx](const Polygon& v) {
						polygonIdx++;
					});
					pBlk->_polyhedra.iterateInOrder([&polyhedronIdx](const Polyhedron& v) {
						polyhedronIdx++;
					});
				}
			}
		}
	}

	vector<Index3DId> pts;
	map<Index3DId, size_t> idToPointIdxMap;
	vector<size_t> faceIndices, faces;
	for (blkIdx[0] = 0; blkIdx[0] < s_volDim[0]; blkIdx[0]++) {
		for (blkIdx[1] = 0; blkIdx[1] < s_volDim[1]; blkIdx[1]++) {
			for (blkIdx[2] = 0; blkIdx[2] < s_volDim[2]; blkIdx[2]++) {
				auto pBlk = _blocks[calLinearBlockIndex(blkIdx)];
				if (!pBlk)
					continue;
				pBlk->_polygons.iterateInOrder([&pts, &idToPointIdxMap, &faceIndices, &faces](Polygon& face) {
					if (!face.isActive())
						return;
					face.orient();
					faceIndices.push_back(faces.size());
					for (const auto& vertId : face.getVertexIds()) {
						auto iter = idToPointIdxMap.find(vertId);
						if (iter == idToPointIdxMap.end()) {
							size_t idx = pts.size();
							iter = idToPointIdxMap.insert(make_pair(vertId, idx)).first;
							pts.push_back(vertId);
						}
						size_t vertIdx = iter->second;
						faces.push_back(vertIdx);
					}
				});
			}
		}
	}

	int dbgBreak = 1;
}

void Volume::writeObj(const string& path, const vector<Index3DId>& cellIds) const
{
	ofstream out(path, ios_base::trunc);
	writeObj(out, cellIds);
}

void Volume::writeObj(ostream& out, const vector<Index3DId>& cellIds) const
{
	set<Index3DId> faceIds;
	for (const auto& cellId : cellIds) {
		auto pBlk = getBlockPtr(cellId);
		pBlk->cellFunc(cellId, [&faceIds](const Polyhedron& cell) {
			const auto& ids = cell.getFaceIds();
			faceIds.insert(ids.begin(), ids.end());
		});
	}

#if 1
	for (const auto& faceId : faceIds) {
		auto pBlk = getBlockPtr(faceId);
		pBlk->faceFunc(faceId, [&pBlk, &faceIds](const Polygon& face) {
			const auto& ids = face.getChildIds();
			faceIds.insert(ids.begin(), ids.end());
		});
	}
#endif

	vector<Vector3d> pts;
	map<Index3DId, size_t> vertIdToPtMap;

	for (const auto& faceId : faceIds) {
		auto pBlk = getBlockPtr(faceId);
		pBlk->faceFunc(faceId, [&pBlk, &vertIdToPtMap, &pts](const Polygon& face) {
			const auto& vIds = face.getVertexIds();
			for (const auto& vertId : vIds) {
				auto iter = vertIdToPtMap.find(vertId);
				if (iter == vertIdToPtMap.end()) {
					size_t vertIdx = pts.size();
					vertIdToPtMap.insert(make_pair(vertId, vertIdx));
					pts.push_back(pBlk->getVertexPoint(vertId));
				}
			}
		});
	}

	out << "#Vertices\n";
	for (const auto& pt : pts) {
		out << "v " << pt[0] << " " << pt[1] << " " << pt[2] << "\n";
	}

	out << "#Faces\n";
	for (const auto& faceId : faceIds) {
		auto pBlk = getBlockPtr(faceId);
		pBlk->faceFunc(faceId, [&out, &vertIdToPtMap](const Polygon& face) {
			out << "f ";
			const auto& vIds = face.getVertexIds();
			for (const auto& vertId : vIds) {
				auto iter = vertIdToPtMap.find(vertId);
				if (iter != vertIdToPtMap.end()) {
					size_t idx = iter->second + 1;
					out << idx << " ";
				}
			}
			out << "\n";
		});
	}
}

void Volume::writePolyMesh(const string& dirNameIn)
{
	consolidateBlocks();

	string dirName(dirNameIn);

	unifyDirectorySeparator(dirName);

	auto pos = dirName.find("constant");
	if (pos == string::npos) {
		appendDir(dirName, "constant");
	} else {
		dirName = dirName.substr(0, pos + 9);
	}
	if (dirName.find("polyMesh") == string::npos)
		appendDir(dirName, "polyMesh/");
	
	filesystem::path dirPath(dirName);
	filesystem::remove_all(dirPath);
	filesystem::create_directories(dirPath);
	consolidateBlocks();
	writePolyMeshPoints(dirName);
}

void Volume::writePolyMeshPoints(const string& dirName) const
{
	ofstream out(dirName + "/points", ios_base::binary);
	writeFOAMHeader(out, "vectorField", "points");
	size_t numPoints = 0;

	Vector3i blkIdx;
	for (blkIdx[0] = 0; blkIdx[0] < s_volDim[0]; blkIdx[0]++) {
		for (blkIdx[1] = 0; blkIdx[1] < s_volDim[1]; blkIdx[1]++) {
			for (blkIdx[2] = 0; blkIdx[2] < s_volDim[2]; blkIdx[2]++) {
				size_t linIdx = calLinearBlockIndex(blkIdx);
				auto& pBlk = _blocks[linIdx];
				if (!pBlk)
					continue;
				pBlk->_baseIdxVerts = numPoints;
				const auto& blkVerts = pBlk->_vertices;
				blkVerts.iterateInOrder([&numPoints](const Vertex& vert) {
					numPoints++;
				});
			}
		}
	}

	out << numPoints << "\n";
	for (blkIdx[0] = 0; blkIdx[0] < s_volDim[0]; blkIdx[0]++) {
		for (blkIdx[1] = 0; blkIdx[1] < s_volDim[1]; blkIdx[1]++) {
			for (blkIdx[2] = 0; blkIdx[2] < s_volDim[2]; blkIdx[2]++) {
				size_t linIdx = calLinearBlockIndex(blkIdx);
				const auto& pBlk = _blocks[linIdx];
				if (!pBlk)
					continue;
				const auto& blkVerts = pBlk->_vertices;
				blkVerts.iterateInOrder([&numPoints, &out](const Vertex& vert) {
					char openParen = '(';
					char closeParen = ')';
					auto pt = vert.getPoint();

					const double& x = pt[0];
					const double& y = pt[1];
					const double& z = pt[2];

					out.write(&openParen, 1);
					out.write((char*)&x, sizeof(x));
					out.write((char*)&y, sizeof(y));
					out.write((char*)&z, sizeof(z));
					out.write(&closeParen, 1);
				});
			}
		}
	}

}

void Volume::writePolyMeshFaces(const string& dirName) const
{
	ofstream out(dirName + "/faces", ios_base::binary);
	writeFOAMHeader(out, "faceCompactList", "faces");
	vector<size_t> startIndices, vertIndices;

	Vector3i blkIdx;
	for (blkIdx[0] = 0; blkIdx[0] < s_volDim[0]; blkIdx[0]++) {
		for (blkIdx[1] = 0; blkIdx[1] < s_volDim[1]; blkIdx[1]++) {
			for (blkIdx[2] = 0; blkIdx[2] < s_volDim[2]; blkIdx[2]++) {
				size_t linIdx = calLinearBlockIndex(blkIdx);
				auto& pBlk = _blocks[linIdx];
				if (!pBlk)
					continue;
				pBlk->_baseIdxPolygons = startIndices.size();
				const auto& blkPolygons = pBlk->_polygons;
				blkPolygons.iterateInOrder([&startIndices](const Polygon& face) {
					const auto& vertIds = face.getVertexIds();
					startIndices.push_back(vertIds.size());
				});
			}
		}
	}

	out << startIndices.size() << "\n";

}

void Volume::writeFOAMHeader(ofstream& out, const string& foamClass, const string& object) const
{
	out << "/*--------------------------------*- C++ -*----------------------------------*/\n";
	out << "| =========                 |                                                 |\n";
	out << "| \\ / F ield | OpenFOAM: The Open Source CFD Toolbox           |\n";
	out << "| \\ / O peration | Version:  v1812                                 |\n";
	out << "|   \\ / A nd | Web:      www.OpenFOAM.com                      |\n";
	out << "|    \\ / M anipulation  |                                                 |\n";
	out << "/* -------------------------------------------------------------------------- - */\n";
	out << "FoamFile\n";
	out << "	{\n";
	out << "		version     2.0;\n";
	out << "		format      binary;\n";
	out << "		class       " << foamClass << ";\n";
	out << "		arch        \"LSB; label = 32; scalar = 64\";\n";
	out << "		location    \"constant / polyMesh\";\n";
	out << "		object      " << object << ";\n";
	out << "	}\n";
	out << "		// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * //\n";
	out << "\n";
	out << "\n";

}

bool Volume::verifyTopology() const
{
	bool result = true;

	runLambda([this, &result](size_t linearIdx)->bool {
		if (_blocks[linearIdx])
			result = result && _blocks[linearIdx]->verifyTopology();
		return true;
	}, RUN_MULTI_THREAD);
	return result;
}

template<class L>
void Volume::runLambda(L fLambda, bool multiCore) const
{
	MultiCore::runLambda([this, fLambda](size_t threadNum, size_t numThreads) {
		Index3D idx;
		for (size_t i = threadNum; i < _blocks.size(); i += numThreads) {
			fLambda(i);
		}
	}, multiCore);
}

template<class L>
void Volume::runLambda(L fLambda, bool multiCore)
{
	const Index3DBaseType stride = 3;
	Index3D phaseIdx, idx;

	startOperation();

	// Pass one, process all cells. That can leave faces in an interim state.
	// If the interim cell is also modified, it should be taken care of on pass 1
	// Adjacents cells with face splits or vertex insertions may be left behind
	for (phaseIdx[0] = 0; phaseIdx[0] < stride; phaseIdx[0]++) {
		for (phaseIdx[1] = 0; phaseIdx[1] < stride; phaseIdx[1]++) {
			for (phaseIdx[2] = 0; phaseIdx[2] < stride; phaseIdx[2]++) {
				// Collect the indices for all blocks in this phase
				vector<size_t> blocksToProcess;

				for (idx[0] = phaseIdx[0]; idx[0] < s_volDim[0]; idx[0] += stride) {
					for (idx[1] = phaseIdx[1]; idx[1] < s_volDim[1]; idx[1] += stride) {
						for (idx[2] = phaseIdx[2]; idx[2] < s_volDim[2]; idx[2] += stride) {
							size_t linearIdx = calLinearBlockIndex(idx);
							blocksToProcess.push_back(linearIdx);
						}
					}
				}

				// Process those blocks in undetermined order
				MultiCore::runLambda([fLambda](size_t linearIdx)->bool {
					return fLambda(linearIdx);
				}, blocksToProcess, multiCore);

			}
		}
	}

	endOperation();
}
