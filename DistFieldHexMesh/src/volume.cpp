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

#include <defines.h>
#include <cmath>

#include <triMesh.h>

#include <block.h>
#include <volume.h>
#include <MultiCoreUtil.h>
#include <vertex.h>
#include <filesystem>
#include <stdexcept>

#ifdef _WIN32
#include <windows.h>
#include <profileapi.h>
#endif // _WIN32


using namespace std;
using namespace DFHM;
using namespace TriMesh;

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
	MultiCore::runLambda([this](size_t linearIdx)->bool {
		_blocks[linearIdx] = nullptr;
		return true;
	},_blocks.size(), RUN_MULTI_THREAD);
	_blocks.clear();
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

#ifdef _DEBUG
bool Volume::isPolygonInUse(const Index3DId& faceId) const
{
	bool result = false;

	lock_guard g(_mutex);
	for (const auto& pBlk : _blocks) {
		if (pBlk) {
			const auto& cells = pBlk->_modelData._polyhedra;
			cells.iterateInOrder([&result, &faceId](const Index3DId& cellId, const Polyhedron& cell) {
				if (cell.containsFace(faceId)) {
					result = true;
				}
			});
		}

		if (result)
			break;
	}

	return result;
}

bool Volume::isPolyhedronInUse(const Index3DId& cellId) const
{
	bool result = false;

	lock_guard g(_mutex);
	for (const auto& pBlk : _blocks) {
		if (pBlk) {
			const auto& faces = pBlk->_modelData._polygons;
			faces.iterateInOrder([&result, &cellId](const Index3DId& faceId, const Polygon& face) {
				if (face.usedByCell(cellId)) {
					result = true;
				}
			});
		}

		if (result)
			break;
	}

	return result;
}
#endif // _DEBUG

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

void Volume::buildCFDHexes(const CMeshPtr& pTriMesh, const BuildCFDParams& params, bool multiCore)
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

	for (size_t i = 0; i < params.numBlockDivs; i++) {
		for (int i = 0; i < 3; i++)
			blockSize[i] *= 2;
	}
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

#ifdef _WIN32
	LARGE_INTEGER startCount, freq;
	QueryPerformanceFrequency(&freq);
	QueryPerformanceCounter(&startCount);
#endif // _WIN32

	MultiCore::runLambda([this, &blockSpan](size_t linearIdx)->bool {
		auto blockIdx = calBlockIndexFromLinearIndex(linearIdx);
		Block::setThreadBlockIdx(calBlockIndexFromLinearIndex(linearIdx));
		_blocks[linearIdx] = createBlock(blockIdx);
		return true;
	}, _blocks.size(), multiCore);

	// Cannot create subBlocks until all blocks are created so they can be connected
	runLambda([this, &blockSpan](size_t linearIdx)->bool {
		Block::setThreadBlockIdx(calBlockIndexFromLinearIndex(linearIdx));
		if (_blocks[linearIdx])
			_blocks[linearIdx]->createSubBlocks(TS_REAL);
		return true;
	}, multiCore);

#ifdef _WIN32
	LARGE_INTEGER endCount;
	QueryPerformanceCounter(&endCount);
	double deltaT = (endCount.QuadPart - startCount.QuadPart) / (double)(freq.QuadPart);
	cout << "Time for createSubBlocks: " << deltaT << " secs\n";
	startCount = endCount;
#endif // _WIN32

	if (params.numSimpleDivs > 0) {
#if 1
		splitSimple(params, multiCore);
#ifdef _WIN32
		QueryPerformanceCounter(&endCount);
		deltaT = (endCount.QuadPart - startCount.QuadPart) / (double)(freq.QuadPart);
		cout << "Time for splitAllCellsAtCentroid: " << deltaT << " secs\n";
		startCount = endCount;
#endif // _WIN32
//		assert(verifyTopology(multiCore));
	}

	if (params.numCurvatureDivs > 0) {
#ifdef _WIN32
		startCount = endCount;
#endif // _WIN32
		splitAtCurvature(params, multiCore);

#ifdef _WIN32
		QueryPerformanceCounter(&endCount);
		deltaT = (endCount.QuadPart - startCount.QuadPart) / (double)(freq.QuadPart);
		cout << "Time for splitAllCellsByCurvature: " << deltaT << " secs\n";
		startCount = endCount;
#endif // _WIN32
#endif
	}

	cout << "Num polyhedra: " << numPolyhedra() << "\n";
	cout << "Num faces. All: " << numFaces(true) << ", outer: " << numFaces(false) << "\n";
}

/*
	Polyhedra are owned by a single block, but their faces and vertices may belong to different blocks.

	Once a polyhedron or polygon is split, it is kept (and marked) for reference but is otherwise "dead."

	Mark all faces and polyhedra to be split on one pass
		If a split would result in a double split of a polyhedron, mark that sell to be split at a higher "phase". Cell will be split by decending phase
	Split polyhedra which must be split due to a Adjacent split in descending phase. There should only be one, but this supports more

	Reference polyhedra must be TOTALLY intact - for simplicity.
		This happens automatically when a polyhedron is split
		It must happen implicitly when a polygon has a vertex imprinted.

*/

void Volume::splitSimple(const BuildCFDParams& params, bool multiCore)
{
	for (size_t i = 0; i < params.numSimpleDivs; i++) {
#ifdef LOGGING_ENABLED
		runLambda([this, i](size_t linearIdx)->bool {
			Block::setThreadBlockIdx(calBlockIndexFromLinearIndex(linearIdx));
			if (_blocks[linearIdx]) {
				auto logger = _blocks[linearIdx]->getLogger();
				auto& out = logger->getStream();
				LOG(out << "\n");
				LOG(out << "*****************************************************************************************************************\n");
				LOG(out << "splitSimple(" << i << ")  ********************************************************************************************\n");
				LOG(out << "*****************************************************************************************************************\n");
			}
			return true;
		}, multiCore);
#endif // LOGGING_ENABLED

		runLambda([this](size_t linearIdx)->bool {
			Block::setThreadBlockIdx(calBlockIndexFromLinearIndex(linearIdx));
			if (_blocks[linearIdx]) {
				_blocks[linearIdx]->setNeedsSimpleSplit();
			}
			return true;
		},  multiCore);

		FinishSplitOptions options;
		options._processPartialSplits = false;
		options._processEdgesWithTVertices = false;
		finishSplits(options, multiCore);
	}
}

void Volume::splitAtCurvature(const BuildCFDParams& params, bool multiCore)
{
	double sharpAngleRadians = params.sharpAngleDegrees / 180.0 * M_PI;
	double sinEdgeAngle = sin(sharpAngleRadians);

	size_t num = params.numCurvatureDivs;
	for (size_t i = 0; i < num; i++) {
#ifdef LOGGING_ENABLED
		runLambda([this, i](size_t linearIdx)->bool {
			Block::setThreadBlockIdx(calBlockIndexFromLinearIndex(linearIdx));
			if (_blocks[linearIdx]) {
				auto logger = _blocks[linearIdx]->getLogger();
				auto& out = logger->getStream();
				LOG(out << "\n");
				LOG(out << "*****************************************************************************************************************\n");
				LOG(out << "splitAtCurvature(" << i << ")  ********************************************************************************************\n");
				LOG(out << "*****************************************************************************************************************\n");
			}
			return true;
		}, multiCore);
#endif // LOGGING_ENABLED

		runLambda([this, &params, sinEdgeAngle](size_t linearIdx)->bool {
			Block::setThreadBlockIdx(calBlockIndexFromLinearIndex(linearIdx));
			if (_blocks[linearIdx]) {
				_blocks[linearIdx]->setNeedsCurvatureSplit(params.divsPerRadius, params.maxCurvatureRadius, sinEdgeAngle);
			}
			return true;
		},  multiCore);

		FinishSplitOptions options;
		options._processPartialSplits = i > 0;
		options._processEdgesWithTVertices = true;
		bool mc = (i < params.numCurvatureDivs - 1) && multiCore;
		finishSplits(options, mc);
		assert(verifyTopology(mc));
	}
}

void Volume::finishSplits(const FinishSplitOptions& options, bool multiCore)
{
#if 1
	if (options._processPartialSplits) {
		bool didSplits = doPresplits(multiCore);
		while (didSplits) {
			didSplits = doPresplits(multiCore);
		}
		int dbgBreak = 1;
	}
#endif

	splitTopology(options, multiCore);
//	assert(verifyTopology(multiCore));

//	dumpOpenCells(multiCore);
}

bool Volume::doPresplits(bool multiCore)
{
	atomic<bool> didSplits = false;

#ifdef LOGGING_ENABLED
	runLambda([this, &didSplits](size_t linearIdx)->bool {
		Block::setThreadBlockIdx(calBlockIndexFromLinearIndex(linearIdx));
		if (_blocks[linearIdx]) {
			auto pBlk = _blocks[linearIdx];
			auto logger = pBlk->getLogger();
			auto& out = logger->getStream();
			LOG(out << "*******************************************************************************************\n");
			LOG(out << "doPresplits_splitPolyhedra****************************************************************************\n");
			LOG(out << "*******************************************************************************************\n");
		}
		return true;
	}, multiCore);
#endif // LOGGING_ENABLED


	runLambda([this, &didSplits](size_t linearIdx)->bool {
		Block::setThreadBlockIdx(calBlockIndexFromLinearIndex(linearIdx));
		if (_blocks[linearIdx]) {
			if (_blocks[linearIdx]->doPresplits_splitPolyhedra())
				didSplits = true;
		}
		return true;
	}, multiCore);

	// We need to imprint TJoints because these splits, may split edges in adjacent cells
	imprintTJointVertices(multiCore);

	return didSplits;
}

void Volume::splitTopology(const FinishSplitOptions& options, bool multiCore)
{
	runLambda([this](size_t linearIdx)->bool {
		Block::setThreadBlockIdx(calBlockIndexFromLinearIndex(linearIdx));
		if (_blocks[linearIdx]) {
			_blocks[linearIdx]->splitRequiredPolyhedra();
		}
		return true;
	}, multiCore);

	if (options._processEdgesWithTVertices) {
		imprintTJointVertices(multiCore);
	}

}

void Volume::imprintTJointVertices(bool multiCore)
{
	runLambda([this](size_t linearIdx)->bool {
		Block::setThreadBlockIdx(calBlockIndexFromLinearIndex(linearIdx));
		if (_blocks[linearIdx]) {
			_blocks[linearIdx]->imprintTJointVertices();
		}
		return true;
	},  multiCore);
}

void Volume::dumpOpenCells(bool multiCore) const
{
#if DUMP_OPEN_CELL_OBJS
	Block::setThreadBlockIdx(calBlockIndexFromLinearIndex(linearIdx));
	runLambda([this](size_t linearIdx)->bool {
		if (_blocks[linearIdx]) {
			_blocks[linearIdx]->dumpOpenCells();
		}
		return true;
	}, multiCore);
#endif
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
	}, _blocks.size(), multiCore);
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
	}, _blocks.size(), multiCore);
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
					pBlk->_vertices.iterateInOrder([&pts](const Index3DId& id, const Vertex& vert) {
						auto fpt = vert.getFixedPt();
						assert(pts.count(fpt) == 0);
						pts.insert(fpt);
					});
				}
			}
		}
	}
#endif
	assert(verifyTopology(true));
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

					pBlk->_vertices.iterateInOrder([&vertIdx](const Index3DId& id, const Vertex& v) {
						vertIdx++;
					});
					pBlk->_modelData._polygons.iterateInOrder([&polygonIdx](const Index3DId& id, const Polygon& v) {
						polygonIdx++;
					});
					pBlk->_modelData._polyhedra.iterateInOrder([&polyhedronIdx](const Index3DId& id, const Polyhedron& v) {
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
				pBlk->_modelData._polygons.iterateInOrder([&pts, &idToPointIdxMap, &faceIndices, &faces](const Index3DId& id, Polygon& face) {
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
		pBlk->cellRealFunc(cellId, [&faceIds](const Polyhedron& cell) {
			const auto& ids = cell.getFaceIds();
			faceIds.insert(ids.begin(), ids.end());
		});
	}

	vector<Vector3d> pts;
	map<Index3DId, size_t> vertIdToPtMap;

	for (const auto& faceId : faceIds) {
		auto pBlk = getBlockPtr(faceId);
		pBlk->faceRealFunc(faceId, [&pBlk, &vertIdToPtMap, &pts](const Polygon& face) {
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
		pBlk->faceRealFunc(faceId, [&out, &vertIdToPtMap](const Polygon& face) {
			out << "#id: " << face.getId() << "\n";
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
				blkVerts.iterateInOrder([&numPoints](const Index3DId& id, const Vertex& vert) {
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
				blkVerts.iterateInOrder([&numPoints, &out](const Index3DId& id, const Vertex& vert) {
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
				const auto& blkPolygons = pBlk->_modelData._polygons;
				blkPolygons.iterateInOrder([&startIndices](const Index3DId& id, const Polygon& face) {
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

bool Volume::verifyTopology(bool multiCore) const
{
	bool result = true;

	runLambda([this, &result](size_t linearIdx)->bool {
		if (_blocks[linearIdx])
			if (!_blocks[linearIdx]->verifyTopology()) {
				result = false;
				exit(0);
			}
		return true;
	}, multiCore);
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
	const unsigned int stride = 3; // Stride = 3 creates a super block 3x3x3 across. Each thread has exclusive access to the super block
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

				sort(blocksToProcess.begin(), blocksToProcess.end());
				// Process those blocks in undetermined order
				MultiCore::runLambda([fLambda](size_t linearIdx)->bool {
					return fLambda(linearIdx);
				}, blocksToProcess, multiCore);

			}
		}
	}

	endOperation();
}
