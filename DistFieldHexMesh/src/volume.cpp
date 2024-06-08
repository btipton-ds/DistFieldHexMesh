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
#include <filesystem>
#include <stdexcept>

#include <defines.h>
#include <cmath>

#include <tm_ioUtil.h>
#include <tm_math.h>
#include <tm_plane.h>
#include <tm_ray.h>
#include <tm_bestFit.h>
#include <tm_plane.h>
#include <triMesh.h>

#include <block.h>
#include <volume.h>
#include <splitParams.h>
#include <vertex.h>
#include <polyhedronSplitter.h>
#include <vertexSpatialTree.h>
#include <tolerances.h>
#include <utils.h>

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
	, _threadPool(MultiCore::getNumCores())
{
}

Volume::~Volume()
{
	clear();
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

	findSharpVertices(_pModelTriMesh, SHARP_EDGE_ANGLE_RADIANS, _sharpVertIndices);
	vector<Vector3d> sharpPoints;
	for (size_t vIdx : _sharpVertIndices) {
		const auto& pt = _pModelTriMesh->getVert(vIdx)._pt;
		sharpPoints.push_back(pt);
	}

	double err;
	_hasSharpVertPlane = bestFitPlane(sharpPoints, _sharpVertPlane, err) && err < Tolerance::sameDistTol();
	findSharpEdgeEdges();
}

void Volume::findSharpEdgeEdges()
{
	const double sinSharpAngle = sin(getSharpAngleRad());
	for (size_t edgeIdx = 0; edgeIdx < _pModelTriMesh->numEdges(); edgeIdx++) {
		const auto& edge = _pModelTriMesh->getEdge(edgeIdx);
		if (edge._numFaces == 2 && _pModelTriMesh->isEdgeSharp(edgeIdx, sinSharpAngle)) {
			_sharpEdgeIndices.insert(edgeIdx);
		}
	}
}

void Volume::findSharpVertices(const TriMesh::CMeshPtr& pMesh, double sharpAngleRadians, std::vector<size_t>& vertIndices)
{
	const double sinSharpAngle = sin(sharpAngleRadians);

	size_t numVerts = pMesh->numVertices();
	for (size_t vIdx = 0; vIdx < numVerts; vIdx++) {
		const auto& vert = pMesh->getVert(vIdx);
		double maxDp = 1;
		const auto& edgeIndices = vert._edgeIndices;
		vector<Vector3d> radiantVectors;
		for (size_t edgeIdx : edgeIndices) {
			if (pMesh->isEdgeSharp(edgeIdx, sinSharpAngle)) {
				auto edge0 = pMesh->getEdge(edgeIdx);
				size_t opIdx0 = edge0._vertIndex[0] == vIdx ? edge0._vertIndex[1] : edge0._vertIndex[0];
				const auto& vert0 = pMesh->getVert(opIdx0);
				Vector3d v0 = (vert0._pt - vert._pt).normalized();
				radiantVectors.push_back(v0);
			}
		}

		if (radiantVectors.size() < 2)
			continue;

		bool hasSmoothEdgePair = false;
		for (size_t i = 0; i < radiantVectors.size(); i++) {
			const auto& v0 = radiantVectors[i];
			for (size_t j = i + 1; j < radiantVectors.size(); j++) {
				const auto& v1 = -radiantVectors[j];
				double cp = v1.cross(v0).norm();
				if (cp < sinSharpAngle) {
					hasSmoothEdgePair = true;
					break;
				}
			}
		}

		if (!hasSmoothEdgePair) {
			vertIndices.push_back(vIdx);
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

	makeFaceTris(triMeshes, Index3D(0, 0, 0), volDim(), true);
}

void Volume::clear()
{
	// Clear contents to remove cross block pointers
	runThreadPool333([this](size_t threadNum, size_t linearIdx, const std::shared_ptr<Block>& pBlk)->bool {
		if (pBlk)
			pBlk->clear();
		return true;
	}, RUN_MULTI_THREAD);

	// Now it's safe to destroy the heaps
	_blocks.clear();
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
	_boundingBox = pTriMesh->getBBox();
	_boundingBox.growPercent(0.0125);
	_originMeters = _boundingBox.getMin();
	_spanMeters = _boundingBox.range();

	Index3D blockSize;
	if (params.uniformRatio) {
		blockSize = Index3D(
			params.minBlocksPerSide,
			params.minBlocksPerSide,
			params.minBlocksPerSide
		);
	} else {
		double minSpan = DBL_MAX;
		for (int i = 0; i < 3; i++) {
			if (_spanMeters[i] < minSpan) {
				minSpan = _spanMeters[i];
			}
		}
		double targetBlockSize = minSpan / params.minBlocksPerSide;
		size_t blockDim = 1;
		Index3D::setBlockDim(blockDim);

		blockSize = Index3D (
			(size_t)(_spanMeters[0] / targetBlockSize + 0.5),
			(size_t)(_spanMeters[1] / targetBlockSize + 0.5),
			(size_t)(_spanMeters[2] / targetBlockSize + 0.5)
		);
	}

	for (size_t i = 0; i < params.numBlockDivs; i++) {
		for (int i = 0; i < 3; i++)
			blockSize[i] *= 2;
	}
	setVolDim(blockSize);

	const auto& dim = volDim();
	Vector3d blockSpan(_spanMeters[0] / dim[0], _spanMeters[1] / dim[1], _spanMeters[2] / dim[2]);

	size_t numBlocks = dim[0] * dim[1] * dim[2];
	_blocks.resize(numBlocks);

	double sharpAngleRadians = params.sharpAngle_degrees / 180.0 * M_PI;
	double sinEdgeAngle = sin(sharpAngleRadians);

	std::vector<size_t> sharpEdges;
	{
		Utils::Timer tmr0(Utils::Timer::TT_analyzeModelMesh);
		_pModelTriMesh->buildCentroids(true);
		_pModelTriMesh->buildNormals(true);
		_pModelTriMesh->calCurvatures(sharpAngleRadians, true);
		sharpEdges = _pModelTriMesh->getSharpEdgeIndices(sharpAngleRadians);
		findFeatures();
	}

	{
		Utils::Timer tmr0(Utils::Timer::TT_buildCFDHexMesh);

		createBlocks(params, blockSpan, multiCore);
		divideSimple(params, multiCore);
		divideConitional(params, multiCore);

// TODO we should be able to clear the reference topology now

//		splitAtSharpVerts(params, multiCore);
//		splitAtSharpEdges(params, multiCore);
//		cutWithTriMesh(params, multiCore);
	}

	Utils::Timer::dumpAll();
	assert(verifyTopology(multiCore));

	cout << "Num polyhedra: " << numPolyhedra() << "\n";
	cout << "Num faces. All: " << numFaces(true) << ", outer: " << numFaces(false) << "\n";
}

void Volume::createBlocks(const BuildCFDParams& params, const Vector3d& blockSpan, bool multiCore)
{
	runThreadPool([this, &blockSpan](size_t threadNum, size_t linearIdx, const std::shared_ptr<Block>& pBlk)->bool {
		auto blockIdx = calBlockIndexFromLinearIndex(linearIdx);
		_blocks[linearIdx] = createBlock(blockIdx);
		return true;
	}, multiCore);

	// Cannot create subBlocks until all blocks are created so they can be connected
	runThreadPool333([this, &blockSpan](size_t threadNum, size_t linearIdx, const std::shared_ptr<Block>& pBlk)->bool {
		pBlk->createBlockCells(TS_REAL);
		return true;
	}, multiCore);
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

void Volume::divideSimple(const BuildCFDParams& params, bool multiCore)
{
	if (params.numSimpleDivs > 0) {

		for (size_t i = 0; i < params.numSimpleDivs; i++) {
			runThreadPool333([this](size_t threadNum, size_t linearIdx, const std::shared_ptr<Block>& pBlk)->bool {
				pBlk->iteratePolyhedraInOrder(TS_REAL, [](const auto& cellId, Polyhedron& cell) {
					cell.setNeedsDivideAtCentroid();
				});
				return true;
			}, multiCore);

			finishSplits(multiCore);
		}

//		assert(verifyTopology(multiCore));
	}
}

void Volume::divideConitional(const BuildCFDParams& params, bool multiCore)
{
	size_t numPasses = params.numConditionalPasses();
	if (numPasses > 0) {
		double sharpAngleRadians = params.sharpAngle_degrees / 180.0 * M_PI;
		double sinEdgeAngle = sin(sharpAngleRadians);

		for (size_t passNum = 0; passNum < numPasses; passNum++) {
			if (passNum > 0) {
				bool didSplit = true;
				int count = 0;
				while (didSplit && count < 3) {
					didSplit = false;
					runThreadPool333([this, &params, &didSplit, sinEdgeAngle](size_t threadNum, size_t linearIdx, const std::shared_ptr<Block>& pBlk)->bool {
						if (pBlk->doPresplits(params))
							didSplit = true;
						return true;
					}, multiCore);
					count++;
				}
			}

			bool changed = false;
			runThreadPool333([this, passNum, &params, sinEdgeAngle, &changed](size_t threadNum, size_t linearIdx, const std::shared_ptr<Block>& pBlk)->bool {
				pBlk->iteratePolyhedraInOrder(TS_REAL, [&changed, passNum, &params](const Index3DId& cellId, Polyhedron& cell) {
					if (cell.setNeedToSplitConditional(passNum, params))
						changed = true;
				});
				return true;
			}, multiCore);

			if (changed)
				finishSplits(multiCore);
			//		assert(verifyTopology(multiCore));

			if (!changed) {
				cout << "No more splits required: " << passNum << "\n";
				break;
			}
		}
	}
}

void Volume::splitAtSharpVerts(const BuildCFDParams& params, bool multiCore)
{
	if (params.splitAtSharpVerts) {
		// Step one, assure that all cells which will be split due to cusp vertices on sharp edges have clean faces, no partial splits
		bool changed = false;
		runThreadPool333([this, &changed, &params](size_t threadNum, size_t linearIdx, const std::shared_ptr<Block>& pBlk)->bool {
			pBlk->iteratePolyhedraInOrder(TS_REAL, [&pBlk, &changed, &params](const Index3DId& cellId, Polyhedron& cell) {
				PolyhedronSplitter sp(pBlk.get(), cellId);
				if (sp.cutAtSharpVerts(params)) {
					changed = true;
				}
			});
			return true;
		}, false && multiCore);

		if (changed)
			finishSplits(multiCore);
	}
}

void Volume::splitAtSharpEdges(const BuildCFDParams& params, bool multiCore)
{
	// Step one, assure that all cells which will be split due to cusp vertices on sharp edges have clean faces, no partial splits
	bool changed = false;
	runThreadPool333([this, &changed, &params](size_t threadNum, size_t linearIdx, const std::shared_ptr<Block>& pBlk)->bool {
		pBlk->iteratePolyhedraInOrder(TS_REAL, [&pBlk, &changed, &params](const Index3DId& cellId, Polyhedron& cell) {
			if (cell.containsSharps()) {
				changed = cell.setNeedsCleanFaces();
			}
		});
		return true;
	}, multiCore);

	if (changed)
		finishSplits(multiCore);

	changed = false;
	runThreadPool333([this, &changed, &params](size_t threadNum, size_t linearIdx, const std::shared_ptr<Block>& pBlk)->bool {
		pBlk->iteratePolyhedraInOrder(TS_REAL, [&pBlk, &changed, &params](const Index3DId& cellId, Polyhedron& cell) {
			PolyhedronSplitter sp(pBlk.get(), cellId);
			if (sp.cutAtSharpEdges(params))
				changed = true;
			});
		return true;
	}, multiCore);

	if (changed)
		finishSplits(multiCore);

#if 0
	// Step 2, split cells which contain sharp edges with cusps (sharp verts)
	if (changed) {
		changed = false;
		runLambda([this, &changed, &params](size_t linearIdx)->bool {
			auto pBlk = _blocks[linearIdx];
			pBlk->iteratePolyhedraInOrder(TS_REAL, [&pBlk, &changed, &params](const Index3DId& cellId, Polyhedron& cell) {
				PolyhedronSplitter sp(pBlk.get(), cellId);
				if (sp.splitAtSharpEdgeCusps(params))
					changed = true;
				});
			return true;
		}, multiCore);
	}
	changed = false;
	runLambda([this, &changed, &params](size_t linearIdx)->bool {
		auto pBlk = _blocks[linearIdx];
		pBlk->iteratePolyhedraInOrder(TS_REAL, [&pBlk, &changed, &params](const Index3DId& cellId, Polyhedron& cell) {
			PolyhedronSplitter sp(pBlk.get(), cellId);
			if (sp.splitAtSharpEdges(params))
				changed = true;
			});
		return true;
	}, multiCore);
#endif
}

void Volume::cutWithTriMesh(const BuildCFDParams& params, bool multiCore)
{
	bool changed = false;
	runThreadPool333([this, &changed, &params](size_t threadNum, size_t linearIdx, const std::shared_ptr<Block>& pBlk)->bool {
		pBlk->iteratePolyhedraInOrder(TS_REAL, [&pBlk, &changed, &params](const Index3DId& cellId, Polyhedron& cell) {
			if (cell.containsSharps()) {
				changed = cell.setNeedsCleanFaces();
			}
		});
		return true;
	}, multiCore);

	changed = false;
	runThreadPool333([this, &changed, &params](size_t threadNum, size_t linearIdx, const std::shared_ptr<Block>& pBlk)->bool {
		pBlk->iteratePolyhedraInOrder(TS_REAL, [&pBlk, &changed, &params](const Index3DId& cellId, Polyhedron& cell) {
			if (cell.intersectsModel()) {
				PolyhedronSplitter ps(pBlk.get(), cellId);
				if (ps.cutWithModelMesh(params))
					changed = true;
			}
		});
		return true;
	}, multiCore);

	if (changed)
		finishSplits(multiCore);
}

void Volume::finishSplits(bool multiCore)
{
	bool done = false;
	int i = 0;
	while (!done) {
		done = true;
		runThreadPool333([this, &done](size_t threadNum, size_t linearIdx, const std::shared_ptr<Block>& pBlk)->bool {
			pBlk->splitRequiredPolyhedra();
			return true;
		}, multiCore);

		runThreadPool333([this, &done](size_t threadNum, size_t linearIdx, const std::shared_ptr<Block>& pBlk)->bool {
			pBlk->updateSplitStack();
			return true;
		}, multiCore);

		runThreadPool333([this, &done](size_t threadNum, size_t linearIdx, const std::shared_ptr<Block>& pBlk)->bool {
			if (pBlk->hasPendingSplits()) {
				done = false;
				return false; // We need to split 1, so we need to split all. Exit early
			}
			return true;
		}, multiCore);

		i++;
		if (i > 20)
			break;
	}
	imprintTJointVertices(multiCore);
	cout << "FinishSplits " << i << "\n";
}

void Volume::imprintTJointVertices(bool multiCore)
{
	runThreadPool333([this](size_t threadNum, size_t linearIdx, const std::shared_ptr<Block>& pBlk)->bool {
		pBlk->imprintTJointVertices();
		return true;
	}, multiCore);
#ifdef _DEBUG
	bool allCellsClosed = true;
	runThreadPool([this, &allCellsClosed](size_t threadNum, size_t linearIdx, const std::shared_ptr<Block>& pBlk)->bool {
		if (!pBlk->allCellsClosed()) {
			allCellsClosed = false;
			return false;
		}
		return true;
	}, multiCore);
	assert(allCellsClosed);
#endif
}

void Volume::dumpOpenCells(bool multiCore) const
{
#if DUMP_OPEN_CELL_OBJS
	runThreadPool([this](size_t threadNum, size_t linearIdx, const std::shared_ptr<Block>& pBlk)->bool {
		pBlk->dumpOpenCells();
		return true;
	}, multiCore);
#endif
}

void Volume::makeFaceTriMesh(FaceType faceType, Block::TriMeshGroup& triMeshes, const shared_ptr<Block>& pBlock, size_t threadNum) const
{
	CBoundingBox3Dd bbox = _boundingBox;
	bbox.merge(pBlock->_boundBox);
	
	CMeshPtr pMesh = triMeshes[faceType][threadNum];
	if (!pMesh) {
		pMesh = make_shared<CMesh>(bbox);
		pMesh->setEnforceManifold(false); // Block meshes are none manifold
		triMeshes[faceType][threadNum] = pMesh;
	}
	pBlock->getBlockTriMesh(faceType, pMesh);
}

void Volume::makeFaceTris(Block::TriMeshGroup& triMeshes, const Index3D& min, const Index3D& max, bool multiCore) const
{

	size_t numThreads = MultiCore::getNumCores();
	triMeshes.resize(4);
	triMeshes[FT_OUTER].resize(numThreads);
	triMeshes[FT_MODEL_BOUNDARY].resize(numThreads);
	triMeshes[FT_BLOCK_BOUNDARY].resize(numThreads);
	triMeshes[FT_ALL].resize(numThreads);
	runThreadPool([this, &triMeshes, &min, &max](size_t threadNum, size_t linearIdx, const std::shared_ptr<Block>& blockPtr)->bool {
		if (blockPtr) {
			MultiCore::scoped_set_local_heap st(blockPtr->getHeapPtr());
			Index3D blkIdx = blockPtr->getBlockIdx();
			if (blkIdx[0] >= min[0] && blkIdx[1] >= min[1] && blkIdx[2] >= min[2] &&
				blkIdx[0] <= max[0] && blkIdx[1] <= max[1] && blkIdx[2] <= max[2]) {
					if (blockPtr->numFaces(true) > 0) {
						makeFaceTriMesh(FT_OUTER, triMeshes, blockPtr, threadNum);
						makeFaceTriMesh(FT_MODEL_BOUNDARY, triMeshes, blockPtr, threadNum);
						makeFaceTriMesh(FT_BLOCK_BOUNDARY, triMeshes, blockPtr, threadNum);
						makeFaceTriMesh(FT_ALL, triMeshes, blockPtr, threadNum);
					}
				}
			}
		return true;
	}, multiCore);

	for (size_t i = 0; i < numThreads; i++) {
		if (triMeshes[FT_OUTER][i])
			triMeshes[FT_OUTER][i]->changed();

		if (triMeshes[FT_MODEL_BOUNDARY][i])
			triMeshes[FT_MODEL_BOUNDARY][i]->changed();

		if (triMeshes[FT_BLOCK_BOUNDARY][i])
			triMeshes[FT_BLOCK_BOUNDARY][i]->changed();

		if (triMeshes[FT_ALL][i])
			triMeshes[FT_ALL][i]->changed();
	}
}

void Volume::makeFaceEdges(FaceType faceType, Block::glPointsGroup& faceEdges, const shared_ptr<Block>& pBlock, size_t threadNum) const
{
	CBoundingBox3Dd bbox = _boundingBox;
	bbox.merge(pBlock->_boundBox);

	Block::glPointsPtr pPoints = faceEdges[faceType][threadNum];
	if (!pPoints) {
		pPoints = make_shared<Block::GlPoints>();
		faceEdges[faceType][threadNum] = pPoints;
	}
	pBlock->makeEdgeSets(faceType, pPoints);
}

void Volume::makeEdgeSets(Block::glPointsGroup& faceEdges, const Index3D& min, const Index3D& max, bool multiCore) const
{
	size_t numThreads = MultiCore::getNumCores();

	faceEdges.resize(4);
	faceEdges[FT_OUTER].resize(numThreads);
	faceEdges[FT_MODEL_BOUNDARY].resize(numThreads);
	faceEdges[FT_BLOCK_BOUNDARY].resize(numThreads);
	faceEdges[FT_ALL].resize(numThreads);

	runThreadPool([this, &faceEdges, &min, &max](size_t threadNum, size_t linearIdx, const std::shared_ptr<Block>& blockPtr)->bool {
		if (blockPtr) {
			MultiCore::scoped_set_local_heap st(blockPtr->getHeapPtr());
			Index3D blkIdx = blockPtr->getBlockIdx();
			if (blkIdx[0] >= min[0] && blkIdx[1] >= min[1] && blkIdx[2] >= min[2] &&
				blkIdx[0] <= max[0] && blkIdx[1] <= max[1] && blkIdx[2] <= max[2]) {
				if (blockPtr->numFaces(true) > 0) {
					makeFaceEdges(FT_OUTER, faceEdges, blockPtr, threadNum);
					makeFaceEdges(FT_MODEL_BOUNDARY, faceEdges, blockPtr, threadNum);
					makeFaceEdges(FT_BLOCK_BOUNDARY, faceEdges, blockPtr, threadNum);
					makeFaceEdges(FT_ALL, faceEdges, blockPtr, threadNum);
				}
			}
		}
		return true;
	}, multiCore);

	for (size_t i = 0; i < numThreads; i++) {
		if (faceEdges[FT_OUTER][i])
			faceEdges[FT_OUTER][i]->changed();

		if (faceEdges[FT_MODEL_BOUNDARY][i])
			faceEdges[FT_MODEL_BOUNDARY][i]->changed();

		if (faceEdges[FT_BLOCK_BOUNDARY][i])
			faceEdges[FT_BLOCK_BOUNDARY][i]->changed();

		if (faceEdges[FT_ALL][i])
			faceEdges[FT_ALL][i]->changed();
	}
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

void Volume::writeObj(const string& path, const vector<Index3DId>& cellIds, bool includeModel, bool useEdges, bool sharpOnly, const std::vector<Vector3d>& pts) const
{
	ofstream out(path, ios_base::trunc);
	writeObj(out, cellIds, includeModel, useEdges, sharpOnly, pts);
}

void Volume::writeObj(ostream& out, const vector<Index3DId>& cellIds, bool includeModel, bool useEdges, bool sharpOnly, const std::vector<Vector3d>& extraPoints) const
{
	set<Index3DId> faceIds;
	vector<size_t> modelTriIndices;
	for (const auto& cellId : cellIds) {
		auto pBlk = getBlockPtr(cellId);
		pBlk->cellFunc(TS_REAL,cellId, [&faceIds, &modelTriIndices, includeModel, useEdges, sharpOnly](const Polyhedron& cell) {
			const auto& ids = cell.getFaceIds();
			faceIds.insert(ids.begin(), ids.end());
			if (includeModel) {
				modelTriIndices = cell.getTriIndices();
			}
		});
	}

	vector<Vector3d> pts;
	set<TriMesh::CEdge> modelEdgeSet;
	VertSearchTree_size_t_8 pointToIdxMap(_boundingBox);

	if (!modelTriIndices.empty()) {
		const double sinSharp = sin(SHARP_EDGE_ANGLE_RADIANS);
		for (auto triIdx : modelTriIndices) {
			const auto& tri = _pModelTriMesh->getTri(triIdx);
			if (useEdges) {
				for (int i = 0; i < 3; i++) {
					int j = (i + 1) % 3;
					TriMesh::CEdge edge(tri[i], tri[j]);
					if (!modelEdgeSet.contains(edge)) {
						size_t edgeIdx = _pModelTriMesh->findEdge(edge);
						if (!sharpOnly || _pModelTriMesh->isEdgeSharp(edgeIdx, sinSharp)) {
							modelEdgeSet.insert(edge);
							for (int k = 0; k < 2; k++) {
								size_t modVertIdx = edge._vertIndex[k];
								const auto& pt = _pModelTriMesh->getVert(modVertIdx)._pt;
								size_t vertIdx;
								if (!pointToIdxMap.find(pt, vertIdx)) {
									vertIdx = pts.size();
									pts.push_back(pt);
									pointToIdxMap.add(pt, vertIdx);
								}
							}
						}
					}
				}
			} else {
				for (int i = 0; i < 3; i++) {
					size_t modVertIdx = tri[i];
					const auto& pt = _pModelTriMesh->getVert(modVertIdx)._pt;
					size_t vertIdx;
					if (!pointToIdxMap.find(pt, vertIdx)) {
						vertIdx = pts.size();
						pts.push_back(pt);
						pointToIdxMap.add(pt, vertIdx);
					}
				}
			}
		}
	}

	for (const auto& faceId : faceIds) {
		auto pBlk = getBlockPtr(faceId);
		pBlk->faceFunc(TS_REAL, faceId, [&pBlk, &pointToIdxMap, &pts](const Polygon& face) {
			const auto& vIds = face.getVertexIds();
			for (const auto& vertId : vIds) {
				Vector3d pt = pBlk->getVertexPoint(vertId);
				size_t vertIdx;
				if (!pointToIdxMap.find(pt, vertIdx)) {
					vertIdx = pts.size();
					pts.push_back(pt);
					pointToIdxMap.add(pt, vertIdx);
				}
			}
		});
	}

	out << "#Vertices " << pts.size() << "\n";
	for (const auto& pt : pts) {
		out << "v " << pt[0] << " " << pt[1] << " " << pt[2] << "\n";
	}

	for (const auto& pt : extraPoints) {
		out << "v " << pt[0] << " " << pt[1] << " " << pt[2] << "\n";
	}

	out << "#Faces " << faceIds.size() << "\n";
	for (const auto& faceId : faceIds) {
		auto pBlk = getBlockPtr(faceId);
		pBlk->faceFunc(TS_REAL, faceId, [&out, &pBlk, &pointToIdxMap](const Polygon& face) {
			out << "#id: " << face.getId() << "\n";
			out << "#NumVerts: " << face.getVertexIds().size() << "\n";
			out << "f ";
			const auto& vIds = face.getVertexIds();
			for (const auto& vertId : vIds) {
				Vector3d pt = pBlk->getVertexPoint(vertId);
				size_t vertIdx;
				if (!pointToIdxMap.find(pt, vertIdx))
					out << (vertIdx + 1) << " ";
			}
			out << "\n";
		});
	}

	if (includeModel) {
		if (useEdges) {
			out << "#Model Edges " << modelEdgeSet.size() << "\n";
			for (const auto& ed : modelEdgeSet) {
				out << "l ";
				for (int i = 0; i < 2; i++) {
					size_t modVertIdx = ed._vertIndex[i];
					Vector3d pt = _pModelTriMesh->getVert(modVertIdx)._pt;
					size_t vertIdx;
					if (!pointToIdxMap.find(pt, vertIdx))
						out << (vertIdx + 1) << " ";
				}
				out << "\n";
			}
		} else {
			out << "#Model Faces " << faceIds.size() << "\n";
			for (auto triIdx : modelTriIndices) {
				const auto& tri = _pModelTriMesh->getTri(triIdx);
				out << "f ";
				for (int i = 0; i < 3; i++) {
					size_t modVertIdx = tri[i];
					Vector3d pt = _pModelTriMesh->getVert(modVertIdx)._pt;
					size_t vertIdx;
					if (!pointToIdxMap.find(pt, vertIdx))
						out << (vertIdx + 1) << " ";
				}
				out << "\n";
			}
		}
	}
}

bool Volume::write(ostream& out) const
{
	uint8_t version = 0;
	out.write((char*)&version, sizeof(version));

	out.write((char*)&_sharpAngleRad, sizeof(_sharpAngleRad));
	s_volDim.write(out);
	writeVector3(out, _originMeters);
	writeVector3(out, _spanMeters);

	_boundingBox.write(out);

	IoUtil::writeVector3(out, _cornerPts);

	size_t num = _blocks.size();
	out.write((char*)&num, sizeof(num));
	for (const auto& pBlock : _blocks) {
		bool isNull = pBlock == nullptr;
		out.write((char*)&isNull, sizeof(isNull));
		if (pBlock)
			pBlock->write(out);
	}

	return true;
}

bool Volume::read(istream& in)
{
	clear();

	uint8_t version = -1;
	in.read((char*)&version, sizeof(version));

	in.read((char*)&_sharpAngleRad, sizeof(_sharpAngleRad));
	s_volDim.read(in);
	readVector3(in, _originMeters);
	readVector3(in, _spanMeters);

	_boundingBox.read(in);

	IoUtil::readVector3(in, _cornerPts);
	size_t num;
	in.read((char*)&num, sizeof(num));
	if (num > 0) {
		_blocks.resize(num);
		for (size_t i = 0; i < _blocks.size(); i++) {
			bool isNull;
			in.read((char*)&isNull, sizeof(isNull));
			if (isNull)
				continue;

			Index3D blockIdx = calBlockIndexFromLinearIndex(i);
			auto pBlock = createBlock(blockIdx);
			_blocks[i] = pBlock;
			pBlock->_pVol = this;
			pBlock->read(in);
		}
	}
	return true;
}

const Vertex& Volume::getVertex(const Index3DId& id) const
{
	const auto& pBlk = getBlockPtr(id);
	assert(pBlk);
	return pBlk->_vertices[id];
}

const DFHM::Polygon& Volume::getPolygon(const Index3DId& id) const
{
	const auto& pBlk = getBlockPtr(id);
	assert(pBlk);
	return pBlk->_modelData._polygons[id];
}

const Polyhedron& Volume::getPolyhedron(const Index3DId& id) const
{
	const auto& pBlk = getBlockPtr(id);
	assert(pBlk);
	return pBlk->_modelData._polyhedra[id];
}

void Volume::writePolyMesh(const string& dirNameIn)
{
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

	PolymeshTables tables;
	createPolymeshTables(tables);

	writePolyMeshPoints(dirName, tables);
	writePolyMeshFaces(dirName, tables);
	writePolyMeshOwnerCells(dirName, tables);
	writePolyMeshNeighborCells(dirName, tables);
	writePolyMeshBoundaries(dirName, tables);
}

void Volume::createPolymeshTables(PolymeshTables& tables)
{

	for (auto pBlk : _blocks) {
		if (pBlk) {
			pBlk->_vertices.iterateInOrder([&tables](const Index3DId& id, const Vertex& vert) {
				int vIdx = (int)tables.vertIdxIdMap.size();
				tables.vertIdxIdMap.push_back(id);
				tables.vertIdIdxMap.insert(make_pair(id, vIdx));
			});
		}
	}

	// Cell order determines the order of shared faces. Create it first.
	for (auto pBlk : _blocks) {
		if (pBlk) {
			pBlk->_modelData._polyhedra.iterateInOrder([&tables](const Index3DId& id, const Polyhedron& cell) {
				int cIdx = (int)tables.cellIdxIdMap.size();
				tables.cellIdxIdMap.push_back(id);
				tables.cellIdIdxMap.insert(make_pair(id, cIdx));
			});
		}
	}

	Vector3d xAxis(1, 0, 0), yAxis(0, 1, 0), zAxis(0, 0, 1);

	Planed planes[] = {
		Planed(_boundingBox.getMin(), xAxis, false), // frontPlane
		Planed(_boundingBox.getMin(), yAxis, false), // leftPlane
		Planed(_boundingBox.getMin(), zAxis, false), // bottomPlane
		Planed(_boundingBox.getMax(), xAxis, false), // backPlane
		Planed(_boundingBox.getMax(), yAxis, false), // rightPlane
		Planed(_boundingBox.getMax(), zAxis, false), // topPlane
	};

	vector<Index3DId> outerBounds[6], wall;
	for (auto pBlk : _blocks) {
		if (pBlk) {
			pBlk->_modelData._polygons.iterateInOrder([&](const Index3DId& id, const Polygon& face) {
				if (face.numCells() == 1) {
					bool onOuterBoundary = false;
					for (int i = 0; i < 6; i++) {
						if (face.isCoplanar(planes[i])) {
							onOuterBoundary = true;
							outerBounds[i].push_back(id);
							break;
						}
					}
					if (!onOuterBoundary) {
						wall.push_back(id);
					}
				}
			});
		}
	}

	// sort shared faces
	for (size_t cellIdx = 0; cellIdx < tables.cellIdxIdMap.size(); cellIdx++) {
		const auto& cellId = tables.cellIdxIdMap[cellIdx];
		const auto& cell = getPolyhedron(cellId);
		const auto& faceIds = cell.getFaceIds();
		vector<Index3DId> faceIdsOwnedByThisCell;
		for (const auto& faceId : faceIds) {
			const auto& face = getPolygon(faceId);
			if (face.numCells() == 2) {
				int ownerIdx = getFaceOwnerIdx(faceId, tables);
				if (ownerIdx == cellIdx) {
					if (tables.faceIdIdxMap.find(faceId) == tables.faceIdIdxMap.end()) {
						faceIdsOwnedByThisCell.push_back(faceId);
					}
				}
			}
		}

		sort(faceIdsOwnedByThisCell.begin(), faceIdsOwnedByThisCell.end(), [this, &tables](const Index3DId& lhsId, const Index3DId& rhsId) {
			int lhsIdx = getFaceNeighbourIdx(lhsId, tables);
			int rhsIdx = getFaceNeighbourIdx(rhsId, tables);
			return lhsIdx < rhsIdx;
		});

		for (const auto& id : faceIdsOwnedByThisCell) {
			int fIdx = (int)tables.faceIdxIdMap.size();
			tables.faceIdxIdMap.push_back(id);
			tables.faceIdIdxMap.insert(make_pair(id, fIdx));
		}
	}

	tables.numInner = (int)tables.faceIdxIdMap.size();

	for (int i = 0; i < 6; i++) {
		tables.boundaryIndices[i] = (int)tables.faceIdxIdMap.size();
		for (const auto& id : outerBounds[i]) {
			int fIdx = (int)tables.faceIdxIdMap.size();
			tables.faceIdxIdMap.push_back(id);
			tables.faceIdIdxMap.insert(make_pair(id, fIdx));
		}
	}

	tables.boundaryIdx = (int)tables.faceIdxIdMap.size();
	for (const auto& id : wall) {
		int fIdx = (int)tables.faceIdxIdMap.size();
		tables.faceIdxIdMap.push_back(id);
		tables.faceIdIdxMap.insert(make_pair(id, fIdx));
	}

}

int Volume::getFaceNeighbourIdx(const Index3DId& faceId, const PolymeshTables& tables) const
{
	const auto& face = getPolygon(faceId);
	const auto& cellIds = face.getCellIds();
	assert(cellIds.size() == 2);
	int maxCellIdx = -1;
	for (const auto& cellId : cellIds) {
		int cellIdx = tables.cellIdIdxMap.find(cellId)->second;
		if (cellIdx > maxCellIdx)
			maxCellIdx = cellIdx;
	}
	return maxCellIdx;
}

int Volume::getFaceOwnerIdx(const Index3DId& faceId, const PolymeshTables& tables) const
{
	const auto& face = getPolygon(faceId);
	const auto& cellIds = face.getCellIds();
	assert(cellIds.size() == 2);
	int minCellIdx = INT_MAX;
	for (const auto& cellId : cellIds) {
		int cellIdx = tables.cellIdIdxMap.find(cellId)->second;
		if (cellIdx < minCellIdx)
			minCellIdx = cellIdx;
	}
	return minCellIdx;
}

void Volume::writePolyMeshPoints(const string& dirName, const PolymeshTables& tables) const
{
	string filename = dirName + "/points";
	FILE* fOut = fopen(filename.c_str(), "wb");
	try {
		writeFOAMHeader(fOut, "binary", "vectorField", "points");

		fprintf(fOut, "%llu\n", tables.vertIdxIdMap.size());
		fprintf(fOut, "(");

		vector<double> vals;
		vals.resize(3 * tables.vertIdxIdMap.size());
		size_t idx = 0;
		for (const auto& vertId : tables.vertIdxIdMap) {
			const auto& vert = getVertex(vertId);
			auto pt = vert.getPoint();

			vals[idx++] = pt[0];
			vals[idx++] = pt[1];
			vals[idx++] = pt[2];
		}
		fwrite(vals.data(), sizeof(double), vals.size(), fOut);
		fprintf(fOut, ")\n");
		fprintf(fOut, "//**********************************************************************************//\n");
	} catch (...) {
	}
	fclose(fOut);
}

bool Volume::needToReverseNormal(const Polygon& face, const PolymeshTables& tables) const
{
	bool result = false;
	Vector3d n = face.calUnitNormal();

	Vector3d faceCtr = face.calCentroid();
	Vector3d cellCtr;
	const auto& cellIds = face.getCellIds();
	if (cellIds.size() == 1) {
		const Index3DId cellId = *cellIds.begin();
		const auto& cell = getPolyhedron(cellId);
		cellCtr = cell.calCentroid();
		Vector3d v = cellCtr - faceCtr;
		if (v.dot(n) > 0)
			result = true;
	}
	else {
		assert(cellIds.size() == 2);
		int maxCellIdx = -1;
		for (const auto& cellId : cellIds) {
			int cellIdx = tables.cellIdIdxMap.find(cellId)->second;
			if (maxCellIdx < cellIdx) {
				maxCellIdx = cellIdx;
			}
		}
		const auto& cell = getPolyhedron(tables.cellIdxIdMap[maxCellIdx]);
		cellCtr = cell.calCentroid();

		Vector3d v = cellCtr - faceCtr;
		if (v.dot(n) < 0)
			result = true;
	}

	return result;
}

void Volume::writePolyMeshFaces(const string& dirName, const PolymeshTables& tables) const
{
	string filename = dirName + "/faces";
	FILE* fOut = fopen(filename.c_str(), "wb");
	try {
		writeFOAMHeader(fOut, "binary", "faceCompactList", "faces");

		// Write poly index table
		// An extra entry is required to compute the number of vertices in the last face
		// So, this is actually #faces + 1, not #faces.
		int idx = 0;
		vector<int> faceIndices;
		faceIndices.resize(tables.faceIdxIdMap.size() + 1);
		for (size_t i = 0; i < tables.faceIdxIdMap.size(); i++) {
			const auto& face = getPolygon(tables.faceIdxIdMap[i]);
			faceIndices[i] = idx;
			idx += (int)face.getVertexIds().size();
		}
		faceIndices[faceIndices.size() - 1] = idx; // Append the index of the start of the next face, even though that face doesn't exist.

		fprintf(fOut, "%llu\n", faceIndices.size());
		fprintf(fOut, "(");

		fwrite(faceIndices.data(), sizeof(int), faceIndices.size(), fOut);
		fprintf(fOut, ")\n");
		fprintf(fOut, "\n");

		// Write the face vertex index list
		vector<int> vertIndices;
		vertIndices.reserve(tables.faceIdxIdMap.size() * 4);
		for (const auto& faceId : tables.faceIdxIdMap) {
			const auto& face = getPolygon(faceId);

			auto verts = face.getVertexIds();
			if (needToReverseNormal(face, tables)) {
				reverse(verts.begin(), verts.end());
			}
			for (const auto& vId : verts) {
				vertIndices.push_back(tables.vertIdIdxMap.find(vId)->second);
			}
		}

		fprintf(fOut, "%llu\n", vertIndices.size());
		fprintf(fOut, "(");
		fwrite(vertIndices.data(), sizeof(int), vertIndices.size(), fOut);
		fprintf(fOut, ")\n");
		fprintf(fOut, "//**********************************************************************************//\n");
	} catch (...) {
	}

	fclose(fOut);
}

void Volume::writePolyMeshOwnerCells(const std::string& dirName, const PolymeshTables& tables) const
{

	string filename = dirName + "/owner";
	FILE* fOut = fopen(filename.c_str(), "wb");
	try {
		writeFOAMHeader(fOut, "binary", "labelList", "owner");

		vector<int> indices;
		indices.reserve(tables.faceIdxIdMap.size());
		for (const auto& faceId : tables.faceIdxIdMap) {
			const auto& face = getPolygon(faceId);
			const auto& cellIds = face.getCellIds();
			int minIdx = INT_MAX;
			for (const auto& cellId : cellIds) {
				int cellIdx = tables.cellIdIdxMap.find(cellId)->second;
				if (cellIdx < minIdx)
					minIdx = cellIdx;
			}
			indices.push_back(minIdx);
		}

		fprintf(fOut, "%llu\n", indices.size());
		fprintf(fOut, "(");
		fwrite(indices.data(), sizeof(int), indices.size(), fOut);
		fprintf(fOut, ")\n");
		fprintf(fOut, "//**********************************************************************************//\n");
	} catch (...) {
	}

	fclose(fOut);
}

void Volume::writePolyMeshNeighborCells(const std::string& dirName, const PolymeshTables& tables) const
{
	string filename = dirName + "/neighbour";
	FILE* fOut = fopen(filename.c_str(), "wb");
	try {
		writeFOAMHeader(fOut, "binary", "labelList", "neighbour");

		vector<int32_t> indices;
		indices.reserve(tables.faceIdxIdMap.size());
		for (const auto& faceId : tables.faceIdxIdMap) {
			const auto& face = getPolygon(faceId);
			const auto& cellIds = face.getCellIds();
			int32_t maxIdx = -1;
			if (cellIds.size() == 2) {
				for (const auto& cellId : cellIds) {
					int32_t cellIdx = tables.cellIdIdxMap.find(cellId)->second;
					if (cellIdx > maxIdx)
						maxIdx = cellIdx;
				}
				indices.push_back(maxIdx);
			}
		}

		fprintf(fOut, "%llu\n", indices.size());
		fprintf(fOut, "(");
		fwrite(indices.data(), sizeof(int32_t), indices.size(), fOut);
		fprintf(fOut, ")\n");
		fprintf(fOut, "//**********************************************************************************//\n");
	} catch (...) {
	}

	fclose(fOut);

}

void Volume::writePolyMeshBoundaries(const std::string& dirName, const PolymeshTables& tables) const
{
	const string names[6] = {
		"front",
		"left",
		"bottom",
		"back",
		"right",
		"top",
	};

	string filename = dirName + "/boundary";
	FILE* fOut = fopen(filename.c_str(), "wb");
	try {
		writeFOAMHeader(fOut, "binary", "polyBoundaryMesh", "boundary");
		int32_t nWallFaces = (int32_t)tables.faceIdxIdMap.size() - tables.boundaryIdx;
		int32_t numBoundaries = 6;
		if (nWallFaces > 0)
			numBoundaries++;

		fprintf(fOut, "%d\n", numBoundaries);
		fprintf(fOut, "(\n");

		for (int32_t i = 0; i < 6; i++) {
			fprintf(fOut, "  %s\n", names[i].c_str());
			fprintf(fOut, "  {\n");
			fprintf(fOut, "    type patch;\n");
			int32_t nFaces;
			if (i + 1 < 6)
				nFaces = tables.boundaryIndices[i + 1] - tables.boundaryIndices[i];
			else
				nFaces = tables.boundaryIdx - tables.boundaryIndices[i];
			fprintf(fOut, "    nFaces %d;\n", nFaces);
			fprintf(fOut, "    startFace %d;\n", tables.boundaryIndices[i]);
			fprintf(fOut, "  }\n");
		}

		if (nWallFaces > 0) {
			fprintf(fOut, "  walls\n");
			fprintf(fOut, "  {\n");
			fprintf(fOut, "    type wall;\n");
			fprintf(fOut, "    nFaces %d;\n", nWallFaces);
			fprintf(fOut, "    startFace %d;\n", tables.boundaryIdx);
			fprintf(fOut, "  }\n");
		}

		fprintf(fOut, ")\n");
		fprintf(fOut, "//**********************************************************************************//\n");
	} catch (...) {
	}

	fclose(fOut);
}

void Volume::writeFOAMHeader(FILE* fOut, const string& fileType, const string& foamClass, const string& object) const
{
	fprintf(fOut, "/*--------------------------------*- C++ -*----------------------------------*\\\n");
	fprintf(fOut, "  =========                 |\n");
	fprintf(fOut, "  \\\\      /  F ield         | OpenFOAM: The Open Source CFD Toolbox\n");
	fprintf(fOut, "   \\\\    /   O peration     | Version:  v1812\n");
	fprintf(fOut, "    \\\\  /    A nd           | Web:      www.OpenFOAM.com\n");
	fprintf(fOut, "     \\\\/     M anipulation  |\n");
	fprintf(fOut, "\\* --------------------------------------------------------------------------*/\n");
	fprintf(fOut, "FoamFile\n");
	fprintf(fOut, "	{\n");
	fprintf(fOut, "		format      %s;\n", fileType.c_str());
	fprintf(fOut, "		class       %s;\n", foamClass.c_str());
	fprintf(fOut, "		location    \"constant/polyMesh\";\n");
	fprintf(fOut, "		object      %s;\n", object.c_str());
	fprintf(fOut, "	}\n");
	fprintf(fOut, "//* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *//\n");
	fprintf(fOut, "\n");
	fprintf(fOut, "\n");

}

bool Volume::verifyTopology(bool multiCore) const
{
	bool result = true;

	runThreadPool([this, &result](size_t threadNum, size_t linearIdx, const std::shared_ptr<Block>& pBlk)->bool {
		if (pBlk)
			if (!pBlk->verifyTopology()) {
				result = false;
			}
		return true;
	}, multiCore);
	return result;
}

template<class L>
inline void Volume::runThreadPool(const L& fLambda, bool multiCore) const
{
	_threadPool.run(_blocks.size(), [this, fLambda](size_t threadNum, size_t linearIdx){
		Index3D blkIdx = calBlockIndexFromLinearIndex(linearIdx);
		auto& pBlk = _blocks[linearIdx];
		if (pBlk) {
			MultiCore::scoped_set_local_heap sth(pBlk->getHeapPtr());
			fLambda(threadNum, linearIdx, pBlk);
		} else {
			fLambda(threadNum, linearIdx, nullptr);
		}
	}, multiCore);
}

template<class L>
inline void Volume::runThreadPool(const L& fLambda, bool multiCore)
{
	_threadPool.run(_blocks.size(), [this, fLambda](size_t threadNum, size_t linearIdx) {
		Index3D blkIdx = calBlockIndexFromLinearIndex(linearIdx);
		auto& pBlk = _blocks[linearIdx];
		if (pBlk) {
			MultiCore::scoped_set_local_heap sth(pBlk->getHeapPtr());
			fLambda(threadNum, linearIdx, pBlk);
		}
		else {
			fLambda(threadNum, linearIdx, nullptr);
		}
	}, multiCore);
}

template<class L>
void Volume::runThreadPool333(const L& fLambda, bool multiCore)
{
	const unsigned int stride = 3; // Stride = 3 creates a super block 3x3x3 across. Each thread has exclusive access to the super block
	Index3D phaseIdx, idx;

	// Pass one, process all cells. That can leave faces in an interim state.
	// If the interim cell is also modified, it should be taken care of on pass 1
	// Adjacents cells with face splits or vertex insertions may be left behind
	vector<size_t> blocksToProcess;
	for (phaseIdx[0] = 0; phaseIdx[0] < stride; phaseIdx[0]++) {
		for (phaseIdx[1] = 0; phaseIdx[1] < stride; phaseIdx[1]++) {
			for (phaseIdx[2] = 0; phaseIdx[2] < stride; phaseIdx[2]++) {
				// Collect the indices for all blocks in this phase
				blocksToProcess.clear();

				for (idx[0] = phaseIdx[0]; idx[0] < s_volDim[0]; idx[0] += stride) {
					for (idx[1] = phaseIdx[1]; idx[1] < s_volDim[1]; idx[1] += stride) {
						for (idx[2] = phaseIdx[2]; idx[2] < s_volDim[2]; idx[2] += stride) {
							size_t linearIdx = calLinearBlockIndex(idx);
							blocksToProcess.push_back(linearIdx);
						}
					}
				}

//				sort(blocksToProcess.begin(), blocksToProcess.end());
				// Process those blocks in undetermined order
				if (!blocksToProcess.empty()) {
					_threadPool.run(blocksToProcess.size(), [this, fLambda, &blocksToProcess](size_t threadNum, size_t idx) {
						size_t linearIdx = blocksToProcess[idx];
						Index3D blkIdx = calBlockIndexFromLinearIndex(linearIdx);
						auto& pBlk = _blocks[linearIdx];
						if (pBlk) {
							MultiCore::scoped_set_local_heap sth(pBlk->getHeapPtr());
							fLambda(threadNum, linearIdx, pBlk);
						} else {
							fLambda(threadNum, linearIdx, nullptr);
						}
					}, multiCore);
				}
			}
		}
	}
}