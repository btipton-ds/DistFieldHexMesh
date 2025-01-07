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
#include <tm_spatialSearch.hpp>
#include <triMesh.h>

#include <block.h>
#include <volume.h>
#include <splitParams.h>
#include <vertex.h>
#include <polyhedronSplitter.h>
#include <vertexSpatialTree.h>
#include <tolerances.h>
#include <utils.h>
#include <gradingOp.h>

using namespace std;
using namespace DFHM;

Volume::Volume(const Index3D& dims)
	: _volDim(dims)
	, _modelDim(dims)
	, _modelDimOrigin(0, 0, 0)
	, _threadPool(MultiCore::getNumCores())
{
}

Volume::Volume(const Volume& src)
	: _volDim(src._volDim)
	, _modelDim(src._modelDim)
	, _modelDimOrigin(src._modelDimOrigin)
	, _pAppData(src._pAppData)
	, _modelBundingBox(src._modelBundingBox)
	, _modelCornerPts(src._modelCornerPts)
	, _volCornerPts(src._volCornerPts)
	, _blocks(src._blocks)
	, _adHocBlockTree(src._adHocBlockTree)
	, _sharpEdgeIndices(src._sharpEdgeIndices)
	, _sharpVertIndices(src._sharpVertIndices)
	, _hasSharpVertPlane(src._hasSharpVertPlane)
	, _sharpVertPlane(src._sharpVertPlane)
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

void Volume::setVolDim(const Index3D& blockSize, bool resetBoundaryDim)
{
	_volDim =  blockSize;
	if (resetBoundaryDim) {
		_modelDimOrigin = Index3D(0, 0, 0);
		_modelDim = _volDim;
	}

	size_t numBlocks = _volDim[0] * _volDim[1] * _volDim[2];
	_blocks.resize(numBlocks);
}

const Index3D& Volume::volDim() const
{
	return _volDim;
}

const Index3D& Volume::modelDim() const
{
	return _modelDim;
}

void Volume::setVolCornerPts(const std::vector<Vector3d>& pts)
{
	_volCornerPts = pts;
}

Index3D Volume::calBlockIndexFromLinearIndex(size_t linearIdx, const Index3D& volDim)
{
	Index3D result;
	size_t temp = linearIdx;

	size_t denom = volDim[0] * volDim[1];
	result[2] = (Index3DBaseType)(temp / denom);
	temp = temp % denom;

	denom = volDim[0];

	result[1] = (Index3DBaseType)(temp / denom);
	temp = temp % denom;
	result[0] = (Index3DBaseType)temp;

	if (calLinearBlockIndex(result, volDim) != linearIdx) {
		throw runtime_error("calBlockIndexFromLinearIndex failed");
	}

	return result;
}

Index3D Volume::calBlockIndexFromLinearIndex(size_t linearIdx) const {
	return calBlockIndexFromLinearIndex(linearIdx, _volDim);
}

void Volume::findFeatures()
{
#if 0
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
#endif
}

void Volume::findSharpEdgeEdges()
{
#if 0
	const double sinSharpAngle = sin(getSharpAngleRad());
	for (size_t edgeIdx = 0; edgeIdx < _pModelTriMesh->numEdges(); edgeIdx++) {
		const auto& edge = _pModelTriMesh->getEdge(edgeIdx);
		auto pTopol = edge.getTopol(_pModelTriMesh->getId());
		if (pTopol->_numFaces == 2 && _pModelTriMesh->isEdgeSharp(edgeIdx, sinSharpAngle)) {
			_sharpEdgeIndices.insert(edgeIdx);
		}
	}
#endif
}

void Volume::findSharpVertices(const TriMesh::CMeshPtr& pMesh, double sharpAngleRadians, std::vector<size_t>& vertIndices)
{
	const double sinSharpAngle = sin(sharpAngleRadians);

	size_t numVerts = pMesh->numVertices();
	for (size_t vIdx = 0; vIdx < numVerts; vIdx++) {
		const auto& vert = pMesh->getVert(vIdx);
		double maxDp = 1;
		const auto pEdgeIndices = vert.getEdgeIndices(pMesh->getId());
		vector<Vector3d> radiantVectors;
		for (size_t edgeIdx : *pEdgeIndices) {
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

BlockPtr& Volume::getBoundingBlock(const Index3D& blkIdx, const Vector3d cPts[8])
{
	size_t linIdx = calLinearBlockIndex(blkIdx);
	assert(linIdx < _blocks.size());
	if (!_blocks[linIdx]) {
		BlockPtr p = make_shared<Block>(this, blkIdx, cPts);
		_blocks[linIdx] = p;

		auto bBox = p->getBBox();
		_adHocBlockTree.add(bBox, p);
	}
	return _blocks[linIdx];
}

Index3D Volume::determineOwnerBlockIdx(const Vector3d& point) const
{
	const int64_t numSubDivisions = 1024;

	Index3D result;
	const double pTol = Tolerance::paramTol();
	const double dTolSqr = Tolerance::sameDistTol() * Tolerance::sameDistTol();

	const double iMax = 1.0 - 1.0 / (1ull << 18);
	const Index3D& mDim = modelDim();
	const auto& modelCorners = getModelCornerPts();

	Vector3<double> uvw;
	bool inBounds = TRI_LERP_INV(point, modelCorners, uvw);
	if (inBounds) {
		for (int i = 0; i < 3; i++) {
			const int64_t numDivisions = mDim[i] * numSubDivisions;
			int64_t n = (int64_t) (numDivisions * uvw[i]);
			if (n < 0 || n > numDivisions) {
				inBounds = false;
				break;
			}

			Index3DBaseType blockIVal = (Index3DBaseType) (n / numSubDivisions);
			int64_t subDivIval = n % numSubDivisions;

			result[i] = blockIVal;
			
			if (subDivIval == numSubDivisions - 1 && result[i] < _volDim[i] - 1)
				result[i]++;
		}

		if (inBounds) {
			result += _modelDimOrigin;
			if (result.isValid()) {
				return result;
			}
		}
	}

	std::vector<BlockPtr> foundBlocks;
	CBoundingBox3Dd ptBox(point);
	if (_adHocBlockTree.find(ptBox, foundBlocks)) {
		for (size_t idx = 0; idx < foundBlocks.size(); idx++) {
			const auto& pBlk = foundBlocks[idx];
			const auto& blockCorners = pBlk->getUnalignedBBox();
			bool storeInAdjBlock[3];
			if (blockCorners.contains(point, numSubDivisions, storeInAdjBlock)) {
				result = pBlk->getBlockIdx();
				for (int i = 0; i < 3; i++) {
					if (storeInAdjBlock[i] && result[i] < _volDim[i] - 1)
						result[i]++;
				}
				return result;
			}
		}
	} else {
		int dbgBreak = 1;
	}

	return result;
}

shared_ptr<Block> Volume::createBlock(const Index3D& blockIdx, bool forReading)
{
	size_t idx = calLinearBlockIndex(blockIdx);
	auto pBlock = _blocks[idx];
	if (pBlock)
		return pBlock;

	return make_shared<Block>(this, blockIdx, _modelCornerPts, forReading);
}

BlockPtr Volume::createBlock(size_t linearIdx)
{
	auto pBlock = _blocks[linearIdx];
	if (pBlock)
		return pBlock;

	Index3D blockIdx = calBlockIndexFromLinearIndex(linearIdx);
	Vector3d uvw[2];
	for (int i = 0; i < 3; i++) {
		uvw[0][i] = blockIdx[i] / (double)_modelDim[i];
		uvw[1][i] = (blockIdx[i] + 1) / (double)_modelDim[i];
	}
	const vector<Vector3d>& cPts = _modelCornerPts;
	std::vector<Vector3d> pts({
		TRI_LERP(cPts, uvw[0][0], uvw[0][1], uvw[0][2]),
		TRI_LERP(cPts, uvw[1][0], uvw[0][1], uvw[0][2]),
		TRI_LERP(cPts, uvw[1][0], uvw[1][1], uvw[0][2]),
		TRI_LERP(cPts, uvw[0][0], uvw[1][1], uvw[0][2]),

		TRI_LERP(cPts, uvw[0][0], uvw[0][1], uvw[1][2]),
		TRI_LERP(cPts, uvw[1][0], uvw[0][1], uvw[1][2]),
		TRI_LERP(cPts, uvw[1][0], uvw[1][1], uvw[1][2]),
		TRI_LERP(cPts, uvw[0][0], uvw[1][1], uvw[1][2]),
	});

	return make_shared<Block>(this, blockIdx, pts);
}

CBoundingBox3Dd Volume::getModelBBox() const
{
	CBoundingBox3Dd result;

	const vector<Vector3d>& cPts = _modelCornerPts;
	for (const auto& pt : cPts)
		result.merge(pt);

	return result;
}

CBoundingBox3Dd Volume::getVolumeBBox() const
{
	CBoundingBox3Dd result;

	const vector<Vector3d>& cPts = _volCornerPts;
	for (const auto& pt : cPts)
		result.merge(pt);

	return result;
}

void Volume::addAllBlocks(Block::TriMeshGroup& triMeshes, Block::glPointsGroup& faceEdges)
{
	const auto& dim = volDim();

	Index3D blkIdx;
	for (blkIdx[0] = 0; blkIdx[0] < dim[0]; blkIdx[0]++) {
		for (blkIdx[1] = 0; blkIdx[1] < dim[1]; blkIdx[1]++) {
			for (blkIdx[2] = 0; blkIdx[2] < dim[2]; blkIdx[2]++) {
				auto linearIdx = calLinearBlockIndex(blkIdx);
				_blocks[linearIdx] = createBlock(blkIdx, false);
			}
		}
	}

	makeFaceTris(triMeshes, Index3D(0, 0, 0), volDim(), true);
}

void Volume::clear()
{
	// Clear contents to remove cross block pointers
	runThreadPool333([this](size_t threadNum, size_t linearIdx, const BlockPtr& pBlk)->bool {
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

void Volume::buildModelBlocks(const BuildCFDParams& params, const Vector3d pts[8], const CMesh::BoundingBox& volBox, bool multiCore)
{
	_modelBundingBox.clear();
	_modelBundingBox.merge(volBox);
	for (int i = 0; i < 8; i++) {
		_modelBundingBox.merge(pts[i]);
		_modelCornerPts[i] = pts[i];
	}
	
	const auto& dim = volDim();
	size_t numBlocks = dim[0] * dim[1] * dim[2];
	_blocks.resize(numBlocks);
	for (size_t linearIdx = 0; linearIdx < _blocks.size(); linearIdx++) {
		_blocks[linearIdx] = createBlock(linearIdx);
	}

/*
	// Cannot create subBlocks until all blocks are created so they can be connected
	runThreadPool333([this](size_t threadNum, size_t linearIdx, const BlockPtr& pBlk)->bool {
		pBlk->createBlockCells(TS_REAL);
		return true;
	}, multiCore);
*/
	makeSurroundingBlocks(params, pts);

	Index3D idx;
	for (idx[0] = 0; idx[0] < _modelDim[0]; idx[0]++) {
		for (idx[1] = 0; idx[1] < _modelDim[1]; idx[1]++) {
			for (idx[2] = 0; idx[2] < _modelDim[2]; idx[2]++) {
				auto pBlk = getBlockPtr(idx + _modelDimOrigin);
				pBlk->createBlockCells(TS_REAL);
			}
		}
	}

	gradeSurroundingBlocks(params);
}

void Volume::makeSurroundingBlocks(const BuildCFDParams& params, const Vector3d cPts[8])
{
	if (!params.symXAxis)
		insertBlocks(params, CFT_BACK);
#if 1
	insertBlocks(params, CFT_FRONT);

	if (!params.symYAxis)
		insertBlocks(params, CFT_LEFT);

	insertBlocks(params, CFT_RIGHT);

	if (!params.symZAxis)
		insertBlocks(params, CFT_BOTTOM);

	insertBlocks(params, CFT_TOP);
#endif

}

void Volume::gradeSurroundingBlocks(const BuildCFDParams& params)
{
	Index3D idx;
	const auto& dims = volDim();

#if 1
	for (idx[0] = 0; idx[0] < dims[0]; idx[0]++) {
		for (idx[1] = 0; idx[1] < dims[1]; idx[1]++) {
			for (int j = 0; j < 2; j++) {
				Index3D divs(1, 1, 1);
				Vector3d grading(1, 1, 1);

				if (j == 0) {
					if (params.symZAxis)
						continue;
					idx[2] = 0;
					divs[2] = params.zMinDivs;
					grading[2] = 1 / params.zMinGrading;
				} else {
					idx[2] = dims[2] - 1;
					divs[2] = params.zMaxDivs;
					grading[2] = params.zMaxGrading;
				}

				if (idx[0] == 0) {
					if (params.symXAxis)
						continue;

					divs[0] = params.xMinDivs;
					grading[0] = 1 / params.xMinGrading;
				} else if (idx[0] == dims[0] - 1) {
					divs[0] = params.xMaxDivs;
					grading[0] = params.xMaxGrading;
				}

				if (idx[1] == 0) {
					if (!params.symYAxis) {
						divs[1] = params.yMinDivs;
						grading[1] = 1 / params.yMinGrading;
					}
				} else if (idx[1] == dims[1] - 1) {
					divs[1] = params.yMaxDivs;
					grading[1] = params.yMaxGrading;
				}

				auto pBlk = getBlockPtr(idx);
				GradingOp gr(pBlk, params, divs, grading);
				gr.createGradedCells();
			}
		}
	}
#endif

#if 1
	for (idx[1] = 0; idx[1] < dims[1]; idx[1]++) {
		for (idx[2] = 1; idx[2] < dims[2] - 1; idx[2]++) {
			for (int j = 0; j < 2; j++) {
				Index3D divs(1, 1, 1);
				Vector3d grading(1, 1, 1);

				if (j == 0) {
					if (params.symXAxis)
						continue;
					idx[0] = 0;
					divs[0] = params.xMinDivs;
					grading[0] = 1 / params.xMinGrading;
				}
				else {
					idx[0] = dims[0] - 1;
					divs[0] = params.xMaxDivs;
					grading[0] = params.xMaxGrading;
				}

				if (idx[1] == 0) {
					if (!params.symYAxis) {
						divs[1] = params.yMinDivs;
						grading[1] = 1 / params.yMinGrading;
					}
				}
				else if (idx[1] == dims[1] - 1) {
					divs[1] = params.yMaxDivs;
					grading[1] = params.yMaxGrading;
				}

				auto pBlk = getBlockPtr(idx);
				GradingOp gr(pBlk, params, divs, grading);
				gr.createGradedCells();
			}
		}
	}
#endif

#if 1
	for (idx[0] = 1; idx[0] < dims[0] - 1; idx[0]++) {
		for (idx[2] = 1; idx[2] < dims[2] - 1; idx[2]++) {
			for (int j = 0; j < 2; j++) {
				Index3D divs(1, 1, 1);
				Vector3d grading(1, 1, 1);

				if (j == 0) {
					if (params.symYAxis)
						continue;

					idx[1] = 0;
					if (!params.symYAxis) {
						divs[1] = params.yMinDivs;
						grading[1] = 1 / params.yMinGrading;
					}
				}
				else {
					idx[1] = dims[1] - 1;
					divs[1] = params.yMaxDivs;
					grading[1] = params.yMaxGrading;
				}

				if (idx[1] == 0) {
					if (!params.symYAxis) {
						divs[1] = params.yMinDivs;
						grading[1] = 1 / params.yMinGrading;
					}
				} else if (idx[1] == dims[1] - 1) {
					divs[1] = params.yMaxDivs;
					grading[1] = params.yMaxGrading;
				}

				auto pBlk = getBlockPtr(idx);
				GradingOp gr(pBlk, params, divs, grading);
				gr.createGradedCells();
			}
		}
	}
#endif
}

void Volume::buildCFDHexes(const CMeshPtr& pTriMesh, const BuildCFDParams& params, bool multiCore)
{
#if 0
	_pModelTriMesh = pTriMesh;
	_boundingBox = pTriMesh->getBBox();
	_boundingBox.growPercent(0.0125);

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

		cutWithTriMesh(params, false && multiCore);
	}

	Utils::Timer::dumpAll();
	assert(verifyTopology(multiCore));

	cout << "Num polyhedra: " << numPolyhedra() << "\n";
	cout << "Num faces. All: " << numFaces(true) << ", outer: " << numFaces(false) << "\n";
#endif
}

void Volume::createBlocks(const BuildCFDParams& params, const Vector3d& blockSpan, bool multiCore)
{
	runThreadPool([this, &blockSpan](size_t threadNum, size_t linearIdx, const BlockPtr& pBlk)->bool {
		auto blockIdx = calBlockIndexFromLinearIndex(linearIdx);
		_blocks[linearIdx] = createBlock(blockIdx, false);
		return true;
	}, multiCore);

	// Cannot create subBlocks until all blocks are created so they can be connected
	runThreadPool333([this, &blockSpan](size_t threadNum, size_t linearIdx, const BlockPtr& pBlk)->bool {
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
			runThreadPool333([this](size_t threadNum, size_t linearIdx, const BlockPtr& pBlk)->bool {
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
					runThreadPool333([this, &params, &didSplit, sinEdgeAngle](size_t threadNum, size_t linearIdx, const BlockPtr& pBlk)->bool {
						if (pBlk->doPresplits(params))
							didSplit = true;
						return true;
					}, multiCore);
					count++;
				}
			}

			bool changed = false;
			runThreadPool333([this, passNum, &params, sinEdgeAngle, &changed](size_t threadNum, size_t linearIdx, const BlockPtr& pBlk)->bool {
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

void Volume::cutWithTriMesh(const BuildCFDParams& params, bool multiCore)
{
	bool changed = false;
	runThreadPool333([this, &changed, &params](size_t threadNum, size_t linearIdx, const BlockPtr& pBlk)->bool {
		pBlk->iteratePolyhedraInOrder(TS_REAL, [&pBlk, &changed, &params](const Index3DId& cellId, Polyhedron& cell) {
			if (cell.containsSharps()) {
				if (cell.setNeedsCleanFaces())
					changed = true;
			}
		});
		return true;
	}, multiCore);

	changed = false;
	runThreadPool333([this, &changed, &params](size_t threadNum, size_t linearIdx, const BlockPtr& pBlk)->bool {
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
		runThreadPool333([this, &done](size_t threadNum, size_t linearIdx, const BlockPtr& pBlk)->bool {
			pBlk->splitRequiredPolyhedra();
			return true;
		}, multiCore);

		runThreadPool333([this, &done](size_t threadNum, size_t linearIdx, const BlockPtr& pBlk)->bool {
			pBlk->updateSplitStack();
			return true;
		}, multiCore);

		runThreadPool333([this, &done](size_t threadNum, size_t linearIdx, const BlockPtr& pBlk)->bool {
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
	runThreadPool333([this](size_t threadNum, size_t linearIdx, const BlockPtr& pBlk)->bool {
		pBlk->imprintTJointVertices();
		return true;
	}, multiCore);
#ifdef _DEBUG
	bool allCellsClosed = true;
	runThreadPool([this, &allCellsClosed](size_t threadNum, size_t linearIdx, const BlockPtr& pBlk)->bool {
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
	runThreadPool([this](size_t threadNum, size_t linearIdx, const BlockPtr& pBlk)->bool {
		pBlk->dumpOpenCells();
		return true;
	}, multiCore);
#endif
}

void Volume::insertBlocks(const BuildCFDParams& params, CubeFaceType face)
{
	size_t linIdxSrc, linIdxDst;
	Index3D idxSrc, idxDst;
	map<Index3D, Index3D> idRemap;

	auto srcDims = volDim();
	auto dstDims = srcDims;
	const size_t srcSize = _blocks.size();

	switch (face) {
	case CFT_FRONT:
		dstDims[0]++;
		break;
	case CFT_BACK:
		dstDims[0]++;
		_modelDimOrigin[0]++;
		break;
	case CFT_RIGHT:
		dstDims[1]++;
		break;
	case CFT_LEFT:
		dstDims[1]++;
		_modelDimOrigin[1]++;
		break;
	case CFT_TOP:
		dstDims[2]++;
		break;
	case CFT_BOTTOM:
		dstDims[2]++;
		_modelDimOrigin[2]++;
		break;
	default:
		break;
	}

	setVolDim(dstDims, false);

	for (size_t i = srcSize - 1; i != -1; i--) {
		Index3D idxSrc = calBlockIndexFromLinearIndex(i, srcDims);
		idxDst = idxSrc;
		switch (face) {
		case CFT_BACK:
			idxDst[0] += 1;
			break;
		case CFT_LEFT:
			idxDst[1] += 1;
			break;
		case CFT_BOTTOM:
			idxDst[2] += 1;
			break;
		default:
			break;
		}

		linIdxSrc = calLinearBlockIndex(idxSrc, srcDims);
		linIdxDst = calLinearBlockIndex(idxDst);
		idRemap[idxSrc] = idxDst;

		_blocks[linIdxDst] = _blocks[linIdxSrc];
		if (linIdxDst != linIdxSrc) {
			_blocks[linIdxSrc] = nullptr;
		}
	}

	for (size_t i = 0; i < _blocks.size(); i++) {
		if (_blocks[i])
			_blocks[i]->remapBlockIndices(idRemap);
	}

	vector<BlockPtr> adHocBlocks;
	Vector3d newCorners[8];
	switch (face) {
	case CFT_FRONT:
		idxSrc[0] = dstDims[0] - 2;
		for (idxSrc[1] = 0; idxSrc[1] < dstDims[1]; idxSrc[1]++) {
			for (idxSrc[2] = 0; idxSrc[2] < dstDims[2]; idxSrc[2]++) {
				idxDst = idxSrc;
				idxDst[0] = dstDims[0] - 1;
				linIdxSrc = calLinearBlockIndex(idxSrc);
				linIdxDst = calLinearBlockIndex(idxDst);
				auto pBlkSrc = _blocks[linIdxSrc];

				const vector<Vector3d>& cPts = pBlkSrc->_corners;
				newCorners[1] = newCorners[0] = cPts[1];
				newCorners[2] = newCorners[3] = cPts[2];
				newCorners[5] = newCorners[4] = cPts[5];
				newCorners[6] = newCorners[7] = cPts[6];
				newCorners[1][0] = newCorners[2][0] = newCorners[5][0] = newCorners[6][0] = params.xMax;
				auto pNewBlock = make_shared<Block>(this, idxDst, newCorners);
				_blocks[linIdxDst] = pNewBlock;
				adHocBlocks.push_back(pNewBlock);
			}
		}
#if 1 && defined(_DEBUG)
		for (const auto& p : _blocks)
			assert(p);
#endif // _DEBUG)

		break;
	case CFT_BACK:
		idxSrc[0] = 1;
		for (idxSrc[1] = 0; idxSrc[1] < dstDims[1]; idxSrc[1]++) {
			for (idxSrc[2] = 0; idxSrc[2] < dstDims[2]; idxSrc[2]++) {
				idxDst = idxSrc;
				idxDst[0] = 0;
				linIdxSrc = calLinearBlockIndex(idxSrc);
				linIdxDst = calLinearBlockIndex(idxDst);
				auto pBlkSrc = _blocks[linIdxSrc];

				const vector<Vector3d>& cPts = pBlkSrc->_corners;
				newCorners[1] = newCorners[0] = cPts[0];
				newCorners[2] = newCorners[3] = cPts[3];
				newCorners[5] = newCorners[4] = cPts[4];
				newCorners[6] = newCorners[7] = cPts[7];
				newCorners[0][0] = newCorners[3][0] = newCorners[4][0] = newCorners[7][0] = params.xMin;
				auto pNewBlock = make_shared<Block>(this, idxDst, newCorners);
				_blocks[linIdxDst] = pNewBlock;
				adHocBlocks.push_back(pNewBlock);
			}
		}
		
#if 1 && defined(_DEBUG)
		for (const auto& p : _blocks)
			assert(p);
#endif // _DEBUG)

		break;
	case CFT_LEFT:
		idxSrc[1] = 1;
		for (idxSrc[0] = 0; idxSrc[0] < dstDims[0]; idxSrc[0]++) {
			for (idxSrc[2] = 0; idxSrc[2] < dstDims[2]; idxSrc[2]++) {
				idxDst = idxSrc;
				idxDst[1] = 0;
				linIdxSrc = calLinearBlockIndex(idxSrc);
				linIdxDst = calLinearBlockIndex(idxDst);
				auto pBlkSrc = _blocks[linIdxSrc];

				const vector<Vector3d>& cPts = pBlkSrc->_corners;
				newCorners[3] = newCorners[0] = cPts[0];
				newCorners[2] = newCorners[1] = cPts[1];
				newCorners[7] = newCorners[4] = cPts[4];
				newCorners[6] = newCorners[5] = cPts[5];
				newCorners[3][1] = newCorners[2][1] = newCorners[7][1] = newCorners[6][1] = params.yMax;
				auto pNewBlock = make_shared<Block>(this, idxDst, newCorners);
				_blocks[linIdxDst] = pNewBlock;
				adHocBlocks.push_back(pNewBlock);
			}
		}
#if 1 && defined(_DEBUG)
		for (const auto& p : _blocks)
			assert(p);
#endif // _DEBUG)

		break;
	case CFT_RIGHT:
		idxSrc[1] = dstDims[1] - 2;
		for (idxSrc[0] = 0; idxSrc[0] < dstDims[0]; idxSrc[0]++) {
			for (idxSrc[2] = 0; idxSrc[2] < dstDims[2]; idxSrc[2]++) {
				idxDst = idxSrc;
				idxDst[1] = dstDims[1] - 1;
				linIdxSrc = calLinearBlockIndex(idxSrc);
				linIdxDst = calLinearBlockIndex(idxDst);
				auto pBlkSrc = _blocks[linIdxSrc];

				const vector<Vector3d>& cPts = pBlkSrc->_corners;
				newCorners[0] = newCorners[3] = cPts[3];
				newCorners[1] = newCorners[2] = cPts[2];
				newCorners[5] = newCorners[6] = cPts[6];
				newCorners[4] = newCorners[7] = cPts[7];
				newCorners[3][1] = newCorners[2][1] = newCorners[6][1] = newCorners[7][1] = params.yMax;
				auto pNewBlock = make_shared<Block>(this, idxDst, newCorners);
				_blocks[linIdxDst] = pNewBlock;
				adHocBlocks.push_back(pNewBlock);
			}
		}

#if 1 && defined(_DEBUG)
		for (const auto& p : _blocks)
			assert(p);
#endif // _DEBUG)

		break;
	case CFT_TOP:
		idxSrc[2] = dstDims[2] - 2;
		for (idxSrc[0] = 0; idxSrc[0] < dstDims[0]; idxSrc[0]++) {
			for (idxSrc[1] = 0; idxSrc[1] < dstDims[1]; idxSrc[1]++) {
				idxDst = idxSrc;
				idxDst[2] = dstDims[2] - 1;
				linIdxSrc = calLinearBlockIndex(idxSrc);
				linIdxDst = calLinearBlockIndex(idxDst);
				auto pBlkSrc = _blocks[linIdxSrc];

				const vector<Vector3d>& cPts = pBlkSrc->_corners;
				newCorners[0] = newCorners[4] = cPts[4];
				newCorners[1] = newCorners[5] = cPts[5];
				newCorners[2] = newCorners[6] = cPts[6];
				newCorners[3] = newCorners[7] = cPts[7];
				newCorners[4][2] = newCorners[5][2] = newCorners[6][2] = newCorners[7][2] = params.zMax;
				auto pNewBlock = make_shared<Block>(this, idxDst, newCorners);
				_blocks[linIdxDst] = pNewBlock;
				adHocBlocks.push_back(pNewBlock);
			}
		}

#if 1 && defined(_DEBUG)
		for (const auto& p : _blocks)
			assert(p);
#endif // _DEBUG)

		break;
	case CFT_BOTTOM:
		idxSrc[2] = 1;
		for (idxSrc[0] = 0; idxSrc[0] < dstDims[0]; idxSrc[0]++) {
			for (idxSrc[1] = 0; idxSrc[1] < dstDims[1]; idxSrc[1]++) {
				idxDst = idxSrc;
				idxDst[2] = 0;
				linIdxSrc = calLinearBlockIndex(idxSrc);
				linIdxDst = calLinearBlockIndex(idxDst);
				auto pBlkSrc = _blocks[linIdxSrc];

				const vector<Vector3d>& cPts = pBlkSrc->_corners;
				newCorners[0] = newCorners[4] = cPts[0];
				newCorners[1] = newCorners[5] = cPts[1];
				newCorners[2] = newCorners[6] = cPts[2];
				newCorners[3] = newCorners[7] = cPts[3];
				newCorners[0][2] = newCorners[1][2] = newCorners[2][2] = newCorners[3][2] = params.zMin;
				auto pNewBlock = make_shared<Block>(this, idxDst, newCorners);
				_blocks[linIdxDst] = pNewBlock;
				adHocBlocks.push_back(pNewBlock);
			}
		}

#if 1 && defined(_DEBUG)
		for (const auto& p : _blocks)
			assert(p);
#endif // _DEBUG)

		break;
	default:
		break;
	}

	for (size_t i = 0; i < _blocks.size(); i++) {
		const auto& pBlk = _blocks[i];
		Index3D blkIdx = calBlockIndexFromLinearIndex(i);
		assert(pBlk->verifyIndices(blkIdx));
	}

	updateAdHocBlockSearchTree(adHocBlocks);
}

void Volume::updateAdHocBlockSearchTree(const std::vector<BlockPtr>& adHocBlocks)
{
	if (_adHocBlockTree.getBounds().empty()) {
		auto volBox = getVolumeBBox();
		volBox.growPercent(0.01);
		_adHocBlockTree.reset(volBox);
	}

	for (const auto& pBlock : adHocBlocks) {
		const auto& blkIdx = pBlock->getBlockIdx();
		const auto& bBox = pBlock->getBBox();

		_adHocBlockTree.add(bBox, pBlock);


		const auto& pts = pBlock->getCornerPts();
		Vector3d ctr(0, 0, 0);
		for (int i = 0; i < 8; i++)
			ctr += pts[i];
		ctr /= 8;
		auto tstIdx = determineOwnerBlockIdx(ctr); // This uses the newly added tree entry
		assert(tstIdx == blkIdx);
	}

#if 0 && defined(_DEBUG)
	for (size_t linIdx = 0; linIdx < _blocks.size(); linIdx++) {
		const auto& pBlk = _blocks[linIdx];
		if (!pBlk->verifyDeterminOwnerBlockIndex()) {
			assert(!"verifyDeterminOwnerBlockIndex failed");
		}
	}
#endif
}

void Volume::makeFaceTriMesh(FaceDrawType faceType, Block::TriMeshGroup& triMeshes, const shared_ptr<Block>& pBlock, size_t threadNum) const
{
	CBoundingBox3Dd bbox = _modelBundingBox;
	bbox.merge(pBlock->_boundBox);

	std::vector<Planed> planes;
	getModelBoundaryPlanes(planes);

	CMeshPtr pMesh = triMeshes[faceType][threadNum];
	if (!pMesh) {
		pMesh = make_shared<CMesh>(bbox);
		pMesh->setEnforceManifold(false); // Block meshes are none manifold
		triMeshes[faceType][threadNum] = pMesh;
	}
	pBlock->getBlockTriMesh(faceType, planes, pMesh);
}

void Volume::makeFaceTris(Block::TriMeshGroup& triMeshes, const Index3D& min, const Index3D& max, bool multiCore) const
{
	size_t numThreads = MultiCore::getNumCores();
	triMeshes.resize(FT_ALL + 1);
	for (int j = FT_WALL; j <= FT_ALL; j++) {
		FaceDrawType ft = (FaceDrawType)j;
		triMeshes[ft].resize(numThreads);
	}

	runThreadPool([this, &triMeshes, &min, &max](size_t threadNum, size_t linearIdx, const BlockPtr& blockPtr)->bool {
		if (blockPtr) {
			MultiCore::scoped_set_local_heap st(blockPtr->getHeapPtr());
			Index3D blkIdx = blockPtr->getBlockIdx();
			if (blkIdx[0] >= min[0] && blkIdx[1] >= min[1] && blkIdx[2] >= min[2] &&
				blkIdx[0] <= max[0] && blkIdx[1] <= max[1] && blkIdx[2] <= max[2]) {

				for (int j = FT_WALL; j <= FT_ALL; j++) {
					FaceDrawType ft = (FaceDrawType)j;
					makeFaceTriMesh(ft, triMeshes, blockPtr, threadNum);
				}
			}
		}
		return true;
	}, multiCore);

	for (size_t i = 0; i < numThreads; i++) {
		for (int j = FT_WALL; j <= FT_ALL; j++) {
			FaceDrawType ft = (FaceDrawType)j;
			if (triMeshes[ft][i])
				triMeshes[ft][i]->changed();
		}
	}
}

void Volume::makeFaceEdges(FaceDrawType faceType, Block::glPointsGroup& faceEdges, const shared_ptr<Block>& pBlock, size_t threadNum) const
{
	CBoundingBox3Dd bbox = _modelBundingBox;
	bbox.merge(pBlock->_boundBox);

	std::vector<Planed> planes;
	getModelBoundaryPlanes(planes);

	Block::glPointsPtr pPoints = faceEdges[faceType][threadNum];
	if (!pPoints) {
		pPoints = make_shared<Block::GlPoints>();
		faceEdges[faceType][threadNum] = pPoints;
	}

	pBlock->makeEdgeSets(faceType, planes, pPoints);
}

void Volume::makeEdgeSets(Block::glPointsGroup& faceEdges, const Index3D& min, const Index3D& max, bool multiCore) const
{
	size_t numThreads = MultiCore::getNumCores();

	faceEdges.resize(FT_ALL + 1);
	for (int j = FT_WALL; j <= FT_ALL; j++) {
		FaceDrawType ft = (FaceDrawType)j;
		faceEdges[ft].resize(numThreads);
	}


	runThreadPool([this, &faceEdges, &min, &max](size_t threadNum, size_t linearIdx, const BlockPtr& blockPtr)->bool {
		if (blockPtr) {
			MultiCore::scoped_set_local_heap st(blockPtr->getHeapPtr());
			Index3D blkIdx = blockPtr->getBlockIdx();
			if (blkIdx[0] >= min[0] && blkIdx[1] >= min[1] && blkIdx[2] >= min[2] &&
				blkIdx[0] <= max[0] && blkIdx[1] <= max[1] && blkIdx[2] <= max[2]) {

				for (int j = FT_WALL; j <= FT_ALL; j++) {
					FaceDrawType ft = (FaceDrawType)j;
					makeFaceEdges(ft, faceEdges, blockPtr, threadNum);
				}
			}
		}
		return true;
	}, multiCore);

	for (size_t i = 0; i < numThreads; i++) {
		for (int j = FT_WALL; j <= FT_ALL; j++) {
			FaceDrawType ft = (FaceDrawType)j;
			if (faceEdges[ft][i])
				faceEdges[ft][i]->changed();
		}
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
#if 0
	map<Index3DId, set<Index3DId>> cellToFaceIdsMap;
	vector<size_t> modelTriIndices;
	for (const auto& cellId : cellIds) {
		auto pBlk = getBlockPtr(cellId);
		pBlk->cellFunc(TS_REAL,cellId, [&cellToFaceIdsMap, &modelTriIndices, &pMesh, includeModel, useEdges, sharpOnly](const Polyhedron& cell) {
			const auto& ids = cell.getFaceIds();
			cellToFaceIdsMap.insert(std::make_pair(cell.getId(), ids));
			if (includeModel) {
				auto tmp = cell.getTriIndices();
				auto bbox = cell.getBoundingBox();
				for (size_t triIdx : tmp) {
					if (pMesh->bboxIntersectsTri(bbox, triIdx))
						modelTriIndices.push_back(triIdx);
				}
			}
		});
	}

	vector<Vector3d> pts;
	set<TriMesh::CEdge> modelEdgeSet;
	VertSearchTree_size_t_8 pointToIdxMap(_modelBundingBox);

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

	for (const auto& pair : cellToFaceIdsMap) {
		for (const auto& faceId : pair.second) {
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
	}

	out << "#Vertices " << pts.size() << "\n";
	for (const auto& pt : pts) {
		out << "v " << pt[0] << " " << pt[1] << " " << pt[2] << "\n";
	}

	for (const auto& pt : extraPoints) {
		out << "v " << pt[0] << " " << pt[1] << " " << pt[2] << "\n";
	}

	size_t num = 0;
	for (const auto& pair : cellToFaceIdsMap) {
		num += pair.second.size();
	}

	out << "#Faces " << num << "\n";
	for (const auto& pair : cellToFaceIdsMap) {
		for (const auto& faceId : pair.second) {
			auto pBlk = getBlockPtr(faceId);
			pBlk->faceFunc(TS_REAL, faceId, [&out, &pBlk, &pair, &pointToIdxMap](const Polygon& face) {
				out << "#id: " << face.getId() << "\n";
				out << "#NumVerts: " << face.getVertexIds().size() << "\n";
				out << "f ";
				const auto vIds = face.getOrientedVertexIds(pair.first);
				for (const auto& vertId : vIds) {
					Vector3d pt = pBlk->getVertexPoint(vertId);
					size_t vertIdx;
					if (pointToIdxMap.find(pt, vertIdx))
						out << (vertIdx + 1) << " ";
				}
				out << "\n";
			});
		}
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
					if (pointToIdxMap.find(pt, vertIdx))
						out << (vertIdx + 1) << " ";
				}
				out << "\n";
			}
		} else {
			out << "#Model Faces " << modelTriIndices.size() << "\n";
			for (auto triIdx : modelTriIndices) {
				const auto& tri = _pModelTriMesh->getTri(triIdx);
				out << "f ";
				for (int i = 0; i < 3; i++) {
					size_t modVertIdx = tri[i];
					Vector3d pt = _pModelTriMesh->getVert(modVertIdx)._pt;
					size_t vertIdx;
					if (pointToIdxMap.find(pt, vertIdx))
						out << (vertIdx + 1) << " ";
				}
				out << "\n";
			}
		}
	}
#endif
}

bool Volume::write(ostream& out) const
{
	uint8_t version = 3;
	out.write((char*)&version, sizeof(version));

	_volDim.write(out);
	_modelDim.write(out);
	_modelDimOrigin.write(out);

	_modelBundingBox.write(out);

	const vector<Vector3d>& cPts = _modelCornerPts;
	IoUtil::writeVector3(out, cPts);

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

	if (version < 3) {
		double oldAngle;
		in.read((char*)&oldAngle, sizeof(oldAngle));
	}
	_volDim.read(in);
	if (version < 1) {
		Vector3d tmpV;
		readVector3(in, tmpV);
		readVector3(in, tmpV);
	} else if (version >= 2) {
		_modelDim.read(in);
		_modelDimOrigin.read(in);
	}

	_modelBundingBox.read(in);

	vector<Vector3d> cPts;
	IoUtil::readVector3(in, cPts);
	_modelCornerPts = cPts;

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
			auto pBlock = createBlock(blockIdx, true);
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

void Volume::getModelBoundaryPlanes(std::vector<Planed>& vals) const
{
	Vector3d xAxis(1, 0, 0), yAxis(0, 1, 0), zAxis(0, 0, 1);

	vals.push_back(Planed(_modelBundingBox.getMin(), zAxis)); // bottomPlane
	vals.push_back(Planed(_modelBundingBox.getMax(), zAxis)); // topPlane
	vals.push_back(Planed(_modelBundingBox.getMin(), yAxis)); // leftPlane
	vals.push_back(Planed(_modelBundingBox.getMax(), yAxis)); // rightPlane
	vals.push_back(Planed(_modelBundingBox.getMin(), xAxis)); // frontPlane
	vals.push_back(Planed(_modelBundingBox.getMax(), xAxis)); // backPlane

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

	std::vector<Planed> planes;
	getModelBoundaryPlanes(planes);

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

#ifdef _WIN32
#define FMT_SIZE "%llu\n"
#else
#define FMT_SIZE "%lu\n"
#endif

void Volume::writePolyMeshPoints(const string& dirName, const PolymeshTables& tables) const
{
	string filename = dirName + "/points";
	FILE* fOut = fopen(filename.c_str(), "wb");
	try {
		writeFOAMHeader(fOut, "binary", "vectorField", "points");

		fprintf(fOut, FMT_SIZE, tables.vertIdxIdMap.size());
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

		fprintf(fOut, FMT_SIZE, faceIndices.size());
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

		fprintf(fOut, FMT_SIZE, vertIndices.size());
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

		fprintf(fOut, FMT_SIZE, indices.size());
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

		fprintf(fOut, FMT_SIZE, indices.size());
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

	runThreadPool([this, &result](size_t threadNum, size_t linearIdx, const BlockPtr& pBlk)->bool {
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

	if (_blocks.empty())
		return;

	// Pass one, process all cells. That can leave faces in an interim state.
	// If the interim cell is also modified, it should be taken care of on pass 1
	// Adjacents cells with face splits or vertex insertions may be left behind
	vector<size_t> blocksToProcess;
	for (phaseIdx[2] = 0; phaseIdx[2] < stride; phaseIdx[2]++) {
		for (phaseIdx[1] = 0; phaseIdx[1] < stride; phaseIdx[1]++) {
			for (phaseIdx[0] = 0; phaseIdx[0] < stride; phaseIdx[0]++) {
				// Collect the indices for all blocks in this phase
				blocksToProcess.clear();

				for (idx[2] = phaseIdx[2]; idx[2] < _volDim[2]; idx[2] += stride) {
					for (idx[1] = phaseIdx[1]; idx[1] < _volDim[1]; idx[1] += stride) {
						for (idx[0] = phaseIdx[0]; idx[0] < _volDim[0]; idx[0] += stride) {
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
