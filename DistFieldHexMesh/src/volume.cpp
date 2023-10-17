

#include <triMesh.h>

#include <cell.h>
#include <block.h>
#include <volume.h>
#include <MultiCoreUtil.h>

using namespace std;
using namespace DFHM;
using namespace TriMesh;

#define RUN_MULTI_THREAD false

namespace
{

inline void assignAxes(const Vector3i& indexIn, const Vector3i& axisOrder, Vector3i& indexOut)
{
	indexOut[axisOrder[0]] = indexIn[0];
	indexOut[axisOrder[1]] = indexIn[1];
	indexOut[axisOrder[2]] = indexIn[2];
}

inline void assignAxes(const Vector3i& axisOrder, Vector3i& indexOut)
{
	assignAxes(Vector3i(0, 1, 2), axisOrder, indexOut);
}

}

inline Vector3i Volume::getAxisOrder(AxisIndex axisIdx)
{
	const Vector3i axisOrder = axisIdx == AxisIndex::Z ? Vector3i(0, 1, 2) : (axisIdx == AxisIndex::Y ? Vector3i(2, 0, 1) : Vector3i(1, 2, 0));
	return axisOrder;
}

Volume::Volume(const Index3& blockSize)
{
	setBlockDims(blockSize);
}

Volume::Volume(const Volume& src)
	: DataPool(src)
	, _originMeters(src._originMeters) 
	, _spanMeters(src._spanMeters)
	, _blockDim(src._blockDim)
	, _blocks(src._blocks)
{
}

void Volume::setBlockDims(const Index3& blockSize)
{
	_blockDim = blockSize;
	_blocks.resize(_blockDim[0] * _blockDim[1] * _blockDim[2], -1);
}

const Index3& Volume::getBlockDims() const
{
	return _blockDim;
}

void Volume::rayCastAxis(const TriMesh::CMeshPtr& pTriMesh, AxisIndex axisIdx, std::vector<bool>& blocksToCreate)
{
	mutex allHitsLock;
	const Vector3i axisOrder = getAxisOrder(axisIdx);

	const Vector3d rayDir = axisIdx == AxisIndex::Z ? Vector3d(0, 0, 1) : (axisIdx == AxisIndex::Y ? Vector3d(0, 1, 0) : Vector3d(1, 0, 0));

	map<size_t, map<size_t, RayBlockIntersectVec>>& allHits = axisIdx == AxisIndex::Z ? _zHits : (axisIdx == AxisIndex::Y ? _yHits : _xHits);

	MultiCore::runLambda([this, &allHitsLock, pTriMesh, axisOrder, rayDir, &allHits](size_t threadNum, size_t numThreads) {
		const double TOL = 1.0e-6;
		const size_t axis0 = axisOrder[0];
		const size_t axis1 = axisOrder[1];
		const size_t axis2 = axisOrder[2];
		const size_t numI = _blockDim[axis0] + 1;
		const size_t numJ = _blockDim[axis1] + 1;

		Vector3d origin(_originMeters);

		for (size_t i = 0; i < numI; i++) {
			double t = i / (numI - 1.0);
			origin[axis0] = _originMeters[axis0] + t * _spanMeters[axis0];

			for (size_t j = 0; j < numJ; j++) {
				const size_t rayIdx = i + numI * j;
				if (rayIdx % numThreads != threadNum)
					continue;

				const double blockSpan = _spanMeters[axis2] / _blockDim[axis2];
				const double cellSpan = blockSpan / Block::getBlockDim();

				double u = j / (numJ - 1.0);
				origin[axis1] = _originMeters[axis1] + u * _spanMeters[axis1];

				Ray ray(origin, rayDir);
				vector<RayHit> hits;
				if (pTriMesh->rayCast(ray, hits)) {
					RayBlockIntersectVec* pBlockHits = nullptr;
					{
						lock_guard<mutex> lock(allHitsLock);
						auto iterX = allHits.find(i);
						if (iterX == allHits.end()) {
							iterX = allHits.insert(make_pair(i, map<size_t, RayBlockIntersectVec>())).first;
						}

						map<size_t, RayBlockIntersectVec>& subMap = iterX->second;
						auto iterY = subMap.find(j);
						if (iterY == subMap.end()) {
							iterY = subMap.insert(make_pair(j, RayBlockIntersectVec())).first;
						}
						pBlockHits = &(iterY->second);
					}
					auto& blockHits = *pBlockHits;
					if (blockHits.empty())
						blockHits.resize(1);
					RayTriIntersectVec& rayHits = blockHits.front();

					for (const auto& triHit : hits) {
						double dist0 = triHit.dist;
						if (dist0 < 0)
							dist0 = 0;
						else if (dist0 >= _spanMeters[axis2])
							dist0 = _spanMeters[axis2];

						RayTriIntersect rti;
						rti._triIdx = triHit.triIdx;

						double w0 = dist0 / _spanMeters[axis2];
							
						rti._blockIdx = (size_t)(w0 * _blockDim[axis2]);
						if (rti._blockIdx >= _blockDim[axis2])
							rti._blockIdx = _blockDim[axis2] - 1;
						double dist1 = dist0 - (rti._blockIdx * blockSpan);

						double w1 = dist1 / blockSpan;
						rti._cellIdx = (size_t)(w1 * Block::getBlockDim());
						if (rti._cellIdx >= Block::getBlockDim())
							rti._cellIdx = Block::getBlockDim() - 1;

						double dist2 = dist1 - (rti._cellIdx * cellSpan);

						rti._w = dist2 / cellSpan;
						rayHits.push_back(rti);
					}
				}
			}
		}
	}, RUN_MULTI_THREAD);

	size_t numHits = 0;
	for (const auto& pair : allHits) {
		numHits += pair.second.size();
	}
	cout << "numHits: " << numHits << "\n";

	const size_t axis0 = axisOrder[0];
	const size_t axis1 = axisOrder[1];
	const size_t axis2 = axisOrder[2];

	if (blocksToCreate.empty())
		blocksToCreate.resize(_blockDim[0] * _blockDim[1] * _blockDim[2], false);

	for (size_t i = 0; i < _blockDim[axis0]; i++) {
		const auto& iterI = allHits.find(i);
		if (iterI != allHits.end()) {
			const auto& jMap = iterI->second;
			for (size_t j = 0; j < _blockDim[axis0]; j++) {
				const auto& iterK = jMap.find(j);
				if (iterK != jMap.end()) {
					const RayBlockIntersectVec& kBlockHits = iterK->second;
					if (!kBlockHits.empty()) {
						const RayTriIntersectVec& kHits = kBlockHits.front();
						for (const auto& kHit : kHits) {
							size_t k = kHit._blockIdx;
							for (int di = -1; di < 2; di++) {
								size_t ii = i + di;
								if (ii >= _blockDim[axis0])
									continue;

								for (int dj = -1; dj < 2; dj++) {
									size_t jj = j + dj;
									if (jj >= _blockDim[axis1])
										continue;

									Vector3i idxIn(ii, jj, k), blockIndex;
									assignAxes(idxIn, axisOrder, blockIndex);
									size_t bIdx = calLinearBlockIndex(blockIndex);
									if (bIdx < blocksToCreate.size())
										blocksToCreate[bIdx] = true;
									else
										cout << "block index out  of bounds\n";
								}

							}
						}
					}
				}
			}
		}

	}


}

CMeshPtr Volume::buildCFDHexes(const CMeshPtr& pTriMesh, double minCellSize, const Vector3d& emptyVolRatio)
{
	CMesh::BoundingBox bb = pTriMesh->getBBox();
	double growDist = -DBL_MAX;
	auto range = bb.range();
	for (int i = 0; i < 3; i++) {
		if (range[i] > growDist)
			growDist = range[i];
	}
	growDist *= 0.05;
	bb.grow(growDist);
	_originMeters = bb.getMin();
	_spanMeters = bb.range();

	Index3 blockSize(
		(size_t)(_spanMeters[0] / minCellSize / Block::getBlockDim() + 0.5),
		(size_t)(_spanMeters[1] / minCellSize / Block::getBlockDim() + 0.5),
		(size_t)(_spanMeters[2] / minCellSize / Block::getBlockDim() + 0.5)
	);

	setBlockDims(blockSize);

	vector<bool> blocksToCreate;
//	rayCastAxis(pTriMesh, AxisIndex::X, blocksToCreate);
	rayCastAxis(pTriMesh, AxisIndex::Y, blocksToCreate);
//	rayCastAxis(pTriMesh, AxisIndex::Z, blocksToCreate);

	MultiCore::runLambda([this, pTriMesh, &blocksToCreate](size_t threadNum, size_t numThreads) {
		Vector3d blockSpan;
		for (int i = 0; i < 3; i++) {
			blockSpan[i] = _spanMeters[i] / (double)_blockDim[i];
		}
		for (size_t i = threadNum; i < blocksToCreate.size(); i += numThreads) {
			if (blocksToCreate[i]) {
				_blocks[i] = _blockPool.create();
			}
		}
	}, RUN_MULTI_THREAD); 

	CMeshPtr result = makeTris(true);

	return result;
}

CMeshPtr Volume::makeTris(bool cells)
{
	CBoundingBox3Dd bbox;
	bbox.merge(_originMeters);
	bbox.merge(_originMeters + _spanMeters);
	auto diagDist = bbox.range().norm();
	bbox.grow(diagDist * 0.05);

	vector<CMeshPtr> results;
	for (size_t i = 0; i < MultiCore::getNumCores(); i++) {
		results.push_back(make_shared<CMesh>());
		results.back()->reset(bbox);
	}

	Vector3d blockSpan;
	for (int i = 0; i < 3; i++)
		blockSpan[i] = _spanMeters[i] / _blockDim[i];

//	for (blockIdx[0] = 0; blockIdx[0] < _blockDim[0]; blockIdx[0]++) {
	MultiCore::runLambda([this, &blockSpan, &results, cells](size_t threadNum, size_t numThreads) {
		Vector3d blockOrigin;
		auto result = results[threadNum];

		Vector3i blockIdx;
		for (blockIdx[0] = threadNum; blockIdx[0] < _blockDim[0]; blockIdx[0] += numThreads) {
			blockOrigin[0] = _originMeters[0] + blockIdx[0] * blockSpan[0];
			for (blockIdx[1] = 0; blockIdx[1] < _blockDim[1]; blockIdx[1]++) {
				blockOrigin[1] = _originMeters[1] + blockIdx[1] * blockSpan[1];
				for (blockIdx[2] = 0; blockIdx[2] < _blockDim[2]; blockIdx[2]++) {
					blockOrigin[2] = _originMeters[2] + blockIdx[2] * blockSpan[2];
					Block* pBlock = getBlock(blockIdx);
					if (pBlock) {
						pBlock->addBlockTris(blockOrigin, blockSpan, result, cells);
					}
				}
			}
		}
	}, RUN_MULTI_THREAD);

	CMeshPtr result = make_shared<CMesh>();
	result->merge(results);
	cout << "Num tris: " << result->numTris();
	cout << "Num cells: " << result->numTris() / 12;
	return result;
}

