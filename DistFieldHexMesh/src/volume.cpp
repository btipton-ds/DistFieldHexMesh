

#include <triMesh.h>

#include <cell.h>
#include <block.h>
#include <volume.h>
#include <MultiCoreUtil.h>

using namespace std;
using namespace DFHM;
using namespace TriMesh;

#define RUN_MULTI_THREAD true

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

void Volume::scanVolumePlaneCreateBlocksWhereNeeded(const CMeshPtr& pTriMesh, std::vector<bool>& blocksToCreate, const Vector3i& axisOrder)
{
	if (blocksToCreate.empty())
		blocksToCreate.resize(_blocks.size());
	MultiCore::runLambda([this, pTriMesh, &blocksToCreate, axisOrder](size_t threadNum, size_t numThreads) {
		const int overFlow = 1;
		Vector3d axis;

		switch (axisOrder[2]) {
		case 0:
			axis = Vector3d(1, 0, 0);
			break;
		case 1:
			axis = Vector3d(0, 1, 0);
			break;
		case 2:
			axis = Vector3d(0, 0, 1);
			break;
		}

		Vector3 blockRange;
		const auto& volumeRange = pTriMesh->getBBox().range();
		for (int i = 0; i < 3; i++) {
			blockRange[i] = volumeRange[i] / _blockDim[i];
		}
		size_t axisIdx0 = axisOrder[0];
		size_t axisIdx1 = axisOrder[1];
		size_t axisIdx2 = axisOrder[2];

		Eigen::Vector3d offset(0, 0, 0);

		for (size_t i = threadNum; i < _blockDim[axisIdx0]; i += numThreads) {
			offset[axisIdx0] = i * blockRange[axisIdx0];

			for (size_t j = 0; j < _blockDim[axisIdx1]; j++) {
				offset[axisIdx1] = j * blockRange[axisIdx1];

				Eigen::Vector3d rayOrigin = _originMeters + offset;

				Ray ray(rayOrigin, axis);
				// We need to support solids and surfaces
				// Solids will always have crossing in pairs, except at tangential crossings
				// Surface crossings are singular
				// This sorts them out
				vector<RayHit> hits;
				if (pTriMesh->rayCast(ray, hits, false)) {
					for (const auto& hit : hits) {
//						cout << "x: " << (offset[axisIdx0] / volumeRange[axisIdx0]) << ", y: " << (offset[axisIdx1] / volumeRange[axisIdx1]) << ", z: " << (hit.dist / volumeRange[axisIdx2]) << "\n";

						double tk = hit.dist / _spanMeters[axisIdx2];
						size_t k = (size_t)(_blockDim[axisIdx2] * tk);
						if (k < 0)
							k = 0;
						if (k >= _blockDim[axisIdx2])
							k = _blockDim[axisIdx2] - 1;
						{
							size_t ix, iy, iz;
							switch (axisOrder[0]) {
								case 0: ix = i; break;
								case 1: iy = i; break;
								case 2: iz = i; break;
							}

							switch (axisOrder[1]) {
								case 0: ix = j; break;
								case 1: iy = j; break;
								case 2: iz = j; break;
							}

							switch (axisOrder[2]) {
								case 0: ix = k; break;
								case 1: iy = k; break;
								case 2: iz = k; break;
							}

							size_t iix = ix, iiy = iy, iiz = iz;
							for (int ii = -overFlow; ii <= overFlow; ii++) {
								switch (axisOrder[0]) {
									case 0: iix = ix + ii; break;
									case 1: iiy = iy + ii; break;
									case 2: iiz = iz + ii; break;
								}

								for (int jj = -overFlow; jj <= overFlow; jj++) {
									switch (axisOrder[1]) {
										case 0: iix = ix + jj; break;
										case 1: iiy = iy + jj; break;
										case 2: iiz = iz + jj; break;
									}

									for (int kk = -overFlow; kk <= overFlow; kk++) {
										switch (axisOrder[2]) {
											case 0: iix = ix + kk; break;
											case 1: iiy = iy + kk; break;
											case 2: iiz = iz + kk; break;
										}

										size_t bIdx = calLinearBlockIndex(iix, iiy, iiz);
										// No mutex required, because order of setting true is not a race condition.
										if (bIdx != -1)
											blocksToCreate[bIdx] = true;
									}
								}
							}	
						}
					}

				}
			}
		}
	}, RUN_MULTI_THREAD);

}

CMeshPtr Volume::buildCFDHexes(const CMeshPtr& pTriMesh, double minCellSize, const Vector3& emptyVolRatio)
{
	CMesh::BoundingBox bb = pTriMesh->getBBox();
	_originMeters = bb.getMin();
	_spanMeters = bb.range();

	Index3 blockSize(
		(size_t)(_spanMeters[0] / minCellSize / Block::getBlockDim() + 0.5),
		(size_t)(_spanMeters[1] / minCellSize / Block::getBlockDim() + 0.5),
		(size_t)(_spanMeters[2] / minCellSize / Block::getBlockDim() + 0.5)
	);

	setBlockDims(blockSize);
	vector<bool> blocksToCreate;

	// Scan the model in from all three orthoganal planes and create all intersecting blocks
	scanVolumePlaneCreateBlocksWhereNeeded(pTriMesh, blocksToCreate, Vector3i(0, 1, 2));
	scanVolumePlaneCreateBlocksWhereNeeded(pTriMesh, blocksToCreate, Vector3i(2, 0, 1));
	scanVolumePlaneCreateBlocksWhereNeeded(pTriMesh, blocksToCreate, Vector3i(1, 2, 0));

	MultiCore::runLambda([this, pTriMesh, &blocksToCreate](size_t threadNum, size_t numThreads) {
		Vector3 blockSpan;
		for (int i = 0; i < 3; i++) {
			blockSpan[i] = _spanMeters[i] / (double)_blockDim[i];
		}
		for (size_t i = threadNum; i < blocksToCreate.size(); i += numThreads) {
			if (blocksToCreate[i]) {
				size_t ix = i % _blockDim[0];
				size_t tempI = i / _blockDim[0];
				size_t iy = tempI % _blockDim[1];
				tempI = tempI / _blockDim[1];
				size_t iz = tempI % _blockDim[2];

				Vector3 blockOrigin(_originMeters);
				blockOrigin += Vector3(ix * blockSpan[0], iy * blockSpan[1], iz * blockSpan[2]);

				Block* pBlock;
				_blocks[i] = _blockPool.getObj(i, pBlock, true);
				vector<bool> cellsToCreate;
				size_t bd = Block::getBlockDim();
				cellsToCreate.resize(bd * bd * bd);

				bool changed = pBlock->scanCreateCellsWhereNeeded(pTriMesh, blockOrigin, blockSpan, cellsToCreate, Vector3i(0, 1, 2));
				changed = pBlock->scanCreateCellsWhereNeeded(pTriMesh, blockOrigin, blockSpan, cellsToCreate, Vector3i(2, 0, 1)) || changed;
				changed = pBlock->scanCreateCellsWhereNeeded(pTriMesh, blockOrigin, blockSpan, cellsToCreate, Vector3i(1, 2, 0)) || changed;

				if (!changed) {
					_blockPool.free(_blocks[i]);
					_blocks[i] = -1;
				} else {
					pBlock->createCells(cellsToCreate);
				}
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

