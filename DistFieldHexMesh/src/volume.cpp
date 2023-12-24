
#include <sstream>
#include <fstream>

#include <triMesh.h>

#include <cell.h>
#include <block.h>
#include <volume.h>
#include <MultiCoreUtil.h>
#include <vertex.h>
#include <filesystem>
using namespace std;
using namespace DFHM;
using namespace TriMesh;

#define RUN_MULTI_THREAD true

namespace
{

inline Vector3i assignAxes(const Vector3i& indexIn, const Vector3i& axisOrder)
{
	Vector3i result;
	result[axisOrder[0]] = indexIn[0];
	result[axisOrder[1]] = indexIn[1];
	result[axisOrder[2]] = indexIn[2];
	return result;
}

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

bool Volume::cellExists(size_t ix, size_t iy, size_t iz) const
{
	size_t blkDim = Block::getBlockDim();
	Vector3i idx(ix, iy, iz);
	Vector3i blockIdx, cellIdx;
	for (int i = 0; i < 3; i++) {
		blockIdx[i] = idx[i] / blkDim;
		cellIdx[i] = idx[i] % blkDim;
	}
	if (!blockExists(blockIdx))
		return false;
	const auto& block = getBlock(blockIdx);
	return block.cellExists(cellIdx);
}

bool Volume::cellExists(const Vector3i& blockIdx) const
{
	return cellExists(blockIdx[0], blockIdx[1], blockIdx[2]);
}

Cell& Volume::getCell(size_t ix, size_t iy, size_t iz)
{
	size_t blkDim = Block::getBlockDim();
	Vector3i idx(ix, iy, iz);
	Vector3i blockIdx, cellIdx;
	for (int i = 0; i < 3; i++) {
		blockIdx[i] = idx[i] / blkDim;
		cellIdx[i] = idx[i] % blkDim;
	}
	auto& block = getBlock(blockIdx);
	auto& cell = block.getCell(cellIdx);
	return cell;
}

const Cell& Volume::getCell(size_t ix, size_t iy, size_t iz) const
{
	size_t blkDim = Block::getBlockDim();
	Vector3i idx(ix, iy, iz);
	Vector3i blockIdx, cellIdx;
	for (int i = 0; i < 3; i++) {
		blockIdx[i] = idx[i] / blkDim;
		cellIdx[i] = idx[i] % blkDim;
	}
	auto& block = getBlock(blockIdx);
	auto& cell = block.getCell(cellIdx);
	return cell;
}

void Volume::processRayHit(const RayHit& triHit, int rayAxis, const Vector3d& blockSpan, const Vector3d& cellSpan, size_t& blockIdx, size_t& cellIdx)
{

	double dist0 = triHit.dist;
	if (dist0 < 0)
		dist0 = 0;
	else if (dist0 >= _spanMeters[rayAxis])
		dist0 = _spanMeters[rayAxis];

	double w0 = dist0 / _spanMeters[rayAxis];

	blockIdx = (size_t)(w0 * _blockDim[rayAxis]);
	if (blockIdx >= _blockDim[rayAxis])
		blockIdx = _blockDim[rayAxis] - 1;
	double dist1 = dist0 - (blockIdx * blockSpan[rayAxis]);

	double w1 = dist1 / blockSpan[rayAxis];
	cellIdx = (size_t)(w1 * Block::getBlockDim());
	if (cellIdx >= Block::getBlockDim())
		cellIdx = Block::getBlockDim() - 1;
}

void Volume::createBlockRays(AxisIndex axisIdx, const TriMesh::CMeshPtr& pTriMesh, vector<bool>& blocksToCreate)
{
	const size_t bd = Block::getBlockDim();
	Vector3d blockSpan, cellSpan;
	for (int i = 0; i < 3; i++) {
		blockSpan[i] = _spanMeters[i] / _blockDim[i];
		cellSpan[i] = blockSpan[i] / bd;
	}

	switch (axisIdx) {
		case AxisIndex::X: {
			MultiCore::runLambda([this, pTriMesh, blockSpan, cellSpan, &blocksToCreate](size_t threadNum, size_t numThreads) {
				const Vector3d rayDir(1, 0, 0);
				const size_t numY = _blockDim[1] + 1;
				const size_t numZ = _blockDim[2] + 1;

				Vector3d origin(_originMeters);

				for (size_t iy = 0; iy < numY; iy++) {
					origin[1] = _originMeters[1] + iy * blockSpan[1];

					for (size_t iz = 0; iz < numZ; iz++) {
						const size_t rayIdx = iy + numY * iz;
						if (rayIdx % numThreads != threadNum)
							continue;

						origin[2] = _originMeters[2] + iz * blockSpan[2];

						Ray ray(origin, rayDir);
						vector<RayHit> hits;
						if (pTriMesh->rayCast(ray, hits)) {
							for (const auto& triHit : hits) {
								size_t blockIdx, cellIdx;
								processRayHit(triHit, 0, blockSpan, cellSpan, blockIdx, cellIdx);
								size_t ix = blockIdx;
								for (int dy = -1; dy < 2; dy++) {
									for (int dz = -1; dz < 2; dz++) {
										size_t bIdx = calLinearBlockIndex(Vector3i(ix, iy + dy, iz + dz));
										if (bIdx < blocksToCreate.size())
											blocksToCreate[bIdx] = true;
									}
								}
							}
						}
					}
				}
			}, RUN_MULTI_THREAD);
			break;
		}
		case AxisIndex::Y: {
			MultiCore::runLambda([this, pTriMesh, blockSpan, cellSpan, &blocksToCreate](size_t threadNum, size_t numThreads) {
				Vector3d rayDir(0, 1, 0);
				const size_t numX = _blockDim[0] + 1;
				const size_t numZ = _blockDim[2] + 1;

				Vector3d origin(_originMeters);

				for (size_t ix = 0; ix < numX; ix++) {
					origin[0] = _originMeters[0] + ix * blockSpan[0];

					for (size_t iz = 0; iz < numZ; iz++) {
						const size_t rayIdx = ix + numX * iz;
						if (rayIdx % numThreads != threadNum)
							continue;

						origin[2] = _originMeters[2] + iz * blockSpan[2];

						Ray ray(origin, rayDir);
						vector<RayHit> hits;
						if (pTriMesh->rayCast(ray, hits)) {
							for (const auto& triHit : hits) {
								size_t blockIdx, cellIdx;
								processRayHit(triHit, 1, blockSpan, cellSpan, blockIdx, cellIdx);
								size_t iy = blockIdx;
								for (int dx = -1; dx < 2; dx++) {
									for (int dz = -1; dz < 2; dz++) {
										size_t bIdx = calLinearBlockIndex(Vector3i(ix + dx, iy, iz + dz));
										if (bIdx < blocksToCreate.size())
											blocksToCreate[bIdx] = true;
									}
								}
							}
						}
					}
				}
			}, RUN_MULTI_THREAD);
			break;
		}
		case AxisIndex::Z: {
			MultiCore::runLambda([this, pTriMesh, blockSpan, cellSpan, &blocksToCreate](size_t threadNum, size_t numThreads) {
				Vector3d rayDir(0, 0, 1);
				const size_t numX = _blockDim[0] + 1;
				const size_t numY = _blockDim[1] + 1;

				Vector3d origin(_originMeters);

				for (size_t ix = 0; ix < numX; ix++) {
					origin[0] = _originMeters[0] + ix * blockSpan[0];

					for (size_t iy = 0; iy < numY; iy++) {
						const size_t rayIdx = ix + numX * iy;
						if (rayIdx % numThreads != threadNum)
							continue;

						origin[1] = _originMeters[1] + iy * blockSpan[1];

						Ray ray(origin, rayDir);
						vector<RayHit> hits;
						if (pTriMesh->rayCast(ray, hits)) {
							for (const auto& triHit : hits) {
								size_t blockIdx, cellIdx;
								processRayHit(triHit, 2, blockSpan, cellSpan, blockIdx, cellIdx);
								size_t iz = blockIdx;
								for (int dx = -1; dx < 2; dx++) {
									for (int dy = -1; dy < 2; dy++) {
										size_t bIdx = calLinearBlockIndex(Vector3i(ix + dx, iy + dy, iz));
										if (bIdx < blocksToCreate.size())
											blocksToCreate[bIdx] = true;
									}
								}
							}
						}
					}
				}
			}, RUN_MULTI_THREAD);
			break;
		}
	}
}

void Volume::createBlockCellRays(AxisIndex axisIdx, const TriMesh::CMeshPtr& pTriMesh, const vector<bool>& blocksToCreate, vector<vector<bool>>& cellsToCreate)
{
	const size_t bd = Block::getBlockDim();
	Vector3d blockSpan, cellSpan;
	for (int i = 0; i < 3; i++) {
		blockSpan[i] = _spanMeters[i] / _blockDim[i];
		cellSpan[i] = blockSpan[i] / bd;
	}

	switch (axisIdx) {
		case AxisIndex::X: {
			MultiCore::runLambda([this, pTriMesh, blockSpan, cellSpan, &blocksToCreate, &cellsToCreate](size_t threadNum, size_t numThreads) {
				const Vector3d rayDir(1, 0, 0);
				const size_t bd = Block::getBlockDim();
				const size_t numBlockCells = bd * bd * bd;

				const size_t numSteps = bd + 1;
				Vector3d blockOrigin(_originMeters);

				for (size_t iyBlk = 0; iyBlk < _blockDim[1]; iyBlk++) {
					blockOrigin[1] = _originMeters[1] + iyBlk * blockSpan[1];

					for (size_t izBlk = 0; izBlk < _blockDim[2]; izBlk++) {
						size_t threadIdx = iyBlk + izBlk * _blockDim[1];
						if (threadIdx % numThreads != threadNum)
							continue;

						bool skipBlock = true;
						for (size_t ixBlk = 0; ixBlk < _blockDim[0]; ixBlk++) {
							size_t bIdx = calLinearBlockIndex(ixBlk, iyBlk, izBlk);
							if (blocksToCreate[bIdx]) {
								skipBlock = false;
								break;
							}
						}
						if (skipBlock)
							continue;

						blockOrigin[2] = _originMeters[2] + izBlk * blockSpan[2];

						Vector3d rayOrigin(blockOrigin);
						for (size_t iyCell = 0; iyCell < numSteps; iyCell++) {
							rayOrigin[1] = blockOrigin[1] + iyCell * cellSpan[1];

							for (size_t izCell = 0; izCell < numSteps; izCell++) {
								rayOrigin[2] = blockOrigin[2] + izCell * cellSpan[2];

								Ray ray(rayOrigin, rayDir);
								vector<RayHit> hits;
								if (pTriMesh->rayCast(ray, hits)) {
									for (const auto& triHit : hits) {
										size_t blockIdx, cellIdx;
										processRayHit(triHit, 0, blockSpan, cellSpan, blockIdx, cellIdx);

										size_t bIdx = calLinearBlockIndex(blockIdx, iyBlk, izBlk);
										if (bIdx < cellsToCreate.size()) {
											vector<bool>& blockCellsToCreate = cellsToCreate[bIdx];
											if (blockCellsToCreate.empty())
												blockCellsToCreate.resize(numBlockCells);

											for (int dy = -1; dy < 2; dy++) {
												for (int dz = -1; dz < 2; dz++) {
													size_t cIdx = Block::calcCellIndex(Vector3i(cellIdx, iyCell + dy, izCell + dz));
													if (cIdx < blockCellsToCreate.size()) {
														blockCellsToCreate[cIdx] = true;
													}
												}
											}
										}
									}
								}
							}
						}
					}
				}
			}, RUN_MULTI_THREAD);
			break;
		}
		case AxisIndex::Y: {
			MultiCore::runLambda([this, pTriMesh, blockSpan, cellSpan, &blocksToCreate, &cellsToCreate](size_t threadNum, size_t numThreads) {
				const Vector3d rayDir(0, 1, 0);
				const size_t bd = Block::getBlockDim();
				const size_t numBlockCells = bd * bd * bd;

				const size_t numSteps = bd + 1;
				Vector3d blockOrigin(_originMeters);

				for (size_t ixBlk = 0; ixBlk < _blockDim[0]; ixBlk++) {
					blockOrigin[0] = _originMeters[0] + ixBlk * blockSpan[0];

					for (size_t izBlk = 0; izBlk < _blockDim[2]; izBlk++) {
						size_t threadIdx = ixBlk + izBlk * _blockDim[2];
						if (threadIdx % numThreads != threadNum)
							continue;

						bool skipBlock = true;
						for (size_t iyBlk = 0; iyBlk < _blockDim[1]; iyBlk++) {
							size_t bIdx = calLinearBlockIndex(ixBlk, iyBlk, izBlk);
							if (blocksToCreate[bIdx]) {
								skipBlock = false;
								break;
							}
						}
						if (skipBlock)
							continue;

						blockOrigin[2] = _originMeters[2] + izBlk * blockSpan[2];

						Vector3d rayOrigin(blockOrigin);
						for (size_t ixCell = 0; ixCell < numSteps; ixCell++) {
							rayOrigin[0] = blockOrigin[0] + ixCell * cellSpan[0];

							for (size_t izCell = 0; izCell < numSteps; izCell++) {
								rayOrigin[2] = blockOrigin[2] + izCell * cellSpan[2];

								Ray ray(rayOrigin, rayDir);
								vector<RayHit> hits;
								if (pTriMesh->rayCast(ray, hits)) {
									for (const auto& triHit : hits) {
										size_t blockIdx, cellIdx;
										processRayHit(triHit, 1, blockSpan, cellSpan, blockIdx, cellIdx);

										size_t bIdx = calLinearBlockIndex(ixBlk, blockIdx, izBlk);
										if (bIdx < cellsToCreate.size()) {
											vector<bool>& blockCellsToCreate = cellsToCreate[bIdx];
											if (blockCellsToCreate.empty())
												blockCellsToCreate.resize(numBlockCells);

											for (int dx = -1; dx < 2; dx++) {
												for (int dz = -1; dz < 2; dz++) {
													size_t cIdx = Block::calcCellIndex(Vector3i(ixCell + dx, cellIdx, izCell + dz));
													if (cIdx < blockCellsToCreate.size()) {
														blockCellsToCreate[cIdx] = true;
													}
												}
											}
										}
									}
								}
							}
						}
					}
				}
			}, RUN_MULTI_THREAD);
			break;
		}
		case AxisIndex::Z: {
			MultiCore::runLambda([this, pTriMesh, blockSpan, cellSpan, &blocksToCreate, &cellsToCreate](size_t threadNum, size_t numThreads) {
				const Vector3d rayDir(0, 0, 1);
				const size_t bd = Block::getBlockDim();
				const size_t numBlockCells = bd * bd * bd;

				const size_t numSteps = bd + 1;
				Vector3d blockOrigin(_originMeters);

				for (size_t ixBlk = 0; ixBlk < _blockDim[0]; ixBlk++) {
					blockOrigin[0] = _originMeters[0] + ixBlk * blockSpan[0];

					for (size_t iyBlk = 0; iyBlk < _blockDim[1]; iyBlk++) {
						size_t threadIdx = ixBlk + iyBlk * _blockDim[0];
						if (threadIdx % numThreads != threadNum)
							continue;

						bool skipBlock = true;
						for (size_t izBlk = 0; izBlk < _blockDim[2]; izBlk++) {
							size_t bIdx = calLinearBlockIndex(ixBlk, iyBlk, izBlk);
							if (blocksToCreate[bIdx]) {
								skipBlock = false;
								break;
							}
						}
						if (skipBlock)
							continue;

						blockOrigin[1] = _originMeters[1] + iyBlk * blockSpan[1];

						Vector3d rayOrigin(blockOrigin);
						for (size_t ixCell = 0; ixCell < numSteps; ixCell++) {
							rayOrigin[0] = blockOrigin[0] + ixCell * cellSpan[0];

							for (size_t iyCell = 0; iyCell < numSteps; iyCell++) {
								rayOrigin[1] = blockOrigin[1] + iyCell * cellSpan[1];

								Ray ray(rayOrigin, rayDir);
								vector<RayHit> hits;
								if (pTriMesh->rayCast(ray, hits)) {
									for (const auto& triHit : hits) {
										size_t blockIdx, cellIdx;
										processRayHit(triHit, 2, blockSpan, cellSpan, blockIdx, cellIdx);

										size_t bIdx = calLinearBlockIndex(ixBlk, iyBlk, blockIdx);
										if (bIdx < cellsToCreate.size()) {
											vector<bool>& blockCellsToCreate = cellsToCreate[bIdx];
											if (blockCellsToCreate.empty())
												blockCellsToCreate.resize(numBlockCells);

											for (int dx = -1; dx < 2; dx++) {
												for (int dy = -1; dy < 2; dy++) {
													size_t cIdx = Block::calcCellIndex(Vector3i(ixCell + dx, iyCell + dy, cellIdx));
													if (cIdx < blockCellsToCreate.size()) {
														blockCellsToCreate[cIdx] = true;
													}
												}
											}
										}
									}
								}
							}
						}
					}
				}
			}, RUN_MULTI_THREAD);
			break;
		}
	}
}
Block& Volume::addBlock(size_t ix, size_t iy, size_t iz, size_t threadNum)
{
	size_t idx = calLinearBlockIndex(ix, iy, iz);
	ObjectPoolId poolId = _blocks[idx];
	if (poolId == -1) {
		poolId = ObjectPoolId(-1, threadNum);
		poolId = _blocks[idx] = _blockPool.add(Block(), poolId);
	}
	return _blockPool[poolId];
}

Block& Volume::addBlock(const Vector3i& blockIdx, size_t threadNum)
{
	return addBlock(blockIdx[0], blockIdx[1], blockIdx[2], threadNum);
}

bool Volume::blockExists(size_t ix, size_t iy, size_t iz) const
{
	size_t idx = calLinearBlockIndex(ix, iy, iz);
	if (idx >= _blocks.size())
		return false;
	ObjectPoolId poolId = _blocks[idx];
	return _blockPool.idExists(poolId);
}

bool Volume::blockExists(const Vector3i& blockIdx) const
{
	return blockExists(blockIdx[0], blockIdx[1], blockIdx[2]);
}

const Block& Volume::getBlock(size_t ix, size_t iy, size_t iz) const
{
	lock_guard<mutex> lock(_mutex);

	size_t idx = calLinearBlockIndex(ix, iy, iz);
	if (idx < _blocks.size()) {
		ObjectPoolId poolId = _blocks[idx];
		if (!_blockPool.idExists(poolId))
			throw exception("Volume::getBlock block not allocated");
		return _blockPool[poolId];
	}
	throw exception("Volume::getBlock index out of range");
}

Block& Volume::getBlock(size_t ix, size_t iy, size_t iz)
{
	lock_guard<mutex> lock(_mutex);

	size_t idx = calLinearBlockIndex(ix, iy, iz);
	if (idx < _blocks.size()) {
		ObjectPoolId poolId = _blocks[idx];
		if (!_blockPool.idExists(poolId))
			throw exception("Volume::getBlock not allocated");
		return _blockPool[poolId];
	}
	throw exception("Volume::getBlock index out of range");
}
CMeshPtr Volume::buildCFDHexes(const CMeshPtr& pTriMesh, double minCellSize, const Vector3d& emptyVolRatio)
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
	blocksToCreate.resize(_blockDim[0] * _blockDim[1] * _blockDim[2]);

	createBlockRays(AxisIndex::X, pTriMesh, blocksToCreate);
	createBlockRays(AxisIndex::Y, pTriMesh, blocksToCreate);
	createBlockRays(AxisIndex::Z, pTriMesh, blocksToCreate);

	vector<vector<bool>> cellsToCreate;
	cellsToCreate.resize(_blockDim[0] * _blockDim[1] * _blockDim[2]);
	createBlockCellRays(AxisIndex::X, pTriMesh, blocksToCreate, cellsToCreate);
	createBlockCellRays(AxisIndex::Y, pTriMesh, blocksToCreate, cellsToCreate);
	createBlockCellRays(AxisIndex::Z, pTriMesh, blocksToCreate, cellsToCreate);

	MultiCore::runLambda([this, pTriMesh, &cellsToCreate](size_t threadNum, size_t numThreads) {
		Vector3i blockIdx;
		for (blockIdx[0] = threadNum; blockIdx[0] < _blockDim[0]; blockIdx[0] += numThreads) {
			for (blockIdx[1] = 0; blockIdx[1] < _blockDim[1]; blockIdx[1]++) {
				for (blockIdx[2] = 0; blockIdx[2] < _blockDim[2]; blockIdx[2]++) {

					size_t bIdx = calLinearBlockIndex(blockIdx);
					if (!cellsToCreate[bIdx].empty()) {
						Block& block = addBlock(blockIdx, threadNum);
						block.createCells(cellsToCreate[bIdx], threadNum);
					}
				}
			}
		}
	}, RUN_MULTI_THREAD);

	CMeshPtr result = makeTris(false);

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
					if (blockExists(blockIdx)) {
						const Block& block = getBlock(blockIdx);
						ObjectPoolId blockId = _blocks[calLinearBlockIndex(blockIdx)];
						block.addBlockTris(blockId, blockOrigin, blockSpan, result, cells);
					}
				}
			}
		}
	}, RUN_MULTI_THREAD);

	CMeshPtr result = results.back();
	results.pop_back();
	result->merge(results, true);
	cout << "Num tris: " << result->numTris();
	cout << "Num cells: " << result->numTris() / 12;
	return result;
}

namespace {
	void appendDir(string& dirName, const string& str)
	{
		if (dirName.find_last_of("/") != dirName.length() - 1)
			dirName += "/";
		dirName += str;
	}
}

void Volume::writePolyMesh(const string& dirNameIn) const
{

	string dirName(dirNameIn);
	if (dirName.find("constant") == string::npos)
		appendDir(dirName, "constant");
	if (dirName.find("polyMesh") == string::npos)
		appendDir(dirName, "polyMesh");
	
	filesystem::path dirPath(dirName);
	filesystem::remove_all(dirPath);
	filesystem::create_directories(dirPath);
	writePolyMeshPoints(dirName);
}

void Volume::writePolyMeshPoints(const std::string& dirName) const
{
	ofstream out(dirName + "/points", ios_base::binary);
	writeFOAMHeader(out, "vectorField", "points");
	size_t numPoints = 0;
	_vertexPool.iterateInOrder([&numPoints](const Vertex& vert) {
		numPoints++;
	});
	out << numPoints << "\n";
	_vertexPool.iterateInOrder([&out, &numPoints](const Vertex& vert) {
		char openParen = '(';
		char closeParen = ')';

		const double& x = vert.getPoint()[0];
		const double& y = vert.getPoint()[1];
		const double& z = vert.getPoint()[2];

		out.write(&openParen, 1);
		out.write((char*)&x, sizeof(x));
		out.write((char*)&y, sizeof(y));
		out.write((char*)&z, sizeof(z));
		out.write(&closeParen, 1);
	});
}

void Volume::writeFOAMHeader(std::ofstream& out, const std::string& foamClass, const std::string& object) const
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

void Volume::dumpSections(const string& dirName) const
{
	const size_t bd = Block::getBlockDim();
	size_t numX = _blockDim[0] * bd;
	size_t numY = _blockDim[1] * bd;
	size_t numZ = _blockDim[2] * bd;

	for (size_t iz = 0; iz < numZ; iz++) {
		stringstream ss;
		ss << dirName << "/dump" << to_string(iz) << ".txt";
		ofstream o(ss.str());
		for (size_t iy = 0; iy < numY; iy++) {
			if (iy % bd == (bd - 1)) {
				for (size_t ix = 0; ix < numX; ix++)
					o << "_";
				o << "\n";
			}

			for (size_t ix = 0; ix < numX; ix++) {
				if (ix % bd == (bd - 1))
					o << "|";

				if (cellExists(ix, iy, iz))
					o << "X";
				else
					o << " ";
			}
			o << "\n";
		}
	}
}
