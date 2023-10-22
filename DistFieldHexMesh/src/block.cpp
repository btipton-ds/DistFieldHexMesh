#include <vector>
#include <cell.h>
#include <block.h>
#include <fstream>

using namespace std;
using namespace TriMesh;
using namespace DFHM;

size_t Block::s_blockDim = 8;

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

void Block::setBlockDim(size_t dim)
{
	s_blockDim = dim;
}

size_t Block::getBlockDim()
{
	return s_blockDim;
}

Block::Block()
{
	size_t bd = getBlockDim();
}

Block::Block(const Block& src)
	: _cells(src._cells)
{

}

Block::~Block()
{
}

size_t Block::createCell(Cell*& pCell)
{
	size_t result = -1;
	if (!_freeCells.empty()) {
		result = _freeCells.back();
		_freeCells.pop_back();
		_cellData[result] = {};
	}
	else {
		result = _cellData.size();
		_cellData.push_back(Cell());
	}

	pCell = &_cellData[result];

	return result;
}

Cell* Block::getCell(size_t ix, size_t iy, size_t iz, bool create)
{
	Cell* result = nullptr;
	if (create)
		fillEmpty();
	size_t idx = calcLinearCellIndex(ix, iy, iz);
	if (idx < _cells.size()) {
		if (create && _cells[idx] == -1) {
			_cells[idx] = createCell(result);
		}
		result = &_cellData[_cells[idx]];
	}

	return result;
}

bool Block::scanCreateCellsWhereNeeded(const TriMesh::CMeshPtr& pTriMesh, const Vector3d& origin, const Vector3d& blockSpan, vector<bool>& cellsToCreate, const Vector3i& axisOrder)
{
	static size_t numCells = 0;

	const int overFlow = 1;

	bool result = false;
	Vector3d axis;
	Vector3d cellSpan = blockSpan * (1.0 / s_blockDim);

	Vector3d rayOrigin(origin);
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

	size_t axisIdx0 = axisOrder[0];
	size_t axisIdx1 = axisOrder[1];
	size_t axisIdx2 = axisOrder[2];

	size_t initialNumCells = numCells;
	for (size_t i = 0; i < s_blockDim; i++) {
		rayOrigin[axisIdx0] = origin[axisIdx0] + i * cellSpan[axisIdx0];

		for (size_t j = 0; j < s_blockDim; j++) {
			rayOrigin[axisIdx1] = origin[axisIdx1] + j * cellSpan[axisIdx1];

			Ray ray(rayOrigin, rayOrigin + blockSpan[axisIdx2] * axis);

			vector<RayHit> hits;
			pTriMesh->rayCast(ray, hits);
			if (!hits.empty()) {
				for (const auto hit : hits) {

					double tk = hit.dist / blockSpan[axisIdx2];
					size_t k = (size_t)(s_blockDim * tk);
					if (0 <= k && k < s_blockDim) {
						size_t icx, icy, iz;
						switch (axisOrder[0]) {
						case 0: icx = i; break;
						case 1: icy = i; break;
						case 2: iz = i; break;
						}

						switch (axisOrder[1]) {
						case 0: icx = j; break;
						case 1: icy = j; break;
						case 2: iz = j; break;
						}

						switch (axisOrder[2]) {
							case 0: {
								icx = k;

								for (int iiy = -1; iiy <= 1; iiy++) {
									if (icy + iiy >= s_blockDim)
										continue;
									for (int iiz = -1; iiz <= 1; iiz++) {
										if (iz + iiz >= s_blockDim)
											continue;

										size_t cIdx = calcLinearCellIndex(icx, icy + iiy, iz + iiz);
										if (cIdx < cellsToCreate.size()) {
											result = true;
											cellsToCreate[cIdx] = true;
										}
									}
								}

								break;
							}
							case 1:
							{
								icy = k;

								for (int iix = -1; iix <= 1; iix++) {
									if (icx + iix >= s_blockDim)
										continue;
									for (int iiz = -1; iiz <= 1; iiz++) {
										if (iz + iiz >= s_blockDim)
											continue;

										size_t cIdx = calcLinearCellIndex(icx + iix, icy, iz + iiz);
										if (cIdx < cellsToCreate.size()) {
											result = true;
											cellsToCreate[cIdx] = true;
										}
									}
								}

								break;
							}
							case 2: {
								iz = k;

								for (int iix = -1; iix <= 1; iix++) {
									if (icx + iix >= s_blockDim)
										continue;
									for (int iiy = -1; iiy <= 1; iiy++) {
										if (icy + iiy >= s_blockDim)
											continue;

										size_t cIdx = calcLinearCellIndex(icx + iix, icy + iiy, iz);
										if (cIdx < cellsToCreate.size()) {
											result = true;
											cellsToCreate[cIdx] = true;
										}
									}
								}

								break;
							}
						}
					}
				}

			}
		}
	}

	return result;
}

void Block::createCells(const Vector3d& blockOrigin, const Vector3d& blockSpan, const Vector3i& blockDim, const Vector3i& blockIdx, const vector<bool>& cellsToCreate, const RayHitRec& hits)
{
	if (_cells.empty())
		_cells.resize(s_blockDim * s_blockDim * s_blockDim, -1);
	const Vector3d cellSpan(blockSpan / s_blockDim);

	if (_cells.size() == cellsToCreate.size()) {
		for (size_t i = 0; i < cellsToCreate.size(); i++) {
			if (cellsToCreate[i]) {
				Cell* pCell = nullptr;
				_cells[i] = createCell(pCell);
			}
		}
	}

	addCellHitsX(blockOrigin, cellSpan, blockDim, blockIdx, hits._xIntersects);
	addCellHitsY(blockOrigin, cellSpan, blockDim, blockIdx, hits._yIntersects);
	addCellHitsZ(blockOrigin, cellSpan, blockDim, blockIdx, hits._zIntersects);
}

void Block::createIntersectionCells(Volume& vol, const Vector3d& blockOrigin, const Vector3d& blockSpan)
{
	if (_cells.empty())
		return;
	size_t bd = getBlockDim();
	const Vector3d cellSpan(blockSpan / s_blockDim);

	for (size_t cIdx = 0; cIdx < _cells.size(); cIdx++) {
		Vector3i cellIdx = calcCartesianCellIndex(cIdx);
		Cell* pCell = getCell(cellIdx);
		if (pCell) {
			Vector3d cellOrigin = calcCellOrigin(cellIdx, blockOrigin, blockSpan);
			pCell->makeIntersectionFaces(vol, _cells[cIdx], cellOrigin, cellSpan);
		}
	}

}

void Block::addBlockTris(const Vector3d& blockOrigin, const Vector3d& blockSpan, TriMesh::CMeshPtr& pMesh, bool useCells)
{

	if (useCells && !_cells.empty()) {
		const Vector3d vX(1, 0, 0), vY(0, 1, 0), vZ(0, 0, 1);
		Vector3d cellOrigin(blockOrigin);
		Vector3d cellSpan = blockSpan / s_blockDim;

		for (size_t icx = 0; icx < s_blockDim; icx++) {
			cellOrigin[0] = blockOrigin[0] + icx * cellSpan[0];
			for (size_t icy = 0; icy < s_blockDim; icy++) {
				cellOrigin[1] = blockOrigin[1] + icy * cellSpan[1];
				for (size_t iz = 0; iz < s_blockDim; iz++) {
					cellOrigin[2] = blockOrigin[2] + iz * cellSpan[2];

					size_t cIdx = calcLinearCellIndex(icx, icy, iz);
					if ((cIdx != -1) && (_cells[cIdx] != -1)) {
						vector<Vector3d> pts = {
							cellOrigin,
							cellOrigin + vX * cellSpan[0],
							cellOrigin + vX * cellSpan[0] + vY * cellSpan[1],
							cellOrigin +                    vY * cellSpan[1],

							cellOrigin + vZ * cellSpan[2],
							cellOrigin + vZ * cellSpan[2] + vX * cellSpan[0],
							cellOrigin + vZ * cellSpan[2] + vX * cellSpan[0] + vY * cellSpan[1],
							cellOrigin + vZ * cellSpan[2] +                    vY * cellSpan[1],
						};

						pMesh->addRectPrism(pts);

					}
				}
			}
		}
	}
	else {
		vector<Vector3d> pts = {
			Vector3d(blockOrigin[0], blockOrigin[1], blockOrigin[2]),
			Vector3d(blockOrigin[0] + blockSpan[0], blockOrigin[1], blockOrigin[2]),
			Vector3d(blockOrigin[0] + blockSpan[0], blockOrigin[1] + blockSpan[1], blockOrigin[2]),
			Vector3d(blockOrigin[0], blockOrigin[1] + blockSpan[1], blockOrigin[2]),

			Vector3d(blockOrigin[0], blockOrigin[1], blockOrigin[2] + blockSpan[2]),
			Vector3d(blockOrigin[0] + blockSpan[0], blockOrigin[1], blockOrigin[2] + blockSpan[2]),
			Vector3d(blockOrigin[0] + blockSpan[0], blockOrigin[1] + blockSpan[1], blockOrigin[2] + blockSpan[2]),
			Vector3d(blockOrigin[0], blockOrigin[1] + blockSpan[1], blockOrigin[2] + blockSpan[2]),
		};
		pMesh->addRectPrism(pts);
	}
}

bool Block::unload(string& filename)
{
	{
		ofstream out(filename, ofstream::binary);
		if (!out.good()) {
			return false;
		}

		size_t count = _cells.size();
		out.write((char*)&count, sizeof(count));
		for (size_t cellIdx : _cells) {
			if (cellIdx == -1)
				continue;
			Cell* pCell = &_cellData[cellIdx];
			if (!pCell->unload(out)) {
				return false;
			}
		}
		if (out.good()) {
			_filename = filename;
		} else {
			return false;
		}
	}

	_cells.clear();

	return true;
}

bool Block::load()
{
	if (_filename.empty())
		return false;
	ifstream in(_filename, ofstream::binary);
	if (!in.good()) return false;

	size_t size;
	in.read((char*) & size, sizeof(size));
	if (!in.good()) return false;

	_cells.resize(size);
	for (size_t cellIdx = 0; cellIdx < size; cellIdx++) {
		Cell* pCell;
		_cells[cellIdx] = createCell(pCell);
		if (!pCell->load(in)) {
			// TODO cleanup here
			return false;
		}
	}

	_filename.clear();

	return true;
}

void Block::addCellHitsX(const Vector3d& blockOrigin, const Vector3d& cellSpan, const Vector3i& blockDim, const Vector3i& blockIdx, const RayBlockIntersectVec& xHits)
{
	size_t hitIdx = blockIdx[1] + blockIdx[2] * blockDim[1];
	if (hitIdx < xHits.size() && xHits[hitIdx]) {
		auto& blockHits = *xHits[hitIdx];
		for (const auto& hit : blockHits) {
			size_t ibx = hit._blockIdx;
			if (ibx != blockIdx[0])
				continue;

			// This hit is in our block
			Vector3d cellOrigin = calcCellOrigin(hit._cellIdx, blockOrigin, cellSpan), offset(0, 0, 0);
			for (int iy = -1; iy <= 0; iy++) {
				offset[1] = iy * cellSpan[1];
				for (int iz = -1; iz <= 0; iz++) {
					offset[2] = iz * cellSpan[2];
					Cell* pCell = getCell(hit._cellIdx[0], hit._cellIdx[1] + iy, hit._cellIdx[2] + iz);
					if (pCell) {
						pCell->addHit(cellOrigin + offset, cellSpan, AxisIndex::X, hit);
					}
				}
			}
		}
	}
}

void Block::addCellHitsY(const Vector3d& blockOrigin, const Vector3d& cellSpan, const Vector3i& blockDim, const Vector3i& blockIdx, const RayBlockIntersectVec& yHits)
{
	size_t hitIdx = blockIdx[0] + blockIdx[2] * blockDim[0];
	if (hitIdx < yHits.size() && yHits[hitIdx]) {
		auto& blockHits = *yHits[hitIdx];
		for (const auto& hit : blockHits) {
			size_t iby = hit._blockIdx;
			if (iby != blockIdx[1])
				continue;

			Vector3d cellOrigin = calcCellOrigin(hit._cellIdx, blockOrigin, cellSpan), offset(0, 0, 0);

			for (int ix = -1; ix <= 0; ix++) {
				offset[0] = ix * cellSpan[0];
				for (int iz = -1; iz <= 0; iz++) {
					offset[2] = iz * cellSpan[2];
					Cell* pCell = getCell(hit._cellIdx[0] + ix, hit._cellIdx[1], hit._cellIdx[2] + iz);
					if (pCell) {
						pCell->addHit(cellOrigin + offset, cellSpan, AxisIndex::Y, hit);
					}
				}
			}
		}
	}
}

void Block::addCellHitsZ(const Vector3d& blockOrigin, const Vector3d& cellSpan, const Vector3i& blockDim, const Vector3i& blockIdx, const RayBlockIntersectVec& zHits)
{
	size_t hitIdx = blockIdx[0] + blockIdx[1] * blockDim[0];
	if (hitIdx < zHits.size() && zHits[hitIdx]) {
		auto& blockHits = *zHits[hitIdx];
		for (const auto& hit : blockHits) {
			size_t ibz = hit._blockIdx;
			if (ibz != blockIdx[2])
				continue;
			Vector3d cellOrigin = calcCellOrigin(hit._cellIdx, blockOrigin, cellSpan), offset(0, 0, 0);

			for (int ix = -1; ix <= 0; ix++) {
				offset[0] = ix * cellSpan[0];
				for (int iy = -1; iy <= 0; iy++) {
					offset[1] = iy * cellSpan[1];
					Cell* pCell = getCell(hit._cellIdx[0] + ix, hit._cellIdx[1] + iy, hit._cellIdx[2]);
					if (pCell) {
						pCell->addHit(cellOrigin + offset, cellSpan, AxisIndex::Z, hit);
					}
				}
			}
		}
	}
}

void Block::fillEmpty()
{
	const size_t numCells = s_blockDim * s_blockDim * s_blockDim;

	if (_cells.size() != numCells)
		_cells.resize(numCells, -1);
}

void Block::pack()
{
	for (size_t id : _cells) {
		if (id != -1)
			return;
	}

	_cells.clear();
}
