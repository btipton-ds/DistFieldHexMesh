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

Vector3i Block::getAxisOrder(AxisIndex axisIdx)
{
	const Vector3i axisOrder = axisIdx == AxisIndex::Z ? Vector3i(0, 1, 2) : (axisIdx == AxisIndex::Y ? Vector3i(2, 0, 1) : Vector3i(1, 2, 0));
	return axisOrder;
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
							case 0: {
								ix = k;

								for (int iiy = -1; iiy <= 1; iiy++) {
									if (iy + iiy >= s_blockDim)
										continue;
									for (int iiz = -1; iiz <= 1; iiz++) {
										if (iz + iiz >= s_blockDim)
											continue;

										size_t cIdx = calcCellIndex(ix, iy + iiy, iz + iiz);
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
								iy = k;

								for (int iix = -1; iix <= 1; iix++) {
									if (ix + iix >= s_blockDim)
										continue;
									for (int iiz = -1; iiz <= 1; iiz++) {
										if (iz + iiz >= s_blockDim)
											continue;

										size_t cIdx = calcCellIndex(ix + iix, iy, iz + iiz);
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
									if (ix + iix >= s_blockDim)
										continue;
									for (int iiy = -1; iiy <= 1; iiy++) {
										if (iy + iiy >= s_blockDim)
											continue;

										size_t cIdx = calcCellIndex(ix + iix, iy + iiy, iz);
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

void Block::createCells(const vector<bool>& cellsToCreate)
{
	if (_cells.empty())
		_cells.resize(s_blockDim * s_blockDim * s_blockDim, -1);

	if (_cells.size() == cellsToCreate.size()) {
		for (size_t i = 0; i < cellsToCreate.size(); i++) {
			if (cellsToCreate[i]) {
#ifdef _DEBUG


				size_t temp = i;

				size_t ix = temp % s_blockDim;
				temp = temp / s_blockDim;

				size_t iy = temp % s_blockDim;
				temp = temp / s_blockDim;

				size_t iz = temp % s_blockDim;

				assert(i == calcCellIndex(ix, iy, iz));
#endif // _DEBUG

				_cells[i] = _cellPool.create();
			}
		}
	}
}

void Block::addBlockTris(const Vector3d& blockOrigin, const Vector3d& blockSpan, TriMesh::CMeshPtr& pMesh, bool useCells)
{

	if (useCells && !_cells.empty()) {
		Vector3d cellOrgin(blockOrigin);
		Vector3d cellSpan = blockSpan * (1.0 / s_blockDim);
		for (size_t ix = 0; ix < s_blockDim; ix++) {
			cellOrgin[0] = blockOrigin[0] + ix * cellSpan[0];
			for (size_t iy = 0; iy < s_blockDim; iy++) {
				cellOrgin[1] = blockOrigin[1] + iy * cellSpan[1];
				for (size_t iz = 0; iz < s_blockDim; iz++) {
					cellOrgin[2] = blockOrigin[2] + iz * cellSpan[2];
					size_t cIdx = calcCellIndex(ix, iy, iz);
					if ((cIdx != -1) && (_cells[cIdx] != -1)) {
						vector<Vector3d> pts = {
							Vector3d(cellOrgin[0], cellOrgin[1], cellOrgin[2]),
							Vector3d(cellOrgin[0] + cellSpan[0], cellOrgin[1], cellOrgin[2]),
							Vector3d(cellOrgin[0] + cellSpan[0], cellOrgin[1] + cellSpan[1], cellOrgin[2]),
							Vector3d(cellOrgin[0], cellOrgin[1] + cellSpan[1], cellOrgin[2]),

							Vector3d(cellOrgin[0], cellOrgin[1], cellOrgin[2] + cellSpan[2]),
							Vector3d(cellOrgin[0] + cellSpan[0], cellOrgin[1], cellOrgin[2] + cellSpan[2]),
							Vector3d(cellOrgin[0] + cellSpan[0], cellOrgin[1] + cellSpan[1], cellOrgin[2] + cellSpan[2]),
							Vector3d(cellOrgin[0], cellOrgin[1] + cellSpan[1], cellOrgin[2] + cellSpan[2]),
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

void Block::processBlock(const TriMesh::CMeshPtr& pTriMesh, size_t blockRayIdx, const Vector3d& blockOrigin, const Vector3d& blockSpan, std::vector<bool>& cellsToCreate)
{
#if 0

	processBlock(pTriMesh, blockRayIdx, blockOrigin, blockSpan, AxisIndex::X, cellsToCreate);
	processBlock(pTriMesh, blockRayIdx, blockOrigin, blockSpan, AxisIndex::Y, cellsToCreate);
	processBlock(pTriMesh, blockRayIdx, blockOrigin, blockSpan, AxisIndex::Z, cellsToCreate);

	Vector3d cellOrigin, cellSpan;
	for (int i = 0; i < 3; i++)
		cellSpan[i] = blockSpan[i] / bd;

	for (size_t i = 0; i < bd; i++) {
		cellOrigin[0] = blockOrigin[0] + i * cellSpan[0];
		for (size_t j = 0; j < bd; j++) {
			cellOrigin[1] = blockOrigin[1] + j * cellSpan[1];
			for (size_t k = 0; k < bd; k++) {
				cellOrigin[2] = blockOrigin[2] + k * cellSpan[2];

				size_t bIdx = calcCellIndex(i, j, k);
				if (cellsToCreate[bIdx]) {
					Cell* pCell = nullptr;
					_cells[bIdx] = _cellPool.getObj(bIdx, pCell, true);
//					pCell->processBlock(pTriMesh, k, blockOrigin, blockSpan);
				}
			}
		}
	}
#endif
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
			Cell* pCell = _cellPool.getObj(cellIdx);
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

	for (size_t cellIdx : _cells) {
		_cellPool.unload(cellIdx);
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
		_cells[cellIdx] = _cellPool.getObj(-1, pCell, true);
		if (!pCell->load(in)) {
			// TODO cleanup here
			return false;
		}
	}

	_filename.clear();

	return true;
}
