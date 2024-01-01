#include <vector>
#include <algorithm>
#include <fstream>

#include <cell.h>
#include <block.h>
#include <volume.h>
#include <vertex.h>
#include <polygon.h>

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
	for (auto id : _cells) {
		if (id != -1)
		_cellPool.free(id);
	}
}

bool Block::cellExists(size_t ix, size_t iy, size_t iz) const
{
	return _cellPool.idExists(calcCellIndex(ix, iy, iz));
}

bool Block::cellExists(const Vector3i& idx) const
{
	return cellExists(idx[0], idx[1], idx[2]);
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

void Block::addCell(size_t ix, size_t iy, size_t iz, size_t threadNum)
{
	if (_cells.empty())
		_cells.resize(s_blockDim * s_blockDim * s_blockDim, -1);
	size_t idx = calcCellIndex(ix, iy, iz);
	_cells[idx] = _cellPool.add(Cell(), ObjectPoolId(idx, threadNum));
}

void Block::addCell(const Vector3i& cellIdx, size_t threadNum)
{
	addCell(cellIdx[0], cellIdx[1], cellIdx[2], threadNum);
}

void Block::createCells(const vector<bool>& cellsToCreate, size_t threadNum)
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

				_cells[i] = _cellPool.add(Cell(), ObjectPoolId(i, threadNum));
			}
		}
	}
}

void Block::addBlockFaces(const ObjectPoolId& blockId, const Vector3d& blockOrigin, const Vector3d& blockSpan, bool makeCells) const
{

	if (makeCells && !_cells.empty()) {
		const Vector3d vX(1, 0, 0), vY(0, 1, 0), vZ(0, 0, 1);
		Vector3d cellOrigin(blockOrigin);
		Vector3d cellSpan = blockSpan / s_blockDim;

		for (size_t ix = 0; ix < s_blockDim; ix++) {
			cellOrigin[0] = blockOrigin[0] + ix * cellSpan[0];
			for (size_t iy = 0; iy < s_blockDim; iy++) {
				cellOrigin[1] = blockOrigin[1] + iy * cellSpan[1];
				for (size_t iz = 0; iz < s_blockDim; iz++) {
					cellOrigin[2] = blockOrigin[2] + iz * cellSpan[2];

					size_t cIdx = calcCellIndex(ix, iy, iz);
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

						addRectPrismFaces(blockId, pts);
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
		addRectPrismFaces(blockId, pts);
	}
}

void Block::addRectPrismFaces(const ObjectPoolId& blockId, const std::vector<Vector3d>& pts) const
{
	addQuadFace(blockId, { pts[0], pts[3], pts[2], pts[1] });
	addQuadFace(blockId, { pts[4], pts[5], pts[6], pts[7] });

	// add left and right
	addQuadFace(blockId, { pts[0], pts[4], pts[7], pts[3] });
	addQuadFace(blockId, { pts[1], pts[2], pts[6], pts[5] });

	// add front and back
	addQuadFace(blockId, { pts[0], pts[1], pts[5], pts[4] });
	addQuadFace(blockId, { pts[2], pts[3], pts[7], pts[6] });

}

void Block::addQuadFace(const ObjectPoolId& blockId, const std::vector<Vector3d>& pts) const
{
	Polygon newPoly, revPoly;
	for (const auto& pt : pts) {
		newPoly.addVertex(_vertexPool.add(pt, ObjectPoolId(-1, blockId.getThreadIndex())));
	}
	auto& verts = newPoly.getVertexIds();
	for (size_t i = verts.size() - 1; i != -1; i--)
		revPoly.addVertex(verts[i]);
	assert(newPoly.getHash() == revPoly.getHash());
	assert(!(newPoly <  revPoly));
	assert(!(revPoly < newPoly));

	Polygon* pPoly = _polygonPool.get(newPoly);
	if (pPoly) {
		assert(!(*pPoly < newPoly));
		assert(!(newPoly < *pPoly));
		assert(pPoly->getNeighborBlockId() == -1);
		pPoly->setNeighborBlockId(blockId);
	} else {
		newPoly.setOwnerBlockId(blockId);
		_polygonPool.add(newPoly, ObjectPoolId(-1, blockId.getThreadIndex()));
		pPoly = _polygonPool.get(newPoly);
		Polygon* pRevPoly = _polygonPool.get(revPoly);
		assert(pPoly != nullptr);
		assert(pPoly == pRevPoly);
		if (pPoly) {
			assert(pPoly == _polygonPool.get(revPoly));
			assert(!(*pPoly < newPoly));
			assert(!(newPoly < *pPoly));
			assert(pPoly->getNeighborBlockId() == -1);
		}
	}
}

size_t Block::getHash() const
{
	return -1;
}

bool Block::operator < (const Block& rhs) const
{
	assert(!"cannot reverse lookup blocks.");
	return false;
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
		for (auto cellIdx : _cells) {
			Cell& cell = _cellPool[cellIdx];
			if (!cell.unload(out)) {
				return false;
			}
		}
		if (out.good()) {
			_filename = filename;
		} else {
			return false;
		}
	}

	for (auto cellIdx : _cells) {
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
		Cell cell;
		if (!cell.load(in)) {
			// TODO cleanup here
			return false;
		}
		_cells[cellIdx] = _cellPool.add(cell, ObjectPoolId(cellIdx, 0));
	}

	_filename.clear();

	return true;
}

void Block::fillEmpty()
{
	const size_t numCells = s_blockDim * s_blockDim * s_blockDim;

	if (_cells.size() != numCells)
		_cells.resize(numCells, -1);
}

void Block::pack()
{
	for (auto id : _cells) {
		if (id != -1)
			return;
	}

	_cells.clear();
}
