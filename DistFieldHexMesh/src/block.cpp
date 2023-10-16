#include <vector>
#include <cell.h>
#include <block.h>

using namespace std;
using namespace TriMesh;
using namespace DFHM;

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

			LineSegment lineSeg(rayOrigin, rayOrigin + blockSpan[axisIdx2] * axis);
			// We need to support solids and surfaces
			// Solids will always have crossing in pairs, except at tangential crossings
			// Surface crossings are singular
			// This sorts them out
			vector<RayHit> hits;
			if (pTriMesh->rayCast(lineSeg, hits)) {
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

									size_t cIdx = calcCellIndex(iix, iiy, iiz);
									// No mutex required, because order of setting true is not a race condition.
									if (cIdx != -1) {
										result = true;
										cellsToCreate[cIdx] = true;
									}
								}
							}
						}
					}
				}

			}
		}
	}

	return result;
}

void Block::createCells(const std::vector<bool>& cellsToCreate)
{

	if (_cells.size() == cellsToCreate.size()) {
		for (size_t i = 0; i < cellsToCreate.size(); i++) {
			if (cellsToCreate[i]) {
				size_t temp = i;

				size_t ix = temp % s_blockDim;
				temp = temp / s_blockDim;

				size_t iy = temp % s_blockDim;
				temp = temp / s_blockDim;

				size_t iz = temp % s_blockDim;

				assert(i == calcCellIndex(ix, iy, iz));

				_cells[i] = _cellPool.create();

				auto pCell = getCell(ix, iy, iz);
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
			cellOrgin[ix] = blockOrigin[ix] + ix * cellSpan[0];
			for (size_t iy = 0; iy < s_blockDim; iy++) {
				cellOrgin[iy] = blockOrigin[iy] + iy * cellSpan[1];
				for (size_t iz = 0; iz < s_blockDim; iz++) {
					cellOrgin[iz] = blockOrigin[iz] + iz * cellSpan[2];
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

/******************** Block **************************/

size_t Block::s_blockDim = 8;

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
	_cells.resize(s_blockDim * s_blockDim * s_blockDim);
}

Block::Block(const Block& src)
	: _cells(src._cells)
{

}
