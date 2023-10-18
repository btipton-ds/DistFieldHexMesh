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
		_cells.resize(s_blockDim * s_blockDim * s_blockDim);

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

void Block::processBlock(const TriMesh::CMeshPtr& pTriMesh, size_t blockRayIdx, const Vector3d& blockOrigin, const Vector3d& blockSpan)
{
	vector<bool> cellsToCreate;
	size_t bd = getBlockDim();
	cellsToCreate.resize(bd * bd * bd);

	processBlock(pTriMesh, blockRayIdx, blockOrigin, blockSpan, AxisIndex::X, cellsToCreate);
	processBlock(pTriMesh, blockRayIdx, blockOrigin, blockSpan, AxisIndex::Y, cellsToCreate);
	processBlock(pTriMesh, blockRayIdx, blockOrigin, blockSpan, AxisIndex::Z, cellsToCreate);

}

void Block::processBlock(const TriMesh::CMeshPtr& pTriMesh, size_t blockRayIdx, const Vector3d& blockOrigin, const Vector3d& blockSpan, AxisIndex axisIdx, vector<bool>& cellsToCreate)
{
	const Vector3i axisOrder = getAxisOrder(axisIdx);

	const Vector3d rayDir = axisIdx == AxisIndex::Z ? Vector3d(0, 0, 1) : (axisIdx == AxisIndex::Y ? Vector3d(0, 1, 0) : Vector3d(1, 0, 0));

	vector<shared_ptr<RayBlockIntersectVec>> allHits;
	size_t bd = getBlockDim();
	size_t numSteps = bd + 1;

	allHits.resize(numSteps * numSteps);
	const double TOL = 1.0e-6;
	const size_t axis0 = axisOrder[0];
	const size_t axis1 = axisOrder[1];
	const size_t axis2 = axisOrder[2];

	Vector3d origin(blockOrigin), cellSpan;
	for (int i = 0; i < 3; i++)
		cellSpan[i] = blockSpan[i] / bd;

	for (size_t i = 0; i < numSteps; i++) {
		double t = i / (double) bd;
		origin[axis0] = origin[axis0] + t * cellSpan[axis0];

		for (size_t j = 0; j < numSteps; j++) {
			const size_t rayIdx = i + numSteps * j;
			double u = j / (double)bd;
			origin[axis1] = origin[axis1] + u * cellSpan[axis1];

			Ray ray(origin, rayDir);
			vector<RayHit> hits;
			if (pTriMesh->rayCast(ray, hits)) {
				if (!allHits[rayIdx])
					allHits[rayIdx] = make_shared<RayBlockIntersectVec>();
				auto& blockHits = *allHits[rayIdx];

				if (blockHits.empty())
					blockHits.resize(1);
				RayTriIntersectVec& rayHits = blockHits.front();

				for (const auto& triHit : hits) {
					double dist0 = triHit.dist;
					if (dist0 < 0)
						dist0 = 0;
					else if (dist0 >= blockSpan[axis2])
						dist0 = blockSpan[axis2];

					RayTriIntersect rti;
					rti._triIdx = triHit.triIdx;
					rti._blockIdx = blockRayIdx;

					double w0 = dist0 / blockSpan[axis2];

					rti._cellIdx = (size_t)(w0 * bd);
					if (rti._cellIdx >= bd)
						rti._cellIdx = bd - 1;
					double dist1 = dist0 - (rti._cellIdx * cellSpan[axis2]);

					rti._w = dist1 / cellSpan[axis2];
					rayHits.push_back(rti);
				}
			}
		}
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
