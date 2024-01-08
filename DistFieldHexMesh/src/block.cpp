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

size_t Block::s_minBlockDim = 8;

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

void Block::setMinBlockDim(size_t dim)
{
	s_minBlockDim = dim;
}

size_t Block::getMinBlockDim()
{
	return s_minBlockDim;
}

Block::Block(vector<Vector3d>& pts)
	: _vertexPool(true)
	, _polygonPool(true)
	, _polyhedronPool(false)
	, _cellPool(false)
	, _blockDim(s_minBlockDim)
{
	assert(pts.size() == 8);

	for (size_t i = 0; i < pts.size(); i++) {
		const auto& pt = pts[i];
		Vertex::FixedPt iPt(
			Vertex::fromDbl(pt[0]),
			Vertex::fromDbl(pt[1]),
			Vertex::fromDbl(pt[2])
			);
		_corners[i] = iPt;
	}
}

Block::Block(const Block& src)
	: _vertexPool(src._vertexPool)
	, _polygonPool(src._polygonPool)
	, _polyhedronPool(src._polyhedronPool)
	, _cellPool(src._cellPool)
{

}

Block::~Block()
{
}

bool Block::cellExists(size_t ix, size_t iy, size_t iz) const
{
//	return _cellPool.idExists(calcCellIndex(ix, iy, iz));
	return false;
}

bool Block::cellExists(const Vector3i& idx) const
{
	return cellExists(idx[0], idx[1], idx[2]);
}

void Block::calBlockOriginSpan(Vector3d& origin, Vector3d& span) const
{
	Vector3d axis;
	vector<Vector3d> pts = getCornerPts();
	origin = pts[0];

	span = Vector3d(
		(pts[1] - origin).norm(),
		(pts[3] - origin).norm(),
		(pts[4] - origin).norm()
	);

}

bool Block::scanCreateCellsWhereNeeded(vector<bool>& cellsToCreate, const Vector3i& axisOrder)
{
	static size_t numCells = 0;

	const int overFlow = 1;

	bool result = false;
	Vector3d axis;
	Vector3d origin, blockSpan;
	calBlockOriginSpan(origin, blockSpan);

	Vector3d cellSpan = blockSpan * (1 / _blockDim);

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
	for (size_t i = 0; i < s_minBlockDim; i++) {
		rayOrigin[axisIdx0] = origin[axisIdx0] + i * cellSpan[axisIdx0];

		for (size_t j = 0; j < s_minBlockDim; j++) {
			rayOrigin[axisIdx1] = origin[axisIdx1] + j * cellSpan[axisIdx1];

			Ray ray(rayOrigin, rayOrigin + blockSpan[axisIdx2] * axis);

			vector<RayHit> hits;
			_pModelTriMesh->rayCast(ray, hits);
			if (!hits.empty()) {
				for (const auto hit : hits) {

					double tk = hit.dist / blockSpan[axisIdx2];
					size_t k = (size_t)(s_minBlockDim * tk);
					if (0 <= k && k < s_minBlockDim) {
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
									if (iy + iiy >= s_minBlockDim)
										continue;
									for (int iiz = -1; iiz <= 1; iiz++) {
										if (iz + iiz >= s_minBlockDim)
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
									if (ix + iix >= s_minBlockDim)
										continue;
									for (int iiz = -1; iiz <= 1; iiz++) {
										if (iz + iiz >= s_minBlockDim)
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
									if (ix + iix >= s_minBlockDim)
										continue;
									for (int iiy = -1; iiy <= 1; iiy++) {
										if (iy + iiy >= s_minBlockDim)
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

void Block::addCell(size_t ix, size_t iy, size_t iz)
{
	if (_cellPool.empty())
		_cellPool.resize(s_minBlockDim * s_minBlockDim * s_minBlockDim);
	size_t idx = calcCellIndex(ix, iy, iz);
	_cellPool.add(Cell(), -1);
}

void Block::addCell(const Vector3i& cellIdx)
{
	addCell(cellIdx[0], cellIdx[1], cellIdx[2]);
}

void Block::createCells(const vector<bool>& cellsToCreate)
{
	if (_cellPool.empty())
		_cellPool.resize(s_minBlockDim * s_minBlockDim * s_minBlockDim);

	if (_cellPool.size() == cellsToCreate.size()) {
		for (size_t i = 0; i < cellsToCreate.size(); i++) {
			if (cellsToCreate[i]) {
#ifdef _DEBUG


				size_t temp = i;

				size_t ix = temp % s_minBlockDim;
				temp = temp / s_minBlockDim;

				size_t iy = temp % s_minBlockDim;
				temp = temp / s_minBlockDim;

				size_t iz = temp % s_minBlockDim;

				assert(i == calcCellIndex(ix, iy, iz));
#endif // _DEBUG

				_cellPool.add(Cell(), i);
			}
		}
	}
}

void Block::addBlockFaces(size_t blockId, bool makeCells)
{
	Vector3d blockOrigin, blockSpan;
	calBlockOriginSpan(blockOrigin, blockSpan);

	if (makeCells && !_cellPool.empty()) {
		const Vector3d vX(1, 0, 0), vY(0, 1, 0), vZ(0, 0, 1);
		Vector3d cellOrigin(blockOrigin);
		Vector3d cellSpan = blockSpan / s_minBlockDim;

		for (size_t ix = 0; ix < s_minBlockDim; ix++) {
			cellOrigin[0] = blockOrigin[0] + ix * cellSpan[0];
			for (size_t iy = 0; iy < s_minBlockDim; iy++) {
				cellOrigin[1] = blockOrigin[1] + iy * cellSpan[1];
				for (size_t iz = 0; iz < s_minBlockDim; iz++) {
					cellOrigin[2] = blockOrigin[2] + iz * cellSpan[2];

					size_t cIdx = calcCellIndex(ix, iy, iz);
					if ((cIdx != -1) && _cellPool.exists(cIdx)) {
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

vector<Vector3d> Block::getCornerPts() const
{
	vector<Vector3d> result;

	for (size_t i = 0; i < 8; i++) {
		auto iPt = _corners[i];
		Vector3d pt(
			Vertex::toDbl(iPt[0]),
			Vertex::toDbl(iPt[1]),
			Vertex::toDbl(iPt[2])
		);
		result.push_back(pt);
	}
	return result;
}

vector<Vector3d> Block::getCellCornerPts(const Vector3i& index) const
{
	vector<Vector3d> blockPts = getCornerPts();
	vector<Vector3d> result = {
		triLinInterp(blockPts, index + Vector3i(0, 0, 0)),
		triLinInterp(blockPts, index + Vector3i(1, 0, 0)),
		triLinInterp(blockPts, index + Vector3i(1, 1, 0)),
		triLinInterp(blockPts, index + Vector3i(0, 1, 0)),

		triLinInterp(blockPts, index + Vector3i(0, 0, 1)),
		triLinInterp(blockPts, index + Vector3i(1, 0, 1)),
		triLinInterp(blockPts, index + Vector3i(1, 1, 1)),
		triLinInterp(blockPts, index + Vector3i(0, 1, 1)),
	};

	return result;
}

Vector3d Block::triLinInterp(const std::vector<Vector3d>& pts, const Vector3i& index) const
{
	Vector3d t(
		index[0] / (double) _blockDim,
		index[1] / (double)_blockDim,
		index[2] / (double)_blockDim
	);

	Vector3d ptX00 = pts[0] + t[0] * (pts[1] - pts[0]);
	Vector3d ptX01 = pts[3] + t[0] * (pts[2] - pts[3]);
	Vector3d ptX10 = pts[4] + t[0] * (pts[5] - pts[4]);
	Vector3d ptX11 = pts[7] + t[0] * (pts[6] - pts[7]);

	Vector3d ptY0 = ptX00 + t[1] * (ptX01 - ptX00);
	Vector3d ptY1 = ptX10 + t[1] * (ptX11 - ptX10);

	Vector3d result = ptY0 + t[2] * (ptY1 - ptY0);

	return result;
}

void Block::addRectPrismFaces(size_t blockId, const vector<Vector3d>& pts)
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

void Block::addQuadFace(size_t blockId, const vector<Vector3d>& pts)
{
#if 0
	Polygon newPoly, revPoly;
	for (const auto& pt : pts) {
		newPoly.addVertex(_vertexPool.add(pt, -1));
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
		_polygonPool.add(newPoly);
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
#endif
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

void Block::processBlock(size_t blockRayIdx, vector<bool>& cellsToCreate)
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
					_cellPool[bIdx] = _cellPool.getObj(bIdx, pCell, true);
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

		size_t count = _cellPool.size();
		out.write((char*)&count, sizeof(count));
		for (size_t id = 0; id < _cellPool.size(); id++) {
			if (_cellPool.exists(id)) {
				Cell& cell = _cellPool[id];
				if (!cell.unload(out)) {
					return false;
				}
			}
		}
		if (out.good()) {
			_filename = filename;
		} else {
			return false;
		}
	}

#if 0
	for (auto cellIdx : _cellPool) {
		_cellPool.unload(cellIdx);
	}
	_cellPool.clear();
#endif

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

	_cellPool.resize(size);
	for (size_t cellIdx = 0; cellIdx < size; cellIdx++) {
		Cell cell;
		if (!cell.load(in)) {
			// TODO cleanup here
			return false;
		}
		_cellPool.add(cell, cellIdx);
	}

	_filename.clear();

	return true;
}

void Block::fillEmpty()
{
	const size_t numCells = s_minBlockDim * s_minBlockDim * s_minBlockDim;

	if (_cellPool.size() != numCells)
		_cellPool.resize(numCells);
}

void Block::processTris(const TriMesh::CMeshPtr& pSrcMesh, const vector<size_t>& triIndices)
{
	_cellDivs.resize(_blockDim * _blockDim * _blockDim, 1);
	addTris(pSrcMesh, triIndices);
	if (_pModelTriMesh->numTris() == 0)
		return;

	setNumDivs();

	if (!_cellLegIntersections.empty())
		cout << "Num cell leg intersections: " << _cellLegIntersections.size() << "\n";
}

void Block::addTris(const TriMesh::CMeshPtr& pSrcMesh, const vector<size_t>& triIndices)
{
	CMesh::BoundingBox subBB;
	for (size_t triIdx : triIndices) {
		const auto& triIndices = pSrcMesh->getTri(triIdx);
		for (int i = 0; i < 3; i++) {
			const auto& vert = pSrcMesh->getVert(triIndices[i]);
			subBB.merge(vert._pt);
		}
	}

	_pModelTriMesh = make_shared<CMesh>(subBB.getMin(), subBB.getMax());

	for (size_t triIdx : triIndices) {
		const auto& triIndices = pSrcMesh->getTri(triIdx);
		Vector3d pts[3];
		for (int i = 0; i < 3; i++) {
			pts[i] = pSrcMesh->getVert(triIndices[i])._pt;
		}
		_pModelTriMesh->addTriangle(pts);
	}
}

vector<LineSegment> Block::getCellEdges(const Vector3i& cellIdx) const
{
	const vector<Vector3d> cellPoints = getCellCornerPts(cellIdx);

	vector<LineSegment> edges = {
		// X legs
		LineSegment(cellPoints[0], cellPoints[1]),
		LineSegment(cellPoints[3], cellPoints[2]),
		LineSegment(cellPoints[4], cellPoints[5]),
		LineSegment(cellPoints[7], cellPoints[6]),

		// Y legs
		LineSegment(cellPoints[0], cellPoints[3]),
		LineSegment(cellPoints[1], cellPoints[2]),
		LineSegment(cellPoints[4], cellPoints[7]),
		LineSegment(cellPoints[5], cellPoints[6]),

		// Z legs
		LineSegment(cellPoints[0], cellPoints[4]),
		LineSegment(cellPoints[1], cellPoints[5]),
		LineSegment(cellPoints[2], cellPoints[6]),
		LineSegment(cellPoints[3], cellPoints[7]),
	};

	return edges;
}

void Block::setNumDivs()
{
	const double segDistTol = 1.0e-5;
	CellLegIntersect ci;
	Vector3i& cellIdx = ci._cellIdx;
	for (cellIdx[0] = 0; cellIdx[0] < _blockDim; cellIdx[0]++) {
		for (cellIdx[1] = 0; cellIdx[1] < _blockDim; cellIdx[1]++) {
			for (cellIdx[2] = 0; cellIdx[2] < _blockDim; cellIdx[2]++) {
				vector<LineSegment> edges = getCellEdges(cellIdx);

				for (ci._edgeNumber = 0; ci._edgeNumber < edges.size(); ci._edgeNumber++) {
					const LineSegment& seg = edges[ci._edgeNumber];
					ci.hits.clear();
					if (_pModelTriMesh->rayCast(seg, ci.hits, segDistTol) && ci.hits.size() > 1) {
						subDivideCellIfNeeded(seg, ci.hits, cellIdx);
						_cellLegIntersections.push_back(ci);
					}
				}
			}
		}
	}
}

void Block::subDivideCellIfNeeded(const LineSegment& seg, const std::vector<RayHit>& hits, const Vector3i& cellIdx)
{
	if (hits.size() < 2)
		return;

	double segLen = seg.calLength();
	for (size_t i = 0; i < hits.size() - 1; i++) {
		size_t diff = 0;
		do {
			size_t divs = _cellDivs[calcCellIndex(cellIdx)];
			double frac0 = hits[i].dist / segLen;
			double frac1 = hits[i + 1].dist / segLen;
			double fracDiff = frac1 - frac0;
			if (segLen * fracDiff < 0.0001)
				break; // Less than 0.1 mm. Too small, don't fix this one. May be in a corner.

			size_t idx0 = (size_t)(frac0 * divs + 0.5);
			size_t idx1 = (size_t)(frac1 * divs + 0.5);
			diff = idx1 - idx0;
			if (diff == 0)
				_cellDivs[calcCellIndex(cellIdx)] *= 2;
		} while (diff == 0);
	}

}

void Block::pack()
{
#if 0
	for (auto id : _cellPool) {
		if (id != -1)
			return;
	}

	_cellPool.clear();
#endif
}
