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
	: _vertices(true)
	, _polygons(true)
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
	: _vertices(src._vertices)
	, _polygons(src._polygons)
	, _polyhedronPool(src._polyhedronPool)
	, _cellPool(src._cellPool)
{

}

Block::~Block()
{
}

Vector3i Block::calCellIndexFromLinear(size_t linearIdx) const
{
	Vector3i result;

	size_t temp = linearIdx;

	size_t denom = _blockDim * _blockDim;
	result[2] = temp / denom;
	temp = temp % denom;

	denom = _blockDim;
	result[1] = temp / denom;
	temp = temp % denom;

	result[0] = temp;
	if (calLinearCellIndex(result) != linearIdx) {
		throw std::runtime_error("calCellIndexFromLinear failed.");
	}
	return result;
}

bool Block::cellExists(size_t ix, size_t iy, size_t iz) const
{
//	return _cellPool.idExists(calLinearCellIndex(ix, iy, iz));
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

										size_t cIdx = calLinearCellIndex(ix, iy + iiy, iz + iiz);
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

										size_t cIdx = calLinearCellIndex(ix + iix, iy, iz + iiz);
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

										size_t cIdx = calLinearCellIndex(ix + iix, iy + iiy, iz);
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
	size_t idx = calLinearCellIndex(ix, iy, iz);
	_cellPool.add(Cell(), -1);
}

void Block::addCell(const Vector3i& cellIdx)
{
	addCell(cellIdx[0], cellIdx[1], cellIdx[2]);
}

void Block::createCellsDeprecated(const vector<bool>& cellsToCreate)
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

				assert(i == calLinearCellIndex(ix, iy, iz));
#endif // _DEBUG

				_cellPool.add(Cell(), i);
			}
		}
	}
}

void Block::createCells()
{
	map<size_t, vector<RayTriHit>> cellIndices;
	for (const auto& rayHit : _rayTriHits) {
		auto iter = cellIndices.find(rayHit._cellIdx);
		if (iter == cellIndices.end()) {
			iter = cellIndices.insert(make_pair(rayHit._cellIdx, vector<RayTriHit>())).first;
		}
		auto& hits = iter->second;
		hits.push_back(rayHit);
	}

	for (const auto& pair : cellIndices) {
		Vector3i cellIdx = calCellIndexFromLinear(pair.first);
		const auto& hits = pair.second;
		if (hits.size() > 10) {
			// Need to organize the hits by block face
			// Then subdivide here.
		}
		auto pts = getCellCornerPts(cellIdx);
		addRectPrismFaces(pair.first, pts);
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

	auto result = TRI_LERP(pts, t[0], t[1], t[2]);

#ifdef _DEBUG
	Vertex::fromDbl(result[0]);
	Vertex::fromDbl(result[1]);
	Vertex::fromDbl(result[2]);
#endif // DEBUG


	return result;
}

void Block::addRectPrismFaces(size_t cellId, const vector<Vector3d>& pts)
{
	addQuadFace(cellId, { pts[0], pts[3], pts[2], pts[1] });
	addQuadFace(cellId, { pts[4], pts[5], pts[6], pts[7] });

	// add left and right
	addQuadFace(cellId, { pts[0], pts[4], pts[7], pts[3] });
	addQuadFace(cellId, { pts[1], pts[2], pts[6], pts[5] });

	// add front and back
	addQuadFace(cellId, { pts[0], pts[1], pts[5], pts[4] });
	addQuadFace(cellId, { pts[2], pts[3], pts[7], pts[6] });

}

void Block::addQuadFace(size_t cellId, const vector<Vector3d>& pts)
{

	Polygon newPoly, revPoly;
	for (const auto& pt : pts) {
		newPoly.addVertex(_vertices.add(pt, -1));
	}
	auto& verts = newPoly.getVertexIds();
	for (size_t i = verts.size() - 1; i != -1; i--)
		revPoly.addVertex(verts[i]);
	newPoly.doneCreating();
	revPoly.doneCreating();
	assert(!(newPoly <  revPoly));
	assert(!(revPoly < newPoly));

	Polygon* pPoly = _polygons.get(newPoly);
	if (pPoly) {
		assert(!(*pPoly < newPoly));
		assert(!(newPoly < *pPoly));
		assert(pPoly->getNeighborCellId() == -1);
		pPoly->setNeighborCellId(cellId);
		assert(!pPoly->isOuter());
	} else {
		newPoly.setOwnerCellId(cellId);
		_polygons.add(newPoly);
		pPoly = _polygons.get(newPoly);
		Polygon* pRevPoly = _polygons.get(revPoly);
		assert(pPoly != nullptr);
		assert(pPoly == pRevPoly);
		if (pPoly) {
			assert(pPoly == _polygons.get(revPoly));
			assert(!(*pPoly < newPoly));
			assert(!(newPoly < *pPoly));
			assert(pPoly->getNeighborCellId() == -1);
			assert(pPoly->isOuter());
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

				size_t bIdx = calLinearCellIndex(i, j, k);
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
	addTris(pSrcMesh, triIndices);

	processTris();
}

void Block::processTris()
{
	if (_pModelTriMesh->numTris() == 0)
		return;

	_cellDivs.resize(_blockDim * _blockDim * _blockDim, 1);

	setNumDivs();
	createCells();
	createIntersectionCells();

	_pModelTriMesh = nullptr;
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

	double span = subBB.range().norm();
	subBB.grow(0.05 * span);

	_pModelTriMesh = make_shared<CMesh>(subBB);

	for (size_t triIdx : triIndices) {
		const auto& triIndices = pSrcMesh->getTri(triIdx);
		Vector3d pts[3];
		for (int i = 0; i < 3; i++) {
			pts[i] = pSrcMesh->getVert(triIndices[i])._pt;
		}
		_pModelTriMesh->addTriangle(pts);
	}
}

TriMesh::CMeshPtr Block::getBlockTriMesh(bool outerOnly) const
{
	if (_polygons.empty())
		return nullptr;

	TriMesh::CMesh::BoundingBox bbox;
	_vertices.iterateInOrder([&bbox](size_t id, const Vertex& vert) {
		auto pt = vert.getPoint();
		bbox.merge(pt);
	});
	double span = bbox.range().norm();
	bbox.grow(0.05 * span);

	TriMesh::CMeshPtr result = make_shared<TriMesh::CMesh>(bbox);
	size_t skipped = 0;
	_polygons.iterateInOrder([this, result, outerOnly, &skipped](size_t id, const Polygon& poly) {
		if (!outerOnly || poly.isOuter()) {
			const auto& vertIds = poly.getVertexIds();
			if (vertIds.size() == 3 || vertIds.size() == 4) {
				vector<Vector3d> pts;
				for (size_t vertId : vertIds) {
					const auto& vert = _vertices[vertId];
					pts.push_back(vert.getPoint());
				}
				if (pts.size() == 3) {
					result->addTriangle(pts[0], pts[1], pts[2]);
				}
				else {
					result->addQuad(pts[0], pts[1], pts[2], pts[3]);
				}
			} 
		} else
			skipped++;
	});

//	cout << "Skipped " << skipped << "inner faces\n";
	return result;
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

void Block::rayCastFace(const std::vector<Vector3d>& pts, size_t samples, int axis, std::vector<RayTriHit>& rayTriHits) const
{
	const double tol = 1.0e-5;
	const Vector3d dir = axis == 0 ? Vector3d(1, 0, 0) : axis == 1 ? Vector3d(0, 1, 0) : Vector3d(0, 0, 1);
	Vector3d pt0, pt1, origin;
	int ix, iy, iz;
	double tx, ty, tz;
	for (int i = 0; i <= samples; i++) {
		for (int j = 0; j <= samples; j++) {
			switch (axis) {
				default:
				case 0: { // X axis is ray dir
					iy = i;
					ty = iy / (double)samples;

					iz = j;
					tz = iz / (double)samples;

					pt0 = BI_LERP(pts[0], pts[3], pts[7], pts[4], ty, tz);
					pt1 = BI_LERP(pts[1], pts[2], pts[6], pts[5], ty, tz);
					break;
				}
				case 1: { // Y axis is ray dir
					ix = i;
					tx = ix / (double)samples;

					iz = j;
					tz = iz / (double)samples;

					pt0 = BI_LERP(pts[0], pts[1], pts[5], pts[4], tx, tz);
					pt1 = BI_LERP(pts[3], pts[2], pts[6], pts[7], tx, tz);
					break;
				}
				case 2: { // Z axis is ray dir
					ix = i;
					tx = ix / (double)samples;

					iy = j;
					ty = iy / (double)samples;
					pt0 = BI_LERP(pts[0], pts[1], pts[2], pts[3], tx, ty);
					pt1 = BI_LERP(pts[4], pts[5], pts[6], pts[7], tx, ty);
					break;
				}
			}

			LineSegment seg(pt0, pt1);
			Ray ray(pt0, pt1 - pt0);
			double segLen = seg.calLength();
			vector<RayHit> hits;
			if (_pModelTriMesh->rayCast(seg, hits, tol)) {
				for (const auto& hit : hits) {
					double tRay = hit.dist / segLen;
					size_t rayIdx = (size_t)(tRay * _blockDim);
					if (rayIdx >= samples)
						rayIdx = samples - 1;
					Vector3i cellIdx;
					switch (axis) {
						default:
						case 0:
							cellIdx = Vector3i(rayIdx, iy, iz);
							break;
						case 1:
							cellIdx = Vector3i(ix, rayIdx, iz);
							break;
						case 2:
							cellIdx = Vector3i(ix, iy, rayIdx);
							break;
					}

					// These hits are on cell boundaries. A cell must be added on both sides of the boundary, for all axes.
					for (int dx = -1; dx <= 0; dx++) {
						for (int dy = -1; dy <= 0; dy++) {
							for (int dz = -1; dz <= 0; dz++) {
								RayTriHit rtHit;
								Vector3i cellIdx2(cellIdx + Vector3i(dx, dy, dz));
								if (cellIdx2[0] < samples && cellIdx2[1] < samples && cellIdx2[2] < samples) {
									rtHit._cellIdx = calLinearCellIndex(cellIdx2);
									if (rtHit._cellIdx < (samples * samples * samples)) {
										rtHit._triIdx = hit.triIdx;
										rtHit._relPt = hit.hitPt;
										rayTriHits.push_back(rtHit);
									} else {
										assert(!"Bad cellIdx");
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

void Block::setNumDivs()
{
	auto pts = getCornerPts();
	rayCastFace(pts, _blockDim, 0, _rayTriHits);
	rayCastFace(pts, _blockDim, 1, _rayTriHits);
	rayCastFace(pts, _blockDim, 2, _rayTriHits);
}

void Block::subDivideCellIfNeeded(const LineSegment& seg, const std::vector<RayHit>& hits, const Vector3i& cellIdx)
{
	if (hits.size() < 2)
		return;

	double segLen = seg.calLength();
	for (size_t i = 0; i < hits.size() - 1; i++) {
		size_t diff = 0;
		do {
			size_t divs = _cellDivs[calLinearCellIndex(cellIdx)];
			double frac0 = hits[i].dist / segLen;
			double frac1 = hits[i + 1].dist / segLen;
			double fracDiff = frac1 - frac0;
			if (segLen * fracDiff < 0.0001)
				break; // Less than 0.1 mm. Too small, don't fix this one. May be in a corner.

			size_t idx0 = (size_t)(frac0 * divs + 0.5);
			size_t idx1 = (size_t)(frac1 * divs + 0.5);
			diff = idx1 - idx0;
			if (diff == 0)
				_cellDivs[calLinearCellIndex(cellIdx)] *= 2;
		} while (diff == 0);
	}

}

void Block::createIntersectionCells()
{
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
