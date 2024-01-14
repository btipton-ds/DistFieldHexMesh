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

void Block::setMinBlockDim(size_t dim)
{
	s_minBlockDim = dim;
}

size_t Block::getMinBlockDim()
{
	return s_minBlockDim;
}

Block::Block(Volume* pVol, const Index3D& blockIdx, vector<Vector3d>& pts)
	: _vertices(true)
	, _polygons(true)
	, _polyhedra(false)
	, _cells(false)
	, _blockDim(s_minBlockDim)
	, _pVol(pVol)
	, _blockIdx(blockIdx)
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

Block::~Block()
{
}

size_t Block::numFaces(bool includeInner) const
{
	if (includeInner)
		return _polygons.size();

	size_t result = 0;
	_polygons.iterateInOrder([&result](size_t id, const Polygon& poly) {
		if (poly.isOuter())
			result++;
	});

	return result;
}

size_t Block::numPolyhedra() const
{
	return _polyhedra.size();
}


Index3D Block::calCellIndexFromLinear(size_t linearIdx) const
{
	Index3D result;

	size_t temp = linearIdx;

	size_t denom = _blockDim * _blockDim;
	result[2] = (Index3DBaseType) (temp / denom);
	temp = temp % denom;

	denom = _blockDim;
	result[1] = (Index3DBaseType) (temp / denom);
	temp = temp % denom;

	result[0] = (Index3DBaseType)temp;
	if (calLinearCellIndex(result) != linearIdx) {
		throw std::runtime_error("calCellIndexFromLinear failed.");
	}
	return result;
}

bool Block::cellExists(size_t ix, size_t iy, size_t iz) const
{
	return _cells.exists(calLinearCellIndex(ix, iy, iz));
}

bool Block::cellExists(const Index3D& idx) const
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

void Block::createCells()
{
	if (_cells.empty()) {
		_cells.resize(_blockDim * _blockDim * _blockDim);
	}
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
		Index3D cellIdx = calCellIndexFromLinear(pair.first);
		const auto& hits = pair.second;
		if (hits.size() > 10) {
			// Need to organize the hits by block face
			// Then subdivide here.
		}

		auto polyHedraId = addHexCell(cellIdx);
		size_t cellId = _cells.add(Cell(), pair.first);
		_cells[cellId].addPolyhdra(polyHedraId);
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

vector<Block::CrossBlockPoint> Block::getCellCornerPts(const Index3D& index) const
{
	vector<Vector3d> blockPts = getCornerPts();
	vector<CrossBlockPoint> result = {
		triLinInterp(blockPts, index + Index3D(0, 0, 0)),
		triLinInterp(blockPts, index + Index3D(1, 0, 0)),
		triLinInterp(blockPts, index + Index3D(1, 1, 0)),
		triLinInterp(blockPts, index + Index3D(0, 1, 0)),

		triLinInterp(blockPts, index + Index3D(0, 0, 1)),
		triLinInterp(blockPts, index + Index3D(1, 0, 1)),
		triLinInterp(blockPts, index + Index3D(1, 1, 1)),
		triLinInterp(blockPts, index + Index3D(0, 1, 1)),
	};

	return result;
}

Block::CrossBlockPoint Block::triLinInterp(const std::vector<Vector3d>& pts, const Index3D& index) const
{
	Vector3d t(
		index[0] / (double) _blockDim,
		index[1] / (double)_blockDim,
		index[2] / (double)_blockDim
	);

	const Index3D blockDims = _pVol->getBlockDims();

	CrossBlockPoint result;
	result._pt = TRI_LERP(pts, t[0], t[1], t[2]);
	result._ownerBlockIdx = _blockIdx;

	for (int i = 0; i < 3; i++) {
		if (index[i] >= _blockDim) {
			result._ownerBlockIdx[i] = (result._ownerBlockIdx[i] + 1) % blockDims[i];
		}
	}

	return result;
}

size_t Block::addHexCell(const Index3D& cellIdx)
{
	auto pts = getCellCornerPts(cellIdx);
	vector<UniversalIndex3D> faceIds;
	faceIds.reserve(6);

	// add left and right
	faceIds.push_back(addFace(0, cellIdx, { pts[0], pts[4], pts[7], pts[3] }));
	faceIds.push_back(addFace(0, cellIdx, { pts[1], pts[2], pts[6], pts[5] }));

	// add front and back
	faceIds.push_back(addFace(1, cellIdx, { pts[0], pts[1], pts[5], pts[4] }));
	faceIds.push_back(addFace(1, cellIdx, { pts[2], pts[3], pts[7], pts[6] }));

	// add bottom and top
	faceIds.push_back(addFace(2, cellIdx, { pts[0], pts[3], pts[2], pts[1] }));
	faceIds.push_back(addFace(2, cellIdx, { pts[4], pts[5], pts[6], pts[7] }));

#if 0
	cout << "Adding hexCell to block: [" << _blockIdx[0] << ", " << _blockIdx[1] << ", " << _blockIdx[2] << "] at cellIdx[" << cellIdx[0] << ", " << cellIdx[1] << ", " << cellIdx[2] << "]\n";
	for (const auto& faceId : faceIds) {
		const auto& faceBlockIdx = faceId.blockIdx();
		cout << "  faceBlockIdx: [" << faceBlockIdx[0] << ", " << faceBlockIdx[1] << ", " << faceBlockIdx[2] << "]: cellId - " << faceId.cellId() << "\n";
	}
#endif

	UniversalIndex3D polyhedronId(_blockIdx, _polyhedra.add(Polyhedron(faceIds)));
	auto pPoly = _polyhedra.get(polyhedronId.cellId());
	assert(pPoly);

	for (const auto& faceId : faceIds) {
		if (faceId.blockIdx() != _blockIdx) {
			int dbgBreak = 1;
		}
		Block* pOwnerBlock = getOwner(faceId.blockIdx());
		lock_guard g(pOwnerBlock->_polygons);
		auto pFace = pOwnerBlock->_polygons.get(faceId.cellId());
		if (pFace) {
			if (!pFace->getOwnerCellId().isValid())
				pFace->setOwnerCellId(polyhedronId);
			else 
				pFace->setNeighborCellId(polyhedronId);
		}
	}

	return polyhedronId.cellId(); // We cells are never shared across blocks, so we can drop the block index
}

inline Block* Block::getOwner(const Index3D& blockIdx)
{
	if (blockIdx == _blockIdx)
		return this;

	return _pVol->getBlockPtr(blockIdx).get();
}

inline const Block* Block::getOwner(const Index3D& blockIdx) const
{
	if (blockIdx == _blockIdx)
		return this;

	return _pVol->getBlockPtr(blockIdx).get();
}

#define OTHER_BLOCK_MASK (Right | Back | Top)

inline UniversalIndex3D Block::addVertex(const CrossBlockPoint& pt, size_t currentId)
{
	Block* pOwner = getOwner(pt._ownerBlockIdx);
	size_t vertId = pOwner->_vertices.add(pt._pt, currentId);
	return UniversalIndex3D(pOwner->_blockIdx, vertId);
}

UniversalIndex3D Block::addFace(const std::vector<CrossBlockPoint>& pts)
{
	Polygon newPoly;
	for (const auto& pt : pts) {
		UniversalIndex3D vertId = addVertex(pt);
		newPoly.addVertex(vertId);
	}
	newPoly.doneCreating();

	UniversalIndex3D polyId(_blockIdx, _polygons.add(newPoly));

	lock_guard g(_polygons);
	auto pPoly = _polygons.get(polyId.cellId());
	assert(pPoly != nullptr);

	const auto& vertIds = pPoly->getVertexIds();
	for (const auto& vertId : vertIds) {
		auto pVertexOwner = getOwner(vertId.blockIdx());
		lock_guard g(pVertexOwner->_vertices);
		auto pVert = pVertexOwner->_vertices.get(vertId.cellId());
		if (pVert) {
			pVert->addPolygonReference(polyId);
		}
	}

	return polyId;
}

UniversalIndex3D Block::addFace(int axis, const Index3D& cellIdx, const vector<CrossBlockPoint>& pts)
{
	Index3D polyBlockIdx(_blockIdx);

	Index3D blockDims = _pVol->getBlockDims();
	Index3D ownerBlockIdx(_blockIdx);
	if (cellIdx[axis] == _blockDim - 1) {
		ownerBlockIdx[axis] = (ownerBlockIdx[axis] + 1) % blockDims[axis];
	}

	Block* pPolygonOwner = getOwner(ownerBlockIdx);

	UniversalIndex3D polyId = pPolygonOwner->addFace(pts);

	return polyId;
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
		for (size_t id = 0; id < _cells.size(); id++) {
			if (_cells.exists(id)) {
				Cell& cell = _cells[id];
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
	for (auto cellIdx : _cells) {
		_cells.unload(cellIdx);
	}
	_cells.clear();
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

	_cells.resize(size);
	for (size_t cellIdx = 0; cellIdx < size; cellIdx++) {
		Cell cell;
		if (!cell.load(in)) {
			// TODO cleanup here
			return false;
		}
		_cells.add(cell, cellIdx);
	}

	_filename.clear();

	return true;
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
				for (const auto& vertId : vertIds) {
					auto pVertexOwner = getOwner(vertId.blockIdx());
					lock_guard g(pVertexOwner->_vertices);
					const auto& vert = pVertexOwner->_vertices[vertId.cellId()];
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

vector<LineSegment> Block::getCellEdges(const Index3D& cellIdx) const
{
	const vector<CrossBlockPoint> cellPoints = getCellCornerPts(cellIdx);

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
					Index3D cellIdx;
					switch (axis) {
						default:
						case 0:
							cellIdx = Index3D(rayIdx, iy, iz);
							break;
						case 1:
							cellIdx = Index3D(ix, rayIdx, iz);
							break;
						case 2:
							cellIdx = Index3D(ix, iy, rayIdx);
							break;
					}

					// These hits are on cell boundaries. A cell must be added on both sides of the boundary, for all axes.
					for (int dx = -1; dx <= 0; dx++) {
						for (int dy = -1; dy <= 0; dy++) {
							for (int dz = -1; dz <= 0; dz++) {
								RayTriHit rtHit;
								Index3D cellIdx2(cellIdx + Index3D(dx, dy, dz));
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

void Block::subDivideCellIfNeeded(const LineSegment& seg, const std::vector<RayHit>& hits, const Index3D& cellIdx)
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

void Block::pack()
{
#if 0
	for (auto id : _cells) {
		if (id != -1)
			return;
	}

	_cells.clear();
#endif
}


