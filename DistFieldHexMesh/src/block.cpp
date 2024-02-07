#include <vector>
#include <algorithm>
#include <fstream>
#define _USE_MATH_DEFINES
#include <cmath>

#include <Index3D.h>
#include <multiLockGuard.h>
#include <vertex.h>
#include <edge.h>
#include <polygon.h>
#include <polyhedron.h>
#include <block.h>
#include <volume.h>

using namespace std;
using namespace TriMesh;
using namespace DFHM;

atomic<size_t> Block::GlPoints::_statId = 0;

Block::GlPoints::GlPoints()
{
	_id = _statId++;
}
Block::GlPoints::GlPoints(const GlPoints& src)
	: vector<float>(src)
{
	_id = _statId++;
}

Block::Block(Volume* pVol, const Index3D& blockIdx, const vector<Vector3d>& pts)
	: _blockIdx(blockIdx)
	, _pVol(pVol)
	, _vertices(this, true)
	, _polygons(this, true)
	, _polyhedra(this, false)
{
	_blockDim = Index3D::getBlockDim();
	assert(pts.size() == 8);
	_corners.resize(8);
	for (size_t i = 0; i < pts.size(); i++) {
		_boundBox.merge(pts[i]);
		_corners[i] = pts[i];
	}

	_innerBoundBox.merge(_boundBox.getMin());
	Vector3d range = _boundBox.range();
	double k = 1.0 / 8;
	Vector3d v(
		(_blockDim - k) / _blockDim * range[0],
		(_blockDim - k) / _blockDim * range[1],
		(_blockDim - k) / _blockDim * range[2]
		);
	Vector3d pt = _boundBox.getMin() + v;
	_innerBoundBox.merge(pt);
}

const Index3D& Block::getBlockIdx() const
{
	return _blockIdx;
}

Index3D Block::calSubBlockIndexFromLinear(size_t linearIdx) const
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
	if (calLinearSubBlockIndex(result) != linearIdx) {
		throw runtime_error("calSubBlockIndexFromLinear failed.");
	}
	return result;
}

void Block::calBlockOriginSpan(Vector3d& origin, Vector3d& span) const
{
	Vector3d axis;
	const auto& pts = getCornerPts();
	origin = pts[0];

	span = Vector3d(
		(pts[1] - origin).norm(),
		(pts[3] - origin).norm(),
		(pts[4] - origin).norm()
	);

}

void Block::getAdjacentBlockIndices(std::set<Index3D>& indices) const
{
	for (int i = -1; i <= 1; i++) {
		for (int j = -1; j <= 1; j++) {
			for (int k = -1; k <= 1; k++) {
				Index3D idx = _blockIdx + Index3D(i, j, k);
				idx.clampInBounds(_pVol->volDim());
				indices.insert(idx);
			}
		}
	}
}

Index3D Block::determineOwnerBlockIdxFromRatios(const Vector3d& ratios, bool& isOnBoundary) const
{
	const double tol = 1.0e-5;
	const auto& volBounds = _pVol->volDim();

	bool boundary[] = {false, false, false};

	Index3D result(_blockIdx);
	for (int i = 0; i < 3; i++) {
		if (ratios[i] >= 1 - tol) {
			result[i] += 1;
			boundary[i] = true;

			// Clamp it inside volume bounds. If it's shared by a block that doesn't exist, there's no chance
			// of a thread access conflict on that boundary
			if (result[i] >= volBounds[i]) {
				result[i] = volBounds[i] - 1;
				boundary[i] = false;
			}
		}
	}

	isOnBoundary = boundary[0] || boundary[1] || boundary[2];

	return result;
}

Index3D Block::determineOwnerBlockIdx(const Vector3d& point, bool& isOnBoundary) const
{
	if (_innerBoundBox.contains(point))
		return _blockIdx;

	auto ratios = invTriLinIterp(point);
	return determineOwnerBlockIdxFromRatios(ratios, isOnBoundary);
}

Index3D Block::determineOwnerBlockIdx(const Vertex& vert, bool& isOnBoundary) const
{
	return determineOwnerBlockIdx(vert.getPoint(), isOnBoundary);
}

Index3D Block::determineOwnerBlockIdx(const vector<Vector3d>& points, bool& isOnBoundary) const
{
	auto volBounds = _pVol->volDim();

	Vector3d ctr(0, 0, 0);
	for (const auto& pt : points) {
		ctr += pt;
	}
	ctr = ctr / points.size();

	return determineOwnerBlockIdx(ctr, isOnBoundary);
}

Index3D Block::determineOwnerBlockIdx(const vector<Index3DId>& verts, bool& isOnBoundary) const
{
	vector<Vector3d> pts;
	pts.resize(verts.size());
	for (size_t i = 0; i < verts.size(); i++) {
		pts[i] = getVertexPoint(verts[i]);
	}
	return determineOwnerBlockIdx(pts, isOnBoundary);
}

Index3D Block::determineOwnerBlockIdx(const Polygon& face, bool& isOnBoundary) const
{
	auto volBounds = _pVol->volDim();

	Vector3d ctr(0, 0, 0);
	const auto& vertIds = face.getVertexIds();
	for (const auto& vertId : vertIds) {
		auto pt = getVertexPoint(vertId);
		ctr += pt;
	}
	ctr = ctr / vertIds.size();
	return determineOwnerBlockIdx(ctr, isOnBoundary);
}

size_t Block::numFaces(bool includeInner) const
{
	size_t result = 0;
	_polygons.iterateInOrder([&result, includeInner](const Polygon& face) {
		if (includeInner || face.isOuter())
			result++;
	});
	return result;
}

size_t Block::numPolyhedra() const
{
	return _polyhedra.size();
}

bool Block::verifyTopology() const
{
	bool result = true;
#ifdef _DEBUG 
	_polyhedra.iterateInOrder([&result](const Polyhedron& cell) {
		if (cell.getId() == Index3DId(Index3D(2, 0, 0), 0)) {
			int dbgBreak = 1;
		}
		// make sure we get block 3,1,1
		MultiLockGuard g(cell, this_thread::get_id());
		bool pass = cell.verifyTopology();
		if (!pass)
			result = false;
	});
#endif

	return result;
}

bool Block::verifyPolyhedronTopology(const Index3DId& cellId) const
{
	bool result = false;
	cellFunc(cellId, [&result](const Polyhedron& cell) {
		result = cell.verifyTopology();
	});
	return result;
}

void Block::addFaceToPolyhedron(const Index3DId& faceId, const Index3DId& cellId)
{
	cellFunc(cellId, [&faceId](Polyhedron& cell) {
		cell.addFace(faceId);
	});

}

vector<size_t> Block::createSubBlocks()
{
	vector<size_t> newCells;
	MultiLockGuard g(this, this_thread::get_id());
	Index3D idx;
	for (idx[0] = 0; idx[0] < _blockDim; idx[0]++) {
		for (idx[1] = 0; idx[1] < _blockDim; idx[1]++) {
			for (idx[2] = 0; idx[2] < _blockDim; idx[2]++) {
				size_t cellIdx = addHexCell(_corners.data(), _blockDim, idx, true);
				if (cellIdx != -1)
					newCells.push_back(cellIdx);
			}
		}
	}

	return newCells;
}

bool Block::dividePolyhedraByCurvature(const vector<Index3DId>& cellIndices, vector<Index3DId>& newCells)
{
	bool result = true;
	for (auto id : cellIndices) {
		auto pCell = _polyhedra.get(id);
		if (!pCell)
			continue;
		const double minCurvature = 1; // 1 meter radius
		const double kDiv = 1.0;

		CBoundingBox3Dd bbox = pCell->getBoundingBox();
		vector<CMesh::SearchEntry> edgeEntries;
		if (_pModelTriMesh->findEdges(bbox, edgeEntries)) {
			double avgSurfCurvature = 0;
			size_t numSamples = 0;
			for (const auto& edgeEntry : edgeEntries) {
				size_t edgeIndex = edgeEntry.getIndex();
				double edgeCurv = _pModelTriMesh->edgeCurvature(edgeIndex);
				if (edgeCurv >= 0) {
					avgSurfCurvature += edgeCurv;
					numSamples++;
				}
			}

			avgSurfCurvature /= edgeEntries.size();
			if (avgSurfCurvature < minCurvature)
				return false;
			double avgSurfRadius = 1 / avgSurfCurvature;
			double avgSurfCircumference = 2 * M_PI * avgSurfRadius;
			double avgSurfArcLength = avgSurfCircumference / 72.0; // 5 deg arc
			auto range = bbox.range();
			double minBoxDim = min(range[0], min(range[1], range[2]));
			if (kDiv * avgSurfArcLength > minBoxDim) {
				vector<Index3DId> splitCells;
				if (!pCell->split(true, splitCells)) {
					return false;
				}
				for (const auto& cellId : splitCells) {
					newCells.push_back(cellId);
				}
			}
		}
	}

	return true;
}

void Block::createSubBlocksForHexSubBlock(const Vector3d* blockPts, const Index3D& subBlockIdx)
{

	auto blockCornerPts = getCornerPts();	
	auto polyId = addHexCell(blockCornerPts.data(), _blockDim, subBlockIdx, false);
}

void Block::addIndexToMap(const Index3D& subBlockIdx, set<Index3D>& subBlockIndices)
{
	subBlockIndices.insert(subBlockIdx);
}

const vector<Vector3d>& Block::getCornerPts() const
{
	return _corners;
}

vector<Vector3d> Block::getSubBlockCornerPts(const Vector3d* blockPts, size_t divs, const Index3D& index) const
{
	vector<Vector3d> result = {
		triLinInterp(blockPts, divs, index + Index3D(0, 0, 0)),
		triLinInterp(blockPts, divs, index + Index3D(1, 0, 0)),
		triLinInterp(blockPts, divs, index + Index3D(1, 1, 0)),
		triLinInterp(blockPts, divs, index + Index3D(0, 1, 0)),

		triLinInterp(blockPts, divs, index + Index3D(0, 0, 1)),
		triLinInterp(blockPts, divs, index + Index3D(1, 0, 1)),
		triLinInterp(blockPts, divs, index + Index3D(1, 1, 1)),
		triLinInterp(blockPts, divs, index + Index3D(0, 1, 1)),
	};

	return result;
}


Vector3d Block::triLinInterp(const Vector3d* pts, size_t divs, const Index3D& index) const
{
	Vector3d t(
		index[0] / (double) divs,
		index[1] / (double) divs,
		index[2] / (double) divs
	);

	Vector3d result;
	result = TRI_LERP(pts, t[0], t[1], t[2]);

	return result;
}

Vector3d Block::invTriLinIterp(const Vector3d& pt) const
{
	return invTriLinIterp(_corners.data(), pt);
}

Vector3d Block::invTriLinIterp(const Vector3d* blockPts, const Vector3d& pt) const
{
	const double tol = 1.0e-5;
	const double tolSqr = tol * tol;

	// Linearly interpolate based on the bounding box. This is precise for a paralelapiped set of blockPts
	Vector3d vRange = _boundBox.range();
	Vector3d delta = pt - _boundBox.getMin();
	Vector3d result;
	for (size_t i = 0; i < 3; i++) {
		result[i] = delta[i] / vRange[i];
		assert(-tol <= result[i] && result[i] <= 1 + tol);
		if (result[i] < 0)
			result[i] = 0;
		else if (result[i] > 1)
			result[i] = 1;
	}

	Vector3d checkPt = TRI_LERP(blockPts, result[0], result[1], result[2]);

	const double step = 1.0e-12;
	Vector3d vErr = checkPt - pt;
	double errSqr = vErr.squaredNorm();
	while (errSqr > tolSqr) {
		// Compute gradient
		Vector3d grad;
		for (size_t axis = 0; axis < 3; axis++) {
			switch (axis) {
			default:
			case 0:
				checkPt = TRI_LERP(blockPts, result[0] + step, result[1], result[2]);
				break;
			case 1:
				checkPt = TRI_LERP(blockPts, result[0], result[1] + step, result[2]);
				break;
			case 2:
				checkPt = TRI_LERP(blockPts, result[0], result[1], result[2] + step);
				break;
			}
			grad[axis] = ((checkPt[axis]) - pt[axis]) / step;
		}
		grad.normalize();

		Vector3d ratio0 = - step * grad;
		Vector3d testPt0 = TRI_LERP(blockPts, ratio0[0], ratio0[1], ratio0[2] + step);

		Vector3d ratio1 = step * grad;
		Vector3d testPt1 = TRI_LERP(blockPts, ratio1[0], ratio1[1], ratio1[2] + step);

		double y0 = (testPt0 - pt).norm();
		double y1 = (result - pt).norm();
		double y2 = (testPt1 - pt).norm();
		y0 = y0 - y1;
		y1 = y2 - y1;
		double a = (y0 + y1) / (2 * step * step);
		double b = (y1 - y0) / (2 * step);
		if (fabs(a) < tol)
			break;
		double dx = -b / (2 * a);
		result = result + dx * grad;
		checkPt = TRI_LERP(blockPts, result[0], result[1], result[2]);
		vErr = checkPt - pt;
		errSqr = vErr.squaredNorm();
	}
		
	for (int i = 0; i < 3; i++) {
		assert(0 <= result[i] && result[i] <= 1);
	}
	return result;
}

const vector<Index3DId>& Block::getFaceVertexIds(const Index3DId& faceId) const
{
	auto pOwner = getOwner(faceId);
	return pOwner->_polygons[faceId].getVertexIds();
}



Index3DId Block::addFace(const vector<Index3DId>& vertIndices)
{
	if (!Polygon::verifyVertsConvexStat(this, vertIndices)) {
		return Index3DId();
	}

	bool isOnBoundary = false;
	auto ownerIdx = determineOwnerBlockIdx(vertIndices, isOnBoundary);
	Polygon newFace;
	for (const auto& vertId : vertIndices) {
		newFace.addVertex(vertId);
	}

	Block* pOwner = getOwner(ownerIdx);
	Index3DId faceId = pOwner->_polygons.findOrAdd(newFace);
	for (const auto& vertId : vertIndices) {
		vertexFunc(vertId, [&faceId](Vertex& vert) {
			vert.addFaceId(faceId);
		});
	}
	return faceId;
}
Index3DId Block::addFace(const vector<Vector3d>& pts)
{
	vector<Index3DId> vertIds;
	for (const auto& pt : pts) {
		auto vertId = addVertex(pt);
		vertIds.push_back(vertId);
	}
	if (!Polygon::verifyUniqueStat(vertIds))
		return Index3DId();
	return addFace(vertIds);
}

Index3DId Block::addFace(int axis, const Index3D& subBlockIdx, const vector<Vector3d>& pts)
{
	bool isOnBoundary = false;
	Index3D ownerBlockIdx = determineOwnerBlockIdx(pts, isOnBoundary);
	Block* pOwner = getOwner(ownerBlockIdx);

	auto faceId = pOwner->addFace(pts);

	return faceId;
}

size_t Block::addCell(const vector<Index3DId>& faceIds)
{
	Index3DId cellId = _polyhedra.findOrAdd(Polyhedron(faceIds));

	cellFunc(cellId, [this, &cellId](Polyhedron& cell) {
		for (auto& faceId : cell.getFaceIds()) {
			faceFunc(faceId, [&cellId](Polygon& face) {
				face.addCell(cellId);
			});
		}

		assert(cell.verifyTopology());
	});
	return cellId.elementId();
}

size_t Block::addHexCell(const Vector3d* blockPts, size_t blockDim, const Index3D& subBlockIdx, bool intersectingOnly)
{
	auto pts = getSubBlockCornerPts(blockPts, blockDim, subBlockIdx);

	CBoundingBox3Dd bbox;
	for (size_t i = 0; i < 8; i++) {
		bbox.merge(pts[i]);
	}

	if (intersectingOnly) {
		vector<CMesh::SearchEntry> triIndices;
		bool found = _pModelTriMesh->findTris(bbox, triIndices);

		if (!found) {
			auto sharps = _pVol->getSharpVertIndices();
			for (const auto& vertIdx : sharps) {
				if (bbox.contains(_pModelTriMesh->getVert(vertIdx)._pt)) {
					found = true;
					break;
				}
			}
		}
		if (!found)
			return -1;
#if 0
		vector<LineSegment> edgeSegs;
		getBlockEdgeSegs(pts.data(), edgeSegs);
		for (const auto& seg : edgeSegs) {
			vector<RayHit> hits;
			if (_pModelTriMesh->rayCast(seg, hits)) {
				hasContents = true;
				break;
			}
		}
#endif
	}

	vector<Index3DId> faceIds;
	faceIds.reserve(6);

	// add left and right
	faceIds.push_back(addFace(0, subBlockIdx, { pts[0], pts[4], pts[7], pts[3] }));
	faceIds.push_back(addFace(0, subBlockIdx, { pts[1], pts[2], pts[6], pts[5] }));

	// add front and back
	faceIds.push_back(addFace(1, subBlockIdx, { pts[0], pts[1], pts[5], pts[4] }));
	faceIds.push_back(addFace(1, subBlockIdx, { pts[2], pts[3], pts[7], pts[6] }));

	// add bottom and top
	faceIds.push_back(addFace(2, subBlockIdx, { pts[0], pts[3], pts[2], pts[1] }));
	faceIds.push_back(addFace(2, subBlockIdx, { pts[4], pts[5], pts[6], pts[7] }));

	const Index3DId polyhedronId = _polyhedra.findOrAdd(Polyhedron(faceIds));
	assert(polyhedronId.isValid());

	for (const auto& faceId : faceIds) {
		faceFunc(faceId, [&polyhedronId](Polygon& face) {
			face.addCell(polyhedronId);
		});
	}

	return polyhedronId.elementId(); // SubBlocks are never shared across blocks, so we can drop the block index
}

Block* Block::getOwner(const Index3D& blockIdx)
{
	if (blockIdx == _blockIdx)
		return this;

	return _pVol->getBlockPtr(blockIdx).get();
}

const Block* Block::getOwner(const Index3D& blockIdx) const
{
	if (blockIdx == _blockIdx)
		return this;

	return _pVol->getBlockPtr(blockIdx).get();
}

Index3DId Block::idOfPoint(const Vector3d& pt)
{
	bool isOnBoundary = false;
	auto ownerBlockIdx = determineOwnerBlockIdx(pt, isOnBoundary);
	Block* pOwner = getOwner(ownerBlockIdx);
	return pOwner->_vertices.findId(pt);
}

Index3DId Block::addVertex(const Vector3d& pt, const Index3DId& currentId)
{
	bool isOnBoundary = false;
	auto ownerBlockIdx = determineOwnerBlockIdx(pt, isOnBoundary);
	Block* pOwner = getOwner(ownerBlockIdx);
	Vertex vert(pt);
	return pOwner->_vertices.findOrAdd(vert, currentId);
}

set<Edge> Block::getVertexEdges(const Index3DId& vertexId) const
{
	set<Edge> result;
	set<Index3DId> faceIds = getVertexFaceIds(vertexId);
	for (const auto& faceId : faceIds) {
		set<Edge> edges;
		faceFunc(faceId, [&edges](const Polygon& face) {
			edges = face.getEdgesNTS();
		});
		for (const auto& edge : edges) {
			if (edge.containsVertex(vertexId)) {
				result.insert(edge);
			}
		}
	}

	return result;
}

const set<Index3DId>& Block::getVertexFaceIds(const Index3DId& vertexId) const
{
	auto pOwner = getOwner(vertexId);
	return pOwner->_vertices[vertexId].getFaceIds();
}

set<Index3DId> Block::getVertexFaceIds(const Index3DId& vertexId, const set<Index3DId>& availFaces) const
{
	auto pOwner = getOwner(vertexId);
	return pOwner->_vertices[vertexId].getFaceIds(availFaces);
}

void Block::addFaceToLookup(const Index3DId& faceId)
{
	auto pOwner = getOwner(faceId);
	pOwner->_polygons.addToLookup(faceId);
}

bool Block::removeFaceFromLookUp(const Index3DId& faceId)
{
	Block* pOwner = getOwner(faceId);
	return pOwner->_polygons.removeFromLookup(faceId);
}

Vector3d Block::getVertexPoint(const Index3DId& vertId) const
{
	auto pOwner = getOwner(vertId);
	return pOwner->_vertices[vertId].getPoint();
}

bool Block::unload(string& filename)
{
	{
		ofstream out(filename, ofstream::binary);
		if (!out.good()) {
			return false;
		}

		if (out.good()) {
			_filename = filename;
		} else {
			return false;
		}
	}

#if 0
	for (auto subBlockIdx : _subBlocks) {
		_subBlocks.unload(subBlockIdx);
	}
	_subBlocks.clear();
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

	_filename.clear();

	return true;
}

size_t Block::processTris()
{
	const int numDivs = 1;

	if (_pModelTriMesh->numTris() == 0)
		return 0;

	size_t count = 0;
	set<Index3DId> newCells;
	_polyhedra.iterateInOrder([&newCells, &count](Polyhedron& cell) {
		MultiLockGuard g(cell, this_thread::get_id());
		count++;
		newCells.insert(cell.getId());
	});

#if 1
	if (!newCells.empty()) {
#if 0
		for (size_t div = 0; div < numDivs; div++) {
			newCells = dividePolyhedraByCurvature(newCells);
		}
#endif
	}
#endif
	return count;
}

void Block::addTris(const CMeshPtr& pSrcMesh)
{
	_pModelTriMesh = pSrcMesh;

}

size_t Block::splitAllCellsWithPlane(const Plane& splittingPlane)
{
	size_t numSplits = 0;
	_polyhedra.iterateInOrder([&splittingPlane, &numSplits](Polyhedron& cell) {
		if (cell.getId() == Index3DId(Index3D(0, 0, 1), 0)) {
			int dbgBreak = 1;
		}
		MultiLockGuard g(cell, this_thread::get_id());
		auto temp = cell.splitWithPlane(splittingPlane, false);
		numSplits += temp.size();
	});

	/*
	_polygons.iterateInOrder([](Polygon& face) {
		face.clearSplitFromId();
	});
	*/
	return numSplits;
}

size_t Block::splitAllCellsWithPrinicpalPlanesAtPoint(const Vector3d& splitPt)
{
	size_t numSplit = 0;
	size_t nCells0, nCells1;

	nCells0 = _polyhedra.size();
	set<Index3DId> nextCells0, nextCells1;
	_polyhedra.iterateInOrder([&splitPt, &nextCells0](Polyhedron& cell) {
		MultiLockGuard g(cell, this_thread::get_id());
		Plane splitPlane(splitPt, Vector3d(1, 0, 0));
		if (cell.contains(splitPlane._origin)) {
			auto temp = cell.splitWithPlane(splitPlane, false);

			nextCells0.insert(temp.begin(), temp.end());
		}
	});

	nCells1 = _polyhedra.size();

	nCells0 = nCells1;

	for (const auto& cellId : nextCells0) {
		cellFunc(cellId, [&splitPt, &nextCells1](Polyhedron& cell) {
			Plane splitPlane(splitPt, Vector3d(0, 1, 0));
			auto temp = cell.splitWithPlane(splitPlane, false);
			nextCells1.insert(temp.begin(), temp.end());
		});
	}

	nCells1 = _polyhedra.size();

	nCells0 = nCells1;

	nextCells0.clear();
	for (const auto& cellId : nextCells1) {
		cellFunc(cellId, [&splitPt, &nextCells0](Polyhedron& cell) {
			Plane splitPlane(splitPt, Vector3d(0, 0, 1));
			auto temp = cell.splitWithPlane(splitPlane, false);
			nextCells0.insert(temp.begin(), temp.end());
		});
	}

	nCells1 = _polyhedra.size();

	return numSplit;
}

bool Block::includeFace(MeshType meshType, size_t minSplitNum, const Polygon& face) const
{
	bool result = false;
	switch (meshType) {
		default:
		case MT_ALL:
			result = true;
			break;
		case MT_INNER:
			result = !face.isOuter() && !face.isBlockBoundary();
			break;
		case MT_OUTER:
			result = face.isOuter();
			break;
		case MT_BLOCK_BOUNDARY:
			result = face.isBlockBoundary();
			break;
	}
	result = result && face.getNumSplits() >= minSplitNum;

	return result;
}

CMeshPtr Block::getBlockTriMesh(MeshType meshType, size_t minSplitNum)
{
	if (numFaces(true) == 0)
		return nullptr;

	CMesh::BoundingBox bbox = _boundBox;
	double span = bbox.range().norm();
	bbox.grow(0.05 * span);

	if (_blockMeshes.empty()) {
		_blockMeshes.resize(MT_ALL);
	}
	CMeshPtr result;
	_polygons.iterateInOrder([this, &result, &bbox, meshType, minSplitNum](const Polygon& face) {
		if (includeFace(meshType, minSplitNum, face)) {
			if (!result) {
				if (!_blockMeshes[meshType])
					_blockMeshes[meshType] = make_shared<CMesh>(bbox);
				result = _blockMeshes[meshType];
			}
			const auto& vertIds = face.getVertexIds();
			vector<Vector3d> pts;
			pts.reserve(vertIds.size());
			for (const auto& vertId : vertIds) {
				pts.push_back(getVertexPoint(vertId));
			}

			for (size_t i = 1; i < pts.size() - 1; i++) {
				size_t idx0 = 0;
				size_t idx1 = i;
				size_t idx2 = i + 1;
				result->addTriangle(pts[idx0], pts[idx1], pts[idx2]);
			}

			result->changed();
		}
	});

//	cout << "Skipped " << skipped << "inner faces\n";
	return result;
}

Block::glPointsPtr Block::makeFaceEdges(MeshType meshType, size_t minSplitNum)
{
	if (_blockEdges.empty())
		_blockEdges.resize(MT_ALL);

	set<Edge> edges;

	_polygons.iterateInOrder([this, meshType, &edges, minSplitNum](const Polygon& face) {
		if (includeFace(meshType, minSplitNum, face)) {
			const auto& faceEdges = face.getEdgesNTS();
			edges.insert(faceEdges.begin(), faceEdges.end());
		}
	});

	glPointsPtr result;
	if (!edges.empty()) {
		result = _blockEdges[meshType];
		if (!result)
			result = make_shared< GlPoints>();
		auto& vals = *result;
		for (const auto& edge : edges) {
			const auto* vertIds = edge.getVertexIds();

			Vector3d pt = getVertexPoint(vertIds[0]);
			vals.push_back((float)pt[0]);
			vals.push_back((float)pt[1]);
			vals.push_back((float)pt[2]);

			pt = getVertexPoint(vertIds[1]);
			vals.push_back((float)pt[0]);
			vals.push_back((float)pt[1]);
			vals.push_back((float)pt[2]);
		}
	}
	return result;
}

void Block::getBlockEdgeSegs(const Vector3d* subBlockPoints, vector<LineSegment>& segs) const
{
	segs.clear();
	segs.reserve(12);
	// X legs
	segs.push_back(LineSegment(subBlockPoints[0], subBlockPoints[1]));
	segs.push_back(LineSegment(subBlockPoints[3], subBlockPoints[2]));
	segs.push_back(LineSegment(subBlockPoints[4], subBlockPoints[5]));
	segs.push_back(LineSegment(subBlockPoints[7], subBlockPoints[6]));

	// Y legs
	segs.push_back(LineSegment(subBlockPoints[0], subBlockPoints[3]));
	segs.push_back(LineSegment(subBlockPoints[1], subBlockPoints[2]));
	segs.push_back(LineSegment(subBlockPoints[4], subBlockPoints[7]));
	segs.push_back(LineSegment(subBlockPoints[5], subBlockPoints[6]));

	// Z legs
	segs.push_back(LineSegment(subBlockPoints[0], subBlockPoints[4]));
	segs.push_back(LineSegment(subBlockPoints[1], subBlockPoints[5]));
	segs.push_back(LineSegment(subBlockPoints[2], subBlockPoints[6]));
	segs.push_back(LineSegment(subBlockPoints[3], subBlockPoints[7]));
}

bool Block::vertexExists(const Index3DId& id) const
{
	auto pOwner = getOwner(id);
	return pOwner->_vertices.exists(id);
}

bool Block::polygonExists(const Index3DId& id) const
{
	auto pOwner = getOwner(id);
	return pOwner->_polygons.exists(id);
}

bool Block::polyhedronExists(const Index3DId& id) const
{
	auto pOwner = getOwner(id);
	return pOwner->_polyhedra.exists(id);
}

void Block::pack()
{
#if 0
	for (auto id : _subBlocks) {
		if (id != -1)
			return;
	}

	_subBlocks.clear();
#endif
}
