#include <vector>
#include <algorithm>
#include <fstream>
#define _USE_MATH_DEFINES
#include <cmath>

#include <Index3D.h>
#include <vertex.h>
#include <edge.h>
#include <polygon.h>
#include <polyhedron.h>
#include <block.h>
#include <volume.h>

using namespace std;
using namespace TriMesh;
using namespace DFHM;

Block::Block(Volume* pVol, const Index3D& blockIdx, const vector<Vector3d>& pts)
	: _vertices(true)
	, _polygons(true)
	, _polyhedra(false)
	, _pVol(pVol)
	, _blockIdx(blockIdx)
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

Block::~Block()
{
}

const Index3D& Block::getBlockIdx() const
{
	return _blockIdx;
}

size_t Block::numFaces(bool includeInner) const
{
	if (includeInner)
		return _polygons.size();

	size_t result = 0;
	_polygons.iterateInOrderTS([&result](size_t id, const Polygon& poly) {
		if (poly.isOuter())
			result++;
	});

	return result;
}

size_t Block::numPolyhedra() const
{
	return _polyhedra.size();
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

void Block::divideSubBlock(const Index3D& subBlockIdx, size_t divs)
{
	size_t linearIdx = calLinearSubBlockIndex(subBlockIdx);
	Polyhedron* poly = _polyhedra.get(linearIdx);

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

Index3D Block::determineOwnerBlockIdxFromRatios(const Vector3d& ratios) const
{
	const double tol = 1.0e-5;
	const auto& volBounds = _pVol->volDim();

	Index3D result(_blockIdx);
	for (int i = 0; i < 3; i++) {
		if (ratios[i] >= 1 - tol) {
			result[i] += 1;

			// Clamp it inside volume bounds. If it's shared by a block that doesn't exist, there's no chance
			// of a thread access conflict on that boundary
			if (result[i] >= volBounds[i]) {
				result[i] = volBounds[i] - 1;
			}
		}
	}

	return result;
}

Index3D Block::determineOwnerBlockIdx(const Vector3d& point) const
{
	if (_innerBoundBox.contains(point))
		return _blockIdx;

	auto ratios = invTriLinIterp(point);
	return determineOwnerBlockIdxFromRatios(ratios);
}

Index3D Block::determineOwnerBlockIdx(const Vertex& vert) const
{
	return determineOwnerBlockIdx(vert.getPoint());
}

Index3D Block::determineOwnerBlockIdx(const vector<Vector3d>& points) const
{
	const double tol = 1.0e-5;
	auto volBounds = _pVol->volDim();

	Vector3d ctr(0, 0, 0);
	for (const auto& pt : points) {
		ctr += pt;
	}
	ctr = ctr / points.size();

	return determineOwnerBlockIdx(ctr);
}

Index3D Block::determineOwnerBlockIdx(const std::vector<Index3DId>& verts) const
{
	vector<Vector3d> pts;
	pts.resize(verts.size());
	for (size_t i = 0; i < verts.size(); i++) {
		pts[i] = getVertexPoint(verts[i]);
	}
	return determineOwnerBlockIdx(pts);
}

Index3D Block::determineOwnerBlockIdx(const Polygon& face) const
{
	const double tol = 1.0e-5;
	auto volBounds = _pVol->volDim();

	Vector3d ctr(0, 0, 0);
	const auto& vertIds = face.getVertexIds();
	for (const auto& vertId : vertIds) {
		auto pt = getVertexPoint(vertId);
		ctr += pt;
	}
	ctr = ctr / vertIds.size();
	return determineOwnerBlockIdx(ctr);
}

bool Block::verifyTopology(bool all) const
{
	bool result = true;
	if (all)
		return _pVol->verifyTopology();

	_vertices.iterateInOrderTS([&result](size_t id, const Vertex& vert) {
		bool pass = vert.verifyTopology();
		if (!pass)
			result = false;
	});

	_polygons.iterateInOrderTS([&result](size_t id, const Polygon& face) {
		bool pass = face.verifyTopology();
		if (!pass)
			result = false;
	});

	_polyhedra.iterateInOrder([this, &result](size_t id, const Polyhedron& cell) {
		bool pass = cell.verifyTopology();
		if (!pass)
			result = false;
	});
	return result;
}

const Polyhedron& Block::getPolyhedron(const Index3DId& cellId) const
{
	auto pOwner = getOwner(cellId);
	if (pOwner == this)
		return _polyhedra[cellId.elementId()];
	else
		return pOwner->getPolyhedron(cellId);
}

Polyhedron& Block::getPolyhedron(const Index3DId& cellId)
{
	auto pOwner = getOwner(cellId);
	if (pOwner == this)
		return _polyhedra[cellId.elementId()];
	else
		return pOwner->getPolyhedron(cellId);
}

vector<size_t> Block::createSubBlocks()
{
	Index3D idx;
	vector<size_t> newCells;
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

vector<size_t> Block::dividePolyhedraByCurvature(const vector<size_t>& cellIndices)
{
	const double minCurvature = 1; // 1 meter radius
	const double kDiv = 1.0;

	vector<size_t> newCells;
	for (size_t idx : cellIndices) {
		if (!_polyhedra.exists(idx))
			continue;
		auto& cell = _polyhedra[idx];
		CBoundingBox3Dd bbox = cell.getBoundingBox();
		vector<size_t> edgeIndices;
		if (_pModelTriMesh->findEdges(bbox, edgeIndices)) {
			double avgSurfCurvature = 0;
			size_t numSamples = 0;
			for (auto edgeIndex : edgeIndices) {
				double edgeCurv = _pModelTriMesh->edgeCurvature(edgeIndex);
				if (edgeCurv >= 0) {
					avgSurfCurvature += edgeCurv;
					numSamples++;
				}
			}

			avgSurfCurvature /= edgeIndices.size();
			if (avgSurfCurvature < minCurvature)
				continue;
			double avgSurfRadius = 1 / avgSurfCurvature;
			double avgSurfCircumference = 2 * M_PI * avgSurfRadius;
			double avgSurfArcLength = avgSurfCircumference / 72.0; // 5 deg arc
			auto range = bbox.range();
			double minBoxDim = min(range[0], min(range[1], range[2]));
			if (kDiv * avgSurfArcLength > minBoxDim) {
				auto splitCells = cell.split(true);
				for (const auto& cellId : splitCells) {
					newCells.push_back(cellId);
				}
			}
		}
	}
	return newCells;
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

Index3DId Block::addFace(const vector<Index3DId>& vertIndices)
{
	if (!Polygon::verifyVertsConvex(this, vertIndices)) {
		return Index3DId();
	}

	auto ownerIdx = determineOwnerBlockIdx(vertIndices);
	Polygon newFace;
	for (const auto& vertId : vertIndices) {
		newFace.addVertex(vertId);
	}

	Block* pOwner = getOwner(ownerIdx);
	if (pOwner == this) {
		lock_guard g(_polygons);
		Index3DId faceId = _polygons.findOrAdd(this, newFace);

		for (const auto& vertId : vertIndices) {
			vertexFunc(vertId, [&faceId](Block* pBlock, Vertex& vert) {
				vert.addFaceId(faceId);
			});
		}
		return faceId;
	} else
		return pOwner->addFace(vertIndices);
}

size_t Block::addCell(const std::vector<Index3DId>& faceIds)
{
	Index3DId cellId = _polyhedra.findOrAdd(this, Polyhedron(faceIds));
	auto& cell = _polyhedra[cellId.elementId()];
	for (auto& faceId : cell.getFaceIds()) {
		faceFunc(faceId, [&cellId](Block* pBlock, Polygon& face) {
			face.addCell(cellId);
		});
	}
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
		vector<LineSegment> edgeSegs;
		getBlockEdgeSegs(pts.data(), edgeSegs);
		size_t numEdgeHits = 0;
		for (const auto& seg : edgeSegs) {
			vector<RayHit> hits;
			if (_pModelTriMesh->rayCast(seg, hits)) {
				numEdgeHits++;
			}
		}

		if (numEdgeHits == 0) {
			bool found = false;
			vector<size_t> vertIndices;
			if (_pModelTriMesh->findVerts(bbox, vertIndices)) {
				for (size_t vertIdx : vertIndices) {
					if (_pVol->getSharpVertIndices().count(vertIdx) != 0) {
						found = true;
						break;
					}
				}
			}
			if (!found)
				return -1;
		}
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

#if 0
	cout << "Adding hexSubBlock to block: [" << _blockIdx[0] << ", " << _blockIdx[1] << ", " << _blockIdx[2] << "] at subBlockIdx[" << subBlockIdx[0] << ", " << subBlockIdx[1] << ", " << subBlockIdx[2] << "]\n";
	for (const auto& faceId : faceIds) {
		const auto& faceBlockIdx = faceId.blockIdx();
		cout << "  faceBlockIdx: [" << faceBlockIdx[0] << ", " << faceBlockIdx[1] << ", " << faceBlockIdx[2] << "]: subBlockId - " << faceId.subBlockId() << "\n";
	}
#endif

	const Index3DId polyhedronId = _polyhedra.findOrAdd(this, Polyhedron(faceIds));
	assert(polyhedronId.isValid());

	for (const auto& faceId : faceIds) {
		if (faceId != _blockIdx) {
			int dbgBreak = 1;
		}
		Block* pOwnerBlock = getOwner(faceId);
		lock_guard g(pOwnerBlock->_polygons);
		auto pFace = pOwnerBlock->_polygons.get(faceId.elementId());
		if (pFace) {
			pFace->addCell(polyhedronId);
		}
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
	auto ownerBlockIdx = determineOwnerBlockIdx(pt);
	Block* pOwner = getOwner(ownerBlockIdx);
	if (pOwner == this)
		return Index3DId(_blockIdx, _vertices.findId(pt));
	else
		return Index3DId(pOwner->_blockIdx, pOwner->_vertices.findId(pt));
}

Index3DId Block::addVertex(const Vector3d& pt, size_t currentId)
{
	auto ownerBlockIdx = determineOwnerBlockIdx(pt);
	Block* pOwner = getOwner(ownerBlockIdx);
	if (pOwner == this)
		return _vertices.findOrAdd(this, pt, currentId);
	else
		return pOwner->addVertex(pt, currentId);
}

set<Edge> Block::getVertexEdges(const Index3DId& vertexId) const
{
	set<Edge> result;
	set<Index3DId> faceIds;
	vertexFunc(vertexId, [&faceIds](const Block* pBlock, const Vertex& vert) {
		faceIds = vert.getFaceIds();
	});
	for (const auto& faceId : faceIds) {
		vector<Edge> edges;
		faceFunc(faceId, [&edges](const Block* pBlock, const Polygon& face) {
			edges = face.getEdges();
		});
		for (const auto& edge : edges) {
			if (edge.containsVertex(vertexId)) {
				result.insert(edge);
			}
		}
	}

	return result;
}

void Block::addToLookup(const Index3DId& faceId)
{
	Block* pOwner = getOwner(faceId);
	if (pOwner == this)
		_polygons.addToLookup(faceId.elementId());
	else
		pOwner->_polygons.addToLookup(faceId.elementId());
}

Index3DId Block::addFace(const vector<Vector3d>& pts)
{
	Polygon newPoly;
	for (const auto& pt : pts) {
		auto vertId = addVertex(pt);
		newPoly.addVertex(vertId);
	}

	Index3D ownerIdx = determineOwnerBlockIdx(pts);
	Block* pOwner = getOwner(ownerIdx);
	Index3DId faceId = pOwner->_polygons.findOrAdd(pOwner, newPoly);
	faceFunc(faceId, [this, &faceId](const Block* pBlock, Polygon& face) {
		const auto& vertIds = face.getVertexIdsNTS();
		for (const auto& vertId : vertIds) {
			vertexFunc(vertId, [&faceId](const Block* pBlock, Vertex& vert) {
				vert.addFaceId(faceId);
			});
		}
	});

	return faceId;
}

Index3DId Block::addFace(int axis, const Index3D& subBlockIdx, const vector<Vector3d>& pts)
{
	Index3D polyBlockIdx(_blockIdx);

	Index3D blockDims = Volume::volDim();
	Index3D ownerBlockIdx = determineOwnerBlockIdx(pts);

	Block* pPolygonOwner = getOwner(ownerBlockIdx);

	auto faceId = pPolygonOwner->addFace(pts);

	return faceId;
}

bool Block::removeFromLookUp(const Index3DId& faceId)
{
	Block* pOwner = getOwner(faceId);
	if (pOwner == this) {
		return _polygons.removeFromLookup(faceId.elementId());
	} else
		return pOwner->_polygons.removeFromLookup(faceId.elementId());
}

Vector3d Block::getVertexPoint(const Index3DId& vertIdx) const
{
	auto pOwner = getOwner(vertIdx);
	lock_guard g(pOwner->_vertices);
	return pOwner->_vertices[vertIdx.elementId()].getPoint();
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

	set<size_t> newCells;
	_polyhedra.iterateInOrder([&newCells](size_t id, Polyhedron& cell) {
		newCells.insert(id);
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
	return _polyhedra.size();
}

void Block::addTris(const TriMesh::CMeshPtr& pSrcMesh)
{
	_pModelTriMesh = pSrcMesh;

}

size_t Block::splitCellsWithPlane(const Plane& splitPlane)
{
	size_t numSplit = 0;
	assert(verifyTopology(true));

	set<size_t> cellIndices;
	_polyhedra.iterateInOrder([&cellIndices](size_t id, const Polyhedron& cell) {
		cellIndices.insert(id);
	});

	for (size_t index : cellIndices) {
		if (!_polyhedra.exists(index))
			continue;
		auto& poly = _polyhedra[index];
		auto newCells = poly.split(splitPlane, false);
		numSplit += newCells.size();
	}

	assert(verifyTopology(true));
	return numSplit;
}

TriMesh::CMeshPtr Block::getBlockTriMesh(bool outerOnly, size_t minSplitNum) const
{
	if (_polygons.empty())
		return nullptr;

	TriMesh::CMesh::BoundingBox bbox;
	_vertices.iterateInOrderTS([&bbox](size_t id, const Vertex& vert) {
		auto pt = vert.getPoint();
		bbox.merge(pt);
	});
	double span = bbox.range().norm();
	bbox.grow(0.05 * span);

	TriMesh::CMeshPtr result = make_shared<TriMesh::CMesh>(bbox);
	size_t skipped = 0;
	_polygons.iterateInOrderTS([this, result, outerOnly, &skipped, minSplitNum](size_t id, const Polygon& poly) {

		if ((minSplitNum == 0 && (!outerOnly || poly.isOuter())) || poly.getNumSplits() >= minSplitNum) {
			const auto& vertIds = poly.getVertexIdsNTS();
			if (vertIds.size() == 3 || vertIds.size() == 4) {
				vector<Vector3d> pts;
				for (const auto& vertId : vertIds) {
					auto pVertexOwner = getOwner(vertId);
					lock_guard g(pVertexOwner->_vertices);
					const auto& vert = pVertexOwner->_vertices[vertId.elementId()];
					pts.push_back(vert.getPoint());
				}
				if (pts.size() == 3) {
					result->addTriangle(pts[0], pts[1], pts[2]);
				}
				else {
					result->addQuad(pts[0], pts[1], pts[2], pts[3]);
				}
			} else if (vertIds.size() > 4) {
				Vector3d ctr = poly.getCentroid();
				for (size_t i = 0; i < vertIds.size(); i++) {
					size_t j = (i + 1) % vertIds.size();
					Vector3d pt0 = getVertexPoint(vertIds[i]);
					Vector3d pt1 = getVertexPoint(vertIds[j]);
					result->addTriangle(ctr, pt0, pt1);
				}
			}
		} else
			skipped++;
	});

//	cout << "Skipped " << skipped << "inner faces\n";
	return result;
}

shared_ptr<vector<float>> Block::makeFaceEdges(bool outerOnly, size_t minSplitNum) const
{
	shared_ptr<vector<float>> result;

	set<Edge> edges;

	_polygons.iterateInOrderTS([this, outerOnly, &edges, minSplitNum](size_t faceId, const Polygon& face) {
		if ((minSplitNum == 0 && (!outerOnly || face.isOuter())) || face.getNumSplits() >= minSplitNum) {
			const auto& faceEdges = face.getEdges();
			edges.insert(faceEdges.begin(), faceEdges.end());
		}
	});

	if (!edges.empty()) {
		result = make_shared<vector<float>>();
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
	if (pOwner == this)
		return _vertices.exists(id.elementId());
	else
		return pOwner->_vertices.exists(id.elementId());
}


bool Block::polygonExists(const Index3DId& id) const

{
	auto pOwner = getOwner(id);
	if (pOwner == this)
		return _polygons.exists(id.elementId());
	else
		return pOwner->_polygons.exists(id.elementId());
}

bool Block::polyhedronExists(const Index3DId& id) const
{
	auto pOwner = getOwner(id);
	if (pOwner == this)
		return _polyhedra.exists(id.elementId());
	else
		return pOwner->_polyhedra.exists(id.elementId());
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


