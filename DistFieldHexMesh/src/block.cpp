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
			if (result[i] >= volBounds[i])
				result[i] = volBounds[i] - 1;
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

	Vector3d avgRatio(0, 0, 0);
	for (const auto& pt : points) {
		avgRatio += invTriLinIterp(pt);
	}
	avgRatio = avgRatio / points.size();

	return determineOwnerBlockIdxFromRatios(avgRatio);
}

Index3D Block::determineOwnerBlockIdx(const Polygon& face) const
{
	const double tol = 1.0e-5;
	auto volBounds = _pVol->volDim();

	Vector3d avgRatio(0, 0, 0);
	const auto& vertIds = face.getVertexIds();
	for (const auto& vertId : vertIds) {
		auto pt = getVertexPoint(vertId);
		avgRatio += invTriLinIterp(pt);
	}
	avgRatio = avgRatio / vertIds.size();

	return determineOwnerBlockIdxFromRatios(avgRatio);
}

size_t Block::createSubBlocks()
{
	const auto& blockPts = getCornerPts();

	Index3D idx;
	for (idx[0] = 0; idx[0] < _blockDim; idx[0]++) {
		for (idx[1] = 0; idx[1] < _blockDim; idx[1]++) {
			for (idx[2] = 0; idx[2] < _blockDim; idx[2]++) {
				addHexCell(blockPts.data(), _blockDim, idx);
			}
		}
	}

	return 1;
}

void Block::createSubBlocksForHexSubBlock(const Vector3d* blockPts, const Index3D& subBlockIdx)
{

	auto blockCornerPts = getCornerPts();	
	auto polyId = addHexCell(blockCornerPts.data(), _blockDim, subBlockIdx);
}

inline void Block::addIndexToMap(const Index3D& subBlockIdx, set<Index3D>& subBlockIndices)
{
	subBlockIndices.insert(subBlockIdx);
}

inline const std::vector<Vector3d>& Block::getCornerPts() const
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
	const double tol = 1.0e-8;
	const double tolSqr = tol * tol;

	// Linearly interpolate based on the bounding box. This is precise for a paralelapiped set of blockPts
	Vector3d vRange = _boundBox.range();
	Vector3d delta = pt - _boundBox.getMin();
	Vector3d result;
	for (size_t i = 0; i < 3; i++) {
		result[i] = delta[i] / vRange[i];
	}

	result = TRI_LERP(blockPts, result[0], result[1], result[2]);

	const double step = 1.0e-12;
	Vector3d vErr = result - pt;
	double errSqr = vErr.squaredNorm();
	while (errSqr > tolSqr) {
		// Compute gradient
		Vector3d grad;
		for (size_t axis = 0; axis < 3; axis++) {
			grad[axis] = ((result[axis] + step) - pt[axis]) / step;
		}
		grad.normalize();

		Vector3d testPt0 = result - step * grad;
		Vector3d testPt1 = result + step * grad;
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
		vErr = result - pt;
		errSqr = vErr.squaredNorm();
	}
		
	return result;
}

size_t Block::addHexCell(const Vector3d* blockPts, size_t blockDim, const Index3D& subBlockIdx)
{
	auto pts = getSubBlockCornerPts(blockPts, blockDim, subBlockIdx);

	CBoundingBox3Dd bbox;
	for (size_t i = 0; i < 8; i++) {
		bbox.merge(pts[i]);
	}

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

	Index3DId polyhedronId(_blockIdx, _polyhedra.findOrAdd(Polyhedron(faceIds)));
	auto pPoly = _polyhedra.get(polyhedronId.elementId());
	assert(pPoly);

	for (const auto& faceId : faceIds) {
		if (faceId != _blockIdx) {
			int dbgBreak = 1;
		}
		Block* pOwnerBlock = getOwner(faceId);
		lock_guard g(pOwnerBlock->_polygons);
		auto pFace = pOwnerBlock->_polygons.get(faceId.elementId());
		if (pFace) {
			if (!pFace->getCreatorCellId().isValid())
				pFace->setCreatorCellId(polyhedronId);
			else 
				pFace->setNeighborCellId(polyhedronId);
		}
	}

	return polyhedronId.elementId(); // We subBlocks are never shared across blocks, so we can drop the block index
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

inline Index3DId Block::addVertex(const Vector3d& pt, size_t currentId)
{
	auto ownerBlockIdx = determineOwnerBlockIdx(pt);
	Block* pOwner = getOwner(ownerBlockIdx);
	size_t vertId = pOwner->_vertices.findOrAdd(pt, currentId);
	return Index3DId(pOwner->_blockIdx, vertId);
}

set<Edge> Block::getVertexEdges(const Index3DId& vertexId) const
{
	std::set<Edge> result;
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

Index3DId Block::addFace(const vector<Vector3d>& pts)
{
	Polygon newPoly;
	for (const auto& pt : pts) {
		auto vertId = addVertex(pt);
		newPoly.addVertex(vertId);
	}
	newPoly.doneCreating();

	Index3D ownerIdx = determineOwnerBlockIdx(pts);
	Index3DId faceId(_blockIdx, _polygons.findOrAdd(newPoly));
	vector<Index3DId> vertIds;
	faceFunc(faceId, [this, &vertIds](const Block* pBlock, Polygon& face) {
		face.setOwnerBlockIdx(_blockIdx);
		vertIds = face.getVertexIds();
	});

	for (const auto& vertId : vertIds) {
		vertexFunc(vertId, [&faceId](const Block* pBlock, Vertex& vert) {
			vert.addFaceId(faceId);
		});
	}

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

void Block::processTris(const TriMesh::CMeshPtr& pSrcMesh)
{
	addTris(pSrcMesh);

	processTris();
}

size_t Block::processTris()
{
	if (_pModelTriMesh->numTris() == 0)
		return 0;

	// Get rid of the ray casting, it's not effective at this stage
	size_t numCreated = createSubBlocks();

	dividePolyhedra();

	return numCreated;
}

void Block::addTris(const TriMesh::CMeshPtr& pSrcMesh)
{
	_pModelTriMesh = pSrcMesh;
}

void Block::dividePolyhedra()
{
	dividePolyhedraAtSharpVerts();
	dividePolyhedraByCurvature();
}

void Block::dividePolyhedraAtSharpVerts()
{
	vector<size_t> deadPolyhedra;
	_polyhedra.iterateInOrder([this, &deadPolyhedra](size_t index, Polyhedron& poly) {
		CBoundingBox3Dd bbox = poly.getBoundingBox(this);
		for (size_t vertIdx : _pVol->getSharpVertIndices()) {
			auto pt = _pModelTriMesh->getVert(vertIdx)._pt;
			if (bbox.contains(pt)) {
				poly.split(this, pt);
				deadPolyhedra.push_back(index);
			}
		}
	});
	for (const auto& polyIdx : deadPolyhedra) {
		_polyhedra.free(polyIdx);
	}
}

void Block::dividePolyhedraByCurvature()
{
	_polyhedra.iterateInOrder([](size_t index, Polyhedron& poly) {
		});
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
			} 
		} else
			skipped++;
	});

//	cout << "Skipped " << skipped << "inner faces\n";
	return result;
}

shared_ptr<vector<float>> Block::makeFaceEdges(bool outerOnly) const
{
	shared_ptr<vector<float>> result;

	set<Edge> edges;

	_polygons.iterateInOrder([outerOnly, &edges](size_t faceId, const Polygon& face) {
		if (!outerOnly || face.isOuter()) {
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

void Block::getBlockEdgeSegs(const Vector3d* subBlockPoints, std::vector<LineSegment>& segs) const
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


