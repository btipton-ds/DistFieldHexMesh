#include <vector>
#include <algorithm>
#include <fstream>
#define _USE_MATH_DEFINES
#include <cmath>

#include <vertex.h>
#include <edge.h>
#include <polygon.h>
#include <subBlock.h>
#include <block.h>
#include <volume.h>

using namespace std;
using namespace TriMesh;
using namespace DFHM;

Block::Block(Volume* pVol, const Index3D& blockIdx, vector<Vector3d>& pts)
	: _vertices(true)
	, _edges(true)
	, _polygons(true)
	, _polyhedra(false)
	, _subBlocks(false)
	, _pVol(pVol)
	, _blockIdx(blockIdx)
{
	_blockDim = Index3D::getBlockDim();
	assert(pts.size() == 8);

	for (size_t i = 0; i < pts.size(); i++) {
		_corners[i] = pts[i];
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
		throw std::runtime_error("calSubBlockIndexFromLinear failed.");
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
	const Vector3d* pts = getCornerPts();
	origin = pts[0];

	span = Vector3d(
		(pts[1] - origin).norm(),
		(pts[3] - origin).norm(),
		(pts[4] - origin).norm()
	);

}

void Block::findFeatures()
{
	const double tol = 1.0e-3;

	// This function is already running on multiple cores, DO NOT calculate centroids or normals using muliple cores.
	_pModelTriMesh->buildCentroids(false);
	_pModelTriMesh->buildNormals(false);

	CBoundingBox3Dd bbox;
	for (size_t i = 0; i < 8; i++)
		bbox.merge(_corners[i]);
	bbox.grow(tol);

	findSharpVertices(bbox);
	findSharpEdgeGroups(bbox);
}

void Block::findSharpEdgeGroups(const CBoundingBox3Dd& bbox)
{
	const double sinSharpAngle = sin(_pVol->getSharpAngleRad());
	vector<size_t> indices;
	_pModelTriMesh->findEdges(bbox, indices);
	for (size_t edgeIdx : indices) {
		const auto& edge = _pModelTriMesh->getEdge(edgeIdx);
		if (edge._numFaces == 2 && _pModelTriMesh->isEdgeSharp(edgeIdx, sinSharpAngle)) {
			_sharpEdgeIndices.push_back(edgeIdx);
		}
	}
}

void Block::findSharpVertices(const CBoundingBox3Dd& bbox)
{
	const double cosSharpAngle = cos(M_PI - _pVol->getSharpAngleRad());
	const double tol = 1.0e-3;

	vector<size_t> vertIndices;
	size_t numVerts = _pModelTriMesh->findVerts(bbox, vertIndices);
	for (size_t vIdx : vertIndices) {
		const auto& vert = _pModelTriMesh->getVert(vIdx);
		if (!bbox.contains(vert._pt))
			continue;
		double maxDp = 1;
		const auto& edgeIndices = vert._edgeIndices;
		for (size_t i = 0; i < edgeIndices.size(); i++) {
			auto edge0 = _pModelTriMesh->getEdge(edgeIndices[i]);
			size_t opIdx0 = edge0._vertIndex[0] == vIdx ? edge0._vertIndex[1] : edge0._vertIndex[0];
			const auto& vert0 = _pModelTriMesh->getVert(opIdx0);
			Vector3d v0 = (vert0._pt - vert._pt).normalized();

			for (size_t j = i + 1; j < edgeIndices.size(); j++) {
				auto edge1 = _pModelTriMesh->getEdge(edgeIndices[j]);
				size_t opIdx1 = edge1._vertIndex[0] == vIdx ? edge1._vertIndex[1] : edge1._vertIndex[0];
				const auto& vert1 = _pModelTriMesh->getVert(opIdx1);
				Vector3d v1 = (vert1._pt - vert._pt).normalized();

				double dp = v0.dot(v1);
				if (dp < maxDp) {
					maxDp = dp;
					if (maxDp < cosSharpAngle)
						break;
				}
			}
			if (maxDp < cosSharpAngle)
				break;
		}
		if (maxDp > cosSharpAngle) {
			_sharpVertIndices.push_back(vIdx);
		}
	}
}

void Block::createSubBlocks()
{
	set<Index3D> subBlockIndices;

	addSubBlockIndicesForMeshVerts(subBlockIndices);
	addSubBlockIndicesForRayHits(subBlockIndices);

	if (subBlockIndices.empty())
		return;

	if (_subBlocks.empty()) {
		_subBlocks.resize(_blockDim * _blockDim * _blockDim);
	}

	const Vector3d* blockPts = getCornerPts();


	for (const auto& subBlockIdx : subBlockIndices) {
		createSubBlocksForHexSubBlock(blockPts, subBlockIdx);
	}
}

void Block::addSubBlockIndicesForMeshVerts(set<Index3D>& subBlockIndices)
{
	const Vector3d* pts = getCornerPts();
	CBoundingBox3Dd bbox;
	for (size_t i = 0; i < 8; i++)
		bbox.merge(pts[i]);

	// Make sure all vertices are surrounded by a subBlock
	size_t numTriVerts = _pModelTriMesh->numVertices();
	for (size_t vertIdx = 0; vertIdx < numTriVerts; vertIdx++) {
		const auto& vertPt = _pModelTriMesh->getVert(vertIdx)._pt;
		if (bbox.contains(vertPt)) {
			Vector3d interp = invTriLinIterp(pts, vertPt);
			Index3D subBlockIdx;
			for (int i = 0; i < 3; i++) {
				subBlockIdx[i] = (Index3DBaseType)(interp[i] * _blockDim);
			}
			if (subBlockIdx[0] < _blockDim && subBlockIdx[1] < _blockDim && subBlockIdx[2] < _blockDim) {
				auto subBlockPts = getSubBlockCornerPts(pts, _blockDim, subBlockIdx);
				CBoundingBox3Dd subBlockBbox;
				for (size_t i = 0; i < 8; i++)
					subBlockBbox.merge(subBlockPts[i]);
				if (subBlockBbox.contains(vertPt))
					addIndexToMap(subBlockIdx, subBlockIndices);
			}
		}
	}
}

void Block::addSubBlockIndicesForRayHits(set<Index3D>& subBlockIndices)
{
	// Make sure all grid to triangle intersection points are in a subBlock
	for (size_t axis = 0; axis < 3; axis++) {
		for (size_t i = 0; i < _blockDim; i++) {
			for (size_t j = 0; j < _blockDim; j++) {
				addHitsForRay(axis, i, j, 0, 0, subBlockIndices);
				addHitsForRay(axis, i, j, 0, 1, subBlockIndices);
				addHitsForRay(axis, i, j, 1, 1, subBlockIndices);
				addHitsForRay(axis, i, j, 1, 0, subBlockIndices);
			}
		}
	}
}

void Block::createSubBlocksForHexSubBlock(const Vector3d* blockPts, const Index3D& subBlockIdx)
{
	size_t subBlockId = _subBlocks.findOrAdd(SubBlock());
	auto& subBlock = _subBlocks[subBlockId];
	subBlock.init(this, subBlockIdx);
	subBlock.addRayHits(blockPts, _blockDim, subBlockIdx, _rayTriHits);
	subBlock.divide();

//	splitAtSharpVertices(blockPts, subBlockIdx);
//	splitAtLegIntersections(blockPts, subBlockIdx);
}

void Block::splitAtSharpVertices(const Vector3d* blockPts, const Index3D& subBlockIdx)
{
	// This a marching cubes style split. Create triangles between the leg intersections, cut and splice into 
	size_t linearIdx = calLinearSubBlockIndex(subBlockIdx);
	auto& subBlock = _subBlocks[linearIdx];
	if (subBlock.numPolyhedra() == 0) {
		auto cellId = addHexCell(blockPts, _blockDim, subBlockIdx);
		subBlock.addPolyhdra(cellId);
	}
}

void Block::splitAtLegIntersections(const Vector3d* blockPts, const Index3D& subBlockIdx)
{
}


void Block::addHitsForRay(size_t axis, size_t i, size_t j, size_t ii, size_t jj, set<Index3D>& subBlockIndices)
{
	size_t ix, iy, iz;

	const auto& faceHits = _rayTriHits[axis];
	const RayTriHitVector& rayHits = faceHits._data[i + ii][j + jj];
	for (const auto& rayHit : rayHits) {
		double t = rayHit._dist / rayHit._segLen;

		size_t rayIdx0 = (size_t)(_blockDim * t);
		if (rayIdx0 >= _blockDim)
			rayIdx0 = _blockDim - 1;

		size_t rayIdx1 = rayIdx0 + 1;
		if (rayIdx1 >= _blockDim)
			rayIdx1 = _blockDim - 1;

		switch (axis) {
		default:
		case 0: {
			iy = i;
			iz = j;
			addIndexToMap(Index3D(rayIdx0, iy, iz), subBlockIndices);
			addIndexToMap(Index3D(rayIdx1, iy, iz), subBlockIndices);
			break;
		}
		case 1:
			ix = i;
			iz = j;
			addIndexToMap(Index3D(ix, rayIdx0, iz), subBlockIndices);
			addIndexToMap(Index3D(ix, rayIdx1, iz), subBlockIndices);
			break;
		case 2:
			ix = i;
			iy = j;
			addIndexToMap(Index3D(ix, iy, rayIdx0), subBlockIndices);
			addIndexToMap(Index3D(ix, iy, rayIdx1), subBlockIndices);
			break;
		}
	}
}

inline void Block::addIndexToMap(const Index3D& subBlockIdx, set<Index3D>& subBlockIndices)
{
	subBlockIndices.insert(subBlockIdx);
}

inline const Vector3d* Block::getCornerPts() const
{
	return _corners;
}

vector<Block::CrossBlockPoint> Block::getSubBlockCornerPts(const Vector3d* blockPts, size_t blockDim, const Index3D& index) const
{
	vector<CrossBlockPoint> result = {
		triLinInterp(blockPts, blockDim, index + Index3D(0, 0, 0)),
		triLinInterp(blockPts, blockDim, index + Index3D(1, 0, 0)),
		triLinInterp(blockPts, blockDim, index + Index3D(1, 1, 0)),
		triLinInterp(blockPts, blockDim, index + Index3D(0, 1, 0)),

		triLinInterp(blockPts, blockDim, index + Index3D(0, 0, 1)),
		triLinInterp(blockPts, blockDim, index + Index3D(1, 0, 1)),
		triLinInterp(blockPts, blockDim, index + Index3D(1, 1, 1)),
		triLinInterp(blockPts, blockDim, index + Index3D(0, 1, 1)),
	};

	return result;
}


Block::CrossBlockPoint Block::triLinInterp(const Vector3d* pts, size_t blockDim, const Index3D& index) const
{
	Vector3d t(
		index[0] / (double) blockDim,
		index[1] / (double) blockDim,
		index[2] / (double) blockDim
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

Vector3d Block::invTriLinIterp(const Vector3d* blockPts, const Vector3d& pt)
{
	const double tol = 1.0e-8;
	const double tolSqr = tol * tol;
	CBoundingBox3Dd bbox;
	for (size_t i = 0; i < 8; i++)
		bbox.merge(blockPts[i]);

	// Linearly interpolate based on the bounding box. This is precise for a paralelapiped set of blockPts
	Vector3d vRange = bbox.range();
	Vector3d result;
	for (size_t i = 0; i < 3; i++) {
		result[i] = (pt[i] - bbox.getMin()[i]) / vRange[i];
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
	vector<Index3DFull> faceIds;
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

	Index3DFull polyhedronId(_blockIdx, subBlockIdx, _polyhedra.findOrAdd(Polyhedron(faceIds)));
	auto pPoly = _polyhedra.get(polyhedronId.elementId());
	assert(pPoly);

	for (const auto& faceId : faceIds) {
		if (faceId.blockIdx() != _blockIdx) {
			int dbgBreak = 1;
		}
		Block* pOwnerBlock = getOwner(faceId.blockIdx());
		lock_guard g(pOwnerBlock->_polygons);
		auto pFace = pOwnerBlock->_polygons.get(faceId.elementId());
		if (pFace) {
			if (!pFace->getOwnerSubBlockId().isValid())
				pFace->setOwnerSubBlockId(polyhedronId);
			else 
				pFace->setNeighborSubBlockId(polyhedronId);
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

inline Index3DFull Block::addVertex(const CrossBlockPoint& pt, size_t currentId)
{
	Block* pOwner = getOwner(pt._ownerBlockIdx);
	size_t vertId = pOwner->_vertices.findOrAdd(pt._pt, currentId);
	return Index3DFull(pOwner->_blockIdx, vertId);
}

Index3DFull Block::addEdge(const Index3DFull& vertId0, const Index3DFull& vertId1)
{
	Edge edge(_blockIdx, vertId0, vertId1);
	Index3DFull edgeId(_blockIdx, _edges.findOrAdd(edge));
	return edgeId;
}

Index3DFull Block::addFace(const std::vector<CrossBlockPoint>& pts)
{
	Polygon newPoly;
	for (const auto& pt : pts) {
		Index3DFull vertId = addVertex(pt);
		newPoly.addVertex(vertId);
	}
	newPoly.doneCreating();

	Index3DFull faceId(_blockIdx, _polygons.findOrAdd(newPoly));

	lock_guard g(_polygons);
	auto pPoly = _polygons.get(faceId.elementId());
	assert(pPoly != nullptr);

	const auto& vertIds = pPoly->getVertexIds();
	for (size_t i = 0; i < vertIds.size(); i++) {
		auto vertId = vertIds[i];
		size_t j = (i + 1) % vertIds.size();
		auto nextVertId = vertIds[j];

		Index3DFull edgeId = addEdge(vertId, nextVertId);
		{
			lock_guard g(_edges);
			auto& edge = _edges[edgeId.elementId()];
			edge.addFaceId(faceId);
		}

		{
			auto pVertexOwner = getOwner(vertId.blockIdx());
			lock_guard g(pVertexOwner->_vertices);
			auto pVert = pVertexOwner->_vertices.get(vertId.elementId());
			if (pVert) {
				pVert->addFaceId(faceId);
				pVert->addEdgeId(edgeId);
			}
		}
	}

	return faceId;
}

Index3DFull Block::addFace(int axis, const Index3D& subBlockIdx, const vector<CrossBlockPoint>& pts)
{
	Index3D polyBlockIdx(_blockIdx);

	Index3D blockDims = _pVol->getBlockDims();
	Index3D ownerBlockIdx(_blockIdx);
	if (subBlockIdx[axis] == _blockDim - 1) {
		ownerBlockIdx[axis] = (ownerBlockIdx[axis] + 1) % blockDims[axis];
	}

	Block* pPolygonOwner = getOwner(ownerBlockIdx);

	Index3DFull faceId = pPolygonOwner->addFace(pts);

	return faceId;
}

bool Block::unload(string& filename)
{
	{
		ofstream out(filename, ofstream::binary);
		if (!out.good()) {
			return false;
		}

		size_t count = _subBlocks.size();
		out.write((char*)&count, sizeof(count));
		for (size_t id = 0; id < _subBlocks.size(); id++) {
			if (_subBlocks.exists(id)) {
				SubBlock& subBlock = _subBlocks[id];
				if (!subBlock.unload(out)) {
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

	_subBlocks.resize(size);
	for (size_t subBlockIdx = 0; subBlockIdx < size; subBlockIdx++) {
		size_t idx = _subBlocks.findOrAdd(SubBlock(), subBlockIdx);
		auto& subBlock = _subBlocks[idx];
		subBlock.init(this, calSubBlockIndexFromLinear(subBlockIdx));
		if (!subBlock.load(in)) {
			// TODO cleanup here
			return false;
		}

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

	rayCastHexBlock(getCornerPts(), _blockDim, _rayTriHits);
	findFeatures();
	createSubBlocks();
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

vector<LineSegment> Block::getSubBlockEdges(const Vector3d* subBlockPoints, const Index3D& subBlockIdx) const
{
//	const vector<CrossBlockPoint> subBlockPoints = getSubBlockCornerPts(subBlockIdx);

	vector<LineSegment> edges = {
		// X legs
		LineSegment(subBlockPoints[0], subBlockPoints[1]),
		LineSegment(subBlockPoints[3], subBlockPoints[2]),
		LineSegment(subBlockPoints[4], subBlockPoints[5]),
		LineSegment(subBlockPoints[7], subBlockPoints[6]),

		// Y legs
		LineSegment(subBlockPoints[0], subBlockPoints[3]),
		LineSegment(subBlockPoints[1], subBlockPoints[2]),
		LineSegment(subBlockPoints[4], subBlockPoints[7]),
		LineSegment(subBlockPoints[5], subBlockPoints[6]),

		// Z legs
		LineSegment(subBlockPoints[0], subBlockPoints[4]),
		LineSegment(subBlockPoints[1], subBlockPoints[5]),
		LineSegment(subBlockPoints[2], subBlockPoints[6]),
		LineSegment(subBlockPoints[3], subBlockPoints[7]),
	};

	return edges;
}

size_t Block::rayCastFace(const Vector3d* pts, size_t samples, int axis, FaceRayHits& rayTriHits) const
{
	const double tol = 1.0e-5;
	const Vector3d dir = axis == 0 ? Vector3d(1, 0, 0) : axis == 1 ? Vector3d(0, 1, 0) : Vector3d(0, 0, 1);

	size_t numHits = 0;

	Vector3d pt0, pt1, origin;
	size_t ix, iy, iz;
	double tx, ty, tz;
	rayTriHits.resize(samples + 1);
	for (size_t i = 0; i <= samples; i++) {
		for (size_t j = 0; j <= samples; j++) {
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
				RayTriHitVector& faceSampleHits = rayTriHits._data[i][j];
				for (const auto& hit : hits) {
					RayTriHit rtHit;
					rtHit._triIdx = hit.triIdx;
					rtHit._hitPt = hit.hitPt;
					rtHit._dist = hit.dist;
					rtHit._segLen = segLen;
					faceSampleHits.push_back(rtHit);
					numHits++;
				}
			}
		}
	}
	return numHits;
}

void Block::rayCastHexBlock(const Vector3d* pts, size_t blockDim, FaceRayHits _rayTriHits[3])
{
	rayCastFace(pts, blockDim, 0, _rayTriHits[0]);
	rayCastFace(pts, blockDim, 1, _rayTriHits[1]);
	rayCastFace(pts, blockDim, 2, _rayTriHits[2]);
}

size_t Block::getGLModelEdgeLoops(std::vector<std::vector<float>>& edgeLoops) const
{
	return 0;
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


