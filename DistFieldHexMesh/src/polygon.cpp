/*
This file is part of the DistFieldHexMesh application/library.

	The DistFieldHexMesh application/library is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	The DistFieldHexMesh application/library is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	This link provides the exact terms of the GPL license <https://www.gnu.org/licenses/>.

	The author's interpretation of GPL 3 is that if you receive money for the use or distribution of the DistFieldHexMesh application/library or a derivative product, GPL 3 no longer applies.

	Under those circumstances, the author expects and may legally pursue a reasoble share of the income. To avoid the complexity of agreements and negotiation, the author makes
	no specific demands in this regard. Compensation of roughly 1% of net or $5 per user license seems appropriate, but is not legally binding.

	In lay terms, if you make a profit by using the DistFieldHexMesh application/library (violating the spirit of Open Source Software), I expect a reasonable share for my efforts.

	Robert R Tipton - Author

	Dark Sky Innovative Solutions http://darkskyinnovation.com/
*/

#define _USE_MATH_DEFINES
#include <cmath>
#include <iostream>
#include <tm_math.h>
#include <vertex.h>
#include <edge.h>
#include <polygon.h>
#include <block.h>

using namespace std;
using namespace DFHM;

Polygon::Polygon(const vector<Index3DId>& verts)
	: _vertexIds(verts)
{
}

size_t Polygon::numSplits() const
{
	size_t result = 0;
	auto parent = _parent;
	while (parent.isValid()) {
		result++;
		Index3DId nextParent;
		faceFunc(parent, [&nextParent](const Polygon& face) {
			nextParent = face.getParentId();
		});
		parent = nextParent;
	}

	return result;
}

bool Polygon::tooManyChildLevels() const
{
	size_t depth = _children.empty() ? 0 : 1;
	for (const auto& childId : _children) {
		faceFunc(childId, [&depth](const Polygon& face) {
			if (!face._children.empty()) {
				depth++;
			}
		});
		if (depth > 1)
			return true;
	}
	return false;
}

void Polygon::addVertex(const Index3DId& vertId)
{
	if (_children.empty()) {
		_vertexIds.push_back(vertId);
		_needSort = true;
	} else
		assert(!"Cannot modify a which has already be split. Create a new face");
}

Index3DId Polygon::getNeighborCellId(const Index3DId& thisCellId) const
{
	Index3DId result;
	if (_cellIds.size() == 2) {
		for (const auto& id : _cellIds) {
			if (id != thisCellId) {
				result = id;
				break;
			}
		}
	}

	return result;
}

bool Polygon::unload(ostream& out, size_t idSelf)
{

	return true;
}

bool Polygon::load(istream& in, size_t idSelf)
{

	return true;
}

void Polygon::sortIds() const
{
	if (_needSort) {
		_sortedIds = _vertexIds;
		sort(_sortedIds.begin(), _sortedIds.end());
		_needSort = false;
	}
}

void Polygon::orient()
{
	Index3DId ownerCellId;
	switch (_cellIds.size()) {
		default:
			return;
		case 1: { // Outer polygon
			ownerCellId = *_cellIds.begin();
			break;
		}
		case 2: { // Inner polygon
			auto iter = _cellIds.begin();
			auto id0 = *iter++;
			auto id1 = *iter;

			ownerCellId = getBlockPtr()->maxCellId(id0, id1);
			break;
		}
	}

	Vector3d norm = calUnitNormal();
	Vector3d faceCtr = calCentroid();
	Vector3d cellCtr;
	cellFunc(ownerCellId, [&cellCtr](const Polyhedron& cell) {
		cellCtr = cell.calCentroid();
		});
	Vector3d v = cellCtr - faceCtr;
	if (v.dot(norm) < Tolerance::paramTol()) {
		reverse(_vertexIds.begin(), _vertexIds.end());
	}
}

void Polygon::pack()
{
	_sortedIds.clear();
}

bool Polygon::operator < (const Polygon& rhs) const
{
	sortIds();
	rhs.sortIds();

	if (_sortedIds.size() < rhs._sortedIds.size())
		return true;
	else if (_sortedIds.size() > rhs._sortedIds.size())
		return false;

	for (size_t i = 0; i < _sortedIds.size(); i++) {
		if (_sortedIds[i] < rhs._sortedIds[i])
			return true;
		else if (rhs._sortedIds[i] < _sortedIds[i])
			return false;
	}
	return false;
}

bool Polygon::isBlockBoundary() const
{
	if (_cellIds.size() == 2) {
		auto iter0 = _cellIds.begin();
		auto iter1 = iter0++;
		return (iter0->blockIdx() != iter1->blockIdx());
	}
	return false;
}

void Polygon::getEdges(set<Edge>& edgeSet) const
{
	createEdgesStat(_vertexIds, edgeSet, _thisId);
}

bool Polygon::containsPt(const Vector3d& pt) const
{
	if (!isPointOnPlane(pt))
		return false;

	Vector3d norm = calUnitNormal();
	for (size_t i = 0; i < _vertexIds.size(); i++) {
		size_t j = (i + 1) % _vertexIds.size();

		Vector3d pt0 = getBlockPtr()->getVertexPoint(_vertexIds[i]);
		Vector3d pt1 = getBlockPtr()->getVertexPoint(_vertexIds[j]);
		Vector3d v1 = (pt1 - pt0).normalized();
		Vector3d v0 = pt - pt0;
		v0 = v0 - v0.dot(v1) * v1;
		Vector3d cp = v1.cross(v0);
		double dist = norm.dot(cp);
	
		if (dist < -Tolerance::sameDistTol())
			return false;
	}
	return true;
}

bool Polygon::isPointOnPlane(const Vector3d& pt) const {
	return distFromPlane(pt) < Tolerance::sameDistTol();
}

bool Polygon::containsEdge(const Edge& edge) const
{
	size_t idx0, idx1;
	return containsEdge(edge, idx0, idx1);
}

bool Polygon::containsEdge(const Edge& edge, size_t& idx0, size_t& idx1) const
{
	for (size_t i = 0; i < _vertexIds.size(); i++) {
		size_t j = (i + 1) % _vertexIds.size();
		const auto& vertId0 = _vertexIds[i];
		const auto& vertId1 = _vertexIds[j];
		Edge testEdge(vertId0, vertId1);
		if (testEdge == edge) {
			idx0 = i;
			idx1 = j;
			return true;
		}
	}

	idx0 = idx1 = -1;
	return false;
}

bool Polygon::containsVert(const Index3DId& vertId) const
{
	for (const auto& id : _vertexIds) {
		if (id == vertId)
			return true;
	}
	return false;
}

bool Polygon::isActive() const
{
	bool result = false;
	for (const auto& cellId : _cellIds) {
		cellFunc(cellId, [&result](const Polyhedron& cell) {
			result = cell.isActive() || result;
		});
	}
	return result;
}

void Polygon::createEdgesStat(const vector<Index3DId>& verts, set<Edge>& edgeSet, const Index3DId& polygonId)
{
	for (size_t i = 0; i < verts.size(); i++) {
		size_t j = (i + 1) % verts.size();
		// If the edges is aready in the set, we get the existing one, not the new one?
		set<Index3DId> faceSet;
		if (polygonId.isValid())
			faceSet.insert(polygonId);
		edgeSet.insert(Edge(verts[i], verts[j], faceSet));
	}
}

Vector3d Polygon::calUnitNormalStat(const Block* pBlock, const vector<Index3DId>& vertIds)
{
	Vector3d norm(0, 0, 0);

	for (size_t i = 0; i < vertIds.size() - 2; i++) {
		size_t j = (i + 1) % vertIds.size();
		size_t k = (j + 1) % vertIds.size();

		Vector3d pt0 = pBlock->getVertexPoint(vertIds[i]);
		Vector3d pt1 = pBlock->getVertexPoint(vertIds[j]);
		Vector3d pt2 = pBlock->getVertexPoint(vertIds[k]);

		Vector3d v0 = pt2 - pt1;
		Vector3d v1 = pt0 - pt1;

		Vector3d n = v0.cross(v1);
		norm += n;
	}
	norm.normalize();
	return norm;
}

Vector3d Polygon::calUnitNormal() const
{
	return calUnitNormalStat(getBlockPtr(), _vertexIds);
}

double Polygon::calVertexAngleStat(const Block* pBlock, const vector<Index3DId>& vertIds, size_t idx1)
{
	const size_t sz = vertIds.size();
	if (idx1 < sz) {
		Vector3d norm = calUnitNormalStat(pBlock, vertIds);
		size_t idx0 = (idx1 + sz - 1) % sz;
		size_t idx2 = (idx1 + 1) % sz;

		Vector3d pt0 = pBlock->getVertexPoint(vertIds[idx0]);
		Vector3d pt1 = pBlock->getVertexPoint(vertIds[idx1]);
		Vector3d pt2 = pBlock->getVertexPoint(vertIds[idx2]);

		Vector3d v0 = pt0 - pt1;
		Vector3d v1 = pt2 - pt1;

		double dp = v1.dot(v0);
		double cp = v1.cross(v0).dot(norm);
		double angle = atan2(cp, dp);
		return angle;
	}

	return nanf("");
}

double Polygon::getShortestEdge() const
{
	double minDist = DBL_MAX;
	for (size_t i = 0; i < _vertexIds.size(); i++) {
		size_t j = (i + 1) % _vertexIds.size();
		Vector3d pt0 = getBlockPtr()->getVertexPoint(_vertexIds[i]);
		Vector3d pt1 = getBlockPtr()->getVertexPoint(_vertexIds[1]);
		double l = (pt1 - pt0).norm();
		if (l < minDist)
			minDist = l;
	}
	return minDist;
}

double Polygon::calVertexAngle(size_t idx1) const
{
	return calVertexAngleStat(getBlockPtr(), _vertexIds, idx1);
}

Vector3d Polygon::interpolatePoint(double t, double u) const
{
	assert(_vertexIds.size() == 4);
	Vector3d pts[] = {
		getBlockPtr()->getVertexPoint(_vertexIds[0]),
		getBlockPtr()->getVertexPoint(_vertexIds[1]),
		getBlockPtr()->getVertexPoint(_vertexIds[2]),
		getBlockPtr()->getVertexPoint(_vertexIds[3]),
	};

	return BI_LERP(pts[0], pts[1], pts[2], pts[3], t, u);
}

Vector3d Polygon::calCentroid() const
{
	double area;
	Vector3d ctr;
	calAreaAndCentroid(area, ctr);
	return ctr;
}

double Polygon::distFromPlane(const Vector3d& pt) const
{
	Plane pl(getBlockPtr()->getVertexPoint(_vertexIds[0]), calUnitNormal());
	return pl.distanceToPoint(pt);
}

void Polygon::calAreaAndCentroid(double& area, Vector3d& centroid) const
{
	set<Edge> edges;
	getEdges(edges);

	vector<Index3DId> orderedVertIds;
	const auto& edge = *edges.begin();
	orderedVertIds.push_back(edge.getVertexIds()[0]);
	orderedVertIds.push_back(edge.getVertexIds()[1]);
	edges.erase(edge);
	while (!edges.empty()) {
		bool found = false;
		for (const auto& edge : edges) {
			if (edge.getVertexIds()[0] == orderedVertIds.back()) {
				orderedVertIds.push_back(edge.getVertexIds()[1]);
				edges.erase(edge);
				found = true;
				break;
			}
			else if (edge.getVertexIds()[1] == orderedVertIds.back()) {
				orderedVertIds.push_back(edge.getVertexIds()[0]);
				edges.erase(edge);
				found = true;
				break;
			}
		}
		assert(found);
	}
	if (orderedVertIds.front() == orderedVertIds.back())
		orderedVertIds.pop_back();
	else
		assert(!"Should never reach this point.");

	area = 0;
	centroid = Vector3d(0, 0, 0);
	Vector3d pt0 = getBlockPtr()->getVertexPoint(orderedVertIds[0]);
	for (size_t j = 1; j < orderedVertIds.size() - 1; j++) {
		size_t k = j + 1;
		Vector3d pt1 = getBlockPtr()->getVertexPoint(orderedVertIds[j]);
		Vector3d pt2 = getBlockPtr()->getVertexPoint(orderedVertIds[k]);
		Vector3d v0 = pt0 - pt1;
		Vector3d v1 = pt2 - pt1;
		Vector3d triCtr = (pt0 + pt1 + pt2) * (1.0 / 3.0);
		Vector3d cp = v1.cross(v0);
		double triArea = cp.norm() / 2.0;

		centroid += triCtr * triArea;
		area += triArea;
	}

	centroid /= area;
}

Vector3d Polygon::projectPoint(const Vector3d& pt) const
{
	Vector3d origin = getBlockPtr()->getVertexPoint(_vertexIds[0]); // And point will do
	Vector3d normal = calUnitNormal();
	Plane pl(origin, normal);
	auto result = pl.projectPoint(pt);

	return result;
}

void Polygon::addCellId(const Index3DId& cellId)
{
	_cellIds.insert(cellId);
	assert(_cellIds.size() <= 2);
}

bool Polygon::splitAtPoint(const Vector3d& pt, set<Index3DId>& newFaceIds, bool dryRun)
{
	if (!_children.empty()) {
		if (_children.size() == 1) {
			// This face has split edges, but no split faces
			if (!dryRun) {
				// This face needs to be resplit and replaced. Delete it and clear it
				getBlockPtr()->freePolygon(*_children.begin());
				_children.clear();
			}
		} else {
			newFaceIds = _children;
			return true;
		}
	}

	newFaceIds.clear();
	assert(_vertexIds.size() == 4);
	vector<Vector3d> edgePts;
	edgePts.resize(_vertexIds.size());
	for (size_t i = 0; i < _vertexIds.size(); i++) {
		size_t j = (i + 1) % _vertexIds.size();
		Edge edge(_vertexIds[i], _vertexIds[j]);

		// Be sure to project directly to the edge itself. 
		// DO NOT project to the face followed by the edge, because that can result in two points on the same edge.
		Vector3d edgePt = edge.projectPt(getBlockPtr(), pt);
		bool inBounds;
		double t = edge.paramOfPt(getBlockPtr(), edgePt, inBounds);
		if (inBounds)
			edgePts[i] = edgePt;
		else
			return false;
	}

	Vector3d facePt = projectPoint(pt);
	if (!containsPt(facePt))
		return false;

	if (dryRun)
		return true; // Report that everything is good to go and return without touching anything

	Index3DId facePtId = getBlockPtr()->addVertex(facePt);
	vector<Index3DId> edgePtIds;
	for (const auto& edgePt : edgePts) {
		edgePtIds.push_back(getBlockPtr()->addVertex(edgePt));
	}

	for (size_t i = 0; i < _vertexIds.size(); i++) {
		size_t j = (i + _vertexIds.size() - 1) % _vertexIds.size();
		auto priorEdgeId = edgePtIds[j];
		auto vertId = _vertexIds[i];
		auto nextEdgeId = edgePtIds[i];
		Polygon face({ facePtId, priorEdgeId, vertId, nextEdgeId});
		face.setParentId(_thisId);

		// The split face separates the same cells as the original, until new cells are created. They 
		// The original cell must replace its id the id of a child cell when the child cell is created
		face.setCellIds(_cellIds);

		auto newFaceId = createFace(face);
		newFaceIds.insert(newFaceId);
	}
	assert(newFaceIds.size() == 4);
	faceFunc(_thisId, [&newFaceIds](Polygon& writableThis) {
		writableThis.setChildIds(newFaceIds);
	});

	return true;
}

bool Polygon::imprintFaceVertices(const Polygon& otherFace)
{
	bool result = false;
	set<Edge> edgeSet;
	getEdges(edgeSet);
	for (const auto& otherVert : otherFace.getVertexIds()) {
		for (const auto& edge : edgeSet) {
			result = imprintVertex(otherVert, edge);
			if (result)
				break;
		}
		edgeSet.clear();
		getEdges(edgeSet);
	}
	return result;
}

bool Polygon::imprintVertex(const Index3DId& vertId, const Edge& edge)
{
	bool inBounds;
	size_t i, j;
	if (!containsVert(vertId) && containsEdge(edge, i, j) && edge.isColinearWith(getBlockPtr(), vertId, inBounds) && inBounds) {
		// the vertex is not in this polygon and lies between i and j
		getBlockPtr()->removeFaceFromLookUp(_thisId);

		_vertexIds.insert(_vertexIds.begin() + j, vertId);
		_needSort = true;

		getBlockPtr()->addFaceToLookup(_thisId);
		return true;
	}
	return false;
}

bool Polygon::imprintVertices(const vector<Index3DId>& vertIdsIn, const vector<Edge>& edgesIn)
{
	bool result = false;
	if (vertIdsIn.size() > 1) {
		int dbgBreak = 1;
	}

	vector<Index3DId> vertIds(vertIdsIn);
	vector<Edge> edges(edgesIn);

	while (!vertIds.empty()) {
		auto vertId = vertIds.back();
		auto edge = edges.back();
		vertIds.pop_back();
		edges.pop_back();
		result = imprintVertex(vertId, edge) || result;
	}
	return result;
}

Index3DId Polygon::createFace(const Polygon& face)
{
	return getBlockPtr()->addFace(face);
}

void Polygon::setChildIds(const set<Index3DId>& childFaceIds)
{
	_children = childFaceIds;
}

bool Polygon::verifyVertsConvexStat(const Block* pBlock, const vector<Index3DId>& vertIds)
{
	for (size_t i = 0; i < vertIds.size(); i++) {
		double angle = calVertexAngleStat(pBlock, vertIds, i);
		if (angle < 0 && angle > M_PI)
			return false;
	}

	return true;
}

bool Polygon::verifyUniqueStat(const vector<Index3DId>& vertIds)
{
	bool valid = true;

	for (size_t i = 0; i < vertIds.size(); i++) {
		for (size_t j = i + 1; j < vertIds.size(); j++) {
			if (vertIds[i] == vertIds[j])
				valid = false;
		}
	}
	return valid;
}

bool Polygon::verifyTopology() const
{
	bool valid = true;
#ifdef _DEBUG 
	vector<Index3DId> vertIds;
	faceFunc(_thisId, [&vertIds](const Polygon& face) {
		vertIds = face.getVertexIds();
	});

	if (!verifyUniqueStat(vertIds))
		valid = false;

	if (_cellIds.size() > 2)
		valid = false;

	set<Edge> edges;
	getEdges(edges);
	for (const auto& edge : edges) {
		auto faceIds = edge.getFaceIds();
		if (faceIds.count(_thisId) == 0) // edge does not link back to this face
			valid = false;
	}

	for (const auto& cellId : _cellIds) {
		if (!getBlockPtr()->polyhedronExists(cellId))
			valid = false;
	}
#endif
	return valid;
}
