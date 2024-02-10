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

Polygon::Polygon(const Polygon& src)
	: ObjectPoolOwnerUser(src)
	, _numSplits(src._numSplits)
	, _vertexIds(src._vertexIds)
	, _cellIds(src._cellIds)
	, _needSort(true)
{
}

Polygon& Polygon::operator = (const Polygon& rhs)
{
	ObjectPoolOwnerUser::operator=(rhs);

	_numSplits = rhs._numSplits;
	_vertexIds = rhs._vertexIds;
	_cellIds = rhs._cellIds;
	_needSort = true;

	return *this;
}

void Polygon::addVertex(const Index3DId& vertId)
{
	Block* pBlock = getOutBlockPtr(_thisId);
	if (pBlock) {
		pBlock->faceFunc(_thisId, [this, pBlock, &vertId](Polygon& face) {
			if (!face.containsVert(vertId)) {
				pBlock->removeFaceFromLookUp(face._thisId);

				face._vertexIds.push_back(vertId);

				pBlock->addFaceToLookup(face._thisId);
			}
			face._needSort = true;
		});
	} else {
		_vertexIds.push_back(vertId);
		_needSort = true;
	}
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

void Polygon::getEdges(std::set<Edge>& edgeSet) const
{
	for (size_t i = 0; i < _vertexIds.size(); i++) {
		size_t j = (i + 1) % _vertexIds.size();
		// If the edges is aready in the set, we get the existing one, not the new one?
		set<Index3DId> faceSet;
		faceSet.insert(_thisId);
		auto result = edgeSet.insert(Edge(_vertexIds[i], _vertexIds[j], faceSet));
		// Set entries are const to prevent breaking the sort order
		// The _faceIds do not affect the sort, so this is safe
		// We cannot do this during creation, because we must accumulate the faces as the are added to the set
	}
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

Vector3d Polygon::calUnitNormalStat(const Block* pBlock, const std::vector<Index3DId>& vertIds)
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

double Polygon::calVertexAngleStat(const Block* pBlock, const std::vector<Index3DId>& vertIds, size_t idx1)
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

void Polygon::getPrincipalEdges(std::set<Edge>& edges) const
{

}

void Polygon::calAreaAndCentroid(double& area, Vector3d& centroid) const
{
	set<Edge> edges;
	getPrincipalEdges(edges);

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
	Vector3d ctr = calCentroid();
	Vector3d norm = calUnitNormal();
	Vector3d v = pt - ctr;
	double dp = v.dot(norm);
	Vector3d result = pt - dp * norm;

	return result;
}

Index3DId Polygon::insertVertexInEdge(const Edge& edge, const Vector3d& pt)
{
	Index3DId newVertId;
	set<Edge> edgeSet;
	getEdges(edgeSet);
	if (edgeSet.count(edge) != 0) {
		newVertId = getBlockPtr()->idOfPoint(pt);
		if (!newVertId.isValid())
			newVertId = getOutBlockPtr(_thisId)->addVertex(pt);

		if (!containsVert(newVertId))
			insertVertexInEdge(edge, newVertId);
	}
	return newVertId;
}

bool Polygon::insertVertexInEdge(const Edge& edge, const Index3DId& newVertId)
{
	assert(verifyUnique());
	assert(!containsVert(newVertId));
	bool result = false;
	size_t idx0 = -1, idx1 = 1;
	if (containsEdge(edge, idx0, idx1)) { // This face does not already contain the new edge
		vector<Index3DId> vertIds = _vertexIds;

		if (idx1 < vertIds.size())
			vertIds.insert(vertIds.begin() + idx1, newVertId);
		else {
			// New vertex goes at the end
			vertIds.push_back(newVertId);
		}
		result = true;

		assert(verifyUniqueStat(vertIds));

		setVertexIds(vertIds);
	}
	set<Edge> edges;
	getEdges(edges);
	for (const auto& edge : edges) {
		assert(edge.onPrincipalAxis(getBlockPtr()));
	}
	return result;
}

namespace
{

struct SplitEdgeRec
{
	inline SplitEdgeRec(const Edge& edge, const Index3DId& splitVertId)
		: _edge(edge)
		, _splitVertId(splitVertId)
	{
	}

	Edge _edge;
	Index3DId _splitVertId;
};

}

bool Polygon::isAbovePlane(const Plane& splitPlane, double tol) const
{
	bool allAbove = true;

	getBlockPtr()->faceFunc(_thisId, [this, &splitPlane, &allAbove, tol](const Polygon& face) {
		auto vertIds = face.getVertexIds();
		for (const auto& vertId : vertIds) {
			Vector3d pt = getBlockPtr()->getVertexPoint(vertId);
			Vector3d v = pt - splitPlane._origin;
			double dp = v.dot(splitPlane._normal);
			if (fabs(dp) > tol) {
				if (dp < 0) {
					allAbove = false;
					break;
				}
			}
		}
	});

	return allAbove;
}

Index3DId Polygon::findOtherSplitFaceId(const Edge& edge) const
{
	Index3DId result;

	Vector3d faceCtr = calCentroid();
	Vector3d edgeCtr = edge.calCenter(getBlockPtr());
	Vector3d thisFaceVec = edgeCtr - faceCtr; // This one points from the face center to the edge center
	Vector3d edgeDir = edge.calUnitDir(getBlockPtr());
	thisFaceVec = thisFaceVec - edgeDir.dot(thisFaceVec) * edgeDir;
	thisFaceVec.normalize();

	auto edgeFaceIds = edge.getFaceIds();
	double maxDp = -DBL_MAX;
	for (const auto& faceId : edgeFaceIds) {
		if (faceId == _thisId)
			continue;
		getBlockPtr()->faceFunc(faceId, [this, &thisFaceVec, &result, &edgeCtr, &edgeDir, &maxDp](const Polygon& face) {
			Vector3d faceCtr = face.calCentroid();
			Vector3d testFaceVec = faceCtr - edgeCtr; // This one points from the edge center to the other face center
			testFaceVec = testFaceVec - edgeDir.dot(testFaceVec) * edgeDir;
			testFaceVec.normalize();
			// An ideal value is 1.0. Others should be less
			auto dp = thisFaceVec.dot(testFaceVec);
			if (dp > maxDp) {
				maxDp = dp;
				result = face.getId();
			}
		});
	}

	assert(maxDp > 0.9);
	assert(result.isValid());
	return result;
}

Index3DId Polygon::splitWithFaceEdges(const Polygon& splittingFace)
{
	assert(verifyTopology());
	assert(splittingFace.verifyTopology());

	const auto& thisFace = *this;
	set<Edge> otherEdges;
	splittingFace.getEdges(otherEdges);
	for (const auto& splittingEdge : otherEdges) {
		assert(splittingEdge.onPrincipalAxis(getBlockPtr()));
		if (containsEdge(splittingEdge)) {
			// Our face already contains the splitting edge. It cannot be split again
			Index3DId existingSplitFaceId = findOtherSplitFaceId(splittingEdge);
			if (existingSplitFaceId.isValid())
				return existingSplitFaceId;
		}
		Index3DId vertId0 = splittingEdge.getVertexIds()[0];
		Index3DId vertId1 = splittingEdge.getVertexIds()[1];
		if (thisFace.containsVert(vertId0) && thisFace.containsVert(vertId1)) {
			// other face does not have this edge but has both vertices to form an edge
			size_t idx0 = -1, idx1 = -1;
			for (size_t i = 0; i < thisFace._vertexIds.size(); i++) {
				const auto& vertId = thisFace._vertexIds[i];
				if ((vertId == vertId0) || (vertId == vertId1)) {
					if (idx0 == -1)
						idx0 = i;
					else
						idx1 = i;
				}
			}

			vector<Index3DId> face0Verts, face1Verts;

			for (size_t i = 0; i < thisFace._vertexIds.size(); i++) {
				size_t index = (i + idx0) % thisFace._vertexIds.size();
				face0Verts.push_back(thisFace._vertexIds[index]);
				if (thisFace._vertexIds[index] == thisFace._vertexIds[idx1])
					break;
			}

			for (size_t i = 0; i < thisFace._vertexIds.size(); i++) {
				size_t index = (i + idx1) % thisFace._vertexIds.size();
				face1Verts.push_back(thisFace._vertexIds[index]);
				if (thisFace._vertexIds[index] == thisFace._vertexIds[idx0])
					break;
			}

			if (face1Verts.size() < face0Verts.size())
				swap(face0Verts, face1Verts);

			assert(verifyUniqueStat(face0Verts));
			assert(verifyUniqueStat(face1Verts));
			assert(verifyVertsConvexStat(getBlockPtr(), face0Verts));
			assert(verifyVertsConvexStat(getBlockPtr(), face1Verts));

			setVertexIds(face0Verts);

			Index3DId newFaceId = getOutBlockPtr(_thisId)->addFace(face1Verts);

			for (const auto& cellId : _cellIds) { // TODO Remove this and let the splitter do it
				getOutBlockPtr(_thisId)->addFaceToPolyhedron(newFaceId, cellId);
			}

#if 1 && defined(_DEBUG)
			// assert the vertex to face back linkage
			assert(verifyTopology());
			assert(splittingFace.verifyTopology());


			auto splittingEdgeFaces = splittingEdge.getFaceIds();
			assert(splittingEdgeFaces.count(_thisId) != 0);
			assert(splittingEdgeFaces.count(newFaceId) != 0);
#endif // _DEBUG
			return newFaceId;
		}
	}

	return Index3DId();
}

void Polygon::setVertexIds(const vector<Index3DId>& verts)
{
	assert(verifyUnique());

	getOutBlockPtr(_thisId)->removeFaceFromLookUp(_thisId);

	_vertexIds = verts;
	_needSort = true;

	getOutBlockPtr(_thisId)->addFaceToLookup(_thisId);
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
	getBlockPtr()->faceFunc(_thisId, [&vertIds](const Polygon& face) {
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
