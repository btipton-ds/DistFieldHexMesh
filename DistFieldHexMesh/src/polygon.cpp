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
	, _splitFromFaceId(src._splitFromFaceId)
	, _vertexIds(src._vertexIds)
	, _cellIds(src._cellIds)
	, _needSort(true)
{
}

Polygon& Polygon::operator = (const Polygon& rhs)
{
	ObjectPoolOwnerUser::operator=(rhs);

	_numSplits = rhs._numSplits;
	_splitFromFaceId = rhs._splitFromFaceId;
	_vertexIds = rhs._vertexIds;
	_cellIds = rhs._cellIds;
	_needSort = true;

	return *this;
}

void Polygon::addVertex(const Index3DId& vertId)
{
	if (getBlockPtr()) {
		getBlockPtr()->faceFunc(_thisId, [this, &vertId](Polygon& face) {
			if (!face.containsVert(vertId)) {
				getBlockPtr()->removeFaceFromLookUp(face._thisId);

				face._vertexIds.push_back(vertId);

				getBlockPtr()->addFaceToLookup(face._thisId);
			}
			face._needSort = true;
		});
	} else {
		_vertexIds.push_back(vertId);
		_needSort = true;
	}
}

void Polygon::setSplitFromData(const Index3DId& sourceFaceId, const std::set<Index3DId>& sourceCellIds)
{
	_numSplits++;
	_splitFromFaceId = sourceFaceId;
	_cellIds = sourceCellIds;
}

void Polygon::clearSplitFromId()
{
	_splitFromFaceId = Index3DId();
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

set<Edge> Polygon::getEdges() const
{
	set<Edge> result;

	getBlockPtr()->faceFunc(_thisId, [&result](const Polygon& self) {
		result = self.getEdgesNTS();
	});

	return result;
}

set<Edge> Polygon::getEdgesNTS() const
{
	set<Edge> result;

	for (size_t i = 0; i < _vertexIds.size(); i++) {
		size_t j = (i + 1) % _vertexIds.size();
		const auto& vertId0 = _vertexIds[i];
		const auto& vertId1 = _vertexIds[j];
		Edge edge(getBlockPtr(), vertId0, vertId1);
		result.insert(edge);
	}

	return result;
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
		Edge testEdge(getBlockPtr(), vertId0, vertId1);
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

bool Polygon::vertsContainFace() const
{
	bool result = true;
	for (const auto& vertId : _vertexIds) {
		getBlockPtr()->vertexFunc(vertId, [this, &result](const Vertex& vert) {
			bool pass = vert.connectedToFace(_thisId);
			if (!pass)
				result = false;
		});
	}
	return result;
}

bool Polygon::ownedByCellNTS(const Index3DId& cellId) const
{
	return _cellIds.contains(cellId);
}

bool Polygon::wasSplitFromNTS(const Index3DId& faceId) const
{
	return _splitFromFaceId == faceId;
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
	Vector3d ctr(0, 0, 0);
	for (const auto& vertId : _vertexIds) {
		Vector3d pt = getBlockPtr()->getVertexPoint(vertId);
		ctr += pt;
	}

	ctr /= (double)_vertexIds.size();
	return ctr;
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

Index3DId Polygon::insertVertexInEdgeNTS(const Edge& edge, const Vector3d& pt)
{
	Index3DId newVertId;
	auto edgeSet = getEdges();
	if (edgeSet.count(edge) != 0) {
		newVertId = getBlockPtr()->idOfPoint(pt);
		if (!newVertId.isValid())
			newVertId = getBlockPtr()->addVertex(pt);

		if (!containsVert(newVertId))
			insertVertexInEdgeNTS(edge, newVertId);
	}
	return newVertId;
}

bool Polygon::insertVertexInEdgeNTS(const Edge& edge, const Index3DId& newVertId)
{
	assert(vertifyUnique());
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

		assert(vertifyUniqueStat(vertIds));

		setVertexIdsNTS(vertIds);

		if (result) {
			getBlockPtr()->vertexFunc(newVertId, [this](Vertex& vert) {
				vert.addFaceId(_thisId);
				});
		}
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
#if 1
	assert(containsEdge(edge));
	size_t numFound = 0;
	auto edgeFaceIds = edge.getFaceIds();
	for (const auto& otherFaceId : edgeFaceIds) {
		bool found;
		getBlockPtr()->faceFunc(otherFaceId, [this, &found](const Polygon& otherFace) {
			found = (otherFace.wasSplitFromNTS(_thisId) || wasSplitFromNTS(otherFace._thisId));
		});

		if (found) {
			result = otherFaceId;
			numFound++;
		}
	}

	assert(numFound == 1);
	if (numFound == 1 && result.isValid())
		return result;
	return Index3DId();
#else
	Vector3d ourNormal = calUnitNormal();
	auto edgeFaceIds = edge.getFaceIds();
	for (const auto& faceId : edgeFaceIds) {
		getBlockPtr()->faceFunc(faceId, [this, &ourNormal, &result](const Polygon& face) {
			const double tol = 1.0e-5;
			Vector3d testNormal = face.calUnitNormal();
			if (fabs(1 - testNormal.dot(testNormal)) < tol) {
				result = face.getId();
			}
		});
		if (result.isValid())
			break;
	}
#endif
	return result;
}

Index3DId Polygon::splitWithFaceEdgesNTS(const Polygon& splittingFace)
{
	assert(verifyTopology());
	assert(splittingFace.verifyTopology());

	const auto& thisFace = *this;
	auto otherEdges = splittingFace.getEdges();
	for (const auto& splittingEdge : otherEdges) {
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

			assert(vertifyUniqueStat(face0Verts));
			assert(vertifyUniqueStat(face1Verts));
			assert(verifyVertsConvexStat(getBlockPtr(), face0Verts));
			assert(verifyVertsConvexStat(getBlockPtr(), face1Verts));

			setVertexIdsNTS(face0Verts);

			Index3DId newFaceId = getBlockPtr()->addFace(face1Verts);
			auto& newFace = getBlockPtr()->getPolygon_UNSAFE(newFaceId); // This is safe because we know the new face is in the block and no one can use it yet.
			newFace.setSplitFromData(_thisId, _cellIds);

			for (const auto& cellId : _cellIds) { // TODO Remove this and let the splitter do it
				getBlockPtr()->addFaceToPolyhedron(newFaceId, cellId);
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

void Polygon::setVertexIdsNTS(const vector<Index3DId>& verts)
{
#ifdef _DEBUG
	for (size_t i = 0; i < verts.size(); i++) {
		for (size_t j = i + 1; j < verts.size(); j++) {
			if (verts[i] == verts[j]) {
				assert(!"duplicate vertex in polygon");
			}
		}
	}
#endif

	getBlockPtr()->removeFaceFromLookUp(_thisId);

	{
		assert(_thisId.blockIdx() == getBlockPtr()->getBlockIdx());

		for (const auto& vertId : _vertexIds) {
			getBlockPtr()->vertexFunc(vertId, [this](Vertex& vert) {
				vert.removeFaceId(_thisId);
				});
		}

		_vertexIds = verts;
		_needSort = true;

		for (const auto& vertId : _vertexIds) {
			getBlockPtr()->vertexFunc(vertId, [this](Vertex& vert) {
				vert.addFaceId(_thisId);
				});
		}
	}

	getBlockPtr()->addFaceToLookup(_thisId);
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

bool Polygon::vertifyUniqueStat(const vector<Index3DId>& vertIds)
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

	if (!vertifyUniqueStat(vertIds))
		valid = false;

	for (const auto& vertId : vertIds) {
		getBlockPtr()->vertexFunc(vertId, [this, &valid](const Vertex& vert) {
			bool pass = vert.connectedToFace(_thisId) && valid;
			if (!pass)
				valid = false;
		});
	}

	if (_cellIds.size() > 2)
		valid = false;

	auto edges = getEdges();
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
