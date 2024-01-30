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

void Polygon::setId(ObjectPoolOwner* pBlock, size_t id)
{
	_pBlock = dynamic_cast<Block*> (pBlock);
	_thisId = Index3DId(pBlock->getBlockIdx(), id);
	assert(_thisId.isValid());
}

void Polygon::addVertex(const Index3DId& vertId)
{
	if (_pBlock) {
		_pBlock->faceFunc(_thisId, [&vertId](Block* pBlock, Polygon& face) {
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

set<Edge> Polygon::getEdges() const
{
	set<Edge> result;

	faceFuncSelf([this, &result]() {
		result = getEdgesNTS();
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
		Edge edge(_pBlock, vertId0, vertId1);
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
		Edge testEdge(_pBlock, vertId0, vertId1);
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
		_pBlock->vertexFunc(vertId, [this, &result](const Block* pBlock, const Vertex& vert) {
			bool pass = vert.connectedToFace(_thisId);
			if (!pass)
				result = false;
		});
	}
	return result;
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
	return calUnitNormalStat(_pBlock, _vertexIds);
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
	return calVertexAngleStat(_pBlock, _vertexIds, idx1);
}

Vector3d Polygon::interpolatePoint(double t, double u) const
{
	assert(_vertexIds.size() == 4);
	Vector3d pts[] = {
		_pBlock->getVertexPoint(_vertexIds[0]),
		_pBlock->getVertexPoint(_vertexIds[1]),
		_pBlock->getVertexPoint(_vertexIds[2]),
		_pBlock->getVertexPoint(_vertexIds[3]),
	};

	return BI_LERP(pts[0], pts[1], pts[2], pts[3], t, u);
}

Vector3d Polygon::calCentroid() const
{
	Vector3d ctr(0, 0, 0);
	for (const auto& vertId : _vertexIds) {
		Vector3d pt = _pBlock->getVertexPoint(vertId);
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
		newVertId = _pBlock->idOfPoint(pt);
		if (!newVertId.isValid())
			newVertId = _pBlock->addVertex(pt);

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
			_pBlock->vertexFunc(newVertId, [this](Block* pBlock, Vertex& vert) {
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

Index3DId Polygon::findExistingSplitFaceId(const Edge& edge) const
{
	Vector3d ourNormal = calUnitNormal();
	auto edgeFaceIds = edge.getFaceIds();
	Index3DId result;
	for (const auto& faceId : edgeFaceIds) {
		_pBlock->faceFunc(faceId, [this, &ourNormal, &result](const Block* pBlock, const Polygon& face) {
			const double tol = 1.0e-5;
			Vector3d testNormal = face.calUnitNormal();
			if (fabs(1 - testNormal.dot(testNormal)) < tol) {
				result = face.getId();
			}
		});
		if (result.isValid())
			break;
	}

	return result;
}

vector<Index3DId> Polygon::splitWithFaceEdgesNTS(const Polygon& splittingFace)
{
	vector<Index3DId> splitFaceIds;
	splitFaceIds.push_back(_thisId);

	const auto& thisFace = *this;
	auto otherEdges = splittingFace.getEdges();
	auto ourEdges = getEdges();
	for (const auto& splittingEdge : otherEdges) {
		if (ourEdges.count(splittingEdge) != 0) {
			// Our face already contains the splitting edge. It cannot be split again
			Index3DId existingSplitFaceId = findExistingSplitFaceId(splittingEdge);
			if (existingSplitFaceId.isValid())
				splitFaceIds.push_back(existingSplitFaceId);
			else
				assert(!"This should be impossible.");
			return splitFaceIds;
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
			assert(verifyVertsConvexStat(_pBlock, face0Verts));
			assert(verifyVertsConvexStat(_pBlock, face1Verts));

			setVertexIdsNTS(face0Verts);
			_numSplits++;

			Index3DId newFaceId = _pBlock->addFace(face1Verts);
			_pBlock->faceFunc(newFaceId, [this](Block* pBlock, Polygon& face) {
				// Assign our cellIds to the new face
				face._cellIds = _cellIds;
				face._numSplits++;
			});

			for (const auto& cellId : _cellIds) {
				_pBlock->addFaceToPolyhedron(newFaceId, cellId);
			}

#if 1 && defined(_DEBUG)
			// assert the vertex to face back linkage
			for (const auto& vertId : _vertexIds) {
				_pBlock->vertexFunc(vertId, [this](const Block* pBlock, const Vertex& vert) {
					assert(vert.connectedToFace(_thisId));
				});
			}

			for (const auto& vertId : splittingFace._vertexIds) {
				_pBlock->vertexFunc(vertId, [&splittingFace](const Block* pBlock, const Vertex& vert) {
					assert(vert.connectedToFace(splittingFace._thisId));
				});
			}

			auto splittingEdgeFaces = splittingEdge.getFaceIds();
			assert(splittingEdgeFaces.count(_thisId) != 0);
			assert(splittingEdgeFaces.count(newFaceId) != 0);
#endif // _DEBUG

			break;

			splitFaceIds.push_back(newFaceId);
		}
	}


	return splitFaceIds;
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

	_pBlock->removeFaceFromLookUp(_thisId);

	{
		patient_lock_guard g(_pBlock->getMutex());
		assert(_thisId.blockIdx() == _pBlock->getBlockIdx());

		for (const auto& vertId : _vertexIds) {
			_pBlock->vertexFunc(vertId, [this](Block* pBlock, Vertex& vert) {
				vert.removeFaceId(_thisId);
				});
		}

		_vertexIds = verts;
		_needSort = true;

		for (const auto& vertId : _vertexIds) {
			_pBlock->vertexFunc(vertId, [this](Block* pBlock, Vertex& vert) {
				vert.addFaceId(_thisId);
				});
		}
	}

	_pBlock->addFaceToLookup(_thisId);
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
	faceFuncSelf([this, &vertIds]() {
		vertIds = getVertexIdsNTS();
	});

	if (!vertifyUniqueStat(vertIds))
		valid = false;

	for (const auto& vertId : vertIds) {
		_pBlock->vertexFunc(vertId, [this, &valid](const Block* pBlock, const Vertex& vert) {
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
		if (!_pBlock->polyhedronExists(cellId))
			valid = false;
	}
#endif
	return valid;
}

vector<Index3DId> Polygon::getVertexIds() const
{
	vector<Index3DId> result;
	faceFuncSelf([this, &result]() {
		result = getVertexIdsNTS();
	});
	return result;
}

template<class LAMBDA>
void Polygon::faceFuncSelf(LAMBDA func) const
{
	auto pOwner = _pBlock->getOwner(_thisId);
	patient_lock_guard g(pOwner->getMutex());
	func();
}

template<class LAMBDA>
void Polygon::faceFuncSelf(LAMBDA func)
{
	auto pOwner = _pBlock->getOwner(_thisId);
	patient_lock_guard g(pOwner->getMutex());
	func();
}
