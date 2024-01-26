#include <iostream>
#include <tm_math.h>
#include <vertex.h>
#include <edge.h>
#include <polygon.h>
#include <polyhedron.h>
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
	if (find(_vertexIds.begin(), _vertexIds.end(), vertId) == _vertexIds.end()) {
		_vertexIds.push_back(vertId);
		_needSort = true;
	} else
		assert(!"Adding duplicate vertex");
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

vector<Edge> Polygon::getEdges() const
{
	vector<Edge> result;

	faceFuncSelf([this, &result]() {
		result = getEdgesNTS();
	});

	return result;
}

vector<Edge> Polygon::getEdgesNTS() const
{
	vector<Edge> result;

	for (size_t i = 0; i < _vertexIds.size(); i++) {
		size_t j = (i + 1) % _vertexIds.size();
		const auto& vertId0 = _vertexIds[i];
		const auto& vertId1 = _vertexIds[j];
		Edge edge(_pBlock, vertId0, vertId1);
		result.push_back(edge);
	}

	return result;
}

bool Polygon::containsEdge(const Edge& edge) const
{
	for (size_t i = 0; i < _vertexIds.size(); i++) {
		size_t j = (i + 1) % _vertexIds.size();
		const auto& vertId0 = _vertexIds[i];
		const auto& vertId1 = _vertexIds[j];
		Edge testEdge(_pBlock, vertId0, vertId1);
		if (testEdge == edge)
			return true;
	}

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

Vector3d Polygon::getUnitNormal() const
{
	Vector3d norm(0, 0, 0);

	for (size_t i = 0; i < _vertexIds.size() - 2; i++) {
		size_t j = (i + 1) % _vertexIds.size();
		size_t k = (j + 1) % _vertexIds.size();

		Vector3d pt0 = _pBlock->getVertexPoint(_vertexIds[i]);
		Vector3d pt1 = _pBlock->getVertexPoint(_vertexIds[j]);
		Vector3d pt2 = _pBlock->getVertexPoint(_vertexIds[k]);

		Vector3d v0 = pt2 - pt1;
		Vector3d v1 = pt0 - pt1;

		Vector3d n = v0.cross(v1);
		norm += n;
	}
	norm.normalize();
	return norm;
}

Vector3d Polygon::getPointAt(double t, double u) const
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

Vector3d Polygon::getCentroid() const
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
	Vector3d ctr = getCentroid();
	Vector3d norm = getUnitNormal();
	Vector3d v = pt - ctr;
	double dp = v.dot(norm);
	Vector3d result = pt - dp * norm;

	return result;
}

bool Polygon::insertVertexNTS(const Index3DId& vert0, const Index3DId& vert1, const Index3DId& newVertId)
{
	if (!containsVert(newVertId))
		return insertVertexNTS(Edge(_pBlock, vert0, vert1), newVertId);
	return false;
}

Index3DId Polygon::insertVertexNTS(const Index3DId& vert0, const Index3DId& vert1, const Vector3d& pt)
{
	// Don't 
	Index3DId newVertId = _pBlock->idOfPoint(pt);
	if (!newVertId.isValid())
		newVertId = _pBlock->addVertex(pt);

	if (!containsVert(newVertId))
		insertVertexNTS(Edge(_pBlock, vert0, vert1), newVertId);

	return newVertId;
}

bool Polygon::insertVertexNTS(const Edge& edge, const Index3DId& newVertId)
{
	assert(!containsVert(newVertId));
	bool result = false;
	size_t idx0 = -1, idx1 = 1;
	for (size_t i = 0; i < _vertexIds.size(); i++) {
		size_t j = (i + 1) % _vertexIds.size();
		const auto& vert0 = _vertexIds[i];
		const auto& vert1 = _vertexIds[j];
		if (
			(vert0 == edge.getVertexIds()[0] && vert1 == edge.getVertexIds()[1]) ||
			(vert0 == edge.getVertexIds()[1] && vert1 == edge.getVertexIds()[0])
			) { // The input edge matches the current vertex pair
			idx0 = i;
			idx1 = j;
			break;
		}
	}
	if (idx0 != -1 && idx1 != -1) {
		if (idx0 > idx1)
			swap(idx0, idx1);

		if (idx1 - idx0 > 1 && idx0 == 0 && idx1 == _vertexIds.size() -1) {
			// New vertex goes at the end
			_vertexIds.push_back(newVertId);
		} else {
			_vertexIds.insert(_vertexIds.begin() + idx1, newVertId);
		}
		result = true;
	}

	if (result) {
		_pBlock->vertexFunc(newVertId, [this](Block* pBlock, Vertex& vert) {
			vert.addFaceId(_thisId);
		});
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

Index3DId Polygon::splitBetweenVertices(const Index3DId& vert0, const Index3DId& vert1)
{
	Index3DId newFaceId;

	faceFuncSelf([this, &vert0, &vert1, &newFaceId]() {
		newFaceId = splitBetweenVerticesNTS(vert0, vert1);
	});

	return newFaceId;
}

Index3DId Polygon::splitBetweenVerticesNTS(const Index3DId& vert0, const Index3DId& vert1)
{
	Index3DId newFaceId;

	size_t idx0 = -1, idx1 = -1;
	for (size_t i = 0; i < _vertexIds.size(); i++) {
		// remove this reference from all vertices
		_pBlock->vertexFunc(_vertexIds[i], [this](Block* pBlock, Vertex& vert) {
			vert.removeFaceId(_thisId);
			});

		const auto& vertId = _vertexIds[i];
		if ((vertId == vert0) || (vertId == vert1)) {
			if (idx0 == -1)
				idx0 = i;
			else
				idx1 = i;
		}
	}
	if ((idx0 == 0 && idx1 == _vertexIds.size() - 1) || ((idx1 - idx0) == 1)) // Insertion points are adjacent, should be a noop
		return Index3DId();

	vector<Index3DId> face0Verts, face1Verts;

	for (size_t i = 0; i < _vertexIds.size(); i++) {
		size_t index = (i + idx0) % _vertexIds.size();
		face0Verts.push_back(_vertexIds[index]);
		if (_vertexIds[index] == _vertexIds[idx1])
			break;
	}

	for (size_t i = 0; i < _vertexIds.size(); i++) {
		size_t index = (i + idx1) % _vertexIds.size();
		face1Verts.push_back(_vertexIds[index]);
		if (_vertexIds[index] == _vertexIds[idx0])
			break;
	}

	if (face1Verts.size() < face0Verts.size())
		swap(face0Verts, face1Verts);

	verifyVertsConvex(_pBlock, face0Verts);
	verifyVertsConvex(_pBlock, face1Verts);

	_pBlock->removeFromLookUp(*this);
	setVertexIds(face0Verts);
	_pBlock->addToLookup(*this);
	_numSplits++;

	newFaceId = _pBlock->addFace(face1Verts);
	_pBlock->faceFunc(newFaceId, [this](Block* pBlock, Polygon& face) {
		// Assign our cellIds to the new face
		face._cellIds = _cellIds;
		face._numSplits++;
		});

	for (const auto& cellId : _cellIds) {
		auto& cell = _pBlock->getPolyhedron(cellId);
		cell.addFace(newFaceId);
	}


	return newFaceId;
}

void Polygon::setVertexIds(const vector<Index3DId>& verts)
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

	for (const auto& vertId : _vertexIds) {
		_pBlock->vertexFunc(vertId, [this](Block* pBlock, Vertex& vert) {
			vert.removeFaceId(_thisId);
			});
	}

	for (const auto& vertId : verts) {
		_pBlock->vertexFunc(vertId, [this](Block* pBlock, Vertex& vert) {
			vert.addFaceId(_thisId);
			});
	}

	assert(_thisId.blockIdx() == _pBlock->getBlockIdx());

	lock_guard g(_pBlock->getFaceMutex());
	_vertexIds = verts;
	_needSort = true;
}

bool Polygon::verifyVertsConvex(const Block* pBlock, const vector<Index3DId>& vertIds)
{
	bool result = true;
#ifdef _DEBUG
	assert(vertIds.size() == 4);
	Vector3d ctr(0, 0, 0);
	for (const auto& vertId : vertIds) {
		Vector3d pt = pBlock->getVertexPoint(vertId);
		ctr += pt;
	}
	ctr /= (double)vertIds.size();

	Vector3d norm0(0, 0, 0);
	for (size_t i = 0; i < vertIds.size(); i++) {
		size_t j = (i + 1) % vertIds.size();
		Vector3d v0 = ctr - pBlock->getVertexPoint(vertIds[i]);
		v0.normalize();
		Vector3d v1 = ctr - pBlock->getVertexPoint(vertIds[j]);
		v1.normalize();
		if (i == 0) {
			norm0 = v1.cross(v0);
		}
		else {
			Vector3d norm1 = v1.cross(v0);

			if (norm0.dot(norm1) <= 0) {
				result = false;
				assert(!"Vertices not convex");
			}
		}
	}
#endif // _DEBUG
	return result;
}

bool Polygon::verifyTopology() const
{
	vector<Index3DId> vertIds;
	faceFuncSelf([this, &vertIds]() {
		vertIds = getVertexIdsNTS();
	});

	bool valid = true;

	for (size_t i = 0; i < vertIds.size(); i++) {
		for (size_t j = i + 1; j < vertIds.size(); j++) {
			if (vertIds[i] == vertIds[j])
				valid = false;

		}
	}
	for (const auto& vertId : vertIds) {
		_pBlock->vertexFunc(vertId, [this, &valid](const Block* pBlock, const Vertex& vert) {
			bool pass = vert.connectedToFace(_thisId) && valid;
			if (!pass)
				valid = false;
		});
	}

	if (_cellIds.size() > 2)
		valid = false;

	for (const auto& cellId : _cellIds) {
		if (!_pBlock->polyhedronExists(cellId))
			valid = false;
	}
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
	lock_guard g(pOwner->getFaceMutex());
	func();
}

template<class LAMBDA>
void Polygon::faceFuncSelf(LAMBDA func)
{
	auto pOwner = _pBlock->getOwner(_thisId);
	lock_guard g(pOwner->getFaceMutex());
	func();
}
