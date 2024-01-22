#include <iostream>
#include <vertex.h>
#include <edge.h>
#include <polygon.h>
#include <block.h>

using namespace std;
using namespace DFHM;

void Polygon::addVertex(const Index3DId& vertId)
{
	assert(find(_vertexIds.begin(), _vertexIds.end(), vertId) == _vertexIds.end());
	_vertexIds.push_back(vertId);
}

bool Polygon::unload(std::ostream& out, size_t idSelf)
{

	return true;
}

bool Polygon::load(std::istream& in, size_t idSelf)
{

	return true;
}

void Polygon::doneCreating()
{
	_sortedIds = _vertexIds;
	sort(_sortedIds.begin(), _sortedIds.end());
}

void Polygon::pack()
{
	_sortedIds.clear();
}

bool Polygon::operator < (const Polygon& rhs) const
{
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

	for (size_t i = 0; i < _vertexIds.size(); i++) {
		size_t j = (i + 1) % _vertexIds.size();
		const auto& vertId0 = _vertexIds[i];
		const auto& vertId1 = _vertexIds[j];
		Edge edge(vertId0, vertId1);
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
		Edge testEdge(vertId0, vertId1);
		if (testEdge == edge)
			return true;
	}

	return false;
}

Vector3d Polygon::getUnitNormal(const Block* pBlock) const
{
	Vector3d norm(0, 0, 0);

	for (size_t i = 0; i < _vertexIds.size() - 2; i++) {
		size_t j = (i + 1) % _vertexIds.size();
		size_t k = (j + 1) % _vertexIds.size();

		Vector3d pt0 = pBlock->getVertexPoint(_vertexIds[i]);
		Vector3d pt1 = pBlock->getVertexPoint(_vertexIds[j]);
		Vector3d pt2 = pBlock->getVertexPoint(_vertexIds[k]);

		Vector3d v0 = pt2 - pt1;
		Vector3d v1 = pt0 - pt1;

		Vector3d n = v0.cross(v1);
		norm += n;
	}
	norm.normalize();
	return norm;
}

Vector3d Polygon::getPointAt(const Block* pBlock, double t, double u) const
{
	Vector3d pts[] = {
		pBlock->getVertexPoint(_vertexIds[0]),
		pBlock->getVertexPoint(_vertexIds[1]),
		pBlock->getVertexPoint(_vertexIds[2]),
		pBlock->getVertexPoint(_vertexIds[3]),
	};

	return BI_LERP(pts[0], pts[1], pts[2], pts[3], t, u);
}

Vector3d Polygon::getCentroid(const Block* pBlock) const
{
	return getPointAt(pBlock, 0.5, 0.5);
}

Vector3d Polygon::projectPoint(const Block* pBlock, const Vector3d& pt) const
{
	Vector3d ctr = getCentroid(pBlock);
	Vector3d norm = getUnitNormal(pBlock);
	Vector3d v = pt - ctr;
	v = v - norm * v.dot(norm);
	Vector3d result = ctr + v;

	return result;
}

bool Polygon::insertVertex(Block* pBlock, const Index3DId& vert0, const Index3DId& vert1, const Index3DId& newVert)
{
	size_t idx0 = -1, idx1 = -1;
	for (size_t i = 0; i < _vertexIds.size(); i++) {
		if (_vertexIds[i] == vert0)
			idx0 = i;
		else if (_vertexIds[i] == vert1)
			idx1 = i;
	}
	if (idx0 < idx1) { // Normal order
		assert(idx1 == idx0 + 1);
		_vertexIds.insert(_vertexIds.begin() + idx0, newVert);
	} else {
	}
	return false;
}

Index3DId Polygon::insertVertex(Block* pBlock, const Index3DId& vert0, const Index3DId& vert1, const Vector3d& pt)
{
	auto newVertId = pBlock->addVertex(pt);
	insertVertex(pBlock, vert0, vert1, newVertId);

	return newVertId;
}

Index3DId Polygon::insertVertex(Block* pBlock, const Index3DId& vert0, const Index3DId& vert1, double t)
{
	Vector3d pt0 = pBlock->getVertexPoint(vert0);
	Vector3d pt1 = pBlock->getVertexPoint(vert0);
	Vector3d pt = pt0 + t * (pt1 - pt0);
	return insertVertex(pBlock, vert0, vert1, pt);
}
