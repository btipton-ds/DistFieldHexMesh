#include <iostream>
#include <tm_math.h>
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

vector<Edge> Polygon::getEdges(const Block* pBlock) const
{
	vector<Edge> result;

	pBlock->faceFunc(_ourId, [this, &result](const Block* pBlock, const Polygon& face) {
		const auto& vertIds = face._vertexIds;
		for (size_t i = 0; i < vertIds.size(); i++) {
			size_t j = (i + 1) % vertIds.size();
			const auto& vertId0 = vertIds[i];
			const auto& vertId1 = vertIds[j];
			Edge edge(vertId0, vertId1);
			result.push_back(edge);
		}
	});

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
	double dp = v.dot(norm);
	Vector3d result = pt - dp * norm;

	return result;
}

bool Polygon::insertVertex(Block* pBlock, const Index3DId& vert0, const Index3DId& vert1, const Index3DId& newVertId)
{
	return insertVertex(pBlock, Edge(vert0, vert1), newVertId);
}

Index3DId Polygon::insertVertex(Block* pBlock, const Index3DId& vert0, const Index3DId& vert1, const Vector3d& pt)
{
	auto newVertId = pBlock->addVertex(pt);
	insertVertex(pBlock, Edge(vert0, vert1), newVertId);

	return newVertId;
}

bool Polygon::insertVertex(Block* pBlock, const Edge& edge, const Index3DId& newVertId)
{
	bool result = false;
	Block* pOwner = pBlock->getOwner(_ourId);
	pOwner->faceFunc(_ourId, [this, &edge, &newVertId, &result](Block* pBlock, Polygon& face) {
		auto& vertIds = face._vertexIds;
		size_t idx0 = -1, idx1 = 1;
		for (size_t i = 0; i < vertIds.size(); i++) {
			size_t j = (i + 1) % vertIds.size();
			const auto& vert0 = vertIds[i];
			const auto& vert1 = vertIds[j];
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

			auto pos = vertIds.begin() + idx1; // The location to insert infront of
			vertIds.insert(pos, newVertId);
			result = true;
		}
	});

	if (result) {
		pBlock->vertexFunc(newVertId, [this](Block* pBlock, Vertex& vert) {
			vert.addFaceId(_ourId);
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

Index3DId Polygon::splitWithPlane(Block* pBlock, const Plane& splittingPlane)
{
	Index3DId newFaceId;

	vector<SplitEdgeRec> splitEdges;
	auto edges = getEdges(pBlock);

	for (const auto& edge : edges) {
		const double tol = edge.sameParamTol(pBlock);
		double t = edge.intesectPlaneParam(pBlock, splittingPlane);
		if (t > tol && t < 1.0 - tol) {
			auto vertId = edge.splitAtParam(pBlock, t);
			if (vertId.isValid()) {
				splitEdges.push_back(SplitEdgeRec(edge, vertId));
			}
		}
	}

	if (!splitEdges.empty()) {
		pBlock->faceFunc(_ourId, [this, &splitEdges, &newFaceId](Block* pBlock, Polygon& face) {
			size_t idx0 = -1, idx1 = -1;
			auto& vertIds = face._vertexIds;
			for (size_t i = 0; i < vertIds.size(); i++) {
				const auto& vertId = vertIds[i];
				if ((vertId == splitEdges[0]._splitVertId) || (vertId == splitEdges[1]._splitVertId)) {
					if (idx0 == -1)
						idx0 = i;
					else
						idx1 = i;
				}
			}

			vector<Index3DId> face0Verts, face1Verts;

			for (size_t i = 0; i < vertIds.size(); i++) {
				size_t index = (i + idx0) % vertIds.size();
				face0Verts.push_back(vertIds[index]);
				if (vertIds[index] == vertIds[idx1])
					break;
			}

			for (size_t i = 0; i < vertIds.size(); i++) {
				size_t index = (i + idx1) % vertIds.size();
				face1Verts.push_back(vertIds[index]);
				if (vertIds[index] == vertIds[idx0])
					break;
			}

			if (face1Verts.size() < face0Verts.size())
				swap(face0Verts, face1Verts);

			vertIds = face0Verts;
			newFaceId = pBlock->addFace(face1Verts);
		});
	}

	return newFaceId;
}
