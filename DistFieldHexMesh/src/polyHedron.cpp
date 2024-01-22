#include <vector>
#include <map>

#include <vertex.h>
#include <edge.h>
#include <polygon.h>
#include <polyhedron.h>
#include <block.h>
#include <volume.h>

using namespace std;
using namespace DFHM;

Polyhedron::Polyhedron(const vector<Index3DId>& faceIds)
	: _faceIds(faceIds)
{
}

bool Polyhedron::unload(std::ostream& out)
{
	return true;
}

bool Polyhedron::load(std::istream& out)
{
	return true;
}

bool Polyhedron::operator < (const Polyhedron& rhs) const
{
	if (_faceIds.size() < rhs._faceIds.size())
		return true;
	else if (_faceIds.size() > rhs._faceIds.size())
		return false;

	vector<Index3DId> lhsIndices(_faceIds), rhsIndices(rhs._faceIds);
	sort(lhsIndices.begin(), lhsIndices.begin());
	sort(rhsIndices.begin(), rhsIndices.begin());
	for (size_t i = 0; i < lhsIndices.size(); i++) {
		if (lhsIndices[i] < rhsIndices[i])
			return true;
		else if (rhsIndices[i] < lhsIndices[i])
			return false;
	}

	return false;
}

vector<Index3DId> Polyhedron::getCornerIds(const Block* pBlock) const
{
	set<Index3DId> idSet;
	for (const auto& faceId : _faceIds) {
		pBlock->faceFunc(faceId, [&idSet](const Block* pBlock, const Polygon& face) {
			const auto& vertexIds = face.getVertexIds();
			idSet.insert(vertexIds.begin(), vertexIds.end());
		});
	}

	vector<Index3DId> result;
	result.insert(result.end(), idSet.begin(), idSet.end());

	return result;
}

vector<Edge> Polyhedron::getEdges(const Block* pBlock) const
{
	set<Edge> idSet;
	for (const auto& faceId : _faceIds) {
		pBlock->faceFunc(faceId, [&idSet](const Block* pBlock, const Polygon& face) {
			const auto& edges = face.getEdges();
			idSet.insert(edges.begin(), edges.end());
		});
	}

	vector<Edge> result;
	result.insert(result.end(), idSet.begin(), idSet.end());

	return result;
}

// Gets the edges for a vertex which belong to this polyhedron
set<Edge> Polyhedron::getVertEdges(const Block* pBlock, const Index3DId& vertId) const
{
	auto cellEdges = getEdges(pBlock);
	set<Edge> cellEdgeSet;
	cellEdgeSet.insert(cellEdges.begin(), cellEdges.end());
	auto allVertEdges = pBlock->getVertexEdges(vertId);

	set<Edge> result;
	for (const auto& edge : allVertEdges) {
		if (cellEdgeSet.count(edge))
			result.insert(edge);
	}

	return result;
}

// Gets the faces for a vertex which belong to this polyhedron
set<Index3DId> Polyhedron::getVertFaces(const Block* pBlock, const Index3DId& vertId) const
{
	auto cellFaces = getFaceIds();
	set<Index3DId> cellFaceSet;
	cellFaceSet.insert(cellFaces.begin(), cellFaces.end());
	set<Index3DId> allVertFaces;
	pBlock->vertexFunc(vertId, [&allVertFaces](const Block* pBlock, const Vertex& vert) {
		allVertFaces = vert.getFaceIds();
	});

	set<Index3DId> result;
	for (const auto& faceId : allVertFaces) {
		if (cellFaceSet.count(faceId))
			result.insert(faceId);
	}

	return result;
}

CBoundingBox3Dd Polyhedron::getBoundingBox(const Block* pBlock) const
{
	CBoundingBox3Dd bbox;
	auto cornerIds = getCornerIds(pBlock);
	for (const auto& vertId : cornerIds) {
		bbox.merge(pBlock->getVertexPoint(vertId));
	}

	return bbox;
}

void Polyhedron::split(Block* pBlock, const Vector3d& ctr)
{
#if 0
	// Make a new hexCell at each corner
	auto corners = getCornerIds(pBlock);
	for (const auto& vertId : corners) {
		auto vertFaces = getVertFaces(pBlock, vertId);
		auto vertEdges = getVertEdges(pBlock, vertId);
		assert(vertFaces.size() == 3);
		assert(vertEdges.size() == 3);

		Edge edge0 = *vertEdges.begin(), edge1, edge2;
		vertEdges.erase(edge0);
		assert(vertEdges.size() == 2);
		auto edgeFaces = edge0.getFaceIds(pBlock, vertFaces);
		assert(edgeFaces.size() == 2);

		auto faceId0 = *edgeFaces.begin();
		edgeFaces.erase(faceId0);

		auto faceId1 = *edgeFaces.begin();

		vertFaces.erase(faceId0);
		vertFaces.erase(faceId1);
		auto faceId2 = *vertFaces.begin();

		pBlock->faceFunc(faceId1, [&vertEdges, &edge1](const Block* pBlock, const Polygon& face) {
			auto edges = face.getEdges();
			for (const auto& edge : edges) {
				if (vertEdges.count(edge) != 0) {
					edge1 = edge;
					break;
				}
			}
		});
		vertEdges.erase(edge1);
		edge2 = *vertEdges.begin();

		Vector3d vertPt = pBlock->getVertexPoint(vertId);
		Vector3d facePt0, facePt1, facePt2;
		pBlock->faceFunc(faceId0, [&facePt0](const Block* pBlock, const Polygon& face) {
			facePt0 = face.getCentroid(pBlock);
		});
		pBlock->faceFunc(faceId1, [&facePt1](const Block* pBlock, const Polygon& face) {
			facePt1 = face.getCentroid(pBlock);
		});
		pBlock->faceFunc(faceId2, [&facePt2](const Block* pBlock, const Polygon& face) {
			facePt2 = face.getCentroid(pBlock);
		});
		Vector3d edgePt0 = edge0.getCenter(pBlock);
		Vector3d edgePt1 = edge1.getCenter(pBlock);
		Vector3d edgePt2 = edge1.getCenter(pBlock);

		Vector3d pts[] = {
			facePt0,
			edgePt0,
			facePt1,
			ctr,
			edgePt2,
			vertPt,
			edgePt1,
			facePt2
		};

		pBlock->addHexCell(pts, 2, pBlock->getBlockIdx());
	}
#endif

}
