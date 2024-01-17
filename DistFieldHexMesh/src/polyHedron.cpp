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

CBoundingBox3Dd Polyhedron::getBoundingBox(const Block* pBlock) const
{
	CBoundingBox3Dd bbox;
	auto cornerIds = getCornerIds(pBlock);
	for (const auto& vertId : cornerIds) {
		bbox.merge(pBlock->getVertexPoint(vertId));
	}

	return bbox;
}

void Polyhedron::split(const Block* pBlock, const Vector3d& pt)
{
	map<Index3DId, Vector3d> edgePts, facePts;

	for (const auto& faceId : _faceIds) {
		pBlock->faceFunc(faceId, [&faceId, &facePts, &pt](const Block* pBlock, const Polygon& face) {
			auto projPt = face.projectPoint(pBlock, pt);
			facePts.insert(make_pair(faceId, projPt));
		});
	}

	auto edges = getEdges(pBlock);
	for (const auto& edge : edges) {
		Vector3d projPt = edge.projectPt(pBlock, pt);
	}

	// Make a new hexCell at each corner
	auto corners = getCornerIds(pBlock);
	for (const auto& vertId : corners) {
		set<Index3DId> vertEdges;
		auto edges = pBlock->getVertexEdges(vertId);
	}

}
