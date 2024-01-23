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
	assert(idSet.size() == 8);
	vector<Index3DId> result;
	result.insert(result.end(), idSet.begin(), idSet.end());

	return result;
}

vector<Edge> Polyhedron::getEdges(const Block* pBlock) const
{
	set<Edge> idSet;
	for (const auto& faceId : _faceIds) {
		pBlock->faceFunc(faceId, [&pBlock , &idSet](const Block* pBlock, const Polygon& face) {
			const auto& edges = face.getEdges(pBlock);
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

Vector3d Polyhedron::calCentroid(const Block* pBlock) const
{
	Vector3d result(0, 0, 0);
	vector<Index3DId> cornerIds = getCornerIds(pBlock);
	for (const auto& vertId : cornerIds) {
		result += pBlock->getVertexPoint(vertId);
	}
	result /= (double) cornerIds.size();

	return result;
}

vector<size_t> Polyhedron::split(Block* pBlock, const Vector3d& ctr, bool intersectingOnly)
{
	vector<size_t> result;
#if 1
	auto edges = getEdges(pBlock);
	vector<Index3DId> newVertIds;
	Plane horizontal(ctr, Vector3d(0, 0, 1));
	for (const auto& edge : edges) {
		auto newVertId = edge.splitWithPlane(pBlock, horizontal);
		if (newVertId.isValid()) {
			newVertIds.push_back(newVertId);
		}
	}

	cout << "Num new verts: " << newVertIds.size() << "\n";

	if (newVertIds.size() >= 3) {
		if (newVertIds.size() > 3) {
		}
		auto newFaceId = pBlock->addFace(newVertIds);
	}	

#else
	// Make a new hexCell at each corner
	vector<Index3DId> corners = getCornerIds(pBlock);
	auto bbox = getBoundingBox(pBlock);
	assert(bbox.contains(ctr));

	for (const auto& vertId : corners) {
		auto vertFaces = getVertFaces(pBlock, vertId);
		if (vertFaces.empty())
			continue;
		auto vertEdges = getVertEdges(pBlock, vertId);
		if (vertFaces.empty()) {
			assert(!"Should not be possible to reach this block.");
			continue;
		}
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
			auto edges = face.getEdges(pBlock);
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
		pBlock->faceFunc(faceId0, [&facePt0, &ctr](const Block* pBlock, const Polygon& face) {
			facePt0 = face.projectPoint(pBlock, ctr);
		});
		pBlock->faceFunc(faceId1, [&facePt1, &ctr](const Block* pBlock, const Polygon& face) {
			facePt1 = face.projectPoint(pBlock, ctr);
		});
		pBlock->faceFunc(faceId2, [&facePt2, &ctr](const Block* pBlock, const Polygon& face) {
			facePt2 = face.projectPoint(pBlock, ctr);
		});

		
		Vector3d edgePt0 = edge0.projectPt(pBlock, ctr);
		Vector3d edgePt1 = edge1.projectPt(pBlock, ctr);
		Vector3d edgePt2 = edge2.projectPt(pBlock, ctr);

		Vector3d pts[] = {
			vertPt,
			edgePt0,
			facePt1,
			edgePt1,
			edgePt2,
			facePt0,
			ctr,
			facePt2
		};

		for (size_t i = 0; i < 8; i++) {
			assert(bbox.contains(pts[i]));
		}
#if 0
		cout << "Polyhedron::split pts:\n";
		for (int i = 0; i < 8; i++) {
			const auto& pt = pts[i];
			cout << "  " << i << "(" << pt[0] << ", " << pt[1] << ", " << pt[2] << ")\n";
		}
#endif
		size_t cellIdx = pBlock->addHexCell(pts, 2, pBlock->getBlockIdx(), intersectingOnly);
		if (cellIdx != -1)
			result.push_back(cellIdx);
	}
#endif
	return result;
}
