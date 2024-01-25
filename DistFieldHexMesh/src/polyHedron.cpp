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

void Polyhedron::addFace(const Index3DId& faceId)
{
	if (find(_faceIds.begin(), _faceIds.end(), faceId) == _faceIds.end())
		_faceIds.push_back(faceId);
}

bool Polyhedron::removeFace(const Index3DId& faceId)
{
	auto pos = find(_faceIds.begin(), _faceIds.end(), faceId);
	if (pos != _faceIds.end()) {
		_faceIds.erase(pos);
		return true;
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

vector<size_t> Polyhedron::split(Block* pBlock, const Vector3d& splitPoint, bool intersectingOnly)
{
	set<size_t> cellSet;
	cellSet.insert(_thisId.elementId());
	for (int i = 0; i < 1; i++) {
		set<size_t> resultSet;
		Vector3d normal(0, 0, 0);
		normal[i] = 1.0;
		for (size_t cellId : cellSet) {
			vector<size_t> splitCells;
			pBlock->cellFunc(cellId, [this, &splitPoint, &normal, intersectingOnly, &splitCells](Block* pBlock, Polyhedron& cell) {
				// TODO FIX ASAP. Adding a cell causes the data to move, causing data corruption during the operation.
				// Smart pointer will fix this, but it's heavier than needed.
				// Maybe reserve space?
				splitCells = cell.split(pBlock, splitPoint, normal, intersectingOnly);
			});
			if (!splitCells.empty())
				resultSet.insert(splitCells.begin(), splitCells.end());
		}
		if (!resultSet.empty())
			cellSet.insert(resultSet.begin(), resultSet.end());
	}

	vector<size_t> result;
	result.insert(result.end(), cellSet.begin(), cellSet.end());

	return result;
}

vector<size_t> Polyhedron::split(Block* pBlock, const Vector3d& splitPoint, const Vector3d& normal, bool intersectingOnly)
{
	vector<size_t> result;
	const Plane cutPlane(splitPoint, normal);

	auto edges = getEdges(pBlock);

	size_t numIntersections = 0;
	for (const auto& edge : edges) {
		double t = edge.intesectPlaneParam(pBlock, cutPlane);
		if (t > 0 && t < 1) {
			numIntersections++;
		}
	}

	if (numIntersections < 3)
		return result;

	vector<Index3DId> newVertIds;
	for (const auto& edge : edges) {
		auto newVertId = edge.splitWithPlane(pBlock, cutPlane);
		if (newVertId.isValid()) {
			newVertIds.push_back(newVertId);
		}
	}

	cout << "Num new verts: " << newVertIds.size() << "\n";

	// Add the splitting face
	Index3DId splittingFace;
	set<Index3DId> newFaceIdSet;
	if (newVertIds.size() >= 3) {
		orderVertIds(pBlock, newVertIds);
		splittingFace = pBlock->addFace(newVertIds);
	} else {
		assert(!"Less then 3 verts. Cannot form a face");
	}

	// Split the cell faces with split edges
	for (size_t i = 0; i < newVertIds.size(); i++) {
		size_t j = (i + 1) % newVertIds.size();
		Edge edge(newVertIds[i], newVertIds[j]);

		const auto& faceId = edge.getCommonFace(pBlock);
		Index3DId newFaceId;
		pBlock->faceFunc(faceId, [this, &edge, &newFaceId](Block* pBlock, Polygon& face) {
			// These are the 1/2 faces of original faces
			newFaceId = face.splitWithEdge(pBlock, edge);
			addFace(newFaceId);
		});

	}

	// Collect the faces below the splitting plane to form a new cell
	// Collect the faces to be removed in a separate list to avoid items moving around in the list being deleted.
	for (const auto& faceId : _faceIds) {
		pBlock->faceFunc(faceId, [this, &cutPlane, &newFaceIdSet, &faceId](const Block* pBlock, const Polygon& face) {
			Vector3d ctr = face.getCentroid(pBlock);
			Vector3d v = ctr - cutPlane._origin;
			if (v.dot(cutPlane._normal) < 0) {
				newFaceIdSet.insert(faceId);
			}
		});
	}
	for (const auto& faceId : newFaceIdSet) {
		removeFace(faceId);
	}
	addFace(splittingFace);

	vector<Index3DId> newFaceIds;
	newFaceIds.insert(newFaceIds.end(), newFaceIdSet.begin(), newFaceIdSet.end());
	newFaceIds.push_back(splittingFace);
	Index3DId newCellId(_thisId, pBlock->addCell(newFaceIds));

	pBlock->faceFunc(splittingFace, [this, &newCellId](Block* pBlock, Polygon& face) {
		face.setCreatorCellId(_thisId);
		face.setNeighborCellId(newCellId);
	});

	if (_thisId.isValid())
		result.push_back(_thisId.elementId());
	if (newCellId.isValid())
		result.push_back(newCellId.elementId());
	return result;
}

void Polyhedron::orderVertIds(Block* pBlock, vector<Index3DId>& vertIds) const
{
	set<Index3DId> vertSet;
	vertSet.insert(vertIds.begin(), vertIds.end());
	vertIds.clear();

	vertIds.push_back(*vertSet.begin());
	vertSet.erase(vertIds.back());
	while (!vertSet.empty()) {
		// Find next vertex by walking faces
		const auto& vertId = vertIds.back();

		// Get cell faces connected to this vertex
		set<Index3DId> faceIds;
		pBlock->vertexFunc(vertId, [this, &faceIds](Block* pBlock, Vertex& vert) {
			faceIds = vert.getFaceIds(_faceIds);
		});

		
		for (const auto& faceId : faceIds) {
			Index3DId nextVertId;
			pBlock->faceFunc(faceId, [&vertId, &vertSet, &nextVertId](Block* pBlock, Polygon& face) {
				const auto& faceVerts = face.getVertexIds();
				for (const auto& faceVertId : faceVerts) {
					if (vertSet.count(faceVertId) != 0) {
						nextVertId = faceVertId;
						return;
					}
				}
			});

			if (nextVertId.isValid()) {
				vertIds.push_back(nextVertId);
				vertSet.erase(nextVertId);
				break;
			}
		}
	}

}
