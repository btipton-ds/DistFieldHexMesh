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

Polyhedron::Polyhedron(const set<Index3DId>& faceIds)
	: _faceIds(faceIds)
{
}

Polyhedron::Polyhedron(const vector<Index3DId>& faceIds)
{
	_faceIds.insert(faceIds.begin(), faceIds.end());
}

void Polyhedron::setId(ObjectPoolOwner* pBlock, size_t id)
{
	_pBlock = dynamic_cast<Block*> (pBlock);
	_thisId = Index3DId(pBlock->getBlockIdx(), id);
	assert(_thisId.isValid());
}


bool Polyhedron::unload(std::ostream& out)
{
	return true;
}

bool Polyhedron::load(std::istream& out)
{
	return true;
}

bool Polyhedron::verifyTopology() const
{
	bool valid = true;
	for (const auto& faceId : _faceIds) {
		if (_pBlock->polygonExists(faceId)) {
			_pBlock->faceFunc(faceId, [this, &valid](const Block* pBlock, const Polygon& face) {
				bool pass = face.ownedByCell(_thisId);
				if (!pass)
					valid = false;
				pass = face.verifyTopology();
				if (!pass)
					valid = false;
			});
		} else
			valid = false;
	}

	set<Index3DId> faceSet;
	faceSet.insert(_faceIds.begin(), _faceIds.end());
	auto edges = getEdges();
	for (const auto& edge : edges) {
		auto edgeFaces = edge.getFaceIds(faceSet);
		bool pass = edgeFaces.size() == 2;
		if (!pass)
			valid = false;
	}
	return valid;
}

bool Polyhedron::operator < (const Polyhedron& rhs) const
{
	assert(!"Polyhdra aren't sorted. Should never call this.");
	return false;
}

void Polyhedron::addFace(const Index3DId& faceId)
{
	if (_faceIds.count(faceId) == 0) {
		_faceIds.insert(faceId);
		_pBlock->faceFunc(faceId, [this](Block* pBlock, Polygon& face) {
			face.addCell(_thisId);
		});
	}
}

bool Polyhedron::removeFace(const Index3DId& faceId)
{
	if (_faceIds.count(faceId) != 0) {
		_faceIds.erase(faceId);
		_pBlock->faceFunc(faceId, [this](Block* _pBlock, Polygon& face) {
			face.removeCell(_thisId);
		});
		return true;
	}

	return false;
}

vector<Index3DId> Polyhedron::getCornerIds() const
{
	set<Index3DId> idSet;
	for (const auto& faceId : _faceIds) {
		_pBlock->faceFunc(faceId, [&idSet](const Block* pBlock, const Polygon& face) {
			const auto vertexIds = face.getVertexIds();
			idSet.insert(vertexIds.begin(), vertexIds.end());
		});
	}

	vector<Index3DId> result;
	result.insert(result.end(), idSet.begin(), idSet.end());

	return result;
}

vector<Edge> Polyhedron::getEdges() const
{
	set<Edge> idSet;
	for (const auto& faceId : _faceIds) {
		_pBlock->faceFunc(faceId, [&idSet](const Block* pBlock, const Polygon& face) {
			const auto& edges = face.getEdges();
			idSet.insert(edges.begin(), edges.end());
		});
	}

	vector<Edge> result;
	result.insert(result.end(), idSet.begin(), idSet.end());

	return result;
}

// Gets the edges for a vertex which belong to this polyhedron
set<Edge> Polyhedron::getVertEdges(const Index3DId& vertId) const
{
	auto cellEdges = getEdges();
	set<Edge> cellEdgeSet;
	cellEdgeSet.insert(cellEdges.begin(), cellEdges.end());
	auto allVertEdges = _pBlock->getVertexEdges(vertId);

	set<Edge> result;
	for (const auto& edge : allVertEdges) {
		if (cellEdgeSet.count(edge))
			result.insert(edge);
	}

	return result;
}

// Gets the faces for a vertex which belong to this polyhedron
set<Index3DId> Polyhedron::getVertFaces(const Index3DId& vertId) const
{
	auto cellFaces = getFaceIds();
	set<Index3DId> cellFaceSet;
	cellFaceSet.insert(cellFaces.begin(), cellFaces.end());
	set<Index3DId> allVertFaces;
	_pBlock->vertexFunc(vertId, [&allVertFaces](const Block* pBlock, const Vertex& vert) {
		allVertFaces = vert.getFaceIds();
	});

	set<Index3DId> result;
	for (const auto& faceId : allVertFaces) {
		if (cellFaceSet.count(faceId))
			result.insert(faceId);
	}

	return result;
}

CBoundingBox3Dd Polyhedron::getBoundingBox() const
{
	CBoundingBox3Dd bbox;
	auto cornerIds = getCornerIds();
	for (const auto& vertId : cornerIds) {
		bbox.merge(_pBlock->getVertexPoint(vertId));
	}

	return bbox;
}

Vector3d Polyhedron::calCentroid() const
{
	Vector3d result(0, 0, 0);
	vector<Index3DId> cornerIds = getCornerIds();
	for (const auto& vertId : cornerIds) {
		result += _pBlock->getVertexPoint(vertId);
	}
	result /= (double) cornerIds.size();

	return result;
}

bool Polyhedron::split(const Vector3d& splitPoint, bool intersectingOnly, vector<size_t>& newFaces)
{
	set<size_t> cellSet;
	cellSet.insert(_thisId.elementId());
	for (int i = 0; i < 1; i++) {
		set<size_t> resultSet;
		Vector3d normal(0, 0, 0);
		normal[i] = 1.0;
		Plane splitPlane(splitPoint, normal);
		for (size_t cellId : cellSet) {
		vector<size_t> splitCells;
		bool pass = true;
		_pBlock->cellFunc(cellId, [this, &splitPlane, intersectingOnly, &splitCells, &pass](Block* pBlock, Polyhedron& cell) {
			// TODO FIX ASAP. Adding a cell causes the data to move, causing data corruption during the operation.
			// Smart pointer will fix this, but it's heavier than needed.
			// Maybe reserve space?
			if (!cell.split(splitPlane, intersectingOnly, splitCells))
				pass = false;
		});
		if (!pass)
			return false;
		if (!splitCells.empty())
			resultSet.insert(splitCells.begin(), splitCells.end());
		}
		if (!resultSet.empty())
			cellSet.insert(resultSet.begin(), resultSet.end());
	}

	newFaces.insert(newFaces.end(), cellSet.begin(), cellSet.end());

	return true;
}

bool Polyhedron::split(const Plane& splitPlane, bool intersectingOnly, vector<size_t>& result)
{
	vector<Edge> edges = getEdges(), edgesToSplit;

	for (const auto& edge : edges) {
		double t = edge.intesectPlaneParam(splitPlane);
		if (t >= 0 && t <= 1) {
			edgesToSplit.push_back(edge);
		}
	}

	if (edgesToSplit.size() < 3)
		return true;

	set<Index3DId> vertIdSet;
	for (const auto& edge : edgesToSplit) {
		auto newVertId = edge.splitWithPlane(splitPlane);
		if (newVertId.isValid()) {
			vertIdSet.insert(newVertId);
		}
	}

	vector<Index3DId> newVertIds;
	newVertIds.insert(newVertIds.end(), vertIdSet.begin(), vertIdSet.end());
	assert(newVertIds.size() >= 3);

	// Add the splitting face
	Index3DId splittingFace;
	set<Index3DId> newFaceIdSet;
	if (newVertIds.size() >= 3) {
		orderVertIdsNTS(newVertIds);
		splittingFace = _pBlock->addFace(newVertIds);

		_pBlock->faceFunc(splittingFace, [](Block* pBlock, Polygon& face) {
			face.setNumSplits(1);
		});
	} else {
		assert(!"Less then 3 verts. Cannot form a face");
	}

	if (!_pBlock->verifyTopology(true))
		return false;

	// Split the cell faces with split edges
	for (size_t i = 0; i < newVertIds.size(); i++) {
		size_t j = (i + 1) % newVertIds.size();
		Edge edge(_pBlock, newVertIds[i], newVertIds[j]);

		const auto& faceId = edge.getCommonFace();
		Index3DId newFaceId;
		_pBlock->faceFunc(faceId, [this, &edge, &newFaceId](Block* pBlock, Polygon& face) {
			// These are the 1/2 faces of original faces
			newFaceId = face.splitBetweenVertices(edge.getVertexIds()[0], edge.getVertexIds()[1]);
			if (newFaceId.isValid())
				addFace(newFaceId);
		});
	}

	// Collect the faces below the splitting plane to form a new cell
	// Collect the faces to be removed in a separate list to avoid items moving around in the list being deleted.
	for (const auto& faceId : _faceIds) {
		_pBlock->faceFunc(faceId, [this, &splitPlane, &newFaceIdSet, &faceId](const Block* pBlock, const Polygon& face) {
			Vector3d ctr = face.getCentroid();
			Vector3d v = ctr - splitPlane._origin;
			if (v.dot(splitPlane._normal) < 0) {
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
	Index3DId newCellId(_thisId, _pBlock->addCell(newFaceIds));

	_pBlock->faceFunc(splittingFace, [this, &newCellId](Block* pBlock, Polygon& face) {
		face.addCell(_thisId);
		face.addCell(newCellId);
	});

	// Face owner cells are messed up by splitting. Need to fix that.
	if (!_pBlock->verifyTopology(true))
		return false;

	if (_thisId.isValid())
		result.push_back(_thisId.elementId());
	if (newCellId.isValid())
		result.push_back(newCellId.elementId());

	return true;
}

void Polyhedron::orderVertIdsNTS(vector<Index3DId>& vertIds) const
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
		_pBlock->vertexFunc(vertId, [this, &faceIds](Block* pBlock, Vertex& vert) {
			faceIds = vert.getFaceIds(_faceIds);
		});

		
		for (const auto& faceId : faceIds) {
			Index3DId nextVertId;
			_pBlock->faceFunc(faceId, [&vertId, &vertSet, &nextVertId](Block* pBlock, Polygon& face) {
				const auto& faceVerts = face.getVertexIdsNTS();
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
