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

Polyhedron::Polyhedron(const Polyhedron& src)
	: ObjectPoolOwnerUser(src)
	, _faceIds(src._faceIds)
{
}

Polyhedron& Polyhedron::operator = (const Polyhedron& rhs)
{
	ObjectPoolOwnerUser::operator=(rhs);
	_faceIds = rhs._faceIds;

	return *this;
}

void Polyhedron::dumpFaces() const
{
	size_t idx = 0;
	for (const auto& faceId : _faceIds) {
		cout << "face[" << idx++ << "]\n";
		getBlockPtr()->faceFunc(faceId, [](const Polygon& face) {
			Vector3d n = face.calUnitNormal();
			auto pBlock = face.getBlockPtr();
			cout << "  n: (" << n[0] << ", " << n[1] << ", " << n[2] << ")\n";
			const auto& vertIds = face.getVertexIds();
			for (const auto& vertId : vertIds) {
				Vector3d pt = pBlock->getVertexPoint(vertId);
				cout << "  p: (" << pt[0] << ", " << pt[1] << ", " << pt[2] << ")\n";
			}
		});
	}
}


bool Polyhedron::unload(ostream& out)
{
	return true;
}

bool Polyhedron::load(istream& out)
{
	return true;
}

bool Polyhedron::verifyTopologyAdj() const
{
	bool result = verifyTopology();
	set<Index3DId> adjacentCells = getAdjacentCells();
	if (!adjacentCells.empty()) {
		for (const auto& cellId : adjacentCells) {
			if (!getBlockPtr()->verifyPolyhedronTopology(cellId)) {
				result = false;
			}
		}
	}
	return result;
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
		getBlockPtr()->faceFunc(faceId, [this](Polygon& face) {
			face.addCell(_thisId);
		});
	}
}

bool Polyhedron::removeFace(const Index3DId& faceId)
{
	if (_faceIds.count(faceId) != 0) {
		_faceIds.erase(faceId);
		getBlockPtr()->faceFunc(faceId, [this](Polygon& face) {
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
		getBlockPtr()->faceFunc(faceId, [&idSet](const Polygon& face) {
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
		getBlockPtr()->faceFunc(faceId, [&idSet](const Polygon& face) {
			const auto& edges = face.getEdges();
			idSet.insert(edges.begin(), edges.end());
		});
	}

	vector<Edge> result;
	result.insert(result.end(), idSet.begin(), idSet.end());

	return result;
}

set<Index3DId> Polyhedron::getAdjacentCells() const
{
	auto cornerIds = getCornerIds();
	set<Index3DId> faceIds, cellIds;

	for (const auto& vertId : cornerIds) {
		getBlockPtr()->vertexFunc(vertId, [&faceIds](const Vertex& vert) {
			const auto& vertFaceIds = vert.getFaceIds();
			if (!vertFaceIds.empty())
				faceIds.insert(vertFaceIds.begin(), vertFaceIds.end());
		});
	}

	for (const auto& faceId : faceIds) {
		getBlockPtr()->faceFunc(faceId, [this, &cellIds](const Polygon& face) {
			const auto& temp = face.getCellIds();
			if (!temp.empty()) {
				for (auto cellId : temp) {
					if (cellId != _thisId)
						cellIds.insert(cellId);
				}
			}
		});
	}

	return cellIds;
}

// Gets the edges for a vertex which belong to this polyhedron
set<Edge> Polyhedron::getVertEdges(const Index3DId& vertId) const
{
	auto cellEdges = getEdges();
	set<Edge> cellEdgeSet;
	cellEdgeSet.insert(cellEdges.begin(), cellEdges.end());
	auto allVertEdges = getBlockPtr()->getVertexEdges(vertId);

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
	getBlockPtr()->vertexFunc(vertId, [&allVertFaces](const Vertex& vert) {
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
		bbox.merge(getBlockPtr()->getVertexPoint(vertId));
	}

	return bbox;
}

bool Polyhedron::contains(const Vector3d& pt) const
{
	bool result = true;
	Vector3d ctr = calCentroid();
	for (const auto& faceId : _faceIds) {
		getBlockPtr()->faceFunc(faceId, [this, &pt, &ctr, &result](const Polygon& face) {
			const double tol = 1.0e-5;
			Vector3d faceCtr = face.calCentroid();
			Vector3d norm = face.calUnitNormal();

			Vector3d vCtr = ctr - faceCtr;
			vCtr.normalize();
			if (vCtr.dot(norm) > 0)
				norm = -norm;

			Vector3d vTestPt = pt - faceCtr;
			vTestPt.normalize();

			if (norm.dot(vTestPt) > tol)
				result = false;
		});
	}

	return result;
}

Vector3d Polyhedron::calCentroid() const
{
	Vector3d result(0, 0, 0);
	vector<Index3DId> cornerIds = getCornerIds();
	for (const auto& vertId : cornerIds) {
		result += getBlockPtr()->getVertexPoint(vertId);
	}
	result /= (double) cornerIds.size();

	return result;
}

bool Polyhedron::split(const Vector3d& splitPoint, bool intersectingOnly, vector<Index3DId>& newCellIds)
{
	set<Index3DId> cellSet;
	cellSet.insert(_thisId);
	for (int i = 0; i < 1; i++) {
		set<Index3DId> resultSet;
		Vector3d normal(0, 0, 0);
		normal[i] = 1.0;
		Plane splitPlane(splitPoint, normal);
		for (auto cellId : cellSet) {
		vector<Index3DId> splitCells = splitWithPlane(splitPlane, intersectingOnly);

		if (!splitCells.empty())
			resultSet.insert(splitCells.begin(), splitCells.end());
		}
		if (!resultSet.empty())
			cellSet.insert(resultSet.begin(), resultSet.end());
	}

	newCellIds.insert(newCellIds.end(), cellSet.begin(), cellSet.end());

	return true;
}

vector<Index3DId> Polyhedron::splitWithPlane(const Plane& splitPlane, bool intersectingOnly)
{
	const double tol = 1.0e-6;
	vector<Index3DId> result;

	assert(isClosed());

	vector<Edge> edges = getEdges(), edgesToSplit;
	set<Index3DId> vertIdSet;

	for (const auto& edge : edges) {
		double t = edge.intesectPlaneParam(splitPlane);
		if (t >= -tol && t <= 1 + tol) {
			Vector3d pt = edge.calPointAt(t);
			Index3DId vertId = getBlockPtr()->idOfPoint(pt);
			if (vertId.isValid())
				vertIdSet.insert(vertId);
			else
				edgesToSplit.push_back(edge);
		}
	}

	if (edgesToSplit.size() + vertIdSet.size() < 3 || edgesToSplit.empty())
		return result;

	for (auto& edge : edgesToSplit) {
		auto newVertId = edge.splitWithPlane(splitPlane);
		if (newVertId.isValid()) {
			vertIdSet.insert(newVertId);
		}
	}

	vector<Index3DId> newVertIds;
	newVertIds.insert(newVertIds.end(), vertIdSet.begin(), vertIdSet.end());
	assert(newVertIds.size() >= 3);

	// Add the splitting face
	Index3DId splittingFaceId;
	set<Index3DId> splitFaceIdSet;
	if (newVertIds.size() >= 3) {
		if (!orderVertIdsNTS(newVertIds))
			return result;
		splittingFaceId = getBlockPtr()->addFace(newVertIds);
	} else {
		assert(!"Less then 3 verts. Cannot form a face");
	}

	getBlockPtr()->faceFunc(splittingFaceId, [this, &splitFaceIdSet, &splittingFaceId](Polygon& spittingFace) {
		for (auto& faceId : _faceIds) {
			vector<Index3DId> splittingFaces;
			getBlockPtr()->faceFunc(faceId, [this, &splitFaceIdSet, &spittingFace, &splittingFaces](Polygon& face) {
				splittingFaces = face.splitWithFaceEdgesNTS(spittingFace);
			});

			if (splittingFaces.size() == 2) {
				splitFaceIdSet.insert(splittingFaces.begin(), splittingFaces.end());
			}
		}
	});

	// Collect the faces below the splitting plane to form a new cell
	// Collect the faces to be removed in a separate list to avoid items moving around in the list being deleted.
	set<Index3DId> faceSet0, faceSet1;

	splitFaceIdSet.insert(_faceIds.begin(), _faceIds.end());
	for (const auto& faceId : splitFaceIdSet) {
		getBlockPtr()->faceFunc(faceId, [&faceId, &splitPlane, &faceSet0, &faceSet1](const Polygon& face) {
			const double tol = 1.0e-5;
			if (face.isAbovePlane(splitPlane, tol))
				faceSet0.insert(faceId);
			else
				faceSet1.insert(faceId);
		});
	}

	if (faceSet0.size() < 4 || faceSet1.size() < 4) { // A tetrohedron has the minimum number of planar faces - 4.
		return result;
	}
	if (faceSet1.size() > faceSet0.size())
		swap(faceSet0, faceSet1);

	for (const auto& faceId : faceSet1) {
		removeFace(faceId);
	}
	addFace(splittingFaceId);

	vector<Index3DId> newFaceIds;
	newFaceIds.insert(newFaceIds.end(), faceSet1.begin(), faceSet1.end());
	newFaceIds.push_back(splittingFaceId);
	Index3DId newCellId = Index3DId(_thisId, getBlockPtr()->addCell(newFaceIds));

	getBlockPtr()->faceFunc(splittingFaceId, [this, &newCellId](Polygon& face) {
		face.addCell(_thisId);
		face.addCell(newCellId);
	});

	assert(verifyTopology());

	if (_thisId.isValid())
		result.push_back(_thisId);
	if (newCellId.isValid())
		result.push_back(newCellId);

	return result;
}

set<Edge> Polyhedron::createEdgesFromVerts(vector<Index3DId>& vertIds) const
{

	// This function takes vertices and looks for faces which contain exactly 2 of the available verts.
	// The edge may already exist.
	set<Edge> edgeSet;
	for (const auto& faceId : _faceIds) {
		getBlockPtr()->faceFunc(faceId, [this, &edgeSet, &vertIds](const Polygon& face) {
			set<Index3DId> vertsInFace;
			for (const auto& vertId : vertIds) {
				if (face.containsVert(vertId))
					vertsInFace.insert(vertId);
			}
			if (vertsInFace.size() == 2) {
				auto iter = vertsInFace.begin();
				const Index3DId& vertId0 = *iter++;
				const Index3DId& vertId1 = *iter;
				Edge newEdge(getBlockPtr(), vertId0, vertId1);


				edgeSet.insert(newEdge);
			}
		});
	}

	return edgeSet;
}

bool Polyhedron::orderVertIdsNTS(vector<Index3DId>& vertIds) const
{

	vector<Index3DId> result;
	auto newEdgeSet = createEdgesFromVerts(vertIds);

	const Edge& e = *newEdgeSet.begin();
	result.push_back(e.getVertexIds()[0]);
	result.push_back(e.getVertexIds()[1]);

	newEdgeSet.erase(newEdgeSet.begin());
	bool found = true;
	while (found && !newEdgeSet.empty()) {
		found = false;
		const auto& lastVert = result.back();
		for (auto iter = newEdgeSet.begin(); iter != newEdgeSet.end(); iter++) {
			const Edge& e = *iter;
			const auto& otherVert = e.getOtherVert(lastVert);
			if (otherVert.isValid()) {
				result.push_back(otherVert);
				newEdgeSet.erase(iter);
				found = true;
				break;
			}
		}
	}

	if (result.front() == result.back()) {
		result.pop_back();
		if (result.size() == vertIds.size()) {
			vertIds = result;
			return true;
		} else {
			assert(!"Ordered list does not match size of input list.");
		}
	}

	return false;
}

set<Index3DId> Polyhedron::getEdgeFaceIds(const Edge& edge) const
{
	// This function returns the faced ids which belong to this edge AND to this cell. 
	// It should always be two for closed cell.
	set<Index3DId> result;

	for (const auto& faceId : _faceIds) {
		getBlockPtr()->faceFunc(faceId, [&edge, &faceId, &result](const Polygon& face) {
			if (face.containsEdge(edge)) {
				result.insert(faceId);
			}
		});
	}

	return result;
}

bool Polyhedron::isClosed() const
{
	bool result = true;
	vector<Edge> edges = getEdges();
	for (const auto& edge : edges) {
		if (getEdgeFaceIds(edge).size() != 2) {
			result = false;
			break;
		}
	}

	return result;
}

bool Polyhedron::verifyTopology() const
{
	bool valid = true;
#ifdef _DEBUG 

	for (const auto& faceId : _faceIds) {
		if (getBlockPtr()->polygonExists(faceId)) {
			if (!isClosed())
				valid = false;
			getBlockPtr()->faceFunc(faceId, [this, &valid](const Polygon& face) {
				bool pass = face.ownedByCell(_thisId);
				if (!pass)
					valid = false;
				pass = face.verifyTopology();
				if (!pass)
					valid = false;
				});
		}
		else
			valid = false;
	}
#endif // _DEBUG 

	return valid;
}
