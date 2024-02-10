#include <vector>
#include <map>

#define _USE_MATH_DEFINES
#include <cmath>

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
		getOutBlockPtr(_thisId)->faceFunc(faceId, [this](Polygon& face) {
			face.addCell(_thisId);
		});
	}
}

bool Polyhedron::removeFace(const Index3DId& faceId)
{
	if (_faceIds.count(faceId) != 0) {
		_faceIds.erase(faceId);
		getOutBlockPtr(_thisId)->faceFunc(faceId, [this](Polygon& face) {
			face.removeCell(_thisId);
		});
		return true;
	}

	return false;
}

void Polyhedron::getVertIds(set<Index3DId>& vertIds) const
{
	for (const auto& faceId : _faceIds) {
		getBlockPtr()->faceFunc(faceId, [&vertIds](const Polygon& face) {
			const auto vertexIds = face.getVertexIds();
			vertIds.insert(vertexIds.begin(), vertexIds.end());
		});
	}
}

void Polyhedron::getEdges(set<Edge>& edges, bool includeNeighborFaces) const
{
	map<Edge, set<Index3DId>> edgeToFaceMap;
	set<Index3DId> adjCellIds;
	for (const auto& faceId : _faceIds) {
		getBlockPtr()->faceFunc(faceId, [this, &edgeToFaceMap, &faceId, &adjCellIds](const Polygon& face) {
			auto temp = face.getCellIds();
			adjCellIds.insert(temp.begin(), temp.end());
			set<Edge> edges;
			face.getEdges(edges);
			for (const auto& edge : edges) {
				auto iter = edgeToFaceMap.find(edge);
				if (iter == edgeToFaceMap.end()) {
					set<Index3DId> data;
					iter = edgeToFaceMap.insert(make_pair(edge, data)).first;
				}
				iter->second.insert(faceId);
			}
		});
	}

	if (includeNeighborFaces) {
		adjCellIds.erase(_thisId);
		for (const auto& adjCellId : adjCellIds) {
			set<Index3DId> faceIds;
			getBlockPtr()->cellFunc(adjCellId, [&faceIds](const Polyhedron& cell) {
				faceIds = cell.getFaceIds();
			});
			for (const auto& faceId : faceIds) {
				getBlockPtr()->faceFunc(faceId, [this, &edgeToFaceMap, &faceId](const Polygon& face) {
					set<Edge> edges;
					face.getEdges(edges);
					for (const auto& edge : edges) {
						auto iter = edgeToFaceMap.find(edge);
						if (iter != edgeToFaceMap.end()) {
							iter->second.insert(faceId);
						}
					}
				});
			}
		}

	}

	for (const auto& pair : edgeToFaceMap) {
		edges.insert(Edge(pair.first, pair.second));
	}
}

void Polyhedron::getPrincipalEdges(set<Edge>& result) const
{
	set<Edge> allEdges;
	getEdges(allEdges, true);

	set<Index3DId> principalPolygonIds;
	getPrincipalPolygons(principalPolygonIds);
	for (const auto& faceId : principalPolygonIds) {
		set<Edge> faceEdges;
		getOutBlockPtr(_thisId)->faceFunc(faceId, [&faceEdges, &allEdges, &result](const Polygon& face) { face.getEdges(faceEdges); });
		// This assures that REAL edges have ALL their faces attached so they will be split when
		// a vertex is inserted
		for (const auto& faceEdge : faceEdges) {
			auto iter = allEdges.find(faceEdge);
			if (iter != allEdges.end())
				result.insert(*iter);
			else
				result.insert(faceEdge);
		}
	}
}

void Polyhedron::getPrincipalPolygons(set<Index3DId>& result) const
{
}

set<Index3DId> Polyhedron::getAdjacentCells() const
{
	set<Index3DId> cornerIds;
	getVertIds(cornerIds);
	set<Index3DId> faceIds, cellIds;

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
	set<Edge> cellEdgeSet;
	getEdges(cellEdgeSet, true);
	set<Index3DId> adjCells = getAdjacentCells();
	for (const auto& adjCellId : adjCells) {
		getBlockPtr()->cellFunc(adjCellId, [&cellEdgeSet](const Polyhedron& adjCell) {
			adjCell.getEdges(cellEdgeSet, true);
		});
	}

	set<Edge> result;
	for (const auto& edge : cellEdgeSet) {
		if (edge.containsVertex(vertId))
			result.insert(edge);
	}

	return result;
}

// Gets the faces for a vertex which belong to this polyhedron
set<Index3DId> Polyhedron::getVertFaces(const Index3DId& vertId) const
{
	set<Index3DId> result;

	auto vertEdges = getVertEdges(vertId);

	for (const Edge& edge : vertEdges) {
		edge.getFaceIds(result);
	}

	return result;
}

CBoundingBox3Dd Polyhedron::getBoundingBox() const
{
	CBoundingBox3Dd bbox;
	set<Index3DId> cornerIds;
	getVertIds(cornerIds);
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
			const double tol = Tolerance::sameDistTol();
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
	auto bbox = getBoundingBox();
	Vector3d testCtr = (bbox.getMin() + bbox.getMax()) * 0.5;
	double area = 0;
	Vector3d ctr(0, 0, 0);

	for (const auto& faceId : _faceIds) {
		getBlockPtr()->faceFunc(faceId, [&area, &ctr](const Polygon& face) {
			double faceArea;
			Vector3d faceCtr;
			face.calAreaAndCentroid(faceArea, faceCtr);
			area += faceArea;
			ctr += faceArea * faceCtr;
		});
	}
	ctr /= area;

	Vector3d err = ctr - testCtr;
	return ctr;
}

bool Polyhedron::splitWithPlanesAtPoint(const Vector3d& splitPoint, bool intersectingOnly, vector<Index3DId>& newCellIds) const
{
	set<Index3DId> cellSet;

	set<Index3DId> resultSet;
	vector<Index3DId> splitCells = splitWithPlane(Plane(splitPoint, Vector3d(1, 0, 0)), intersectingOnly);

	if (!splitCells.empty())
		resultSet.insert(splitCells.begin(), splitCells.end());

	if (!resultSet.empty())
		cellSet.insert(resultSet.begin(), resultSet.end());

	resultSet.clear();
	for (auto cellId : cellSet) {
		getOutBlockPtr(_thisId)->cellFunc(cellId, [&splitPoint, intersectingOnly, &resultSet](Polyhedron& cell) {
			auto splitCells = cell.splitWithPlane(Plane(splitPoint, Vector3d(0, 1, 0)), intersectingOnly);

			if (!splitCells.empty())
				resultSet.insert(splitCells.begin(), splitCells.end());
		});
	}

	if (!resultSet.empty())
		cellSet.insert(resultSet.begin(), resultSet.end());

	resultSet.clear();
	for (auto cellId : cellSet) {
		getOutBlockPtr(_thisId)->cellFunc(cellId, [&splitPoint, intersectingOnly, &resultSet](Polyhedron& cell) {
			auto splitCells = cell.splitWithPlane(Plane(splitPoint, Vector3d(0, 0, 1)), intersectingOnly);

			if (!splitCells.empty())
				resultSet.insert(splitCells.begin(), splitCells.end());
		});
	}

	newCellIds.insert(newCellIds.end(), resultSet.begin(), resultSet.end());

	return true;
}

void Polyhedron::splitByCurvature(const TriMesh::CMeshPtr& pTriMesh, size_t circleDivs) const
{
	const double sinEdgeAngle = sin(30.0 / 180.0 * M_PI);
	const double minCurvature = 0.1; // 1 meter radius
	const double kDiv = 4.0;

	CBoundingBox3Dd bbox = getBoundingBox();
	vector<CMesh::SearchEntry> triEntries;
	if (pTriMesh->findTris(bbox, triEntries)) {
		double avgSurfCurvature = 0;
		size_t numSamples = 0;
		for (const auto& triEntry : triEntries) {
			size_t triIndex = triEntry.getIndex();
			Vector3i tri = pTriMesh->getTri(triIndex);
			for (int i = 0; i < 3; i++) {
				int j = (i + 1) % 3;
				size_t edgeIdx = pTriMesh->findEdge(tri[i], tri[j]);
				double edgeCurv = pTriMesh->edgeCurvature(edgeIdx);
				if (edgeCurv > 0) {
					avgSurfCurvature += edgeCurv;
					numSamples++;
				}
			}
		}

		avgSurfCurvature /= numSamples;
		if (avgSurfCurvature < minCurvature)
			return;
		double avgSurfRadius = 1 / avgSurfCurvature;
		cout << "Surf radius: " << avgSurfRadius << "\n";
		double avgSurfCircumference = 2 * M_PI * avgSurfRadius;
		double avgSurfArcLength = avgSurfCircumference / circleDivs; // 5 deg arc
		auto range = bbox.range();
		double minBoxDim = min(range[0], min(range[1], range[2]));
		if (kDiv * avgSurfArcLength > minBoxDim) {
			vector<Index3DId> splitCells;
			if (!splitWithPlanesAtCentroid(false, splitCells)) {
				return;
			}
		}
	}
}

vector<Index3DId> Polyhedron::splitWithPlane(const Plane& splitPlane, bool intersectingOnly) const
{
	const double tol = Tolerance::paramTol();
	vector<Index3DId> result;

	assert(isClosed());

	/*
	* collect all the splitting vertices based on the principal edges.
	* In some cases they will be existing vertices from a prior split 
	*/
	set<Edge> principalEdges;
	getPrincipalEdges(principalEdges);
	assert(principalEdges.size() == 12);
	vector<Edge> edgesToSplit;
	set<Index3DId> vertIdSet;

	for (const auto& edge : principalEdges) {
		assert(edge.onPrincipalAxis(getBlockPtr()));
		double t = edge.intesectPlaneParam(getBlockPtr(), splitPlane);
		if (t >= -tol && t <= 1 + tol) {
			Vector3d pt = edge.calPointAt(getBlockPtr(), t);
			Index3DId vertId = getBlockPtr()->idOfPoint(pt);
			if (vertId.isValid())
				vertIdSet.insert(vertId);
			else {
				// In this case, the principal edge has not been split by this plane
				edgesToSplit.push_back(edge);
			}
		}
	}

	if (edgesToSplit.size() + vertIdSet.size() < 3 || edgesToSplit.empty())
		return result;

	for (auto& edge : edgesToSplit) {
		assert(edge.onPrincipalAxis(getBlockPtr()));
		set<Index3DId> faceIds;
		auto newVertId = edge.splitWithPlane(getOutBlockPtr(_thisId), splitPlane, faceIds);
		if (newVertId.isValid()) {
			vertIdSet.insert(newVertId);
			for (const auto& faceId : faceIds) {
				getOutBlockPtr(_thisId)->faceFunc(faceId, [&edge, &newVertId](Polygon& splitFace) {
					if (!splitFace.containsVert(newVertId))
						splitFace.insertVertexInEdge(edge, newVertId);
				});
			}
		}
	}

	vector<Index3DId> newVertIds;
	newVertIds.insert(newVertIds.end(), vertIdSet.begin(), vertIdSet.end());
	assert(newVertIds.size() >= 3);

	// Add the splitting face
	Index3DId splittingFaceId;
	set<Index3DId> splitFaceIdSet;
	if (newVertIds.size() >= 3) {
		if (!orderVertIds(newVertIds))
			return result;
		splittingFaceId = getOutBlockPtr(_thisId)->addFace(newVertIds);
	} else {
		assert(!"Less then 3 verts. Cannot form a face");
	}

	for (auto& faceId : _faceIds) {
		Index3DId newFaceId;
		Index3DId dbgCheck(Index3D(4, 0, 1), 2);
		if (faceId == dbgCheck) {
			int dbgBreak = 1;
		}

		getOutBlockPtr(_thisId)->faceFunc2(faceId, splittingFaceId, [this, &newFaceId](Polygon& face, Polygon& splittingFace) {
			newFaceId = face.splitWithFaceEdges(splittingFace);
		});

		if (newFaceId.isValid()) {
			splitFaceIdSet.insert(newFaceId);
		}
	}

	// Collect the faces below the splitting plane to form a new cell
	// Collect the faces to be removed in a separate list to avoid items moving around in the list being deleted.
	set<Index3DId> faceSet0, faceSet1;

	splitFaceIdSet.insert(_faceIds.begin(), _faceIds.end());
	for (const auto& faceId : splitFaceIdSet) {
		getBlockPtr()->faceFunc(faceId, [&faceId, &splitPlane, &faceSet0, &faceSet1](const Polygon& face) {
			const double tol = Tolerance::paramTol();
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


	faceSet0.insert(splittingFaceId);
	faceSet1.insert(splittingFaceId);

	Index3DId newCellId0 = Index3DId(_thisId, getOutBlockPtr(_thisId)->addCell(faceSet0));
	Index3DId newCellId1 = Index3DId(_thisId, getOutBlockPtr(_thisId)->addCell(faceSet1));

	getOutBlockPtr(_thisId)->faceFunc(splittingFaceId, [this, &newCellId0, &newCellId1](Polygon& face) {
		face.addCell(newCellId0);
		face.addCell(newCellId1);
	});

	getOutBlockPtr(_thisId)->faceFunc(splittingFaceId, [this, &newCellId0, &newCellId1](Polygon& face) {
		face.addCell(newCellId0);
		face.addCell(newCellId1);
	});

	if (newCellId0.isValid())
		result.push_back(newCellId0);
	if (newCellId1.isValid())
		result.push_back(newCellId1);

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
				Edge newEdge(vertId0, vertId1);

				edgeSet.insert(newEdge);
			}
		});
	}

	return edgeSet;
}

bool Polyhedron::orderVertIds(vector<Index3DId>& vertIds) const
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

bool Polyhedron::isClosed() const
{
	bool result = true;
	set<Edge> edges;
	getEdges(edges, false);
	for (const auto& edge : edges) {
		if (edge.getFaceIds().size() != 2) {
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

