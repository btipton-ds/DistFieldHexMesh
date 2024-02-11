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
		faceFunc(faceId, [](const Polygon& face) {
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

bool Polyhedron::operator < (const Polyhedron& rhs) const
{
	assert(!"Polyhdra aren't sorted. Should never call this.");
	return false;
}

void Polyhedron::addFace(const Index3DId& faceId)
{
	if (_faceIds.count(faceId) == 0) {
		_faceIds.insert(faceId);
	}
}

bool Polyhedron::removeFace(const Index3DId& faceId)
{
	if (_faceIds.count(faceId) != 0) {
		_faceIds.erase(faceId);
		return true;
	}

	return false;
}

void Polyhedron::getVertIds(set<Index3DId>& vertIds) const
{
	for (const auto& faceId : _faceIds) {
		faceFunc(faceId, [&vertIds](const Polygon& face) {
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
		faceFunc(faceId, [this, &edgeToFaceMap, &faceId, &adjCellIds](const Polygon& face) {
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
				faceFunc(faceId, [this, &edgeToFaceMap, &faceId](const Polygon& face) {
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

set<Index3DId> Polyhedron::getAdjacentCells() const
{
	set<Index3DId> cornerIds;
	getVertIds(cornerIds);
	set<Index3DId> faceIds, cellIds;

	for (const auto& faceId : faceIds) {
		faceFunc(faceId, [this, &cellIds](const Polygon& face) {
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
void Polyhedron::getVertEdges(const Index3DId& vertId, set<Edge>& result, bool includeAdjacentCells) const
{
	set<Edge> cellEdgeSet;
	getEdges(cellEdgeSet, includeAdjacentCells);
	if (includeAdjacentCells) {
		set<Index3DId> adjCells = getAdjacentCells();
		for (const auto& adjCellId : adjCells) {
			getBlockPtr()->cellFunc(adjCellId, [&cellEdgeSet](const Polyhedron& adjCell) {
				adjCell.getEdges(cellEdgeSet, true);
				});
		}
	}

	for (const auto& edge : cellEdgeSet) {
		if (edge.containsVertex(vertId))
			result.insert(edge);
	}
}

// Gets the faces for a vertex which belong to this polyhedron
set<Index3DId> Polyhedron::getVertFaces(const Index3DId& vertId) const
{
	set<Index3DId> result;

	set<Edge> vertEdges;
	getVertEdges(vertId, vertEdges, false);

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
		faceFunc(faceId, [this, &pt, &ctr, &result](const Polygon& face) {
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
		faceFunc(faceId, [&area, &ctr](const Polygon& face) {
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

bool Polyhedron::splitAtPoint(const Vector3d& pt, set<Index3DId>& newCellIds) const
{
	set<Index3DId> vertIds;
	getVertIds(vertIds);

	while (!vertIds.empty()) {
		const auto vertId = *vertIds.begin();
		vertIds.erase(vertId);

		set<Edge> vertEdges;
		getVertEdges(vertId, vertEdges, false);
		if (vertEdges.size() != 3) {
			assert(!"This case is not supported yet");
			continue;
		}
		vector<Edge> orderedEdges;
		if (orderVertEdges(vertEdges, orderedEdges)) {
			Index3DId cellCenterPtId = getOutBlockPtr()->addVertex(pt);

			vector<Index3DId> edgePtIds, facePtIds, sourceFaceIds;
			for (size_t i = 0; i < orderedEdges.size(); i++) {
				size_t j = (i + 1) % orderedEdges.size();
				const auto& edge = orderedEdges[i];
				const auto& edge1 = orderedEdges[j];
				bool found = false;
				for (const auto& faceId0 : edge.getFaceIds()) {
					if (edge1.getFaceIds().contains(faceId0)) {
						faceFunc(faceId0, [this, &pt, &edge, &edgePtIds, &facePtIds, &sourceFaceIds](const Polygon& face) {
							Vector3d faceCtr = face.projectPoint(pt);
							facePtIds.push_back(getOutBlockPtr()->addVertex(faceCtr));

							Vector3d edgeCtr = edge.projectPt(getBlockPtr(), pt);
							edgePtIds.push_back(getOutBlockPtr()->addVertex(edgeCtr));
							sourceFaceIds.push_back(face.getId());
						});
						found = true;
						break;
					}
				}
				if (!found) {
					assert(!"Could not find mid face");
					return false;
				}
			}

			vector<Index3DId> corners = {
				vertId, edgePtIds[0], facePtIds[0], edgePtIds[1],
				edgePtIds[2], facePtIds[2], cellCenterPtId, facePtIds[1]
			};

			vector<Index3DId> faceIds;
			faceIds.push_back(getOutBlockPtr()->addFace(vector({ corners[0], corners[1], corners[2], corners[3] })));
			faceIds.push_back(getOutBlockPtr()->addFace(vector({ corners[4], corners[5], corners[6], corners[7] })));

			faceIds.push_back(getOutBlockPtr()->addFace(vector({ corners[0], corners[1], corners[5], corners[4] })));
			faceIds.push_back(getOutBlockPtr()->addFace(vector({ corners[3], corners[2], corners[6], corners[7] })));

			faceIds.push_back(getOutBlockPtr()->addFace(vector({ corners[1], corners[2], corners[6], corners[5] })));
			faceIds.push_back(getOutBlockPtr()->addFace(vector({ corners[0], corners[3], corners[7], corners[4] })));

			Index3DId newCellId = getOutBlockPtr()->addCell(faceIds);
			for (const auto& faceId : faceIds) {
				faceOutFunc(faceId, [&newCellId](Polygon& face) { face.addCell(newCellId); });
			}

			faceOutFunc(sourceFaceIds[0], [](Polygon& face) { face.addChildId(face.getId()); });
			faceOutFunc(sourceFaceIds[5], [](Polygon& face) { face.addChildId(face.getId()); });
			faceOutFunc(sourceFaceIds[2], [](Polygon& face) { face.addChildId(face.getId()); });
		}
	}

	return true;
}

bool Polyhedron::orderVertEdges(set<Edge>& edgesIn, vector<Edge>& orderedEdges) const
{
	orderedEdges.clear();
	set<Edge> edges(edgesIn);
	while (!edges.empty()) {
		Edge lastEdge;
		if (orderedEdges.empty()) {
			lastEdge = *edges.begin();
			orderedEdges.push_back(lastEdge);
			edges.erase(lastEdge);
		} else {
			lastEdge = orderedEdges.back();
		}

		const auto& edgeFaces = lastEdge.getFaceIds();
		assert(edgeFaces.size() == 2);

		bool found = false;
		for (const auto& otherEdge : edges) {
			const auto& otherEdgeFaces = otherEdge.getFaceIds();

			for (const auto& faceId : otherEdgeFaces) {
				if (edgeFaces.contains(faceId)) {
					found = true;
					orderedEdges.push_back(otherEdge);
					edges.erase(otherEdge);
					break;
				}
			}
			if (found)
				break;
		}
		if (!found)
			return false;
	}

	return true;
}

void Polyhedron::splitByCurvature(double maxArcAngleDegrees) const
{
	const double sinEdgeAngle = sin(30.0 / 180.0 * M_PI);
	const double minCurvature = 0.1; // 1 meter radius
	const double kDiv = 4.0;
	const size_t circleDivs = (size_t)(360.0 / maxArcAngleDegrees);

	auto pTriMesh =getBlockPtr()->getModelMesh();
	Vector3d centroid = calCentroid();

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
			set<Index3DId> splitCells;
			if (!splitAtPoint(centroid, splitCells)) {
				return;
			}
		}
	}
}

set<Edge> Polyhedron::createEdgesFromVerts(vector<Index3DId>& vertIds) const
{

	// This function takes vertices and looks for faces which contain exactly 2 of the available verts.
	// The edge may already exist.
	set<Edge> edgeSet;
	for (const auto& faceId : _faceIds) {
		faceFunc(faceId, [this, &edgeSet, &vertIds](const Polygon& face) {
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
			faceFunc(faceId, [this, &valid](const Polygon& face) {
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

