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

void Polyhedron::addChild(const Index3DId& id)
{
	_children.insert(id);
}

void Polyhedron::setParent(const Index3DId& id)
{
	_parent = id;
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

Index3DId Polyhedron::createFace(const std::vector<Index3DId>& vertIds)
{
#if _DEBUG
	set<Edge> edgeSet;
	Polygon::createEdgesStat(vertIds, edgeSet);
	for (const auto& edge : edgeSet) {
		assert(edge.getLength(getBlockPtr()) > Tolerance::sameDistTol());
		assert(edge.onPrincipalAxis(getBlockPtr()));
	}
#endif
	Index3DId result = getBlockPtr()->findFace(vertIds);
	if (!result.isValid())
		result = getBlockPtr()->addFace(vertIds);

	return result;
}

void Polyhedron::createHexahedralFaces(const std::vector<Index3DId>& corners, std::vector<Index3DId>& faceIds)
{
	faceIds.push_back(createFace({ corners[0], corners[1], corners[2], corners[3] }));
	faceIds.push_back(createFace({ corners[4], corners[5], corners[6], corners[7] }));

	faceIds.push_back(createFace({ corners[0], corners[1], corners[5], corners[4] }));
	faceIds.push_back(createFace({ corners[3], corners[2], corners[6], corners[7] }));

	faceIds.push_back(createFace({ corners[1], corners[2], corners[6], corners[5] }));
	faceIds.push_back(createFace({ corners[0], corners[3], corners[7], corners[4] }));
}

bool Polyhedron::splitAtPoint(const Vector3d& centerPoint, set<Index3DId>& newCellIds)
{
	if (!_children.empty())
		return false; // We've been split, don't split again. However, our child faces CAN be split
	bool canSplitAll = true;
	for (const auto& faceId : _faceIds) {
		faceFunc(faceId, [&centerPoint, &canSplitAll](Polygon& face) {
			if (canSplitAll) {
				set<Index3DId> newFaceIds;
				canSplitAll = face.splitAtPoint(centerPoint, newFaceIds, true);
			}
		});
		if (!canSplitAll)
			break;
	}
	
	if (!canSplitAll)
		return false;

	Vector3d ctr = calCentroid();
	Index3DId cellMidId = getBlockPtr()->addVertex(ctr);
	for (const auto& faceId : _faceIds) {
		faceFunc(faceId, [&centerPoint](Polygon& face) {
			set<Index3DId> temp;
			face.splitAtPoint(centerPoint, temp, false);
		});
	}

	set<Index3DId> corners;
	getVertIds(corners);
	while (!corners.empty()) {
		Index3DId vert = *corners.begin();
		corners.erase(vert);
		auto vertFaces = getVertFaces(vert);
		map<Edge, set<Index3DId>> vertEdgeFaceMap;
		for (const auto& faceId : vertFaces) {
			faceFunc(faceId, [&vert, &vertEdgeFaceMap, &vertFaces](const Polygon& face) {
				set<Edge> faceEdges;
				face.getEdges(faceEdges);
				for (const auto& edge : faceEdges) {
					if (edge.containsVertex(vert)) {
						auto iter = vertEdgeFaceMap.find(edge);
						if (iter == vertEdgeFaceMap.end())
							iter = vertEdgeFaceMap.insert(make_pair(edge, set<Index3DId>())).first;
						iter->second.insert(face.getId());
						vertFaces.insert(face.getId());
					}
				}
			});
		}

		for (const auto& pair : vertEdgeFaceMap) {
			const auto& edge = pair.first;
			const auto& edgeFaces = pair.second;

			const auto& edgeVert1 = edge.getOtherVert(vert);

			if (edgeFaces.size() != 2) {
				assert(!"Should never happen, the edge must have two faces attached");
				return false;
			}
			Index3DId faceVert0, faceVert1;
			auto iter = edgeFaces.begin();
			faceFunc(*iter++, [&faceVert0](const Polygon& face0) { faceVert0 = face0.getVertexIds()[0]; });
			faceFunc(*iter, [&faceVert1](const Polygon& face0) { faceVert1 = face0.getVertexIds()[0]; });

			vector<Index3DId> verts = {
				edgeVert1,
				faceVert0,
				cellMidId,
				faceVert1,
			};
			vertFaces.insert(addFace(verts));
		}
		assert(vertFaces.size() == 6);
		Polyhedron newCell(vertFaces);
		newCell.setParent(_thisId);
		auto newCellId = getBlockPtr()->addCell(newCell);
		cellFunc(newCellId, [&newCellId](Polyhedron& newCell) {
			assert(newCell.isClosed());
		});

		for (const auto& faceId : vertFaces) {
			faceFunc(faceId, [&newCellId](Polygon& face) {
				face.addCellId(newCellId);
			});
		}

		cellFunc(_thisId, [&newCellId](Polyhedron& self) {
			self.addChild(newCellId);
		});
	}	

	return true;
}

Index3DId Polyhedron::addFace(const std::vector<Index3DId>& vertIds)
{
	Polygon face(vertIds);
#if _DEBUG
	set<Edge> edges;
	face.getEdges(edges);
	for (const auto& edge : edges) {
		assert(edge.onPrincipalAxis(getBlockPtr()));
	}
#endif

	return getBlockPtr()->addFace(face);
}


void Polyhedron::finishCellSplits()
{
	set<Index3DId> newFaceSet;
	bool needsPromotion = false;
	for (const auto& faceId : _faceIds) {
		faceFunc(faceId, [this, &needsPromotion, &newFaceSet](Polygon& face) {
			const auto& childIds = face.getChildIds();
			if (childIds.empty()) {
				newFaceSet.insert(face.getId()); // This one's fine, just move it to the new list
			} else {
				needsPromotion = true;
				newFaceSet.insert(childIds.begin(), childIds.end());
			}
		});
	}

	vector<Index3DId> newFaceIds;
	newFaceIds.insert(newFaceIds.end(), newFaceSet.begin(), newFaceSet.end());
	for (size_t i = 0; i < newFaceIds.size(); i++) {
		for (size_t j = i + 1; j < newFaceIds.size(); j++) {
			faceFunc(newFaceIds[i], [this, &newFaceIds, j](Polygon& face0) {
				faceFunc(newFaceIds[j], [&face0](Polygon& face1) {
					face0.imprintFaceVertices(face1);
					face1.imprintFaceVertices(face0);
				});
			});
		}
	}

	if (needsPromotion) {
		cellFunc(_thisId, [this, &newFaceSet](Polyhedron& outputCell) {
			outputCell._faceIds = newFaceSet;
			for (const auto& faceId : outputCell._faceIds) {
				faceFunc(faceId, [&outputCell](Polygon& newFace) {
					newFace.addCellId(outputCell.getId());
				});
			}
		});
	}
}

bool Polyhedron::splitAtCentroid(std::set<Index3DId>& newCellIds)
{
	return splitAtPoint(calCentroid(), newCellIds);
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

void Polyhedron::splitByCurvature(double maxArcAngleDegrees)
{
	const double sinEdgeAngle = sin(30.0 / 180.0 * M_PI);
	const double minCurvature = 0.1; // 1 meter radius
	const double kDiv = 4.0;
	const size_t circleDivs = (size_t)(360.0 / maxArcAngleDegrees);

	auto pTriMesh =getBlockPtr()->getModelMesh();

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
			if (!splitAtCentroid(splitCells)) {
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
	if (_children.empty())
		return valid;
#ifdef _DEBUG 

	if (!isClosed())
		valid = false;
	for (const auto& faceId : _faceIds) {
		if (getBlockPtr()->polygonExists(faceId)) {
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

