/*
This file is part of the DistFieldHexMesh application/library.

	The DistFieldHexMesh application/library is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	The DistFieldHexMesh application/library is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	This link provides the exact terms of the GPL license <https://www.gnu.org/licenses/>.

	The author's interpretation of GPL 3 is that if you receive money for the use or distribution of the DistFieldHexMesh application/library or a derivative product, GPL 3 no longer applies.

	Under those circumstances, the author expects and may legally pursue a reasoble share of the income. To avoid the complexity of agreements and negotiation, the author makes
	no specific demands in this regard. Compensation of roughly 1% of net or $5 per user license seems appropriate, but is not legally binding.

	In lay terms, if you make a profit by using the DistFieldHexMesh application/library (violating the spirit of Open Source Software), I expect a reasonable share for my efforts.

	Robert R Tipton - Author

	Dark Sky Innovative Solutions http://darkskyinnovation.com/
*/

#include <vector>
#include <map>

#include <defines.h>
#include <cmath>

#include <vertex.h>
#include <edge.h>
#include <polygon.h>
#include <polyhedron.h>
#include <block.h>
#include <volume.h>
#include <logger.h>

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

Polyhedron::~Polyhedron()
{
	if (!isOwnerBeingDestroyed()) {
		unlinkFromFaces();
	}
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

void Polyhedron::unlinkFromFaces() const
{
	if (_thisId.isValid()) {
		Block* pBlk = const_cast<Block*> (getBlockPtr());
		for (const auto& faceId : _faceIds) {
			pBlk->faceFunc(faceId, [this](Polygon& face) {
				face.unlinkFromCell(_thisId);
			});
		}
	}
}

Polyhedron& Polyhedron::operator = (const Polyhedron& rhs)
{
	unlinkFromFaces();

	ObjectPoolOwnerUser::operator=(rhs);

	_intersectsModel = rhs._intersectsModel; // Cached value
	_needsCurvatureCheck = rhs._needsCurvatureCheck;
	_faceIds = rhs._faceIds;

	return *this;
}

void Polyhedron::addFace(const Index3DId& faceId)
{
	_faceIds.insert(faceId);
	faceFunc(faceId, [this](Polygon& face) {
		face.addCellId(_thisId);
	});
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

void Polyhedron::getEdges(set<Edge>& edges, bool includeAdjacentCellFaces) const
{
	map<Edge, set<Index3DId>> edgeToFaceMap;
	set<Index3DId> adjCellIds;
	for (const auto& faceId : _faceIds) {
		faceFunc(faceId, [this, &edgeToFaceMap, &faceId, &adjCellIds, includeAdjacentCellFaces](const Polygon& face) {
			if (includeAdjacentCellFaces) {
				auto temp = face.getCellIds();
				adjCellIds.insert(temp.begin(), temp.end());
			}
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

	if (includeAdjacentCellFaces) {
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

set<Index3DId> Polyhedron::getAdjacentCells(bool includeCornerCells) const
{
	set<Index3DId> cellIds;

	if (includeCornerCells) {
		set<Edge> edges;
		getEdges(edges, true);
		for (const auto& edge : edges) {
			const auto& faceIds = edge.getFaceIds();
			for (const auto& faceId : faceIds) {
				faceFunc(faceId, [&cellIds](const Polygon& face) {
					const auto& tmp = face.getCellIds();
					cellIds.insert(tmp.begin(), tmp.end());
				});
			}
		}
	} else {
		for (const auto& faceId : _faceIds) {
			faceFunc(faceId, [this, &cellIds](const Polygon& face) {
				const auto temp = face.getCellIds();
				cellIds.insert(temp.begin(), temp.end());
				});
		}
	}

	cellIds.erase(_thisId);
	return cellIds;
}

// Gets the edges for a vertex which belong to this polyhedron
void Polyhedron::getVertEdges(const Index3DId& vertId, set<Edge>& result, bool includeAdjacentCells) const
{
	set<Edge> cellEdgeSet;
	getEdges(cellEdgeSet, includeAdjacentCells);
	if (includeAdjacentCells) {
		set<Index3DId> adjCells = getAdjacentCells(false);
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
	set<Index3DId> vertIds;
	getVertIds(vertIds);
	for (const auto& vertId : vertIds) {
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

bool Polyhedron::intersectsModel() const
{
	if (_intersectsModel == IS_UNKNOWN) {
		CBoundingBox3Dd bbox = getBoundingBox();
		auto pTriMesh = getBlockPtr()->getModelMesh();
		auto sharps = getBlockPtr()->getVolume()->getSharpVertIndices();
		for (size_t idx : sharps) {
			if (bbox.contains(pTriMesh->getVert(idx)._pt))
				return true;
		}
		vector<CMesh::SearchEntry> triEntries;
		_intersectsModel = pTriMesh->findTris(bbox, triEntries) > 0 ? IS_TRUE : IS_FALSE;
	}

	return _intersectsModel == IS_TRUE; // Don't test split cells
}

bool Polyhedron::canSplitFaceWithoutSplitting(const Index3DId& faceId) const
{
	set<Edge> edges;
	getEdges(edges, false);
	bool result = true;
	faceFunc(faceId, [this, &edges, &result](const Polygon& face) {
		const double sinTol = sin(Tolerance::angleTol());
		const auto& vertIds = face.getVertexIds();
		for (size_t i = 0; i < vertIds.size(); i++) {
			size_t j = (i + 1) % vertIds.size();
			auto iter = edges.find(Edge(vertIds[i], vertIds[j]));
			if (iter != edges.end()) {
				const auto& edge = *iter;
				const auto& faceIds = iter->getFaceIds();
				assert(faceIds.size() == 2);
				auto fIter = faceIds.begin();
				const auto& faceId0 = *fIter++;
				const auto& faceId1 = *fIter++;
				Vector3d norm0, norm1;
				faceFunc(faceId0, [&norm0](const Polygon& face) { norm0 = face.calUnitNormal(); });
				faceFunc(faceId1, [&norm1](const Polygon& face) { norm1 = face.calUnitNormal(); });
				double cp = norm1.cross(norm0).norm();
				if (cp < sinTol) {
					result = false;
					break;
				}
			}
		}
	});
	return result;
}

void Polyhedron::splitAtCentroid() const
{
	auto ctr = calCentroid();

	splitAtPoint(ctr);
}

void Polyhedron::splitAtPoint(const Vector3d& centerPoint) const
{
	Block* pBlk = const_cast<Block*> (getBlockPtr());
	assert(pBlk);
	assert(!pBlk->polyhedronExists(TS_REAL, _thisId));
	assert(pBlk->isPolyhedronReference(this));
#if LOGGING_ENABLED
	auto pLogger = pBlk->getLogger();
	auto& out = pLogger->getStream();

	LOG(out << Logger::Pad() << "Polyhedron::splitRefFaceAtPoint " << *this);
#endif

	if (Index3DId(4, 6, 0, 5) == _thisId) {
		int dbgBreak = 1;
	}

	Index3DId cellMidId = pBlk->addVertex(centerPoint);
	map<Index3DId, set<Index3DId>> vertToFaceMap;
	for (const auto& faceId : _faceIds) {
		assert(pBlk->polygonExists(TS_REFERENCE, faceId));
		pBlk->faceRefFunc(faceId, [this, &centerPoint, &vertToFaceMap, pBlk](Polygon& refFace) {
			set<Index3DId> childFaceIds = refFace._splitProductIds;
			assert(childFaceIds.size() == refFace.getVertexIds().size());
			for (const auto& childFaceId : childFaceIds) {
				assert(pBlk->polygonExists(TS_REAL, childFaceId));
				assert(!pBlk->polygonExists(TS_REFERENCE, childFaceId));
				pBlk->faceFunc(childFaceId, [this, &vertToFaceMap](const Polygon& childFace) {
					for (const auto& vertId : childFace.getVertexIds()) {
						auto iter = vertToFaceMap.find(vertId);
						if (iter == vertToFaceMap.end()) {
							iter = vertToFaceMap.insert(make_pair(vertId, set<Index3DId>())).first;
						}
						iter->second.insert(childFace.getId());
					}
				});
			}
		});
	}

	set<Index3DId> corners;
	getVertIds(corners);

	for (const auto& vert : corners) {
		auto vertFaces = vertToFaceMap.find(vert)->second;
		assert(vertFaces.count(_thisId) == 0);
		map<Edge, set<Index3DId>> vertEdgeFaceMap;
		for (const auto& faceId : vertFaces) {
			pBlk->faceFunc(faceId, [&vert, &vertEdgeFaceMap, &vertFaces](const Polygon& face) {
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
				return;
			}
			Index3DId faceVert0, faceVert1;
			auto iter = edgeFaces.begin();
			pBlk->faceFunc(*iter++, [&faceVert0](const Polygon& face0) { faceVert0 = face0.getVertexIds()[0]; });
			pBlk->faceFunc(*iter, [&faceVert1](const Polygon& face0) { faceVert1 = face0.getVertexIds()[0]; });

			vector<Index3DId> verts = {
				edgeVert1,
				faceVert0,
				cellMidId,
				faceVert1,
			};

			auto newFaceId = pBlk->addFace(verts);
			vertFaces.insert(newFaceId);
		}

		Polyhedron newCell(vertFaces);

		auto newCellId = pBlk->addCell(newCell);

#if LOGGING_ENABLED
		pBlk->cellFunc(newCellId, [this, &out](Polyhedron& newCell) {
			Logger::Indent indent;
			LOG(out << Logger::Pad() << "to: " << newCell);
		});
#endif
	}

	LOG(out << Logger::Pad() << "Post split " << *this << "\n" << Logger::Pad() << "=================================================================================================\n");
}

bool Polyhedron::isSplitRequired() const
{
	return getBlockPtr()->isPolyhedronInSplitList(_thisId);
}

void Polyhedron::setNeedToSplitAtCentroid()
{
	if (getBlockPtr()->isPolyhedronInSplitList(_thisId))
		return;
#if LOGGING_ENABLED
	auto pLogger = getBlockPtr()->getLogger();
	auto& out = pLogger->getStream();
#endif

	Logger::Indent indent;
	LOG(out << Logger::Pad() << "setNeedToSplitAtCentroid c" << _thisId << "\n");

	getBlockPtr()->addPolyhedronToSplitList(_thisId);

	for (const auto& faceId : _faceIds) {
		faceFunc(faceId, [](Polygon& face) {
			face.setNeedToSplit();
		});
	}
}

void Polyhedron::setNeedToSplitCurvature(int divsPerRadius, double maxCurvatureRadius, double sinEdgeAngle)
{
#if LOGGING_ENABLED
	auto pLogger = getBlockPtr()->getLogger();
	auto& out = pLogger->getStream();
#endif

	CBoundingBox3Dd bbox = getBoundingBox();

	bool needToSplit = false;
#if 1 // Split at sharp vertices
	auto sharpVerts = getBlockPtr()->getVolume()->getSharpVertIndices();
	for (size_t vertIdx : sharpVerts) {
		auto pTriMesh = getBlockPtr()->getModelMesh();
		Vector3d pt = pTriMesh->getVert(vertIdx)._pt;
		if (bbox.contains(pt)) {
			needToSplit = true;
			break;
		}
	}
#endif

	if (!needToSplit) {
		double refRadius = calReferenceSurfaceRadius(bbox, maxCurvatureRadius, sinEdgeAngle);
		if (refRadius > 0 && refRadius < maxCurvatureRadius) {
			double maxLength = refRadius / divsPerRadius;
			auto range = bbox.range();
			double minDim = min(range[0], min(range[1], range[2]));
			if (minDim > maxLength) {
				needToSplit = true;
			}
		}
	}

	if (needToSplit) {
		Logger::Indent indent;
		LOG(out << Logger::Pad() << "setNeedToSplitCurvature c" << _thisId << "\n");
		setNeedToSplitAtCentroid();
	}
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
				if (edgeFaces.count(faceId) != 0) {
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

double Polyhedron::calReferenceSurfaceRadius(const CBoundingBox3Dd& bbox, double maxCurvatureRadius, double sinEdgeAngle) const
{
	auto pTriMesh = getBlockPtr()->getModelMesh();
	vector<CMesh::SearchEntry> edgeEntries;
	if (pTriMesh->findEdges(bbox, edgeEntries) > 0) {
		vector<double> edgeRadii;
		for (const auto& edgeEntry : edgeEntries) {
			size_t edgeIdx = edgeEntry.getIndex();
			double edgeCurv = pTriMesh->edgeCurvature(edgeIdx);
			double edgeRad = edgeCurv > 0 ? 1 / edgeCurv : 10000;
			if (edgeRad > 0 && edgeRad < maxCurvatureRadius)
				edgeRadii.push_back(edgeRad);
		}
		if (edgeRadii.empty())
			return 1e6;

		sort(edgeRadii.begin(), edgeRadii.end());
#if 0
		{
			static mutex m;
			lock_guard g(m);
			size_t idx = getBlockPtr()->getVolume()->calLinearBlockIndex(getId());
			cout << "Cell idx: " << idx << "\n";
			for (double r : edgeRadii)
				cout << "r: " << r << "\n";
			cout << "\n";
		}
#endif
		size_t num = std::min((size_t)10, edgeRadii.size());
		if (num == 0)
			return -1;
		double avgRad = 0;
		for (size_t i = 0; i < num; i++)
			avgRad += edgeRadii[i];
		avgRad /= num;
		if (avgRad > 0)
			return avgRad;
		return -1;
	}
	return 0;
}

double Polyhedron::getShortestEdge() const
{
	double minDist = DBL_MAX;
	for (const auto& faceId : _faceIds) {
		faceFunc(faceId, [&minDist](const Polygon& face) {
			double l = face.getShortestEdge();
			if (l < minDist)
				minDist = l;
		});
	}

	return minDist;
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
				if (face.containsVertex(vertId))
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
		if (getBlockPtr()->polygonExists(TS_REAL, faceId)) {
			faceFunc(faceId, [this, &valid](const Polygon& face) {
				if (!face.usedByCell(_thisId))
					valid = false;
				if (!face.verifyTopology())
					valid = false;
				});
		} else
			valid = false;
	}

	if (!isClosed())
		valid = false;
#endif // _DEBUG 

	return valid;
}

ostream& DFHM::operator << (ostream& out, const Polyhedron& cell)
{
	auto pBlk = cell.getBlockPtr();

	set<Edge> edges;
	cell.getEdges(edges, false);
	bool closed = true;
	for (const auto& edge : edges) {
		if (edge.getFaceIds().size() != 2)
			closed = false;
	}

	out << "Cell: c" << cell.getId() << (closed ? "CLOSED" : "ERROR NOT CLOSED") << "\n";
	{
		Logger::Indent indent;

		out << Logger::Pad() << "faces(" << cell._faceIds.size() << "): {\n";
		for (const auto& faceId : cell._faceIds) {
			pBlk->faceFunc(faceId, [&out](const Polygon& face) {
				Logger::Indent indent;
				out << Logger::Pad() << face << "\n";
			});
		}
		out << Logger::Pad() << "}\n";

		if (!closed) {
			out << Logger::Pad() << "edges(" << edges.size() << "): {\n";
			for (const auto& edge : edges) {
				edge.setBlockPtr(pBlk);
				Logger::Indent indent;
				out << Logger::Pad() << edge << "\n";
			}
			out << Logger::Pad() << "}\n";
		}
	}

	return out;
}
