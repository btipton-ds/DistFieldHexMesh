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
#include <splitters.h>

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
	: _faceIds(src._faceIds)
{
}

Polyhedron& Polyhedron::operator = (const Polyhedron& rhs)
{
	clearCache();
	_faceIds = rhs._faceIds;

	return *this;
}

void Polyhedron::dumpFaces() const
{
	size_t idx = 0;
	for (const auto& faceId : _faceIds) {
		cout << "face[" << idx++ << "]\n";
		faceRealFunc(faceId, [](const Polygon& face) {
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

TopolgyState Polyhedron::getState() const
{
	if (getBlockPtr()->isPolyhedronReference(this))
		return TS_REFERENCE;
	else
		return TS_REAL;
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

void Polyhedron::addFace(const Index3DId& faceId, size_t splitLevel)
{
	_faceIds.insert(faceId);
	faceRealFunc(faceId, [this, splitLevel](Polygon& face) {
		face.addCellId(_thisId, splitLevel);
	});

	clearCache();
}

void Polyhedron::getVertIds(set<Index3DId>& vertIds) const
{
	for (const auto& faceId : _faceIds) {
		faceAvailFunc(faceId, getState(), [&vertIds](const Polygon& refFace) {
			const auto vertexIds = refFace.getVertexIds();
			vertIds.insert(vertexIds.begin(), vertexIds.end());
		});
	}
}

const set<Edge>& Polyhedron::getEdges(bool includeAdjacentCellFaces) const
{
	bool cacheValid = includeAdjacentCellFaces ? _cachedEdges1Vaild : _cachedEdges0Vaild;
	if (!cacheValid) {
		map<Edge, set<Index3DId>> edgeToFaceMap;
		set<Index3DId> adjCellIds;
		for (const auto& faceId : _faceIds) {
			faceAvailFunc(faceId, getState(), [this, &edgeToFaceMap, &faceId, &adjCellIds, includeAdjacentCellFaces](const Polygon& face) {
				if (includeAdjacentCellFaces) {
					auto temp = face.getCellIds();
					adjCellIds.insert(temp.begin(), temp.end());
				}
				const auto& edges = face.getEdges();
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
				cellAvailFunc(adjCellId, getState(), [&faceIds](const Polyhedron& cell) {
					faceIds = cell.getFaceIds();
					});
				for (const auto& faceId : faceIds) {
					faceAvailFunc(faceId, getState(), [this, &edgeToFaceMap, &faceId](const Polygon& face) {
						const auto& edges = face.getEdges();
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
			if (includeAdjacentCellFaces) {
				_cachedEdges1.insert(Edge(pair.first, pair.second));
			} else {
				_cachedEdges0.insert(Edge(pair.first, pair.second));
			}
		}
	}

	if (includeAdjacentCellFaces) {
		_cachedEdges1Vaild = true;
		return _cachedEdges1;
	} else {
		_cachedEdges0Vaild = true;
		return _cachedEdges0;
	}
}

set<Index3DId> Polyhedron::getAdjacentCells(bool includeCornerCells) const
{
	set<Index3DId> cellIds;

	if (includeCornerCells) {
		const auto& edges = getEdges(true);
		for (const auto& edge : edges) {
			const auto& faceIds = edge.getFaceIds();
			for (const auto& faceId : faceIds) {
				faceAvailFunc(faceId, getState(), [&cellIds](const Polygon& face) {
					const auto& tmp = face.getCellIds();
					cellIds.insert(tmp.begin(), tmp.end());
				});
			}
		}
	} else {
		for (const auto& faceId : _faceIds) {
			faceAvailFunc(faceId, getState(), [this, &cellIds](const Polygon& face) {
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
	auto cellEdgeSet = getEdges(includeAdjacentCells);
	if (includeAdjacentCells) {
		set<Index3DId> adjCells = getAdjacentCells(false);
		for (const auto& adjCellId : adjCells) {
			cellAvailFunc(adjCellId, getState(), [&cellEdgeSet](const Polyhedron& adjCell) {
				const auto& tmp = adjCell.getEdges(true);
				cellEdgeSet.insert(tmp.begin(), tmp.end());
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
		faceRealFunc(faceId, [this, &pt, &ctr, &result](const Polygon& face) {
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
	if (!getBlockPtr()->isPolyhedronReference(this) && getBlockPtr()->polyhedronExists(TS_REFERENCE, _thisId)) {
		Vector3d ctr;
		cellRefFunc(_thisId, [&ctr](const Polyhedron& self) {
			ctr = self.calCentroid();
		});
		return ctr;
	}

	auto bbox = getBoundingBox();
	Vector3d testCtr = (bbox.getMin() + bbox.getMax()) * 0.5;
	double area = 0;
	Vector3d ctr(0, 0, 0);

	for (const auto& faceId : _faceIds) {
		faceAvailFunc(faceId, getState(), [&area, &ctr](const Polygon& face) {
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

bool Polyhedron::needToImprintTVertices() const
{
	return !isClosed();
}

void Polyhedron::imprintTVertices(Block* pDstBlock) const
{
	if (Index3DId(3, 0, 5, 7) == _thisId) {
		int dbgBreak = 1;
	}
	set<Index3DId> refFaceIds;
	cellRefFunc(_thisId, [&refFaceIds](const Polyhedron& refCell) {
		refFaceIds = refCell.getFaceIds();
	});

	map<Edge, Index3DId> splitEdgeVertMap;
	for (const auto& refFaceId : refFaceIds) {
		faceAvailFunc(refFaceId, TS_REFERENCE, [&splitEdgeVertMap](const Polygon& refFace) {
			const auto& tmp = refFace.getSplitEdgeVertMap();
			splitEdgeVertMap.insert(tmp.begin(), tmp.end());
		});
	}

	set<Index3DId> imprintIds;
	for (const auto& faceId : _faceIds) {
		if (getBlockPtr()->polygonExists(TS_REAL, faceId)) {
			faceRealFunc(faceId, [&splitEdgeVertMap, &imprintIds](const Polygon& face) {
				if (face.needToImprintVertices(splitEdgeVertMap))
					imprintIds.insert(face.getId());
			});
		}
	}

	for (const auto& faceId : imprintIds) {
		pDstBlock->makeRefPolygonIfRequired(faceId);
		if (getBlockPtr()->polygonExists(TS_REAL, faceId)) {
			pDstBlock->faceRealFunc(faceId, [&splitEdgeVertMap](Polygon& face) {
				face.imprintVertices(splitEdgeVertMap);
			});
		}
	}

	clearCache();
}

void Polyhedron::replaceFaces(const Index3DId& curFaceId, const std::set<Index3DId>& newFaceIds, size_t splitLevel)
{
	clearCache();
	_faceIds.erase(curFaceId);

	faceRealFunc(curFaceId, [this](Polygon& curFace) {
		curFace.removeCellId(_thisId);
	});

	for (const auto& newFaceId : newFaceIds) {
		_faceIds.insert(newFaceId);
		faceRealFunc(newFaceId, [this, splitLevel](Polygon& newFace) {
			assert(!getBlockPtr()->isPolygonReference(&newFace));
			newFace.addCellId(_thisId, splitLevel);
		});
	}
}

bool Polyhedron::canSplit(set<Index3DId>& blockingCellIds) const
{
	blockingCellIds.clear();
	for (const auto& faceId : _faceIds) {
		faceRealFunc(faceId, [this, &blockingCellIds](const Polygon& face) {
			for (const auto& cellId : face.getCellIds()) {
				if (cellId != _thisId) {
					if (face.getSplitLevel(cellId) > 0) {
						blockingCellIds.insert(cellId);
					}
				}
			}
		});

		if (getBlockPtr()->polygonExists(TS_REFERENCE, faceId)) {
			faceRefFunc(faceId, [this, &blockingCellIds](const Polygon& refFace) {
				for (const auto& childFaceId : refFace.getSplitFaceProductIds()) {
					if (getBlockPtr()->polygonExists(TS_REFERENCE, childFaceId)) {
						blockingCellIds.insert(_thisId);
					}
				}
			});
		}
	}

	return blockingCellIds.empty();
}

bool Polyhedron::needsPreSplit() const
{
	if (Index3DId(0, 5, 4, 0) == _thisId) {
		int dbgBreak = 1;
	}
	// If this cell and its faces have never been modified, it cannot have split faces - skip it.
	if (!getBlockPtr()->polyhedronExists(TS_REFERENCE, _thisId))
		return false;
	set<Index3DId> refFaces;
	cellRefFunc(_thisId, [&refFaces](const Polyhedron& cell) {
		refFaces = cell.getFaceIds();
		});
	if (refFaces.size() == _faceIds.size())
		return false;

	map<Index3DId, set<Index3DId>> faceToCellMap;
	for (const auto& faceId : refFaces) {
		faceAvailFunc(faceId, TS_REFERENCE, [this, &faceToCellMap](const Polygon& face) {
			const auto& splits = face.getSplitFaceProductIds();
			if (splits.empty()) {
				for (const auto& cellId : face.getCellIds()) {
					if (cellId != _thisId) {
						auto iter = faceToCellMap.find(face.getId());
						if (iter == faceToCellMap.end()) {
							iter = faceToCellMap.insert(make_pair(face.getId(), set<Index3DId>())).first;
						}
						iter->second.insert(cellId);
					}
				}
			} else {
				for (const auto& splitFaceId : splits) {
					faceRealFunc(splitFaceId, [this, &face, &faceToCellMap](const Polygon& splitFace) {
						for (const auto& cellId : splitFace.getCellIds()) {
							if (cellId != _thisId) {
								auto iter = faceToCellMap.find(face.getId());
								if (iter == faceToCellMap.end()) {
									iter = faceToCellMap.insert(make_pair(face.getId(), set<Index3DId>())).first;
								}
								iter->second.insert(cellId);
							}
						}
						});
				}
			}
			});
	}

	bool needsSplit = false;
	for (const auto& pair : faceToCellMap) {
		const auto& refFaceId = pair.first;
		const auto& adjCellIds = pair.second;
		if (adjCellIds.size() > 1) {
			for (const auto& adjCellId : adjCellIds) {
				if (getBlockPtr()->isPolyhedronInSplitList(adjCellId)) {
					needsSplit = true;
					break;
				}
			}
		}
	}

	return needsSplit;
}

void Polyhedron::setNeedToSplitAtCentroid()
{
#if LOGGING_ENABLED
	auto pLogger = getBlockPtr()->getLogger();
	auto& out = pLogger->getStream();
#endif

	Logger::Indent indent;
	LOG(out << Logger::Pad() << "setNeedToSplitAtCentroid c" << _thisId << "\n");

	getBlockPtr()->addPolyhedronToSplitList(_thisId);
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

inline bool Polyhedron::polygonExists(TopolgyState refState, const Index3DId& id) const
{
	return getBlockPtr()->polygonExists(refState, id);
}

double Polyhedron::getShortestEdge() const
{
	double minDist = DBL_MAX;
	for (const auto& faceId : _faceIds) {
		faceRealFunc(faceId, [&minDist](const Polygon& face) {
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
		faceRealFunc(faceId, [this, &edgeSet, &vertIds](const Polygon& face) {
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
	const auto& edges = getEdges(false);
	for (const auto& edge : edges) {
		if (edge.getFaceIds().size() != 2) {
			result = false;
			break;
		}
	}

	return result;
}

namespace
{

	class FixedPlane {
	public:
		FixedPlane(const Plane& src)
			: _origin(src._origin)
			, _normal(src._normal)
		{
		}

		bool operator < (const FixedPlane& rhs) const
		{
			if (_origin < rhs._origin)
				return true;
			else if (rhs._origin < _origin)
				return false;

			const double sinAngleTol = sin(Tolerance::angleTol());
			Vector3d n0(FixedPt::toDbl(_normal)), n1(FixedPt::toDbl(rhs._normal));
			double cp = n1.cross(n0).norm();

			if (cp < sinAngleTol)
				return false;

			if (_normal < rhs._normal)
				return true;
			else if (rhs._normal < _normal)
				return false;
			return false;
		}

	private:
		FixedPt _origin, _normal;
	};
}

bool Polyhedron::hasTooManySplits() const
{
	bool result = false;

	for (const auto& faceId : _faceIds) {
		faceRealFunc(faceId, [this, &result](const Polygon& face) {
			if (face.getSplitLevel(_thisId) > 1)
				result = true;
		});
		if (result)
			break;
	}

	return result;
}

bool Polyhedron::verifyTopology() const
{
	bool valid = true;

	for (const auto& faceId : _faceIds) {
		if (valid && polygonExists(TS_REAL, faceId)) {
			faceRealFunc(faceId, [this, &valid](const Polygon& face) {
				if (valid && !face.usedByCell(_thisId)) {
					valid = false;
				}
				if (valid && !face.verifyTopology())
					valid = false;
				});
		} else
			valid = false;
	}

	if (valid && !isClosed()) {
		valid = false;
	}
	
	if (valid && hasTooManySplits())
		valid = false;
	if (false && !valid) {
		getBlockPtr()->dumpObj({ _thisId });
	}

	return valid;
}

void Polyhedron::clearCache() const
{
	_intersectsModel = IS_UNKNOWN; // Cached value
	_needsCurvatureCheck = true;

	_cachedEdges0Vaild = false;
	_cachedEdges1Vaild = false;

	_cachedEdges0.clear();
	_cachedEdges1.clear();
}

ostream& DFHM::operator << (ostream& out, const Polyhedron& cell)
{
	auto pBlk = cell.getBlockPtr();

	const auto& edges = cell.getEdges(false);
	bool closed = true;
	for (const auto& edge : edges) {
		if (edge.getFaceIds().size() != 2)
			closed = false;
	}

#if LOGGING_VERBOSE_ENABLED

	out << "Cell: c" << cell.getId() << (closed ? " CLOSED" : " ERROR NOT CLOSED") << "\n";
	{
		Logger::Indent indent;

		out << Logger::Pad() << "faces(" << cell._faceIds.size() << "): {\n";
		for (const auto& faceId : cell._faceIds) {
			pBlk->faceAvailFunc(faceId, TS_REAL, [&out](const Polygon& face) {
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
#else
	out << "Cell: c" << cell.getId() << (closed ? " CLOSED" : " ERROR NOT CLOSED") << "\n";
	{
		Logger::Indent indent;

		out << Logger::Pad() << "faceIds(" << cell._faceIds.size() << "): { ";
		for (const auto& faceId : cell._faceIds) {
			out << "f" << faceId << " ";
		}
		out << "}\n";
	}
#endif

return out;
}

//LAMBDA_CLIENT_IMPLS(Polyhedron)

void Polyhedron::vertexRealFunc(const Index3DId& id, const std::function<void(const Vertex& obj)>& func) const {
	auto p = getBlockPtr(); 
	p->vertexRealFunc(id, func);
} 

void Polyhedron::vertexRealFunc(const Index3DId& id, const std::function<void(Vertex& obj)>& func) {
	auto p = getBlockPtr(); 
	p->vertexRealFunc(id, func);
} 

void Polyhedron::faceRealFunc(const Index3DId& id, const std::function<void(const Polygon& obj)>& func) const {
	auto p = getBlockPtr(); 
	p->faceRealFunc(id, func);
} 

void Polyhedron::faceRealFunc(const Index3DId& id, const std::function<void(Polygon& obj)>& func) {
	auto p = getBlockPtr(); 
	p->faceRealFunc(id, func);
} 

void Polyhedron::cellRealFunc(const Index3DId& id, const std::function<void(const Polyhedron& obj)>& func) const {
	auto p = getBlockPtr(); 
	p->cellRealFunc(id, func);
} 

void Polyhedron::cellRealFunc(const Index3DId& id, const std::function<void(Polyhedron& obj)>& func) {
	auto p = getBlockPtr(); 
	p->cellRealFunc(id, func);
} 

void Polyhedron::faceRefFunc(const Index3DId& id, const std::function<void(const Polygon& obj)>& func) const {
	auto p = getBlockPtr(); 
	p->faceRefFunc(id, func);
} 

void Polyhedron::cellRefFunc(const Index3DId& id, const std::function<void(const Polyhedron& obj)>& func) const {
	auto p = getBlockPtr(); 
	p->cellRefFunc(id, func);
} 

void Polyhedron::faceAvailFunc(const Index3DId& id, TopolgyState prefState, const std::function<void(const Polygon& obj)>& func) const {
	auto p = getBlockPtr(); 
	p->faceAvailFunc(id, prefState, func);
} 

void Polyhedron::cellAvailFunc(const Index3DId& id, TopolgyState prefState, const std::function<void(const Polyhedron& obj)>& func) const {
	auto p = getBlockPtr(); 
	p->cellAvailFunc(id, prefState, func);
}

