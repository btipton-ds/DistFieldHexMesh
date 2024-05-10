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

#include <tm_lineSegment.h>
#include <tm_ioUtil.h>
#include <splitParams.h>
#include <vertex.h>
#include <edge.h>
#include <polygon.h>
#include <polyhedron.h>
#include <block.h>
#include <volume.h>
#include <logger.h>
#include <polyhedronSplitter.h>

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
	, _needsSplitAtCentroid(src._needsSplitAtCentroid)
	, _needsSplitAtPt(src._needsSplitAtPt)
	, _needsSplitAtPlane(src._needsSplitAtPlane)
	, _splitPt(src._splitPt)
	, _splitPlane(src._splitPlane)
{
}

Polyhedron& Polyhedron::operator = (const Polyhedron& rhs)
{
	clearCache();
	_faceIds = rhs._faceIds;
	_needsSplitAtCentroid = rhs._needsSplitAtCentroid;
	_needsSplitAtPt = rhs._needsSplitAtPt;
	_needsSplitAtPlane = rhs._needsSplitAtPlane;
	_splitPt = rhs._splitPt;
	_splitPlane = rhs._splitPlane;

	return *this;
}

void Polyhedron::dumpFaces() const
{
	size_t idx = 0;
	for (const auto& faceId : _faceIds) {
		cout << "face[" << idx++ << "]\n";
		faceFunc(TS_REAL,faceId, [](const Polygon& face) {
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

size_t Polyhedron::getNumSplitFaces() const
{
	size_t n = 0;
	for (const auto& faceId : _faceIds) {
		if (getBlockPtr()->polygonExists(TS_REFERENCE, faceId)) {
			faceFunc(TS_REAL,faceId, [&n](const Polygon& face) {
				if (face.isSplit())
					n++;
			});
		}
	}
	return n;
}

void Polyhedron::write(std::ostream& out) const
{
	uint8_t version = 0;
	out.write((char*)&version, sizeof(version));

	IoUtil::write(out, _faceIds);
	out.write((char*)&_splitLevel, sizeof(size_t));
}

void Polyhedron::read(std::istream& in)
{
	uint8_t version;
	in.read((char*)&version, sizeof(version));

	IoUtil::read(in, _faceIds);
	in.read((char*)&_splitLevel, sizeof(size_t));
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
	faceFunc(TS_REAL,faceId, [this, splitLevel](Polygon& face) {
		face.addCellId(_thisId, splitLevel);
	});

	clearCache();
}

void Polyhedron::getVertIds(set<Index3DId>& vertIds) const
{
	for (const auto& faceId : _faceIds) {
		faceAvailFunc(getState(), faceId, [&vertIds](const Polygon& refFace) {
			const auto vertexIds = refFace.getVertexIds();
			vertIds.insert(vertexIds.begin(), vertexIds.end());
		});
	}
}

const set<Edge>& Polyhedron::getEdges(bool includeAdjacentCellFaces) const
{
	bool cacheValid = includeAdjacentCellFaces ? _cachedEdges1Vaild : _cachedEdges0Vaild;
	if (!cacheValid) {
		auto curState = getState();
		map<Edge, set<Index3DId>> edgeToFaceMap;
		set<Index3DId> adjCellIds;
		for (const auto& faceId : _faceIds) {
			faceAvailFunc(curState, faceId, [this, &edgeToFaceMap, &faceId, &adjCellIds, includeAdjacentCellFaces](const Polygon& face) {
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
				cellAvailFunc(curState, adjCellId, [&faceIds](const Polyhedron& cell) {
					faceIds = cell.getFaceIds();
				});
				for (const auto& faceId : faceIds) {
					faceAvailFunc(curState, faceId, [this, &edgeToFaceMap, &faceId](const Polygon& face) {
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
				faceAvailFunc(getState(), faceId, [&cellIds](const Polygon& face) {
					const auto& tmp = face.getCellIds();
					cellIds.insert(tmp.begin(), tmp.end());
				});
			}
		}
	} else {
		for (const auto& faceId : _faceIds) {
			faceAvailFunc(getState(), faceId, [this, &cellIds](const Polygon& face) {
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
			cellAvailFunc(getState(), adjCellId, [&cellEdgeSet](const Polyhedron& adjCell) {
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
		faceFunc(TS_REAL,faceId, [this, &pt, &ctr, &result](const Polygon& face) {
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
		cellFunc(TS_REFERENCE,_thisId, [&ctr](const Polyhedron& self) {
			ctr = self.calCentroid();
		});
		return ctr;
	}

	auto bbox = getBoundingBox();
	Vector3d testCtr = (bbox.getMin() + bbox.getMax()) * 0.5;
	double area = 0;
	Vector3d ctr(0, 0, 0);

	for (const auto& faceId : _faceIds) {
		faceAvailFunc(getState(), faceId, [&area, &ctr](const Polygon& face) {
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
		vector<size_t> triEntries;
		_intersectsModel = getBlockPtr()->processTris(bbox, triEntries) > 0 ? IS_TRUE : IS_FALSE;
	}

	return _intersectsModel == IS_TRUE; // Don't test split cells
}

void Polyhedron::setNeedsSplitAtCentroid()
{
	_needsSplitAtCentroid = true;
	addToSplitStack();
}

bool Polyhedron::needsSplitAtCentroid() const
{
	return _needsSplitAtCentroid;
}

void Polyhedron::setNeedsSplitAtPoint(const Vector3d& splitPt)
{
	_needsSplitAtPt = true;
	_splitPt = splitPt;
	addToSplitStack();
}

bool Polyhedron::needsSplitAtPoint(Vector3d& splitPt) const
{
	if (_needsSplitAtPt) {
		splitPt = _splitPt;
		return true;
	}
	return false;
}

void Polyhedron::setNeedsSplitAtPlane(const Plane<double>& splitPlane)
{
	_needsSplitAtPlane = true;
	_splitPlane = splitPlane;
	addToSplitStack();
}

bool Polyhedron::needsSplitAtPlane(Plane<double>& splitPlane) const
{
	if (_needsSplitAtPlane) {
		splitPlane = _splitPlane;
		return true;
	}
	return false;
}

void Polyhedron::imprintTVertices(Block* pDstBlock)
{
#if DEBUG_BREAKS && defined(_DEBUG)
	if (Index3DId(3, 0, 5, 7) == _thisId) {
		int dbgBreak = 1;
	}
#endif

#if 1
	set<Index3DId> verts;
	for (const auto& faceId : _faceIds) {
		faceFunc(TS_REAL, faceId, [&verts](const Polygon& face) {
			const auto& tmp = face.getVertexIds();
			verts.insert(tmp.begin(), tmp.end());
		});
	}

	bool didImprint = false;
	for (const auto& faceId : _faceIds) {
		if (Index3DId(3, 1, 0, 2) == faceId) {
			int dbgBreak = 1;
		}
		set<Index3DId> imprintVertices;
		faceFunc(TS_REAL, faceId, [&verts, &imprintVertices](Polygon& face) {
			face.needToImprintVertices(verts, imprintVertices);
		});

		if (!imprintVertices.empty()) {
			getBlockPtr()->makeRefPolygonIfRequired(faceId);
			getBlockPtr()->removeFaceFromLookUp(faceId);
			faceFunc(TS_REAL, faceId, [&imprintVertices](Polygon& face) {
				for (const auto& vertId : imprintVertices) {
					face.imprintVertex(vertId);
				}
			});
			getBlockPtr()->addFaceToLookup(faceId);
			didImprint = true;
		}
	}
	if (didImprint)
		clearCache();
#else
	set<Index3DId> refFaceIds;
	cellFunc(TS_REFERENCE,_thisId, [&refFaceIds](const Polyhedron& refCell) {
		refFaceIds = refCell.getFaceIds();
	});

	map<Edge, Index3DId> splitEdgeVertMap;
	for (const auto& refFaceId : refFaceIds) {
		faceAvailFunc(TS_REFERENCE, refFaceId, [&splitEdgeVertMap](const Polygon& refFace) {
			const auto& tmp = refFace.getSplitEdgeVertMap();
			splitEdgeVertMap.insert(tmp.begin(), tmp.end());
		});
	}

	set<Index3DId> imprintIds;
	for (const auto& faceId : _faceIds) {
		if (getBlockPtr()->polygonExists(TS_REAL, faceId)) {
			faceFunc(TS_REAL,faceId, [&splitEdgeVertMap, &imprintIds](const Polygon& face) {
				if (face.needToImprintVertices(splitEdgeVertMap))
					imprintIds.insert(face.getId());
			});
		}
	}

	if (!imprintIds.empty()) {

		for (const auto& faceId : imprintIds) {
			pDstBlock->makeRefPolygonIfRequired(faceId);
			if (getBlockPtr()->polygonExists(TS_REAL, faceId)) {
				pDstBlock->faceFunc(TS_REAL,faceId, [&splitEdgeVertMap](Polygon& face) {
					face.imprintVertices(splitEdgeVertMap);
				});
			}
		}

		clearCache();
	}
#endif
}

void Polyhedron::replaceFaces(const Index3DId& curFaceId, const std::set<Index3DId>& newFaceIds, size_t splitLevel)
{
	clearCache();
	_faceIds.erase(curFaceId);

	faceFunc(TS_REAL,curFaceId, [this](Polygon& curFace) {
		curFace.removeCellId(_thisId);
	});

	for (const auto& newFaceId : newFaceIds) {
		_faceIds.insert(newFaceId);
		faceFunc(TS_REAL,newFaceId, [this, splitLevel](Polygon& newFace) {
			assert(!getBlockPtr()->isPolygonReference(&newFace));
			newFace.addCellId(_thisId, splitLevel);
		});
	}
}

void Polyhedron::addToSplitStack()
{
#if LOGGING_ENABLED
	auto pLogger = getBlockPtr()->getLogger();
	auto& out = pLogger->getStream();

	Logger::Indent indent;
	LOG(out << Logger::Pad() << "setNeedToSplitAtCentroid c" << _thisId << "\n");
#endif

	getBlockPtr()->addToSplitStack({ _thisId });
}

#ifdef _DEBUG
namespace
{

bool boxesEqualTol(const CBoundingBox3Dd& a, const CBoundingBox3Dd& b)
{
	const double tol = 1.0e-6;

	const auto& minA = a.getMin();
	const auto& minB = b.getMin();
	const auto& maxA = a.getMax();
	const auto& maxB = b.getMax();
	for (int i = 0; i < 3; i++) {
		if (fabs(minA[i] - minB[i]) > tol)
			return false;
		if (fabs(maxA[i] - maxB[i]) > tol)
			return false;
	}
	return true;
}

}

#endif

bool Polyhedron::canSplit(set<Index3DId>& blockingCellIds) const
{
	blockingCellIds.clear();
	for (const auto& faceId : _faceIds) {
		Polygon::CellId_SplitLevel adjCellId;
		size_t faceSplitLevel;
		faceAvailFunc(TS_REFERENCE, faceId, [this, &adjCellId, &faceSplitLevel](const Polygon& face) {
			for (const auto& id : face.getCellIds()) {
				if (id != _thisId) {
					adjCellId = id;
					faceSplitLevel = face.getSplitLevel(adjCellId);
				}
			}
		});

		if (adjCellId.getId().isValid()) {
			if (faceSplitLevel > 0) {
				blockingCellIds.insert(adjCellId);
			} else {
				cellAvailFunc(TS_REAL, adjCellId, [this, &blockingCellIds](const Polyhedron& adjCell) {
					if (adjCell.getSplitLevel() < _splitLevel) {
						blockingCellIds.insert(adjCell.getId());
					}
				});
			}
		}
	}

	return blockingCellIds.empty();
}

bool Polyhedron::needToSplitConditional(const BuildCFDParams& params)
{
#if LOGGING_ENABLED
	auto pLogger = getBlockPtr()->getLogger();
	auto& out = pLogger->getStream();
#endif
	if (!_needsConditionalSplitTest)
		return false;
	_needsConditionalSplitTest = false;
	CBoundingBox3Dd bbox = getBoundingBox();

	bool needToSplit = false;
	set<Edge> edges;
	cellAvailFunc(TS_REFERENCE, _thisId, [&edges](const Polyhedron& cell) {
		edges = cell.getEdges(false);
	});

	double maxEdgeLength = 0;
	for (const auto& edge : edges) {
		auto seg = edge.getSegment(getBlockPtr());
		double l = seg.calLength();
		if (l > maxEdgeLength)
			maxEdgeLength = l;
	}

#if 0 // Split at sharp vertices
	if (maxEdgeLength > params.minSplitEdgeLengthSharpVertex_meters) {
		int numSharps = 0;
		auto sharpVerts = getBlockPtr()->getVolume()->getSharpVertIndices();
		for (size_t vertIdx : sharpVerts) {
			auto pTriMesh = getBlockPtr()->getModelMesh();
			Vector3d pt = pTriMesh->getVert(vertIdx)._pt;
			if (bbox.contains(pt)) {
				numSharps++;
			}
		}
		if (numSharps > 1)
			needToSplit = true;
	}
#endif

	if (!needToSplit) {
		double refRadius = calReferenceSurfaceRadius(bbox, params);
		if (refRadius > 0) {
			double gap = minGap();
			double maxAllowedEdgeLen = DBL_MAX;
			if (gap < params.maxGapSize) {
				if (maxEdgeLength > params.minSplitEdgeLengthGapCurvature_meters) {
					maxAllowedEdgeLen = refRadius / params.divsPerGapCurvatureRadius;
				}
			} else {
				if (maxEdgeLength > params.minSplitEdgeLengthCurvature_meters) {
					maxAllowedEdgeLen = refRadius / params.divsPerCurvatureRadius;
				}
			}
			if (maxEdgeLength > maxAllowedEdgeLen)
				needToSplit = true;
		}
	}

	if (needToSplit) {
		Logger::Indent indent;
		LOG(out << Logger::Pad() << "setNeedToSplitCurvature c" << _thisId << "\n");
		setNeedsSplitAtCentroid();
	}

	return needToSplit;
}

bool Polyhedron::needToSplitDueToSplitFaces(const BuildCFDParams& params)
{
	if (_faceIds.size() > params.maxCellFaces) {
		setNeedsSplitAtCentroid();
		return true;
	}

	return false;
}

bool Polyhedron::setNeedToSplitSharpVertices(const BuildCFDParams& params, const vector<size_t>& sharpVerts)
{
	auto bbox = getBoundingBox();
	auto pMesh = getBlockPtr()->getModelMesh();
	for (size_t idx : sharpVerts) {
		Vector3d pt = pMesh->getVert(idx)._pt;
		if (bbox.contains(pt)) {
			double minDist = DBL_MAX;
			for (const auto& faceId : _faceIds) {
				faceAvailFunc(TS_REAL, faceId, [&minDist, &pt](const Polygon& face) {
					double d = fabs(face.distanceToPoint(pt));
					if (d < minDist)
						minDist = d;
				});
			}

			// Don't split if the point is already close enough to a face
			if (minDist > Tolerance::sameDistTol()) {
				Plane<double> plane;
				auto pVol = getBlockPtr()->getVolume();
				bool hasPlane = pVol->getSharpVertPlane(plane);
				if (hasPlane) {
					setNeedsSplitAtPlane(plane);
				} else {
					setNeedsSplitAtPoint(pt);
				}
				return true;
			}
		}
	}
	return false;
}

void Polyhedron::setEdgeIndices(const std::vector<size_t>& indices)
{
#if 0
	auto pTriMesh = getBlockPtr()->getModelMesh();
	auto bbox = getBoundingBox();

	for (auto idx : indices) {
		const auto& edge = pTriMesh->getEdge(idx);
		auto seg = edge.getSeg(pTriMesh);
		if (bbox.intersects(seg))
			_edgeIndices.push_back(idx);
	}
#endif
}

void Polyhedron::setTriIndices(const std::vector<size_t>& indices)
{
	auto pTriMesh = getBlockPtr()->getModelMesh();
	auto bbox = getBoundingBox();

	pTriMesh->processFoundTris(indices, bbox, _triIndices);
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

double Polyhedron::calReferenceSurfaceRadius(const CBoundingBox3Dd& bbox, const BuildCFDParams& params) const
{
	if (!_needsCurvatureCheck)
		return 0;
	_needsCurvatureCheck = false;

	if (_triIndices.empty())
		return 0;

	auto pTriMesh = getBlockPtr()->getModelMesh();
#if 1
	const auto& blkIdx = _thisId.blockIdx();
	if (blkIdx[0] == 0 && blkIdx[1] == 0) {
		int dbgBreak = 1;
	}

	vector<size_t> triIndices;
//	size_t numTris = getBlockPtr()->processTris(bbox, triIndices);
	size_t numTris = pTriMesh->processFoundTris(_triIndices, bbox, triIndices);
	if (numTris > 0) {
		static thread_local vector<double> radii;
		if (radii.size() < pTriMesh->numTris()) {
			radii.reserve(pTriMesh->numTris());
		}
		radii.clear();
		for (const auto triIdx : triIndices) {
			double triCurv = pTriMesh->triCurvature(triIdx);
			if (triCurv > 2) { // Radius < 1/2
				radii.push_back(1 / triCurv);
			}
		}

		if (radii.empty())
			return 1e6;

		sort(radii.begin(), radii.end());
		double maxRad = 0;
		double avgRad = 0;
		size_t count = 0;
		for (size_t i = 0; i < radii.size(); i++) {
			if (maxRad == 0 || radii[i] <= 1.5 * maxRad) {
				if (radii[i] > maxRad)
					maxRad = radii[i];

				avgRad += radii[i];
				count++;
			} else
				break;
		}

		if (count > 0 && avgRad >= 0) {
			avgRad /= count;
			return avgRad;
		}
		return -1;
	}
#else
	vector<size_t> edgeIndices;
	if (getBlockPtr()->processEdges(bbox, edgeIndices)) {
		vector<double> edgeRadii;
		double maxRad = 0;
		for (const auto edgeIdx : edgeIndices) {
			double edgeCurv = pTriMesh->edgeCurvature(edgeIdx);
			if (edgeCurv > 2) { // Radius < 1/2
				double edgeRad = 1 / edgeCurv;
				if (maxRad == 0 || edgeRad <= 1.5 * maxRad) {
					if (edgeRad > maxRad)
						maxRad = edgeRad;
					edgeRadii.push_back(edgeRad);
				} else
					break;
			}
		}
		if (edgeRadii.empty())
			return 1e6;

		sort(edgeRadii.begin(), edgeRadii.end());
		size_t num = edgeRadii.size();
		double avgRad = 0;
		for (size_t i = 0; i < num; i++) {
			avgRad += edgeRadii[i];
		}

		avgRad /= num;
		if (avgRad >= 0)
			return avgRad;
		return -1;
	}
#endif

	return 0;
}

double Polyhedron::minGap() const
{
	double result = DBL_MAX;

	auto pTriMesh = getBlockPtr()->getModelMesh();
	auto bbox = getBoundingBox();
	vector<size_t> triIndices;
	if (getBlockPtr()->processTris(bbox, triIndices)) {
		for (size_t idx : triIndices) {
			double gap = pTriMesh->triGap(idx);
			if (gap < result)
				result = gap;
		}
	}

	return result;
}

bool Polyhedron::polygonExists(TopolgyState refState, const Index3DId& id) const
{
	return getBlockPtr()->polygonExists(refState, id);
}

double Polyhedron::getShortestEdge() const
{
	double minDist = DBL_MAX;
	for (const auto& faceId : _faceIds) {
		faceFunc(TS_REAL,faceId, [&minDist](const Polygon& face) {
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
		faceFunc(TS_REAL,faceId, [this, &edgeSet, &vertIds](const Polygon& face) {
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
		FixedPlane(const Plane<double>& src)
			: _origin(src.getOrgin())
			, _normal(src.getNormal())
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
		faceFunc(TS_REAL,faceId, [this, &result](const Polygon& face) {
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
			faceFunc(TS_REAL,faceId, [this, &valid](const Polygon& face) {
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

#if DUMP_BAD_CELL_OBJS
	if (!valid) {
		getBlockPtr()->dumpObj({ _thisId });
	}
#endif

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
			pBlk->faceAvailFunc(TS_REAL, faceId, [&out](const Polygon& face) {
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
void Polyhedron::vertexFunc(const Index3DId& id, const std::function<void(const Vertex& obj)>& func) const {
	auto p = getBlockPtr(); 
	p->vertexFunc(id, func);
} 

void Polyhedron::vertexFunc(const Index3DId& id, const std::function<void(Vertex& obj)>& func) {
	auto p = getBlockPtr(); 
	p->vertexFunc(id, func);
} 

void Polyhedron::faceFunc(TopolgyState state, const Index3DId& id, const std::function<void(const Polygon& obj)>& func) const {
	auto p = getBlockPtr(); 
	p->faceFunc(state, id, func);
} 

void Polyhedron::faceFunc(TopolgyState state, const Index3DId& id, const std::function<void(Polygon& obj)>& func) {
	auto p = getBlockPtr(); 
	p->faceFunc(state, id, func);
} 

void Polyhedron::cellFunc(TopolgyState state, const Index3DId& id, const std::function<void(const Polyhedron& obj)>& func) const {
	auto p = getBlockPtr(); 
	p->cellFunc(state, id, func);
} 

void Polyhedron::cellFunc(TopolgyState state, const Index3DId& id, const std::function<void(Polyhedron& obj)>& func) {
	auto p = getBlockPtr(); 
	p->cellFunc(state, id, func);
} 

void Polyhedron::faceAvailFunc(TopolgyState prefState, const Index3DId& id, const std::function<void(const Polygon& obj)>& func) const {
	auto p = getBlockPtr(); 
	p->faceAvailFunc(prefState, id, func);
} 

void Polyhedron::cellAvailFunc(TopolgyState prefState, const Index3DId& id, const std::function<void(const Polyhedron& obj)>& func) const {
	auto p = getBlockPtr(); 
	p->cellAvailFunc(prefState, id, func);
}
