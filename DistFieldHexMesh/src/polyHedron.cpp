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
#include <tm_lineSegment.hpp>
#include <tm_ioUtil.h>
#include <pool_vector.h>
#include <pool_map.h>
#include <pool_set.h>
#include <io_utils.h>
#include <splitParams.h>
#include <vertex.h>
#include <edge.h>
#include <polygon.h>
#include <polyhedron.h>
#include <block.h>
#include <volume.h>
#include <logger.h>
#include <polyhedronSplitter.h>
#include <tolerances.h>
#include <utils.h>

using namespace std;
using namespace DFHM;

Polyhedron::Polyhedron(const MultiCore::set<Index3DId>& faceIds)
	: _faceIds(faceIds)
{
}

Polyhedron::Polyhedron(const MultiCore::vector<Index3DId>& faceIds)
{
	_faceIds.insert(faceIds.begin(), faceIds.end());
}


Polyhedron::Polyhedron(const std::set<Index3DId>& faceIds)
	: _faceIds(faceIds)
{
}

Polyhedron::Polyhedron(const std::vector<Index3DId>& faceIds)
{
	_faceIds.insert(faceIds.begin(), faceIds.end());
}

Polyhedron::Polyhedron(const Polyhedron& src)
	: _faceIds(src._faceIds)
	, _needsSplitAtCentroid(src._needsSplitAtCentroid)
	, _cachedIsClosed(src._cachedIsClosed)
{
}

Polyhedron& Polyhedron::operator = (const Polyhedron& rhs)
{
	clearCache();
	_faceIds = rhs._faceIds;
	_needsSplitAtCentroid = rhs._needsSplitAtCentroid;
	_cachedIsClosed = rhs._cachedIsClosed;

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

MTC::vector<size_t> Polyhedron::getSharpVertIndices() const
{
	MTC::vector<size_t> result;
#if 0
	auto bbox = getBoundingBox();
	auto pMesh = getBlockPtr()->getModelMesh();
	const auto& allVerts = getBlockPtr()->getVolume()->getSharpVertIndices();
	for (size_t vertIdx : allVerts) {
		const auto& pt = pMesh->getVert(vertIdx)._pt;
		if (bbox.contains(pt, Tolerance::sameDistTol())) {
			result.push_back(vertIdx);
		}
	}
#endif
	return result;
}

bool Polyhedron::getSharpEdgeIndices(MTC::vector<size_t>& result, const BuildCFDParams& params) const
{
	const double sinEdgeAngle = sin(params.getSharpAngleRadians());

	result.clear();

#if 0
	auto pMesh = getBlockPtr()->getModelMesh();
	for (size_t edgeIdx : _edgeIndices) {
		if (pMesh->isEdgeSharp(edgeIdx, sinEdgeAngle)) {
			result.push_back(edgeIdx);
		}
	}
#endif
	return !result.empty();
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

void Polyhedron::remapId(const std::map<Index3D, Index3D>& idRemap)
{
	ObjectPoolOwnerUser::remapId(idRemap);

	remap(idRemap, _faceIds);
}

void Polyhedron::addFace(const Index3DId& faceId, size_t splitLevel)
{
	_faceIds.insert(faceId);
	faceFunc(TS_REAL,faceId, [this, splitLevel](Polygon& face) {
		face.addCellId(_thisId, splitLevel);
	});

	clearCache();
}

void Polyhedron::getVertIds(MTC::set<Index3DId>& vertIds) const
{
	for (const auto& faceId : _faceIds) {
		faceAvailFunc(getState(), faceId, [&vertIds](const Polygon& refFace) {
			const auto vertexIds = refFace.getVertexIds();
			vertIds.insert(vertexIds.begin(), vertexIds.end());
		});
	}
}

const MTC::set<Edge>& Polyhedron::getEdges(bool includeAdjacentCellFaces) const
{
	bool cacheValid = includeAdjacentCellFaces ? _cachedEdges1Vaild : _cachedEdges0Vaild;
	if (!cacheValid) {
		auto curState = getState();
		MTC::map<Edge, MTC::set<Index3DId>> edgeToFaceMap;
		MTC::set<Index3DId> adjCellIds;
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
						MTC::set<Index3DId> data;
						iter = edgeToFaceMap.insert(MTC::make_pair(edge, data)).first;
					}
					iter->second.insert(faceId);
				}
			});
		}

		if (includeAdjacentCellFaces) {
			adjCellIds.erase(_thisId);
			for (const auto& adjCellId : adjCellIds) {
				MTC::set<Index3DId> faceIds;
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

MTC::set<Index3DId> Polyhedron::getAdjacentCells(bool includeCornerCells) const
{
	MTC::set<Index3DId> cellIds;

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
void Polyhedron::getVertEdges(const Index3DId& vertId, MTC::set<Edge>& result, bool includeAdjacentCells) const
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
MTC::set<Index3DId> Polyhedron::getVertFaces(const Index3DId& vertId) const
{
	MTC::set<Index3DId> result;

	MTC::set<Edge> vertEdges;
	getVertEdges(vertId, vertEdges, false);

	for (const Edge& edge : vertEdges) {
		edge.getFaceIds(result);
	}

	return result;
}

CBoundingBox3Dd Polyhedron::getBoundingBox() const
{
	CBoundingBox3Dd bbox;
	MTC::set<Index3DId> vertIds;
	getVertIds(vertIds);
	for (const auto& vertId : vertIds) {
		bbox.merge(getVertexPoint(vertId));
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

double Polyhedron::calVolume() const
{
	double vol = 0;
	for (const auto& faceId : _faceIds) {
		faceFunc(TS_REAL, faceId, [this, &vol](const Polygon& face) {
			const Vector3d zAxis(0, 0, 1);
			Vector3d norm = face.calOrientedUnitNormal(getId());
			double area;
			Vector3d ctr;
			face.calAreaAndCentroid(area, ctr);
			double h = ctr.dot(zAxis);
			double dp = norm.dot(zAxis);
			double v = dp * h * area;
			vol += v;
		});
	}
	return vol;
}

bool Polyhedron::isOriented() const
{
	const auto& edges = getEdges(false);
	for (const auto& edge : edges) {
		if (!edge.isOriented(getBlockPtr(), getId()))
			return false;
	}
	return true;
}

void Polyhedron::classifyEdges(MTC::set<Edge>& convexEdges, MTC::set<Edge>& concaveEdges) const
{
	const auto& edges = getEdges(false);
	for (const auto& edge : edges) {
		if (edge.isConvex(getBlockPtr(), getId()))
			convexEdges.insert(edge);
		else
			concaveEdges.insert(edge);
	}
}

bool Polyhedron::isConvex() const
{
	if (!isClosed()) {
		assert(!"open cell cannot be tested for convexity");
		return false;
	}

	const auto& edges = getEdges(false);
	for (const auto& edge : edges) {
		if (!edge.isConvex(getBlockPtr(), getId())) {
			return false;
		}
	}

	return true;
}

bool Polyhedron::intersectsModel() const
{
#if 0
	if (_intersectsModel == IS_UNKNOWN) {
		CBoundingBox3Dd bbox = getBoundingBox();
		auto pTriMesh = getBlockPtr()->getModelMesh();
		auto sharps = getBlockPtr()->getVolume()->getSharpVertIndices();
		for (size_t idx : sharps) {
			if (bbox.contains(pTriMesh->getVert(idx)._pt, Tolerance::sameDistTol()))
				return true;
		}

		_intersectsModel = IS_FALSE;
		vector<size_t> triEntries;
		if (pTriMesh->processFoundTris(_triIndices, bbox, triEntries) > 0) {
			for (const auto& faceId : _faceIds) {
				faceFunc(TS_REAL, faceId, [this, &triEntries, &pTriMesh](const Polygon& face) {
					const double tol = Tolerance::sameDistTol();
					const auto& vertIds = face.getVertexIds();
					for (size_t i = 0; i < vertIds.size(); i++) {
						size_t j = (i + 1) % vertIds.size();
						LineSegmentd seg(getVertexPoint(vertIds[i]), getVertexPoint(vertIds[j]));
						for (size_t triIdx : triEntries) {
							RayHitd hit;
							if (pTriMesh->intersectsTri(seg, triIdx, tol, hit)) {
								_intersectsModel = IS_TRUE;
								break;
							}
						}
						if (_intersectsModel = IS_TRUE)
							break;
					}
				});

				if (_intersectsModel = IS_TRUE)
					break;
			}
		}
	}
#endif
	return _intersectsModel == IS_TRUE; // Don't test split cells
}

bool Polyhedron::intersectsModelPrecise() const
{
	assert(isConvex());
#if 0
	auto pMesh = getBlockPtr()->getModelMesh();
	auto bbox = getBoundingBox();
	vector<size_t> triEntries;
	if (pMesh->processFoundTris(_triIndices, bbox, triEntries) > 0) {
		for (const auto& faceId : _faceIds) {
			bool result = false;
			faceFunc(TS_REAL, faceId, [this, &triEntries, &pMesh, &result](const Polygon& face) {
				const double tol = Tolerance::sameDistTol();
				const auto& vertIds = face.getVertexIds();
				for (size_t i = 0; i < vertIds.size(); i++) {
					size_t j = (i + 1) % vertIds.size();
					const auto& vertId0 = vertIds[i];
					const auto& vertId1 = vertIds[j];

					Vector3d pt0 = getVertexPoint(vertId0);
					Vector3d pt1 = getVertexPoint(vertId1);
					LineSegmentd seg(pt0, pt1);
					for (size_t triIdx : triEntries) {
						RayHitd hit;
						if (pMesh->intersectsTri(seg, triIdx, tol, hit)) {
							if (containsPointPrecise(pt0) || containsPointPrecise(pt1)) {
								result = true;
								break;
							}
						}
					}
					if (result)
						break;
				}
			});

			if (result)
				return true;
		}
	}
#endif
	return false;
}

bool Polyhedron::containsPointPrecise(const Vector3d& pt) const
{
	if (!isConvex()) {
		assert(!"Cell is not convex.");
		return false;
	}
	for (const auto& faceId : _faceIds) {
		bool result = true;
		faceFunc(TS_REAL, faceId, [this, &pt, &result](const Polygon& face) {
			face.iterateTriangles([this, &pt, &result](const Index3DId& vertId0, const Index3DId& vertId1, const Index3DId& vertId2)->bool {
				auto pt0 = getVertexPoint(vertId0);
				auto pt1 = getVertexPoint(vertId1);
				auto pt2 = getVertexPoint(vertId2);
				Vector3d v0 = pt0 - pt1;
				Vector3d v1 = pt2 - pt1;
				Vector3d n = v1.cross(v0).normalized();
				Planed pl(pt0, n);
				double d = pl.distanceToPoint(pt);
				result = d < -Tolerance::sameDistTol();
				return result;
			});
		});
		if (!result)
			return false;
	}

	return true;
}

bool Polyhedron::sharpEdgesIntersectModel(const BuildCFDParams& params) const
{
	if (_sharpEdgesIntersectModel == IS_FALSE || _edgeIndices.empty())
		return false;
#if 0
	const double sinSharpEdgeAngle = sin(params.getSharpAngleRadians());
	MTC::vector<size_t> sharpEdges;
	auto pMesh = getBlockPtr()->getModelMesh();
	for (size_t edgeIdx : _edgeIndices) {
		if (pMesh->isEdgeSharp(edgeIdx, sinSharpEdgeAngle))
			sharpEdges.push_back(edgeIdx);
	}

	if (sharpEdges.empty())
		return false;

	_sharpEdgesIntersectModel = IS_FALSE;
	for (const auto& faceId : getFaceIds()) {
		faceFunc(TS_REAL, faceId, [this, &sharpEdges, &pMesh](const Polygon& face) {
			size_t numHits = 0;
			for (size_t edgeIdx : sharpEdges) {
				const auto& edge = pMesh->getEdge(edgeIdx);
				auto seg = edge.getSeg(pMesh);
				RayHitd hit;
				if (face.intersect(seg, hit)) {
					numHits += 1;
				}
			}
			if (numHits > 1)
				_sharpEdgesIntersectModel = IS_TRUE;
		});

		if (_sharpEdgesIntersectModel == IS_TRUE)
			return true;
	}
#endif
	return _sharpEdgesIntersectModel == IS_TRUE;
}

Vector3d Polyhedron::getVertexPoint(const Index3DId& vertId) const
{
	return getOurBlockPtr()->getVertexPoint(vertId);
}

void Polyhedron::setNeedsDivideAtCentroid()
{
	_needsSplitAtCentroid = true;
	addToSplitStack();
}

bool Polyhedron::needsDivideAtCentroid() const
{
	return _needsSplitAtCentroid;
}

bool Polyhedron::setNeedsCleanFaces()
{
	if (getBlockPtr()->polyhedronExists(TS_REFERENCE, _thisId)) {
		setNeedsDivideAtCentroid();
		return true;
	}

	return false;
}

bool Polyhedron::containsSharps() const
{
	auto vertIndices = getBlockPtr()->getVolume()->getSharpVertIndices();
#if 0
	auto bbox = getBoundingBox();
	auto pMesh = getBlockPtr()->getModelMesh();
	for (size_t vertIdx : vertIndices) {
		const auto& pt = pMesh->getVert(vertIdx)._pt;
		if (bbox.contains(pt, Tolerance::sameDistTol()))
			return true;
	}

	vector<size_t> triIndices;
	if (pMesh->processFoundTris(_triIndices, bbox, triIndices) > 0) {
	}
#endif
	return false;
}

void Polyhedron::orientFaces()
{
	if (_isOriented)
		return;

	for (const auto& faceId : _faceIds) {
		faceFunc(TS_REAL, faceId, [this](Polygon& face) {
			face.setReversed(getId(), false);
		});
	}

	MTC::set<Index3DId> orientedIds, unorientedIds(_faceIds);
	orientedIds.insert(*unorientedIds.begin());
	unorientedIds.erase(*orientedIds.begin());
	while (!unorientedIds.empty()) {
		bool found = false;
		for (const auto& oriented : orientedIds) {
			faceFunc(TS_REAL, oriented, [&](const Polygon& orientedFace) {
				orientedFace.iterateOrientedEdges([&](const Edge& edgeA) {
					for (const auto& unoriented : unorientedIds) {
						faceFunc(TS_REAL, unoriented, [&](Polygon& unorientedFace) {
							unorientedFace.iterateOrientedEdges([&](const Edge& edgeB) {
								if (edgeA == edgeB) {
									if (edgeA.getVertex(0) == edgeB.getVertex(0)) { // face is reversed
										unorientedFace.flipReversed(getId());
									}
									orientedIds.insert(unoriented);
									unorientedIds.erase(unoriented);
									found = true;
								}
								return !found;
							}, getId());
						});

						if (found)
							break;
					}
					return !found;
				}, getId());
			});

			if (found)
				break;
		}
	}


	double vol = calVolume();
	if (vol < 0) {
		// flip all face orientation flags
		for (auto& faceId : _faceIds) {
			faceFunc(TS_REAL, faceId, [this](Polygon& face) {
				face.flipReversed(getId());
			});
		}
		vol = calVolume();
		assert(vol > 0);
	}

	_isOriented = true;
}

void Polyhedron::imprintTVertices(Block* pDstBlock)
{
#if DEBUG_BREAKS && defined(_DEBUG)
	if (Index3DId(3, 0, 5, 7) == _thisId) {
		int dbgBreak = 1;
	}
#endif

#if 1
	MTC::set<Index3DId> verts;
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
		MTC::set<Index3DId> imprintVertices;
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

void Polyhedron::attachFaces()
{
	for (const auto& faceId : _faceIds) {
		if (getBlockPtr()->polygonExists(TS_REAL, faceId)) {
			faceFunc(TS_REAL, faceId, [this](Polygon& face) {
				face.addCellId(getId(), 0);
			});
		}
	}
}

void Polyhedron::detachFaces()
{
	for (const auto& faceId : _faceIds) {
		if (getBlockPtr()->polygonExists(TS_REAL, faceId)) {
			faceFunc(TS_REAL, faceId, [this](Polygon& face) {
				face.removeCellId(getId());
			});
		}
	}
}

void Polyhedron::replaceFaces(const Index3DId& curFaceId, const MTC::set<Index3DId>& newFaceIds, size_t splitLevel)
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
	getOurBlockPtr()->addToSplitStack0(_thisId);
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

bool Polyhedron::canSplit(MTC::set<Index3DId>& blockingCellIds) const
{
	// You can't cache this because it depends on the state of the neighbors
	// If this cell cannot be split due to a neighbor, then the neighbor is split - now, this cell can be split; even though this cell didn't change.
	blockingCellIds.clear();
	for (const auto& faceId : _faceIds) {
		Index3DId adjCellId;
		size_t faceSplitLevel;
		faceFunc(TS_REAL, faceId, [this, &adjCellId, &faceSplitLevel](const Polygon& face) {
			for (const auto& id : face.getCellIds()) {
				if (id != _thisId) {
					adjCellId = id;
					faceSplitLevel = face.getSplitLevel(adjCellId);
				}
			}
		});

		if (adjCellId.isValid()) {
			if (faceSplitLevel > 0) {
				blockingCellIds.insert(adjCellId);
			} else {
				cellFunc(TS_REAL, adjCellId, [this, &blockingCellIds](const Polyhedron& adjCell) {
					if (adjCell.getSplitLevel() < _splitLevel) {
						blockingCellIds.insert(adjCell.getId());
					}
				});
			}
		}
	}

	return blockingCellIds.empty();
}

bool Polyhedron::setNeedToSplitConditional(size_t passNum, const BuildCFDParams& params)
{
	if (!_needsConditionalSplitTest)
		return false;

	Utils::Timer tmr(Utils::Timer::TT_needToSplitConditional);

	_needsConditionalSplitTest = false;

	bool needToSplit = false;
	if (!needToSplit && passNum < params.numIntersectionDivs) {
		if (intersectsModel())
			needToSplit = true;
	}

	if (!needToSplit && passNum < params.numSharpEdgeIntersectionDivs) {
		if (sharpEdgesIntersectModel(params))
			needToSplit = true;
	}

	if (!needToSplit && passNum < params.numCurvatureDivs) {
		double maxEdgeLength = 0;

		CBoundingBox3Dd bbox = getBoundingBox();
		set<Edge> edges;
		cellAvailFunc(TS_REFERENCE, _thisId, [&edges](const Polyhedron& cell) {
			edges = cell.getEdges(false);
		});

		for (const auto& edge : edges) {
			auto seg = edge.getSegment(getBlockPtr());
			double l = seg.calLength();
			if (l > maxEdgeLength)
				maxEdgeLength = l;
		}

		double refRadius = calReferenceSurfaceRadius(bbox, params);
		if (refRadius > 0) {
			if (refRadius < 0.01) {
				int dbgBreak = 1;
			}
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
			if (2 * maxEdgeLength > maxAllowedEdgeLen)
				needToSplit = true;
		}
	}

	if (needToSplit) {
		setNeedsDivideAtCentroid();
	}

	return needToSplit;
}

bool Polyhedron::needToDivideDueToSplitFaces(const BuildCFDParams& params)
{
	if (_faceIds.size() > params.maxCellFaces) {
		setNeedsDivideAtCentroid();
		return true;
	}

	return false;
}

void Polyhedron::setEdgeIndices(const std::vector<size_t>& indices)
{
	_edgeIndices.clear();
	if (!indices.empty()) {
#if 0
		auto pTriMesh = getBlockPtr()->getModelMesh();
		auto bbox = getBoundingBox();
		pTriMesh->processFoundEdges(indices, bbox, _edgeIndices);
#endif
	}
}

void Polyhedron::setTriIndices(const std::vector<size_t>& indices)
{
	_triIndices.clear();
	if (!indices.empty()) {
#if 0
		auto pTriMesh = getBlockPtr()->getModelMesh();
		auto bbox = getBoundingBox();
		pTriMesh->processFoundTris(indices, bbox, _triIndices);
#endif
	}
}

bool Polyhedron::orderVertEdges(MTC::set<Edge>& edgesIn, MTC::vector<Edge>& orderedEdges) const
{
	orderedEdges.clear();
	MTC::set<Edge> edges(edgesIn);
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
#if 0
	_needsCurvatureCheck = false;

	if (_triIndices.empty())
		return 0;

	auto pTriMesh = getBlockPtr()->getModelMesh();

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
#endif
	return 0;
}

double Polyhedron::minGap() const
{
#if 0
	if (_cachedMinGap < 0) {
		_cachedMinGap = DBL_MAX;

		auto pTriMesh = getBlockPtr()->getModelMesh();
		auto bbox = getBoundingBox();
		vector<size_t> triIndices;
		if (pTriMesh->processFoundTris(_triIndices, bbox, triIndices)) {
			for (size_t idx : triIndices) {
				double gap = pTriMesh->triGap(idx);
				if (gap < _cachedMinGap)
					_cachedMinGap = gap;
			}
		}
	}
#endif
	return _cachedMinGap;
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

MTC::set<Edge> Polyhedron::createEdgesFromVerts(MTC::vector<Index3DId>& vertIds) const
{
	// This function takes vertices and looks for faces which contain exactly 2 of the available verts.
	// The edge may already exist.
	MTC::set<Edge> edgeSet;
	for (const auto& faceId : _faceIds) {
		faceFunc(TS_REAL,faceId, [this, &edgeSet, &vertIds](const Polygon& face) {
			MTC::set<Index3DId> vertsInFace;
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

bool Polyhedron::orderVertIds(MTC::vector<Index3DId>& vertIds) const
{

	MTC::vector<Index3DId> result;
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
	if (_cachedIsClosed == Trinary::IS_UNKNOWN) {
		_cachedIsClosed = Trinary::IS_TRUE;
		const auto& edges = getEdges(false);
		for (const auto& edge : edges) {
			if (edge.getFaceIds().size() != 2) {
				_cachedIsClosed = Trinary::IS_FALSE;
				break;
			}
		}
	}
	return _cachedIsClosed == Trinary::IS_TRUE;
}

bool Polyhedron::lineSegmentIntersects(const LineSegmentd& seg, MTC::vector<RayHitd>& hits, MTC::vector<Index3DId>& faceIds) const
{
	for (const auto& faceId : _faceIds) {
		faceFunc(TS_REAL, faceId, [this, &seg, &hits, &faceIds] (const Polygon& face) {
			RayHitd hit;
			if (face.intersect(seg, hit)) {
				hits.push_back(hit);
				faceIds.push_back(face.getId());
			}
		});
	}
	return !faceIds.empty() && hits.size() == faceIds.size();
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
		getBlockPtr()->dumpPolyhedraObj({ _thisId }, false, false, false);
	}
#endif

	return valid;
}

void Polyhedron::clearCache() const
{
	_needsCurvatureCheck = true;
	_cachedEdges0Vaild = false;
	_cachedEdges1Vaild = false;
	_isOriented = false;

	_intersectsModel = IS_UNKNOWN; // Cached value
	_cachedIsClosed = IS_UNKNOWN;

	_cachedMinGap = -1;

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
