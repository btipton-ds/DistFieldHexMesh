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
#include <tm_spatialSearch.hpp>
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
#include <splitter3D.h>
#include <tolerances.h>
#include <utils.h>
#include <meshData.h>
#include <splitter2D.h>

using namespace std;
using namespace DFHM;

Polyhedron::Polyhedron(const MultiCore::set<Index3DId>& faceIds, const MultiCore::vector<Index3DId>& cornerVertIds)
{
	for (const auto& id : faceIds)
		_faceIds.insert(id);
	_canonicalVertices = cornerVertIds;
}

Polyhedron::Polyhedron(const std::set<Index3DId>& faceIds, const std::vector<Index3DId>& cornerVertIds)
{
	for (const auto& id : faceIds)
		_faceIds.insert(id);
	_canonicalVertices.clear();
	_canonicalVertices.insert(_canonicalVertices.end(), cornerVertIds.begin(), cornerVertIds.end());
}

Polyhedron::Polyhedron(const MultiCore::vector<Index3DId>& faceIds, const MultiCore::vector<Index3DId>& cornerVertIds)
{
	_faceIds = faceIds;
	_canonicalVertices = cornerVertIds;
}


Polyhedron::Polyhedron(const std::vector<Index3DId>& faceIds, const std::vector<Index3DId>& cornerVertIds)
{
	_faceIds = faceIds;
	_canonicalVertices.clear();
	_canonicalVertices.insert(_canonicalVertices.end(), cornerVertIds.begin(), cornerVertIds.end());
}

Polyhedron::Polyhedron(const Polyhedron& src)
	: ObjectPoolOwnerUser(src)
	, _faceIds(src._faceIds)
	, _canonicalVertices(src._canonicalVertices)
	, _needsSplitAtCentroid(src._needsSplitAtCentroid)
	, _cachedIsClosed(src._cachedIsClosed)
	, _layerNum(src._layerNum)
	, _triIndices(src._triIndices)
{
}

const Index3DId& Polyhedron::getId() const
{
	return _thisId;
}

void Polyhedron::setId(const Index3DId& id)
{
	_thisId = id;
}

void Polyhedron::clear()
{
	ObjectPoolOwnerUser::clear();

	MTC::set<Index3DId> deadFaceIds;
	for (const auto& faceId : _faceIds) {
		faceFunc(faceId, [this, &deadFaceIds](Polygon& face) {
			face.removeCellId(getId());
			if (face.numCells() == 0) {
				deadFaceIds.insert(face.getId());
			}
		});
	}

	_faceIds.clear();

	for (const auto& faceId : deadFaceIds) {
		cout << "Freeing face: " << faceId << "\n";
		getBlockPtr()->freePolygon(faceId);
	}
}

Polyhedron& Polyhedron::operator = (const Polyhedron& rhs)
{
	ObjectPoolOwnerUser::operator=(rhs);
	clearCache();
	auto tmp = _faceIds;
	for (const auto& faceId : tmp) {
		if (getBlockPtr()->polygonExists(faceId)) {
			bool isDetached = false;
			faceFunc(faceId, [this, &isDetached](Polygon& face) {
				face.removeCellId(getId());
				isDetached = face.getCellIds().empty();
			});
			if (isDetached)
				getBlockPtr()->freePolygon(faceId);
		}
	}

	_thisId = rhs._thisId;
	_faceIds = rhs._faceIds;
	_canonicalVertices = rhs._canonicalVertices;
	_splitLevel = rhs._splitLevel;
	_layerNum = rhs._layerNum;
	_needsSplitAtCentroid = rhs._needsSplitAtCentroid;
	_exists = rhs._exists;
	_triIndices = rhs._triIndices;

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

bool Polyhedron::getSharpEdgeIndices(MTC::vector<size_t>& result, const SplittingParams& params) const
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

	IoUtil::writeObj(out, _faceIds);
	IoUtil::writeObj(out, _canonicalVertices);

	out.write((char*)&_splitLevel, sizeof(size_t));

	out.write((char*)&_splitLevel, sizeof(_splitLevel));
	out.write((char*)&_layerNum, sizeof(_layerNum));
}

void Polyhedron::read(std::istream& in)
{
	uint8_t version;
	in.read((char*)&version, sizeof(version));

	IoUtil::readObj(in, _faceIds);
	IoUtil::readObj(in, _canonicalVertices);
	in.read((char*)&_splitLevel, sizeof(size_t));

	in.read((char*)&_splitLevel, sizeof(_splitLevel));
	in.read((char*)&_layerNum, sizeof(_layerNum));
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

void Polyhedron::remapId(const std::vector<size_t>& idRemap, const Index3D& srcDims)
{
	remap(idRemap, srcDims, _thisId);
	remap(idRemap, srcDims, _faceIds);
	remap(idRemap, srcDims, _canonicalVertices);
}

void Polyhedron::addFace(const Index3DId& faceId)
{
	_faceIds.insert(faceId);
	faceFunc(faceId, [this](Polygon& face) {
		face.addCellId(getId());
	});

	clearCache();
}

void Polyhedron::removeFace(const Index3DId& faceId)
{
	_faceIds.erase(faceId);
	faceFunc(faceId, [this](Polygon& face) {
		face.removeCellId(getId());
	});

	clearCache();
}

size_t Polyhedron::getVertIds(MTC::set<Index3DId>& result) const
{
	for (const auto& faceId : _faceIds) {
		faceFunc(faceId, [this, &result](const Polygon& face) {
			const auto& v = face.getVertexIds();
			result.insert(v.begin(), v.end());
		});
	}

	return result.size();
}

size_t Polyhedron::getNumFaces() const
{
	return _faceIds.size();
}


FastBisectionSet<EdgeKey> Polyhedron::getEdgeKeys(bool includeAdjacentCellFaces) const
{
	FastBisectionSet<EdgeKey> result;
	if (includeAdjacentCellFaces) {
		MTC::set<Index3DId> vertIds;
		getVertIds(vertIds);
		for (const auto& vertId : vertIds) {
			vertexFunc(vertId, [&result](const Vertex& vert) {
				auto edges = vert.getEdges();
				result.insert(edges.begin(), edges.end());
			});
		}
	} else {
		for (const auto& faceId : _faceIds) {
			faceFunc(faceId, [&result](const Polygon& face) {
				auto edgeKeys = face.getEdgeKeys();
				result.insert(edgeKeys.begin(), edgeKeys.end());
			});
		}
	}

	return result;
}

void Polyhedron::updateAllTopolCaches() const
{
#if 0
	updateCachedVerts();
	if (!_cachedAdjCellIds.empty() && !_cachedEdges.empty() && !_cachedEdgesAdj.empty())
		return;

	map<Edge, FastBisectionSet<Index3DId>> edgeToFaceIdMap;
	FastBisectionSet<Index3DId> testedFaceIds;
	FastBisectionSet<EdgeKey> connectedEdges0;
	for (const auto& faceId : _faceIds) {
		testedFaceIds.insert(faceId);
		faceFunc(faceId, [this, &testedFaceIds, &edgeToFaceIdMap](const Polygon& face) {
			const auto& adjCellIds = face.getCellIds();
			for (const auto& cellId : adjCellIds) {
				if (cellId != getId())
					_cachedAdjCellIds.insert(cellId);
			}

			const auto& edges = face.getEdges();
			for (const auto& edge : edges) {
				auto iter = edgeToFaceIdMap.find(edge);
				if (iter == edgeToFaceIdMap.end()) {
					iter = edgeToFaceIdMap.insert(make_pair(edge, FastBisectionSet<Index3DId>())).first;
				}
				iter->second.insert(face.getId());
			}
		});
	}

	FastBisectionSet <Index3DId> addedCellIds;
	do {
		addedCellIds.clear();
		for (const auto& cellId : _cachedAdjCellIds) {
			FastBisectionSet<Index3DId> faceIds;
			cellFunc(cellId, [&faceIds](const Polyhedron& cell) {
				faceIds = cell.getFaceIds();
			});

			for (const auto& faceId : faceIds) {
				if (testedFaceIds.contains(faceId))
					continue;

				testedFaceIds.insert(faceId);

				const MTC::vector<Index3DId>* pVertIds;
				FastBisectionSet<Index3DId> adjFaceCellIds;

				faceFunc(faceId, [this, &pVertIds, &adjFaceCellIds, &edgeToFaceIdMap](const Polygon& face) {
					pVertIds = &face.getVertexIds();
					const auto& cellIds = &face.getCellIds();
					const auto& edges = face.getEdges();
					for (const auto& cellId : cellIds->asVector()) {
						if (cellId != getId() && !_cachedAdjCellIds.contains(cellId)) {
							adjFaceCellIds.insert(cellId);
							for (const auto& edge : edges) {
								auto iter = edgeToFaceIdMap.find(edge);
								if (iter != edgeToFaceIdMap.end()) {
									iter->second.insert(face.getId());
								}
							}
						}
					}
				});

				if (!adjFaceCellIds.empty()) {
					bool faceConntectToSeedCell = false;
					for (const auto& vertId : *pVertIds) {
						for (const auto& id : _cachedVertIds) {
							if (id == vertId) {
								faceConntectToSeedCell = true;
								break;
							}
						}
						if (faceConntectToSeedCell)
							break;
					}
					if (faceConntectToSeedCell) {
						const auto& v = adjFaceCellIds;
						addedCellIds.insert(v.begin(), v.end());
					}
				}
			}
		}
		const auto& addedVec = addedCellIds;
		_cachedAdjCellIds.insert(addedVec.begin(), addedVec.end());
	} while (!addedCellIds.empty());

	for (const auto& pair : edgeToFaceIdMap) {
		const auto& edge = pair.first;
		const auto& faceIds = pair.second;

		set<Index3DId> localFaceIds;
		for (const auto& faceId : faceIds) {
			if (_faceIds.contains(faceId))
				localFaceIds.insert(faceId);
		}

		_cachedEdges.insert(Edge(edge, localFaceIds));
		_cachedEdgesAdj.insert(Edge(edge, faceIds));
	}

#if (0 || VALIDATION_ON) && defined(_DEBUG)
	static size_t maxCells = 0;
	static size_t minCells = 1000;
	if (_cachedAdjCellIds.size() > maxCells || _cachedAdjCellIds.size() < minCells) {
		if (_cachedAdjCellIds.size() > maxCells)
			maxCells = _cachedAdjCellIds.size();
		if (_cachedAdjCellIds.size() < minCells)
			minCells = _cachedAdjCellIds.size();
		cout << "Min Cells: " << minCells << ", Max Cells: " << maxCells << "\n";
	}

	for (const auto& id : _cachedAdjCellIds) {
		FastBisectionSet<Index3DId> faceIds;
		cellFunc(id, [&faceIds](const Polyhedron& cell) {
			faceIds = cell.getFaceIds();
		});

		bool found = false;
		for (const auto& faceId : faceIds) {
			faceFunc(faceId, [this, &found](const Polygon& face) {
				const auto& vertIds = face.getVertexIds();
				for (const auto& vertId : vertIds) {
					if (_cachedVertIds.contains(vertId)) {
						found = true;
						break;
					}
					if (found)
						break;
				}
			});
			if (found)
				break;
		}
		assert(found);
	}

	assert(_cachedEdges.size() == 12);
	for (const auto& edge : _cachedEdges) {
		assert(edge.getFaceIds().size() == 2);
	}

	assert(_cachedEdgesAdj.size() == 12);
	for (const auto& edge : _cachedEdgesAdj) {
		assert(edge.getFaceIds().size() >= 2);
	}

#endif
#endif
}

void Polyhedron::updateCachedVerts() const
{
#if 0
	if (_cachedVertIds.empty()) {
		if (_faceIds.size() == 6) {
			faceFunc(_faceIds[0], [this](const Polygon& face) {
				const auto& vertIds = face.getVertexIds();
				_cachedVertIds.push_back(vertIds[0]);
				_cachedVertIds.push_back(vertIds[3]);
				_cachedVertIds.push_back(vertIds[2]);
				_cachedVertIds.push_back(vertIds[1]);
			});
			faceFunc(_faceIds[1], [this](const Polygon& face) {
				const auto& vertIds = face.getVertexIds();
				_cachedVertIds.push_back(vertIds[0]);
				_cachedVertIds.push_back(vertIds[1]);
				_cachedVertIds.push_back(vertIds[2]);
				_cachedVertIds.push_back(vertIds[3]);
			});
		}
	}
#endif
}


FastBisectionSet<Index3DId> Polyhedron::getAdjacentCells() const
{
	FastBisectionSet<Index3DId> result;

	MTC::set<Index3DId> vertIds;
	getVertIds(vertIds);
	for (const auto& vertId : vertIds) {
		vertexFunc(vertId, [&result](const Vertex& vertex) {
			auto cellIds = vertex.getCellIds();
			result.insert(cellIds.begin(), cellIds.end());
		});
	}
	result.erase(getId());
	return result;
}

// Gets the edges for a vertex which belong to this polyhedron
void Polyhedron::getVertEdges(const Index3DId& vertId, FastBisectionSet<EdgeKey>& result, bool includeAdjacentCells) const
{
	auto cellEdgeSet = getEdgeKeys(includeAdjacentCells);
	if (includeAdjacentCells) {
		auto adjCells = getAdjacentCells();
		for (const auto& adjCellId : adjCells) {
			cellFunc(adjCellId, [&cellEdgeSet](const Polyhedron& adjCell) {
				const auto& tmp = adjCell.getEdgeKeys(true);
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
FastBisectionSet<Index3DId> Polyhedron::getVertFaces(const Index3DId& vertId) const
{
	FastBisectionSet<Index3DId> result;

	FastBisectionSet<EdgeKey> vertEdgeKeys;
	getVertEdges(vertId, vertEdgeKeys, false);

	for (const auto& edgeKey : vertEdgeKeys) {
		edgeFunc(edgeKey, [&result](const Edge& edge) {
			auto& tmp = edge.getFaceIds();
			result.insert(tmp.begin(), tmp.end());
		});
	}

	return result;
}

bool Polyhedron::exists() const
{
	return _exists;
}

size_t Polyhedron::classify(MTC::vector<Vector3d>& corners) const
{
	corners.clear();

	Vector3d cellCtr = calCentroidApproxFast();

	int numQuads = 0, numTris = 0;
	MTC::vector<Index3DId> baseFaceVerts, oppFaceVerts;
	for (const auto& faceId : _faceIds) {
		faceFunc(faceId, [this, &numQuads, &numTris, &baseFaceVerts](const Polygon& face) {
			const auto& vertIds = face.getVertexIds();
			if (vertIds.size() == 3)
				numTris++;
			else if (vertIds.size() == 4)
				numQuads++;

			if (baseFaceVerts.empty())
				baseFaceVerts = vertIds;
			});
	}

	if (numQuads == 6 && numTris == 0) {
		// May be hexahedral
		auto edges = getEdgeKeys(false);
		for (size_t i = 0; i < baseFaceVerts.size(); i++) {
			size_t j = (i + 1) % baseFaceVerts.size();
			edges.erase(EdgeKey(baseFaceVerts[i], baseFaceVerts[j]));
		}
		for (size_t i = 0; i < baseFaceVerts.size(); i++) {
			for (const auto& e : edges) {
				if (e.containsVertex(baseFaceVerts[i])) {
					oppFaceVerts.push_back(e.getOtherVert(baseFaceVerts[i]));
				}
			}
		}
		if (baseFaceVerts.size() != 4 || oppFaceVerts.size() != 4)
			return -1;

		vector<Vector3d> pts, oppPts;
		Vector3d faceCtr0, faceCtr1;
		for (int i = 0; i < 4; i++) {
			const Vector3d& pt0 = getVertexPoint(baseFaceVerts[i]);
			pts.push_back(pt0);
			faceCtr0 += pt0;

			const Vector3d& pt1 = getVertexPoint(oppFaceVerts[i]);
			oppPts.push_back(pt1);
			faceCtr1 += pt1;
		}
		faceCtr0 /= 4;
		faceCtr1 /= 4;

		Vector3d v, v0, v1, norm;
		v0 = pts[1] - pts[0];
		v1 = pts[3] - pts[0];
		norm = v1.cross(v0).normalized();
		v = (cellCtr - faceCtr0).normalized();
		if (v.dot(norm) < 0) {
			std::swap(pts[1], pts[3]);
			std::swap(oppPts[1], oppPts[3]);
		}

		v0 = oppPts[1] - oppPts[0];
		v1 = oppPts[3] - oppPts[0];
		norm = v1.cross(v0).normalized();
		v = (cellCtr - faceCtr1).normalized();
		if (v.dot(norm) < 0) {
			corners.insert(corners.end(), pts.begin(), pts.end());
			corners.insert(corners.end(), oppPts.begin(), oppPts.end());
		}
	}
	else if (numQuads == 1 && numTris == 4) {
		// May be pyramid
	}
	else if (numQuads == 3 && numTris == 2) {
		// May be tri prism
	}
	else if (numQuads == 3 && numTris == 2) {
		// May be tri prism
	}
	else if (numTris == 4 && numQuads == 0) {
		// Must be tetrahedron
	}
	return corners.size();
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
	const double tol = Tolerance::sameDistTol();
	bool result = true;

	auto cellCtr = calCentroidApproxFast(); // Just need a point inside
	for (const auto& faceId : _faceIds) {
		faceFunc(faceId, [&pt, &cellCtr, &result](const Polygon& face) {
			result = face.isPointInside(pt, cellCtr);
		});
		if (!result)
			break;
	}

	return result;
}

bool Polyhedron::containsVertex(const Index3DId& vertId) const
{
	MTC::set<Index3DId> vertIds;
	getVertIds(vertIds);

	return vertIds.contains(vertId);
}

bool Polyhedron::isVertexConnectedToCell(const Index3DId& cellId) const
{
	bool result = false;
	cellFunc(cellId, [this, &result](const Polyhedron& otherCell) {
		MTC::set<Index3DId> vertIds;
		getVertIds(vertIds);
		for (const auto& id : vertIds) {
			if (otherCell.containsVertex(id)) {
				result = true;
				break;
			}
		}
	});

	return result;
}

Vector3d Polyhedron::calCentroid() const
{
	// Use the  curl algorithm over all triangles
	Vector3d ctr(0, 0, 0);
	double vol = 0;
	for (const auto& faceId : _faceIds) {
		faceFunc(faceId, [this, &vol, &ctr](const Polygon& face) {
			face.iterateTriangles([this, &vol, &ctr](const Vector3d& pt0, const Index3DId& idx1, const Index3DId& idx2)->bool {
				const Vector3d axis(1, 0, 0);
				auto pBlk = getBlockPtr();
				const auto& a = pt0;
				const auto& b = pBlk->getVertexPoint(idx1);
				const auto& c = pBlk->getVertexPoint(idx2);

				Vector3d triCtr = (a + b + c) / 3.0;
				Vector3d norm = (b - a).cross(c - a);
				double triArea = norm.norm() / 2;
				Vector3d unitNorm = norm.normalized();

				double dp = axis.dot(unitNorm);
				double projArea = triArea * dp;
				vol += projArea * triCtr.dot(axis);
				for (int i = 0; i < 3; i++) {
					double ab = a[i] + b[i];
					double bc = b[i] + c[i];
					double ca = c[i] + a[i];

					ctr[i] += norm[i] * (ab * ab + bc * bc + ca * ca);
				}
				return true;
			});
		});
	}
	ctr *= 1 / (vol * 48.0);
	return ctr;
}

Vector3d Polyhedron::calCentroidApproxFast() const
{
	MTC::set<Index3DId> vertIds;
	getVertIds(vertIds);
	Vector3d ctr(0, 0, 0);
	for (const auto& id : vertIds) {
		ctr += getBlockPtr()->getVertexPoint(id);
	}

	ctr /= vertIds.size();

	return ctr;
}

double Polyhedron::calVolume() const
{
	double vol = 0;
	for (const auto& faceId : _faceIds) {
		faceFunc(faceId, [this, &vol](const Polygon& face) {
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
	bool result = true;
	auto edgeKeys = getEdgeKeys(false);
	for (const auto& edgeKey : edgeKeys) {
		edgeFunc(edgeKey, [this, &result](const Edge& edge) {
			if (!edge.isOriented(getId()))
				result = false;
		});
	}

	return result;
}

void Polyhedron::classifyEdges(MTC::set<EdgeKey>& convexEdges, MTC::set<EdgeKey>& concaveEdges) const
{
	auto edgeKeys = getEdgeKeys(false);
	for (const auto& edgeKey : edgeKeys) {
		edgeFunc(edgeKey, [this, &convexEdges, &concaveEdges](const Edge& edge) {
			if (edge.isConvex(getId()))
				convexEdges.insert(edge);
			else
				concaveEdges.insert(edge);
		});
	}
}

bool Polyhedron::isConvex() const
{
	if (!isClosed()) {
		assert(!"open cell cannot be tested for convexity");
		return false;
	}

	bool result = true;
	const auto& edgeKeys = getEdgeKeys(false);
	for (const auto& edgeKey : edgeKeys) {
		edgeFunc(edgeKey, [this, &result](const Edge& edge) {
			if (!edge.isConvex(getId())) {
				result = false;
			}
		});

		if (!result)
			break;
	}
	return result;
}

bool Polyhedron::intersectsModel() const
{
	if (_intersectsModel == IS_UNKNOWN) {
		if (!_triIndices.empty()) {
			for (const auto& faceId : _faceIds) {
				faceFunc(faceId, [this](const Polygon& face) {
					if (face.intersectsModel(_triIndices)) {
						_intersectsModel = IS_TRUE;
					}
				});

				if (_intersectsModel == IS_TRUE)
					return true;
			}
		}
		_intersectsModel = IS_FALSE;
	}

	return _intersectsModel == IS_TRUE; // Don't test split cells
}

bool Polyhedron::sharpEdgesIntersectModel(const SplittingParams& params) const
{
	if (_sharpEdgesIntersectModel == IS_FALSE)
		return false;

	CBoundingBox3Dd bbox = getBoundingBox();
	const double sinSharpEdgeAngle = sin(params.getSharpAngleRadians());
	MTC::vector<size_t> sharpEdges;
	auto& meshData = getBlockPtr()->getModelMeshData();
	for (auto& pData : meshData) {
		auto& pMesh = pData->getMesh();
		vector<size_t> edgeIndices;
		if (pMesh->findEdges(bbox, edgeIndices)) {
			for (size_t edgeIdx : edgeIndices) {
				if (pMesh->isEdgeSharp(edgeIdx, sinSharpEdgeAngle))
					sharpEdges.push_back(edgeIdx);
			}
		}

		if (sharpEdges.empty())
			return false;

		_sharpEdgesIntersectModel = IS_FALSE;
		for (const auto& faceId : getFaceIds()) {
			faceFunc(faceId, [this, &sharpEdges, &pMesh](const Polygon& face) {
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
	}
	return _sharpEdgesIntersectModel == IS_TRUE;
}

#if USE_CELL_HISTOGRAM
void Polyhedron::addToFaceCountHistogram(std::map<size_t, size_t>& histo) const
{
	size_t numFaces = getNumFaces();

	switch (numFaces) {
		case 6:
		case 9:
		case 12:
			break;
		default: {
			cout << "Split history of bad cell with " << numFaces << " faces:\n";
			for (const auto& id : _parentIds) {
				cout << "  " << id << "\n";
			}
			cout << "  " << getId() << "\n";

			break;
		}
	}
	auto iter = histo.find(numFaces);
	if (iter == histo.end())
		iter = histo.insert(make_pair(numFaces, 0)).first;
	iter->second++;
}
#endif

bool Polyhedron::isTooComplex(const SplittingParams& params) const
{
	return _faceIds.size() > params.maxCellFaces;
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
	if (getBlockPtr()->polyhedronExists(_thisId)) {
		setNeedsDivideAtCentroid();
		return true;
	}

	return false;
}

bool Polyhedron::containsSharps() const
{
	auto vertIndices = getBlockPtr()->getVolume()->getSharpVertIndices();

	auto bbox = getBoundingBox();
	const auto& meshData = getBlockPtr()->getModelMeshData();
	for (const auto& pData : meshData) {
		auto pMesh = pData->getMesh();
		for (size_t vertIdx : vertIndices) {
			const auto& pt = pMesh->getVert(vertIdx)._pt;
			if (bbox.contains(pt, Tolerance::sameDistTol()))
				return true;
		}

		vector<size_t> triIndices;
		if (pMesh->findTris(bbox, triIndices) > 0) {
		}
	}
	return false;
}

void Polyhedron::orientFaces()
{
	if (_isOriented)
		return;

	for (const auto& faceId : _faceIds) {
		faceFunc(faceId, [this](Polygon& face) {
			face.setReversed(getId(), false);
		});
	}

	MTC::set<Index3DId> orientedIds, unorientedIds;
	unorientedIds.insert(_faceIds.begin(), _faceIds.end());
	orientedIds.insert(*unorientedIds.begin());
	unorientedIds.erase(*orientedIds.begin());
	while (!unorientedIds.empty()) {
		bool found = false;
		for (const auto& oriented : orientedIds) {
			faceFunc(oriented, [&](const Polygon& orientedFace) {
				orientedFace.iterateOrientedEdges([&](const Edge& edgeA) {
					for (const auto& unoriented : unorientedIds) {
						faceFunc(unoriented, [&](Polygon& unorientedFace) {
							unorientedFace.iterateOrientedEdges([&](const Edge& edgeB) {
								if (edgeA == edgeB) {
									if (edgeA[0] == edgeB[0]) { // face is reversed
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
			faceFunc(faceId, [this](Polygon& face) {
				face.flipReversed(getId());
			});
		}
		vol = calVolume();
		assert(vol > 0);
	}

	_isOriented = true;
}

void Polyhedron::attachFaces()
{
	for (const auto& faceId : _faceIds) {
		faceFunc(faceId, [this](Polygon& face) {
			face.addCellId(getId());
		});
	}
}

void Polyhedron::detachFaces()
{
	for (const auto& faceId : _faceIds) {
		faceFunc(faceId, [this](Polygon& face) {
			face.removeCellId(getId());
		});
	}
}

void Polyhedron::connectVertEdgeTopology()
{
	for (const auto& faceId : _faceIds) {
		faceFunc(faceId, [this](Polygon& face) {
			face.connectVertEdgeTopology();
		});
	}
}

void Polyhedron::disconnectVertEdgeTopology()
{
	for (const auto& faceId : _faceIds) {
		faceFunc(faceId, [this](Polygon& face) {
			face.disconnectVertEdgeTopology();
		});
	}
}

void Polyhedron::imprintFaceEdges(const Index3DId& newFaceId)
{
	faceFunc(newFaceId, [this](Polygon& newFace) {
		newFace.imprintFaces(_faceIds);
		});

	MTC::vector<EdgeKey> newEdgeKeys;
	MTC::vector<Index3DId> vertIds;
	faceFunc(newFaceId, [&newEdgeKeys, &vertIds](const Polygon& face) {
		newEdgeKeys = face.getEdgeKeys();
		vertIds = face.getVertexIds();
		});

	auto tmp = _faceIds; // Make a copy because this will change as we work

	for (const auto& faceId : tmp) {
		assert(getBlockPtr()->polygonExists(faceId));
		faceFunc(faceId, [this, &vertIds](Polygon& face) {
			auto edgeKeys = face.getEdgeKeys();
			for (const auto& ek : edgeKeys) {
				edgeFunc(ek, [vertIds](Edge& edge) {
					edge.imprintVertices(vertIds);
					});
			}
			});
	}

	assert(isClosed());

	for (const auto& faceId : tmp) {
		assert(getBlockPtr()->polygonExists(faceId));
		MTC::vector<MTC::vector<Vector3d>> splitFacePoints;
		FastBisectionSet<Index3DId> cellIds;
		faceFunc(faceId, [this, &newEdgeKeys, &splitFacePoints, &cellIds](Polygon& face) {
			Splitter2D sp(face.calPlane());

			cellIds = face.getCellIds();
			face.iterateEdges([this, &sp](const Edge& edge)->bool {
				const auto& pt0 = getVertexPoint(edge[0]);
				const auto& pt1 = getVertexPoint(edge[1]);
				sp.add3DEdge(pt0, pt1);
				return true;
			});

			for (const auto& ek : newEdgeKeys) {
				const auto& pt0 = getVertexPoint(ek[0]);
				const auto& pt1 = getVertexPoint(ek[1]);
				bool cp0 = face.isCoplanar(pt0);
				bool cp1 = face.isCoplanar(pt1);
				if (cp0 && cp1) {
					sp.add3DEdge(pt0, pt1);
				}
			}
			sp.getFacePoints(splitFacePoints);
		});

		if (splitFacePoints.size() < 2)
			continue;

		set<Index3DId> newFaceIds;
		for (const auto& facePts : splitFacePoints) {
			MTC::vector<Index3DId> vertIds;
			for (const auto& pt : facePts) {
				vertIds.push_back(getBlockPtr()->addVertex(pt));
			}
			auto newFaceId = getBlockPtr()->addPolygon(Polygon(vertIds));
			newFaceIds.insert(newFaceId);
		}

		if (!newFaceIds.empty()) {
			for (const auto& cellId : cellIds) {
				cellFunc(cellId, [this, &faceId, &newFaceIds](Polyhedron& cell) {
					cell.removeFace(faceId);
					for (const auto& newFaceId : newFaceIds) {
						cell.addFace(newFaceId);
					}

					assert(cell.isClosed());
					});
			}

			getBlockPtr()->freePolygon(faceId);
		}
	}
}

void Polyhedron::addToSplitStack()
{
	getOurBlockPtr()->addToSplitStack(_thisId);
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

bool Polyhedron::setNeedToSplitConditional(size_t passNum, const SplittingParams& params)
{
	Utils::Timer tmr(Utils::Timer::TT_needToSplitConditional);

	if (passNum < params.numIntersectionDivs && intersectsModel()) {
		setNeedsDivideAtCentroid();
		return true;
	}

	if (passNum < params.numSharpEdgeIntersectionDivs && sharpEdgesIntersectModel(params)) {
		setNeedsDivideAtCentroid();
		return true;		
	}

	if (passNum < params.numCurvatureDivs) {
		double maxEdgeLength = 0;

		CBoundingBox3Dd bbox = getBoundingBox();
		FastBisectionSet<EdgeKey> edgeKeys;
		cellFunc(_thisId, [&edgeKeys](const Polyhedron& cell) {
			edgeKeys = cell.getEdgeKeys(false);
		});

		for (const auto& edgeKey : edgeKeys) {
			edgeFunc(edgeKey, [&maxEdgeLength](const Edge& edge) {
				auto seg = edge.getSegment();
				double l = seg.calLength();
				if (l > maxEdgeLength)
					maxEdgeLength = l;
			});
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
			if (2 * maxEdgeLength > maxAllowedEdgeLen) {
				setNeedsDivideAtCentroid();
				return true;
			}
		}
	}

	return false;
}

double Polyhedron::calReferenceSurfaceRadius(const CBoundingBox3Dd& bbox, const SplittingParams& params) const
{
	if (!_needsCurvatureCheck)
		return 0;

	_needsCurvatureCheck = false;
	const auto& meshData = getBlockPtr()->getModelMeshData();

	if (meshData.empty())
		return 0;

	for (const auto& pData : meshData) {
		auto pTriMesh = pData->getMesh();

		const auto& blkIdx = _thisId.blockIdx();
		if (blkIdx[0] == 0 && blkIdx[1] == 0) {
			int dbgBreak = 1;
		}

		vector<size_t> triIndices;
		size_t numTris = pTriMesh->findTris(bbox, triIndices);
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
				}
				else
					break;
			}

			if (count > 0 && avgRad >= 0) {
				avgRad /= count;
				return avgRad;
			}
			return -1;
		}
	}
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

inline bool Polyhedron::polygonExists(const Index3DId& id) const
{
	return getBlockPtr()->polygonExists(id);
}

void Polyhedron::clearLayerNum()
{
	_layerNum = -1;
}

void Polyhedron::addMeshToTriIndices(const vector<MeshDataPtr>& meshData)
{
	auto bbox = getBoundingBox();
	for (size_t i = 0; i < meshData.size(); i++) {
		const auto& pMesh = meshData[i]->getMesh();
		std::vector<size_t> triIndices;
		if (pMesh->findTris(bbox, triIndices)) {
			for (const auto& triIdx : triIndices) {
				_triIndices.push_back(TriMeshIndex(i, triIdx));
			}
		}
	}
}

void Polyhedron::setTriIndices(const Polyhedron& srcCell)
{
	const auto tol = Tolerance::sameDistTol();
	auto bbox = getBoundingBox();
	const auto& meshData = getBlockPtr()->getModelMeshData();
	for (const auto& triMeshIdx : srcCell._triIndices) {
		const auto& pMesh = meshData[triMeshIdx.getMeshIdx()]->getMesh();
		const auto& triBbox = pMesh->getTriBBox(triMeshIdx.getTriIdx());
		if (bbox.intersectsOrContains(triBbox, tol)) {
			_triIndices.push_back(triMeshIdx);
		}
	}
}

void Polyhedron::setLayerNum(int32_t val, bool force)
{
	if (_layerNum == -2 || force)
		_layerNum = val;
}

void Polyhedron::setLayerNumOnNextPass(int32_t val)
{
	if (_layerNum == -1)
		_layerNum = -2;
}


MTC::set<EdgeKey> Polyhedron::createEdgesFromVerts(MTC::vector<Index3DId>& vertIds) const
{
	// This function takes vertices and looks for faces which contain exactly 2 of the available verts.
	// The edge may already exist.
	MTC::set<EdgeKey> edgeSet;
	for (const auto& faceId : _faceIds) {
		faceFunc(faceId, [this, &edgeSet, &vertIds](const Polygon& face) {
			MTC::set<Index3DId> vertsInFace;
			for (const auto& vertId : vertIds) {
				if (face.containsVertex(vertId))
					vertsInFace.insert(vertId);
			}
			if (vertsInFace.size() == 2) {
				auto iter = vertsInFace.begin();
				const Index3DId& vertId0 = *iter++;
				const Index3DId& vertId1 = *iter;
				EdgeKey newEdge(vertId0, vertId1);

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

	const auto& e = *newEdgeSet.begin();
	result.push_back(e.getVertexIds()[0]);
	result.push_back(e.getVertexIds()[1]);

	newEdgeSet.erase(newEdgeSet.begin());
	bool found = true;
	while (found && !newEdgeSet.empty()) {
		found = false;
		const auto& lastVert = result.back();
		for (auto iter = newEdgeSet.begin(); iter != newEdgeSet.end(); iter++) {
			const auto& e = *iter;
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
		auto edgeKeys = getEdgeKeys(false);
		for (const auto& edgeKey : edgeKeys) {
			edgeFunc(edgeKey, [this](const Edge& edge) {
				size_t count = 0;
				const auto& faceIds = edge.getFaceIds();
				for (const auto& id : faceIds) {
					if (_faceIds.contains(id)) {
						count++;
					}
				}
				if (count < 2)
					_cachedIsClosed = Trinary::IS_FALSE;
			});

			if (_cachedIsClosed == Trinary::IS_FALSE)
				break;
		}
	}
	return _cachedIsClosed == Trinary::IS_TRUE;
}

bool Polyhedron::lineSegmentIntersects(const LineSegmentd& seg, MTC::vector<RayHitd>& hits, MTC::vector<Index3DId>& faceIds) const
{
	for (const auto& faceId : _faceIds) {
		faceFunc(faceId, [this, &seg, &hits, &faceIds] (const Polygon& face) {
			RayHitd hit;
			if (face.intersect(seg, hit)) {
				hits.push_back(hit);
				faceIds.push_back(face.getId());
			}
		});
	}
	return !faceIds.empty() && hits.size() == faceIds.size();
}

bool Polyhedron::verifyTopology() const
{
	const auto& params = getBlockPtr()->getSplitParams();
	bool valid = true;

	for (const auto& faceId : _faceIds) {
		if (valid && polygonExists(faceId)) {
			faceFunc(faceId, [this, &valid](const Polygon& face) {
				if (valid && !face.usedByCell(getId())) {
					valid = false;
				}
				if (valid && !face.verifyTopology())
					valid = false;
			});
		}
	}

	if (valid && !isClosed()) {
		valid = false;
	}
	
	if (valid && isTooComplex(params))
		valid = false;

#if DUMP_BAD_CELL_OBJS
	if (!valid) {
		getBlockPtr()->dumpPolyhedraObj({ getId() }, false, false, false);
	}
#endif

	return valid;
}

void Polyhedron::clearCache() const
{
	_needsCurvatureCheck = true;
	_isOriented = false;

	_intersectsModel = IS_UNKNOWN; // Cached value
	_sharpEdgesIntersectModel = IS_UNKNOWN;
	_cachedIsClosed = IS_UNKNOWN;

	_cachedMinGap = -1;
}

ostream& DFHM::operator << (ostream& out, const Polyhedron& cell)
{
	auto pBlk = cell.getBlockPtr();

	auto edgeKeys = cell.getEdgeKeys(false);
	bool closed = true;
	for (const auto& edgeKey : edgeKeys) {
		cell.edgeFunc(edgeKey, [&closed](const Edge& edge) {
			if (edge.getFaceIds().size() != 2)
				closed = false;
		});

		if (!closed)
			break;
	}

#if LOGGING_VERBOSE_ENABLED

	out << "Cell: c" << cell.getId() << (closed ? " CLOSED" : " ERROR NOT CLOSED") << "\n";
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
			out << Logger::Pad() << "edges(" << edgeKeys.size() << "): {\n";
			for (const auto& edgeKey : edgeKeys) {
				Logger::Indent indent;
				out << Logger::Pad() << edgeKey << "\n";
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
	const auto p = getBlockPtr(); 
	p->vertexFunc(id, func);
} 

void Polyhedron::vertexFunc(const Index3DId& id, const std::function<void(Vertex& obj)>& func) {
	auto p = getBlockPtr(); 
	p->vertexFunc(id, func);
} 

void Polyhedron::faceFunc(const Index3DId& id, const std::function<void(const Polygon& obj)>& func) const {
	const auto p = getBlockPtr(); 
	p->faceFunc(id, func);
} 

void Polyhedron::faceFunc(const Index3DId& id, const std::function<void(Polygon& obj)>& func) {
	auto p = getBlockPtr(); 
	p->faceFunc(id, func);
} 

void Polyhedron::cellFunc(const Index3DId& id, const std::function<void(const Polyhedron& obj)>& func) const {
	const auto p = getBlockPtr(); 
	p->cellFunc(id, func);
} 

void Polyhedron::cellFunc(const Index3DId& id, const std::function<void(Polyhedron& obj)>& func) {
	auto p = getBlockPtr(); 
	p->cellFunc(id, func);
} 

void Polyhedron::edgeFunc(const EdgeKey& key, const std::function<void(const Edge& obj)>& func) const {
	const auto p = getBlockPtr(); 
	p->edgeFunc(key, func);
} 

void Polyhedron::edgeFunc(const EdgeKey& key, const std::function<void(Edge& obj)>& func) {
	auto p = getBlockPtr(); 
	p->edgeFunc(key, func);
}
