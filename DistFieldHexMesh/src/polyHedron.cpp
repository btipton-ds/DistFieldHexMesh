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
#include <tm_repo.h>
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
#include <meshData.h>

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
	: ObjectPoolOwnerUser(src)
	, _faceIds(src._faceIds)
	, _needsSplitAtCentroid(src._needsSplitAtCentroid)
	, _cachedIsClosed(src._cachedIsClosed)
{
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
		getBlockPtr()->freePolygon(faceId);
	}
}

Polyhedron& Polyhedron::operator = (const Polyhedron& rhs)
{
	ObjectPoolOwnerUser::operator=(rhs);
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

size_t Polyhedron::getNumSplitFaces() const
{
	size_t n = 0;
	for (const auto& faceId : _faceIds) {
		if (getBlockPtr()->polygonExists(faceId)) {
			faceFunc(faceId, [&n](const Polygon& face) {
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
	uint8_t version = 2;
	out.write((char*)&version, sizeof(version));

	IoUtil::writeObj(out, _faceIds);
	out.write((char*)&_splitLevel, sizeof(size_t));

	out.write((char*)&_splitLevel, sizeof(_splitLevel));
	out.write((char*)&_layerNum, sizeof(_layerNum));
}

void Polyhedron::read(std::istream& in)
{
	uint8_t version;
	in.read((char*)&version, sizeof(version));

	if (version < 2)
		IoUtil::read(in, _faceIds);
	else
		IoUtil::readObj(in, _faceIds);

	in.read((char*)&_splitLevel, sizeof(size_t));

	if (version >= 1) {
		in.read((char*)&_splitLevel, sizeof(_splitLevel));
		in.read((char*)&_layerNum, sizeof(_layerNum));
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

void Polyhedron::remapId(const std::vector<size_t>& idRemap, const Index3D& srcDims)
{
	ObjectPoolOwnerUser::remapId(idRemap, srcDims);

	remap(idRemap, srcDims, _faceIds);
}

void Polyhedron::addFace(const Index3DId& faceId, size_t splitLevel)
{
	_faceIds.insert(faceId);
	faceFunc(faceId, [this, splitLevel](Polygon& face) {
		face.addCellId(_thisId);
	});

	clearCache();
}

const MTC::set<Index3DId>& Polyhedron::getVertIds() const
{
	if (_cachedVertIds.empty()) {
		for (const auto& faceId : _faceIds) {
			faceFunc(faceId, [this](const Polygon& refFace) {
				const auto vertexIds = refFace.getVertexIds();
				_cachedVertIds.insert(vertexIds.begin(), vertexIds.end());
			});
		}
	}

	return _cachedVertIds;
}

const MTC::set<Edge>& Polyhedron::getEdges(bool includeAdjacentCellFaces) const
{
	if (includeAdjacentCellFaces) {
		if (!_cachedEdges1.empty())
			return _cachedEdges1;
	} else if (!_cachedEdges0.empty())
		return _cachedEdges0;

	MTC::map<Edge, MTC::set<Index3DId>> edgeToFaceMap;
	MTC::set<Index3DId> adjCellIds;
	for (const auto& faceId : _faceIds) {
		faceFunc(faceId, [this, &edgeToFaceMap, &faceId, &adjCellIds, includeAdjacentCellFaces](const Polygon& face) {
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
			cellFunc(adjCellId, [&faceIds](const Polyhedron& cell) {
				faceIds = cell.getFaceIds();
			});
			for (const auto& faceId : faceIds) {
				faceFunc(faceId, [this, &edgeToFaceMap, &faceId](const Polygon& face) {
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
	

	if (includeAdjacentCellFaces) {
		return _cachedEdges1;
	} else {
		return _cachedEdges0;
	}
}

const MTC::set<Index3DId>& Polyhedron::getAdjacentCells() const
{
	if (_cachedAdjCellIds.empty()) { // All cells have adjacent cells. If this is empty, it hasn't been set
		set<Edge> edges = getEdges(true);
		for (const auto& faceId : _faceIds) {
			faceFunc(faceId, [this, &edges](const Polygon& face) {
				const auto& cellIds1 = face.getCellIds();
				for (const auto& cellId : cellIds1) {
					set<Edge> adjEdges;
					cellFunc(cellId, [this, &adjEdges](const Polyhedron& adjCell) {
						adjEdges = adjCell.getEdges(true);
						});
					const auto& vertIds = getVertIds();
					for (const auto& edge : adjEdges) {
						for (const auto& vertId : vertIds) {
							if (edge.containsVertex(vertId)) {
								edges.insert(edge);
							}
						}
					}
				}
			});
		}

		auto pBlk = getBlockPtr();
		for (const Edge& edge : edges) {
			set<Index3DId> adjCellIds;
			edge.getCellIds(pBlk, adjCellIds);
			for (const auto& adjCellId : adjCellIds) {
				if (isVertexConnectedToCell(adjCellId))
					_cachedAdjCellIds.insert(adjCellId);
			}
		}

		_cachedAdjCellIds.erase(_thisId);
	}

	return _cachedAdjCellIds;
}

// Gets the edges for a vertex which belong to this polyhedron
void Polyhedron::getVertEdges(const Index3DId& vertId, MTC::set<Edge>& result, bool includeAdjacentCells) const
{
	auto cellEdgeSet = getEdges(includeAdjacentCells);
	if (includeAdjacentCells) {
		set<Index3DId> adjCells = getAdjacentCells();
		for (const auto& adjCellId : adjCells) {
			cellFunc(adjCellId, [&cellEdgeSet](const Polyhedron& adjCell) {
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
		auto edges = getEdges(false);
		for (size_t i = 0; i < baseFaceVerts.size(); i++) {
			size_t j = (i + 1) % baseFaceVerts.size();
			edges.erase(Edge(baseFaceVerts[i], baseFaceVerts[j]));
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
	const auto& vertIds = getVertIds();
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
		faceFunc(faceId, [this, &tol, &pt, &cellCtr, &result](const Polygon& face) {
			const double tol = Tolerance::sameDistTol();
			face.iterateTriangles([this, &tol, &pt, &cellCtr, &result](const Index3DId& id0, const Index3DId& id1, const Index3DId& id2)->bool {
				Vector3d triPts[] = {
					getVertexPoint(id0),
					getVertexPoint(id1),
					getVertexPoint(id2),
				};
				Planed pl(triPts[0], triPts[1], triPts[2]);
				Vector3d v = triPts[0] - cellCtr;

				// Assure vector is pointing outwards
				if (v.dot(pl.getNormal()) < 0)
					pl.reverse();

				double dist = pl.distanceToPoint(pt);
				return dist > tol;
			});
		});
		if (!result)
			break;
	}

	return result;
}

bool Polyhedron::containsVertex(const Index3DId& vertId) const
{
	const auto& ourVerts = getVertIds();
	return ourVerts.contains(vertId);
}

bool Polyhedron::isVertexConnectedToCell(const Index3DId& cellId) const
{
	bool result = false;
	cellFunc(cellId, [this, &result](const Polyhedron& cell) {
		const auto& ourVerts = getVertIds();
		const auto& otherVerts = cell.getVertIds();
		for (const auto& otherId : otherVerts) {
			if (ourVerts.contains(otherId)) {
				result = true;
				break;
			}
		}
	});

	return result;
}

bool Polyhedron::isVertexConnectedToFace(const Index3DId& faceId) const
{
	bool result = false;
	faceFunc(faceId, [this, &result](const Polygon& face) {
		const auto& ourVerts = getVertIds();
		const auto& otherVerts = face.getVertexIds();
		for (const auto& otherId : otherVerts) {
			if (ourVerts.contains(otherId)) {
				result = true;
			}
		}
	});

	return false;
}

Vector3d Polyhedron::calCentroid() const
{
	// Use the  curl algorithm over all triangles
	Vector3d ctr(0, 0, 0);
	double vol = 0;
	for (const auto& faceId : _faceIds) {
		faceFunc(faceId, [this, &vol, &ctr](const Polygon& face) {
			face.iterateTriangles([this, &vol, &ctr](const Index3DId& idx0, const Index3DId& idx1, const Index3DId& idx2)->bool {
				const Vector3d axis(1, 0, 0);
				auto pBlk = getBlockPtr();
				const auto& a = pBlk->getVertexPoint(idx0);
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
	const auto& vertIds = getVertIds();
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
	if (true || _intersectsModel == IS_UNKNOWN) {
		CBoundingBox3Dd bbox = getBoundingBox(), modelBBox;

		auto& meshData = *getBlockPtr()->getModelMeshData();
		for (auto& pair : meshData) {
			auto& pMesh = pair.second->getMesh();
			vector<size_t> triIndices;
			if (pMesh->findTris(bbox, triIndices)) {
				for (size_t triIdx : triIndices) {
					const auto& tri = pMesh->getTri(triIdx);
					Vector3d pts[3];
					for (int i = 0; i < 3; i++) {
						const auto& pt = pMesh->getVert(tri[i])._pt;
#if 0
						if (contains(pt)) {
							_intersectsModel = IS_TRUE;
							return true;
						}
#endif
						pts[i] = pt;
					}

					for (const auto& faceId : _faceIds) {
						faceFunc(faceId, [this, &pts](const Polygon& face) {
							if (face.intersectsTri(pts)) {
								_intersectsModel = IS_TRUE;
							}
						});

						if (_intersectsModel == IS_TRUE)
							return true;
					}

				}
			}
		}

		_intersectsModel = IS_FALSE;
	}

	return _intersectsModel == IS_TRUE; // Don't test split cells
}

bool Polyhedron::sharpEdgesIntersectModel(const BuildCFDParams& params) const
{
	if (_sharpEdgesIntersectModel == IS_FALSE)
		return false;

	CBoundingBox3Dd bbox = getBoundingBox();
	const double sinSharpEdgeAngle = sin(params.getSharpAngleRadians());
	MTC::vector<size_t> sharpEdges;
	auto& meshData = *getBlockPtr()->getModelMeshData();
	for (auto& pair : meshData) {
		auto& pMesh = pair.second->getMesh();
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
	const auto& meshData = *getBlockPtr()->getModelMeshData();
	for (const auto& pair : meshData) {
		auto pMesh = pair.second->getMesh();
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

	MTC::set<Index3DId> orientedIds, unorientedIds(_faceIds);
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
			faceFunc(faceId, [this](Polygon& face) {
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
		faceFunc(faceId, [&verts](const Polygon& face) {
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
		faceFunc(faceId, [&verts, &imprintVertices](Polygon& face) {
			face.needToImprintVertices(verts, imprintVertices);
		});

		if (!imprintVertices.empty()) {
			getBlockPtr()->removeFaceFromLookUp(faceId);
			faceFunc(faceId, [&imprintVertices](Polygon& face) {
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
	cellFunc(_thisId, [&refFaceIds](const Polyhedron& refCell) {
		refFaceIds = refCell.getFaceIds();
	});

	map<Edge, Index3DId> splitEdgeVertMap;
	for (const auto& refFaceId : refFaceIds) {
		faceFunc(refFaceId, [&splitEdgeVertMap](const Polygon& refFace) {
			const auto& tmp = refFace.getSplitEdgeVertMap();
			splitEdgeVertMap.insert(tmp.begin(), tmp.end());
		});
	}

	set<Index3DId> imprintIds;
	for (const auto& faceId : _faceIds) {
		if (getBlockPtr()->polygonExists(faceId)) {
			faceFunc(faceId, [&splitEdgeVertMap, &imprintIds](const Polygon& face) {
				if (face.needToImprintVertices(splitEdgeVertMap))
					imprintIds.insert(face.getId());
			});
		}
	}

	if (!imprintIds.empty()) {

		for (const auto& faceId : imprintIds) {
			pDstBlock->makeRefPolygonIfRequired(faceId);
			if (getBlockPtr()->polygonExists(faceId)) {
				pDstBlock->faceFunc(faceId, [&splitEdgeVertMap](Polygon& face) {
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
		if (getBlockPtr()->polygonExists(faceId)) {
			faceFunc(faceId, [this](Polygon& face) {
				face.addCellId(getId());
			});
		}
	}
}

void Polyhedron::detachFaces()
{
	for (const auto& faceId : _faceIds) {
		if (getBlockPtr()->polygonExists(faceId)) {
			faceFunc(faceId, [this](Polygon& face) {
				face.removeCellId(getId());
			});
		}
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
		faceFunc(faceId, [this, &adjCellId, &faceSplitLevel](const Polygon& face) {
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
				cellFunc(adjCellId, [this, &blockingCellIds](const Polyhedron& adjCell) {
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
		set<Edge> edges;
		cellFunc(_thisId, [&edges](const Polyhedron& cell) {
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
			if (2 * maxEdgeLength > maxAllowedEdgeLen) {
				setNeedsDivideAtCentroid();
				return true;
			}
		}
	}

	return false;
}

bool Polyhedron::needToDivideDueToSplitFaces(const BuildCFDParams& params)
{
	if (_faceIds.size() > params.maxCellFaces) {
		setNeedsDivideAtCentroid();
		return true;
	}

	return false;
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

	_needsCurvatureCheck = false;
	const auto& meshData = *getBlockPtr()->getModelMeshData();

	if (meshData.empty())
		return 0;

	for (const auto& pair : meshData) {
		auto pTriMesh = pair.second->getMesh();

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

bool Polyhedron::polygonExists(const Index3DId& id) const
{
	return getBlockPtr()->polygonExists(id);
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

void Polyhedron::clearLayerNum()
{
	_layerNum = -1;
}

bool Polyhedron::setLayerNum(int thisLayerNum, bool propagate)
{
	bool changed = false;
	if (thisLayerNum == 0) {
		if (_layerNum == -1 && intersectsModel()) {
			auto pBlk = getBlockPtr();
			pBlk->addToSeedFillList(_thisId);
			_layerNum = thisLayerNum;
			changed = true;
		}
	} else {
		auto pBlk = getBlockPtr();
		if (propagate) {
			const auto& adj = getAdjacentCells();
			for (const auto& id : adj) {
				if (pBlk->polyhedronExists(id)) {
					cellFunc(id, [this, thisLayerNum, &changed](Polyhedron& adjCell) {
						if (adjCell.exists() && adjCell.getLayerNum() == -1) {
							adjCell.setLayerNum(thisLayerNum, false);
							changed = true;
						}
					});
				}
			}
		} else {
			pBlk->addToSeedFillList(_thisId);
			_layerNum = thisLayerNum;
			changed = true;
		}
	}

	return changed;
}

MTC::set<Edge> Polyhedron::createEdgesFromVerts(MTC::vector<Index3DId>& vertIds) const
{
	// This function takes vertices and looks for faces which contain exactly 2 of the available verts.
	// The edge may already exist.
	MTC::set<Edge> edgeSet;
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

bool Polyhedron::hasTooManySplits() const
{
	bool result = false;

	for (const auto& faceId : _faceIds) {
		faceFunc(faceId, [this, &result](const Polygon& face) {
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
		if (valid && polygonExists(faceId)) {
			faceFunc(faceId, [this, &valid](const Polygon& face) {
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
	_isOriented = false;

	_intersectsModel = IS_UNKNOWN; // Cached value
	_cachedIsClosed = IS_UNKNOWN;

	_cachedMinGap = -1;

	_cachedEdges0.clear();
	_cachedEdges1.clear();
	_cachedVertIds.clear();

	clearAdjCellIdCache();
}

void Polyhedron::clearAdjCellIdCache() const
{
	_cachedAdjCellIds.clear();
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

LAMBDA_CLIENT_IMPLS(Polyhedron)
