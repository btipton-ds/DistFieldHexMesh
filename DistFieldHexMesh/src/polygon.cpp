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

#include <defines.h>
#include <cmath>
#include <iostream>
#include <fstream>
#include <assert.h>
#include <pool_vector.h>
#include <pool_set.h>
#include <tm_math.h>
#include <tm_lineSegment.h>
#include <tm_ray.h>
#include <tm_ioUtil.h>
#include <objectPool.hpp>
#include <vertex.h>
#include <edge.h>
#include <polygon.h>
#include <block.h>
#include <logger.h>
#include <volume.h>
#include <edge.h>
#include <utils.h>
#include <io_utils.h>
#include <tolerances.h>
#include <splitParams.h>
#include <meshData.h>

#define CACHE_BIT_SORTED 1
#define CACHE_BIT_EDGES 2

using namespace std;
using namespace DFHM;

Polygon::Polygon(const std::vector<Index3DId>& verts)
	: _vertexIds(verts)
{
}

#if USE_MULTI_THREAD_CONTAINERS
Polygon::Polygon(const MTC::vector<Index3DId>& verts)
	: _vertexIds(verts)
{
}
#endif

Polygon::Polygon(const std::initializer_list<Index3DId>& verts)
	: _vertexIds(verts)
{
}

Polygon::Polygon(const Polygon& src)
	: ObjectPoolOwnerUser(src)
	, _createdDuringSplitNumber(src._createdDuringSplitNumber)
	, _splitIds(src._splitIds)
	, _vertexIds(src._vertexIds)
	, _cellIds(src._cellIds)
	, _cachedIntersectsModel(src._cachedIntersectsModel)
	, _cachedEdgesVaild(src._cachedEdgesVaild)
	, _cachedEdges(src._cachedEdges)
	// Don't copy the caches
{
}

DFHM::Polygon& DFHM::Polygon::operator = (const Polygon& rhs)
{
	clearCache();
	ObjectPoolOwnerUser::operator=(rhs);
	_createdDuringSplitNumber = rhs._createdDuringSplitNumber;
	_splitIds = rhs._splitIds;
	_vertexIds = rhs._vertexIds;
	_cellIds = rhs._cellIds;
	_cachedIntersectsModel = rhs._cachedIntersectsModel;
	_cachedEdgesVaild = rhs._cachedEdgesVaild;
	_cachedEdges = rhs._cachedEdges;

	return *this;
}

void Polygon::remapId(const std::vector<size_t>& idRemap, const Index3D& srcDims)
{
	ObjectPoolOwnerUser::remapId(idRemap, srcDims);

	remap(idRemap, srcDims, _splitIds);

	remap(idRemap, srcDims, _vertexIds);
	remap(idRemap, srcDims, _cellIds);
}

void Polygon::addVertex(const Index3DId& vertId)
{
	_vertexIds.push_back(vertId);
	clearCache();
}

void Polygon::clearCache() const
{
	_sortCacheVaild = false;
	_cachedEdgesVaild = false;
	_isConvex = IS_UNKNOWN;
	_cachedIntersectsModel = IS_UNKNOWN;
	_cachedEdges.clear();
	_sortedIds.clear();

	// Clear our owner cells' caches
	for (const auto& cellId : _cellIds) {
		cellFunc(cellId, [](const Polyhedron& cell) {
			cell.clearCache();
		});
	}
}

bool Polygon::cellsOwnThis() const
{
	for (const auto& cellId : _cellIds) {
		if (!getBlockPtr()->polyhedronExists(cellId))
			return false;
		bool result = true;
		cellFunc(cellId, [this, &result](const Polyhedron& cell) {
			if (!cell.containsFace(_thisId))
				result = false;
		});

		if (!result)
			return false;
	}

	return true;
}

size_t Polygon::getSplitLevel(const Index3DId& cellId) const
{
	auto iter = _cellIds.find(cellId);
	assert(iter != _cellIds.end());
	if (iter != _cellIds.end())
		return iter->getSplitLevel();

	return -1;
}

Index3DId Polygon::getAdjacentCellId(const Index3DId& thisCellId) const
{
	Index3DId result;
	if (_cellIds.size() == 2) {
		for (const auto& id : _cellIds) {
			if (id != thisCellId) {
				result = id;
				break;
			}
		}
	}

	return result;
}

void Polygon::write(ostream& out) const
{
	uint8_t version = 1;
	out.write((char*)&version, sizeof(version));

	out.write((char*)&_createdDuringSplitNumber, sizeof(_createdDuringSplitNumber));

	IoUtil::write(out, _splitIds);
	IoUtil::write(out, _vertexIds);
	IoUtil::write(out, _cellIds);

}

void Polygon::read(istream& in)
{
	uint8_t version;
	in.read((char*)&version, sizeof(version));

	in.read((char*)&_createdDuringSplitNumber, sizeof(_createdDuringSplitNumber));

	IoUtil::read(in, _splitIds);
	if (version == 0) {
		MTC::map<Edge, Index3DId> deprecated;
		IoUtil::read(in, deprecated);
	}
	IoUtil::read(in, _vertexIds);
	IoUtil::read(in, _cellIds);
}

bool Polygon::unload(ostream& out, size_t idSelf)
{

	return true;
}

bool Polygon::load(istream& in, size_t idSelf)
{

	return true;
}

void Polygon::initVertices(const Volume* pVol) const
{
}

void Polygon::sortIds() const
{
	if (true || !_sortCacheVaild) {
		_sortCacheVaild = true;
		_sortedIds = _vertexIds;
		sort(_sortedIds.begin(), _sortedIds.end());
	}
}


bool Polygon::operator < (const Polygon& rhs) const
{
	sortIds();
	rhs.sortIds();

	if (_sortedIds.size() < rhs._sortedIds.size())
		return true;
	else if (_sortedIds.size() > rhs._sortedIds.size())
		return false;

	for (size_t i = 0; i < _sortedIds.size(); i++) {
		if (_sortedIds[i] < rhs._sortedIds[i])
			return true;
		else if (rhs._sortedIds[i] < _sortedIds[i])
			return false;
	}
	return false;
}

bool Polygon::isBlockBoundary() const
{
	if (_cellIds.size() == 2) {
		auto iter0 = _cellIds.begin();
		auto iter1 = iter0++;
		Index3DId id0 = *iter0;
		Index3DId id1 = *iter1;
		return (id0.blockIdx() != id1.blockIdx());
	}
	return false;
}

const MTC::set<Edge>& Polygon::getEdges() const
{
	if (!_cachedEdgesVaild) {
		createEdgesStat(_vertexIds, _cachedEdges, _thisId);
		_cachedEdgesVaild = true;
	}
#if 0 && defined(_DEBUG)
	{
		set<Edge> test;
		createEdgesStat(_vertexIds, test, _thisId);
		assert(_cachedEdges.size() == test.size());
		for (const auto& e : test) {
			assert(_cachedEdges.contains(e));
		}
		for (const auto& e : _cachedEdges) {
			assert(test.contains(e));
		}
	}
#endif // _DEBUG

	return _cachedEdges;
}

bool Polygon::containsPoint(const Vector3d& pt) const
{
	if (!isPointOnPlane(pt) || !isConvex())
		return false;

	Vector3d norm = calUnitNormal();
	for (size_t i = 0; i < _vertexIds.size(); i++) {
		size_t j = (i + 1) % _vertexIds.size();

		Vector3d pt0 = getVertexPoint(_vertexIds[i]);
		Vector3d pt1 = getVertexPoint(_vertexIds[j]);
		Vector3d v1 = (pt1 - pt0).normalized();
		Vector3d v0 = pt - pt0;
		v0 = v0 - v0.dot(v1) * v1;
		Vector3d cp = v1.cross(v0);
		double dist = norm.dot(cp);
	
		if (dist < -Tolerance::sameDistTol())
			return false;
	}
	return true;
}

bool Polygon::isPointOnPlane(const Vector3d& pt) const {
	return distFromPlane(pt) < Tolerance::sameDistTol();
}

bool Polygon::findPiercePoints(const std::vector<size_t>& edgeIndices, MTC::vector<RayHitd>& piercePoints) const
{
	piercePoints.clear();
#if 0
	Planed pl = calPlane();
	auto pMesh = getBlockPtr()->getModelMesh();
	for (size_t edgeIdx : edgeIndices) {
		const auto& edge = pMesh->getEdge(edgeIdx);
		auto seg = edge.getSeg(pMesh);
		RayHitd hit;
		if (pl.intersectLineSegment(seg, hit, Tolerance::sameDistTol())) {
			hit.edgeIdx = edgeIdx;
			piercePoints.push_back(hit);
		}
	}
#endif
	return !piercePoints.empty();
}

bool Polygon::usesEdge(const Edge& edge) const
{
	size_t idx0, idx1;
	return usesEdge(edge, idx0, idx1);
}

bool Polygon::usesEdge(const Edge& edge, size_t& idx0, size_t& idx1) const
{
	for (size_t i = 0; i < _vertexIds.size(); i++) {
		size_t j = (i + 1) % _vertexIds.size();
		const auto& vertId0 = _vertexIds[i];
		const auto& vertId1 = _vertexIds[j];
		Edge testEdge(vertId0, vertId1);
		if (testEdge == edge) {
			idx0 = i;
			idx1 = j;
			return true;
		}
	}

	idx0 = idx1 = -1;
	return false;
}

bool Polygon::contains(const Edge& edge, bool& isUsed) const
{
	isUsed = false;
	if (!isCoplanar(edge))
		return false;

	if (usesEdge(edge)) {
		isUsed = true;
		return true;
	}

	Vector3d pt0 = getBlockPtr()->getVertexPoint(edge.getVertex(0));
	Vector3d pt1 = getBlockPtr()->getVertexPoint(edge.getVertex(1));
	bool intersects = false;
	if (isConvex()) {
		if (containsPoint(pt0) || containsPoint(pt1))
			return true;

		Vector3d faceNorm = calUnitNormal();
		Vector3d v = edge.calUnitDir(getBlockPtr());
		Vector3d iNorm = v.cross(faceNorm).normalized();
		Planed iPlane(pt0, iNorm);
		iterateEdges([this, &iPlane, &intersects](const Edge& ie) {
			auto seg = ie.getSegment(getBlockPtr());
			RayHitd hit;
			if (iPlane.intersectLineSegment(seg, hit, Tolerance::sameDistTol())) {
				intersects = true;
			}
			return !intersects;
		});

	} else {
		iterateEdges([this, &pt0, &pt1, &intersects](const Edge& ie) {
			if (ie.pointLiesOnEdge(getBlockPtr(), pt0) || ie.pointLiesOnEdge(getBlockPtr(), pt1))
				intersects = true;
			return !intersects;
		});
	}

	return intersects;
}

bool Polygon::containsVertex(const Index3DId& vertId) const
{
	for (const auto& id : _vertexIds) {
		if (id == vertId)
			return true;
	}
	return false;
}

bool Polygon::isCoplanar(const Vector3d& pt) const
{
	Planed pl = calPlane();
	return (fabs(pl.distanceToPoint(pt)) < Tolerance::sameDistTol());
}

bool Polygon::isCoplanar(const Planed& pl) const
{
	Planed ourPlane = calPlane();
	return ourPlane.isCoincident(pl, Tolerance::planeCoincidentDistTol(), Tolerance::planeCoincidentCrossProductTol());
}

bool Polygon::isCoplanar(const Edge& edge) const
{
	Planed pl = calPlane();
	const auto pt0 = getVertexPoint(edge.getVertex(0));
	if (!pl.isCoincident(pt0, Tolerance::sameDistTol()))
		return false;

	const auto pt1 = getVertexPoint(edge.getVertex(1));
	if (!pl.isCoincident(pt1, Tolerance::sameDistTol()))
		return false;

	return true;
}

void Polygon::createEdgesStat(const MTC::vector<Index3DId>& verts, MTC::set<Edge>& edgeSet, const Index3DId& polygonId)
{
	for (size_t i = 0; i < verts.size(); i++) {
		size_t j = (i + 1) % verts.size();
		// If the edges is aready in the set, we get the existing one, not the new one?
		set<Index3DId> faceSet;
		if (polygonId.isValid())
			faceSet.insert(polygonId);
		edgeSet.insert(Edge(verts[i], verts[j], faceSet));
	}
}
Vector3d Polygon::calCentroidStat(const Block* pBlock, const MTC::vector<Index3DId>& vertIds)
{
	Vector3d ctr(0, 0, 0);
	if (vertIds.empty())
		return Vector3d(DBL_MAX, DBL_MAX, DBL_MAX);
	for (size_t i = 0; i < vertIds.size(); i++)
		ctr += pBlock->getVertexPoint(vertIds[i]);
	ctr /= vertIds.size();

	return ctr;
}

void Polygon::calCoordSysStat(const Block* pBlock, const MTC::vector<Index3DId>& vertIds, Vector3d& origin, Vector3d& xAxis, Vector3d& yAxis, Vector3d& zAxis)
{
	origin = calCentroidStat(pBlock, vertIds);
	zAxis = Polygon::calUnitNormalStat(pBlock, vertIds);
	xAxis = Vector3d(1, 0, 0);
	if (fabs(xAxis.dot(zAxis)) > 0.7071) {
		xAxis = Vector3d(0, 1, 0);
		if (fabs(xAxis.dot(zAxis)) > 0.7071) {
			xAxis = Vector3d(0, 0, 1);
		}
	}
	xAxis = xAxis - xAxis.dot(zAxis) * zAxis;
	xAxis.normalize();
	yAxis = zAxis.cross(xAxis);
}

void Polygon::findConcaveVertIdsStat(const Block* pBlock, const MTC::vector<Index3DId>& vertIds, MTC::set<Index3DId>& cVertIds)
{
	auto zAxis = calUnitNormalStat(pBlock, vertIds);

	for (size_t i = 0; i < vertIds.size(); i++) {
		size_t j = (i + 1) % vertIds.size();
		size_t k = (j + 1) % vertIds.size();

		auto pt0 = pBlock->getVertexPoint(vertIds[i]);
		auto pt1 = pBlock->getVertexPoint(vertIds[j]);
		auto pt2 = pBlock->getVertexPoint(vertIds[k]);

		Vector3d v0 = pt0 - pt1;
		Vector3d v1 = pt2 - pt1;
		Vector3d vN = v1.cross(v0);
		if (vN.dot(zAxis) < 0) {
			// Concave vertex
			cVertIds.insert(vertIds[j]);
		}
	}
}

Vector3d Polygon::calUnitNormalStat(const Block* pBlock, const MTC::vector<Index3DId>& vertIds)
{
	Vector3d norm(0, 0, 0);

	size_t i = 0;
	Vector3d pt0 = pBlock->getVertexPoint(vertIds[i]);
	for (size_t j = 1; j < vertIds.size() - 1; j++) {
		size_t k = (j + 1) % vertIds.size();

		Vector3d pt1 = pBlock->getVertexPoint(vertIds[j]);
		Vector3d pt2 = pBlock->getVertexPoint(vertIds[k]);

		Vector3d v0 = pt0 - pt1;
		Vector3d v1 = pt2 - pt1;

		Vector3d n = v1.cross(v0);
		double l = n.norm();
		if (l > Tolerance::angleTol()) {
			n /= l;
			norm += n;
		}
	}
	norm.normalize();
	return norm;
}

void Polygon::dumpPolygonPoints(const Block* pBlock, ostream& out, const MTC::vector<Index3DId>& vertIds)
{
	MTC::vector<Vector3d> pts;
	for (const auto& id : vertIds) {
		Vector3d pt = pBlock->getVertexPoint(id);
		pts.push_back(pt);
	}

	dumpPolygonPoints(out, pts);
}

void Polygon::dumpPolygonPoints(ostream& out, const MTC::vector<Vector3d>& pts)
{
	out << "poly pts\n";
	for (const auto& pt : pts) {
		out << "v " << pt[0] << " " << pt[1] << " " << pt[2] << "\n";
	}

	out << "f";
	for (size_t i = 0; i < pts.size(); i++)
		out << " " << (i + 1);
	out << "\n";
}

Vector3d Polygon::calUnitNormal() const
{
	return calUnitNormalStat(getBlockPtr(), _vertexIds);
}

Vector3d Polygon::calOrientedUnitNormal(const Index3DId& cellId) const
{
	Vector3d result = calUnitNormalStat(getBlockPtr(), _vertexIds);
	if (isReversed(cellId))
		return -result;

	return result;
}

Planed Polygon::calPlane() const
{
	Vector3d origin = calCentroid(); // Use every point to get more preceision
	Vector3d normal = calUnitNormal();
	Planed result(origin, normal);

#if 0 && defined(_DEBUG)
	for (const auto& vId : _vertexIds) {
		Vector3d pt = getVertexPoint(vId);
		assert(result.distanceToPoint(pt) < Tolerance::sameDistTol());
	}
#endif // _DEBUG

	return result;
}

Planed Polygon::calOrientedPlane(const Index3DId& cellId) const
{
	Vector3d origin = calCentroid(); // Use every point to get more preceision
	Vector3d normal = calOrientedUnitNormal(cellId);
	Planed result(origin, normal);

#if 0 && defined(_DEBUG)
	for (const auto& vId : _vertexIds) {
		Vector3d pt = getVertexPoint(vId);
		assert(result.distanceToPoint(pt) < Tolerance::sameDistTol());
	}
#endif // _DEBUG

	return result;
}

double Polygon::calVertexAngleStat(const Block* pBlock, const MTC::vector<Index3DId>& vertIds, size_t idx1)
{
	const size_t sz = vertIds.size();
	if (idx1 < sz) {
		Vector3d norm = calUnitNormalStat(pBlock, vertIds);
		size_t idx0 = (idx1 + sz - 1) % sz;
		size_t idx2 = (idx1 + 1) % sz;

		Vector3d pt0 = pBlock->getVertexPoint(vertIds[idx0]);
		Vector3d pt1 = pBlock->getVertexPoint(vertIds[idx1]);
		Vector3d pt2 = pBlock->getVertexPoint(vertIds[idx2]);

		Vector3d v0 = pt0 - pt1;
		Vector3d v1 = pt2 - pt1;

		double dp = v1.dot(v0);
		double cp = v1.cross(v0).dot(norm);
		double angle = atan2(cp, dp);
		return angle;
	}

	return nanf("");
}

double Polygon::getShortestEdge() const
{
	double minDist = DBL_MAX;
	for (size_t i = 0; i < _vertexIds.size(); i++) {
		size_t j = (i + 1) % _vertexIds.size();
		Vector3d pt0 = getVertexPoint(_vertexIds[i]);
		Vector3d pt1 = getVertexPoint(_vertexIds[1]);
		double l = (pt1 - pt0).norm();
		if (l < minDist)
			minDist = l;
	}
	return minDist;
}

double Polygon::distanceToPoint(const Vector3d& pt) const
{
	Vector3d ctr = calCentroid();
	for (size_t i = 0; i < _vertexIds.size() - 1; i++) {
		size_t j = (i + 1) % _vertexIds.size();
		Vector3d pt0 = getVertexPoint(_vertexIds[i]);
		Vector3d pt1 = getVertexPoint(_vertexIds[j]);
		Vector3d v0 = pt0 - ctr;
		Vector3d v1 = pt1 - ctr;
		Vector3d n = v1.cross(v0).normalized();
		Planed pl(ctr, n);
		double d = pl.distanceToPoint(pt);
		return d;
	}
	return DBL_MAX;
}

double Polygon::calVertexAngle(size_t idx1) const
{
	return calVertexAngleStat(getBlockPtr(), _vertexIds, idx1);
}

Vector3d Polygon::interpolatePoint(double t, double u) const
{
	assert(_vertexIds.size() == 4);
	Vector3d pts[] = {
		getVertexPoint(_vertexIds[0]),
		getVertexPoint(_vertexIds[1]),
		getVertexPoint(_vertexIds[2]),
		getVertexPoint(_vertexIds[3]),
	};

	return BI_LERP(pts[0], pts[1], pts[2], pts[3], t, u);
}

Vector3d Polygon::calCentroid() const
{
	double area;
	Vector3d ctr;
	calAreaAndCentroid(area, ctr);
	return ctr;
}

bool Polygon::intersectsModel() const
{
	if (_cachedIntersectsModel == IS_UNKNOWN) {
		_cachedIntersectsModel = IS_FALSE;

		for (auto& cellId : _cellIds) {
			cellFunc(cellId, [this](const Polyhedron& cell) {
				auto& meshData = *getBlockPtr()->getModelMeshData();
				for (auto& pair : meshData) {
					auto& pMesh = pair.second->getMesh();
					vector<size_t> triIndices;
					if (pMesh->findTris(cell.getBoundingBox(), triIndices)) {
						for (const auto& i : triIndices) {
							const auto& triIdx = pMesh->getTri(i);
							Vector3d modelTri[] = {
								pMesh->getVert(triIdx[0])._pt,
								pMesh->getVert(triIdx[1])._pt,
								pMesh->getVert(triIdx[2])._pt
							};
							bool result = false;
							getTriPoints([this, modelTri](const Vector3d& pt0, const Vector3d& pt1, const Vector3d& pt2) {
								Vector3d meshTri[] = { pt0, pt1, pt2 };
								if (collisionTriTri(modelTri, meshTri, Tolerance::sameDistTol())) {
									_cachedIntersectsModel = IS_TRUE;
								}
							},
								[](const Vector3d& pt0, const Vector3d& pt1) {
							});

							if (_cachedIntersectsModel == IS_TRUE)
								break;
						}
					}
				}
			});
		}
#if 0
			const auto& meshData = *getBlockPtr()->getModelMeshData();
			for (const auto& pair : meshData) {
				auto pMesh = pair.second->getMesh();;
				CBoundingBox3Dd bbox;
				for (const auto& vertId : _vertexIds) {
					bbox.merge(getVertexPoint(vertId));
				}

				std::vector<size_t> triIndices;
				if (pMesh->processFoundTris(cellTris, bbox, triIndices)) {
					const auto& edges = getEdges();

					for (const auto& triIdx : triIndices) {
						const auto& tri = pMesh->getTri(triIdx);
						const Vector3d* pts[3] = {
							pts[0] = &(pMesh->getVert(tri[0])._pt),
							pts[1] = &(pMesh->getVert(tri[1])._pt),
							pts[2] = &(pMesh->getVert(tri[2])._pt),
						};

						for (const auto& edge : edges) {
							auto seg = edge.getSegment(getBlockPtr());
							RayHitd hit;
							if (seg.intersectTri(pts, hit, Tolerance::sameDistTol())) {
								_cachedIntersectsModel = IS_TRUE;
								break;
							}
						}
						if (_cachedIntersectsModel == IS_TRUE)
							break;
					}
				}
			}
		}
#endif
	}

	return _cachedIntersectsModel == IS_TRUE; // Don't test split cells
}

double Polygon::distFromPlane(const Vector3d& pt) const
{
	Plane pl(getVertexPoint(_vertexIds[0]), calUnitNormal());
	return pl.distanceToPoint(pt);
}

void Polygon::calAreaAndCentroid(double& area, Vector3d& centroid) const
{
	area = 0;
	centroid = Vector3d(0, 0, 0);
	Vector3d pt0 = getVertexPoint(_vertexIds[0]);
	for (size_t j = 1; j < _vertexIds.size() - 1; j++) {
		size_t k = j + 1;
		Vector3d pt1 = getVertexPoint(_vertexIds[j]);
		Vector3d pt2 = getVertexPoint(_vertexIds[k]);
		Vector3d v0 = pt0 - pt1;
		Vector3d v1 = pt2 - pt1;
		Vector3d triCtr = (pt0 + pt1 + pt2) * (1.0 / 3.0);
		Vector3d cp = v1.cross(v0);
		double triArea = cp.norm() / 2.0;

		centroid += triCtr * triArea;
		area += triArea;
	}

	centroid /= area;
}

Vector3d Polygon::projectPoint(const Vector3d& pt) const
{
	Vector3d origin = getVertexPoint(_vertexIds[0]); // And point will do
	Vector3d normal = calUnitNormal();
	Plane pl(origin, normal);
	auto result = pl.projectPoint(pt);

	return result;
}

void Polygon::removeCellId(const Index3DId& cellId)
{
	_cellIds.erase(cellId);
}

void Polygon::removeDeadCellIds()
{
	set<Index3DId> tmp;
	for (const auto& cellId : _cellIds) {
		if (getBlockPtr()->polyhedronExists(cellId))
			tmp.insert(cellId);
	}

#if DEBUG_BREAKS && defined(_DEBUG)
	if (_cellIds.size() != tmp.size()) {
		int dbgBreak = 1;
	}
#endif

	_cellIds = tmp;
}

void Polygon::addCellId(const Index3DId& cellId, size_t level)
{
#if DEBUG_BREAKS && defined(_DEBUG)
	if (Index3DId(4, 6, 1, 0) == cellId) {
		int dbgBreak = 1;
	}
#endif

	_cellIds.erase(cellId); // Erase to clear split level and replace with the new one
	cellId.setSplitLevel(level);
	_cellIds.insert(cellId);
#if 1 && defined(_DEBUG)
	if (_cellIds.size() > 2) {
		for (const auto& cellId1 : _cellIds) {
			assert(getBlockPtr()->polyhedronExists(cellId1));
			cellFunc(cellId1, [this](const Polyhedron& cell) {
				assert(cell.containsFace(_thisId));
			});
		}
		assert(_cellIds.size() <= 2);
	}
#endif
}

void Polygon::unlinkFromCell(const Index3DId& cellId)
{
	_cellIds.erase(cellId);
}

void Polygon::addToSplitFaceProductIds(const Index3DId& id) const
{
	assert(getBlockPtr()->isPolygonReference(this));
	Polygon* refSelf = const_cast<Polygon*>(this);
	refSelf->_splitIds.insert(id);
}

void Polygon::needToImprintVertices(const MTC::set<Index3DId>& verts, MTC::set<Index3DId>& imprintVerts) const
{

	MTC::vector<Index3DId> onFaceVerts;
	MTC::set<Index3DId> vertSet;
	vertSet.insert(_vertexIds.begin(), _vertexIds.end());
	for (const auto& vertId : verts) {
		if (!vertSet.contains(vertId)) { // ignore vertices already in the face
			Vector3d pt = getVertexPoint(vertId);
			if (distanceToPoint(pt) < SAME_DIST_TOL) {
				onFaceVerts.push_back(vertId);
			}
		}
	}

	for (size_t i = 0; i < _vertexIds.size(); i++) {
		size_t j = (i + 1) % _vertexIds.size();
		Edge edge(_vertexIds[i], _vertexIds[j]);
		auto seg = edge.getSegment(getBlockPtr());
		for (const auto& vertId : onFaceVerts) {
			Vector3d pt = getVertexPoint(vertId);
			double t;
			if (seg.contains(pt, t, Tolerance::sameDistTol())) {
				imprintVerts.insert(vertId);
				break;
			}
		}
	}
}

size_t Polygon::getImprintIndex(const Vector3d& imprintPoint) const
{
	cout << "Searching for imprint point " << imprintPoint << "\n";
	for (size_t i = 0; i < _vertexIds.size(); i++) {
		size_t j = (i + 1) % _vertexIds.size();
		Edge edge(_vertexIds[i], _vertexIds[j]);
		auto seg = edge.getSegment(getBlockPtr());
		double t;
		if (seg.contains(imprintPoint, t, Tolerance::sameDistTol())) {
			return i;
		}
	}
	return -1;
}

size_t Polygon::getImprintIndex(const Index3DId& imprintVert) const
{
	Vector3d pt = getVertexPoint(imprintVert);
	return getImprintIndex(pt);
}

bool Polygon::imprintVertex(const Index3DId& imprintVert)
{
	auto tmp = _vertexIds;
	_vertexIds.clear();
	_vertexIds.reserve(tmp.size() + 1);
	bool imprinted = false;
	for (size_t i = 0; i < tmp.size(); i++) {
		_vertexIds.push_back(tmp[i]);

		if (!imprinted) {
			size_t j = (i + 1) % tmp.size();
			Edge edge(tmp[i], tmp[j]);
			auto seg = edge.getSegment(getBlockPtr());
			Vector3d pt = getVertexPoint(imprintVert);
			double t;
			if (seg.contains(pt, t, Tolerance::sameDistTol())) {
				_vertexIds.push_back(imprintVert);
				imprinted = true;
			}
		}
	}

	if (imprinted)
		clearCache();

	return imprinted;
}

bool Polygon::isSplit() const
{
	return _splitIds.size() == _vertexIds.size();
}

bool Polygon::isPlanar() const
{
	if (_vertexIds.size() == 3)
		return true;

	Vector3d ctr = calCentroid();
	Vector3d norm = calUnitNormal();
	Planed pl(ctr, norm);
	for (size_t i = 0; i < _vertexIds.size(); i++) {
		Vector3d pt = getVertexPoint(_vertexIds[i]);
		if (pl.distanceToPoint(pt) > Tolerance::sameDistTol())
			return false;
	}

	return true;
}

bool Polygon::intersect(const LineSegmentd& seg, RayHitd& hit) const
{
	Vector3d pt0 = getVertexPoint(_vertexIds[0]);
	for (size_t i = 1; i < _vertexIds.size() - 1; i++) {
		size_t j = (i + 1);
		Vector3d pt1 = getVertexPoint(_vertexIds[i]);
		Vector3d pt2 = getVertexPoint(_vertexIds[j]);
		if (seg.intersectTri(pt0, pt1, pt2, hit, Tolerance::sameDistTol()))
			return true;
	}

	return false;
}

bool Polygon::intersect(const Planed& pl, LineSegmentd& intersectionSeg) const
{
	// This collapses duplicate corner hits to a single hit
	MTC::set<Vector3d> intersectionPoints;
	for (size_t i = 0; i < _vertexIds.size(); i++) {
		size_t j = (i + 1) % _vertexIds.size();
		Edge edge(_vertexIds[i], _vertexIds[j]);
		auto edgeSeg = edge.getSegment(getBlockPtr());
		RayHitd hit;
		if (pl.intersectLineSegment(edgeSeg, hit, Tolerance::sameDistTol())) {
			intersectionPoints.insert(hit.hitPt);
		}
	}

	if (intersectionPoints.size() == 2) {
		auto iter = intersectionPoints.begin();
		const auto& pt0 = *iter++;
		const auto& pt1 = *iter++;
		intersectionSeg = LineSegmentd(pt0, pt1);
		return true;
	} else if (!intersectionPoints.empty()) {
		int dbgBreak = 1;
	}

	return false;
}

bool Polygon::isPointOnEdge(const Vector3d& pt) const
{
	for (size_t i = 0; i < _vertexIds.size(); i++) {
		size_t j = (i + 1) % _vertexIds.size();
		Edge e(_vertexIds[i], _vertexIds[j]);
		auto seg = e.getSegment(getBlockPtr());
		double t;
		if (seg.contains(pt, t, Tolerance::sameDistTol()))
			return true;
	}
	return false;
}

namespace
{

bool addPairToVerts(MTC::vector<Index3DId>& verts, const MTC::map<Index3DId, Vector3d>& vertModelNormalMap, MTC::map<Index3DId, MTC::vector<Edge>>& vertEdgeMap, 
	MTC::vector<Edge>& edges)
{
	bool addedToList = false;
	for (size_t i = edges.size() - 1; i != -1; i--) {
		Index3DId firstId, lastId, nextId;
		const auto edge = edges[i];
		firstId = verts.front();
		lastId = verts.back();

		if (lastId == edge.getVertex(0) || lastId == edge.getVertex(1)) {
			addedToList = true;
			nextId = edge.getOtherVert(lastId);
			verts.push_back(nextId);
			edges.pop_back();
		} else if (firstId == edge.getVertex(0) || firstId == edge.getVertex(1)) {
			addedToList = true;
			nextId = edge.getOtherVert(firstId);
			verts.push_back(nextId);
			edges.pop_back();
		}

		if (nextId.isValid()) {
			auto iter = vertEdgeMap.find(nextId);
			if (iter != vertEdgeMap.end()) {
				auto& otherEdges = iter->second;
				auto iter2 = std::find(otherEdges.begin(), otherEdges.end(), edge);
				if (iter2 != otherEdges.end())
					otherEdges.erase(iter2);
			}
		}
	}

	return addedToList;
}

}

bool Polygon::verifyVertsConvexStat(const Block* pBlock, const MTC::vector<Index3DId>& vertIds)
{
	for (size_t i = 0; i < vertIds.size(); i++) {
		double angle = calVertexAngleStat(pBlock, vertIds, i);
		if (angle < 0 && angle > M_PI)
			return false;
	}

	return true;
}

bool Polygon::verifyUniqueStat(const MTC::vector<Index3DId>& vertIds)
{
	bool valid = true;

	for (size_t i = 0; i < vertIds.size(); i++) {
		for (size_t j = i + 1; j < vertIds.size(); j++) {
			if (vertIds[i] == vertIds[j])
				valid = false;
		}
	}
	return valid;
}

bool Polygon::verifyTopology() const
{
	bool valid = true;
	if (!verifyUnique())
		valid = false;

	if (valid && _cellIds.size() > 2)
		valid = false;

	if (valid) {
		const auto& edges = getEdges();
		for (const auto& edge : edges) {
			auto faceIds = edge.getFaceIds();
			if (valid && faceIds.count(_thisId) == 0) // edge does not link back to this face
				valid = false;
		}
	}

	if (valid) {
		for (const auto& cellId : _cellIds) {
			if (valid && !getBlockPtr()->polyhedronExists(cellId))
				valid = false;

			cellFunc(cellId, [this, &valid](const Polyhedron& cell) {
				if (valid && !cell.containsFace(_thisId)) {
					valid = false;
				}
				});
			if (!valid)
				return valid;
		}
	}

	if (valid) {
		std::vector<Planed> boundaryPlanes;
		getOurBlockPtr()->getVolume()->getModelBoundaryPlanes(boundaryPlanes);
		auto facePlane = calPlane();
		bool isModelFace = true;
		bool isBoundaryFace = false;

		for (const auto& bPl : boundaryPlanes) {
			if (facePlane.isCoincident(bPl, Tolerance::planeCoincidentDistTol(), Tolerance::planeCoincidentCrossProductTol())) {
				isBoundaryFace = true;
			}
		}
		for (const auto& vertId : _vertexIds) {
			vertexFunc(vertId, [&isModelFace, &isBoundaryFace, &facePlane](const Vertex& vert) {
				if (vert.getLockType() != VLT_MODEL_MESH)
					isModelFace = false;
			});
		}

		if (isModelFace || isBoundaryFace) {
			if (numCells() != 1)
				return false;
		} else {
			if (numCells() != 2)
				return false;
		}
	}

	return valid;
}

ostream& DFHM::operator << (ostream& out, const Polygon& face)
{
#if LOGGING_VERBOSE_ENABLED
	out << "Face: f" << face.getId() << "\n";
	{
		Logger::Indent sp;
		
		Vector3d norm = face.calUnitNormal();
		out << Logger::Pad() << "Normal: (" << norm << ")\n";
		out << Logger::Pad() << "vertexIds: (" << face._vertexIds.size() << "): {\n";
		for (const auto& vertId : face._vertexIds) {
			Logger::Indent sp;
			auto pt = face.getVertexPoint(vertId);
			out << Logger::Pad() << "v" << vertId << ": (" << pt << ")\n";
		}
		out << "}\n";

		out << Logger::Pad() << "cellIds: (" << face._cellIds.size() << "): {";
		for (const auto& cellId : face._cellIds) {
			auto sl = face.getSplitLevel(cellId);
			out << "c" << cellId << ".split: " << sl << " ";
		}
		out << "}\n";

		if (!face._splitIds.empty()) {
			out << Logger::Pad() << "splitFaceIds: (" << face._splitIds.size() << "): {";
			for (const auto& faceId : face._splitIds) {
				out << "f" << faceId << " ";
			}
			out << "}\n";
		}
	}
#else
	out << "Face: f" << face.getId() << "\n";
#endif

	return out;
}

inline Vector3d Polygon::getVertexPoint(const Index3DId& id) const
{
	return getBlockPtr()->getVertexPoint(id);
}


LAMBDA_CLIENT_IMPLS(Polygon)
