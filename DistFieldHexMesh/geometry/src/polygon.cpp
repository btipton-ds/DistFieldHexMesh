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

	Copyright Robert R Tipton, 2022, all rights reserved except those granted in prior license statement.

	Dark Sky Innovative Solutions http://darkskyinnovation.com/
*/

#include <defines.h>
#include <set>
#include <cmath>
#include <iostream>
#include <fstream>
#include <assert.h>
#include <pool_vector.h>
#include <pool_set.h>
#include <tm_math.h>
#include <tm_lineSegment.hpp>
#include <tm_lineSegment_byref.hpp>
#include <tm_spatialSearch.hpp>
#include <tm_ray.h>
#include <tm_ioUtil.h>
#include <tm_bestFit.h>
#include <objectPool.hpp>
#include <vertex.h>
#include <edge.h>
#include <polygon.h>
#include <block.h>
#include <polyMesh.h>
#include <logger.h>
#include <volume.h>
#include <edge.h>
#include <utils.h>
#include <io_utils.h>
#include <tolerances.h>
#include <splitParams.h>
#include <triMeshIndex.h>
#include <meshData.h>
#include <model.h>
#include <splitter2D.h>

#define CACHE_BIT_SORTED 1
#define CACHE_BIT_EDGES 2

using namespace std;
using namespace DFHM;


/*********************************************************************************************************/

Polygon::Polygon(const MTC::vector<Index3DId>& verts)
	: _vertexIds(verts)
{
#if VALIDATION_ON
	assert(verifyUnique());
#endif
}

Polygon::Polygon(const std::initializer_list<Index3DId>& verts)
	: _vertexIds(verts)
{
#if VALIDATION_ON
	assert(verifyUnique());
#endif
}

Polygon::Polygon(const Polygon& src)
	: ObjectPoolOwnerUser(src)
	, PolygonSearchKey(src)
	, _vertexIds(src._vertexIds)
	, _cellIds(src._cellIds)
	, _isConvex(src._isConvex)
{
#if VALIDATION_ON
	assert(verifyUnique());
#endif
}

Polygon::Polygon(const Block* pBlock, const Polygon& src)
	: ObjectPoolOwnerUser(src)
	, PolygonSearchKey(pBlock, src._vertexIds)
	, _vertexIds(src._vertexIds)
	, _cellIds(src._cellIds)
	, _isConvex(src._isConvex)
{
#if VALIDATION_ON
	assert(verifyUnique());
#endif
}

void Polygon::updateObjectKey()
{
	auto pBlk = getOurBlockPtr();
	PolygonSearchKey::set(pBlk, PolygonSearchKey::makeNonColinearVertexIds(pBlk, _vertexIds));
}

void Polygon::connectVertEdgeTopology() {
	if (!getId().isValid())
		return;
	for (size_t i = 0; i < _vertexIds.size(); i++) {
		size_t j = (i + 1) % _vertexIds.size();
		vertexFunc(_vertexIds[i], [this](Vertex& vert) {
			vert.addFaceId(getId());
		});
	}
}

void Polygon::disconnectVertEdgeTopology(bool markVerticesForDeletion) {
	if (!getId().isValid())
		return;

	auto pBlk = getBlockPtr();
	for (size_t i = 0; i < _vertexIds.size(); i++) {
		size_t j = (i + 1) % _vertexIds.size();
		auto& vert = getVertex(_vertexIds[i]);
		vert.removeFaceId(getId());
		if (markVerticesForDeletion && vert.getFaceIds().empty()) {
			pBlk->markVertexForDeletion(vert.getId());
		}
	}
}

DFHM::Polygon& DFHM::Polygon::operator = (const Polygon& rhs)
{
	clearCache();
	disconnectVertEdgeTopology(true);
	auto tmp = _cellIds;
	for (const auto& cellId : tmp) {
		cellFunc(cellId, [this](Polyhedron& cell) {
			cell.removeFace(getId());
		});
	}

	ObjectPoolOwnerUser::operator=(rhs);
	PolygonSearchKey::operator<(rhs);
	_thisId = rhs._thisId;
	_vertexIds = rhs._vertexIds;
	_cellIds = rhs._cellIds;
	_needsGapTest = rhs._needsGapTest;

	_cachedIntersectsModel = rhs._cachedIntersectsModel;

	// DO NOT call connectVertEdgeTopology() here!!! The polygon is not in position yet
	return *this;
}

void Polygon::copyCaches(const Polygon& src)
{
	_isConvex = src._isConvex;
	_cachedIntersectsModel = src._cachedIntersectsModel;
	_cachedArea = src._cachedArea;
	_cachedCentroid = src._cachedCentroid;
	_cachedPlane = src._cachedPlane;
	_cachedNormal = src._cachedNormal;
}

void Polygon::postAddToPoolActions()
{
	if (!_vertexIds.empty()) {
		auto pBlk = getOurBlockPtr();
		if (pBlk) {
			PolygonSearchKey::set(pBlk, _vertexIds);
		} else {
			PolygonSearchKey::set(getPolyMeshPtr(), _vertexIds);
		}
	}

	connectVertEdgeTopology();
}

const Index3DId& Polygon::getId() const
{
	return _thisId;
}

void Polygon::setId(const Index3DId& id)
{
	_thisId = id;
}

void Polygon::remapId(const std::vector<size_t>& idRemap, const Index3D& srcDims)
{
	remap(idRemap, srcDims, _thisId);

	remap(idRemap, srcDims, _vertexIds);
	remap(idRemap, srcDims, _cellIds);
}

void Polygon::addVertex(const Index3DId& vertId)
{
	_vertexIds.push_back(vertId);
#if VALIDATION_ON
	assert(verifyUnique());
#endif
	clearCache();
}

const MTC::vector<Index3DId>& Polygon::getNonColinearVertexIds() const
{
	lock_guard lg(_nonColinearVertexIdsMutex);
	if (_nonColinearVertexIds.empty()) {
		auto p = getOurBlockPtr();
		if (p)
			_nonColinearVertexIds = PolygonSearchKey::makeNonColinearVertexIds(p, _vertexIds);
		else {
			auto p2 = getPolyMeshPtr();
			if (p2)
				_nonColinearVertexIds = PolygonSearchKey::makeNonColinearVertexIds(p2, _vertexIds);
		}
	}
	return _nonColinearVertexIds;
}

void Polygon::clearCache(bool clearSortIds) const
{
	_isConvex = CONVEXITY_UNKNOWN;
	_cachedIntersectsModel = IS_UNKNOWN;
	_cachedArea = -1;
	_cachedCentroid = Vector3d(DBL_MAX, DBL_MAX, DBL_MAX);
	_cachedPlane = Planed(Vector3d(DBL_MAX, DBL_MAX, DBL_MAX), Vector3d(DBL_MAX, DBL_MAX, DBL_MAX), true);
	_cachedNormal = Vector3d(DBL_MAX, DBL_MAX, DBL_MAX);
	_nonColinearVertexIds.clear();

	if (clearSortIds)
		PolygonSearchKey::clear();
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

void Polygon::getConnectedFaceIds(MTC::set<Index3DId>& ids) const
{
	ids.clear();
	size_t num = _vertexIds.size();
	const auto pData = _vertexIds.data();
	for (size_t i = 0; i < num; i++) {
		const auto& vert = getVertex(pData[i]);
		const auto& otherFaceIds = vert.getFaceIds();
		size_t num1 = otherFaceIds.size();
		const auto pData1 = otherFaceIds.data();
		for (size_t j = 0; j < num1; j++) {
			if (pData1[j] != getId())
				ids.insert(pData1[j]);
		}
	}
}

double Polygon::flatten(bool allowQuads)
{
	const auto tolLoose = Tolerance::sameDistTolFloat();
	const auto tolTight = Tolerance::sameDistTol();

	if (!allowQuads && _vertexIds.size() <= 4)
		return false;

	vector<Vector3d> pts;
	for (const auto& vertId : _vertexIds) {
		pts.push_back(getVertexPoint(vertId));
	}

	Planed plane;
	double err;
	if (bestFitPlane(pts, plane, err) && (err > tolTight && (allowQuads || err < tolLoose))) {
		for (const auto& vertId : _vertexIds) {
			auto& vert = getVertex(vertId);
			auto pt = vert.getPoint();
			if (!plane.isCoincident(pt, tolTight)) {
				pt = plane.projectPoint(pt, tolTight);
				assert(plane.isCoincident(pt, tolTight));
				vert.replacePoint(pt);
			}
		}
	}

	return err;
}

void Polygon::reverse()
{
	disconnectVertEdgeTopology(false);
	clearCache();

	std::reverse(_vertexIds.begin(), _vertexIds.end());

	connectVertEdgeTopology();

	// No need to update PolygonSearchKey because it's vertices are noncolinear and sorted by id, not winding order
}

void Polygon::write(ostream& out) const
{
	uint8_t version = 1;
	IoUtil::write(out, version);

	IoUtil::writeEnum(out, _needsGapTest);
	IoUtil::writeObj(out, _vertexIds);
	IoUtil::writeObj(out, _cellIds);
}

void Polygon::read(istream& in)
{
	uint8_t version;
	IoUtil::read(in, version);

	if (version > 0)
		IoUtil::readEnum(in, _needsGapTest);
	else
		_needsGapTest = IS_UNKNOWN;
	IoUtil::readObj(in, _vertexIds);
	IoUtil::readObj(in, _cellIds);
}

bool Polygon::unload(ostream& out, size_t idSelf)
{

	return true;
}

bool Polygon::load(istream& in, size_t idSelf)
{

	return true;
}

bool Polygon::isBlockBoundary() const
{
	if (_cellIds.size() == 2) {
		Index3DId id0 = _cellIds[0];
		Index3DId id1 = _cellIds[1];
		return (id0.blockIdx() != id1.blockIdx());
	}
	return false;
}

MTC::vector<EdgeKey> Polygon::getEdgeKeys() const
{
	MTC::vector<EdgeKey> result;

	for (size_t i = 0; i < _vertexIds.size(); i++) {
		size_t j = (i + 1) % _vertexIds.size();
		result.push_back(EdgeKey(_vertexIds[i], _vertexIds[j]));
	}

	return result;
}

bool Polygon::isPointOnPlane(const Vector3d& pt) const {
	return distFromPlane(pt) < Tolerance::sameDistTol();
}

bool Polygon::usesEdge(const Edge& edgeKey) const
{
	size_t idx0, idx1;
	return usesEdge(edgeKey, idx0, idx1);
}

bool Polygon::usesEdge(const Edge& edgeKey, size_t& idx0, size_t& idx1) const
{
	for (size_t i = 0; i < _vertexIds.size(); i++) {
		size_t j = (i + 1) % _vertexIds.size();
		const auto& vertId0 = _vertexIds[i];
		const auto& vertId1 = _vertexIds[j];
		EdgeKey testEdge(vertId0, vertId1);
		if (testEdge == edgeKey) {
			idx0 = i;
			idx1 = j;
			return true;
		}
	}

	idx0 = idx1 = -1;
	return false;
}

bool Polygon::containsVertex(const Index3DId& vertId) const
{
	size_t num = _vertexIds.size();
	auto pData = _vertexIds.data();
	for (size_t i = 0; i < num; i++) {
		if (pData[i] == vertId)
			return true;
	}
	return false;
}

bool Polygon::containsEdge(const EdgeKey& edge) const
{
	for (size_t i = 0; i < _vertexIds.size(); i++) {
		size_t j = (i + 1) % _vertexIds.size();
		if (EdgeKey(_vertexIds[i], _vertexIds[j]) == edge)
			return true;
	}
	return false;
}

bool Polygon::isCoplanar(const Vector3d& pt) const
{
	Planed pl = calPlane();
	return pl.distanceToPoint(pt) < Tolerance::sameDistTol();
}

bool Polygon::isCoplanar(const Planed& pl) const
{
	Planed ourPlane = calPlane();
	return ourPlane.isCoincident(pl, Tolerance::planeCoincidentDistTol(), Tolerance::planeCoincidentCrossProductTol());
}

bool Polygon::isCoplanar(const EdgeKey& edgeKey) const
{
	Planed pl = calPlane();
	const auto pt0 = getVertexPoint(edgeKey[0]);
	if (!pl.isCoincident(pt0, Tolerance::sameDistTol()))
		return false;

	const auto pt1 = getVertexPoint(edgeKey[1]);
	if (!pl.isCoincident(pt1, Tolerance::sameDistTol()))
		return false;

	return true;
}

Vector3d Polygon::calCentroidApproxStat(const Block* pBlock, const MTC::vector<Index3DId>& vertIds)
{
	Vector3d ctr(0, 0, 0);
	if (vertIds.empty())
		return Vector3d(DBL_MAX, DBL_MAX, DBL_MAX);
	for (size_t i = 0; i < vertIds.size(); i++)
		ctr += pBlock->getVertexPoint(vertIds[i]);
	ctr /= vertIds.size();

	return ctr;
}

Vector3d Polygon::calCentroidApproxStat(const PolyMesh* pPolyMesh, const MTC::vector<Index3DId>& vertIds)
{
	Vector3d ctr(0, 0, 0);
	if (vertIds.empty())
		return Vector3d(DBL_MAX, DBL_MAX, DBL_MAX);
	for (size_t i = 0; i < vertIds.size(); i++)
		ctr += pPolyMesh->getVertexPoint(vertIds[i]);
	ctr /= vertIds.size();

	return ctr;
}

void Polygon::calCoordSysStat(const Block* pBlock, const MTC::vector<Index3DId>& vertIds, Vector3d& origin, Vector3d& xAxis, Vector3d& yAxis, Vector3d& zAxis)
{
	origin = calCentroidApproxStat(pBlock, vertIds);
	if (!Polygon::calUnitNormalStat(pBlock, vertIds, zAxis))
		throw runtime_error("calUnitNormalStat failed");

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
	Vector3d zAxis;
	if (!calUnitNormalStat(pBlock, vertIds, zAxis)) {
		cVertIds.insert(vertIds.begin(), vertIds.end());
		return;
	}

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

void Polygon::findConcaveVertIdsStat(const PolyMesh* pPolyMesh, const MTC::vector<Index3DId>& vertIds, MTC::set<Index3DId>& cVertIds)
{
	Vector3d zAxis;
	if (!calUnitNormalStat(pPolyMesh, vertIds, zAxis)) {
		cVertIds.insert(vertIds.begin(), vertIds.end());
		return;
	}

	for (size_t i = 0; i < vertIds.size(); i++) {
		size_t j = (i + 1) % vertIds.size();
		size_t k = (j + 1) % vertIds.size();

		auto pt0 = pPolyMesh->getVertexPoint(vertIds[i]);
		auto pt1 = pPolyMesh->getVertexPoint(vertIds[j]);
		auto pt2 = pPolyMesh->getVertexPoint(vertIds[k]);

		Vector3d v0 = pt0 - pt1;
		Vector3d v1 = pt2 - pt1;
		Vector3d vN = v1.cross(v0);
		if (vN.dot(zAxis) < 0) {
			// Concave vertex
			cVertIds.insert(vertIds[j]);
		}
	}
}

bool Polygon::calUnitNormalStat(const Block* pBlock, const MTC::vector<Index3DId>& vertIds, Vector3d& norm)
{
	norm = Vector3d(0, 0, 0);
	Vector3d lastNorm(DBL_MAX, DBL_MAX, DBL_MAX);

	double maxNorm = 0;
	size_t i = 0;
	Vector3d pt0 = pBlock->getVertexPoint(vertIds[i]);
	for (size_t j = 1; j < vertIds.size() - 1; j++) {
		size_t k = (j + 1) % vertIds.size();

		Vector3d pt1 = pBlock->getVertexPoint(vertIds[j]);
		Vector3d pt2 = pBlock->getVertexPoint(vertIds[k]);

		Vector3d v0 = pt0 - pt1;
		Vector3d v1 = pt2 - pt1;

		Vector3d n = v1.cross(v0);
#ifdef _DEBUG
		if (j > 1) {
			auto dp = n.dot(lastNorm);
			assert(dp > -Tolerance::paramTol());
		}
		lastNorm = n;
#endif // _DEBUG

		double l = n.norm();
		if (l > maxNorm) {
			maxNorm = l;
			n /= l;
			norm = n;
		}
	}

	if (norm.isNAN())
		return false;
	return true;
}

bool Polygon::calUnitNormalStat(const PolyMesh* pPolyMesh, const MTC::vector<Index3DId>& vertIds, Vector3d& norm)
{
	norm = Vector3d(0, 0, 0);

	size_t i = 0;
	Vector3d pt0 = pPolyMesh->getVertexPoint(vertIds[i]);
	for (size_t j = 1; j < vertIds.size() - 1; j++) {
		size_t k = (j + 1) % vertIds.size();

		const auto& pt1 = pPolyMesh->getVertexPoint(vertIds[j]);
		const auto& pt2 = pPolyMesh->getVertexPoint(vertIds[k]);

		Vector3d v0 = pt0 - pt1;
		Vector3d v1 = pt2 - pt1;

		Vector3d n = v1.cross(v0);
#if 1 && defined(_DEBUG)
		if (norm.squaredNorm() > 0) {
			auto dp = n.dot(norm);
			if (dp < 0)
				return false;
		}
#endif
		norm += n;
	}

	norm.normalize();

	if (norm.isNAN())
		return false;
	return true;
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

const Vector3d& Polygon::calUnitNormal() const
{
	assert(!_cachedNormal.isNAN());
	if (_cachedNormal[0] == DBL_MAX) {
		auto& nonColin = getNonColinearVertexIds();
		auto p = getBlockPtr();
		Vector3d tmp;
		if (p) {
			if (!calUnitNormalStat(p, nonColin, tmp))
				throw runtime_error("calUnitNormal() failed");
		} else {
			if (!calUnitNormalStat(getPolyMeshPtr(), nonColin, tmp))
				throw runtime_error("calUnitNormal() failed");
		}
#if _DEBUG
		auto err = tmp.squaredNorm() - 1.0;
		if (fabs(err) > 1.0e-12) {
			assert(!"bad squaredNorm");
		}
#endif
		assert(!tmp.isNAN());
		_cachedNormal = tmp;
	}
	return _cachedNormal;
}

void Polygon::setUnitNormal_risky(const Vector3d& val)
{
	_cachedNormal = val;
	assert(!_cachedNormal.isNAN());
}

Vector3d Polygon::calOrientedUnitNormal(const Index3DId& cellId) const
{
	auto pBlk = getBlockPtr();
	Vector3d result = calUnitNormal();
	auto& nonColin = getNonColinearVertexIds();
	auto faceCtr = calCentroidApproxStat(pBlk, nonColin);
	auto& cell = getPolyhedron(cellId);
	Vector3d cellApproxCtr = cell.calCentroid();
	Vector3d v = faceCtr - cellApproxCtr;
	if (v.dot(result) < 0)
		return -result;

	return result;
}

const Planed& Polygon::calPlane() const
{
	if (_cachedPlane.getOrigin()[0] == DBL_MAX) {
		auto& origin = calCentroid(); // Use every point to get more preceision
		assert(!origin.isNAN());
		auto& normal = calUnitNormal();
		assert(!normal.isNAN());
		_cachedPlane = Planed(origin, normal, true);
	}

	return _cachedPlane;
}

Planed Polygon::calOrientedPlane(const Index3DId& cellId) const
{
	Vector3d origin = calCentroid(); // Use every point to get more preceision
	Vector3d normal = calOrientedUnitNormal(cellId);
	Planed result(origin, normal);

#if VALIDATION_ON && defined(_DEBUG)
	for (const auto& vId : _vertexIds) {
		Vector3d pt = getVertexPoint(vId);
		assert(result.distanceToPoint(pt) < Tolerance::sameDistTol());
	}
#endif // _DEBUG

	return result;
}

bool Polygon::isReversed(const Index3DId& cellId) const
{
	auto& cell = getPolyhedron(cellId);
	Planed opl;
	cell.calOrientatedPlane(getId(), opl);
	auto& pl = calPlane();
	auto result = pl.getNormal().dot(opl.getNormal());
	return result < 0;
}

double Polygon::calVertexAngleStat(const Block* pBlock, const MTC::vector<Index3DId>& vertIds, const Index3DId& vertId)
{
	size_t idx1 = -1;
	for (size_t i = 0; i < vertIds.size(); i++) {
		if (vertIds[i] == vertId) {
			idx1 = i;
			break;
		}
	}
	return calVertexAngleStat(pBlock, vertIds, idx1);
}

double Polygon::calVertexAngleStat(const Block* pBlock, const MTC::vector<Index3DId>& vertIds, size_t idx1)
{
	const size_t sz = vertIds.size();
	if (idx1 < sz) {
		Vector3d norm;
		if (!calUnitNormalStat(pBlock, vertIds, norm)) {
			return DBL_MAX;
		}
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

double Polygon::calVertexAngleStat(const PolyMesh* pMesh, const MTC::vector<Index3DId>& vertIds, const Index3DId& vertId)
{
	size_t idx1 = -1;
	for (size_t i = 0; i < vertIds.size(); i++) {
		if (vertIds[i] == vertId) {
			idx1 = i;
			break;
		}
	}
	return calVertexAngleStat(pMesh, vertIds, idx1);
}

double Polygon::calVertexAngleStat(const PolyMesh* pMesh, const MTC::vector<Index3DId>& vertIds, size_t idx1)
{
	const size_t sz = vertIds.size();
	if (idx1 < sz) {
		Vector3d norm;
		if (!calUnitNormalStat(pMesh, vertIds, norm)) {
			return DBL_MAX;
		}
		size_t idx0 = (idx1 + sz - 1) % sz;
		size_t idx2 = (idx1 + 1) % sz;

		Vector3d pt0 = pMesh->getVertexPoint(vertIds[idx0]);
		Vector3d pt1 = pMesh->getVertexPoint(vertIds[idx1]);
		Vector3d pt2 = pMesh->getVertexPoint(vertIds[idx2]);

		Vector3d v0 = pt0 - pt1;
		Vector3d v1 = pt2 - pt1;

		double dp = v1.dot(v0);
		double cp = v1.cross(v0).dot(norm);
		double angle = atan2(cp, dp);
		return angle;
	}

	return nanf("");
}

double Polygon::calVertexError(const vector<Index3DId>& testVertIds) const
{
	if (testVertIds.size() != _vertexIds.size())
		return DBL_MAX;

	double avgErr = 0;
	for (const auto& faceVertId : _vertexIds) {
		const auto& facePt = getVertexPoint(faceVertId);
		double minErr = DBL_MAX;
		for (const auto& testId : testVertIds) {
			const auto& testPt = getVertexPoint(testId);
			double err = (testPt - facePt).norm();
			if (err < minErr) {
				minErr = err;
				if (minErr < Tolerance::sameDistTol())
					break;
			}
		}
		avgErr += minErr;
	}

	avgErr /= _vertexIds.size();

	return avgErr;
}

double Polygon::calVertexAngle(const Index3DId& vertId) const
{
	auto p = getBlockPtr();
	if (p)
		return calVertexAngleStat(p, _vertexIds, vertId);

	return calVertexAngleStat(getPolyMeshPtr(), _vertexIds, vertId);
}

const Vector3d& Polygon::calCentroid() const
{
	if (_cachedCentroid[0] == DBL_MAX) {
		double area;
		calAreaAndCentroid(area, _cachedCentroid);
	}
	return _cachedCentroid;
}

void Polygon::setCentroid_risky(const Vector3d& val)
{
	_cachedCentroid = val;
}

void Polygon::setIsConvex_risky(Convexity convexity)
{
	_isConvex = convexity;
}

double Polygon::distFromPlane(const Vector3d& pt) const
{
	auto& pl = calPlane();
	return pl.distanceToPoint(pt);
}

double Polygon::minDistToPoint(const Vector3d& pt, Vector3d& closestPt) const
{
	double dist = DBL_MAX;
	auto& pl = calPlane();
	auto projPt = pl.projectPoint(pt, Tolerance::sameDistTol());
	if (isPointInside(projPt)) {
		closestPt = pt;
		auto v = closestPt - pl.getOrigin();
		dist = fabs(pl.getNormal().dot(v)); 
	} else {
		iterateTrianglePts([&pt, &closestPt, &dist](const Vector3d& pt0, const Vector3d& pt1, const Vector3d& pt2 )->bool {
			const Vector3d* pts[] = { &pt0, &pt1, &pt2 };
			for (int i = 0; i < 3; i++) {
				int j = (i + 1) % 3;
				LineSegmentd seg(*pts[i], *pts[j]);
				double t;
				Vector3d hitPt;
				double minSegDist = seg.distanceToPoint(pt, hitPt, t);
				if (minSegDist > 0 && minSegDist < dist) {
					dist = minSegDist;
					closestPt = hitPt;
				}
			}
			return true;
		});
	}
	return dist;
}

void Polygon::calAreaAndCentroid(double& area, Vector3d& centroid) const
{
	if (_cachedArea > 0 && _cachedCentroid[0] != DBL_MAX) {
		area = _cachedArea;
		centroid = _cachedCentroid;
		return;
	}
	area = 0;
	centroid = Vector3d(0, 0, 0);
	auto& nonColinVerts = getNonColinearVertexIds();
	Vector3d pt0 = getVertexPoint(nonColinVerts[0]);
	for (size_t j = 1; j < nonColinVerts.size() - 1; j++) {
		size_t k = j + 1;
		Vector3d pt1 = getVertexPoint(nonColinVerts[j]);
		Vector3d pt2 = getVertexPoint(nonColinVerts[k]);
		Vector3d v0 = pt0 - pt1;
		Vector3d v1 = pt2 - pt1;

		// This was confirmed with a second, geometric calculation with gave the same result.
		Vector3d triCtr = (pt0 + pt1 + pt2) * (1.0 / 3.0);

		Vector3d cp = v1.cross(v0);
		double triArea = cp.norm() / 2.0;

		centroid += triCtr * triArea;
		area += triArea;
	}

	centroid /= area;

	_cachedArea = area;
	if (_cachedCentroid[0] == DBL_MAX)
		_cachedCentroid = centroid;
}

Vector2d Polygon::projectPoint2d(const Vector3d& pt, const Planed& pl) const
{
	auto& origin = pl.getOrigin();
	auto& xAxis = pl.getXRef();
	auto& zAxis = pl.getNormal();
	Vector3d yAxis = zAxis.cross(xAxis);

	Vector3d v = pt - origin;
	Vector2d result(v.dot(xAxis), v.dot(yAxis));

	return result;
}

void Polygon::removeCellId(const Index3DId& cellId)
{
	_cellIds.erase(cellId);
}

void Polygon::addCellId(const Index3DId& cellId)
{
#if DEBUG_BREAKS && defined(_DEBUG)
	if (Index3DId(4, 6, 1, 0) == cellId) {
		int dbgBreak = 1;
	}
#endif

	_cellIds.insert(cellId);
#if VALIDATION_ON && defined(_DEBUG)
	if (_cellIds.size() > 2) {
		for (const auto& cellId1 : _cellIds) {
			assert(getBlockPtr()->polyhedronExists(cellId1));
			cellFunc(cellId1, [this](const Polyhedron& cell) {
				assert(cell.containsFace(getId()));
			});
		}
		if (_cellIds.size() > 2) {
			int i = 0;
			auto iter = _cellIds.begin();
			for (size_t i = 0; i < _cellIds.size(); i++) {
				stringstream ss;
				ss << "D:/DarkSky/Projects/output/objs/tooManyCellIds_cell_" << i << ".obj";
				getBlockPtr()->getVolume()->writeObj(ss.str().c_str(), {*iter}, false, false, false);
				iter++;
			}
			getBlockPtr()->getVolume()->writeObj("D:/DarkSky/Projects/output/objs/tooManyCellIds_face.obj", { _vertexIds });
			assert(!"Too many cell ids");
		}
	}
#endif
}

bool Polygon::imprintFace(const Index3DId& faceId)
{
	bool result = false;

	const auto& face = getPolygon(faceId);
	const auto& vertIds = face.getVertexIds();
	for (size_t i = 0; i < vertIds.size(); i++) {
		size_t j = (i + 1) % vertIds.size();
		EdgeKey otherEk(vertIds[i], vertIds[j]);
		result = imprintEdge(otherEk) && result;
	}

	return result;
}

bool Polygon::imprintEdge(const EdgeKey& edgeKey)
{
	if (containsEdge(edgeKey))
		return false; // we've already go that edge.

	MTC::vector<Index3DId> vertIds = { edgeKey[0], edgeKey[1] };

	// Include non planar intersections where other edges intersect the face plane.
	edgeFunc(edgeKey, [this, &vertIds](const Edge& edge) {
		auto seg = edge.getSegment();
		RayHitd hp;
		if (intersect(seg, hp)) {
			vertIds.push_back(getBlockPtr()->getVertexIdOfPoint(hp.hitPt));
		}
	});

	if (imprintVerts(vertIds))
		return true;

	return false;
}

bool Polygon::imprintFaces(const FastBisectionSet<Index3DId>& faceIds)
{
	bool result = false;
	for (const auto& id : faceIds) {
		result = imprintFace(id) && result;
	}

	return result;
}

bool Polygon::imprintVert(const Index3DId& vertId)
{
	const double tol = Tolerance::sameDistTol();

	MTC::vector<Index3DId> tmp;
	tmp.reserve(_vertexIds.size() + 1);

	for (size_t i = 0; i < _vertexIds.size(); i++) {
		size_t j = (i + 1) % _vertexIds.size();
		tmp.push_back(_vertexIds[i]);

		const auto& pt0 = getVertexPoint(_vertexIds[i]);
		const auto& pt1 = getVertexPoint(_vertexIds[j]);

		LineSegment_byrefd seg(pt0, pt1);
		double t;
		const auto& pt = getVertexPoint(vertId);
		bool contains = seg.contains(pt, t, tol) && tol < t && t < 1 - tol;
		if (contains) {
			tmp.push_back(vertId);
		}
	}
	

	if (tmp.size() > _vertexIds.size()) {
		disconnectVertEdgeTopology(false);
		clearCache(false);

#if VALIDATION_ON		
		assert(verifyUniqueStat(tmp));
#endif
		_vertexIds = tmp;

		connectVertEdgeTopology();

		// TODO This should be moved to only where it's needed
		for (const auto& id : _cellIds) {
			getBlockPtr()->addToTouchedCellList(id);
		}
		return true;
	}

	return false;
}

bool Polygon::imprintVerts(const vector<Index3DId>& vertIds)
{
	bool result = false;
	for (const auto& vertId : vertIds) {
		result = imprintVert(vertId) || result;
	}

	return result;
}

bool Polygon::isPlanar() const
{
	if (_vertexIds.size() == 3)
		return true;

	Planed pl = calPlane();
	for (size_t i = 0; i < _vertexIds.size(); i++) {
		auto& pt = getVertexPoint(_vertexIds[i]);
		if (pl.distanceToPoint(pt) > Tolerance::sameDistTol())
			return false;
	}

	return true;
}

bool Polygon::intersect(const Rayd& ray, RayHitd& hit) const
{
	if (isConvex() == Convexity::IS_CONVEX) {
		auto& plane = calPlane();
		if (plane.intersectRay(ray, hit, Tolerance::sameDistTol())) {
			return isPointInsideInner(hit.hitPt, plane);
		}
	} else {
		auto& vertIds = getNonColinearVertexIds();
		vector<Vector3d> pts;
		pts.resize(vertIds.size());
		for (size_t i = 0; i < vertIds.size(); i++)
			pts[i] = getVertexPoint(vertIds[i]);

		auto& pt0 = pts[0];
		for (size_t i = 1; i < vertIds.size() - 1; i++) {
			size_t j = (i + 1);
			auto& pt1 = pts[i];
			auto& pt2 = pts[j];
			if (intersectRayTri(ray, pt0, pt1, pt2, hit, Tolerance::sameDistTol()))
				return true;
		}
	}
	return false;
}

bool Polygon::intersect(const LineSegmentd& seg, RayHitd& hit) const
{
	if (isConvex() == Convexity::IS_CONVEX) {
		auto& pl = calPlane();
		if (pl.intersectLineSegment(seg, hit, Tolerance::sameDistTol())) {
			if (isPointInsideInner(hit.hitPt, pl.getNormal()))
				return true;
		}
	} else {
		auto& vertIds = getNonColinearVertexIds();
		vector<Vector3d> pts;
		pts.resize(vertIds.size());
		for (size_t i = 0; i < vertIds.size(); i++)
			pts[i] = getVertexPoint(vertIds[i]);

		auto& pt0 = pts[0];
		for (size_t i = 1; i < vertIds.size() - 1; i++) {
			size_t j = (i + 1);
			auto& pt1 = pts[i];
			auto& pt2 = pts[j];
			if (seg.intersectTri(pt0, pt1, pt2, hit, Tolerance::sameDistTol()))
				return true;
		}
	}
	return false;
}

bool Polygon::intersect(const LineSegment_byrefd& seg, RayHitd& hit) const
{
	if (isConvex() == Convexity::IS_CONVEX) {
		auto& pl = calPlane();
		if (pl.intersectLineSegment(seg, hit, Tolerance::sameDistTol())) {
			if (isPointInsideInner(hit.hitPt, pl.getNormal()))
				return true;
		}
	} else {
		auto& vertIds = getNonColinearVertexIds();
		vector<Vector3d> pts;
		pts.resize(vertIds.size());
		for (size_t i = 0; i < vertIds.size(); i++)
			pts[i] = getVertexPoint(vertIds[i]);

		auto& pt0 = pts[0];
		for (size_t i = 1; i < vertIds.size() - 1; i++) {
			size_t j = (i + 1);
			auto& pt1 = pts[i];
			auto& pt2 = pts[j];
			if (seg.intersectTri(pt0, pt1, pt2, hit, Tolerance::sameDistTol()))
				return true;
		}
	}
	return false;
}

bool Polygon::intersect(const Planed& pl, LineSegmentd& intersectionSeg) const
{
	// This collapses duplicate corner hits to a single hit
	MTC::set<Vector3d> intersectionPoints;
	for (size_t i = 0; i < _vertexIds.size(); i++) {
		size_t j = (i + 1) % _vertexIds.size();
		const Vector3d& pt0 = getVertexPoint(_vertexIds[i]);
		const Vector3d& pt1 = getVertexPoint(_vertexIds[j]);

		LineSegmentd edgeSeg(pt0, pt1);;
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
		assert(!"Unexpected case");
		int dbgBreak = 1;
	}

	return false;
}

bool Polygon::intersect(const Polygon& otherFace, bool dumpObj) const
{
	const auto tol = Tolerance::paramTol();

	// Refactor to use Splitter2d instead of polygon AND use the splitters planes for the plane/plane intersection.
	// This will prevent minor deviations between the polygon planes and the splitter planes which is dropping projections.

	Splitter2D sp(*this), sp2(otherFace);

	if (dumpObj) {
		sp.writeBoundaryEdgesObj("D:/DarkSky/Projects/output/objs/meshFace.obj");
		sp2.writeBoundaryEdgesObj("D:/DarkSky/Projects/output/objs/modelFace.obj");
	}
	auto& ourPlane = sp.getPlane();
	auto& otherPlane = sp2.getPlane();
	Rayd ray;
	if (ourPlane.intersectPlane(otherPlane, ray, tol)) {
		vector<LineSegmentd> segs;
		if (sp.intersectWithRay(ray, segs)) {
			const auto& pt0 = ray._origin;
			const auto& pt1 = pt0 + ray._dir;

			assert(ourPlane.isCoincident(pt0, tol));
			assert(ourPlane.isCoincident(pt1, tol));
			assert(otherPlane.isCoincident(pt0, tol));
			assert(otherPlane.isCoincident(pt1, tol));

			for (const auto& seg : segs) {
				assert(ourPlane.isCoincident(seg._pt0, tol));
				assert(ourPlane.isCoincident(seg._pt1, tol));
				assert(otherPlane.isCoincident(seg._pt0, tol));
				assert(otherPlane.isCoincident(seg._pt1, tol));

				if (sp2.intersectWithSeg(seg)) {
					return true;
				}
			}
		}
	}
	return false;
}

namespace
{
	bool intersectPlaneTri(const Planed& plane, const LineSegment_byrefd segs[3], LineSegmentd& iSeg, double tol)
	{

		int numHits = 0;

		Vector3d iPts[2];
		RayHitd hit;

		for (int i = 0; i < 3; i++) {
			if (plane.intersectLineSegment(segs[i], hit, tol)) {
				iPts[numHits++] = hit.hitPt;
				if (numHits == 2) {
					iSeg = LineSegmentd(iPts[0], iPts[1]);
					return true;
				}
			}
		}

		return false;
	}

	inline bool segsOverlap(const LineSegmentd segs[2], double tol, double paramTol)
	{
		for (int i = 0; i < 2; i++) {
			const auto& segBase = segs[i];
			const auto& segTest = segs[1 - i];
			Vector3d vBase = segBase._pt1 - segBase._pt0;
			double lenBase = vBase.norm();
			if (lenBase > paramTol) {
				vBase /= lenBase;
				for (int j = 0; j < 2; j++) {
					const auto& pt = (j == 0) ? segTest._pt0 : segTest._pt1;
					Vector3d v = pt - segBase._pt0;
					double dist = vBase.dot(v);
					if (-tol < dist && dist < lenBase + tol) {
						return true;
					}
				}
			}
		}

		return false;
	}
}

void Polygon::intersectHexMeshTris(size_t numTris, const pair<const Vector3d*, const Polygon*>* pMeshTriData, Trinary& result) const
{
	const auto& verts = getNonColinearVertexIds();
	if (verts.size() < 3) {
		throw (std::runtime_error("Less than three vertices"));
	}

	std::vector<const Vector3d*> pts;
	pts.resize(verts.size());
	for (size_t i = 0; i < verts.size(); i++)
		pts[i] = &getVertexPoint(verts[i]);

	size_t i = 0;
	for (size_t j = 1; j < pts.size() - 1; j++) {
		size_t k = (j + 1) % pts.size();
		const auto& pt0 = *pts[i];
		const auto& pt1 = *pts[j];
		const auto& pt2 = *pts[k];

		const auto tol = Tolerance::sameDistTol();
		const Vector3d* modelTriPts[] = { &pt0, &pt1, &pt2 };
		Planed modelTriPlane(modelTriPts, false);
		const LineSegment_byrefd modelTriSegs[] = {
			LineSegment_byrefd(pt0, pt1),
			LineSegment_byrefd(pt1, pt2),
			LineSegment_byrefd(pt2, pt0),
		};


		for (size_t i = 0; i < numTris; i++) {
			size_t triIdx = 3 * i;
			const Vector3d* meshTriPts[] = {
				pMeshTriData[triIdx + 0].first,
				pMeshTriData[triIdx + 1].first,
				pMeshTriData[triIdx + 2].first,
			};

			const LineSegment_byrefd meshTriSegs[] = {
				LineSegment_byrefd(*meshTriPts[0], *meshTriPts[1]),
				LineSegment_byrefd(*meshTriPts[1], *meshTriPts[2]),
				LineSegment_byrefd(*meshTriPts[2], *meshTriPts[0]),
			};

			LineSegmentd iSeg[2];
			if (!intersectPlaneTri(modelTriPlane, meshTriSegs, iSeg[0], tol))
				continue;

			Planed meshTriPlane(meshTriPts, false);

			if (!intersectPlaneTri(meshTriPlane, modelTriSegs, iSeg[1], tol))
				continue;

			if (segsOverlap(iSeg, tol, Tolerance::divideByZeroTol())) {
				result = IS_TRUE;
				auto pFace = pMeshTriData[triIdx].second;
				if (pFace)
					pFace->setIntersectsModel(IS_TRUE);
				break;
			}
		}
	}
}

bool Polygon::isPointInside(const Vector3d& pt) const
{
	bool result;
	Vector3d norm = calUnitNormal();
	result = isPointInsideInner(pt, norm);

	return result;
}

bool Polygon::isPointInside(const Vector3d& pt, const Vector3d& norm) const
{
	bool result;
	result = isPointInsideInner(pt, norm);

	return result;
}

bool Polygon::isConnected(const Polygon& otherFace) const
{
	// Extremely slow function in debug mode due to stl iterator debugging
	size_t num = _vertexIds.size();
	auto pData = _vertexIds.data();
	for (size_t i = 0; i < num; i++) {
		if (otherFace.containsVertex(pData[i]))
			return true;
	}

	return false;
}

bool Polygon::isPointInsideInner(const Vector3d& pt, const Vector3d& norm) const
{
	auto& pt0 = getVertexPoint(_vertexIds[0]);
	Planed pl(pt0, norm);
	return isPointInsideInner(pt, pl);
}

bool Polygon::isPointInsideInner(const Vector3d& pt, const Planed& pl) const
{
	const double tol = Tolerance::sameDistTol();

	if (!pl.isCoincident(pt, tol))
		return false;

	auto& nclinVerts = getNonColinearVertexIds();
	vector<Vector2d> pts2D;
	pts2D.resize(nclinVerts.size());
	for (size_t i = 0; i < nclinVerts.size(); i++) {
		pts2D[i] = projectPoint2d(getVertexPoint(nclinVerts[i]), pl);
	}

	Vector2d pt2d = projectPoint2d(pt, pl);
	LineSegment2d ray(pt2d, pt2d + Vector2d(1, 0));
	size_t posCount = 0, negCount = 0;
	for (size_t i = 0; i < pts2D.size(); i++) {
		size_t j = (i + 1) % nclinVerts.size();
		const auto& pt0 = pts2D[i];
		const auto& pt1 = pts2D[j];
		Vector2d vLeg = pt1 - pt0;
		Vector2d vPerp(-vLeg[1], vLeg[0]);
		vPerp.normalize();

		Vector2d v = pt2d - pt0;
		double dist = vPerp.dot(v);
		if (dist < tol)
			negCount++;

		if (dist > -tol)
			posCount++;
	}

	return negCount == pts2D.size() || posCount == pts2D.size();
}

Trinary Polygon::isInsideSolid(const std::shared_ptr<const PolyMeshSearchTree>& pSearchTree) const
{
	if (!pSearchTree)
		return IS_UNKNOWN;

	auto& model = getOurBlockPtr()->getModel();
	std::set<size_t> outside, inside;
	for (size_t i =0; i < _vertexIds.size(); i++) {
		size_t j = (i + 1) % _vertexIds.size();
		auto& pt0 = getVertexPoint(_vertexIds[i]);
		auto& pt1 = getVertexPoint(_vertexIds[j]);
		LineSegment_byrefd seg(pt0, pt1);

		CBoundingBox3Dd bbox;
		bbox.merge(pt0);
		bbox.merge(pt1);
		bbox.grow(Tolerance::sameDistTol());

		pSearchTree->traverse(bbox, [this, &seg, &model, i, j, &outside, &inside](const PolyMeshIndex& polyIdx)->bool {
			const auto tol = Tolerance::sameDistTol();
			if (model.isClosed(polyIdx)) {
				auto pFace = model.getPolygon(polyIdx);
				RayHitd hp;
				if (pFace && pFace->intersect(seg, hp)) {
					auto& vert0 = getVertex(_vertexIds[i]);
					auto& vert1 = getVertex(_vertexIds[j]);
					auto& pl = pFace->calPlane();

					// Only test pt0 so we only test each vertex once.
					auto dist0 = pl.distanceToPoint(seg._pt0, false);
					if (dist0 > tol) {
						outside.insert(i);
					} else if (dist0 < -tol) {
						inside.insert(i);
					}

					auto dist1 = pl.distanceToPoint(seg._pt1, false);
					if (dist1 > tol) {
						outside.insert(j);
					} else if (dist1 < -tol) {
						inside.insert(j);
					}
				}
			}
			return true;
		});
	}

	if (!outside.empty())
		return IS_FALSE; // Outside or crossing solid boundary

	if (!inside.empty())
		return IS_TRUE; // Has no outside or crossing and at least one inside

	return IS_UNKNOWN;
}

bool Polygon::isCoincident(const Planed& plane) const
{
	return isCoincident(plane, Tolerance::sameDistTol());
}

bool Polygon::isCoincident(const Planed& plane, double sameDistTol) const
{
	/*
	Plane vs plane coincidence is error prone.
	This uses linear distance from each vertex to the plane within a distance tolerance which is more stable.
	*/
	if (sameDistTol < 0)
		sameDistTol = Tolerance::sameDistTol();
	for (const auto& id : _vertexIds) {
		const auto& pt = getVertexPoint(id);
		auto dist = plane.distanceToPoint(pt);
		if (dist > sameDistTol)
			return false;
	}

	return true;
}

void Polygon::setOnSymmetryPlane(const std::vector<Planed>& symPlanes, double tol)
{
	if (_isOnSymmetryPlane == IS_UNKNOWN) {
		_isOnSymmetryPlane = IS_FALSE;
		for (const auto& pl : symPlanes) {
			if (isCoincident(pl, tol)) {
				_isOnSymmetryPlane = IS_TRUE;
				break;
			}
		}
	}
}

bool Polygon::isOnSymmetryPlane() const
{
	return _isOnSymmetryPlane == IS_TRUE;
}

bool Polygon::isPointOnEdge(const Vector3d& pt) const
{
	for (size_t i = 0; i < _vertexIds.size(); i++) {
		size_t j = (i + 1) % _vertexIds.size();
		const Vector3d& pt0 = getVertexPoint(_vertexIds[i]);
		const Vector3d& pt1 = getVertexPoint(_vertexIds[j]);

		LineSegmentd seg(pt0, pt1);;
		double t;
		if (seg.contains(pt, t, Tolerance::sameDistTol()))
			return true;
	}
	return false;
}

namespace
{

bool addPairToVerts(MTC::vector<Index3DId>& verts, const MTC::map<Index3DId, Vector3d>& vertModelNormalMap, MTC::map<Index3DId, MTC::vector<EdgeKey>>& vertEdgeMap, 
	MTC::vector<EdgeKey>& edges)
{
	bool addedToList = false;
	for (size_t i = edges.size() - 1; i != -1; i--) {
		Index3DId firstId, lastId, nextId;
		const auto edge = edges[i];
		firstId = verts.front();
		lastId = verts.back();

		if (lastId == edge[0] || lastId == edge[1]) {
			addedToList = true;
			nextId = edge.getOtherVert(lastId);
			verts.push_back(nextId);
			edges.pop_back();
		} else if (firstId == edge[0] || firstId == edge[1]) {
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
	for (size_t i = 0; i < vertIds.size(); i++) {
		for (size_t j = i + 1; j < vertIds.size(); j++) {
			if (vertIds[i] == vertIds[j])
				return false;
		}
	}
	return true;
}

CBoundingBox3Dd Polygon::getBBox() const
{
	CBoundingBox3Dd result;
	for (const auto& id : _vertexIds) {
		result.merge(getVertexPoint(id));
	}
	result.grow(bboxOffsetDist());
	return result;
}

int64_t Polygon::getLayerNum() const
{
	// Get the layer number of the lowest layer numbered cell.
	int64_t layerNum = -1;
	for (const auto& cellId : _cellIds) {
		cellFunc(cellId, [&layerNum](const Polyhedron& cell) {
			int64_t cellLayerNum = cell.getLayerNum();
			if (cellLayerNum != -1 && (layerNum == -1 || cellLayerNum < layerNum))
				layerNum = cellLayerNum;			
		});
	}

	return layerNum;
}

bool Polygon::verifyTopology() const
{
	bool valid = true;
	if (!verifyUnique())
		valid = false;

	if (valid && _cellIds.size() > 2)
		valid = false;

	if (valid) {
		auto edgeKeys = getEdgeKeys();
		for (const auto& edgeKey : edgeKeys) {
			edgeFunc(edgeKey, [this, &valid](const Edge& edge) {
				if (!edge.containsFace(getId()))
					valid = false;
				auto& faceIds = edge.getFaceIds();
				if (valid && !faceIds.contains(getId())) // edge does not link back to this face
					valid = false;
			});
		}
	}

	if (valid) {
		for (const auto& cellId : _cellIds) {
			if (valid && !getBlockPtr()->polyhedronExists(cellId))
				valid = false;

			cellFunc(cellId, [this, &valid](const Polyhedron& cell) {
				if (valid && !cell.containsFace(getId())) {
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
			if (isCoincident(bPl, Tolerance::sameDistTol())) {
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
#if 0
			if (numCells() != 2)
				return false;
#endif
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
		out << "}\n";
	}
#else
	out << "Face: f" << face.getId() << "\n";
#endif

	return out;
}

const Vector3d& Polygon::getVertexPoint(const Index3DId& id) const
{
	auto p = getOurBlockPtr();
	if (p)
		return p->getVertexPoint(id);
	
	return getPolyMeshPtr()->getVertexPoint(id);
}


//LAMBDA_CLIENT_IMPLS(Polygon)
void Polygon::vertexFunc(const Index3DId& id, const std::function<void(const Vertex& obj)>& func) const {
	const auto p = getBlockPtr(); 
	if (p) 
		p->vertexFunc(id, func); 
	else {
		const auto p2 = getPolyMeshPtr(); 
		if (p2) 
			p2->vertexFunc(id, func);
	}
}

void Polygon::vertexFunc(const Index3DId& id, const std::function<void(Vertex& obj)>& func) {
	auto p = getBlockPtr(); 
	if (p) 
		p->vertexFunc(id, func); 
	else {
		auto p2 = getPolyMeshPtr(); 
		if (p2) 
			p2->vertexFunc(id, func);
	}
}

void Polygon::faceFunc(const Index3DId& id, const std::function<void(const Polygon& obj)>& func) const {
	const auto p = getBlockPtr(); 
	if (p) 
		p->faceFunc(id, func); 
	else {
		const auto p2 = getPolyMeshPtr(); 
		if (p2) 
			p2->faceFunc(id, func);
	}
}

void Polygon::faceFunc(const Index3DId& id, const std::function<void(Polygon& obj)>& func) {
	auto p = getBlockPtr(); 
	if (p) 
		p->faceFunc(id, func); 
	else {
		auto p2 = getPolyMeshPtr(); 
		if (p2) 
			p2->faceFunc(id, func);
	}
}

void Polygon::cellFunc(const Index3DId& id, const std::function<void(const Polyhedron& obj)>& func) const {
	const auto p = getBlockPtr(); 
	if (p) 
		p->cellFunc(id, func); 
	else {
		const auto p2 = getPolyMeshPtr(); 
		if (p2) 
			p2->cellFunc(id, func);
	}
}

void Polygon::cellFunc(const Index3DId& id, const std::function<void(Polyhedron& obj)>& func) {
	auto p = getBlockPtr(); 
	if (p) 
		p->cellFunc(id, func); 
	else {
		auto p2 = getPolyMeshPtr(); 
		if (p2) 
			p2->cellFunc(id, func);
	}
}

const Vertex& Polygon::getVertex(const Index3DId& id) const {
	const auto p = getBlockPtr(); 
	if (p) 
		return p->getVertex(id); 
	else {
		const auto p2 = getPolyMeshPtr(); 
		if (p2) 
			return p2->getVertex(id);
	} 
	throw std::runtime_error("Entity does not exist");
}

Vertex& Polygon::getVertex(const Index3DId& id) {
	auto p = getBlockPtr(); 
	if (p) 
		return p->getVertex(id); 
	else {
		auto p2 = getPolyMeshPtr(); 
		if (p2) 
			return p2->getVertex(id);
	} 
	throw std::runtime_error("Entity does not exist");
}

const DFHM::Polygon& Polygon::getPolygon(const Index3DId& id) const {
	const auto p = getBlockPtr(); 
	if (p) 
		return p->getPolygon(id); 
	else {
		const auto p2 = getPolyMeshPtr(); 
		if (p2) return 
			p2->getPolygon(id);
	} 
	throw std::runtime_error("Entity does not exist");
}

DFHM::Polygon& Polygon::getPolygon(const Index3DId& id) {
	auto p = getBlockPtr(); 
	if (p) 
		return p->getPolygon(id); 
	else {
		auto p2 = getPolyMeshPtr(); 
		if (p2) 
			return p2->getPolygon(id);
	} 
	throw std::runtime_error("Entity does not exist");
} 

const Polyhedron& Polygon::getPolyhedron(const Index3DId& id) const {
	const auto p = getBlockPtr(); 
	if (p) 
		return p->getPolyhedron(id); 
	else {
		const auto p2 = getPolyMeshPtr(); 
		if (p2) 
			return p2->getPolyhedron(id);
	} 
	
	throw std::runtime_error("Entity does not exist");
}  

Polyhedron& Polygon::getPolyhedron(const Index3DId& id) {
	auto p = getBlockPtr(); 
	if (p) 
		return p->getPolyhedron(id); 
	else {
		auto p2 = getPolyMeshPtr(); 
		if (p2) 
			return p2->getPolyhedron(id);
	} 
	throw std::runtime_error("Entity does not exist");
} 

void Polygon::edgeFunc(const EdgeKey& key, const std::function<void(const Edge& obj)>& func) const {
	const auto p = getBlockPtr(); 
	if (p) 
		p->edgeFunc(key, func); 
	else {
		const auto p2 = getPolyMeshPtr(); 
		if (p2) 
			p2->edgeFunc(key, func);
	}
} 

void Polygon::edgeFunc(const EdgeKey& key, const std::function<void(Edge& obj)>& func) {
	auto p = getBlockPtr(); 
	if (p) 
		p->edgeFunc(key, func); 
	else {
		auto p2 = getPolyMeshPtr(); 
		if (p2) 
			p2->edgeFunc(key, func);
	}
}
