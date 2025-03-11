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

void Polygon::recreateToMatch(const std::vector<Index3DId>& newVertIds, MTC::set<Index3DId>& newFaceIds)
{
	const double tol = Tolerance::paramTol();
	const double tolSqr = tol * tol;
	// This is one of the most important functions in cell splitting.
	// NewVertIds describes and "ideal" new face, but that face may not fit with its neighbor faces.
	// This function restructures the new and old faces to match each other.
#ifdef _DEBUG
	for (const auto& cellId : _cellIds) {
		cellFunc(cellId, [](Polyhedron& cell) {
			assert(cell.isClosed());
		});
	}

	for (const auto& id : newVertIds) {
		assert(id.isValid());
	}

	Vector3d pt0 = getVertexPoint(newVertIds[0]);
	Vector3d pt1 = getVertexPoint(newVertIds[1]);
	Vector3d pt2 = getVertexPoint(newVertIds[2]);
	Vector3d v0 = pt0 - pt1;
	Vector3d v1 = pt2 - pt1;
	Vector3d newNorm = v1.cross(v0).normalized();
	Vector3d norm = calUnitNormal();
	Vector3d cp = norm.cross(newNorm);
	assert(cp.squaredNorm() < tolSqr);
	getBlockPtr()->getVolume()->writeObj("D:/DarkSky/Projects/output/objs/faceSplitOld.obj", { _vertexIds });
	getBlockPtr()->getVolume()->writeObj("D:/DarkSky/Projects/output/objs/faceSplitNew.obj", { newVertIds });
	int dbgBreak = 1;
#endif // _DEBUG

	Polygon newPoly(newVertIds);
	Index3DId id = getBlockPtr()->findFace(newPoly);
	if (id.isValid()) {
		faceFunc(id, [](const Polygon& face) {
			assert(face.getCellIds().size() < 2);
		});
		// There is already a cell which matches exactly, use it.
		newFaceIds.insert(id);
	} else if (_cellIds.empty()) {
		// The old face is not attached to another cell. It's going to be deleted when the owner cell is deleted so skip it.
		id = getBlockPtr()->addFace(newPoly);
		assert(id.isValid());
		faceFunc(id, [](const Polygon& face) {
			assert(face.getCellIds().size() < 2);
		});
		newFaceIds.insert(id);
	} else {
		vector<size_t> insertionIndices;
		vector<Index3DId> newVertsToInsert;
		for (size_t i = 0; i < _vertexIds.size(); i++) {
			size_t j = (i + 1) % _vertexIds.size();
			LineSegmentd seg(getVertexPoint(_vertexIds[i]), getVertexPoint(_vertexIds[j]));
			for (size_t k = 0; k < newVertIds.size(); k++) {
				const auto& newVertId = newVertIds[k];
				if (find(_vertexIds.begin(), _vertexIds.end(), newVertId) == _vertexIds.end()) {
					const auto& pt = getVertexPoint(newVertId);
					double t;
					if (seg.contains(pt, t, Tolerance::sameDistTol())) {
						newVertsToInsert.push_back(newVertId);
						insertionIndices.push_back(i);
						break;
					}
				}
			}
		}
		if (insertionIndices.size() == 1) {
			recreateToMatch1(newVertsToInsert[0], insertionIndices, newPoly, newFaceIds);
		} else if (insertionIndices.size() == 2) {
			recreateToMatch2(newVertsToInsert, insertionIndices, newPoly, newFaceIds);
		} else {
//			getBlockPtr()->getVolume()->writeObj("D:/DarkSky/Projects/output/objs/faceSplitOld.obj", { _vertexIds });
//			getBlockPtr()->getVolume()->writeObj("D:/DarkSky/Projects/output/objs/faceSplitNew.obj", { newVertIds });
			id = getBlockPtr()->addFace(newPoly);
			assert(id.isValid());
			newFaceIds.insert(id);
		}

	}
}

void Polygon::recreateToMatch1(const Index3DId& newVertToInsert, const MTC::vector<size_t>& insertionIndices, const Polygon& newPoly, MTC::set<Index3DId>& newFaceIds)
{
	auto edgeKeyVecs = getEdgeKeys();
	set<EdgeKey> edgeKeys, disconnectedEdgeKeys;
	edgeKeys.insert(edgeKeyVecs.begin(), edgeKeyVecs.end());
	for (const auto& edgeKey : edgeKeys) {
		const auto& newVertIds = newPoly.getVertexIds();
		for (size_t i = 0; i < newVertIds.size(); i++) {
			size_t j = (i + 1) % newVertIds.size();
			EdgeKey e(newVertIds[i], newVertIds[j]);
			if (!edgeKeys.contains(e)) {
				disconnectedEdgeKeys.insert(e);
			}
		}
	}
	int dbgBreak = 1;
}

void Polygon::recreateToMatch2(const MTC::vector<Index3DId>& newVertsToInsert, const MTC::vector<size_t>& insertionIndices, const Polygon& newPoly, MTC::set<Index3DId>& newFaceIds)
{
	assert(newVertsToInsert.size() == 2);
	MTC::vector<Index3DId> verts0, verts1;

	verts0.push_back(newVertsToInsert[0]);
	size_t idx = insertionIndices[0];
	do {
		idx = (idx + 1) % _vertexIds.size();
		verts0.push_back(_vertexIds[idx]);
	} while (idx != insertionIndices[1]);
	verts0.push_back(newVertsToInsert[1]);

	verts1.push_back(newVertsToInsert[1]);
	do {
		idx = (idx + 1) % _vertexIds.size();
		verts1.push_back(_vertexIds[idx]);
	} while (idx != insertionIndices[0]);
	verts1.push_back(newVertsToInsert[0]);

	set<Index3DId> adjFaceIds;
	for (size_t i = 0; i < _vertexIds.size(); i++) {
		size_t j = (i + 1) % _vertexIds.size();
		edgeFunc(EdgeKey(_vertexIds[i], _vertexIds[j]), [this, &adjFaceIds](const Edge& edge) {
			const auto& t = edge.getFaceIds();
			for (const auto& faceId : t) {
				if (getId() != faceId)
					adjFaceIds.insert(faceId);
			}
		});
	}
	disconnectVertEdgeTopology();
	for (const auto& faceId : adjFaceIds) {
		faceFunc(faceId, [newVertsToInsert](Polygon& face) {
			for (const auto& vertId : newVertsToInsert) {
				face.imprintVertex(vertId);
			}
		});
	}

	auto faceId0 = getBlockPtr()->addFace(Polygon(verts0));
	assert(faceId0.isValid());
	faceFunc(faceId0, [&newPoly, &newFaceIds](Polygon& face) {
		assert(face.getCellIds().size() < 2);
		if (face == newPoly)
			newFaceIds.insert(face.getId());
		});

	auto faceId1 = getBlockPtr()->addFace(Polygon(verts1));
	assert(faceId1.isValid());
	faceFunc(faceId1, [&newPoly, &newFaceIds](Polygon& face) {
		assert(face.getCellIds().size() < 2);

		if (face == newPoly)
			newFaceIds.insert(face.getId());
	});

	auto tmp = _cellIds;
	for (const auto& cellId : tmp) {
		cellFunc(cellId, [this, &faceId0, &faceId1](Polyhedron& cell) {
			assert(cell.isClosed());
			cell.removeFace(getId());
			cell.addFace(faceId0);
			cell.addFace(faceId1);
			assert(cell.isClosed());
		});
	}
}

void Polygon::connectVertEdgeTopology() {
	for (size_t i = 0; i < _vertexIds.size(); i++) {
		size_t j = (i + 1) % _vertexIds.size();
		size_t k = (j + 1) % _vertexIds.size();
		vertexFunc(_vertexIds[j], [this, i, k](Vertex& vert) {
			vert.addConnectedVertexId(_vertexIds[i]);
			vert.addConnectedVertexId(_vertexIds[k]);
		});

		EdgeKey edgeKey(_vertexIds[i], _vertexIds[j]);
		auto newEdgeKey = getBlockPtr()->addEdge(edgeKey);
		assert(newEdgeKey == edgeKey);
		edgeFunc(newEdgeKey, [this](Edge& edge) {
			edge.addFaceId(getId());
		});
	}
}

void Polygon::disconnectVertEdgeTopology() {
	for (size_t i = 0; i < _vertexIds.size(); i++) {
		size_t j = (i + 1) % _vertexIds.size();
		size_t k = (j + 1) % _vertexIds.size();
		vertexFunc(_vertexIds[j], [this, i, k](Vertex& vert) {
			vert.removeConnectedVertexId(_vertexIds[i]);
			vert.removeConnectedVertexId(_vertexIds[k]);
		});

		EdgeKey edgeKey(_vertexIds[i], _vertexIds[j]);
		edgeFunc(edgeKey, [this](Edge& edge) {
			edge.removeFaceId(getId());
		});
	}
}

Polygon::Polygon(const Polygon& src)
	: ObjectPoolOwnerUser(src)
	, _vertexIds(src._vertexIds)
	, _cellIds(src._cellIds)
	, _isConvex(src._isConvex)
	, _cachedIntersectsModel(src._cachedIntersectsModel)
	, _sortedIds(src._sortedIds)
	, _cachedArea(src._cachedArea)
	, _cachedCentroid(src._cachedCentroid)
	, _cachedNormal(src._cachedNormal)
{
}

DFHM::Polygon& DFHM::Polygon::operator = (const Polygon& rhs)
{
	clearCache();
	disconnectVertEdgeTopology();

	ObjectPoolOwnerUser::operator=(rhs);
	_vertexIds = rhs._vertexIds;
	_cellIds = rhs._cellIds;
	_cachedIntersectsModel = rhs._cachedIntersectsModel;

	connectVertEdgeTopology();
	return *this;
}

void Polygon::postAddToPoolActions()
{
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
	clearCache();
}

void Polygon::clearCache() const
{
	_sortedIdsVaild = false;
	_cachedCentroidValid = false;
	_cachedNormalValid = false;
	_isConvex = IS_UNKNOWN;
	_cachedIntersectsModel = IS_UNKNOWN;
	_sortedIds.clear();
}

bool Polygon::cellsOwnThis() const
{
	for (const auto& cellId : _cellIds) {
		if (!getBlockPtr()->polyhedronExists(cellId))
			return false;
		bool result = true;
		cellFunc(cellId, [this, &result](const Polyhedron& cell) {
			if (!cell.containsFace(getId()))
				result = false;
		});

		if (!result)
			return false;
	}

	return true;
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
	uint8_t version = 0;
	out.write((char*)&version, sizeof(version));

	IoUtil::writeObj(out, _vertexIds);
	IoUtil::writeObj(out, _cellIds);

}

void Polygon::read(istream& in)
{
	uint8_t version;
	in.read((char*)&version, sizeof(version));

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

void Polygon::sortIds() const
{
	if (!_sortedIdsVaild) {
		_sortedIdsVaild = true;
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

bool Polygon::operator == (const Polygon& rhs) const
{
	return !operator <(rhs) && !rhs.operator<(*this);
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
		result.push_back(Edge(_vertexIds[i], _vertexIds[j]));
	}

	return result;
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
		Edge testEdge(vertId0, vertId1);
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

bool Polygon::isCoplanar(const EdgeKey& edgeKey) const
{
	Planed pl = calPlane();
	const auto pt0 = getVertexPoint(edgeKey.getVertex(0));
	if (!pl.isCoincident(pt0, Tolerance::sameDistTol()))
		return false;

	const auto pt1 = getVertexPoint(edgeKey.getVertex(1));
	if (!pl.isCoincident(pt1, Tolerance::sameDistTol()))
		return false;

	return true;
}

Vector3d Polygon::calCentroidApproxFastStat(const Block* pBlock, const MTC::vector<Index3DId>& vertIds)
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
	origin = calCentroidApproxFastStat(pBlock, vertIds);
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
	if (_cachedNormalValid)
		return _cachedNormal;

	_cachedNormal = calUnitNormalStat(getBlockPtr(), _vertexIds);
	_cachedNormalValid = true;
	return _cachedNormal;
}

Vector3d Polygon::calOrientedUnitNormal(const Index3DId& cellId) const
{
	auto pBlk = getBlockPtr();
	Vector3d result = calUnitNormal();
	auto faceApproxCtr = calCentroidApproxFastStat(pBlk, _vertexIds);
	Vector3d cellApproxCtr;
	cellFunc(cellId, [&cellApproxCtr](const Polyhedron& cell) {
		cellApproxCtr = cell.calCentroidApproxFast();
	});
	Vector3d v = faceApproxCtr - cellApproxCtr;
	if (v.dot(result) < 0)
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

Vector3d Polygon::calCentroid() const
{
	double area;
	Vector3d ctr;
	calAreaAndCentroid(area, ctr);
	return ctr;
}

Vector3d Polygon::calCentroidApproxFast() const
{
	return calCentroidApproxFastStat(getBlockPtr(), _vertexIds);
}

bool Polygon::intersectsModel(const std::vector<TriMeshIndex>& triIndices) const
{
	const double tol = Tolerance::sameDistTol();
	if (_cachedIntersectsModel == IS_UNKNOWN) {
		_cachedIntersectsModel = IS_FALSE;
		const auto& modelMesh = getBlockPtr()->getModelMeshData();
		for (const auto& triIdx : triIndices) {
			const auto& pData = modelMesh[triIdx.getMeshIdx()];
			const auto& pMesh = pData->getMesh();
			const auto& tri = pMesh->getTri(triIdx.getTriIdx());
			const Vector3d* pts[] = {
				&pMesh->getVert(tri[0])._pt,
				&pMesh->getVert(tri[1])._pt,
				&pMesh->getVert(tri[2])._pt,
			};
			iterateTriangles([this, &pts, tol](const Index3DId& id0, const Index3DId& id1, const Index3DId& id2)->bool {
				const Vector3d* facePts[] = {
					&getVertexPoint(id0),
					&getVertexPoint(id1),
					&getVertexPoint(id2),
				};

				if (intersectTriTri(pts, facePts, tol)) {
					_cachedIntersectsModel = IS_TRUE;
				}

				return _cachedIntersectsModel != IS_TRUE; // false exits the lambda for loop
			});
		}
	}

	return _cachedIntersectsModel == IS_TRUE;
}

double Polygon::distFromPlane(const Vector3d& pt) const
{
	Plane pl(getVertexPoint(_vertexIds[0]), calUnitNormal());
	return pl.distanceToPoint(pt);
}

void Polygon::calAreaAndCentroid(double& area, Vector3d& centroid) const
{
	if (_cachedCentroidValid) {
		area = _cachedArea;
		centroid = _cachedCentroid;
		return;
	}
	area = 0;
	centroid = Vector3d(0, 0, 0);
	Vector3d pt0 = getVertexPoint(_vertexIds[0]);
	for (size_t j = 1; j < _vertexIds.size() - 1; j++) {
		size_t k = j + 1;
		Vector3d pt1 = getVertexPoint(_vertexIds[j]);
		Vector3d pt2 = getVertexPoint(_vertexIds[k]);
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
	_cachedCentroid = centroid;
	_cachedCentroidValid = true;
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

void Polygon::addCellId(const Index3DId& cellId)
{
#if DEBUG_BREAKS && defined(_DEBUG)
	if (Index3DId(4, 6, 1, 0) == cellId) {
		int dbgBreak = 1;
	}
#endif

	_cellIds.insert(cellId);
#if 1 && defined(_DEBUG)
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

size_t Polygon::getPossibleOverlappingFaceIds(const MTC::vector<EdgeKey>& ourEdgeKeys, MTC::set<Index3DId>& faceIds)
{
	for (const auto& edgeKey : ourEdgeKeys) {
		edgeFunc(edgeKey, [this, &faceIds](const Edge& edge) {
			auto& tmp = edge.getFaceIds();
			faceIds.insert(tmp.begin(), tmp.end());
		});
	}

	faceIds.erase(getId());
	return faceIds.size();
}

bool Polygon::imprintVertex(const Index3DId& imprintVert)
{
	if (containsVertex(imprintVert)) {
		return false;
	}
	MTC::vector<Index3DId> tmp;
	tmp.reserve(tmp.size() + 1);
	bool imprinted = false;
	for (size_t i = 0; i < _vertexIds.size(); i++) {
		tmp.push_back(_vertexIds[i]);

		if (!imprinted) {
			size_t j = (i + 1) % _vertexIds.size();
			edgeFunc(EdgeKey(_vertexIds[i], _vertexIds[j]), [this, &tmp, &imprintVert, &imprinted](Edge& edge) {
				auto seg = edge.getSegment();
				Vector3d pt = getVertexPoint(imprintVert);
				double t;
				if (seg.contains(pt, t, Tolerance::sameDistTol())) {
					tmp.push_back(imprintVert);
					imprinted = true;
				}
			});
		}
	}

	if (imprinted) {
		disconnectVertEdgeTopology();
		clearCache();
		_vertexIds = tmp;
		connectVertEdgeTopology();
	}

	return imprinted;
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
		auto edgeSeg = edge.getSegment();
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

bool Polygon::isPointInside(const Vector3d& pt, const Vector3d& insidePt) const
{
	bool result;
	result = isPointInsideInner(pt, insidePt);

	return result;
}

bool Polygon::isPointInsideInner(const Vector3d& pt, const Vector3d& insidePt) const
{
	bool above;
	const double tol = Tolerance::sameDistTol();
	iterateTriangles([this, &tol, &pt, &insidePt, &above](const Index3DId& id0, const Index3DId& id1, const Index3DId& id2)->bool {
		Vector3d triPts[] = {
			getVertexPoint(id0),
			getVertexPoint(id1),
			getVertexPoint(id2),
		};
		Planed pl(triPts[0], triPts[1], triPts[2]);
		Vector3d v = triPts[0] - insidePt;

		// Assure vector is pointing outwards
		if (v.dot(pl.getNormal()) < 0)
			pl.reverse();

		double dist = pl.distanceToPoint(pt);
		above = dist > tol;
		return above;
	});

	return !above;
}

bool Polygon::isPointOnEdge(const Vector3d& pt) const
{
	for (size_t i = 0; i < _vertexIds.size(); i++) {
		size_t j = (i + 1) % _vertexIds.size();
		auto* pEdge = getBlockPtr()->getEdge(Edge(_vertexIds[i], _vertexIds[j]));
		auto seg = pEdge->getSegment();
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
		out << "}\n";
	}
#else
	out << "Face: f" << face.getId() << "\n";
#endif

	return out;
}

inline const Vector3d& Polygon::getVertexPoint(const Index3DId& id) const
{
	return getBlockPtr()->getVertexPoint(id);
}


//LAMBDA_CLIENT_IMPLS(Polygon)
void Polygon::vertexFunc(const Index3DId& id, const std::function<void(const Vertex& obj)>& func) const {
	const auto p = getBlockPtr(); 
	p->vertexFunc(id, func);
} 

void Polygon::vertexFunc(const Index3DId& id, const std::function<void(Vertex& obj)>& func) {
	auto p = getBlockPtr(); 
	p->vertexFunc(id, func);
} 

void Polygon::faceFunc(const Index3DId& id, const std::function<void(const Polygon& obj)>& func) const {
	const auto p = getBlockPtr(); 
	p->faceFunc(id, func);
} 

void Polygon::faceFunc(const Index3DId& id, const std::function<void(Polygon& obj)>& func) {
	auto p = getBlockPtr(); 
	p->faceFunc(id, func);
} 

void Polygon::cellFunc(const Index3DId& id, const std::function<void(const Polyhedron& obj)>& func) const {
	const auto p = getBlockPtr(); 
	p->cellFunc(id, func);
} 

void Polygon::cellFunc(const Index3DId& id, const std::function<void(Polyhedron& obj)>& func) {
	auto p = getBlockPtr(); 
	p->cellFunc(id, func);
} 

void Polygon::edgeFunc(const EdgeKey& key, const std::function<void(const Edge& obj)>& func) const {
	const auto p = getBlockPtr(); 
	p->edgeFunc(key, func);
} 

void Polygon::edgeFunc(const EdgeKey& key, const std::function<void(Edge& obj)>& func) {
	auto p = getBlockPtr(); 
	p->edgeFunc(key, func);
}
