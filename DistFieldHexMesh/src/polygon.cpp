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
#include <fstream>
#include <tm_math.h>
#include <tm_lineSegment.h>
#include <tm_ray.h>
#include <tm_ioUtil.h>
#include <vertex.h>
#include <edge.h>
#include <polygon.h>
#include <block.h>
#include <logger.h>
#include <volume.h>

#define CACHE_BIT_SORTED 1
#define CACHE_BIT_EDGES 2

using namespace std;
using namespace DFHM;

Polygon::Polygon(const vector<Index3DId>& verts)
	: _vertexIds(verts)
{
}

Polygon::Polygon(const Polygon& src)
	: _createdDuringSplitNumber(src._createdDuringSplitNumber)
	, _splitFaceProductIds(src._splitFaceProductIds)
	, _vertexIds(src._vertexIds)
	, _cellIds(src._cellIds)
	// Don't copy the caches
{
}

Polygon& Polygon::operator = (const Polygon& rhs)
{
	clearCache();
	_createdDuringSplitNumber = rhs._createdDuringSplitNumber;
	_splitFaceProductIds = rhs._splitFaceProductIds;
	_vertexIds = rhs._vertexIds;
	_cellIds = rhs._cellIds;

	return *this;
}

void Polygon::addVertex(const Index3DId& vertId)
{
	_vertexIds.push_back(vertId);
	clearCache();
}

void Polygon::clearCache() const
{
	_sortCacheVaild = false;
	_edgeCacheVaild = false;
	_cachedEdges.clear();
	_sortedIds.clear();
}

bool Polygon::cellsOwnThis() const
{
	if (getBlockPtr()->isPolygonReference(this))
		return true;
	for (const auto& cellId : _cellIds) {
		if (!getBlockPtr()->polyhedronExists(TS_REAL, cellId))
			return false;
		bool result = true;
		cellFunc(TS_REAL,cellId, [this, &result](const Polyhedron& cell) {
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
	uint8_t version = 0;
	out.write((char*)&version, sizeof(version));

	out.write((char*)&_createdDuringSplitNumber, sizeof(_createdDuringSplitNumber));

	IoUtil::write(out, _splitFaceProductIds);
	IoUtil::write(out, _splitEdgeVertMap);
	IoUtil::write(out, _vertexIds);
	IoUtil::write(out, _cellIds);
}

void Polygon::read(istream& in)
{
	uint8_t version;
	in.read((char*)&version, sizeof(version));

	in.read((char*)&_createdDuringSplitNumber, sizeof(_createdDuringSplitNumber));

	IoUtil::read(in, _splitFaceProductIds);
	IoUtil::read(in, _splitEdgeVertMap);
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
	if (!_sortCacheVaild) {
		_sortCacheVaild = true;
		_sortedIds = _vertexIds;
		sort(_sortedIds.begin(), _sortedIds.end());
	}
}

void Polygon::orient()
{
	Index3DId ownerCellId;
	switch (_cellIds.size()) {
		default:
			return;
		case 1: { // Outer polygon
			ownerCellId = *_cellIds.begin();
			break;
		}
		case 2: { // Inner polygon
			auto iter = _cellIds.begin();
			auto id0 = *iter++;
			auto id1 = *iter;

			ownerCellId = getBlockPtr()->maxCellId(id0, id1);
			break;
		}
	}

	Vector3d norm = calUnitNormal();
	Vector3d faceCtr = calCentroid();
	Vector3d cellCtr;
	cellFunc(TS_REAL,ownerCellId, [&cellCtr](const Polyhedron& cell) {
		cellCtr = cell.calCentroid();
		});
	Vector3d v = cellCtr - faceCtr;
	if (v.dot(norm) < Tolerance::paramTol()) {
		reverse(_vertexIds.begin(), _vertexIds.end());
	}
}

void Polygon::pack()
{
	_sortedIds.clear();
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

const set<Edge>& Polygon::getEdges() const
{
	if (!_edgeCacheVaild) {
		createEdgesStat(_vertexIds, _cachedEdges, _thisId);
		_edgeCacheVaild = true;
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
	if (!isPointOnPlane(pt))
		return false;

	Vector3d norm = calUnitNormal();
	for (size_t i = 0; i < _vertexIds.size(); i++) {
		size_t j = (i + 1) % _vertexIds.size();

		Vector3d pt0 = getBlockPtr()->getVertexPoint(_vertexIds[i]);
		Vector3d pt1 = getBlockPtr()->getVertexPoint(_vertexIds[j]);
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

bool Polygon::containsEdge(const Edge& edge) const
{
	size_t idx0, idx1;
	return containsEdge(edge, idx0, idx1);
}

bool Polygon::containsEdge(const Edge& edge, size_t& idx0, size_t& idx1) const
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

bool Polygon::containsVertex(const Index3DId& vertId) const
{
	for (const auto& id : _vertexIds) {
		if (id == vertId)
			return true;
	}
	return false;
}

bool Polygon::coplanar(const Plane<double>& pl) const
{
	for (const auto& vertId : _vertexIds) {
		auto pt = getBlockPtr()->getVertexPoint(vertId);
		if (fabs(pl.distanceToPoint(pt)) > Tolerance::sameDistTol())
			return false;
	}

	return true;
}

void Polygon::createEdgesStat(const vector<Index3DId>& verts, set<Edge>& edgeSet, const Index3DId& polygonId)
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

Vector3d Polygon::calUnitNormalStat(const Block* pBlock, const vector<Index3DId>& vertIds)
{
	Vector3d norm(0, 0, 0);

	for (size_t i = 0; i < vertIds.size() - 2; i++) {
		size_t j = (i + 1) % vertIds.size();
		size_t k = (j + 1) % vertIds.size();

		Vector3d pt0 = pBlock->getVertexPoint(vertIds[i]);
		Vector3d pt1 = pBlock->getVertexPoint(vertIds[j]);
		Vector3d pt2 = pBlock->getVertexPoint(vertIds[k]);

		Vector3d v0 = pt2 - pt1;
		Vector3d v1 = pt0 - pt1;

		Vector3d n = v0.cross(v1);
		norm += n;
	}
	norm.normalize();
	return norm;
}

Vector3d Polygon::calUnitNormal() const
{
	return calUnitNormalStat(getBlockPtr(), _vertexIds);
}

double Polygon::calVertexAngleStat(const Block* pBlock, const vector<Index3DId>& vertIds, size_t idx1)
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
		Vector3d pt0 = getBlockPtr()->getVertexPoint(_vertexIds[i]);
		Vector3d pt1 = getBlockPtr()->getVertexPoint(_vertexIds[1]);
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
		Vector3d pt0 = getBlockPtr()->getVertexPoint(_vertexIds[i]);
		Vector3d pt1 = getBlockPtr()->getVertexPoint(_vertexIds[j]);
		Vector3d v0 = pt0 - ctr;
		Vector3d v1 = pt1 - ctr;
		Vector3d n = v1.cross(v0).normalized();
		Plane<double> pl(ctr, n, false);
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
		getBlockPtr()->getVertexPoint(_vertexIds[0]),
		getBlockPtr()->getVertexPoint(_vertexIds[1]),
		getBlockPtr()->getVertexPoint(_vertexIds[2]),
		getBlockPtr()->getVertexPoint(_vertexIds[3]),
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
	auto pMesh = getBlockPtr()->getModelMesh();
	CBoundingBox3Dd bbox;
	for (const auto& vertId : _vertexIds) {
		bbox.merge(getBlockPtr()->getVertexPoint(vertId));
	}

	vector<size_t> triEntries;
	if (getBlockPtr()->processTris(bbox, triEntries)) {
		auto pTriMesh = getBlockPtr()->getModelMesh();
		vector<Edge> edges;
		for (size_t i = 0; i < _vertexIds.size(); i++) {
			size_t j = (i + 1) % _vertexIds.size();
			const auto& vertId0 = _vertexIds[i];
			Edge edge(_vertexIds[i], _vertexIds[j]);
			edges.push_back(edge);
		}

		for (const auto& triIdx : triEntries) {
			const auto& tri = pTriMesh->getTri(triIdx);
			const Vector3d* pts[3] = {
				pts[0] = &(pTriMesh->getVert(tri[0])._pt),
				pts[1] = &(pTriMesh->getVert(tri[1])._pt),
				pts[2] = &(pTriMesh->getVert(tri[2])._pt),
			};

			for (const auto& edge : edges) {
				auto seg = edge.getSegment(getBlockPtr());
				RayHit<double> hit;
				if (seg.intersectTri(pts, hit))
					return true;
			}
		}
	}

	return false; // Don't test split cells
}

double Polygon::distFromPlane(const Vector3d& pt) const
{
	Plane pl(getBlockPtr()->getVertexPoint(_vertexIds[0]), calUnitNormal(), false);
	return pl.distanceToPoint(pt);
}

void Polygon::calAreaAndCentroid(double& area, Vector3d& centroid) const
{
	if (!getBlockPtr()->isPolygonReference(this) && getBlockPtr()->polygonExists(TS_REFERENCE, _thisId)) {
		faceFunc(TS_REFERENCE, _thisId, [&area, &centroid](const Polygon& self) {
			self.calAreaAndCentroid(area, centroid);
		});
		return;
	}

	area = 0;
	centroid = Vector3d(0, 0, 0);
	Vector3d pt0 = getBlockPtr()->getVertexPoint(_vertexIds[0]);
	for (size_t j = 1; j < _vertexIds.size() - 1; j++) {
		size_t k = j + 1;
		Vector3d pt1 = getBlockPtr()->getVertexPoint(_vertexIds[j]);
		Vector3d pt2 = getBlockPtr()->getVertexPoint(_vertexIds[k]);
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
	Vector3d origin = getBlockPtr()->getVertexPoint(_vertexIds[0]); // And point will do
	Vector3d normal = calUnitNormal();
	Plane pl(origin, normal, false);
	auto result = pl.projectPoint(pt);

	return result;
}

void Polygon::removeCellId(const Index3DId& cellId)
{
	_cellIds.erase(cellId);
}

void Polygon::removeDeadCellIds()
{
	set<CellId_SplitLevel> tmp;
	for (const auto& cellId : _cellIds) {
		if (getBlockPtr()->polyhedronExists(TS_REAL, cellId))
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
	_cellIds.insert(CellId_SplitLevel(cellId, level));
#if 1
	if (_cellIds.size() > 2) {
		for (const auto& cellId1 : _cellIds) {
			assert(getBlockPtr()->polyhedronExists(TS_REAL, cellId1));
			cellFunc(TS_REAL,cellId1, [this](const Polyhedron& cell) {
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

TopolgyState Polygon::getState() const
{
	if (getBlockPtr()->isPolygonReference(this))
		return TS_REFERENCE;
	else
		return TS_REAL;
}

void Polygon::addToSplitFaceProductIds(const Index3DId& id) const
{
	assert(getBlockPtr()->isPolygonReference(this));
	Polygon* refSelf = const_cast<Polygon*>(this);
	refSelf->_splitFaceProductIds.insert(id);
}

void Polygon::addSplitEdgeVert(const Edge& edge, const Index3DId& vertId) const
{
	assert(getBlockPtr()->isPolygonReference(this));
	Polygon* refSelf = const_cast<Polygon*>(this);
	refSelf->_splitEdgeVertMap.insert(make_pair(edge, vertId));
}

void Polygon::needToImprintVertices(const set<Index3DId>& verts, set<Index3DId>& imprintVerts) const
{

	vector<Index3DId> onFaceVerts;
	set<Index3DId> vertSet;
	vertSet.insert(_vertexIds.begin(), _vertexIds.end());
	for (const auto& vertId : verts) {
		if (!vertSet.contains(vertId)) { // ignore vertices already in the face
			Vector3d pt = getBlockPtr()->getVertexPoint(vertId);
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
			Vector3d pt = getBlockPtr()->getVertexPoint(vertId);
			double t;
			if (seg.contains(pt, t) && t > 0 && t < 1) {
				imprintVerts.insert(vertId);
				break;
			}
		}
	}
}

void Polygon::imprintVertex(const Index3DId& imprintVert)
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
			Vector3d pt = getBlockPtr()->getVertexPoint(imprintVert);
			double t;
			if (seg.contains(pt, t) && t > 0 && t < 1) {
				_vertexIds.push_back(imprintVert);
				imprinted = true;
			}
		}
	}

	if (imprinted)
		clearCache();
}

bool Polygon::needToImprintVertices_deprecated(const map<Edge, Index3DId>& edgeVertMap) const
{
	const auto& edges = getEdges();
	for (const auto& edge : edges) {
		if (edgeVertMap.find(edge) != edgeVertMap.end()) {
			return true;
		}
	}

	return false;
}

void Polygon::imprintVertices_deprecated(const map<Edge, Index3DId>& edgeVertMap)
{
#if LOGGING_ENABLED
	auto pLogger = getBlockPtr()->getLogger();
	auto& out = pLogger->getStream();

#if LOGGING_VERBOSE_ENABLED
	LOG(out << Logger::Pad() << "imprintVertices:" << *this);
#else
	LOG(out << Logger::Pad() << "imprintVertices: f" << _thisId << "\n");
#endif
#endif
	auto pBlk = getBlockPtr();

	// Increment down so insertions do not corrupt order
	// the vertex is not in this polygon and lies between i and j
	if (_thisId.isValid())
		pBlk->removeFaceFromLookUp(_thisId);

	LOG(out << Logger::Pad() << "vertexIds(" << _vertexIds.size() << "): {");
	for (const auto& vertId0 : _vertexIds) {
		LOG(out << "v" << vertId0 << " ");
	}
	LOG(out << "}\n");

	vector<Index3DId> newVerts;
	for (size_t i = 0; i < _vertexIds.size(); i++) {
		newVerts.push_back(_vertexIds[i]);
		size_t j = (i + 1) % _vertexIds.size();
		Edge edge(_vertexIds[i], _vertexIds[j]);
		auto iter = edgeVertMap.find(edge);
		if (iter != edgeVertMap.end()) {
			const auto& vertId = iter->second;
			newVerts.push_back(vertId);
		}
	}

	_vertexIds = newVerts;

	LOG(out << Logger::Pad() << "vertexIds(" << _vertexIds.size() << "): {");
	for (const auto& vertId0 : _vertexIds) {
		LOG(out << "v" << vertId0 << " ");
	}
	LOG(out << "}\n");

	clearCache();

	if (_thisId.isValid())
		pBlk->addFaceToLookup(_thisId);
}

bool Polygon::isSplit() const
{
	return _splitFaceProductIds.size() == _vertexIds.size();
}

bool Polygon::verifyVertsConvexStat(const Block* pBlock, const vector<Index3DId>& vertIds)
{
	for (size_t i = 0; i < vertIds.size(); i++) {
		double angle = calVertexAngleStat(pBlock, vertIds, i);
		if (angle < 0 && angle > M_PI)
			return false;
	}

	return true;
}

bool Polygon::verifyUniqueStat(const vector<Index3DId>& vertIds)
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
			if (valid && !getBlockPtr()->polyhedronExists(TS_REAL, cellId))
				valid = false;

			cellFunc(TS_REAL,cellId, [this, &valid](const Polyhedron& cell) {
				if (valid && !cell.containsFace(_thisId)) {
					valid = false;
				}
				});
			if (!valid)
				return valid;
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
		
		out << Logger::Pad() << "vertexIds: (" << face._vertexIds.size() << "): {";
		for (const auto& vertId : face._vertexIds) {
			out << "v" << vertId << " ";
		}
		out << "}\n";

		out << Logger::Pad() << "cellIds: (" << face._cellIds.size() << "): {";
		for (const auto& cellId : face._cellIds) {
			auto sl = face.getSplitLevel(cellId);
			out << "c" << cellId << ".split: " << sl << " ";
		}
		out << "}\n";

		if (!face._splitFaceProductIds.empty()) {
			out << Logger::Pad() << "splitFaceIds: (" << face._splitFaceProductIds.size() << "): {";
			for (const auto& faceId : face._splitFaceProductIds) {
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

Polygon::CellId_SplitLevel::CellId_SplitLevel(const Index3DId& cellId, size_t splitLevel)
	: _cellId(cellId)
	, _splitLevel(splitLevel)
{
}

bool Polygon::CellId_SplitLevel::operator < (const CellId_SplitLevel& rhs) const
{
	return _cellId < rhs._cellId;
}

Polygon::CellId_SplitLevel::operator const Index3DId& () const
{
	return _cellId;
}

const Index3DId& Polygon::CellId_SplitLevel::getId() const
{
	return _cellId;
}

size_t Polygon::CellId_SplitLevel::getSplitLevel() const
{
	return _splitLevel;
}

void Polygon::CellId_SplitLevel::write(ostream& out) const
{
	uint8_t version = 0;
	out.write((char*)&version, sizeof(uint8_t));
	_cellId.write(out);
	out.write((char*)&_splitLevel, sizeof(size_t));
}

void Polygon::CellId_SplitLevel::read(istream& in)
{
	uint8_t version;
	in.read((char*)&version, sizeof(uint8_t));
	_cellId.read(in);
	in.read((char*)&_splitLevel, sizeof(size_t));
}

//LAMBDA_CLIENT_IMPLS(Polygon)
void Polygon::vertexFunc(const Index3DId& id, const function<void(const Vertex& obj)>& func) const {
	auto p = getBlockPtr(); 
	p->vertexFunc(id, func);
} 

void Polygon::vertexFunc(const Index3DId& id, const function<void(Vertex& obj)>& func) {
	auto p = getBlockPtr(); 
	p->vertexFunc(id, func);
} 

void Polygon::faceFunc(TopolgyState state, const Index3DId& id, const function<void(const Polygon& obj)>& func) const {
	auto p = getBlockPtr(); 
	p->faceFunc(state, id, func);
} 

void Polygon::faceFunc(TopolgyState state, const Index3DId& id, const function<void(Polygon& obj)>& func) {
	auto p = getBlockPtr(); 
	p->faceFunc(state, id, func);
} 

void Polygon::cellFunc(TopolgyState state, const Index3DId& id, const function<void(const Polyhedron& obj)>& func) const {
	auto p = getBlockPtr(); 
	p->cellFunc(state, id, func);
} 

void Polygon::cellFunc(TopolgyState state, const Index3DId& id, const function<void(Polyhedron& obj)>& func) {
	auto p = getBlockPtr(); 
	p->cellFunc(state, id, func);
} 

void Polygon::faceAvailFunc(TopolgyState prefState, const Index3DId& id, const function<void(const Polygon& obj)>& func) const {
	auto p = getBlockPtr(); 
	p->faceAvailFunc(prefState, id, func);
} 

void Polygon::cellAvailFunc(TopolgyState prefState, const Index3DId& id, const function<void(const Polyhedron& obj)>& func) const {
	auto p = getBlockPtr(); 
	p->cellAvailFunc(prefState, id, func);
}
