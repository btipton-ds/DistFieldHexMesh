#pragma once

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

#include <memory>
#include <vector>
#include <set>
#include <iostream>
#include <triMeshPatch.h>
#include <patient_lock_guard.h>
#include <pool_map.h>
#include <pool_set.h>
#include <pool_vector.h>
#include <index3D.h>
#include <objectPool.h>
#include <lambdaMacros.h>
#include <fastBisectionSet.h>
#include <vertex.h>
#include <edge.h>

template<class T>
class Plane;
template<class T>
struct RayHit;

using RayHitd = RayHit<double>;
using Planed = Plane<double>;

namespace DFHM {

class MeshData;
using MeshDataPtr = std::shared_ptr<MeshData>;
class Edge;
class Block;
class TriMeshIndex;
struct SplittingParams;

struct VertEdgePair {
	inline VertEdgePair(const Index3DId& vertId, const Edge& edge)
		: _vertId(vertId)
		, _edge(edge)
	{
	}

	bool operator < (const VertEdgePair& rhs) const
	{
		if (_edge < rhs._edge)
			return true;
		else if (rhs._edge < _edge)
			return false;

		return _vertId < rhs._vertId;
	}

	Index3DId _vertId;
	Edge _edge;
};

// A polygon is owned by a single block, but it's vertices may belong to more than one block.
// Once a polygon is split, it is kept for reference but is otherwise dead.
// If an edge is split, the polygon must also be split.
// If a polygon has been split, it can be split again but DOES NOT become reference.

class Polygon : public ObjectPoolOwnerUser {
public:
	static bool verifyUniqueStat(const MTC::vector<Index3DId>& vertIds);
	static bool verifyVertsConvexStat(const Block* pBlock, const MTC::vector<Index3DId>& vertIds);
	static double calVertexAngleStat(const Block* pBlock, const MTC::vector<Index3DId>& vertIds, size_t index);
	static Vector3d calUnitNormalStat(const Block* pBlock, const MTC::vector<Index3DId>& vertIds);
	static Vector3d calCentroidApproxFastStat(const Block* pBlock, const MTC::vector<Index3DId>& vertIds);
	static void calCoordSysStat(const Block* pBlock, const MTC::vector<Index3DId>& vertIds, Vector3d& origin, Vector3d& xAxis, Vector3d& yAxis, Vector3d& zAxis);
	static void findConcaveVertIdsStat(const Block* pBlock, const MTC::vector<Index3DId>& vertIds, MTC::set<Index3DId>& cVertIds);

	static void dumpPolygonPoints(const Block* pBlock, std::ostream& out, const MTC::vector<Index3DId>& vertIds);
	static void dumpPolygonPoints(std::ostream& out, const MTC::vector<Vector3d>& pts);

	Polygon() = default;
	Polygon(const std::vector<Index3DId>& verts);
#if USE_MULTI_THREAD_CONTAINERS
	explicit Polygon(const MTC::vector<Index3DId>& verts);
#endif
	Polygon(const std::initializer_list<Index3DId>& verts);
	Polygon(const Polygon& src);

	Polygon& operator = (const Polygon& rhs);

	void postAddToPoolActions() override;
	Index3DId getId() const override;
	void remapId(const std::vector<size_t>& idRemap, const Index3D& srcDims) override;

	void addVertex(const Index3DId& vertId);
	void connectToplogy();
	void disconnectTopology();

	void addCellId(const Index3DId& cellId);
	void removeCellId(const Index3DId& cellId);
	bool isReversed(const Index3DId& cellId) const; // Orientation is relative to cellId
	void setReversed(const Index3DId& cellId, bool reversed); // Orientation is relative to cellId
	void flipReversed(const Index3DId& cellId); // Orientation is relative to cellId
	size_t numCells() const;
	const FastBisectionSet<Index3DId>& getCellIds() const;
	void getNestedCellIds(const Index3DId& testId, FastBisectionSet<Index3DId>& cellIds) const;

	bool usedByCell(const Index3DId& cellId) const;
	size_t getSplitLevel(const Index3DId& cellId) const;
	bool isCoplanar(const Vector3d& pt) const;
	bool isCoplanar(const Planed& pl) const;
	bool isCoplanar(const Edge& edge) const;
	bool isConvex()const;
	bool isWall() const;
	bool isBlockBoundary() const;
	bool isPointOnPlane(const Vector3d& pt) const;
	bool usesEdge(const Edge& edge) const;
	bool usesEdge(const Edge& edge, size_t& idx0, size_t& idx1) const;
	bool isPointOnEdge(const Vector3d& pt) const;
	bool containsPoint(const Vector3d& pt) const;
	bool containsVertex(const Index3DId& vertId) const;
	bool containsVertexNested(const Index3DId& vertId) const;
	bool containsEdge(const Edge& edge, bool& isUsed) const;
	bool containsFace(const Index3DId& faceId, size_t& level) const;
	bool isTooComplex(const SplittingParams& params) const;
	size_t numSplitLevels() const;

	bool findPiercePoints(const std::vector<size_t>& edgeIndices, MTC::vector<RayHitd>& piercePoints) const;
	template<class TRI_FUNC, class EDGE_FUNC>
	void getTriPoints(TRI_FUNC triFunc, EDGE_FUNC edgeFunc) const;
	int64_t getLayerNum() const;

	bool verifyUnique() const;
	bool verifyTopology() const;

	bool operator < (const Polygon& rhs) const;

	const MTC::vector<Index3DId>& getVertexIds() const;
	MTC::vector<Index3DId> getOrientedVertexIds(const Index3DId& cellId) const;
	size_t getNestedVertexIds(MTC::set<Index3DId>& vertIds) const;
	void clearCache() const;
	MTC::vector<EdgeKey> getEdgeKeys() const;
	Index3DId getAdjacentCellId(const Index3DId& thisCellId) const;
	void setSplitFaceIds(const MTC::vector<Index3DId>& faceIds);
	size_t numFaceIds(bool includeSplits) const;
	FastBisectionSet<Index3DId> getNestedFaceIds() const;

	double calVertexAngle(size_t index) const;
	double calVertexError(const std::vector<Vector3d>& testPts) const;
	double distanceToPoint(const Vector3d& pt) const;
	Planed calPlane() const;
	Planed calOrientedPlane(const Index3DId& cellId) const;
	Vector3d calUnitNormal() const;
	Vector3d calOrientedUnitNormal(const Index3DId& cellId) const;
	Vector3d calCentroid() const;
	Vector3d calCentroidApproxFast() const;
	bool intersectsModel(const std::vector<TriMeshIndex>& tris) const;
	bool intersectsModelTri(const std::vector<MeshDataPtr>& modelMesh, const TriMeshIndex& triIdx) const;
	double distFromPlane(const Vector3d& pt) const;
	void calAreaAndCentroid(double& area, Vector3d& centroid) const;
	Vector3d projectPoint(const Vector3d& pt) const;

	const FastBisectionSet<Index3DId>& getSplitIds() const;

	bool cellsOwnThis() const;
	void needToImprintVertices(const MTC::set<Index3DId>& verts, MTC::set<Index3DId>& imprintVerts) const;
	bool imprintVertex(const Index3DId& imprintVert);
	size_t getImprintIndex(const Vector3d& imprintPoint) const;
	size_t getImprintIndex(const Index3DId& imprintVert) const;
	bool isSplit() const;
	bool isPlanar() const;
	bool intersect(const LineSegmentd& seg, RayHitd& hit) const;
	bool intersect(const Planed& pl, LineSegmentd& intersectionSeg) const;
	bool isPointInside(const Vector3d& pt, const Vector3d& insidePt) const;

	const Vector3d& getVertexPoint(const Index3DId& id) const;
	size_t getCreatedDuringSplitNumber() const;
	void setCreatedDuringSplitNumber(size_t val);

	void write(std::ostream& out) const;
	void read(std::istream& in);

	bool unload(std::ostream& out, size_t idSelf);
	bool load(std::istream& out, size_t idSelf);

	template<class F>
	void iterateEdges(F fLambda) const;
	template<class F>
	void iterateOrientedEdges(F fLambda, const Index3DId& cellId) const;

	template<class F>
	void iterateTriangles(F fLambda) const;
	template<class F>
	void iterateOrientedTriangles(F fLambda, const Index3DId& cellId) const;

	LAMBDA_CLIENT_DECLS

protected:
	void setId(const Index3DId& id) override;

private:
	friend class Block;
	friend class Polyhedron;
	friend class PolygonSplitter;
	friend class Splitter;
	friend std::ostream& operator << (std::ostream& out, const Polygon& face);

	bool imprintVertexInner(const Index3DId& imprintVert);
	bool isPointInsideInner(const Vector3d& pt, const Vector3d& insidePt) const;
	void sortIds() const;

	Index3DId _thisId;
	size_t _createdDuringSplitNumber = 0;
	FastBisectionSet<Index3DId> _splitIds;	// Entities referencing this one

	MTC::vector<Index3DId> _vertexIds;
	FastBisectionSet<Index3DId> _cellIds;

	mutable bool _sortedIdsVaild = false;
	mutable bool _cachedCentroidValid = false;
	mutable bool _cachedNormalValid = false;

	mutable Trinary _isConvex = IS_UNKNOWN;
	mutable Trinary _cachedIntersectsModel = IS_UNKNOWN;
	mutable MTC::vector<Index3DId> _sortedIds;
	mutable double _cachedArea = -1;
	mutable Vector3d _cachedCentroid, _cachedNormal;
};

inline bool Polygon::verifyUnique() const
{
	return verifyUniqueStat(_vertexIds);
}

inline void Polygon::setReversed(const Index3DId& cellId, bool reversed)
{
	for (auto& id : _cellIds) {
		if (id == cellId) {
			id.setUserFlag(UF_FACE_REVERSED, reversed);
			break;
		}
	}
}

inline void Polygon::flipReversed(const Index3DId& cellId) // Orientation is relative to cellId
{
	for (const auto& id : _cellIds) {
		if (id == cellId) {
			bool reversed = id.isUserFlagSet(UF_FACE_REVERSED);
			id.setUserFlag(UF_FACE_REVERSED, !reversed);
			break;
		}
	}
}

inline bool Polygon::isReversed(const Index3DId& cellId) const
{
	for (const auto& id : _cellIds) {
		if (id == cellId) {
			return id.isUserFlagSet(UF_FACE_REVERSED);
		}
	}
	return false;

}

inline size_t Polygon::numCells() const
{
	return _cellIds.size();
}

inline const FastBisectionSet<Index3DId>& Polygon::getCellIds() const
{
	return _cellIds;
}

inline bool Polygon::usedByCell(const Index3DId& cellId) const
{
	return _cellIds.contains(cellId);
}

inline bool Polygon::isConvex() const
{
	if (_isConvex == IS_UNKNOWN) {
		MTC::set<Index3DId> tmp;
		findConcaveVertIdsStat(getBlockPtr(), _vertexIds, tmp);
		_isConvex = tmp.empty() ? IS_TRUE : IS_FALSE;
	}
	return _isConvex == IS_TRUE;
}

inline bool Polygon::isWall() const
{
	return _cellIds.size() == 1;
}

inline const MTC::vector<Index3DId>& Polygon::getVertexIds() const
{
	return _vertexIds;
}

inline MTC::vector<Index3DId> Polygon::getOrientedVertexIds(const Index3DId& cellId) const
{
	MTC::vector<Index3DId> result(_vertexIds);
	if (isReversed(cellId))
		std::reverse(result.begin(), result.end());
	return result;
}

inline const FastBisectionSet<Index3DId>& Polygon::getSplitIds() const
{
	return _splitIds;
}

inline size_t Polygon::getCreatedDuringSplitNumber() const
{
	return _createdDuringSplitNumber;
}

inline void Polygon::setCreatedDuringSplitNumber(size_t val)
{
	_createdDuringSplitNumber = val;
}

template<class F>
void Polygon::iterateEdges(F fLambda) const
{
	const auto& verts = _vertexIds;
	for (size_t i = 0; i < verts.size(); i++) {
		size_t j = (i + 1) % verts.size();
		Edge e(verts[i], verts[j]);
		if (!fLambda(e))
			break;
	}
}

template<class F>
void Polygon::iterateOrientedEdges(F fLambda, const Index3DId& cellId) const
{
	const auto& verts = _vertexIds;

	bool reversed = isReversed(cellId);
	for (size_t i = 0; i < verts.size(); i++) {
		const size_t j = (i + 1) % verts.size();
		size_t ii = i;
		size_t jj = j;
		if (reversed)
			std::swap(ii, jj);

		Edge e(verts[ii], verts[jj]);
		if (!fLambda(e))
			break;
	}
}

template<class F>
void Polygon::iterateTriangles(F fLambda) const
{
	const auto& verts = _vertexIds;
	size_t i = 0;
	for (size_t j = 1; j < verts.size() - 1; j++) {
		size_t k = j + 1;
		if (!fLambda(verts[i], verts[j], verts[k]))
			break;
	}
}

template<class F>
void Polygon::iterateOrientedTriangles(F fLambda, const Index3DId& cellId) const
{
	const auto& verts = _vertexIds;

	const size_t i = 0;
	for (size_t j = 1; j < verts.size() - 1; j++) {
		const size_t k = j + 1;

		size_t ii = i;
		size_t jj = j;
		size_t kk = k;
		if (isReversed(cellId))
			std::swap(ii, kk);

		if (!fLambda(verts[ii], verts[jj], verts[kk]))
			break;
	}
}

template<class TRI_FUNC, class EDGE_FUNC>
void Polygon::getTriPoints(TRI_FUNC triFunc, EDGE_FUNC edgeFunc) const
{
	if (_splitIds.empty()) {
		std::vector<Vector3d> pts;
		pts.resize(_vertexIds.size());
		for (size_t i = 0; i < _vertexIds.size(); i++)
			pts[i] = getVertexPoint(_vertexIds[i]);

		if (pts.size() > 4) {
			Vector3d ctr = calCentroid();
			for (size_t idx0 = 0; idx0 < pts.size(); idx0++) {
				size_t idx1 = (idx0 + 1) % pts.size();
				triFunc(ctr, pts[idx0], pts[idx1]);
			}
		} else {
			for (size_t i = 1; i < pts.size() - 1; i++) {
				size_t idx0 = 0;
				size_t idx1 = i;
				size_t idx2 = i + 1;
				triFunc(pts[idx0], pts[idx1], pts[idx2]);
			}
		}

		for (size_t i = 0; i < pts.size(); i++) {
			size_t idx0 = i;
			size_t idx1 = (idx0 + 1) % pts.size();
			edgeFunc(pts[idx0], pts[idx1]);
		}
	}
}

std::ostream& operator << (std::ostream& out, const Polygon& face);

}
