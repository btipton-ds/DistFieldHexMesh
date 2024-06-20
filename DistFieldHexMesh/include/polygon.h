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
#include <vertex.h>
#include <edge.h>
#include <intersectEdge.h>

template<class T>
class Plane;
template<class T>
struct RayHit;

using RayHitd = RayHit<double>;
using Planed = Plane<double>;

namespace DFHM {

class Edge;
class Block;
struct BuildCFDParams;

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
	class CellId_SplitLevel {
	public:
		CellId_SplitLevel(const Index3DId& _cellId = Index3DId(), size_t splitLevel = 0);
		CellId_SplitLevel(const CellId_SplitLevel& src) = default;

		bool operator < (const CellId_SplitLevel& rhs) const;
		bool operator == (const CellId_SplitLevel& rhs) const;
		bool operator != (const CellId_SplitLevel& rhs) const;

		bool operator < (const Index3DId& rhs) const;
		bool operator == (const Index3DId& rhs) const;
		bool operator != (const Index3DId& rhs) const;

		operator const Index3DId& () const;
		const Index3DId& getId() const;

		size_t getSplitLevel() const;
		void write(std::ostream& out) const;
		void read(std::istream& in);

	private:
		Index3DId _cellId;
		size_t _splitLevel;
	};

	static bool verifyUniqueStat(const MTC::vector<Index3DId>& vertIds);
	static bool verifyVertsConvexStat(const Block* pBlock, const MTC::vector<Index3DId>& vertIds);
	static double calVertexAngleStat(const Block* pBlock, const MTC::vector<Index3DId>& vertIds, size_t index);
	static void createEdgesStat(const MTC::vector<Index3DId>& verts, MTC::set<Edge>& edgeSet, const Index3DId& polygonId = Index3DId());
	static Vector3d calUnitNormalStat(const Block* pBlock, const MTC::vector<Index3DId>& vertIds);
	static Vector3d calCentroidStat(const Block* pBlock, const MTC::vector<Index3DId>& vertIds);

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

	void addVertex(const Index3DId& vertId);

	void addCellId(const Index3DId& cellId, size_t level);
	void removeCellId(const Index3DId& cellId);
	void removeDeadCellIds();
	void unlinkFromCell(const Index3DId& cellId);
	size_t numCells() const;
	const MTC::set<CellId_SplitLevel>& getCellIds() const;

	bool usedByCell(const Index3DId& cellId) const;
	size_t getSplitLevel(const Index3DId& cellId) const;
	bool isOuter() const;
	bool isBlockBoundary() const;
	bool containsPoint(const Vector3d& pt) const;
	bool isPointOnPlane(const Vector3d& pt) const;
	bool findPiercePoints(const std::vector<size_t>& edgeIndices, MTC::vector<RayHitd>& piercePoints) const;
	bool containsEdge(const Edge& edge) const;
	bool containsEdge(const Edge& edge, size_t& idx0, size_t& idx1) const;
	bool containsVertex(const Index3DId& vertId) const;
	bool isCoplanar(const Vector3d& pt) const;
	bool isCoplanar(const Planed& pl) const;
	bool isCoplanar(const Edge& edge) const;
	bool verifyUnique() const;
	bool verifyTopology() const;

	bool operator < (const Polygon& rhs) const;

	const MTC::vector<Index3DId>& getVertexIds() const;
	const MTC::set<Edge>& getEdges() const;
	Index3DId getAdjacentCellId(const Index3DId& thisCellId) const;

	double getShortestEdge() const;
	double calVertexAngle(size_t index) const;
	double distanceToPoint(const Vector3d& pt) const;
	Planed calPlane() const;
	Vector3d calUnitNormal() const;
	Vector3d calCentroid() const;
	bool intersectsModel() const;
	double distFromPlane(const Vector3d& pt) const;
	void calAreaAndCentroid(double& area, Vector3d& centroid) const;
	Vector3d interpolatePoint(double t, double u) const;
	Vector3d projectPoint(const Vector3d& pt) const;

	const MTC::set<Index3DId>& getSplitFaceProductIds() const;
	const MTC::map<Edge, Index3DId>& getSplitEdgeVertMap() const;

	bool cellsOwnThis() const;
	void addSplitEdgeVert(const Edge& edge, const Index3DId& vertId) const;
	void needToImprintVertices(const MTC::set<Index3DId>& verts, MTC::set<Index3DId>& imprintVerts) const;
	bool imprintVertex(const Index3DId& imprintVert);
	size_t getImprintIndex(const Vector3d& imprintPoint) const;
	size_t getImprintIndex(const Index3DId& imprintVert) const;
	bool isSplit() const;
	bool isPlanar() const;
	bool intersect(const LineSegmentd& seg, RayHitd& hit) const;
	bool intersect(const Planed& pl, LineSegmentd& intersectionSeg) const;
	bool intersectModelTris(const TriMesh::PatchPtr& pPatch, MTC::set<IntersectEdge>& newEdges);
	bool isPointOnEdge(const Vector3d& pt) const;
	void createTrimmedFacesFromFaces(const MTC::set<Index3DId>& modelFaces, MTC::vector<Index3DId>& newFaceIds);
	void createModelInterectionEdges(const std::vector<size_t>& modFaceTris, const BuildCFDParams& params, MTC::set<Edge>& faceEdges);
	void createTrimmedFaceEdges(const MTC::set<Edge>& modFaceEdges, MTC::set<Edge>& trimEdges);

	size_t getCreatedDuringSplitNumber() const;
	void setCreatedDuringSplitNumber(size_t val);

	void pack();

	void write(std::ostream& out) const;
	void read(std::istream& in);

	bool unload(std::ostream& out, size_t idSelf);
	bool load(std::istream& out, size_t idSelf);

	LAMBDA_CLIENT_DECLS

private:
	friend class Block;
	friend class Polyhedron;
	friend class PolygonSplitter;
	friend class PolyhedronSplitter;
	friend std::ostream& operator << (std::ostream& out, const Polygon& face);

	void initVertices(const Volume* pVol) const;
	void sortIds() const;
	void addToSplitFaceProductIds(const Index3DId& id) const;
	TopolgyState getState() const;
	void clearCache() const;
	size_t getCellTris(std::vector<size_t>& indices) const;
	void imprintModelEdges(const MTC::set<Edge>& modelEdges, MTC::set<Edge>& edges);
	bool calModelNorm(const MTC::map<Index3DId, MTC::set<Edge>>& vertModEdgeMap, const Index3DId& vertId, Vector3d& norm) const;

	size_t _createdDuringSplitNumber = 0;
	MTC::set<Index3DId> _splitFaceProductIds;	// Entities referencing this one
	MTC::map<Edge, Index3DId> _splitEdgeVertMap;

	MTC::vector<Index3DId> _vertexIds;
	MTC::set<CellId_SplitLevel> _cellIds;

	mutable bool _sortCacheVaild = false;
	mutable bool _cachedEdgesVaild = false;
	mutable Trinary _cachedIntersectsModel = Trinary::IS_UNKNOWN;
	mutable MTC::vector<Index3DId> _sortedIds;
	mutable MTC::set<Edge> _cachedEdges;
};

inline bool Polygon::CellId_SplitLevel::operator < (const CellId_SplitLevel& rhs) const
{
	return _cellId < rhs._cellId;
}

inline bool Polygon::CellId_SplitLevel::operator == (const CellId_SplitLevel& rhs) const
{
	return _cellId == rhs._cellId;
}

inline bool Polygon::CellId_SplitLevel::operator != (const CellId_SplitLevel& rhs) const
{
	return _cellId != rhs._cellId;
}

inline bool Polygon::CellId_SplitLevel::operator < (const Index3DId& rhs) const
{
	return _cellId < rhs;
}

inline bool Polygon::CellId_SplitLevel::operator == (const Index3DId& rhs) const
{
	return _cellId == rhs;
}

inline bool Polygon::CellId_SplitLevel::operator != (const Index3DId& rhs) const
{
	return _cellId != rhs;
}

inline Polygon::CellId_SplitLevel::operator const Index3DId& () const
{
	return _cellId;
}

inline const Index3DId& Polygon::CellId_SplitLevel::getId() const
{
	return _cellId;
}

inline size_t Polygon::CellId_SplitLevel::getSplitLevel() const
{
	return _splitLevel;
}

inline bool Polygon::verifyUnique() const
{
	return verifyUniqueStat(_vertexIds);
}

inline size_t Polygon::numCells() const
{
	return _cellIds.size();
}

inline const MTC::set<Polygon::CellId_SplitLevel>& Polygon::getCellIds() const
{
	return _cellIds;
}

inline bool Polygon::usedByCell(const Index3DId& cellId) const
{
	return _cellIds.count(CellId_SplitLevel(cellId)) != 0;
}

inline bool Polygon::isOuter() const
{
	return _cellIds.size() == 1;
}

inline const MTC::vector<Index3DId>& Polygon::getVertexIds() const
{
	return _vertexIds;
}

inline const MTC::set<Index3DId>& Polygon::getSplitFaceProductIds() const
{
	return _splitFaceProductIds;
}

inline const MTC::map<Edge, Index3DId>& Polygon::getSplitEdgeVertMap() const
{
	return _splitEdgeVertMap;
}

inline size_t Polygon::getCreatedDuringSplitNumber() const
{
	return _createdDuringSplitNumber;
}

inline void Polygon::setCreatedDuringSplitNumber(size_t val)
{
	_createdDuringSplitNumber = val;
}

std::ostream& operator << (std::ostream& out, const Polygon& face);

}
