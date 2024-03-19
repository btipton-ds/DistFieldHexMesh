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
#include <patient_lock_guard.h>
#include <index3D.h>
#include <objectPool.h>
#include <lambdaMacros.h>
#include <vertex.h>
#include <edge.h>

struct Plane;

namespace DFHM {

class Edge;
class Block;

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
	static bool verifyUniqueStat(const std::vector<Index3DId>& vertIds);
	static bool verifyVertsConvexStat(const Block* pBlock, const std::vector<Index3DId>& vertIds);
	static double calVertexAngleStat(const Block* pBlock, const std::vector<Index3DId>& vertIds, size_t index);
	static void createEdgesStat(const std::vector<Index3DId>& verts, std::set<Edge>& edgeSet, const Index3DId& polygonId = Index3DId());
	static Vector3d calUnitNormalStat(const Block* pBlock, const std::vector<Index3DId>& vertIds);
	static Vector3d calCentroidStat(const Block* pBlock, const std::vector<Index3DId>& vertIds);

	Polygon() = default;
	Polygon(const std::vector<Index3DId>& verts);
	Polygon(const Polygon& src) = default;

	const Index3DId& getId() const;

	void addVertex(const Index3DId& vertId);
	const Index3DId& getReferenceEntityId() const;

	void addCellId(const Index3DId& cellId);
	void removeCellId(const Index3DId& cellId);
	void clearCellIds();
	void setCellIds(const std::set<Index3DId>& cellIds);
	size_t numCells() const;
	const std::set<Index3DId>& getCellIds() const;

	bool usedByCell(const Index3DId& cellId) const;
	bool isReference() const;
	bool hasSplitEdges() const;
	bool isOuter() const;
	bool isBlockBoundary() const;
	bool containsPoint(const Vector3d& pt) const;
	bool isPointOnPlane(const Vector3d& pt) const;
	bool containsEdge(const Edge& edge) const;
	bool containsEdge(const Edge& edge, size_t& idx0, size_t& idx1) const;
	bool containsVertex(const Index3DId& vertId) const;
	bool verifyUnique() const;
	bool verifyVertsConvex() const;
	bool verifyTopology() const;
	bool addRequiredImprintPairs(const Index3DId& vertId, std::set<VertEdgePair>& pairs) const;

	bool operator < (const Polygon& rhs) const;

	const std::vector<Index3DId>& getVertexIds() const;
	void getEdges(std::set<Edge>& edgeSet) const;
	Index3DId getNeighborCellId(const Index3DId& thisCellId) const;

	double getShortestEdge() const;
	double calVertexAngle(size_t index) const;
	Vector3d calUnitNormal() const;
	Vector3d calCentroid() const;
	double distFromPlane(const Vector3d& pt) const;
	void calAreaAndCentroid(double& area, Vector3d& centroid) const;
	Vector3d interpolatePoint(double t, double u) const;
	Vector3d projectPoint(const Vector3d& pt) const;
	void setNeedToSplitAtPoint(const Vector3d& pt);
	void splitIfRequred(int phase);

	int getSplitPhase() const;
	void splitAtPoint(const Vector3d& pt);
	void orient();
	void pack();
	bool unload(std::ostream& out, size_t idSelf);
	bool load(std::istream& out, size_t idSelf);

	LAMBDA_CLIENT_FUNC_PAIR_DECL(vertex);
	LAMBDA_CLIENT_FUNC_PAIR_DECL(face);
	LAMBDA_CLIENT_FUNC_PAIR_DECL(cell);

private:
	friend class Polyhedron;

	void sortIds() const;
	void makeOrphan();
	Index3DId createFace(const Polygon& face);
	bool imprintFaceVertices(const Polygon& otherFace);

	bool imprintVertex(const Index3DId& vertId, const Edge& edge);
	bool imprintVertex(Block* pBlk, const Index3DId& vertId, const Edge& edge);

	bool _splitRequired = false;
	Vector3d _splitPt;

	Index3DId _referenceEntityId;				// This points to the original refence entity used to create this one.
	std::set<Index3DId> _referencingEntityIds;	// Entities referencing this one

	std::vector<Index3DId> _vertexIds;
	std::set<Index3DId> _cellIds;

	mutable bool _needSort = true;
	mutable std::vector<Index3DId> _sortedIds;
};

inline const Index3DId& Polygon::getId() const
{
	return _thisId;
}

inline bool Polygon::verifyUnique() const
{
	return verifyUniqueStat(_vertexIds);
}

inline bool Polygon::verifyVertsConvex() const
{
	return verifyVertsConvexStat(getBlockPtr(), _vertexIds);
}

inline const Index3DId& Polygon::getReferenceEntityId() const
{
	return _referenceEntityId;
}

inline void Polygon::removeCellId(const Index3DId& cellId)
{
	_cellIds.erase(cellId);
}

inline void Polygon::setCellIds(const std::set<Index3DId>& cellIds)
{
	_cellIds = cellIds;
}

inline void Polygon::clearCellIds()
{
	_cellIds.clear();
}

inline size_t Polygon::numCells() const
{
	return _cellIds.size();
}

inline const std::set<Index3DId>& Polygon::getCellIds() const
{
	return _cellIds;
}

inline bool Polygon::usedByCell(const Index3DId& cellId) const
{
	return _cellIds.count(cellId) != 0;
}

inline bool Polygon::isReference() const
{
	return !_referencingEntityIds.empty();
}

inline bool Polygon::isOuter() const
{
	return _cellIds.size() == 1;
}

inline const std::vector<Index3DId>& Polygon::getVertexIds() const
{
	return _vertexIds;
}


}
