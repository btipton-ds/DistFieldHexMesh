#pragma once

#include <vector>
#include <set>
#include <patient_lock_guard.h>
#include <index3D.h>
#include <objectPool.h>
#include <lambdaMacros.h>
#include <vertex.h>

struct Plane;

namespace DFHM {

class Edge;
class Block;

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
	void setParentId(const Index3DId& id);
	const Index3DId& getParentId() const;
	void addChildId(const Index3DId& id);
	const std::set<Index3DId>& getChildIds() const;
	void clearChildIds();

	void addCellId(const Index3DId& cellId);
	void removeCellId(const Index3DId& cellId);
	void clearCellIds();
	void setCellIds(const std::set<Index3DId>& cellIds);
	size_t numCells() const;
	size_t numSplits() const; // This counts upward to show how many times this was split
	bool tooManyChildLevels() const;
	const std::set<Index3DId>& getCellIds() const;

	void setMarkVal(unsigned int val);
	void clearMarkVal(unsigned int val);
	bool isMarkSet(unsigned int val) const;

	bool ownedByCell(const Index3DId& cellId) const;
	bool isOuter() const;
	bool isLevelBoundary(size_t& innerLevel) const;
	bool isBlockBoundary() const;
	bool containsPt(const Vector3d& pt) const;
	bool isPointOnPlane(const Vector3d& pt) const;
	bool containsEdge(const Edge& edge) const;
	bool containsEdge(const Edge& edge, size_t& idx0, size_t& idx1) const;
	bool containsVert(const Index3DId& vertId) const;
	bool verifyUnique() const;
	bool verifyVertsConvex() const;
	bool verifyTopology() const;

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

	void pack();
	bool unload(std::ostream& out, size_t idSelf);
	bool load(std::istream& out, size_t idSelf);

	LAMBDA_CLIENT_FUNC_PAIR_DECL(vertex);
	LAMBDA_CLIENT_FUNC_PAIR_DECL(face);
	LAMBDA_CLIENT_FUNC_PAIR_DECL(cell);

private:
	friend class Polyhedron;

	void sortIds() const;
	Index3DId createFace(const Polygon& face);
	void setChildIds(const std::set<Index3DId>& childFaceIds);
	bool splitAtPoint(const Vector3d& pt, std::set<Index3DId>& newFaceIds, bool dryRun);
	bool imprintFaceVertices(const Polygon& otherFace);

	bool imprintVertexInEdge(const Index3DId& vertId, const Edge& edge);

	unsigned int _markVal = 0;
	Index3DId _parent; // This records the id of the polygon this polygon was split from
	std::set<Index3DId> _children;
	std::vector<Index3DId> _vertexIds;
	std::set<Index3DId> _cellIds;

	mutable bool _needSort = true;
	mutable std::vector<Index3DId> _sortedIds;
};

inline const Index3DId& Polygon::getId() const
{
	return _thisId;
}

inline void Polygon::setMarkVal(unsigned int val)
{
	_markVal |= val;
}

inline void Polygon::clearMarkVal(unsigned int val)
{
	_markVal &= ~val;
}

inline bool Polygon::isMarkSet(unsigned int val) const
{
	return _markVal & val;
}

inline bool Polygon::verifyUnique() const
{
	return verifyUniqueStat(_vertexIds);
}

inline bool Polygon::verifyVertsConvex() const
{
	return verifyVertsConvexStat(getBlockPtr(), _vertexIds);
}

inline void Polygon::setParentId(const Index3DId& id)
{
	_parent = id;
}

inline const Index3DId& Polygon::getParentId() const
{
	return _parent;
}

inline void Polygon::addChildId(const Index3DId& id)
{
	_children.insert(id);
}

inline const std::set<Index3DId>& Polygon::getChildIds() const
{
	return _children;
}

inline void Polygon::clearChildIds()
{
	_children.clear();
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

inline bool Polygon::ownedByCell(const Index3DId& cellId) const
{
	return _cellIds.count(cellId) != 0;
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
