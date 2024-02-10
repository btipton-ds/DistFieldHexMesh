#pragma once

#include <vector>
#include <set>
#include <patient_lock_guard.h>
#include <index3D.h>
#include <objectPool.h>
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
	static Vector3d calUnitNormalStat(const Block* pBlock, const std::vector<Index3DId>& vertIds);
	static Vector3d calCentroidStat(const Block* pBlock, const std::vector<Index3DId>& vertIds);

	Polygon() = default;
	Polygon(const Polygon& src);
	Polygon& operator = (const Polygon& rhs);

	const Index3DId& getId() const;
	size_t getNumSplits() const;

	bool unload(std::ostream& out, size_t idSelf);
	bool load(std::istream& out, size_t idSelf);

	void addVertex(const Index3DId& vertId);

	void addCell(const Index3DId& cellId);
	void removeCell(const Index3DId& cellId);
	bool numCells() const;
	const std::set<Index3DId>& getCellIds() const;
	bool ownedByCell(const Index3DId& cellId) const;

	// Required for use with object pool
	void pack();

	bool isOuter() const;
	bool isBlockBoundary() const;

	bool operator < (const Polygon& rhs) const;

	const std::vector<Index3DId>& getVertexIds() const;
	void setVertexIds(const std::vector<Index3DId>& verts);
	void getEdges(std::set<Edge>& edgeSet) const;

	bool isOrphan() const; // No longer used by a cell
	bool containsEdge(const Edge& edge) const;
	bool containsEdge(const Edge& edge, size_t& idx0, size_t& idx1) const;
	bool containsVert(const Index3DId& vertId) const;
	bool isAbovePlane(const Plane& plane, double tol) const;
	bool allEdgesPrincipal() const;

	bool verifyUnique() const;
	bool verifyVertsConvex() const;
	bool verifyTopology() const;
	double calVertexAngle(size_t index) const;
	Vector3d calUnitNormal() const;
	Vector3d calCentroid() const;
	void calAreaAndCentroid(double& area, Vector3d& centroid) const;
	Vector3d interpolatePoint(double t, double u) const;
	Vector3d projectPoint(const Vector3d& pt) const;

	// inserts a vertex between vert0 and vert1.
	Index3DId insertVertexInEdge(const Edge& edge, const Vector3d& pt);
	bool insertVertexInEdge(const Edge& edge, const Index3DId& newVertId);

	Index3DId splitWithFaceEdges(const Polygon& splittingFace);

private:
	void sortIds() const;
	Index3DId findOtherSplitFaceId(const Edge& edge) const;

	size_t _numSplits = 0;
	std::vector<Index3DId> _vertexIds;
	std::set<Index3DId> _cellIds;

	mutable bool _needSort = true;
	mutable std::vector<Index3DId> _sortedIds;
};

inline const Index3DId& Polygon::getId() const
{
	return _thisId;
}

inline size_t Polygon::getNumSplits() const
{
	return _numSplits;
}

inline bool Polygon::isOrphan() const
{
	return _cellIds.empty();
}

inline bool Polygon::verifyUnique() const
{
	return verifyUniqueStat(_vertexIds);
}

inline bool Polygon::verifyVertsConvex() const
{
	return verifyVertsConvexStat(getBlockPtr(), _vertexIds);
}

inline void Polygon::addCell(const Index3DId& cellId)
{
	_cellIds.insert(cellId);
	assert(_cellIds.size() <= 2);
}

inline void Polygon::removeCell(const Index3DId& cellId)
{
	_cellIds.erase(cellId);
}

inline bool Polygon::numCells() const
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
