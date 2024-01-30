#pragma once

#include <vector>
#include <set>
#include <vertex.h>
#include <index3D.h>

struct Plane;

namespace DFHM {

class Edge;
class Block;

class Polygon {
public:
	static bool vertifyUniqueStat(const std::vector<Index3DId>& vertIds);
	static bool verifyVertsConvexStat(const Block* pBlock, const std::vector<Index3DId>& vertIds);
	static double calVertexAngleStat(const Block* pBlock, const std::vector<Index3DId>& vertIds, size_t index);
	static Vector3d calUnitNormalStat(const Block* pBlock, const std::vector<Index3DId>& vertIds);
	static Vector3d calCentroidStat(const Block* pBlock, const std::vector<Index3DId>& vertIds);

	void setId(ObjectPoolOwner* pBlock, size_t id);
	const Index3DId& getId() const;
	void setNumSplits(size_t val);
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

	std::vector<Index3DId> getVertexIds() const;
	const std::vector<Index3DId>& getVertexIdsNTS() const;
	void setVertexIdsNTS(const std::vector<Index3DId>& verts);
	std::set<Edge> getEdges() const;
	bool containsEdge(const Edge& edge) const;
	bool containsEdge(const Edge& edge, size_t& idx0, size_t& idx1) const;
	bool containsVert(const Index3DId& vertId) const;
	bool vertsContainFace() const;

	bool vertifyUnique() const;
	bool verifyVertsConvex() const;
	bool verifyTopology() const;
	double calVertexAngle(size_t index) const;
	Vector3d calUnitNormal() const;
	Vector3d calCentroid() const;
	Vector3d interpolatePoint(double t, double u) const;
	Vector3d projectPoint(const Vector3d& pt) const;

	// inserts a vertex between vert0 and vert1.
	Index3DId insertVertexInEdgeNTS(const Edge& edge, const Vector3d& pt);
	bool insertVertexInEdgeNTS(const Edge& edge, const Index3DId& newVertId);

	std::vector<Index3DId> splitWithFaceEdgesNTS(const Polygon& splittingFace);

private:
	void sortIds() const;
	Index3DId findExistingSplitFaceId(const Edge& edge) const;

	template<class LAMBDA>
	void faceFuncSelf(LAMBDA func) const;

	template<class LAMBDA>
	void faceFuncSelf(LAMBDA func);

	std::set<Edge> getEdgesNTS() const;

	Block* _pBlock = nullptr;
	size_t _numSplits = 0;
	Index3DId _thisId;
	std::vector<Index3DId> _vertexIds;
	std::set<Index3DId> _cellIds;

	mutable bool _needSort = true;
	mutable std::vector<Index3DId> _sortedIds;
};

inline const Index3DId& Polygon::getId() const
{
	return _thisId;
}

inline void Polygon::setNumSplits(size_t val)
{
	_numSplits = val;
}

inline size_t Polygon::getNumSplits() const
{
	return _numSplits;
}

inline bool Polygon::vertifyUnique() const
{
	return vertifyUniqueStat(_vertexIds);
}

inline bool Polygon::verifyVertsConvex() const
{
	return verifyVertsConvexStat(_pBlock, _vertexIds);
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

inline const std::vector<Index3DId>& Polygon::getVertexIdsNTS() const
{
	return _vertexIds;
}

}
