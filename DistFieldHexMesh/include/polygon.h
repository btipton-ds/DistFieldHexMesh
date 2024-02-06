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

	MutexType& getMutex() const;

	const Index3DId& getId() const;
	size_t getNumSplits() const;

	bool unload(std::ostream& out, size_t idSelf);
	bool load(std::istream& out, size_t idSelf);

	void addVertex(const Index3DId& vertId);

	void addCell(const Index3DId& cellId);
	void removeCell(const Index3DId& cellId);
	bool numCells() const;
	const std::set<Index3DId>& getCellIds() const;
	void setSplitFromData(const Index3DId& sourceFaceId);
	void clearSplitFromId();
	bool ownedByCell(const Index3DId& cellId) const;

	// Required for use with object pool
	void pack();

	bool isOuter() const;
	bool isBlockBoundary() const;

	bool operator < (const Polygon& rhs) const;

	const std::vector<Index3DId>& getVertexIds() const;
	void setVertexIdsNTS(const std::vector<Index3DId>& verts);
	std::set<Edge> getEdgesNTS() const;
	bool containsEdge(const Edge& edge) const;
	bool containsEdge(const Edge& edge, size_t& idx0, size_t& idx1) const;
	bool containsVert(const Index3DId& vertId) const;
	bool vertsContainFace() const;
	bool ownedByCellNTS(const Index3DId& cellId) const;
	bool wasSplitFromNTS(const Index3DId& faceId) const;
	bool isAbovePlane(const Plane& plane, double tol) const;

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

	Index3DId splitWithFaceEdgesNTS(const Polygon& splittingFace);

private:
	void sortIds() const;
	Index3DId findOtherSplitFaceId(const Edge& edge) const;

	mutable MutexType _mutex;
	size_t _numSplits = 0;
	std::set<Index3DId> _splitFromFaceIds;
	std::vector<Index3DId> _vertexIds;
	std::set<Index3DId> _cellIds;

	mutable bool _needSort = true;
	mutable std::vector<Index3DId> _sortedIds;
};

inline const Index3DId& Polygon::getId() const
{
	return _thisId;
}

inline MutexType& Polygon::getMutex() const
{
	return _mutex;
}

inline size_t Polygon::getNumSplits() const
{
	return _numSplits;
}

inline bool Polygon::vertifyUnique() const
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
