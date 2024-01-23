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
	void setOurId(const Index3DId& id);
	bool unload(std::ostream& out, size_t idSelf);
	bool load(std::istream& out, size_t idSelf);

	void addVertex(const Index3DId& vertId);
	void setCreatorCellId(const Index3DId& cellId);
	const Index3DId& getCreatorCellId() const;
	void setNeighborCellId(const Index3DId& subBlockId);
	const Index3DId& getNeighborCellId() const;

	void doneCreating();
	void pack();

	bool isOuter() const;

	bool operator < (const Polygon& rhs) const;

	const std::vector<Index3DId>& getVertexIds() const;
	std::vector<Edge> getEdges(const Block* pBlock) const;
	bool containsEdge(const Edge& edge) const;

	Vector3d getUnitNormal(const Block* pBlock) const;
	Vector3d getCentroid(const Block* pBlock) const;
	Vector3d getPointAt(const Block* pOwnerBlock, double t, double u) const;
	Vector3d projectPoint(const Block* pBlock, const Vector3d& pt) const;

	// inserts a vertex between vert0 and vert1.
	bool insertVertex(Block* pBlock, const Index3DId& vert0, const Index3DId& vert1, const Index3DId& newVertId);
	Index3DId insertVertex(Block* pBlock, const Index3DId& vert0, const Index3DId& vert1, const Vector3d& pt);
	bool insertVertex(Block* pBlock, const Edge& edge, const Index3DId& newVertId);

	Index3DId splitWithPlane(Block* pBlock, const Plane& splittingPlane);

private:
	Index3DId _ourId;
	std::vector<Index3DId> _vertexIds, _sortedIds;
	Index3DId _creatorCellId, _neighborCellId;
};

inline void Polygon::setOurId(const Index3DId& id)
{
	_ourId = id;
}

inline void Polygon::setCreatorCellId(const Index3DId& subBlockId)
{
	_creatorCellId = subBlockId;
}

inline void Polygon::setNeighborCellId(const Index3DId& subBlockId)
{
	assert(subBlockId != _creatorCellId);
	_neighborCellId = subBlockId;
}


inline const Index3DId& Polygon::getCreatorCellId() const
{
	return _creatorCellId;
}

inline const Index3DId& Polygon::getNeighborCellId() const
{
	return _neighborCellId;
}

inline bool Polygon::isOuter() const
{
	return !_neighborCellId.isValid();
}

inline const std::vector<Index3DId>& Polygon::getVertexIds() const
{
	return _vertexIds;
}

}
