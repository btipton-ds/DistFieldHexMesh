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
	void setId(const Index3DId& id);
	const Index3DId& getId() const;
	void setNumSplits(size_t val);
	size_t getNumSplits() const;
	void verifyVertsConvex(const Block* pBlock) const;
	bool verifyTopology(const Block* pBlock) const;

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

	bool operator < (const Polygon& rhs) const;

	std::vector<Index3DId> getVertexIds(const Block* pBlock) const;
	const std::vector<Index3DId>& getVertexIdsNTS() const;
	void setVertexIds(Block* pBlock, const std::vector<Index3DId>& verts);
	std::vector<Edge> getEdges(const Block* pBlock) const;
	bool containsEdge(const Edge& edge) const;
	bool containsVert(const Index3DId& vertId) const;

	Vector3d getUnitNormal(const Block* pBlock) const;
	Vector3d getCentroid(const Block* pBlock) const;
	Vector3d getPointAt(const Block* pOwnerBlock, double t, double u) const;
	Vector3d projectPoint(const Block* pBlock, const Vector3d& pt) const;

	// inserts a vertex between vert0 and vert1.
	bool insertVertex(Block* pBlock, const Index3DId& vert0, const Index3DId& vert1, const Index3DId& newVertId);
	Index3DId insertVertex(Block* pBlock, const Index3DId& vert0, const Index3DId& vert1, const Vector3d& pt);
	bool insertVertex(Block* pBlock, const Edge& edge, const Index3DId& newVertId);

	Index3DId splitBetweenVertices(Block* pBlock, const Index3DId& vert0, const Index3DId& vert1);

private:
	void sortIds() const;
	static void verifyVertsConvex(const Block* pBlock, const std::vector<Index3DId>& vertIds);

	template<class LAMBDA>
	void faceFuncSelf(const Block* pBlock, LAMBDA func) const;

	template<class LAMBDA>
	void faceFuncSelf(Block* pBlock, LAMBDA func);

	std::vector<Edge> getEdgesNTS() const;
	Index3DId splitBetweenVerticesNTS(Block* pBlock, const Index3DId& vert0, const Index3DId& vert1);

	size_t _numSplits = 0;
	Index3DId _thisId;
	std::vector<Index3DId> _vertexIds;
	std::set<Index3DId> _cellIds;

	mutable bool _needSort = true;
	mutable std::vector<Index3DId> _sortedIds;
};

inline void Polygon::setId(const Index3DId& id)
{
	_thisId = id;
	assert(_thisId.isValid());
}

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

inline void Polygon::verifyVertsConvex(const Block* pBlock) const
{
	verifyVertsConvex(pBlock, _vertexIds);
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
