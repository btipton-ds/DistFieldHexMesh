#pragma once

#include <vector>
#include <set>
#include <index3D.h>

namespace DFHM {

class Block;

class Edge {
public:
	Edge() = default;
	Edge(const Edge& src) = default;
	Edge(const Index3D& ownerBlockIdx, const Index3DId& vert0, const Index3DId& vert1);

	const Index3D& getOwnerBlockIdx() const;
	void addFaceId(const Index3DId& faceId);
	void removeFaceId(const Index3DId& faceId);
	size_t numFaceIds() const;
	const std::set<Index3DId>& getFaceIds() const;

	bool unload(std::ostream& out, size_t idSelf);
	bool load(std::istream& out, size_t idSelf);

	bool operator < (const Edge& rhs) const;

	const Index3DId* getVertexIds() const;

	// Should only be called from Polygon::split which will handle topology
	Index3D split(Block* pOwnerBlock, double t);

private:
	Index3D _ownerBlockIdx;
	Index3DId _vertexIds[2];
	std::set<Index3DId> _faceIds; // Should be 2, but this allows creation of nonmanifold edges if needed.
};

inline Edge::Edge(const Index3D& ownerBlockIdx, const Index3DId& vert0, const Index3DId& vert1)
	: _ownerBlockIdx(ownerBlockIdx)
{
	_vertexIds[0] = vert0;
	_vertexIds[1] = vert1;
}

inline const Index3D& Edge::getOwnerBlockIdx() const
{
	return _ownerBlockIdx;
}

inline size_t Edge::numFaceIds() const
{
	return _faceIds.size();
}

inline const std::set<Index3DId>& Edge::getFaceIds() const
{
	return _faceIds;
}

inline const Index3DId* Edge::getVertexIds() const
{
	return _vertexIds;
}

}
