#pragma once

#include <vector>
#include <UniversalIndex3D.h>

namespace DFHM {

class Block;

class Edge {
public:
	Edge() = default;
	Edge(const Edge& src) = default;
	Edge(const UniversalIndex3D& vert0, const UniversalIndex3D& vert1);

	void addFaceId(const UniversalIndex3D& faceId);
	void removeFaceId(const UniversalIndex3D& faceId);
	size_t numFaceIds() const;
	const UniversalIndex3D& getFaceId(size_t idx) const;

	bool unload(std::ostream& out, size_t idSelf);
	bool load(std::istream& out, size_t idSelf);

	bool operator < (const Edge& rhs) const;

	const UniversalIndex3D* getVertexIds() const;

	// Should only be called from Polygon::split which will handle topology
	UniversalIndex3D split(Block* pOwnerBlock, double t);

private:
	UniversalIndex3D _vertexIds[2];
	std::vector<UniversalIndex3D> _faceIds; // Should be 2, but this allows creation of nonmanifold edges if needed.
};

inline Edge::Edge(const UniversalIndex3D& vert0, const UniversalIndex3D& vert1)
{
	_vertexIds[0] = vert0;
	_vertexIds[1] = vert1;
}

inline size_t Edge::numFaceIds() const
{
	return _faceIds.size();
}

inline const UniversalIndex3D& Edge::getFaceId(size_t idx) const
{
	return _faceIds[idx];
}

inline const UniversalIndex3D* Edge::getVertexIds() const
{
	return _vertexIds;
}

}
