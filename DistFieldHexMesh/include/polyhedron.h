#pragma once

#include <iostream>
#include <tm_boundingBox.h>
#include <index3D.h>

namespace DFHM {

class Block;
class Edge;

class Polyhedron {
public:
	Polyhedron() = default;
	Polyhedron(const Polyhedron& src) = default;
	Polyhedron(const std::vector<Index3DId>& faceIds);

	const std::vector<Index3DId>& getFaceIds() const;
	std::vector<Index3DId> getCornerIds(const Block* pBlock) const;
	std::vector<Edge> getEdges(const Block* pBlock) const;

	// Gets the edges for a vertex which belong to this polyhedron
	std::set<Edge> getVertEdges(const Block* pBlock, const Index3DId& vertId) const;
	// Gets the faces for a vertex which belong to this polyhedron
	std::set<Index3DId> getVertFaces(const Block* pBlock, const Index3DId& vertId) const;

	CBoundingBox3Dd getBoundingBox(const Block* pBlock) const;
	Vector3d calCentroid(const Block* pBlock) const;
	std::vector<size_t> split(Block* pBlock);
	std::vector<size_t> split(Block* pBlock, const Vector3d& pt);

	bool unload(std::ostream& out);
	bool load(std::istream& out);

	bool operator < (const Polyhedron& rhs) const;

private:
	std::vector<Index3DId> _faceIds;
};

inline const std::vector<Index3DId>& Polyhedron::getFaceIds() const
{
	return _faceIds;
}

inline std::vector<size_t> Polyhedron::split(Block* pBlock)
{
	return split(pBlock, calCentroid(pBlock));
}

}

