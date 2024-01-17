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

	CBoundingBox3Dd getBoundingBox(const Block* pBlock) const;
	void split(const Block* pBlock, const Vector3d& pt);

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

}

