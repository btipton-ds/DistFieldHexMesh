#pragma once

#include <iostream>
#include <tm_boundingBox.h>
#include <index3D.h>

namespace DFHM {

class Block;
class Edge;

// Polyhedra are never cross block, so they use size_t for indexing.
// Faces and vertices in a cell are cross block
class Polyhedron {
public:
	Polyhedron() = default;
	Polyhedron(const Polyhedron& src) = default;
	Polyhedron(const std::vector<Index3DId>& faceIds);

	// Required for use with object pool
	void setId(const Index3DId& thisId);
	Index3DId getIndex() const;

	void addFace(const Index3DId& faceId);
	bool removeFace(const Index3DId& faceId);
	const std::vector<Index3DId>& getFaceIds() const;
	std::vector<Index3DId> getCornerIds(const Block* pBlock) const;
	std::vector<Edge> getEdges(const Block* pBlock) const;

	// Gets the edges for a vertex which belong to this polyhedron
	std::set<Edge> getVertEdges(const Block* pBlock, const Index3DId& vertId) const;
	// Gets the faces for a vertex which belong to this polyhedron
	std::set<Index3DId> getVertFaces(const Block* pBlock, const Index3DId& vertId) const;

	CBoundingBox3Dd getBoundingBox(const Block* pBlock) const;
	Vector3d calCentroid(const Block* pBlock) const;
	std::vector<size_t> split(Block* pBlock, bool intersectingOnly);
	std::vector<size_t> split(Block* pBlock, const Vector3d& pt, bool intersectingOnly);

	bool unload(std::ostream& out);
	bool load(std::istream& out);

	bool operator < (const Polyhedron& rhs) const;

private:
	std::vector<size_t> split(Block* pBlock, const Vector3d& pt, const Vector3d& normal, bool intersectingOnly);
	void orderVertIds(Block* pBlock, std::vector<Index3DId>& vertIds) const;

	Index3DId _thisId;
	std::vector<Index3DId> _faceIds;
};

inline void Polyhedron::setId(const Index3DId& thisId)
{
	_thisId = thisId;
	assert(_thisId.isValid());
}

inline Index3DId Polyhedron::getIndex() const
{
	return _thisId;
}

inline const std::vector<Index3DId>& Polyhedron::getFaceIds() const
{
	return _faceIds;
}

inline std::vector<size_t> Polyhedron::split(Block* pBlock, bool intersectingOnly)
{
	return split(pBlock, calCentroid(pBlock), intersectingOnly);
}

}

