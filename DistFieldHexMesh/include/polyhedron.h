#pragma once

#include <iostream>
#include <tm_boundingBox.h>
#include <index3D.h>
#include <objectPool.h>
#include <block.h>

namespace DFHM {

class Block;
class Edge;

// Polyhedra are never cross block, so they use size_t for indexing.
// Faces and vertices in a cell are cross block
class Polyhedron {
public:
	Polyhedron() = default;
	Polyhedron(const Polyhedron& src) = default;
	Polyhedron(const std::set<Index3DId>& faceIds);
	Polyhedron(const std::vector<Index3DId>& faceIds);

	// Required for use with object pool
	void setId(ObjectPoolOwner* pBlock, size_t id);
	Index3DId getIndex() const;

	void addFace(const Index3DId& faceId);
	bool removeFace(const Index3DId& faceId);
	const std::set<Index3DId>& getFaceIds() const;
	std::vector<Index3DId> getCornerIds() const;
	std::vector<Edge> getEdges() const;

	// Gets the edges for a vertex which belong to this polyhedron
	std::set<Edge> getVertEdges(const Index3DId& vertId) const;
	// Gets the faces for a vertex which belong to this polyhedron
	std::set<Index3DId> getVertFaces(const Index3DId& vertId) const;

	CBoundingBox3Dd getBoundingBox() const;
	Vector3d calCentroid() const;
	std::vector<size_t> split(const Plane& splitPlane, bool intersectingOnly);
	std::vector<size_t> split(bool intersectingOnly);
	std::vector<size_t> split(const Vector3d& pt, bool intersectingOnly);

	bool unload(std::ostream& out);
	bool load(std::istream& out);

	bool verifyTopology() const;
	bool operator < (const Polyhedron& rhs) const;

private:
	void orderVertIdsNTS(std::vector<Index3DId>& vertIds) const;

	Block* _pBlock = nullptr;
	Index3DId _thisId;
	std::set<Index3DId> _faceIds;
};

inline Index3DId Polyhedron::getIndex() const
{
	return _thisId;
}

inline const std::set<Index3DId>& Polyhedron::getFaceIds() const
{
	return _faceIds;
}

inline std::vector<size_t> Polyhedron::split(bool intersectingOnly)
{
	return split(calCentroid(), intersectingOnly);
}

}

