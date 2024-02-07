#pragma once

#include <iostream>
#include <tm_boundingBox.h>
#include <index3D.h>
#include <objectPool.h>
#include <triMesh.h>

namespace DFHM {

class Block;
class Edge;

// Polyhedra are never cross block, so they use size_t for indexing.
// Faces and vertices in a cell are cross block
class Polyhedron : public ObjectPoolOwnerUser {
public:
	Polyhedron() = default;
	Polyhedron(const std::set<Index3DId>& faceIds);
	Polyhedron(const std::vector<Index3DId>& faceIds);
	Polyhedron(const Polyhedron& src);
	Polyhedron& operator = (const Polyhedron& rhs);

	void getBlocksToLock(std::set<Index3D>& blocksToLock) const;
	// Required for use with object pool
	Index3DId getId() const;

	void addFace(const Index3DId& faceId);
	bool removeFace(const Index3DId& faceId);
	const std::set<Index3DId>& getFaceIds() const;
	std::vector<Index3DId> getCornerIds() const;
	std::set<Edge> getEdges() const;
	std::set<Index3DId> getEdgeFaceIds(const Edge& edge) const;

	std::set<Index3DId> getAdjacentCells() const;
	// Must be callable from MultiLockGuard's constructor
	std::set<Index3D> getAdjacentBlockIndices_UNSAFE() const;

	// Gets the edges for a vertex which belong to this polyhedron
	std::set<Edge> getVertEdges(const Index3DId& vertId) const;
	// Gets the faces for a vertex which belong to this polyhedron
	std::set<Index3DId> getVertFaces(const Index3DId& vertId) const;

	CBoundingBox3Dd getBoundingBox() const;
	bool contains(const Vector3d& pt) const;
	Vector3d calCentroid() const;
	std::vector<Index3DId> splitWithPlane(const Plane& splitPlane, bool intersectingOnly);
	bool splitWithPlanesAtCentroid(bool intersectingOnly, std::vector<Index3DId>& newCellIds);
	bool splitWithPlanesAtPoint(const Vector3d& pt, bool intersectingOnly, std::vector<Index3DId>& newCellIds);
	void splitByCurvature(const TriMesh::CMeshPtr& pTriMesh, size_t circleDivs);

	bool unload(std::ostream& out);
	bool load(std::istream& out);
	void dumpFaces() const;

	bool isClosed() const;
	bool verifyTopology() const;
	bool verifyTopologyAdj() const;
	bool operator < (const Polyhedron& rhs) const;

private:
	friend class Block;
	std::set<Edge> createEdgesFromVerts(std::vector<Index3DId>& vertIds) const;
	bool orderVertIdsNTS(std::vector<Index3DId>& vertIds) const;

	std::set<Index3DId> _faceIds;
};

inline Index3DId Polyhedron::getId() const
{
	return _thisId;
}

inline const std::set<Index3DId>& Polyhedron::getFaceIds() const
{
	return _faceIds;
}

inline bool Polyhedron::splitWithPlanesAtCentroid(bool intersectingOnly, std::vector<Index3DId>& newCellIds)
{
	return splitWithPlanesAtPoint(calCentroid(), intersectingOnly, newCellIds);
}

}

