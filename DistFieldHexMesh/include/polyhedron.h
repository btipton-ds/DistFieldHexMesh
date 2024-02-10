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

	// Required for use with object pool
	Index3DId getId() const;

	void addFace(const Index3DId& faceId);
	bool removeFace(const Index3DId& faceId);
	const std::set<Index3DId>& getFaceIds() const;
	void getVertIds(std::set<Index3DId>& vertIds) const;
	void getEdges(std::set<Edge>& edgeSet, bool includeNeighborFaces) const;
	void getPrincipalEdges(std::set<Edge>& result) const;
	void getPrincipalPolygons(std::set<Index3DId>& result) const;

	std::set<Index3DId> getAdjacentCells() const;

	// Gets the edges for a vertex which belong to this polyhedron
	std::set<Edge> getVertEdges(const Index3DId& vertId) const;
	// Gets the faces for a vertex which belong to this polyhedron
	std::set<Index3DId> getVertFaces(const Index3DId& vertId) const;

	CBoundingBox3Dd getBoundingBox() const;
	bool contains(const Vector3d& pt) const;
	Vector3d calCentroid() const;

	// Splitting functions are const to prevent reusing the split cell. After splitting, the cell should be removed from the block
	std::vector<Index3DId> splitWithPlane(const Plane& splitPlane, bool intersectingOnly) const;
	bool splitWithPlanesAtCentroid(bool intersectingOnly, std::vector<Index3DId>& newCellIds) const;
	bool splitWithPlanesAtPoint(const Vector3d& pt, bool intersectingOnly, std::vector<Index3DId>& newCellIds) const;
	void splitByCurvature(const TriMesh::CMeshPtr& pTriMesh, size_t circleDivs) const;

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
	bool orderVertIds(std::vector<Index3DId>& vertIds) const;

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

inline bool Polyhedron::splitWithPlanesAtCentroid(bool intersectingOnly, std::vector<Index3DId>& newCellIds) const
{
	return splitWithPlanesAtPoint(calCentroid(), intersectingOnly, newCellIds);
}

}

