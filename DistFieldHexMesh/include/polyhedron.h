#pragma once

#include <iostream>
#include <triMesh.h>
#include <tm_boundingBox.h>
#include <index3D.h>
#include <objectPool.h>
#include <lambdaMacros.h>

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

	std::set<Index3DId> getAdjacentCells() const;

	// Gets the edges for a vertex which belong to this polyhedron
	void getVertEdges(const Index3DId& vertId, std::set<Edge>& edges, bool includeAdjacentCells) const;
	// Gets the faces for a vertex which belong to this polyhedron
	std::set<Index3DId> getVertFaces(const Index3DId& vertId) const;

	CBoundingBox3Dd getBoundingBox() const;
	bool contains(const Vector3d& pt) const;
	Vector3d calCentroid() const;

	// Splitting functions are const to prevent reusing the split cell. After splitting, the cell should be removed from the block
	bool splitAtPoint(const Vector3d& pt, std::set<Index3DId>& newCellIds) const;
	void splitByCurvature(double maxArcAngleDegrees) const;

	bool unload(std::ostream& out);
	bool load(std::istream& out);
	void dumpFaces() const;

	bool isClosed() const;
	bool verifyTopology() const;
	bool operator < (const Polyhedron& rhs) const;

	LAMBDA_FUNC_PAIR_DECL(vertex);
	LAMBDA_FUNC_PAIR_DECL(face);
	LAMBDA_FUNC_PAIR_DECL(cell);

private:
	friend class Block;
	std::set<Edge> createEdgesFromVerts(std::vector<Index3DId>& vertIds) const;
	bool orderVertIds(std::vector<Index3DId>& vertIds) const;
	bool orderVertEdges(std::set<Edge>& edges, std::vector<Edge>& orderedEdges) const;
	void copyToOut() const;

	Index3DId _parent; // This records the id of the polygon this polygon was split from
	std::vector<Index3DId> _children;
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

CLIENT_LAMBDA_FUNC_PAIR_IMPL(Polyhedron, vertex);
CLIENT_LAMBDA_FUNC_PAIR_IMPL(Polyhedron, face);
CLIENT_LAMBDA_FUNC_PAIR_IMPL(Polyhedron, cell);

}

