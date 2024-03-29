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
	Polyhedron(const Polyhedron& src) = default;

	// Required for use with object pool
	Index3DId getId() const;

	void addFace(const Index3DId& faceId);
	bool removeFace(const Index3DId& faceId);
	bool containsFace(const Index3DId& faceId) const;
	void addChild(const Index3DId& id);
	void setParent(const Index3DId& id);
	const std::set<Index3DId>& getFaceIds() const;
	void getVertIds(std::set<Index3DId>& vertIds) const;
	void getEdges(std::set<Edge>& edgeSet, bool includeNeighborFaces) const;
	size_t getLevel() const;
	void resetLevel();
	void incrementLevel(size_t newLevel);
	void setParentLevel();

	std::set<Index3DId> getAdjacentCells() const;

	// Gets the edges for a vertex which belong to this polyhedron
	void getVertEdges(const Index3DId& vertId, std::set<Edge>& edges, bool includeAdjacentCells) const;
	// Gets the faces for a vertex which belong to this polyhedron
	std::set<Index3DId> getVertFaces(const Index3DId& vertId) const;

	CBoundingBox3Dd getBoundingBox() const;
	bool contains(const Vector3d& pt) const;
	Vector3d calCentroid() const;
	bool intersectsModel() const;
	void markFaces(unsigned int markVal);

	// Splitting functions are const to prevent reusing the split cell. After splitting, the cell should be removed from the block
	bool splitAtCentroid(std::set<Index3DId>& newCellIds);
	bool splitAtPoint(const Vector3d& pt, std::set<Index3DId>& newCellIds);
	void splitByCurvature(int divsPerRadius, double maxCurvatureRadius, double sinEdgeAngle);
	void splitIfTooManyFaceSplits();
	void promoteSplitFacesWithSplitEdges();
	double getShortestEdge() const;

	bool unload(std::ostream& out);
	bool load(std::istream& out);
	void dumpFaces() const;

	bool isClosed() const;
	bool isActive() const;
	bool verifyTopology() const;
	bool operator < (const Polyhedron& rhs) const;

	LAMBDA_CLIENT_FUNC_PAIR_DECL(vertex);
	LAMBDA_CLIENT_FUNC_PAIR_DECL(face);
	LAMBDA_CLIENT_FUNC_PAIR_DECL(cell);

private:
	friend class Block;
	std::set<Edge> createEdgesFromVerts(std::vector<Index3DId>& vertIds) const;
	bool orderVertIds(std::vector<Index3DId>& vertIds) const;
	bool orderVertEdges(std::set<Edge>& edges, std::vector<Edge>& orderedEdges) const;
	void copyToOut() const;
	Index3DId createFace(const std::vector<Index3DId>& vertIds);
	void createHexahedralFaces(const std::vector<Index3DId>& cornerIds, std::vector<Index3DId>& faceIds);
	double calReferenceSurfaceRadius(const CBoundingBox3Dd& bbox, double maxCurvatureRadius, double sinEdgeAngle) const;

	mutable Trinary _intersectsModel = IS_UNKNOWN; // Cached value
	bool _needsCurvatureCheck = true;
	size_t _level = 0, _numSplits = 0;
	Index3DId addFace(const std::vector<Index3DId>& vertIds);
	Index3DId _parent; // This records the id of the polygon this polygon was split from
	std::set<Index3DId> _children;
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

inline size_t Polyhedron::getLevel() const
{
	return _level;
}

inline void Polyhedron::resetLevel()
{
	_level = 0;
}

inline bool Polyhedron::containsFace(const Index3DId& faceId) const
{
	return _faceIds.contains(faceId);
}

/*
CLIENT_LAMBDA_FUNC_PAIR_IMPL(Polyhedron, vertex);
CLIENT_LAMBDA_FUNC_PAIR_IMPL(Polyhedron, face);
CLIENT_LAMBDA_FUNC_PAIR_IMPL(Polyhedron, cell);
*/

CLIENT_LAMBDA_FUNC_PAIR_IMPL(Polyhedron, vertex);
CLIENT_LAMBDA_FUNC_PAIR_IMPL(Polyhedron, face);
CLIENT_LAMBDA_FUNC_PAIR_IMPL(Polyhedron, cell);

}

