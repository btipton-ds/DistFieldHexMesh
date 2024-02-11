#pragma once

#include <vector>
#include <set>
#include <index3D.h>
#include <objectPool.h>

struct Plane;
struct LineSegment;

namespace DFHM {

class Block;
class Polyhedron;

class Edge {
public:

	Edge() = default;
	Edge(const Edge& src) = default;
	Edge(const Index3DId& vert0, const Index3DId& vert1, const std::set<Index3DId>& faceIds = std::set<Index3DId>());
	Edge(const Edge& src, const std::set<Index3DId>& faceIds);

	bool isValid() const;
	bool operator < (const Edge& rhs) const;
	bool operator == (const Edge& rhs) const;

	const Index3DId* getVertexIds() const;
	bool containsVertex(const Index3DId& vertexId) const;
	const std::set<Index3DId>& getFaceIds() const;
	void getFaceIds(std::set<Index3DId>& faceIds) const;
	Index3DId getOtherVert(const Index3DId& vert) const;

	double sameParamTol(const Block* pBlock) const;
	double getLength(const Block* pBlock) const;
	Vector3d calCenter(const Block* pBlock) const;
	Vector3d calUnitDir(const Block* pBlock) const;
	Vector3d calPointAt(const Block* pBlock, double t) const;
	double paramOfPt(const Block* pBlock, const Vector3d& pt, bool& inBounds) const;
	Vector3d projectPt(const Block* pBlock, const Vector3d& pt) const;
	bool onPrincipalAxis(const Block* pBlock) const;
	bool isColinearWith(const Block* pBlock, const Edge& other) const;
	bool isConnectedTo(const Edge& other) const;
	LineSegment getSegment(const Block* pBlock) const;

	double intesectPlaneParam(const Block* pBlock, const Plane& splittingPlane) const;
	Index3DId splitAtParam(Block* pBlock, double t, std::set<Index3DId>& faceIds) const;
	Index3DId splitWithPlane(Block* pBlock, const Plane& splittingPlane, std::set<Index3DId>& faceIds) const;

private:
	Index3DId _vertexIds[2];
	std::set<Index3DId> _faceIds;
};

inline const std::set<Index3DId>& Edge::getFaceIds() const
{
	return _faceIds;
}

inline const Index3DId* Edge::getVertexIds() const
{
	return _vertexIds;
}

}
