#pragma once

#include <vector>
#include <set>
#include <index3D.h>
#include <objectPool.h>

struct Plane;

namespace DFHM {

class Block;
class Polyhedron;

class Edge {
public:

	Edge() = default;
	Edge(const Edge& src) = default;
	Edge(const Index3DId& vert0, const Index3DId& vert1);

	bool isValid() const;
	bool operator < (const Edge& rhs) const;
	bool operator == (const Edge& rhs) const;

	const Index3DId* getVertexIds() const;

	double sameParamTol(const Block* pBlock) const;
	double getLength(const Block* pBlock) const;
	Vector3d calCenter(const Block* pBlock) const;
	Vector3d calUnitDir(const Block* pBlock) const;
	Vector3d calPointAt(const Block* pBlock, double t) const;
	double paramOfPt(const Block* pBlock, const Vector3d& pt, bool& inBounds) const;
	Vector3d projectPt(const Block* pBlock, const Vector3d& pt) const;
	bool containsVertex(const Index3DId& vertexId) const;
	std::set<Index3DId> getFaceIds(const Block* pBlock) const;
	Index3DId getOtherVert(const Index3DId& vert) const;

	double intesectPlaneParam(const Block* pBlock, const Plane& splittingPlane) const;
	Index3DId splitAtParam(Block* pBlock, double t, std::set<Index3DId>& faceIds);
	Index3DId splitWithPlane(Block* pBlock, const Plane& splittingPlane, std::set<Index3DId>& faceIds);

private:
	Index3DId _vertexIds[2];
};

inline const Index3DId* Edge::getVertexIds() const
{
	return _vertexIds;
}

}
