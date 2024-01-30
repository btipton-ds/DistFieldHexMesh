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
	Edge(ObjectPoolOwner* pBlock, const Index3DId& vert0, const Index3DId& vert1);

	bool isValid() const;
	bool operator < (const Edge& rhs) const;
	bool operator == (const Edge& rhs) const;

	const Index3DId* getVertexIds() const;

	double sameParamTol() const;
	double getLength() const;
	Vector3d calCenter() const;
	Vector3d calPointAt(double t) const;
	double paramOfPt(const Vector3d& pt, bool& inBounds) const;
	Vector3d projectPt(const Vector3d& pt) const;
	bool containsVertex(const Index3DId& vertexId) const;
	std::set<Index3DId> getFaceIds() const;
	Index3DId getOtherVert(const Index3DId& vert) const;

	double intesectPlaneParam(const Plane& splittingPlane) const;
	Index3DId splitAtParam(double t) const;
	Index3DId splitWithPlane(const Plane& splittingPlane) const;

private:
	Block* _pBlock = nullptr;
	Index3DId _vertexIds[2];
};

inline const Index3DId* Edge::getVertexIds() const
{
	return _vertexIds;
}

}
