#pragma once

#include <vector>
#include <set>
#include <index3D.h>

struct Plane;

namespace DFHM {

class Block;

class Edge {
public:

	Edge() = default;
	Edge(const Edge& src) = default;
	Edge(const Index3DId& vert0, const Index3DId& vert1);

	bool operator < (const Edge& rhs) const;
	bool operator == (const Edge& rhs) const;

	const Index3DId* getVertexIds() const;

	double sameParamTol(const Block* pBlock) const;
	double getLength(const Block* pBlock) const;
	Vector3d calCenter(const Block* pBlock) const;
	Vector3d calPointAt(const Block* pBlock, double t) const;
	double paramOfPt(const Block* pBlock, const Vector3d& pt, bool& inBounds) const;
	Vector3d projectPt(const Block* pBlock, const Vector3d& pt) const;
	bool containsVertex(const Index3DId& vert) const;
	std::set<Index3DId> getFaceIds(const Block* pBlock) const;
	std::set<Index3DId> getFaceIds(const Block* pBlock, std::set<Index3DId>& availFaces) const;
	Index3DId getCommonFace(const Block* pBlock) const;

	double intesectPlaneParam(Block* pBlock, const Plane& splittingPlane) const;
	Index3DId splitAtParam(Block* pBlock, double t) const;
	Index3DId splitWithPlane(Block* pBlock, const Plane& splittingPlane) const;

private:
	Index3DId _vertexIds[2];
};

inline Edge::Edge(const Index3DId& vert0, const Index3DId& vert1)
{
	if (vert0 < vert1) {
		_vertexIds[0] = vert0;
		_vertexIds[1] = vert1;
	} else {
		_vertexIds[0] = vert1;
		_vertexIds[1] = vert0;
	}
}

inline const Index3DId* Edge::getVertexIds() const
{
	return _vertexIds;
}

}
