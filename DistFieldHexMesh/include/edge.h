#pragma once

/*
This file is part of the DistFieldHexMesh application/library.

	The DistFieldHexMesh application/library is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	The DistFieldHexMesh application/library is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	This link provides the exact terms of the GPL license <https://www.gnu.org/licenses/>.

	The author's interpretation of GPL 3 is that if you receive money for the use or distribution of the DistFieldHexMesh application/library or a derivative product, GPL 3 no longer applies.

	Under those circumstances, the author expects and may legally pursue a reasoble share of the income. To avoid the complexity of agreements and negotiation, the author makes
	no specific demands in this regard. Compensation of roughly 1% of net or $5 per user license seems appropriate, but is not legally binding.

	In lay terms, if you make a profit by using the DistFieldHexMesh application/library (violating the spirit of Open Source Software), I expect a reasonable share for my efforts.

	Robert R Tipton - Author

	Dark Sky Innovative Solutions http://darkskyinnovation.com/
*/

#include <vector>
#include <set>
#include <index3D.h>
#include <objectPool.h>
#include <iostream>

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

	void setBlockPtr(const Block* pBlock) const;
	const Block* getBlockPtr() const;

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
	bool isColinearWith(const Block* pBlock, const Index3DId& vert, double& param) const;
	bool isConnectedTo(const Edge& other) const;
	double calSinDihedralAngle(const Block* pBlock) const;
	LineSegment getSegment(const Block* pBlock) const;

private:
	Index3DId _vertexIds[2];
	std::set<Index3DId> _faceIds;
	mutable const Block* _pBlock;
};

inline void Edge::setBlockPtr(const Block* pBlock) const
{
	_pBlock = pBlock;
}

inline const Block* Edge::getBlockPtr() const
{
	return _pBlock;
}

inline const std::set<Index3DId>& Edge::getFaceIds() const
{
	return _faceIds;
}

inline const Index3DId* Edge::getVertexIds() const
{
	return _vertexIds;
}

std::ostream& operator << (std::ostream& out, const Edge& edge);

}
