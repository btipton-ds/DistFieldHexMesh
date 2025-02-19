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
#include <pool_set.h>
#include <index3D.h>
#include <objectPool.h>
#include <iostream>
#include <fastBisectionSet.h>

template<class T>
class Plane;
template<class T>
struct LineSegment;
using LineSegmentd = LineSegment<double>;

namespace DFHM {

class Block;
class Polyhedron;

class Edge {
public:

	Edge() = default;
	Edge(const Edge& src) = default;
	Edge(const Index3DId& vert0, const Index3DId& vert1, const MTC::set<Index3DId>& faceIds = MTC::set<Index3DId>());
	Edge(const Edge& src, const MTC::set<Index3DId>& faceIds);

	void setBlockPtr(const Block* pBlock) const;
	const Block* getBlockPtr() const;

	bool isValid() const;
	bool operator < (const Edge& rhs) const;
	bool operator == (const Edge& rhs) const;
	bool operator != (const Edge& rhs) const;

	const Index3DId& getVertex(size_t idx) const;
	const Index3DId* getVertexIds() const;
	bool containsVertex(const Index3DId& vertexId) const;
	bool vertexLiesOnEdge(const Block* pBlock, const Index3DId& vertexId) const;
	bool pointLiesOnEdge(const Block* pBlock, const Vector3d& pt) const;
	const FastBisectionSet<Index3DId>& getFaceIds() const;
	void getFaceIds(FastBisectionSet<Index3DId>& faceIds) const;
	void getCellIds(const Block* pBlock, MTC::set<Index3DId>& cellIds) const;
	Index3DId getOtherVert(const Index3DId& vert) const;

	double sameParamTol(const Block* pBlock) const;
	double getLength(const Block* pBlock) const;
	Vector3d calCenter(const Block* pBlock) const;
	Vector3d calUnitDir(const Block* pBlock) const;
	Vector3d calCoedgeUnitDir(const Block* pBlock, const Index3DId& faceId, const Index3DId& cellId) const;
	Vector3d calPointAt(const Block* pBlock, double t) const;
	double paramOfPt(const Block* pBlock, const Vector3d& pt, bool& inBounds) const;
	Vector3d projectPt(const Block* pBlock, const Vector3d& pt) const;
	bool onPrincipalAxis(const Block* pBlock) const;
	bool isColinearWith(const Block* pBlock, const Edge& other) const;
	bool isColinearWith(const Block* pBlock, const Index3DId& vert, double& param) const;
	bool isConnectedTo(const Edge& other) const;
	LineSegmentd getSegment(const Block* pBlock) const;

	/*
	The dihedral angle is defined so that 
		coplanar faces with aligned normals have angle == 0
		external, perpendicular faces have angle == pi / 2
		internal, perpendicular faces have angle == -pi / 2
		external, highly acute faces have angle ~ 0.99 * pi
		internal, highly acute faces have angle ~ -0.99 * pi
		Positive angles are convex, negative are concave

		This corresponds to a lofting definition of a negative surface having concave regions.
	*/
	double calDihedralAngleRadians(const Block* pBlock, const Index3DId& refCellId) const;
	bool isConvex(const Block* pBlock, const Index3DId& refCellId) const;
	bool isOriented(const Block* pBlock, const Index3DId& refCellId) const;

	void write(std::ostream& out) const;
	void read(std::istream& in);

private:
	Index3DId _vertexIds[2];
	FastBisectionSet<Index3DId> _faceIds;
	bool _reversed = false;
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

inline const FastBisectionSet<Index3DId>& Edge::getFaceIds() const
{
	return _faceIds;
}

inline const Index3DId& Edge::getVertex(size_t idx) const
{
	return _vertexIds[idx];
}

inline const Index3DId* Edge::getVertexIds() const
{
	return _vertexIds;
}

std::ostream& operator << (std::ostream& out, const Edge& edge);

}
