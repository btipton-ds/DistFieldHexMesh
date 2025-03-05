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
#include <iostream>
#include <fastBisectionSet.h>
#include <pool_set.h>
#include <index3D.h>
#include <objectPool.h>
#include <lambdaMacros.h>

template<class T>
class Plane;
template<class T>
struct LineSegment;
using LineSegmentd = LineSegment<double>;

namespace DFHM {

class Block;
class Vertex;
class Polygon;
class Polyhedron;

class EdgeKey {
public:
	EdgeKey() = default;
	EdgeKey(const EdgeKey& src) = default;
	EdgeKey(const Index3DId& vert0, const Index3DId& vert1);

	bool isValid() const;
	bool operator < (const EdgeKey& rhs) const;
	bool operator == (const EdgeKey& rhs) const;
	bool operator != (const EdgeKey& rhs) const;

	const Index3DId& getVertex(size_t idx) const;
	const Index3DId* getVertexIds() const;
	bool containsVertex(const Index3DId& vertexId) const;
	Index3DId getOtherVert(const Index3DId& vert) const;

protected:
	Index3DId _vertexIds[2];
};

class Edge : public EdgeKey, public ObjectPoolOwnerUser {
public:

	Edge() = default;
	Edge(const Edge& src) = default;
	Edge(const EdgeKey& src);
	Edge(const Index3DId& vert0, const Index3DId& vert1, const MTC::set<Index3DId>& faceIds = MTC::set<Index3DId>());
	Edge(const Edge& src, const FastBisectionSet<Index3DId>& faceIds);
	Edge(const Edge& src, const MTC::set<Index3DId>& faceIds);

	Index3DId getId() const override;
	void remapId(const std::vector<size_t>& idRemap, const Index3D& srcDims) override;

	void addFaceId(const Index3DId& faceId);
	void removeFaceId(const Index3DId& faceId);

	bool vertexLiesOnEdge(const Index3DId& vertexId) const;
	bool pointLiesOnEdge(const Vector3d& pt) const;
	const FastBisectionSet<Index3DId>& getFaceIds() const;
	MTC::set<Index3DId> getCellIds() const;

	double sameParamTol() const;
	double getLength() const;
	Vector3d calCenter() const;
	Vector3d calUnitDir() const;
	Vector3d calCoedgeUnitDir(const Index3DId& faceId, const Index3DId& cellId) const;
	Vector3d calPointAt(double t) const;
	double paramOfPt(const Vector3d& pt, bool& inBounds) const;
	Vector3d projectPt(const Vector3d& pt) const;
	bool onPrincipalAxis() const;
	bool isColinearWith(const Edge& other) const;
	bool isColinearWith(const Index3DId& vert, double& param) const;
	bool isConnectedTo(const Edge& other) const;
	LineSegmentd getSegment() const;

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
	double calDihedralAngleRadians(const Index3DId& refCellId) const;
	bool isConvex(const Index3DId& refCellId) const;
	bool isOriented(const Index3DId& refCellId) const;

	void write(std::ostream& out) const;
	void read(std::istream& in);

	LAMBDA_CLIENT_DECLS

protected:
	void setId(const Index3DId& id) override;

private:
	FastBisectionSet<Index3DId> _faceIds;
};

inline const FastBisectionSet<Index3DId>& Edge::getFaceIds() const
{
	return _faceIds;
}

inline const Index3DId& EdgeKey::getVertex(size_t idx) const
{
	return _vertexIds[idx];
}

inline const Index3DId* EdgeKey::getVertexIds() const
{
	return _vertexIds;
}

inline void Edge::addFaceId(const Index3DId& faceId)
{
	_faceIds.insert(faceId);
}

inline void Edge::removeFaceId(const Index3DId& faceId)
{
	_faceIds.erase(faceId);
}

std::ostream& operator << (std::ostream& out, const Edge& edge);

}
