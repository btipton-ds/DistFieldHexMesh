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

	Copyright Robert R Tipton, 2022, all rights reserved except those granted in prior license statement.

	Dark Sky Innovative Solutions http://darkskyinnovation.com/
*/

#include <vector>
#include <set>
#include <iostream>
#include <fastBisectionSet.h>
#include <pool_set.h>
#include <index3D.h>
#include <lambdaMacros.h>
#include <edgeKey.h>

template<class T>
class Plane;
template<class T>
struct LineSegment;
using LineSegmentd = LineSegment<double>;

namespace DFHM {

struct SplittingParams;
class Block;
class PolyMesh;
class Vertex;
class Polygon;
class Polyhedron;
class Edge;

// EdgeStorage holds data for an Edge which is not used by an EdgeKey.
// It is not REQUIRED, but having it assures that _faceIds can only be accessed by getFaceIds() 
// which assures that _faceIds is initialized first.
// Lack of this led to several lost days fixing a new function which did not call initFaceIds().
class EdgeStorage : public EdgeKey {
public:
	EdgeStorage(const EdgeStorage& src) = default;
	EdgeStorage(const EdgeKey& src, const Block* pBlock);
	EdgeStorage(const EdgeKey& src, const PolyMesh* pBlock);
	EdgeStorage(const EdgeStorage& src, const Block* pBlock);
	EdgeStorage(const EdgeStorage& src, const PolyMesh* pBlock);
	const MTC::set<Index3DId>& getFaceIds() const;
	MTC::set<Index3DId> getCellIds() const;

	LAMBDA_CLIENT_DECLS

protected:
	Block* getBlockPtr();
	const Block* getBlockPtr() const;

	const PolyMesh* getPolyMeshPtr() const;
	PolyMesh* getPolyMeshPtr();

	void initFaceIds() const;

private:
	Block* _pBlock = nullptr;
	PolyMesh* _pPolyMesh = nullptr;
	mutable MTC::set<Index3DId> _faceIds;
};

inline Block* EdgeStorage::getBlockPtr()
{
	return _pBlock;
}

inline const Block* EdgeStorage::getBlockPtr() const
{
	return _pBlock;
}

inline const PolyMesh* EdgeStorage::getPolyMeshPtr() const
{
	return _pPolyMesh;
}

inline PolyMesh* EdgeStorage::getPolyMeshPtr()
{
	return _pPolyMesh;
}

inline const MTC::set<Index3DId>& EdgeStorage::getFaceIds() const
{
	initFaceIds();
	return _faceIds;
}

/*******************************************************************************************************/

class Edge : public EdgeStorage {
public:

	Edge(const Edge& src) = default;
	Edge(const EdgeKey& src, const Block* pBlock);
	Edge(const EdgeKey& src, const PolyMesh* pBlock);

	bool vertexLiesOnEdge(const Index3DId& vertexId) const;
	bool pointLiesOnEdge(const Vector3d& pt) const;

	double sameParamTol() const;
	double getLength() const;
	Vector3d calCenter() const;
	Vector3d calUnitDir() const;
	Vector3d calCoedgeUnitDir(const Index3DId& faceId, const Index3DId& cellId) const;
	Vector3d calPointAt(double t) const;
	double paramOfPt(const Vector3d& pt, bool& inBounds) const;
	double calLength() const;
	double calCurvature(const SplittingParams& params) const;

	Vector3d projectPt(const Vector3d& pt) const;
	bool onPrincipalAxis() const;
	bool isColinearWith(const Edge& other) const;
	bool isColinearWith(const Index3DId& vert, double& param) const;
	bool isConnectedTo(const Edge& other) const;
	bool containsFace(const Index3DId& faceId) const;
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

	bool imprintVertex(const Index3DId& vertId);

	void write(std::ostream& out) const;
	void read(std::istream& in);

	bool verifyTopology() const;

private:
	const Vector3d& getVertexPoint(const Index3DId& id) const;
	double calCurvature(const Vector3d& origin, const Vector3d& ptAxis, const Vector3d& pt0, const Vector3d& pt1, const SplittingParams& params) const;
};

std::ostream& operator << (std::ostream& out, const EdgeKey& edge);

}
