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

#include <iostream>
#include <triMesh.h>
#include <tm_boundingBox.h>
#include <index3D.h>
#include <objectPool.h>
#include <lambdaMacros.h>

namespace DFHM {

class Block;
class Edge;

class Polyhedron : public ObjectPoolOwnerUser {
public:
	Polyhedron() = default;
	Polyhedron(const std::set<Index3DId>& faceIds);
	Polyhedron(const std::vector<Index3DId>& faceIds);
	Polyhedron(const Polyhedron& src) = default;

	void addFace(const Index3DId& faceId);
	bool removeFace(const Index3DId& faceId);
	bool containsFace(const Index3DId& faceId) const;
	const std::set<Index3DId>& getFaceIds() const;
	void getVertIds(std::set<Index3DId>& vertIds) const;
	void getEdges(std::set<Edge>& edgeSet, bool includeAdjacentCellFaces) const;

	std::set<Index3DId> getAdjacentCells(bool includeCornerCells) const;

	// Gets the edges for a vertex which belong to this polyhedron
	void getVertEdges(const Index3DId& vertId, std::set<Edge>& edges, bool includeAdjacentCells) const;
	// Gets the faces for a vertex which belong to this polyhedron
	std::set<Index3DId> getVertFaces(const Index3DId& vertId) const;

	CBoundingBox3Dd getBoundingBox() const;
	bool contains(const Vector3d& pt) const;
	Vector3d calCentroid() const;
	bool intersectsModel() const;
	bool requiresMultipleFaceSplit() const;

	// Splitting functions are const to prevent reusing the split cell. After splitting, the cell should be removed from the block
	void setNeedToSplitAtCentroid();
	void setNeedToMakeReference();
	void setNeedToSplitCurvature(int divsPerRadius, double maxCurvatureRadius, double sinEdgeAngle);

	void splitAtCentroid(Block* pDstBlock) const;
	void splitAtPoint(Block* pDstBlock, const Vector3d& pt) const;
	double getShortestEdge() const;

	bool unload(std::ostream& out);
	bool load(std::istream& out);
	void dumpFaces() const;

	bool isClosed() const;
	bool verifyTopology() const;
	bool operator < (const Polyhedron& rhs) const;

	LAMBDA_CLIENT_DECLS
private:
	friend class Block;
	friend std::ostream& operator << (std::ostream& out, const Polyhedron& face);

	std::set<Edge> createEdgesFromVerts(std::vector<Index3DId>& vertIds) const;
	bool orderVertIds(std::vector<Index3DId>& vertIds) const;
	bool orderVertEdges(std::set<Edge>& edges, std::vector<Edge>& orderedEdges) const;
	void copyToOut() const;
	double calReferenceSurfaceRadius(const CBoundingBox3Dd& bbox, double maxCurvatureRadius, double sinEdgeAngle) const;

	mutable Trinary _intersectsModel = IS_UNKNOWN; // Cached value
	bool _needsCurvatureCheck = true;

	std::set<Index3DId> _faceIds;
};

inline const std::set<Index3DId>& Polyhedron::getFaceIds() const
{
	return _faceIds;
}

inline bool Polyhedron::containsFace(const Index3DId& faceId) const
{
	return _faceIds.count(faceId) != 0;
}

//LAMBDA_CLIENT_IMPLS(Polyhedron)
template<class LAMBDA> 
inline void Polyhedron::vertexFunc(const Index3DId& id, LAMBDA func) const {
	getBlockPtr()->vertexFunc(id, func);
} 

template<class LAMBDA> 
inline void Polyhedron::vertexFunc(const Index3DId& id, LAMBDA func) {
	getBlockPtr()->vertexFunc(id, func);
} 

template<class LAMBDA> 
inline void Polyhedron::faceFunc(const Index3DId& id, LAMBDA func) const {
	getBlockPtr()->faceFunc(id, func);
} 

template<class LAMBDA> 
inline void Polyhedron::faceFunc(const Index3DId& id, LAMBDA func) {
	getBlockPtr()->faceFunc(id, func);
} 

template<class LAMBDA> 
inline void Polyhedron::faceRefFunc(const Index3DId& id, LAMBDA func) const {
	getBlockPtr()->faceRefFunc(id, func);
} 

template<class LAMBDA> 
inline void Polyhedron::cellFunc(const Index3DId& id, LAMBDA func) const {
	getBlockPtr()->cellFunc(id, func);
} 

template<class LAMBDA> 
inline void Polyhedron::cellFunc(const Index3DId& id, LAMBDA func) {
	getBlockPtr()->cellFunc(id, func);
} 

template<class LAMBDA> 
inline void Polyhedron::cellRefFunc(const Index3DId& id, LAMBDA func) const {
	getBlockPtr()->cellRefFunc(id, func);
}

std::ostream& operator << (std::ostream& out, const Polyhedron& cell);

}

