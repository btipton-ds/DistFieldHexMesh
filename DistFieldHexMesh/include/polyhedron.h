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

/*
	Mark all faces and cells to split on one pass
	Split all the faces with a map from old faces to new faces
	Split all the cells usnig the map of old to new to populate the new cells

*/
// Polyhedra are owned by a single block, but their faces may belong to different blocks.
// Once a polyhedron is split, it is kept for reference but is otherwise dead.
// If a face or edge is split, the polyhedron must also be split.
// If a polyhdron has been split, it can be split again but DOES NOT become reference.

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
	const std::set<Index3DId>& getFaceIds() const;
	void getVertIds(std::set<Index3DId>& vertIds) const;
	void getEdges(std::set<Edge>& edgeSet, bool includeNeighborFaces) const;

	std::set<Index3DId> getAdjacentCells(bool includeCornerCells) const;

	// Gets the edges for a vertex which belong to this polyhedron
	void getVertEdges(const Index3DId& vertId, std::set<Edge>& edges, bool includeAdjacentCells) const;
	// Gets the faces for a vertex which belong to this polyhedron
	std::set<Index3DId> getVertFaces(const Index3DId& vertId) const;

	CBoundingBox3Dd getBoundingBox() const;
	bool contains(const Vector3d& pt) const;
	Vector3d calCentroid() const;
	bool intersectsModel() const;
	bool hasSplits() const;
	Trinary needsSplit() const;

	// Splitting functions are const to prevent reusing the split cell. After splitting, the cell should be removed from the block
	void setNeedToSplitAtCentroid(bool val);
	void setNeedToSplitCurvature(int divsPerRadius, double maxCurvatureRadius, double sinEdgeAngle);
	void setPolygonsCellId();
	void splitIfRequred();
	void replaceSplitFaces();
	void imprintVertices();
	bool splitAtCentroid(std::set<Index3DId>& newCellIds);
	bool splitAtPoint(const Vector3d& pt, std::set<Index3DId>& newCellIds);
	void markIfNeedsSplitByCurvature(int divsPerRadius, double maxCurvatureRadius, double sinEdgeAngle);
	void splitByCurvature(int divsPerRadius, double maxCurvatureRadius, double sinEdgeAngle);
	void splitIfTooManyFaceSplits();
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

	void setSourceId(const Index3DId& id);
	void setReference();
	std::set<Edge> createEdgesFromVerts(std::vector<Index3DId>& vertIds) const;
	bool orderVertIds(std::vector<Index3DId>& vertIds) const;
	bool orderVertEdges(std::set<Edge>& edges, std::vector<Edge>& orderedEdges) const;
	void copyToOut() const;
	Index3DId createFace(const std::vector<Index3DId>& vertIds);
	void createHexahedralFaces(const std::vector<Index3DId>& cornerIds, std::vector<Index3DId>& faceIds);
	double calReferenceSurfaceRadius(const CBoundingBox3Dd& bbox, double maxCurvatureRadius, double sinEdgeAngle) const;
	Index3DId addFace(const std::vector<Index3DId>& vertIds);

	mutable Trinary _intersectsModel = IS_UNKNOWN; // Cached value
	bool _needsCurvatureCheck = true, 
		_isReference = false;
	Trinary _needsSplit = Trinary::IS_UNKNOWN;
	Index3DId _sourceId; // This is the id of the polyhedron this polyhedron was split from - may be null
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

inline bool Polyhedron::containsFace(const Index3DId& faceId) const
{
	return _faceIds.count(faceId) != 0;
}

inline Trinary Polyhedron::needsSplit() const
{
	return _needsSplit;
}

}

