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
#include <pool_vector.h>
#include <pool_set.h>
#include <objectPool.h>
#include <lambdaMacros.h>

template<class T>
class Plane;

namespace DFHM {

struct BuildCFDParams;
class Block;
class Edge;

class Polyhedron : public ObjectPoolOwnerUser {
public:
	Polyhedron() = default;
	Polyhedron(const MultiCore::set<Index3DId>& faceIds);
	explicit Polyhedron(const std::set<Index3DId>& faceIds);
	Polyhedron(const MultiCore::vector<Index3DId>& faceIds);
	explicit Polyhedron(const std::vector<Index3DId>& faceIds);
	Polyhedron(const Polyhedron& src);

	Polyhedron& operator = (const Polyhedron& rhs);

	void addFace(const Index3DId& faceId, size_t splitLevel);
	bool containsFace(const Index3DId& faceId) const;
	const MTC::set<Index3DId>& getFaceIds() const;
	void getVertIds(MTC::set<Index3DId>& vertIds) const;
	const MTC::set<Edge>& getEdges(bool includeAdjacentCellFaces) const;

	MTC::set<Index3DId> getAdjacentCells(bool includeCornerCells) const;

	// Gets the edges for a vertex which belong to this polyhedron
	void getVertEdges(const Index3DId& vertId, MTC::set<Edge>& edges, bool includeAdjacentCells) const;
	// Gets the faces for a vertex which belong to this polyhedron
	MTC::set<Index3DId> getVertFaces(const Index3DId& vertId) const;

	CBoundingBox3Dd getBoundingBox() const;
	void clearCache() const;
	bool contains(const Vector3d& pt) const;
	Vector3d calCentroid() const;
	bool intersectsModel() const;

	size_t createIntersectionFacePoints(const Planed& plane, MTC::vector<Vector3d>& points) const;
	Index3DId createIntersectionFace(const Planed& plane) const;

	void setNeedsDivideAtCentroid();
	bool needsDivideAtCentroid() const;

	// If this cell has any partial splits/split faces, it will mark it for centroid splitting.
	bool setNeedsCleanFaces();

	bool containsSharps() const;

	// Splitting functions are const to prevent reusing the split cell. After splitting, the cell should be removed from the block
	void addToSplitStack();
	void setNeedToMakeReference();
	bool setNeedToSplitConditional(const BuildCFDParams& params);
	bool needToDivideDueToSplitFaces(const BuildCFDParams& params);
	void setEdgeIndices(const std::vector<size_t>& indices);
	void setTriIndices(const std::vector<size_t>& indices);
	void getOutwardOrientedFaces(MTC::vector<Polygon>& faces) const;

	void imprintTVertices(Block* pDstBlock);
	void replaceFaces(const Index3DId& curFaceId, const MTC::set<Index3DId>& newFaceIds, size_t splitLevel);
	bool canSplit(MTC::set<Index3DId>& blockingCellIds) const;
	double getShortestEdge() const;

	size_t getSplitLevel() const;
	void setSplitLevel(size_t val);
	TopolgyState getState() const;
	size_t getNumSplitFaces() const;
	const std::vector<size_t>& getTriIndices() const;
	const std::vector<size_t>& getEdgeIndices() const;
	std::vector<size_t> getSharpVertIndices() const;

	void write(std::ostream& out) const;
	void read(std::istream& in);

	bool unload(std::ostream& out);
	bool load(std::istream& out);
	void dumpFaces() const;

	bool isClosed() const;
	bool hasTooManySplits() const;
	bool verifyTopology() const;
	bool operator < (const Polyhedron& rhs) const;

#if 0
	void faceMatchFunc(const Index3DId& id, std::function<void(const Polygon& obj)> func) const;
	void cellMatchFunc(const Index3DId& id, std::function<void(const Polyhedron& obj)> func) const;
#endif
	LAMBDA_CLIENT_DECLS
private:
	friend class Block;
	friend std::ostream& operator << (std::ostream& out, const Polyhedron& face);

	MTC::set<Edge> createEdgesFromVerts(MTC::vector<Index3DId>& vertIds) const;
	bool orderVertIds(MTC::vector<Index3DId>& vertIds) const;
	bool orderVertEdges(MTC::set<Edge>& edges, MTC::vector<Edge>& orderedEdges) const;
	void copyToOut() const;
	double calReferenceSurfaceRadius(const CBoundingBox3Dd& bbox, const BuildCFDParams& params) const;
	double minGap() const;
	bool polygonExists(TopolgyState refState, const Index3DId& id) const;
	bool intersect(LineSegmentd& seg, RayHitd& hit) const;

	MTC::set<Index3DId> _faceIds;
	size_t _splitLevel = 0;

	bool _needsSplitAtCentroid = false;
	std::vector<size_t> _triIndices, _edgeIndices;

	mutable bool _needsConditionalSplitTest = true;
	mutable MTC::set<Edge> _cachedEdges0, _cachedEdges1;
	mutable Trinary _cachedIsClosed = Trinary::IS_UNKNOWN;
	mutable bool _needsCurvatureCheck = true;
	mutable bool _cachedEdges0Vaild = false;
	mutable bool _cachedEdges1Vaild = false;
	mutable Trinary _intersectsModel = IS_UNKNOWN; // Cached value
	mutable double _cachedMinGap = -1;
};

inline const MTC::set<Index3DId>& Polyhedron::getFaceIds() const
{
	return _faceIds;
}

inline bool Polyhedron::containsFace(const Index3DId& faceId) const
{
	return _faceIds.count(faceId) != 0;
}

inline size_t Polyhedron::getSplitLevel() const
{
	return _splitLevel;
}

inline void Polyhedron::setSplitLevel(size_t val)
{
	_splitLevel = val;
}

inline const std::vector<size_t>& Polyhedron::getTriIndices() const
{
	return _triIndices;
}

inline const std::vector<size_t>& Polyhedron::getEdgeIndices() const
{
	return _edgeIndices;
}

std::ostream& operator << (std::ostream& out, const Polyhedron& cell);

}

