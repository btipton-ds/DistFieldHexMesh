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
#include <fastBisectionSet.h>

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

	void clear() override;

	Polyhedron& operator = (const Polyhedron& rhs);

	void remapId(const std::vector<size_t>& idRemap, const Index3D& srcDims) override;

	void addFace(const Index3DId& faceId, size_t splitLevel);
	const FastBisectionSet<Index3DId>& getFaceIds() const;
	size_t getNumFaces() const;
	const FastBisectionSet<Index3DId>& getVertIds() const;
	const FastBisectionSet<Edge>& getEdges(bool includeAdjacentCellFaces) const;

	const FastBisectionSet<Index3DId>& getAdjacentCells() const;

	// Gets the edges for a vertex which belong to this polyhedron
	void getVertEdges(const Index3DId& vertId, FastBisectionSet<Edge>& edges, bool includeAdjacentCells) const;
	// Gets the faces for a vertex which belong to this polyhedron
	FastBisectionSet<Index3DId> getVertFaces(const Index3DId& vertId) const;

	CBoundingBox3Dd getBoundingBox() const;
	void clearCache() const;
	void clearAdjCellIdCache() const;
	bool contains(const Vector3d& pt) const;
	bool containsVertex(const Index3DId& vertId) const;
	bool containsFace(const Index3DId& faceId) const;
	bool isVertexConnectedToCell(const Index3DId& cellId) const;
	bool isVertexConnectedToFace(const Index3DId& faceId) const;
	Vector3d calCentroid() const;
	Vector3d calCentroidApproxFast() const;
	double calVolume() const;
	bool isClosed() const;
	bool isOriented() const;
	bool exists() const;
	size_t classify(MTC::vector<Vector3d>& corners) const;
	void classifyEdges(MTC::set<Edge>& convexEdges, MTC::set<Edge>& concaveEdges) const;
	bool isConvex() const;
	bool intersectsModel() const;		 // Uses bounding box
	bool sharpEdgesIntersectModel(const BuildCFDParams& params) const;

	void setNeedsDivideAtCentroid();
	bool needsDivideAtCentroid() const;

	// If this cell has any partial splits/split faces, it will mark it for centroid splitting.
	bool setNeedsCleanFaces();

	bool containsSharps() const;

	// Splitting functions are const to prevent reusing the split cell. After splitting, the cell should be removed from the block
	void addToSplitStack();
	void setNeedToMakeReference();
	bool setNeedToSplitConditional(size_t passNum, const BuildCFDParams& params);
	bool needToDivideDueToSplitFaces(const BuildCFDParams& params);
	void orientFaces();

	void imprintTVertices(Block* pDstBlock);
	void attachFaces();
	void detachFaces();
	bool canSplit(MTC::set<Index3DId>& blockingCellIds) const;
	double getShortestEdge() const;

	size_t getSplitLevel() const;
	void setSplitLevel(size_t val);
	int32_t getLayerNum() const;
	void clearLayerNum();
	bool setLayerNum(int i, bool propagate);

	size_t getNumSplitFaces() const;
	MTC::vector<size_t> getSharpVertIndices() const;
	bool getSharpEdgeIndices(MTC::vector<size_t>& result, const BuildCFDParams& params) const;

	void write(std::ostream& out) const;
	void read(std::istream& in);

	bool unload(std::ostream& out);
	bool load(std::istream& out);
	void dumpFaces() const;

	bool lineSegmentIntersects(const LineSegmentd& seg, MTC::vector<RayHitd>& hits, MTC::vector<Index3DId>& faceIds) const;
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
	bool polygonExists(const Index3DId& id) const;
	bool intersect(LineSegmentd& seg, RayHitd& hit) const;
	Vector3d getVertexPoint(const Index3DId& vertId) const;

	/* 
		The faceIds are invariant.
		If a face is split, it's id stays in the list, but the face itself records it's been split and into which faces
		The original face is marked as split, no longer really exists, and points to it's replacements.
		The same applies for faces with split edges, but the number of faces is only 1.
	*/
	FastBisectionSet<Index3DId> _faceIds;
	size_t _splitLevel = 0;
	int32_t _layerNum = -1;

	bool _needsSplitAtCentroid = false;
	bool _exists = true;

	mutable FastBisectionSet<Edge> _cachedEdges0, _cachedEdges1;

	mutable FastBisectionSet<Index3DId> _cachedAdjCellIds;
	mutable FastBisectionSet<Index3DId> _cachedVertIds;

	mutable bool _isOriented = false;
	mutable bool _needsCurvatureCheck = true;

	mutable Trinary _cachedIsClosed = Trinary::IS_UNKNOWN;
	mutable Trinary _intersectsModel = IS_UNKNOWN; // Cached value
	mutable Trinary _sharpEdgesIntersectModel = IS_UNKNOWN; // Cached value

	mutable double _cachedMinGap = -1;
};

inline const FastBisectionSet<Index3DId>& Polyhedron::getFaceIds() const
{
	return _faceIds;
}

inline size_t Polyhedron::getNumFaces() const
{
	return _faceIds.size();
}

inline bool Polyhedron::containsFace(const Index3DId& faceId) const
{
	return _faceIds.contains(faceId) != 0;
}

inline size_t Polyhedron::getSplitLevel() const
{
	return _splitLevel;
}

inline int32_t Polyhedron::getLayerNum() const
{
	return _layerNum;
}

inline void Polyhedron::setSplitLevel(size_t val)
{
	_splitLevel = val;
}

std::ostream& operator << (std::ostream& out, const Polyhedron& cell);

}

