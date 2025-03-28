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
#include <memory>
#include <triMesh.h>
#include <tm_boundingBox.h>
#include <index3D.h>
#include <pool_vector.h>
#include <pool_set.h>
#include <objectPool.h>
#include <lambdaMacros.h>
#include <fastBisectionSet.h>
#include <meshData.h>

template<class T>
class Plane;

namespace DFHM {

struct SplittingParams;
class Block;
class Edge;

class Polyhedron : public ObjectPoolOwnerUser {
public:
	Polyhedron() = default;
	Polyhedron(const MultiCore::set<Index3DId>& faceIds, const MultiCore::vector<Index3DId>& cornerVertIds = MultiCore::vector<Index3DId>());
	explicit Polyhedron(const std::set<Index3DId>& faceIds, const std::vector<Index3DId>& cornerVertIds = std::vector<Index3DId>());
	Polyhedron(const MultiCore::vector<Index3DId>& faceIds, const MultiCore::vector<Index3DId>& cornerVertIds = MultiCore::vector<Index3DId>());
	explicit Polyhedron(const std::vector<Index3DId>& faceIds, const std::vector<Index3DId>& cornerVertIds = std::vector<Index3DId>());
	Polyhedron(const Polyhedron& src);

	void clear() override;

	Polyhedron& operator = (const Polyhedron& rhs);

	const Index3DId& getId() const override;
	void remapId(const std::vector<size_t>& idRemap, const Index3D& srcDims) override;

	void addFace(const Index3DId& faceId);
	void removeFace(const Index3DId& faceId);
	const FastBisectionSet<Index3DId>& getFaceIds() const;
	size_t getNumFaces() const;
	size_t getVertIds(MTC::set<Index3DId>& result) const;
	const MTC::vector<Index3DId>& getCanonicalVertIds() const;
	FastBisectionSet<EdgeKey> getEdgeKeys(bool includeAdjacentCellFaces) const;

	FastBisectionSet<Index3DId> getAdjacentCells() const;

	// Gets the edges for a vertex which belong to this polyhedron
	void getVertEdges(const Index3DId& vertId, FastBisectionSet<EdgeKey>& edges, bool includeAdjacentCells) const;
	// Gets the faces for a vertex which belong to this polyhedron
	FastBisectionSet<Index3DId> getVertFaces(const Index3DId& vertId) const;

	CBoundingBox3Dd getBoundingBox() const;
	void clearCache() const;
	void updateAllTopolCaches() const;

	bool containsVertex(const Index3DId& vertId) const;
	bool containsFace(const Index3DId& faceId) const;
	bool isVertexConnectedToCell(const Index3DId& cellId) const;
	const Vector3d& calCentroid() const;
	Vector3d calCentroidApprox() const;
	double calVolume() const;
	double calMaxCurvature2D(const MTC::vector<Vector3d>& quadPoints, int axis) const;
	bool containsHighCurvatureTris(const SplittingParams& params) const;
	bool isClosed() const;
	bool isOriented() const;
	bool exists() const;
	size_t classify(MTC::vector<Vector3d>& corners) const;
	void classifyEdges(MTC::set<EdgeKey>& convexEdges, MTC::set<EdgeKey>& concaveEdges) const;
	bool isConvex() const;
	bool pointInside(const Vector3d& pt) const;
	bool intersectsModel() const;
	bool sharpEdgesIntersectModel(const SplittingParams& params) const;
#if USE_CELL_HISTOGRAM
	void addToFaceCountHistogram(std::map<size_t, size_t>& histo) const;
#endif
	void createPlanarFaceSet(MTC::vector<MTC::set<Index3DId>>& planarFaceSet) const;
	bool isTooComplex(const SplittingParams& params) const;
	bool isTooComplex(const SplittingParams& params, MTC::vector<MTC::set<Index3DId>>& planarFaceSet) const;
	double maxNonOrthogonality() const;

	void setNeedsDivideAtCentroid();
	bool needsDivideAtCentroid() const;

	// If this cell has any partial splits/split faces, it will mark it for centroid splitting.
	bool setNeedsCleanFaces();

	bool containsSharps() const;

	// Splitting functions are const to prevent reusing the split cell. After splitting, the cell should be removed from the block
	void addToSplitStack();
	bool setNeedToSplitConditional(size_t passNum, const SplittingParams& params);
	void orientFaces();

	void attachFaces();
	void detachFaces();

	void connectVertEdgeTopology();
	void disconnectVertEdgeTopology();

	void imprintFaceEdges(const Index3DId& imptintFaceId, FastBisectionSet<Index3DId>& touchedCellIds);

	int32_t getLayerNum() const;
	void clearLayerNum();
	void setLayerNum(int32_t val, bool force);
	void setLayerNumOnNextPass(int32_t val);

	MTC::vector<size_t> getSharpVertIndices() const;
	bool getSharpEdgeIndices(MTC::vector<size_t>& result, const SplittingParams& params) const;

	void write(std::ostream& out) const;
	void read(std::istream& in);

	bool unload(std::ostream& out);
	bool load(std::istream& out);
	void dumpFaces() const;

	bool lineSegmentIntersects(const LineSegmentd& seg, MTC::vector<RayHitd>& hits, MTC::vector<Index3DId>& faceIds) const;
	bool verifyTopology() const;
	bool operator < (const Polyhedron& rhs) const;

#if USE_CELL_HISTOGRAM
	inline void setParentId(const Index3DId& id)
	{
		_parentIds.push_back(id);
	}
	const std::vector<Index3DId>& getParentIds() const
	{
		return _parentIds;
	}
#endif
	LAMBDA_CLIENT_DECLS

protected:
	void setId(const Index3DId& id) override;

private:
	friend class Block;
	friend std::ostream& operator << (std::ostream& out, const Polyhedron& face);

	void updateCachedVerts() const;

	MTC::set<EdgeKey> createEdgesFromVerts(MTC::vector<Index3DId>& vertIds) const;
	bool orderVertIds(MTC::vector<Index3DId>& vertIds) const;
	void copyToOut() const;
	double calReferenceSurfaceRadius(const CBoundingBox3Dd& bbox, const SplittingParams& params) const;
	double minGap() const;
	bool polygonExists(const Index3DId& id) const;
	const Vector3d& getVertexPoint(const Index3DId& vertId) const;

	Index3DId _thisId;
	/*
		The faceIds are invariant.
		If a face is split, it's id stays in the list, but the face itself records it's been split and into which faces
		The original face is marked as split, no longer really exists, and points to it's replacements.
		The same applies for faces with split edges, but the number of faces is only 1.
	*/
#if USE_CELL_HISTOGRAM
	std::vector<Index3DId> _parentIds;
#endif
	FastBisectionSet<Index3DId> _faceIds;

	// _canonicalVertices are the vertices that define the polyhedron 4 = tet, 5 = pyramid, 6 = triangular prism, 8 = hexahedron, 12 = hexagonal cylinder
	// If it's empty, the cell is not canonical
	MTC::vector<Index3DId> _canonicalVertices; 

	size_t _splitLevel = 0;
	int32_t _layerNum = -1; // -1 is not set yet, -2 is mark for setting on set pass

	bool _needsSplitAtCentroid = false;
	bool _exists = true;

	mutable bool _isOriented = false;
	mutable bool _needsCurvatureCheck = true;

	mutable Trinary _cachedIsClosed = IS_UNKNOWN;
	mutable Trinary _intersectsModel = IS_UNKNOWN; // Cached value
	mutable Trinary _sharpEdgesIntersectModel = IS_UNKNOWN; // Cached value

	mutable double _cachedMinGap = -1;

	mutable Vector3d _cachedCtr = Vector3d(DBL_MAX, DBL_MAX, DBL_MAX);
};

inline const MTC::vector<Index3DId>& Polyhedron::getCanonicalVertIds() const
{
	return _canonicalVertices;
}

inline const FastBisectionSet<Index3DId>& Polyhedron::getFaceIds() const
{
	return _faceIds;
}

inline bool Polyhedron::containsFace(const Index3DId& faceId) const
{
	return _faceIds.contains(faceId) != 0;
}

inline int32_t Polyhedron::getLayerNum() const
{
	return _layerNum;
}

std::ostream& operator << (std::ostream& out, const Polyhedron& cell);

}

