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

#include <iostream>
#include <memory>
#include <triMesh.h>
#include <tm_boundingBox.h>
#include <tm_lineSegment.h>
#include <tm_lineSegment_byref.h>
#include <index3D.h>
#include <pool_vector.h>
#include <pool_set.h>
#include <objectPool.h>
#include <lambdaMacros.h>
#include <fastBisectionSet.h>
#include <polyMeshIndex.h>
#include <meshData.h>
#include <model.h>

template<class T>
class Plane;

namespace DFHM {

struct SplittingParams;
class Block;
class Edge;
class AppData;
using PolyMeshSearchTree = CSpatialSearchBase<double, PolyMeshIndex, 25>;

class Polyhedron : public ObjectPoolOwnerUser, public PolyMeshSearchTree::Refiner {
public:
	Polyhedron() = default;
	Polyhedron(const MultiCore::set<Index3DId>& faceIds, const MultiCore::vector<Index3DId>& cornerVertIds = MultiCore::vector<Index3DId>());
	explicit Polyhedron(const std::set<Index3DId>& faceIds, const std::vector<Index3DId>& cornerVertIds = std::vector<Index3DId>());
	Polyhedron(const MultiCore::vector<Index3DId>& faceIds, const MultiCore::vector<Index3DId>& cornerVertIds = MultiCore::vector<Index3DId>());
	explicit Polyhedron(const std::vector<Index3DId>& faceIds, const std::vector<Index3DId>& cornerVertIds = std::vector<Index3DId>());
	Polyhedron(const Polyhedron& src);

	void initializeSearchTree() const;

	void clear() override;
	const PolyMeshSearchTree::Refiner* getRefiner() const;
	bool entryIntersects(const PolyMeshSearchTree::BOX_TYPE& bbox, const PolyMeshSearchTree::Entry& entry) const override;

	Polyhedron& operator = (const Polyhedron& rhs);
	void copyCaches(const Polyhedron& src);

	const Index3DId& getId() const override;
	void remapId(const std::vector<size_t>& idRemap, const Index3D& srcDims) override;

	void addFace(const Index3DId& faceId);
	void removeFace(const Index3DId& faceId);
	const FastBisectionSet<Index3DId>& getFaceIds() const;
	size_t getNumFaces() const;
	size_t getVertIds(MTC::set<Index3DId>& result) const;
	const MTC::vector<Index3DId>& getCanonicalVertIds() const;
	const MTC::vector<Vector3d>& getCanonicalPoints() const;
	MTC::set<EdgeKey> getCanonicalHexEdgeKeys(int ignoreAxis = -1) const;
	MTC::set<EdgeKey> getEdgeKeys(bool includeAdjacentCellFaces) const;

	FastBisectionSet<Index3DId> getAdjacentCells() const;

	// Gets the edges for a vertex which belong to this polyhedron
	void getVertEdges(const Index3DId& vertId, FastBisectionSet<EdgeKey>& edges, bool includeAdjacentCells) const;
	// Gets the faces for a vertex which belong to this polyhedron
	FastBisectionSet<Index3DId> getVertFaces(const Index3DId& vertId) const;

	const CBoundingBox3Dd& getBoundingBox() const;
	void clearCache() const;

	bool containsVertex(const Index3DId& vertId) const;
	bool containsFace(const Index3DId& faceId) const;
	bool isVertexConnectedToCell(const Index3DId& cellId) const;
	const Vector3d& calCentroid() const;
	Vector3d calCentroidApprox() const;
	double calVolume() const;
	double calCurvature2D(const SplittingParams& params, const MTC::vector<Vector3d>& quadPoints, size_t step) const;
	void calOrientatedPlane(const Index3DId& faceId, Planed& facePlane) const;
	void calOrientatedPlane(const Index3DId& faceId, const Vector3d& cellCtr, Planed& facePlane) const;
	bool isClosed() const;
	bool isOriented() const;
	bool exists() const;
	size_t classify(MTC::vector<Vector3d>& corners) const;
	void classifyEdges(MTC::set<EdgeKey>& convexEdges, MTC::set<EdgeKey>& concaveEdges) const;
	bool isConvex() const;
	bool pointInside(const Vector3d& pt) const;
	bool segInside(const LineSegment_byrefd& seg) const;
	bool insideModel() const;

	bool entryIntersectsModel(const PolyMeshIndex& index) const;
	size_t getPolyIndices(std::vector<PolyMeshIndex>& indices) const;

	bool entryIntersectsModel(const TriMeshIndex& index) const;
	size_t getTriIndices(std::vector<TriMeshIndex>& indices) const;

	bool intersectsModel() const;

	void setIntersectsModel(bool val);
	bool sharpEdgesIntersectModel(const SplittingParams& params) const;

	void createPlanarFaceSet(MTC::vector<MTC::set<Index3DId>>& planarFaceSet) const;
	bool isTooComplex(const SplittingParams& params) const;
	bool hasTooHighCurvature(const SplittingParams& params) const;
	bool hasTooManFaces(const SplittingParams& params) const;
	bool needsCurvatureSplit(const SplittingParams& params, int splittingPlaneNormalAxis) const;
	Vector3d calSpan() const;
	double calCurvatureXYPlane(const SplittingParams& params) const;
	double calCurvatureYZPlane(const SplittingParams& params) const;
	double calCurvatureZXPlane(const SplittingParams& params) const;

	double maxOrthogonalityAngleRadians() const;

	double getComplexityScore(const SplittingParams& params) const;

	CellType getCellType() const;

	void setNeedsDivideAtCentroid();
	bool needsDivideAtCentroid() const;

	// If this cell has any partial splits/split faces, it will mark it for centroid splitting.
	bool setNeedsCleanFaces();

	bool containsSharps() const;

	// Splitting functions are const to prevent reusing the split cell. After splitting, the cell should be removed from the block
	void addToSplitStack();
	bool setNeedToSplitConditional(size_t passNum, const SplittingParams& params);

	void attachFaces();
	void detachFaces();

	void connectVertEdgeTopology();
	void disconnectVertEdgeTopology();

	int32_t getLayerNum() const;
	void clearLayerNum();
	void setLayerNum(int32_t val, bool force);
	void setLayerNumOnNextPass(int32_t val);

	MTC::vector<size_t> getSharpVertIndices() const;
	bool getSharpEdgeIndices(MTC::vector<size_t>& result, const SplittingParams& params) const;

	MTC::vector<Index3DId> getParents() const;

	void makeHexCellPoints(int axis, MTC::vector<MTC::vector<Vector3d>>& subCells, MTC::vector<Vector3d>& partingFacePts) const;
	void makeHexFacePoints(int axis, double w, MTC::vector<Vector3d>& facePts) const;

	void write(std::ostream& out) const;
	void read(std::istream& in);

	bool unload(std::ostream& out);
	bool load(std::istream& out);
	void dumpFaces() const;

	bool lineSegmentIntersects(const LineSegmentd& seg, MTC::vector<RayHitd>& hits, MTC::vector<Index3DId>& faceIds) const;
	bool verifyTopology() const;
	bool operator < (const Polyhedron& rhs) const;

	inline void setParentId(const Index3DId& id)
	{
		_parentId = id;
	}

	LAMBDA_CLIENT_DECLS

protected:
	void setId(const Index3DId& id) override;

private:
	friend class Block;
	friend std::ostream& operator << (std::ostream& out, const Polyhedron& face);
	friend class Splitter3D;
	friend class AppData;

	const std::shared_ptr<const PolyMeshSearchTree> getPolySearchTree() const;
	const Model& getModel() const;

	MTC::set<EdgeKey> createEdgesFromVerts(MTC::vector<Index3DId>& vertIds) const;
	bool orderVertIds(MTC::vector<Index3DId>& vertIds) const;
	void copyToOut() const;
	double minGap() const;
	bool polygonExists(const Index3DId& id) const;
	const Vector3d& getVertexPoint(const Index3DId& vertId) const;

	double getCurvatureByNormalAxis(const SplittingParams& params, int axis) const;
	void initCurvatureByNormalAxis(const SplittingParams& params, int orthoAxis0) const;

	void createTriPoints(std::vector<std::pair<const Vector3d*, const Polygon*>>& cellTriPts) const;
	void createTriPoints(std::vector<std::pair<const Vector3d, const Polygon*>>& cellTriPts) const;

	Index3DId _thisId;
	/*
		The faceIds are invariant.
		If a face is split, it's id stays in the list, but the face itself records it's been split and into which faces
		The original face is marked as split, no longer really exists, and points to it's replacements.
		The same applies for faces with split edges, but the number of faces is only 1.
	*/
	Index3DId _parentId;
	FastBisectionSet<Index3DId> _faceIds;

	// _canonicalVertices are the vertices that define the polyhedron 4 = tet, 5 = pyramid, 6 = triangular prism, 8 = hexahedron, 12 = hexagonal cylinder
	// If it's empty, the cell is not canonical
	MTC::vector<Index3DId> _canonicalVertices; 

	int32_t _layerNum = -1; // -1 is not set yet, -2 is mark for setting on set pass

	bool _needsSplitAtCentroid = false;
	bool _exists = true;

	mutable bool _needsCurvatureCheck = true;

	mutable Trinary _cachedIsClosed = IS_UNKNOWN;
	mutable Trinary _cachedIntersectsModel = IS_UNKNOWN; // Cached value
	mutable Trinary _sharpEdgesIntersectModel = IS_UNKNOWN; // Cached value

	mutable double _cachedMinGap = -1;
	mutable double _cachedComplexityScore = -1;
	mutable double _maxOrthogonalityAngleRadians = -1;

	// The axes are cached separately because they are accessed separately when determining best split axis
	mutable double _cachedCurvatureXYPlane = -1;
	mutable double _cachedCurvatureYZPlane = -1;
	mutable double _cachedCurvatureZXPlane = -1;

	mutable MTC::vector<Vector3d> _cachedCanonicalPoints;
	mutable Vector3d _cachedCtr = Vector3d(DBL_MAX, DBL_MAX, DBL_MAX);
	mutable CBoundingBox3Dd _cachedBBox;
	mutable bool _hasSetSearchTree = false;

	mutable std::shared_ptr<const PolyMeshSearchTree> _pPolySearchTree;
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

inline double Polyhedron::calCurvatureXYPlane(const SplittingParams& params) const
{
	return getCurvatureByNormalAxis(params, 2);
}

inline double Polyhedron::calCurvatureYZPlane(const SplittingParams& params) const
{
	return getCurvatureByNormalAxis(params, 0);
}

inline double Polyhedron::calCurvatureZXPlane(const SplittingParams& params) const
{
	return getCurvatureByNormalAxis(params, 1);
}

std::ostream& operator << (std::ostream& out, const Polyhedron& cell);

}

