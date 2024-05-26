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

#include <stdexcept>
#include <vertex.h>
#include <chrono>
#include <mutex>
#include <atomic>

#include <tm_vector3.h>
#include <triMesh.h>
#include <tm_spatialSearch.h>
#include <enums.h>
#include <index3D.h>
#include <objectPool.h>
#include <pool_allocator.h>
#include <logger.h>
#include <lambdaMacros.h>
#include <vertex.h>
#include <polygon.h>
#include <polyhedron.h>
#include <vertexSpatialTree.h>

namespace DFHM {

using CMesh = TriMesh::CMesh;
using CMeshPtr = TriMesh::CMeshPtr;

struct BuildCFDParams;
class Volume;
class Edge;
class Polygon;
class Polyhedron;

class Block : public ObjectPoolOwner {
public:
	class GlPoints : public std::vector<float>
	{
	public:
		GlPoints();
		GlPoints(const GlPoints& src);
		size_t getId() const;
		size_t changeNumber() const;
		void changed();
	private:
		static std::atomic<size_t> _statId;
		size_t _id, _changeNumber = 0;
	};

	using glPointsPtr = std::shared_ptr<GlPoints>;
	using glPointsVector = std::vector<glPointsPtr>;
	using glPointsGroup = std::vector<glPointsVector>;

	using TriMeshVector = std::vector<CMeshPtr>;
	using TriMeshGroup = std::vector<TriMeshVector>;

	Volume* getVolume() override;
	const Volume* getVolume() const override;
	const Index3D& getBlockIdx() const override;
	const Block* getOwner(const Index3D& blockIdx) const override;
	Block* getOwner(const Index3D& blockIdx) override;

	Block(Volume* pVol, const Index3D& blockIdx, const std::vector<Vector3d>& pts);
	Block(const Block& src);

	PoolUtils::localHeap* getHeapPtr() const;

	size_t blockDim() const;

	// These method determine with block owns an entity based on it's location
	Index3D determineOwnerBlockIdx(const Vector3d& point) const;
	Index3D determineOwnerBlockIdx(const Vertex& vert) const;
	Index3D determineOwnerBlockIdx(const std::vector<Vector3d>& points) const;
	Index3D determineOwnerBlockIdx(const std::vector<Index3DId>& verts) const;
	Index3D determineOwnerBlockIdx(const Polygon& face) const;

	size_t pointOpenFoamLabel(const Index3DId& id) const;
	size_t faceOpenFoamLabel(const Index3DId& id) const;
	size_t cellOpenFoamLabel(const Index3DId& id) const;
	Index3DId maxCellId(const Index3DId& id0, const Index3DId& id1) const;

	size_t numFaces(bool includeInner) const;
	size_t numPolyhedra() const;

	bool verifyTopology() const;
	void createSubBlocks(TopolgyState refState);

	size_t calLinearSubBlockIndex(const Index3D& subBlockIdx) const;
	Index3D calSubBlockIndexFromLinear(size_t linearIdx) const;
	void addSubBlockFaces();
	void createBlockFaces();

	const CMeshPtr& getModelMesh() const;
	void getBlockTriMesh(FaceType meshType, CMeshPtr& pMesh);
	void makeEdgeSets(FaceType meshType, glPointsPtr& points);

	Index3DId idOfPoint(const Vector3d& pt) const;
	Index3DId addVertex(const Vector3d& pt, const Index3DId& currentId = Index3DId());
	Vector3d getVertexPoint(const Index3DId& vertId) const;
	void setVertexLockType(const Index3DId& vertId, VertexLockType val);
	VertexLockType getVertexLockType(const Index3DId& vertId) const;

	Index3DId addFace(const std::vector<Index3DId>& vertIndices);
	Index3DId addFace(const Polygon& face);
	Index3DId addFace(const std::vector<Vector3d>& pts);
	void addFaceToLookup(const Index3DId& faceId);
	bool removeFaceFromLookUp(const Index3DId& faceId);

	Index3DId addCell(const Polyhedron& cell);
	Index3DId addCell(const std::set<Index3DId>& faceIds);
	Index3DId addCell(const std::vector<Index3DId>& faceIds);
	Index3DId addHexCell(const Vector3d* blockPts, size_t divs, const Index3D& subBlockIdx, bool intersectingOnly);

	bool vertexExists(const Index3DId& id) const;
	bool polygonExists(TopolgyState refState, const Index3DId& id) const;
	bool isPolygonReference(const Polygon* face) const;

	bool polyhedronExists(TopolgyState refState, const Index3DId& id) const;
	bool isPolyhedronReference(const Polyhedron* cell) const;
	bool allCellsClosed() const;

	void makeRefPolygonIfRequired(const Index3DId& id);
	void makeRefPolyhedronIfRequired(const Index3DId& id);

	Polygon& getPolygon(TopolgyState refState, const Index3DId& id);
	Polyhedron& getPolyhedron(TopolgyState refState, const Index3DId& id);

	void addToSplitStack(const std::set<Index3DId>& cellIds);
	void updateSplitStack();
	bool hasPendingSplits() const;

	void freePolygon(const Index3DId& id);
	void freePolyhedron(const Index3DId& id);

	size_t processEdges(const TriMesh::CMesh::BoundingBox& bbox, std::vector<size_t>& edgeIndices) const;
	size_t processTris(const TriMesh::CMesh::BoundingBox& bbox, std::vector<size_t>& triIndices) const;

	// pack removes the subBlock array if there's nothing interesting in it. It's a full search of the array and can be time consuming.
	void pack();
	bool write(std::ostream& outStream) const;
	bool read(std::istream& inStream);

	bool isUnloaded() const;
	bool unload(std::string& filename);
	bool load();

	void dumpPolyhedraObj(const std::vector<Index3DId>& cellIds, bool includeModel, bool useEdges, bool sharpOnly, const std::vector<Vector3d>& pts = std::vector<Vector3d>()) const;
	void dumpPolygonObj(std::string& fileName, const std::vector<Index3DId>& faceIds) const;
	void dumpEdgeObj(std::string& fileName, const std::vector<Edge>& edges) const;

	template<class F>
	void iteratePolygonsInOrder(TopolgyState state, F fLambda) const;

	template<class F>
	void iteratePolygonsInOrder(TopolgyState state, F fLambda);

	template<class F>
	void iteratePolyhedraInOrder(TopolgyState state, F fLambda) const;

	template<class F>
	void iteratePolyhedraInOrder(TopolgyState state, F fLambda);

	LAMBDA_BLOCK_DECLS

private:
	friend class Volume;
	friend class TestBlock;
	friend class MultiLockGuard;

	enum class AxisIndex {
		X, Y, Z
	};

	struct ModelData {
		ModelData(Block* pBlk);
		ModelData(Block* pBlk, const ModelData& src);

		ObjectPool<Polygon> _polygons;
		ObjectPool<Polyhedron> _polyhedra;
	};

	using SearchTree = VertSearchTreeIndex3DId_4;
	using SearchTreePtr = std::shared_ptr<SearchTree>;
	using SearchTreeConstPtr = std::shared_ptr<const SearchTree>;

	static void addIndexToMap(const Index3D& subBlockIdx, std::set<Index3D>& subBlockIndices);

	void setIsOutput(bool val);
	void getAdjacentBlockIndices(std::set<Index3D>& indices) const;
	Index3D determineOwnerBlockIdxFromRatios(const Vector3d& ratios) const;

	const std::vector<Vector3d>& getCornerPts() const; // Change to returning fractions so we can assign boundary values.
	std::vector<Vector3d> getSubBlockCornerPts(const Vector3d* blockPts, size_t divs, const Index3D& subBlockIdx) const;

	Vector3d triLinInterp(const Vector3d* blockPts, size_t divs, const Index3D& pt) const;
	void createSubBlocksForHexSubBlock(const Vector3d* blockPts, const Index3D& subBlockIdx);

	Index3DId addFace(int axis, const Index3D& subBlockIdx, const std::vector<Vector3d>& pts);

	void calBlockOriginSpan(Vector3d& origin, Vector3d& span) const;
	bool includeFaceInRender(FaceType meshType, const Polygon& face) const;

	bool doPresplits(const BuildCFDParams& params);
	void dumpOpenCells() const;

	bool splitRequiredPolyhedra();

	void imprintTJointVertices();
	
	const ModelData& data(TopolgyState refState) const;
	ModelData& data(TopolgyState refState);

	std::string getObjFilePath() const;

#ifdef _DEBUG
	bool isPolygonInUse(const Index3DId& faceId) const;
	bool isPolyhedronInUse(const Index3DId& cellId) const;
#endif // _DEBUG

	bool _hadConditionalSplits = true;
	Index3D _blockIdx;

	Volume* _pVol;
	CBoundingBox3Dd 
		_boundBox,      // The precise bounding box for this box
		_innerBoundBox; // An inner bounding box with a span of (_blockDim - 0.125) / _blockDim. Any vertex or face which is not completely within the inner box
						// must be tested to see if it belongs to this box or a Adjacent box.
						// This required for mutex management for objects which may be modified by more than one box/thread. Items belonging to this box do not require 
						// locking the mutex.Objects which lie on the boundary do require locking.
	
	size_t _blockDim;   // This is the dimension of the block = the number of cells across the block

	std::vector<Vector3d> _corners;

	std::string _filename;

	size_t _baseIdxVerts = 0, _baseIdxPolygons = 0, _baseIdxPolyhedra = 0;
	std::set<Index3DId> _needToSplit, _cantSplitYet;

	std::vector<size_t> _edgeIndices, _triIndices;
	SearchTreePtr _vertTree;
	ObjectPool<Vertex> _vertices;
	ModelData _modelData, _refData;

	mutable PoolUtils::localHeap _localHeap;
};

inline size_t Block::GlPoints::getId() const
{
	return _id;
}

inline size_t Block::GlPoints::changeNumber() const
{
	return _changeNumber;
}

inline void Block::GlPoints::changed()
{
	_changeNumber++;
}

inline size_t Block::blockDim() const
{
	return _blockDim;
}

inline PoolUtils::localHeap* Block::getHeapPtr() const
{
	return &_localHeap;
}

inline Volume* Block::getVolume()
{
	return _pVol;
}

inline const Volume* Block::getVolume() const
{
	return _pVol;
}

inline size_t Block::calLinearSubBlockIndex(const Index3D& subBlockIdx) const
{
	if (subBlockIdx.isInBounds(_blockDim))
		return subBlockIdx[0] + _blockDim * (subBlockIdx[1] + _blockDim * subBlockIdx[2]);
	return -1;
}

inline bool Block::isUnloaded() const
{
	return !_filename.empty();
}

inline const Block::ModelData& Block::data(TopolgyState refState) const
{
	return refState == TS_REAL ? _modelData : _refData;
}

inline Block::ModelData& Block::data(TopolgyState refState)
{
	return refState == TS_REAL ? _modelData : _refData;
}

template<class F>
inline void Block::iteratePolygonsInOrder(TopolgyState state, F fLambda) const
{
	data(state)._polygons.iterateInOrder(fLambda);
}

template<class F>
inline void Block::iteratePolygonsInOrder(TopolgyState state, F fLambda)
{
	data(state)._polygons.iterateInOrder(fLambda);
}

template<class F>
inline void Block::iteratePolyhedraInOrder(TopolgyState state, F fLambda) const
{
	data(state)._polyhedra.iterateInOrder(fLambda);
}

template<class F>
inline void Block::iteratePolyhedraInOrder(TopolgyState state, F fLambda)
{
	data(state)._polyhedra.iterateInOrder(fLambda);
}


}
