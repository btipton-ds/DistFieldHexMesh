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

#include <stdint.h>
#include <vector>
#include <map>
#include <mutex>
#include <MultiCoreUtil.h>
#include <index3D.h>
#include <triMesh.h>
#include <tm_spatialSearch.h>
#include <unalignedBBox.h>
#include <polygon.h>
#include <polyhedron.h>
#include <block.h>

namespace TriMesh {
	class CMesh;
	using CMeshPtr = std::shared_ptr<CMesh>;
}

namespace DFHM {

struct BuildCFDParams;

class AppData;
using AppDataPtr = std::shared_ptr<AppData>;

class Block;
using BlockPtr = std::shared_ptr<Block>;

class MeshData;
using MeshDataPtr = std::shared_ptr<MeshData>;

class Volume {
public:
	Volume(const Index3D& dims = Index3D(0, 0, 0));
	Volume(const Volume& src);
	~Volume();

	static void findSharpVertices(const TriMesh::CMeshPtr& pMesh, double sharpAngleRadians, std::vector<size_t>& vertIndices);

	void startOperation();
	void endOperation();

	void setVolDim(const Index3D& size, bool resetBoundaryDim);
	const Index3D& volDim() const;
	void setVolCornerPts(const std::vector<Vector3d>& pts);

	const Index3D& modelDim() const;

	const Block* getBlockPtr(const Index3D& blockIdx) const;
	Block* getBlockPtr(const Index3D& blockIdx);
	BlockPtr& getBoundingBlock(const Index3D& blkIdx, const Vector3d cPts[8]);
	Index3D determineOwnerBlockIdx(const Vector3d& point) const;

	static size_t calLinearBlockIndex(const Index3D& blockIdx, const Index3D& volDim);
	size_t calLinearBlockIndex(const Index3D& blockIdx) const;

	static Index3D calBlockIndexFromLinearIndex(size_t linearIdx, const Index3D& volDim);
	Index3D calBlockIndexFromLinearIndex(size_t linearIdx) const;

	void setAppData(const AppDataPtr& pAppData);
	const AppDataPtr& getAppData() const;
	CBoundingBox3Dd getModelBBox() const;
	CBoundingBox3Dd getVolumeBBox() const;

	void addAllBlocks(Block::GlHexMeshGroup& triMeshes, Block::glPointsGroup& faceEdges);

	void buildModelBlocks(const BuildCFDParams& params, const Vector3d pts[8], const CMesh::BoundingBox& volBox, bool multiCore);
	void buildCFDHexes(const CMeshPtr& pTriMesh, const BuildCFDParams& params, bool multiCore);

	void createHexFaceTris(Block::GlHexMeshGroup& triMeshes, const Index3D& min, const Index3D& max, bool multiCore) const;

	size_t numFaces(bool includeInner) const;
	size_t numPolyhedra() const;
	size_t numBytes() const;

	const std::vector<size_t>& getSharpVertIndices() const;
	bool getSharpVertPlane(Planed& plane) const;
	const std::set<size_t>& getSharpEdgeIndices() const;
	const std::vector<Vector3d>& getModelCornerPts() const;

	void setLayerNums();

	void insertBlocks(const BuildCFDParams& params, CubeFaceType face, bool multiCore);

	void makeFaceTriMesh(FaceDrawType faceType, Block::GlHexFacesPtr& pFace, const BlockPtr& pBlock) const;
	void getModelBoundaryPlanes(std::vector<Planed>& vals) const;

	void writeObj(const std::string& path, const std::vector<Index3DId>& cellIds, bool includeModel, bool useEdges, bool sharpOnly, const std::vector<Vector3d>& pts = std::vector<Vector3d>()) const;
	void writeObj(std::ostream& out, const std::vector<Index3DId>& cellIds, bool includeModel, bool useEdges, bool sharpOnly, const std::vector<Vector3d>& pts = std::vector<Vector3d>()) const;

	void writePolyMesh(const std::string& dirName);

	bool write(std::ostream& out) const;
	bool read(std::istream& inStream);

	bool verifyTopology(bool multiCore) const;
	bool verifyUniqueGeometry(bool multiCore) const;
	bool verifyUniquePoints(bool multiCore) const;
	bool verifyUniquePolygons(bool multiCore) const;

private:
	friend class Vertex;
	friend class Polygon;
	friend class Polyhedron;
	friend class Block;
	friend class ObjectPoolOwnerUser;
	friend class Index3DBase;

	using AxisIndex = Block::AxisIndex;
	using ThreadPool = MultiCore::ThreadPool;

	void clear();

	// Get the block using a block index
	bool blockExists(const Index3D& blockIdx) const;
	BlockPtr createBlock(const Index3D& blockIdx, bool forReading);
	BlockPtr createBlock(size_t linearIdx);

	const Vertex& getVertex(const Index3DId& id) const;
	const Polygon& getPolygon(const Index3DId& id) const;
	const Polyhedron& getPolyhedron(const Index3DId& id) const;

	void createBlocks(const BuildCFDParams& params, const Vector3d& blockSpan, bool multiCore);
	void buildSurroundingBlocks(const BuildCFDParams& params, const Vector3d cPts[8], bool multiCore);
	void gradeSurroundingBlocks(const BuildCFDParams& params, bool multiCore);
	void createAdHocBlockSearchTree();
	void divideSimple(const BuildCFDParams& params, bool multiCore);
	void divideConitional(const BuildCFDParams& params, bool multiCore);
	void cutWithTriMesh(const BuildCFDParams& params, bool multiCore);
	void doPreSplits(const BuildCFDParams& params, bool multiCore);
	bool splitRequiredPolyhedra(bool multiCore);
	void finishSplits(bool multiCore);
	void imprintTJointVertices(bool multiCore);
	void dumpOpenCells(bool multiCore) const;

	void findFeatures();
	void findSharpEdgeEdges();

	struct PolymeshTables {
		int32_t numInner;
		int32_t boundaryIdx;
		int32_t boundaryIndices[6];
		std::vector<Index3DId> vertIdxIdMap, faceIdxIdMap, cellIdxIdMap;
		std::map<Index3DId, int32_t> vertIdIdxMap, faceIdIdxMap, cellIdIdxMap;
	};

	void createPolymeshTables(PolymeshTables& tables);
	int getFaceOwnerIdx(const Index3DId& faceId, const PolymeshTables& tables) const;
	int getFaceNeighbourIdx(const Index3DId& faceId, const PolymeshTables& tables) const;
	bool needToReverseNormal(const Polygon& face, const PolymeshTables& tables) const;

	void writePolyMeshPoints(const std::string& dirName, const PolymeshTables& tables) const;
	void writePolyMeshFaces(const std::string& dirName, const PolymeshTables& tables) const;
	void writePolyMeshOwnerCells(const std::string& dirName, const PolymeshTables& tables) const;
	void writePolyMeshNeighborCells(const std::string& dirName, const PolymeshTables& tables) const;
	void writePolyMeshBoundaries(const std::string& dirName, const PolymeshTables& tables) const;
	void writeFOAMHeader(FILE* fOut, const std::string& fileType, const std::string& foamClass, const std::string& object) const;

	template<class L>
	void runThreadPool(const L& fLambda, bool multiCore) const;

	template<class L>
	void runThreadPool(const L& fLambda, bool multiCore);

	template<class L>
	void runThreadPool_IJK(const L& fLambda, bool multiCore);

	template<class L>
	void runThreadPool_IJ(const BuildCFDParams& params, const L& fLambda, bool multiCore);

	template<class L>
	void runThreadPool_JK(const BuildCFDParams& params, const L& fLambda, bool multiCore);

	template<class L>
	void runThreadPool_IK(const BuildCFDParams& params, const L& fLambda, bool multiCore);

	Index3D _volDim, _modelDim, _modelDimOrigin = Index3D(0, 0, 0);

	AppDataPtr _pAppData;
	CMesh::BoundingBox _modelBundingBox;

	UnalignedBBoxd _modelCornerPts, _volCornerPts;
	std::vector<BlockPtr> _blocks;
	CSpatialSearch<double, BlockPtr> _adHocBlockTree;   // This is for boundary blocks which don't follow the cartesian grid assignment rules.
														// Must use BlockPtr, NOT size_t, because the indices change during insertions.
														// The block is still stored in _blocks, but is not under cartesian access.

	std::set<size_t> _sharpEdgeIndices;
	std::vector<size_t> _sharpVertIndices;
	bool _hasSharpVertPlane = false;
	Planed _sharpVertPlane;

	MultiCore::ThreadPool _threadPool;
};

using VolumePtr = std::shared_ptr<Volume>;

inline void Volume::setAppData(const AppDataPtr& pAppData)
{
	_pAppData = pAppData;
}

inline const AppDataPtr& Volume::getAppData() const
{
	return _pAppData;
}

inline const Block* Volume::getBlockPtr(const Index3D& blockIdx) const
{
	auto idx = calLinearBlockIndex(blockIdx);
	return _blocks[idx].get();
}

inline Block* Volume::getBlockPtr(const Index3D& blockIdx)
{
	auto idx = calLinearBlockIndex(blockIdx);
	return _blocks[idx].get();
}

inline const std::vector<Vector3d>& Volume::getModelCornerPts() const
{
	return _modelCornerPts;
}

inline size_t Volume::calLinearBlockIndex(const Index3D& blockIdx, const Index3D& volDim)
{
	return blockIdx[0] + volDim[0] * (blockIdx[1] + volDim[1] * blockIdx[2]);
}

inline size_t Volume::calLinearBlockIndex(const Index3D& blockIdx) const
{
	return calLinearBlockIndex(blockIdx, _volDim);
}

inline const std::vector<size_t>& Volume::getSharpVertIndices() const
{
	return _sharpVertIndices;
}

inline bool Volume::getSharpVertPlane(Planed& plane) const
{
	if (_hasSharpVertPlane) {
		plane = _sharpVertPlane;
		return true;
	}
	return false;
}

inline const std::set<size_t>& Volume::getSharpEdgeIndices() const
{
	return _sharpEdgeIndices;

}

} // end namespace DFHM
