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
#include <index3D.h>
#include <triMesh.h>
#include <polygon.h>
#include <polyhedron.h>
#include <block.h>

namespace TriMesh {
	class CMesh;
	using CMeshPtr = std::shared_ptr<CMesh>;
}

namespace DFHM {

struct BuildCFDParams;

class Volume {
public:
	Volume();
	Volume(const Volume& src);
	~Volume();

	static void setVolDim(const Index3D& size);
	static const Index3D& volDim();
	static void findSharpVertices(const TriMesh::CMeshPtr& pMesh, double sharpAngleRadians, std::vector<size_t>& vertIndices);

	void startOperation();
	void endOperation();

	const Block* getBlockPtr(const Index3D& blockIdx) const;
	Block* getBlockPtr(const Index3D& blockIdx);

	void setModelMesh(const CMeshPtr& pMesh);
	const CMeshPtr& getModelMesh() const;

	void setOrigin(const Vector3d& origin);
	void setSpan(const Vector3d& span);

	void addAllBlocks(Block::TriMeshGroup& triMeshes, Block::glPointsGroup& faceEdges);

	void buildCFDHexes(const CMeshPtr& pTriMesh, const BuildCFDParams& params, bool multiCore);
	void makeFaceTris(Block::TriMeshGroup& triMeshes, const Index3D& min, const Index3D& max, bool multiCore) const;
	void makeEdgeSets(Block::glPointsGroup& faceEdges, const Index3D& min, const Index3D& max, bool multiCore) const;

	size_t numFaces(bool includeInner) const;
	size_t numPolyhedra() const;
	double getSharpAngleRad() const;

	const std::vector<size_t>& getSharpVertIndices() const;
	bool getSharpVertPlane(Plane<double>& plane) const;
	const std::set<size_t>& getSharpEdgeIndices() const;

	void makeFaceTriMesh(FaceType faceType, Block::TriMeshGroup& triMeshes, const std::shared_ptr<Block>& pBlock, size_t threadNum) const;
	void makeFaceEdges(FaceType faceType, Block::glPointsGroup& faceEdges, const std::shared_ptr<Block>& pBlock, size_t threadNum) const;

	void writeObj(const std::string& path, const std::vector<Index3DId>& cellIds) const;
	void writeObj(std::ostream& out, const std::vector<Index3DId>& cellIds) const;

	void writePolyMesh(const std::string& dirName);

	bool write(std::ostream& out) const;
	bool read(std::istream& inStream);

	bool verifyTopology(bool multiCore) const;

private:
	friend class Vertex;
	friend class Edge;
	friend class Polygon;
	friend class Polyhedron;
	friend class Block;
	friend class ObjectPoolOwnerUser;
	friend class Index3DBase;

	using AxisIndex = Block::AxisIndex;

	void clear();

	// Get the block using a block index
	bool blockExists(const Index3D& blockIdx) const;
	std::shared_ptr<Block> createBlock(const Index3D& blockIdx);

	// Currently flow direction is along positive x axis.
	size_t calLinearBlockIndex(const Index3D& blockIdx) const;
	Index3D calBlockIndexFromLinearIndex(size_t linearIdx) const;

	const Vertex& getVertex(const Index3DId& id) const;
	const Polygon& getPolygon(const Index3DId& id) const;
	const Polyhedron& getPolyhedron(const Index3DId& id) const;

	void createBlocks(const BuildCFDParams& params, const Vector3d& blockSpan, bool multiCore);
	void splitSimple(const BuildCFDParams& params, bool multiCore);
	void splitAtCurvature(const BuildCFDParams& params, bool multiCore);
	void splitDueToSplitFaces(const BuildCFDParams& params, bool multiCore);
	void splitAtSharpVertices(const BuildCFDParams& params, bool multiCore);
	void splitAtSharpEdges(const BuildCFDParams& params, bool multiCore);
	void finishSplits(bool multiCore);
	void imprintTJointVertices(bool multiCore);
	void dumpOpenCells(bool multiCore) const;

	void findFeatures();
	void findSharpEdgeGroups();

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
	void runLambda(L fLambda, bool multiCore) const;
	template<class L>
	void runLambda(L fLambda, bool multiCore);
	template<class L>
	void runLambda(L fLambda, size_t numBlocks, bool multiCore);
	template<class L>
	void runLambda(L fLambda, size_t numBlocks, bool multiCore) const;

	static Index3D s_volDim;

	CMeshPtr _pModelTriMesh;
	Vector3d _originMeters, _spanMeters;
	double _sharpAngleRad;
	CMesh::BoundingBox _boundingBox;

	std::vector<Vector3d> _cornerPts;
	std::vector<std::shared_ptr<Block>> _blocks;
	std::set<size_t> _sharpEdgeIndices;
	std::vector<size_t> _sharpVertIndices;
	bool _hasSharpVertPlane = false;
	Plane<double> _sharpVertPlane;
};

using VolumePtr = std::shared_ptr<Volume>;

inline void Volume::setModelMesh(const CMeshPtr& pMesh)
{
	_pModelTriMesh = pMesh;
}

inline const CMeshPtr& Volume::getModelMesh() const
{
	return _pModelTriMesh;
}

inline void Volume::setOrigin(const Vector3d& origin)
{
	_originMeters = origin;
}

inline void Volume::setSpan(const Vector3d& span)
{
	_spanMeters = span;
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

inline size_t Volume::calLinearBlockIndex(const Index3D& blockIdx) const
{
	return blockIdx[0] + s_volDim[0] * (blockIdx[1] + s_volDim[1] * blockIdx[2]);
}

inline double Volume::getSharpAngleRad() const
{
	return _sharpAngleRad;
}

inline const std::vector<size_t>& Volume::getSharpVertIndices() const
{
	return _sharpVertIndices;
}

inline bool Volume::getSharpVertPlane(Plane<double>& plane) const
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
