#pragma once

/*
* DistFieldHexMesh
* 
* volume.h
* 
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

class Volume {
public:
	Volume();
	Volume(const Volume& src);
	~Volume();

	static void setVolDim(const Index3D& size);
	static const Index3D& volDim();

	void startOperation();
	void endOperation();

	const Block* getBlockPtr(const Index3D& blockIdx) const;
	Block* getBlockPtr(const Index3D& blockIdx);

	const CMeshPtr& getModelMesh() const;

	void setOrigin(const Vector3d& origin);
	void setSpan(const Vector3d& span);

	void addAllBlocks(Block::TriMeshGroup& triMeshes, Block::glPointsGroup& faceEdges);

	struct BuildCFDParams {
		size_t minBlocksPerSide = 6;
		size_t numSimpleDivs = 2;
		size_t numCurvatureDivs = 4;
		double sharpAngleDegrees = 30.0;
		double curvatureArcAngleDegrees = 15.0;
	};
	void buildCFDHexes(const CMeshPtr& pTriMesh, const BuildCFDParams& params);
	void makeFaceTris(Block::TriMeshGroup& triMeshes, bool multiCore) const;
	void makeEdgeSets(Block::glPointsGroup& faceEdges, bool multiCore) const;

	size_t numFaces(bool includeInner) const;
	size_t numPolyhedra() const;
	double getSharpAngleRad() const;

	const std::set<size_t>& getSharpVertIndices() const;
	const std::set<size_t>& getSharpEdgeIndices() const;

	void writePolyMesh(const std::string& dirName) const;

	bool verifyTopology() const;

private:
	friend class Vertex;
	friend class Edge;
	friend class Polygon;
	friend class Polyhedron;
	friend class Block;
	friend class ObjectPoolOwnerUser;

	using AxisIndex = Block::AxisIndex;

	// Get the block using a block index
	bool blockExists(const Index3D& blockIdx) const;
	std::shared_ptr<Block> createBlock(const Index3D& blockIdx);

	// Currently flow direction is along positive x axis.
	size_t calLinearBlockIndex(const Index3D& blockIdx) const;
	Index3D calBlockIndexFromLinearIndex(size_t linearIdx) const;

	void findFeatures();
	void findSharpVertices();
	void findSharpEdgeGroups();

	void writePolyMeshPoints(const std::string& dirName) const;
	void writeFOAMHeader(std::ofstream& out, const std::string& foamClass, const std::string& object) const;

	template<class L>
	void runLambda(L fLambda, bool multiCore) const;
	template<class L>
	void runLambda(L fLambda, bool multiCore);

	static Index3D s_volDim;

	CMeshPtr _pModelTriMesh;
	Vector3d _originMeters, _spanMeters;
	double _sharpAngleRad;

	std::vector<Vector3d> _cornerPts;
	std::vector<std::shared_ptr<Block>> _blocks;
//	std::vector<std::shared_ptr<Block>> _outBlocks;
	std::set<size_t> _sharpVertIndices, _sharpEdgeIndices;

};

using VolumePtr = std::shared_ptr<Volume>;

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
	if (idx < _blocks.size() && _blocks[idx])
		return _blocks[idx].get();

	return nullptr;
}

inline Block* Volume::getBlockPtr(const Index3D& blockIdx)
{
	auto idx = calLinearBlockIndex(blockIdx);
	if (idx < _blocks.size() && _blocks[idx])
		return _blocks[idx].get();

	return nullptr;
}

inline size_t Volume::calLinearBlockIndex(const Index3D& blockIdx) const
{
	const auto& dim = volDim();
	if (blockIdx.isInBounds(dim))
		return blockIdx[0] + dim[0] * (blockIdx[1] + dim[1] * blockIdx[2]);

	return -1;
}

inline double Volume::getSharpAngleRad() const
{
	return _sharpAngleRad;
}

inline const std::set<size_t>& Volume::getSharpVertIndices() const
{
	return _sharpVertIndices;
}

inline const std::set<size_t>& Volume::getSharpEdgeIndices() const
{
	return _sharpEdgeIndices;

}

} // end namespace DFHM
