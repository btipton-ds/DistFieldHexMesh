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

using CMesh = TriMesh::CMesh;

class Volume {
public:
	using glPoints = std::vector<float>;
	using glPointsPtr = std::shared_ptr<glPoints>;
	using glPointsGroup = std::vector<std::vector<glPointsPtr>>;
	using TriMeshGroup = std::vector<std::vector<TriMesh::CMeshPtr>>;

	Volume();
	Volume(const Volume& src);
	~Volume();

	static void setVolDim(const Index3D& size);
	static const Index3D& volDim();

	void setOrigin(const Vector3d& origin);
	void setSpan(const Vector3d& span);

	void addAllBlocks(std::vector<std::vector<TriMesh::CMeshPtr>>& triMeshes, std::vector<std::vector<glPointsPtr>>& faceEdges);

	// Get the block using a block index
	bool blockExists(const Index3D& blockIdx) const;
	bool blockInBounds(const Index3D& blockIdx) const;
	Block& addBlock(const Index3D& blockIdx);

	std::shared_ptr<Block> getBlockPtr(const Index3D& blockIdx);

	const Block& getBlock(const Index3D& blockIdx) const;
	Block& getBlock(const Index3D& blockIdx);

	// Currently flow direction is along positive x axis.
	size_t calLinearBlockIndex(const Index3D& blockIdx) const;
	Index3D calBlockIndexFromLinearIndex(size_t linearIdx) const;

	void buildCFDHexes(const TriMesh::CMeshPtr& pTriMesh, double minSubBlockSize);
	void makeTris(TriMeshGroup& triMeshes, size_t minSplitNum, bool multiCore) const;
	void makeFaceEdges(glPointsGroup& faceEdges, size_t minSplitNum, bool multiCore) const;

	size_t numFaces(bool includeInner) const;
	size_t numPolyhedra() const;
	double getSharpAngleRad() const;

	void writePolyMesh(const std::string& dirName) const;

	const std::set<size_t>& getSharpVertIndices() const;
	const std::set<size_t>& getSharpEdgeIndices() const;

	bool verifyTopology() const;

private:
	using AxisIndex = Block::AxisIndex;

	void findFeatures();
	void findSharpVertices();
	void findSharpEdgeGroups();

	void processRayHit(const RayHit& triHit, int rayAxis, const Vector3d& blockSpan, const Vector3d& subBlockSpan, size_t& blockIdx, size_t& subBlockIdx);

	void writePolyMeshPoints(const std::string& dirName) const;
	void writeFOAMHeader(std::ofstream& out, const std::string& foamClass, const std::string& object) const;

	static Index3D s_volDim;

	TriMesh::CMeshPtr _pModelTriMesh;
	Vector3d _originMeters, _spanMeters;
	double _sharpAngleRad;

	std::vector<Vector3d> _cornerPts;
	std::vector<std::shared_ptr<Block>> _blocks;
	std::set<size_t> _sharpVertIndices, _sharpEdgeIndices;

};

using VolumePtr = std::shared_ptr<Volume>;


inline void Volume::setOrigin(const Vector3d& origin)
{
	_originMeters = origin;
}

inline void Volume::setSpan(const Vector3d& span)
{
	_spanMeters = span;
}

inline std::shared_ptr<Block> Volume::getBlockPtr(const Index3D& blockIdx)
{
	return _blocks[calLinearBlockIndex(blockIdx)];
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
