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
#include <Index3D.h>
#include <triMesh.h>
#include <polygon.h>
#include <polyhedron.h>
#include <subBlock.h>
#include <block.h>

namespace TriMesh {
	class CMesh;
	using CMeshPtr = std::shared_ptr<CMesh>;
}

namespace DFHM {

using CMesh = TriMesh::CMesh;


class Volume {
public:
	Volume(const Index3D& size = Index3D(0, 0, 0));
	Volume(const Volume& src);
	~Volume();

	void setOrigin(const Vector3d& origin);
	void setSpan(const Vector3d& span);
	void setBlockDims(const Index3D& size);
	const Index3D& getBlockDims() const;
	const Index3D& getDims() const;

	std::vector<TriMesh::CMeshPtr> addAllBlocks();

	// Get the block using a block index
	bool blockExists(size_t ix, size_t iy, size_t iz) const;
	bool blockExists(const Index3D& blockIdx) const;

	bool blockInBounds(const Index3D& blockIdx) const;

	Block& addBlock(size_t ix, size_t iy, size_t iz);
	Block& addBlock(const Index3D& blockIdx);

	std::shared_ptr<Block> getBlockPtr(const Index3D& blockIdx);

	const Block& getBlock(size_t ix, size_t iy, size_t iz) const;
	const Block& getBlock(const Index3D& blockIdx) const;
	Block& getBlock(size_t ix, size_t iy, size_t iz);
	Block& getBlock(const Index3D& blockIdx);

	// Get the subBlock using a subBlock index
	bool subBlockExists(size_t ix, size_t iy, size_t iz) const;
	bool subBlockExists(const Index3D& blockIdx) const;
	SubBlock& getSubBlock(size_t ix, size_t iy, size_t iz);
	SubBlock& getSubBlock(const Index3D& subBlockIdx);
	const SubBlock& getSubBlock(size_t ix, size_t iy, size_t iz) const;
	const SubBlock& getSubBlock(const Index3D& subBlockIdx) const;

	// Currently flow direction is along positive x axis.
	size_t calLinearBlockIndex(size_t ix, size_t iy, size_t iz) const;
	size_t calLinearBlockIndex(const Index3D& blockIdx) const;
	Index3D calBlockIndexFromLinearIndex(size_t linearIdx) const;

	std::vector<TriMesh::CMeshPtr> buildCFDHexes(const TriMesh::CMeshPtr& pTriMesh, double minSubBlockSize, bool outerFacesOnly);
	std::vector<TriMesh::CMeshPtr> makeTris(bool outerOnly, bool multiCore);
	size_t getGLModelEdgeLoops(std::vector<std::vector<float>>& edgeLoops) const;

	size_t numFaces(bool includeInner) const;
	size_t numPolyhedra() const;
	double getSharpAngleRad() const;

	void writePolyMesh(const std::string& dirName) const;

private:
	using AxisIndex = Block::AxisIndex;

	void processRayHit(const RayHit& triHit, int rayAxis, const Vector3d& blockSpan, const Vector3d& subBlockSpan, size_t& blockIdx, size_t& subBlockIdx);

	void writePolyMeshPoints(const std::string& dirName) const;
	void writeFOAMHeader(std::ofstream& out, const std::string& foamClass, const std::string& object) const;

	TriMesh::CMeshPtr _pModelTriMesh;
	Vector3d _originMeters, _spanMeters;
	Index3D _blockDim;
	double _sharpAngleRad;

	std::vector<std::shared_ptr<Block>> _blocks;

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

inline const Block& Volume::getBlock(const Index3D& blockIdx) const
{
	return getBlock(blockIdx[0], blockIdx[1], blockIdx[2]);
}

inline Block& Volume::getBlock(const Index3D& blockIdx)
{
	return getBlock(blockIdx[0], blockIdx[1], blockIdx[2]);
}

inline size_t Volume::calLinearBlockIndex(size_t ix, size_t iy, size_t iz) const
{
	if (ix < _blockDim[0] && iy < _blockDim[1] && iz < _blockDim[2])
		return ix + _blockDim[0] * (iy + _blockDim[1] * iz);

	return -1;
}

inline size_t Volume::calLinearBlockIndex(const Index3D& blockIdx) const
{
	return calLinearBlockIndex(blockIdx[0], blockIdx[1], blockIdx[2]);
}

inline Index3D Volume::calBlockIndexFromLinearIndex(size_t linearIdx) const
{
	Index3D result;
	size_t temp = linearIdx;

//		ix + _blockDim[0] * iy + _blockDim[0] * _blockDim[1] * iz;


	size_t denom = _blockDim[0] * _blockDim[1];
	result[2] = (Index3DBaseType) (temp / denom);
	temp = temp % denom;

	denom = _blockDim[0];

	result[1] = (Index3DBaseType)(temp / denom);
	temp = temp % denom;
	result[0] = (Index3DBaseType)temp;

	if (calLinearBlockIndex(result) != linearIdx) {
		throw std::runtime_error("calBlockIndexFromLinearIndex failed");
	}

	return result;
}

inline SubBlock& Volume::getSubBlock(const Index3D& subBlockIdx)
{
	return getSubBlock(subBlockIdx[0], subBlockIdx[1], subBlockIdx[2]);
}

inline const SubBlock& Volume::getSubBlock(const Index3D& subBlockIdx) const
{
	return getSubBlock(subBlockIdx[0], subBlockIdx[1], subBlockIdx[2]);
}

inline double Volume::getSharpAngleRad() const
{
	return _sharpAngleRad;
}

} // end namespace DFHM
