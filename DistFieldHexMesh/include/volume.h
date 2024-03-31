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
		double maxCurvatureRadius = 0.1; // 10 cm
		int divsPerRadius = 2;
	};
	void buildCFDHexes(const CMeshPtr& pTriMesh, const BuildCFDParams& params, bool multiCore);
	void makeFaceTris(Block::TriMeshGroup& triMeshes, bool multiCore) const;
	void makeEdgeSets(Block::glPointsGroup& faceEdges, bool multiCore) const;

	size_t numFaces(bool includeInner) const;
	size_t numPolyhedra() const;
	double getSharpAngleRad() const;

	const std::set<size_t>& getSharpVertIndices() const;
	const std::set<size_t>& getSharpEdgeIndices() const;

	void writeObj(const std::string& path, const std::vector<Index3DId>& cellIds) const;
	void writeObj(std::ostream& out, const std::vector<Index3DId>& cellIds) const;

	void writePolyMesh(const std::string& dirName);

	bool verifyTopology(bool multiCore) const;

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

	void splitSimple(const BuildCFDParams& params, bool multiCore);
	void splitAtCurvature(const BuildCFDParams& params, bool multiCore);
	void finishSplits(bool multiCore);
	void makeRefPolyhedraIfRequired(bool multiCore);
	void splitIfAdjacentRequiresIt(bool multiCore);
	void splitTopology(bool multiCore);
	void imprintTJointVertices(bool multiCore);
	void fixLinkages(bool multiCore);
	void dumpOpenCells(bool multiCore) const;

	void findFeatures();
	void findSharpVertices();
	void findSharpEdgeGroups();

	void consolidateBlocks();
	void writePolyMeshPoints(const std::string& dirName) const;
	void writePolyMeshFaces(const std::string& dirName) const;
	void writeFOAMHeader(std::ofstream& out, const std::string& foamClass, const std::string& object) const;

	template<class L>
	void runLambda(L fLambda, bool multiCore) const;
	template<class L>
	void runLambda(L fLambda, bool multiCore, unsigned int stride = 2);

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
