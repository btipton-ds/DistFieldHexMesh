
#include <sstream>
#include <fstream>

#include <triMesh.h>

#include <cell.h>
#include <block.h>
#include <volume.h>
#include <MultiCoreUtil.h>
#include <vertex.h>
#include <filesystem>
#include <stdexcept>

using namespace std;
using namespace DFHM;
using namespace TriMesh;

#define RUN_MULTI_THREAD true

namespace
{

inline Index3D assignAxes(const Index3D& indexIn, const Index3D& axisOrder)
{
	Index3D result;
	result[axisOrder[0]] = indexIn[0];
	result[axisOrder[1]] = indexIn[1];
	result[axisOrder[2]] = indexIn[2];
	return result;
}

}

Volume::Volume(const Index3D& blockSize)
{
	setBlockDims(blockSize);
}

Volume::Volume(const Volume& src)
	: _originMeters(src._originMeters) 
	, _spanMeters(src._spanMeters)
	, _blockDim(src._blockDim)
	, _blocks(src._blocks)
{
}

Volume::~Volume()
{
}

void Volume::setBlockDims(const Index3D& blockSize)
{
	_blockDim = blockSize;
	_blocks.resize(_blockDim[0] * _blockDim[1] * _blockDim[2]);
}

const Index3D& Volume::getBlockDims() const
{
	return _blockDim;
}

bool Volume::cellExists(size_t ix, size_t iy, size_t iz) const
{
	Index3DBaseType blkDim = (Index3DBaseType)Block::getMinBlockDim();
	Index3D idx(ix, iy, iz);
	Index3D blockIdx, cellIdx;
	for (int i = 0; i < 3; i++) {
		blockIdx[i] = idx[i] / blkDim;
		cellIdx[i] = idx[i] % blkDim;
	}
	if (!blockExists(blockIdx))
		return false;
	const auto& block = getBlock(blockIdx);
	return block.cellExists(cellIdx);
}

bool Volume::cellExists(const Index3D& blockIdx) const
{
	return cellExists(blockIdx[0], blockIdx[1], blockIdx[2]);
}

Cell& Volume::getCell(size_t ix, size_t iy, size_t iz)
{
	Index3DBaseType blkDim = (Index3DBaseType)Block::getMinBlockDim();
	Index3D idx(ix, iy, iz);
	Index3D blockIdx, cellIdx;
	for (int i = 0; i < 3; i++) {
		blockIdx[i] = idx[i] / blkDim;
		cellIdx[i] = idx[i] % blkDim;
	}
	auto& block = getBlock(blockIdx);
	auto& cell = block.getCell(cellIdx);
	return cell;
}

const Cell& Volume::getCell(size_t ix, size_t iy, size_t iz) const
{
	Index3DBaseType blkDim = (Index3DBaseType)Block::getMinBlockDim();
	Index3D idx(ix, iy, iz);
	Index3D blockIdx, cellIdx;
	for (int i = 0; i < 3; i++) {
		blockIdx[i] = idx[i] / blkDim;
		cellIdx[i] = idx[i] % blkDim;
	}
	auto& block = getBlock(blockIdx);
	auto& cell = block.getCell(cellIdx);
	return cell;
}

void Volume::processRayHit(const RayHit& triHit, int rayAxis, const Vector3d& blockSpan, const Vector3d& cellSpan, size_t& blockIdx, size_t& cellIdx)
{

	double dist0 = triHit.dist;
	if (dist0 < 0)
		dist0 = 0;
	else if (dist0 >= _spanMeters[rayAxis])
		dist0 = _spanMeters[rayAxis];

	double w0 = dist0 / _spanMeters[rayAxis];

	blockIdx = (size_t)(w0 * _blockDim[rayAxis]);
	if (blockIdx >= _blockDim[rayAxis])
		blockIdx = _blockDim[rayAxis] - 1;
	double dist1 = dist0 - (blockIdx * blockSpan[rayAxis]);

	double w1 = dist1 / blockSpan[rayAxis];
	cellIdx = (size_t)(w1 * Block::getMinBlockDim());
	if (cellIdx >= Block::getMinBlockDim())
		cellIdx = Block::getMinBlockDim() - 1;
}

Block& Volume::addBlock(size_t ix, size_t iy, size_t iz)
{
	return addBlock(Index3D(ix, iy, iz));
}

Block& Volume::addBlock(const Index3D& blockIdx)
{
	const Vector3d xAxis(1, 0, 0);
	const Vector3d yAxis(0, 1, 0);
	const Vector3d zAxis(0, 0, 1);

	size_t idx = calLinearBlockIndex(blockIdx);
	auto pBlock = _blocks[idx];
	if (pBlock)
		return *pBlock;

	Vector3d origin, span;
	for (int i = 0; i < 3; i++) {
		span[i] = _spanMeters[i] / _blockDim[i];
	}

	origin[0] = _originMeters[0] + blockIdx[0] * span[0];
	origin[1] = _originMeters[1] + blockIdx[1] * span[1];
	origin[2] = _originMeters[2] + blockIdx[2] * span[2];
	vector<Vector3d> pts = {
		origin,
		origin + xAxis * span[0],
		origin + xAxis * span[0] + yAxis * span[1],
		origin + yAxis * span[1],

		origin + zAxis * span[2],
		origin + zAxis * span[2] + xAxis * span[0],
		origin + zAxis * span[2] + xAxis * span[0] + yAxis * span[1],
		origin + zAxis * span[2] + yAxis * span[1],
	};

	_blocks[idx] = make_shared<Block>(this, blockIdx, pts);

	pBlock = _blocks[idx];

	return *pBlock;
}

std::vector<TriMesh::CMeshPtr> Volume::addAllBlocks()
{
	Vector3d origin, blockSpan;
	for (int i = 0; i < 3; i++) {
		blockSpan[i] = _spanMeters[i] / _blockDim[i];
	}

	Index3D blkIdx;
	for (blkIdx[0] = 0; blkIdx[0] < _blockDim[0]; blkIdx[0]++) {
		origin[1] = _originMeters[1] + blkIdx[0] * blockSpan[1];
		for (blkIdx[1] = 0; blkIdx[1] < _blockDim[1]; blkIdx[1]++) {
			origin[1] = _originMeters[1] + blkIdx[1] * blockSpan[1];

			for (blkIdx[2] = 0; blkIdx[2] < _blockDim[2]; blkIdx[2]++) {
				origin[2] = _originMeters[2] + blkIdx[2] * blockSpan[2];
				Block& block = addBlock(blkIdx);
			}
		}
	}

	return makeTris(true, true);
}

bool Volume::blockExists(size_t ix, size_t iy, size_t iz) const
{
	size_t idx = calLinearBlockIndex(ix, iy, iz);
	if (idx >= _blocks.size())
		return false;
	return _blocks[idx] != nullptr;
}

bool Volume::blockExists(const Index3D& blockIdx) const
{
	return blockExists(blockIdx[0], blockIdx[1], blockIdx[2]);
}

bool Volume::blockInBounds(const Index3D& blockIdx) const
{
	return (blockIdx[0] < _blockDim[0] && blockIdx[1] < _blockDim[1] && blockIdx[2] < _blockDim[1]);
}

const Block& Volume::getBlock(size_t ix, size_t iy, size_t iz) const
{
	size_t idx = calLinearBlockIndex(ix, iy, iz);
	if (idx < _blocks.size()) {
		auto pBlock = _blocks[idx];
		if (!pBlock)
			throw runtime_error("Volume::getBlock block not allocated");
		return *pBlock;
	}
	throw runtime_error("Volume::getBlock index out of range");
}

Block& Volume::getBlock(size_t ix, size_t iy, size_t iz)
{
	size_t idx = calLinearBlockIndex(ix, iy, iz);
	if (idx < _blocks.size()) {
		auto pBlock = _blocks[idx];
		if (!pBlock)
			throw runtime_error("Volume::getBlock not allocated");
		return *pBlock;
	}
	throw runtime_error("Volume::getBlock index out of range");
}

std::vector<TriMesh::CMeshPtr> Volume::buildCFDHexes(const CMeshPtr& pTriMesh, double targetBlockSize, bool outerFacesOnly)
{
	_pModelTriMesh = pTriMesh;
	CMesh::BoundingBox bb = pTriMesh->getBBox();
	_originMeters = bb.getMin();
	_spanMeters = bb.range();

	Index3D blockSize(
		(size_t)(_spanMeters[0] / targetBlockSize + 0.5),
		(size_t)(_spanMeters[1] / targetBlockSize + 0.5),
		(size_t)(_spanMeters[2] / targetBlockSize + 0.5)
	);

	setBlockDims(blockSize);

	Vector3d blockSpan(_spanMeters[0] / _blockDim[0], _spanMeters[1] / _blockDim[1], _spanMeters[2] / _blockDim[2]);

	size_t numBlocks = _blockDim[0] * _blockDim[1] * _blockDim[2];

#define QUICK_TEST 0
	MultiCore::runLambda([this, &blockSpan](size_t linearIdx) {
		Vector3d blockOrigin;

		Index3D blockIdx = calBlockIndexFromLinearIndex(linearIdx);

		blockOrigin[0] = _originMeters[0] + blockIdx[0] * blockSpan[0];
		blockOrigin[1] = _originMeters[1] + blockIdx[1] * blockSpan[1];
		blockOrigin[2] = _originMeters[2] + blockIdx[2] * blockSpan[2];
		CMesh::BoundingBox blbb;
		blbb.merge(blockOrigin);
		blbb.merge(blockOrigin + blockSpan);
		double span = blbb.range().norm();
		blbb.grow(0.05 * span);
					
		auto& bl = addBlock(blockIdx);
		vector<size_t> triIndices;
		if (_pModelTriMesh->findTris(blbb, triIndices, CMesh::BoxTestType::Intersects)) {
			cout << "Processing : " << (linearIdx * 100.0 / _blocks.size()) << "%\n";
			bl.addTris(_pModelTriMesh, triIndices);
		}
		
	}, numBlocks, RUN_MULTI_THREAD);

#if QUICK_TEST
	map<size_t, std::shared_ptr<Block>> orderedBlocks;
	for (size_t linearIdx = 0; linearIdx < _blocks.size(); linearIdx++) {
		if (_blocks[linearIdx] && _blocks[linearIdx]->getModelMesh()) {
			size_t numTris = _blocks[linearIdx]->getModelMesh()->numTris();
			orderedBlocks.insert(make_pair(numTris, _blocks[linearIdx]));
		}
	}

	size_t count = 0;
	for (auto iter = orderedBlocks.rbegin(); iter != orderedBlocks.rend(); iter++) {
		const auto& idx = iter->second->getBlockIdx();
		iter->second->processTris();
		count++;

		if (count > 3)
			break;
	}
#else
	MultiCore::runLambda([this, &blockSpan](size_t linearIdx) {
		if (_blocks[linearIdx]) {
			_blocks[linearIdx]->processTris();
		}
		}, numBlocks, RUN_MULTI_THREAD);
#endif


	cout << "Num polyhedra: " << numPolyhedra() << "\n";
	cout << "Num faces. All: " << numFaces(true) << ", outer: " << numFaces(false) << "\n";

	auto result = makeTris(outerFacesOnly, true);

	return result;
}

vector<TriMesh::CMeshPtr> Volume::makeTris(bool outerOnly, bool multiCore)
{
	CBoundingBox3Dd bbox;
	bbox.merge(_originMeters);
	bbox.merge(_originMeters + _spanMeters);
	auto diagDist = bbox.range().norm();
	bbox.grow(diagDist * 0.05);


	vector<TriMesh::CMeshPtr> temp;
	temp.resize(_blocks.size());
	MultiCore::runLambda([this, &temp, outerOnly](size_t index) {
		const auto& blockPtr = _blocks[index];
		if (!blockPtr)
			return;
		auto pMesh = blockPtr->getBlockTriMesh(outerOnly);
		if (pMesh) {
			temp[index] = pMesh;
		}
	}, _blocks.size(), multiCore && RUN_MULTI_THREAD);

	// Remove the null pointers from the result
	vector<TriMesh::CMeshPtr> result;
	size_t numTris = 0;
	for (auto p : temp) {
		if (p) {
			numTris += p->numTris();
			result.push_back(p);
		}
	}

	cout << "Num tris: " << numTris << "\n";
	return result;
}

size_t Volume::numFaces(bool includeInner) const
{
	size_t result = 0;
	for (auto pBlock : _blocks) {
		if (pBlock)
			result += pBlock->numFaces(includeInner);
	}

	return result;
}

size_t Volume::numPolyhedra() const
{
	size_t result = 0;
	for (auto pBlock : _blocks) {
		if (pBlock)
			result += pBlock->numPolyhedra();
	}

	return result;
}

namespace {
	void appendDir(string& dirName, const string& str)
	{
		if (dirName.find_last_of("/") != dirName.length() - 1)
			dirName += "/";
		dirName += str;
	}
}

void Volume::writePolyMesh(const string& dirNameIn) const
{

	string dirName(dirNameIn);
	if (dirName.find("constant") == string::npos)
		appendDir(dirName, "constant");
	if (dirName.find("polyMesh") == string::npos)
		appendDir(dirName, "polyMesh");
	
	filesystem::path dirPath(dirName);
	filesystem::remove_all(dirPath);
	filesystem::create_directories(dirPath);
	writePolyMeshPoints(dirName);
}

void Volume::writePolyMeshPoints(const std::string& dirName) const
{
#if 0
	ofstream out(dirName + "/points", ios_base::binary);
	writeFOAMHeader(out, "vectorField", "points");
	size_t numPoints = 0;
	_vertexPool.iterateInOrder([&numPoints](const ObjectPoolId& id, const Vertex& vert) {
		numPoints++;
	});
	out << numPoints << "\n";
	_vertexPool.iterateInOrder([&out, &numPoints](const ObjectPoolId& id, const Vertex& vert) {
		char openParen = '(';
		char closeParen = ')';

		const double& x = vert.getPoint()[0];
		const double& y = vert.getPoint()[1];
		const double& z = vert.getPoint()[2];

		out.write(&openParen, 1);
		out.write((char*)&x, sizeof(x));
		out.write((char*)&y, sizeof(y));
		out.write((char*)&z, sizeof(z));
		out.write(&closeParen, 1);
	});
#endif
}

void Volume::writeFOAMHeader(std::ofstream& out, const std::string& foamClass, const std::string& object) const
{
	out << "/*--------------------------------*- C++ -*----------------------------------*/\n";
	out << "| =========                 |                                                 |\n";
	out << "| \\ / F ield | OpenFOAM: The Open Source CFD Toolbox           |\n";
	out << "| \\ / O peration | Version:  v1812                                 |\n";
	out << "|   \\ / A nd | Web:      www.OpenFOAM.com                      |\n";
	out << "|    \\ / M anipulation  |                                                 |\n";
	out << "/* -------------------------------------------------------------------------- - */\n";
	out << "FoamFile\n";
	out << "	{\n";
	out << "		version     2.0;\n";
	out << "		format      binary;\n";
	out << "		class       " << foamClass << ";\n";
	out << "		arch        \"LSB; label = 32; scalar = 64\";\n";
	out << "		location    \"constant / polyMesh\";\n";
	out << "		object      " << object << ";\n";
	out << "	}\n";
	out << "		// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * //\n";
	out << "\n";
	out << "\n";

}

