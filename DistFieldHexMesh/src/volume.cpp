
#include <sstream>
#include <fstream>

#define _USE_MATH_DEFINES
#include <cmath>

#include <triMesh.h>

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

Index3D Volume::s_volDim;

Volume::Volume()
{
	_sharpAngleRad = 30.0 / 180.0 * M_PI;
}

Volume::Volume(const Volume& src)
	: _originMeters(src._originMeters) 
	, _spanMeters(src._spanMeters)
	, _blocks(src._blocks)
//	, _outBlocks(src._outBlocks)
{
}

Volume::~Volume()
{
}

void Volume::startOperation()
{
/*
	_outBlocks.resize(_blocks.size());
	MultiCore::runLambda([this](size_t i)->bool {
		auto idx = calBlockIndexFromLinearIndex(i);
		if (_blocks[i]) {
			_outBlocks[i] = make_shared<Block>(*_blocks[i]);
			_outBlocks[i]->setIsOutput(true);
		} else
			_outBlocks[i] = createBlock(true, idx);
		_outBlocks[i]->setIsOutput(true);
		return true;
	}, _blocks.size(), RUN_MULTI_THREAD);
*/
}

void Volume::endOperation()
{
	MultiCore::runLambda([this](size_t i)->bool {
		if (_blocks[i]) {
			_blocks[i]->setCellDepths();
		}
		return true;
	}, _blocks.size(), RUN_MULTI_THREAD);
}

void Volume::setVolDim(const Index3D& blockSize)
{
	s_volDim = blockSize;
}

const Index3D& Volume::volDim()
{
	return s_volDim;
}

Index3D Volume::calBlockIndexFromLinearIndex(size_t linearIdx) const
{
	Index3D result;
	size_t temp = linearIdx;
	const auto& dim = volDim();

	size_t denom = dim[0] * dim[1];
	result[2] = (Index3DBaseType)(temp / denom);
	temp = temp % denom;

	denom = dim[0];

	result[1] = (Index3DBaseType)(temp / denom);
	temp = temp % denom;
	result[0] = (Index3DBaseType)temp;

	if (calLinearBlockIndex(result) != linearIdx) {
		throw runtime_error("calBlockIndexFromLinearIndex failed");
	}

	return result;
}

void Volume::findFeatures()
{
	// This function is already running on multiple cores, DO NOT calculate centroids or normals using muliple cores.
	_pModelTriMesh->buildCentroids(false);
	_pModelTriMesh->buildNormals(false);

	findSharpVertices();
	findSharpEdgeGroups();
}

void Volume::findSharpEdgeGroups()
{
	const double sinSharpAngle = sin(getSharpAngleRad());
	for (size_t edgeIdx = 0; edgeIdx < _pModelTriMesh->numEdges(); edgeIdx++) {
		const auto& edge = _pModelTriMesh->getEdge(edgeIdx);
		if (edge._numFaces == 2 && _pModelTriMesh->isEdgeSharp(edgeIdx, sinSharpAngle)) {
			_sharpEdgeIndices.insert(edgeIdx);
		}
	}
}

void Volume::splitAllCellsAtSharpVertices()
{
	size_t numBlocks;
	auto sharpVerts = getSharpVertIndices();
	vector<Vector3d> splittingPoints;
	for (size_t vertIdx : sharpVerts) {
		splittingPoints.push_back(_pModelTriMesh->getVert(vertIdx)._pt);
	}

	atomic<size_t> numSplit = 0;
	for (size_t ipt = 0; ipt < splittingPoints.size(); ipt++) {
		const auto& splitPt = splittingPoints[ipt];
		numBlocks = _blocks.size();
		MultiCore::runLambda([this, &splitPt, &numSplit](size_t linearIdx)-> bool {
			if (_blocks[linearIdx]) {
				size_t numNewCells = _blocks[linearIdx]->splitAllCellsAtPoint(splitPt);
				numSplit++;
			}
			return true;
		}, numBlocks, RUN_MULTI_THREAD);


	}
}

void Volume::findSharpVertices()
{
	const double cosSharpAngle = cos(M_PI - getSharpAngleRad());

	size_t numVerts = _pModelTriMesh->numVertices();
	for (size_t vIdx = 0; vIdx < numVerts; vIdx++) {
		const auto& vert = _pModelTriMesh->getVert(vIdx);
		double maxDp = 1;
		const auto& edgeIndices = vert._edgeIndices;
		for (size_t i = 0; i < edgeIndices.size(); i++) {
			auto edge0 = _pModelTriMesh->getEdge(edgeIndices[i]);
			size_t opIdx0 = edge0._vertIndex[0] == vIdx ? edge0._vertIndex[1] : edge0._vertIndex[0];
			const auto& vert0 = _pModelTriMesh->getVert(opIdx0);
			Vector3d v0 = (vert0._pt - vert._pt).normalized();

			for (size_t j = i + 1; j < edgeIndices.size(); j++) {
				auto edge1 = _pModelTriMesh->getEdge(edgeIndices[j]);
				size_t opIdx1 = edge1._vertIndex[0] == vIdx ? edge1._vertIndex[1] : edge1._vertIndex[0];
				const auto& vert1 = _pModelTriMesh->getVert(opIdx1);
				Vector3d v1 = (vert1._pt - vert._pt).normalized();

				double dp = v0.dot(v1);
				if (dp < maxDp) {
					maxDp = dp;
					if (maxDp < cosSharpAngle)
						break;
				}
			}
			if (maxDp < cosSharpAngle)
				break;
		}
		if (maxDp > cosSharpAngle) {
			_sharpVertIndices.insert(vIdx);
		}
	}
}

shared_ptr<Block> Volume::createBlock(const Index3D& blockIdx)
{
	const auto& dim = volDim();
	const Vector3d xAxis(1, 0, 0);
	const Vector3d yAxis(0, 1, 0);
	const Vector3d zAxis(0, 0, 1);

	size_t idx = calLinearBlockIndex(blockIdx);
	auto pBlock = _blocks[idx];
	if (pBlock)
		return pBlock;

	Vector3d origin, span;
	for (int i = 0; i < 3; i++) {
		span[i] = _spanMeters[i] / dim[i];
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

	return make_shared<Block>(this, blockIdx, pts);
}

void Volume::addAllBlocks(Block::TriMeshGroup& triMeshes, Block::glPointsGroup& faceEdges)
{
	const auto& dim = volDim();
	Vector3d origin, blockSpan;
	for (int i = 0; i < 3; i++) {
		blockSpan[i] = _spanMeters[i] / dim[i];
	}

	Index3D blkIdx;
	for (blkIdx[0] = 0; blkIdx[0] < dim[0]; blkIdx[0]++) {
		origin[1] = _originMeters[1] + blkIdx[0] * blockSpan[1];
		for (blkIdx[1] = 0; blkIdx[1] < dim[1]; blkIdx[1]++) {
			origin[1] = _originMeters[1] + blkIdx[1] * blockSpan[1];

			for (blkIdx[2] = 0; blkIdx[2] < dim[2]; blkIdx[2]++) {
				origin[2] = _originMeters[2] + blkIdx[2] * blockSpan[2];
				auto linearIdx = calLinearBlockIndex(blkIdx);
				_blocks[linearIdx] = createBlock(blkIdx);
			}
		}
	}

	makeFaceTris(triMeshes, 1, true);
}

bool Volume::blockExists(const Index3D& blockIdx) const
{
	size_t idx = calLinearBlockIndex(blockIdx);
	if (idx >= _blocks.size())
		return false;
	return _blocks[idx] != nullptr;
}

void Volume::buildCFDHexes(const CMeshPtr& pTriMesh, double maxBlockSize)
{
	_pModelTriMesh = pTriMesh;
	CMesh::BoundingBox bb = pTriMesh->getBBox();
	bb.growPercent(0.0125);
	_originMeters = bb.getMin();
	_spanMeters = bb.range();

	double minSpan = DBL_MAX;
	for (int i = 0; i < 3; i++) {
		if (_spanMeters[i] < minSpan) {
			minSpan = _spanMeters[i];
		}
	}
	double targetBlockSize = minSpan / 8;
	size_t blockDim = 1; // (size_t)(targetBlockSize / maxBlockSize + 0.5);
	Index3D::setBlockDim(blockDim);

	Index3D blockSize(
		(size_t)(_spanMeters[0] / targetBlockSize + 0.5),
		(size_t)(_spanMeters[1] / targetBlockSize + 0.5),
		(size_t)(_spanMeters[2] / targetBlockSize + 0.5)
	);

	setVolDim(blockSize);

	const auto& dim = volDim();
	Vector3d blockSpan(_spanMeters[0] / dim[0], _spanMeters[1] / dim[1], _spanMeters[2] / dim[2]);

	size_t numBlocks = dim[0] * dim[1] * dim[2];
	_blocks.resize(numBlocks);

	double sharpAngleRadians = 30.0 / 180.0 * M_PI;
	_pModelTriMesh->buildCentroids(true);
	_pModelTriMesh->buildNormals(true);
	_pModelTriMesh->calCurvatures(sharpAngleRadians, true);
	const auto& sharpEdges = _pModelTriMesh->getSharpEdgeIndices(sharpAngleRadians);
	findFeatures();

	MultiCore::runLambda([this, &blockSpan](size_t linearIdx)->bool {
		auto blockIdx = calBlockIndexFromLinearIndex(linearIdx);
		_blocks[linearIdx] = createBlock(blockIdx);
		return true;
	}, _blocks.size(), RUN_MULTI_THREAD);

	// Cannot create subBlocks until all blocks are created
	runLambda([this, &blockSpan](size_t linearIdx)->bool {
		if (_blocks[linearIdx])
			_blocks[linearIdx]->createSubBlocks();
		return true;
	}, RUN_MULTI_THREAD);

	assert(verifyTopology());

#if 1
	size_t numSimpleDivs = 2;
	size_t numCurvatureDivs = 4;

	for (size_t i = 0; i < numSimpleDivs; i++) {
		runLambda([this, &blockSpan](size_t linearIdx)->bool {
			if (_blocks[linearIdx]) {
				_blocks[linearIdx]->splitAllCellsAtCentroid();
			}
			return true;
		}, RUN_MULTI_THREAD);
	}
	assert(verifyTopology());

	for (size_t i = 0; i < numCurvatureDivs; i++) {
		runLambda([this, &blockSpan](size_t linearIdx)->bool {
			if (_blocks[linearIdx]) {
				_blocks[linearIdx]->splitAllCellsByCurvature(5.0);
			}
			return true;
		}, RUN_MULTI_THREAD);
	}
	assert(verifyTopology());
#endif


	cout << "Num polyhedra: " << numPolyhedra() << "\n";
	cout << "Num faces. All: " << numFaces(true) << ", outer: " << numFaces(false) << "\n";
}

void Volume::makeFaceTris(Block::TriMeshGroup& triMeshes, size_t minSplitNum, bool multiCore) const
{
	CBoundingBox3Dd bbox;
	bbox.merge(_originMeters);
	bbox.merge(_originMeters + _spanMeters);
	auto diagDist = bbox.range().norm();
	bbox.grow(diagDist * 0.05);


	triMeshes.resize(4);
	triMeshes[FT_OUTER].resize(_blocks.size());
	triMeshes[FT_INNER].resize(_blocks.size());
	triMeshes[FT_LAYER_BOUNDARY].resize(_blocks.size());
	triMeshes[FT_BLOCK_BOUNDARY].resize(_blocks.size());
	MultiCore::runLambda([this, &triMeshes, minSplitNum](size_t index)->bool {
		const auto& blockPtr = _blocks[index];
		if (blockPtr) {

			triMeshes[FT_OUTER][index] = blockPtr->getBlockTriMesh(FT_OUTER, minSplitNum);
			triMeshes[FT_INNER][index] = blockPtr->getBlockTriMesh(FT_INNER, minSplitNum);
			triMeshes[FT_LAYER_BOUNDARY][index] = blockPtr->getBlockTriMesh(FT_LAYER_BOUNDARY, minSplitNum);
			triMeshes[FT_BLOCK_BOUNDARY][index] = blockPtr->getBlockTriMesh(FT_BLOCK_BOUNDARY, minSplitNum);
		}
		return true;
	}, _blocks.size(), multiCore && RUN_MULTI_THREAD);
}

void Volume::makeFaceEdges(Block::glPointsGroup& faceEdges, size_t minSplitNum, bool multiCore) const
{
	faceEdges.resize(4);
	faceEdges[FT_OUTER].resize(_blocks.size());
	faceEdges[FT_INNER].resize(_blocks.size());
	faceEdges[FT_LAYER_BOUNDARY].resize(_blocks.size());
	faceEdges[FT_BLOCK_BOUNDARY].resize(_blocks.size());
	MultiCore::runLambda([this, &faceEdges, minSplitNum](size_t index)->bool {
		const auto& blockPtr = _blocks[index];
		if (blockPtr) {
			faceEdges[FT_OUTER][index] = blockPtr->makeFaceEdges(FT_OUTER, minSplitNum);
			faceEdges[FT_INNER][index] = blockPtr->makeFaceEdges(FT_INNER, minSplitNum);
			faceEdges[FT_LAYER_BOUNDARY][index] = blockPtr->makeFaceEdges(FT_BLOCK_BOUNDARY, minSplitNum);
			faceEdges[FT_BLOCK_BOUNDARY][index] = blockPtr->makeFaceEdges(FT_BLOCK_BOUNDARY, minSplitNum);
		}
		return true;
	}, _blocks.size(), multiCore && RUN_MULTI_THREAD);
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

void Volume::writePolyMeshPoints(const string& dirName) const
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

void Volume::writeFOAMHeader(ofstream& out, const string& foamClass, const string& object) const
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

bool Volume::verifyTopology() const
{
	bool result = true;

	runLambda([this, &result](size_t linearIdx)->bool {
		if (_blocks[linearIdx])
			result = result && _blocks[linearIdx]->verifyTopology();
		return true;
	}, RUN_MULTI_THREAD);
	return result;
}

template<class L>
void Volume::runLambda(L fLambda, bool multiCore) const
{
	MultiCore::runLambda([this, fLambda](size_t threadNum, size_t numThreads) {
		Index3D idx;
		for (size_t i = threadNum; i < _blocks.size(); i += numThreads) {
			fLambda(i);
		}
	}, multiCore);
}

template<class L>
void Volume::runLambda(L fLambda, bool multiCore)
{
	const Index3DBaseType stride = 3;
	Index3D phaseIdx, idx;

	startOperation();

	// Pass one, process all cells. That can leave faces in an interim state.
	// If the interim cell is also modified, it should be taken care of on pass 1
	// Adjacents cells with face splits or vertex insertions may be left behind
	for (phaseIdx[0] = 0; phaseIdx[0] < stride; phaseIdx[0]++) {
		for (phaseIdx[1] = 0; phaseIdx[1] < stride; phaseIdx[1]++) {
			for (phaseIdx[2] = 0; phaseIdx[2] < stride; phaseIdx[2]++) {
				// Collect the indices for all blocks in this phase
				vector<size_t> blocksToProcess;

				for (idx[0] = phaseIdx[0]; idx[0] < s_volDim[0]; idx[0] += stride) {
					for (idx[1] = phaseIdx[1]; idx[1] < s_volDim[1]; idx[1] += stride) {
						for (idx[2] = phaseIdx[2]; idx[2] < s_volDim[2]; idx[2] += stride) {
							size_t linearIdx = calLinearBlockIndex(idx);
							blocksToProcess.push_back(linearIdx);
						}
					}
				}

				// Process those blocks in undetermined order
				MultiCore::runLambda([fLambda](size_t linearIdx)->bool {
					return fLambda(linearIdx);
				}, blocksToProcess, multiCore);

			}
		}
	}

	endOperation();
}
