
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
#define QUICK_TEST 0

Index3D Volume::s_volDim;

Volume::Volume()
{
	_sharpAngleRad = 30.0 / 180.0 * M_PI;
}

Volume::Volume(const Volume& src)
	: _originMeters(src._originMeters) 
	, _spanMeters(src._spanMeters)
	, _blocks(src._blocks)
{
}

Volume::~Volume()
{
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
	const double tol = 1.0e-3;

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

void Volume::findSharpVertices()
{
	const double cosSharpAngle = cos(M_PI - getSharpAngleRad());
	const double tol = 1.0e-3;

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


void Volume::processRayHit(const RayHit& triHit, int rayAxis, const Vector3d& blockSpan, const Vector3d& subBlockSpan, size_t& blockIdx, size_t& subBlockIdx)
{
	const auto& dim = volDim();

	double dist0 = triHit.dist;
	if (dist0 < 0)
		dist0 = 0;
	else if (dist0 >= _spanMeters[rayAxis])
		dist0 = _spanMeters[rayAxis];

	double w0 = dist0 / _spanMeters[rayAxis];

	blockIdx = (size_t)(w0 * dim[rayAxis]);
	if (blockIdx >= dim[rayAxis])
		blockIdx = dim[rayAxis] - 1;
	double dist1 = dist0 - (blockIdx * blockSpan[rayAxis]);

	double w1 = dist1 / blockSpan[rayAxis];
	subBlockIdx = (size_t)(w1 * Index3D::getBlockDim());
	if (subBlockIdx >= Index3D::getBlockDim())
		subBlockIdx = Index3D::getBlockDim() - 1;
}

Block& Volume::addBlock(const Index3D& blockIdx)
{
	const auto& dim = volDim();
	const Vector3d xAxis(1, 0, 0);
	const Vector3d yAxis(0, 1, 0);
	const Vector3d zAxis(0, 0, 1);

	size_t idx = calLinearBlockIndex(blockIdx);
	auto pBlock = _blocks[idx];
	if (pBlock)
		return *pBlock;

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

	_blocks[idx] = make_shared<Block>(this, blockIdx, pts);

	pBlock = _blocks[idx];

	return *pBlock;
}

void Volume::addAllBlocks(vector<TriMesh::CMeshPtr>& triMeshes, vector<shared_ptr<vector<float>>>& faceEdges)
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
				Block& block = addBlock(blkIdx);
			}
		}
	}

	makeTris(triMeshes, true, true);
}

bool Volume::blockExists(const Index3D& blockIdx) const
{
	size_t idx = calLinearBlockIndex(blockIdx);
	if (idx >= _blocks.size())
		return false;
	return _blocks[idx] != nullptr;
}

bool Volume::blockInBounds(const Index3D& blockIdx) const
{
	const auto& dim = volDim();
	return (blockIdx[0] < dim[0] && blockIdx[1] < dim[1] && blockIdx[2] < dim[1]);
}

const Block& Volume::getBlock(const Index3D& blockIdx) const
{
	size_t idx = calLinearBlockIndex(blockIdx);
	if (idx < _blocks.size()) {
		auto pBlock = _blocks[idx];
		if (!pBlock)
			throw runtime_error("Volume::getBlock block not allocated");
		return *pBlock;
	}
	throw runtime_error("Volume::getBlock index out of range");
}

Block& Volume::getBlock(const Index3D& blockIdx)
{
	size_t idx = calLinearBlockIndex(blockIdx);
	if (idx < _blocks.size()) {
		auto pBlock = _blocks[idx];
		if (!pBlock)
			throw runtime_error("Volume::getBlock not allocated");
		return *pBlock;
	}
	throw runtime_error("Volume::getBlock index out of range");
}

void Volume::buildCFDHexes(const CMeshPtr& pTriMesh, double targetBlockSize, bool outerFacesOnly)
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
		bl.addTris(_pModelTriMesh);
		
	}, numBlocks, RUN_MULTI_THREAD);

	size_t count = 0;
	MultiCore::runLambda([this, &blockSpan, &count](size_t linearIdx) {
		if (_blocks[linearIdx]) {
			cout << "Processing : " << (linearIdx * 100.0 / _blocks.size()) << "%\n";
#if QUICK_TEST
			if (count < 5 && _blocks[linearIdx]->processTris() > 0)
				count++;
#else
			_blocks[linearIdx]->processTris();
#endif
		}
	}, numBlocks, !QUICK_TEST && RUN_MULTI_THREAD);


	cout << "Num polyhedra: " << numPolyhedra() << "\n";
	cout << "Num faces. All: " << numFaces(true) << ", outer: " << numFaces(false) << "\n";
}

void Volume::makeTris(vector<TriMesh::CMeshPtr>& triMeshes, bool outerOnly, bool multiCore)
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

	for (auto p : temp) {
		if (p) {
			triMeshes.push_back(p);
		}
	}
}

void Volume::makeFaceEdges(vector<shared_ptr<vector<float>>>& faceEdges, bool outerOnly, bool multiCore)
{
	vector<shared_ptr<vector<float>>> temp;
	temp.resize(_blocks.size());
	MultiCore::runLambda([this, &temp, outerOnly](size_t index) {
		const auto& blockPtr = _blocks[index];
		if (!blockPtr)
			return;
		shared_ptr<vector<float>> pEdges =  blockPtr->makeFaceEdges(outerOnly);
		if (pEdges && !pEdges->empty()) {
			temp[index] = pEdges;
		}
	}, _blocks.size(), multiCore && RUN_MULTI_THREAD);

	for (const auto& p : temp) {
		if (p)
			faceEdges.push_back(p);
	}
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

