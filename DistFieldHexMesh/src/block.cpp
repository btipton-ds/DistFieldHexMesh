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

#include <vector>
#include <algorithm>
#include <fstream>
#include <defines.h>
#include <cmath>
#include <filesystem>

#include <tm_ioUtil.h>
#include <tm_spatialSearch.hpp>
#include <tm_math.h>

#include <tolerances.h>
#include <index3D.h>
#include <splitParams.h>
#include <vertex.h>
#include <edge.h>
#include <polygon.h>
#include <polyhedron.h>
#include <polyhedronSplitter.h>
#include <block.h>
#include <volume.h>
#include <logger.h>
#include <meshData.h>
#include <appData.h>

//using namespace std;
using namespace DFHM;

atomic<size_t> Block::GlPoints::_statId = 0;

Block::GlPoints::GlPoints()
{
	_id = _statId++;
}
Block::GlPoints::GlPoints(const GlPoints& src)
	: vector<float>(src)
{
	_id = _statId++;
}

Block::Block(Volume* pVol, const Index3D& blockIdx, const vector<Vector3d>& pts, bool forReading)
	: Block(pVol, blockIdx, pts.data(), forReading)
{
	assert(pts.size() == 8);
}

Block::Block(Volume* pVol, const Index3D& blockIdx, const Vector3d pts[8], bool forReading)
	: _blockIdx(blockIdx)
	, _pVol(pVol)
	, _modelData(this)
	, _vertices(this, true)
	, _refData(this)
#if USE_MULTI_THREAD_CONTAINERS			
	, _heap(1, 512)
#endif
{
	_blockDim = Index3D::getBlockDim();
//	_corners.resize(8);
	for (size_t i = 0; i < 8; i++) {
		_boundBox.merge(pts[i]);
		_corners[i] = pts[i];
	}

	_innerBoundBox.merge(_boundBox.getMin());
	Vector3d range = _boundBox.range();
	double k = 1.0 / 8;
	Vector3d v(
		(_blockDim - k) / _blockDim * range[0],
		(_blockDim - k) / _blockDim * range[1],
		(_blockDim - k) / _blockDim * range[2]
		);
	Vector3d pt = _boundBox.getMin() + v;
	_innerBoundBox.merge(pt);

	CBoundingBox3Dd treeBBox(_boundBox);
	treeBBox.grow(Tolerance::sameDistTol());
	_pVertTree = make_shared<SearchTree>(treeBBox);
		
	// This is close to working, but the full search is finding solutions the partial search is not
	auto pMeshData = _pVol->getAppData()->getMeshData();
	if (pMeshData && !forReading) { // if we're reading, we'll read indices in to save time
		for (auto& pair : *pMeshData) {
			auto pData = pair.second;
			auto pMesh = pData->getMesh();
			pMesh->findEdges(_boundBox, _edgeIndices);
			pMesh->findTris(_boundBox, _triIndices);
		}
	}

#if USE_MULTI_THREAD_CONTAINERS			
	MultiCore::scoped_set_local_heap st(&_heap);
	_pLocalData = new LocalData();
#endif
}

Block::~Block()
{
	delete _pLocalData;
}

void Block::clear()
{
#if USE_MULTI_THREAD_CONTAINERS			
	MultiCore::scoped_set_local_heap st(&_heap);
#endif

	delete (_pLocalData);
	_pLocalData = nullptr;

	_baseIdxVerts = 0;
	_baseIdxPolygons = 0;
	_baseIdxPolyhedra = 0;

	_edgeIndices.clear();
	_triIndices.clear();
	_pVertTree->clear();
	_vertices.clear();
	_modelData.clear();
	_refData.clear();
}

void Block::ModelData::clear()
{
	_polygons.clear();
	_polyhedra.clear();
}

size_t Block::ModelData::numBytes() const
{
	size_t result = 0;
	result += sizeof(ModelData);
	result += _polygons.numBytes();
	result += _polyhedra.numBytes();

	return result;
}

void Block::ModelData::remapIds(const map<Index3D, Index3D>& idRemap)
{
	_polygons.iterateInOrder([&idRemap](const Index3DId& faceId, Polygon& face) {
		face.remapId(idRemap);
	});

	_polyhedra.iterateInOrder([&idRemap](const Index3DId& faceId, Polyhedron& cell) {
		cell.remapId(idRemap);
	});
}

bool Block::ModelData::verifyIndices(const Index3D& idx) const
{
	bool result = true;

	_polygons.iterateInOrder([&idx, &result](const Index3DId& faceId, const Polygon& face) {
		if (!face.verifyIndices(idx))
			result = false;
	});

	_polyhedra.iterateInOrder([&idx, &result](const Index3DId& faceId, const Polyhedron& cell) {
		if (!cell.verifyIndices(idx))
			result = false;
	});

	return result;
}

const Index3D& Block::getBlockIdx() const
{
	return _blockIdx;
}

Index3D Block::calSubBlockIndexFromLinear(size_t linearIdx) const
{
	Index3D result;

	size_t temp = linearIdx;

	size_t denom = _blockDim * _blockDim;
	result[2] = (Index3DBaseType) (temp / denom);
	temp = temp % denom;

	denom = _blockDim;
	result[1] = (Index3DBaseType) (temp / denom);
	temp = temp % denom;

	result[0] = (Index3DBaseType)temp;
	if (calLinearSubBlockIndex(result) != linearIdx) {
		throw runtime_error("calSubBlockIndexFromLinear failed.");
	}
	return result;
}

void Block::calBlockOriginSpan(Vector3d& origin, Vector3d& span) const
{
	Vector3d axis;
	const auto& pts = getCornerPts();
	origin = pts[0];

	span = Vector3d(
		(pts[1] - origin).norm(),
		(pts[3] - origin).norm(),
		(pts[4] - origin).norm()
	);

}

const std::shared_ptr<const std::map<std::wstring, MeshDataPtr>> Block::getModelMeshData() const
{
	auto& pAppData = _pVol->getAppData();
	return pAppData->getMeshData();
}

void Block::remapBlockIndices(const map<Index3D, Index3D>& idRemap)
{
	auto iter = idRemap.find(_blockIdx);
	if (iter != idRemap.end())
		_blockIdx = iter->second;


	_vertices.iterateInOrder([&idRemap](const Index3DId& id, Vertex& vert) {
		vert.remapId(idRemap);
	});
	_modelData.remapIds(idRemap);
	_refData.remapIds(idRemap);
}

void Block::getAdjacentBlockIndices(MTC::set<Index3D>& indices) const
{
	for (int i = -1; i <= 1; i++) {
		for (int j = -1; j <= 1; j++) {
			for (int k = -1; k <= 1; k++) {
				Index3D idx = _blockIdx + Index3D(i, j, k);
				idx.clampInBounds(_pVol->volDim());
				indices.insert(idx);
			}
		}
	}
}

Index3D Block::determineOwnerBlockIdxFromRatios(const Vector3d& ratios) const
{
	const double tol = Tolerance::paramTol();
	const auto& volBounds = _pVol->volDim();

	Index3D result(_blockIdx);
	for (int i = 0; i < 3; i++) {
		if (ratios[i] >= 1 - tol) {
			result[i] += 1;

			// Clamp it inside volume bounds. If it's shared by a block that doesn't exist, there's no chance
			// of a thread access conflict on that boundary
			if (result[i] >= volBounds[i]) {
				result[i] = volBounds[i] - 1;
			}
		}
	}

	return result;
}

Index3D Block::determineOwnerBlockIdx(const Vector3d& point) const
{
	return _pVol->determineOwnerBlockIdx(point);
}

Index3D Block::determineOwnerBlockIdx(const Vertex& vert) const
{
	return determineOwnerBlockIdx(vert.getPoint());
}

Index3D Block::determineOwnerBlockIdx(const MTC::vector<Vector3d>& points) const
{
	auto volBounds = _pVol->volDim();

	Vector3d ctr(0, 0, 0);
	for (const auto& pt : points) {
		ctr += pt;
	}
	ctr = ctr / points.size();

	return determineOwnerBlockIdx(ctr);
}

Index3D Block::determineOwnerBlockIdx(const MTC::vector<Index3DId>& verts) const
{
	MTC::vector<Vector3d> pts;
	pts.resize(verts.size());
	for (size_t i = 0; i < verts.size(); i++) {
		pts[i] = getVertexPoint(verts[i]);
	}
	return determineOwnerBlockIdx(pts);
}

Index3D Block::determineOwnerBlockIdx(const Polygon& face) const
{
	auto volBounds = _pVol->volDim();

	Vector3d ctr(0, 0, 0);
	const auto& vertIds = face.getVertexIds();
	for (const auto& vertId : vertIds) {
		auto pt = getVertexPoint(vertId);
		ctr += pt;
	}
	ctr = ctr / vertIds.size();
	return determineOwnerBlockIdx(ctr);
}

Index3DId Block::getVertexIdOfPoint(const Vector3d& point)
{
	auto blkIdx = determineOwnerBlockIdx(point);
	auto pBlockOwner = _pVol->getBlockPtr(blkIdx);
	if (pBlockOwner) {
		Vertex vert(point);
		return pBlockOwner->_vertices.findOrAdd(vert);
	}
	assert(!"bad index");
	return Index3DId();
}

size_t Block::numFaces(bool includeInner) const
{
	size_t result = 0;
	_modelData._polygons.iterateInOrder([&result, includeInner](const Index3DId& id, const Polygon& face) {
		if (includeInner || face.isBlockBoundary())
			result++;
	});
	return result;
}

size_t Block::numPolyhedra() const
{
	return _modelData._polyhedra.size();
}

size_t Block::numBytes() const
{
	size_t result = 0;
	result += _modelData.numBytes();
	result += _refData.numBytes();
	return result;
}

bool Block::verifyTopology() const
{
	bool result = true;
	MTC::vector<Index3DId> badCellIds;
	_modelData._polyhedra.iterateInOrder([&result, &badCellIds](const Index3DId& id, const Polyhedron& cell) {
		if (!cell.verifyTopology()) {
			badCellIds.push_back(id);
			result = false;
		}
	});

	if (!result) {
		dumpPolyhedraObj(badCellIds, false, false, false);
	}

	return result;
}

bool Block::verifyDeterminOwnerBlockIndex() const
{
	// Check the centroid is in our block
	Vector3d ctr(0, 0, 0);
	for (int i = 0; i < 8; i++)
		ctr += _corners[i];
	ctr /= 8;

	Index3D 
		blkIdx = determineOwnerBlockIdx(ctr);
	if (blkIdx != getBlockIdx())
		return false;

	Vector3d facePts[4];
	for (int i = 0; i < 6; i++) {
		CubeFaceType ft = (CubeFaceType) i;
		switch (ft) {
		case CFT_BACK:
			facePts[0] = _corners[0];
			facePts[1] = _corners[4];
			facePts[2] = _corners[7];
			facePts[3] = _corners[3];
			break;
		case CFT_FRONT:
			facePts[0] = _corners[1];
			facePts[1] = _corners[2];
			facePts[2] = _corners[6];
			facePts[3] = _corners[5];
			break;
		case CFT_BOTTOM:
			facePts[0] = _corners[0];
			facePts[1] = _corners[1];
			facePts[2] = _corners[2];
			facePts[3] = _corners[3];
			break;
		case CFT_TOP:
			facePts[0] = _corners[4];
			facePts[1] = _corners[5];
			facePts[2] = _corners[6];
			facePts[3] = _corners[7];
			break;
		case CFT_LEFT:
			facePts[0] = _corners[0];
			facePts[1] = _corners[1];
			facePts[2] = _corners[5];
			facePts[3] = _corners[6];
			break;
		case CFT_RIGHT:
			facePts[0] = _corners[2];
			facePts[1] = _corners[6];
			facePts[2] = _corners[7];
			facePts[3] = _corners[3];
			break;
		default:
			return false;
		}

		size_t steps = 3;
		for (size_t i = 0; i < steps; i++) {
			double t = i / (steps - 1.0);
			for (size_t j = 0; j < steps; j++) {
				double u = i / (steps - 1.0);

				Vector3d testPt = BI_LERP(facePts[0], facePts[1], facePts[2], facePts[3], t, u);
				blkIdx = determineOwnerBlockIdx(testPt);
				switch (ft) {
				case CFT_FRONT:
					blkIdx[0] += 1;
					if (i == steps - 1)
						blkIdx[1] += 1;
					if (j == steps - 1)
						blkIdx[2] += 1;
					break;
				case CFT_TOP:
					blkIdx[2] += 1;
					if (i == steps - 1)
						blkIdx[0] += 1;
					if (j == steps - 1)
						blkIdx[1] += 1;
					break;
				case CFT_RIGHT:
					blkIdx[1] += 1;
					if (i == steps - 1)
						blkIdx[0] += 1;
					if (j == steps - 1)
						blkIdx[2] += 1;
					break;
				default:
					break;
				}

				const auto& volDim = _pVol->volDim();
				for (int i = 0; i < 3; i++) {
					if (blkIdx[i] >= volDim[i])
						blkIdx[i] = volDim[i] - 1;
				}

				if (blkIdx != getBlockIdx())
					return false;
			}
		}

	}

	return true;
}

bool Block::verifyIndices(const Index3D& idx) const
{
	if (idx != _blockIdx)
		return false;

	auto OPBlkIdx = getBlockIdx();
	if (OPBlkIdx != idx)
		return false;

	auto pOwner = _pVol->getBlockPtr(idx);
	if (pOwner != this)
		return false;

	_modelData.verifyIndices(idx);
	_refData.verifyIndices(idx);

	bool result = true;
	_vertices.iterateInOrder([&idx, &result](const Index3DId& id, const Vertex& vert) {
		if (!vert.verifyIndices(idx))
			result = false;
	});

	return true;
}

void Block::createBlockCells(TopolgyState refState)
{
	Index3D idx;
	for (idx[0] = 0; idx[0] < _blockDim; idx[0]++) {
		for (idx[1] = 0; idx[1] < _blockDim; idx[1]++) {
			for (idx[2] = 0; idx[2] < _blockDim; idx[2]++) {
				addHexCell(_corners, _blockDim, idx, false);
			}
		}
	}
}

void Block::createSubBlocksForHexSubBlock(const Vector3d* blockPts, const Index3D& subBlockIdx)
{

	auto blockCornerPts = getCornerPts();	
	auto polyId = addHexCell(blockCornerPts, _blockDim, subBlockIdx, false);
}

const vector<Vector3d>& Block::getCornerPts() const
{
	return _corners;
}

MTC::vector<Index3DId> Block::getSubBlockCornerVertIds(const std::vector<Vector3d>& blockPts, size_t divs, const Index3D& index)
{
	MTC::vector<Index3DId> result;
	result.reserve(8);
	if (divs <= 1) {
		for (const auto& pt : blockPts) {
			auto vertId = addVertex(pt);
			result.push_back(vertId);
		}
	} else {
		Vector3d pts[] = {
			triLinInterp(blockPts.data(), divs, index + Index3D(0, 0, 0)),
			triLinInterp(blockPts.data(), divs, index + Index3D(1, 0, 0)),
			triLinInterp(blockPts.data(), divs, index + Index3D(1, 1, 0)),
			triLinInterp(blockPts.data(), divs, index + Index3D(0, 1, 0)),

			triLinInterp(blockPts.data(), divs, index + Index3D(0, 0, 1)),
			triLinInterp(blockPts.data(), divs, index + Index3D(1, 0, 1)),
			triLinInterp(blockPts.data(), divs, index + Index3D(1, 1, 1)),
			triLinInterp(blockPts.data(), divs, index + Index3D(0, 1, 1)),
		};

		for (const auto& pt : pts) {
			auto vertId = addVertex(pt);
			result.push_back(vertId);
		}
	}
	return result;
}


Vector3d Block::triLinInterp(const Vector3d* pts, size_t divs, const Index3D& index) const
{
	Vector3d t(
		index[0] / (double) divs,
		index[1] / (double) divs,
		index[2] / (double) divs
	);

	Vector3d result;
	result = TRI_LERP(pts, t[0], t[1], t[2]);

	return result;
}

void Block::makeRefPolygonIfRequired(const Index3DId& id)
{
#if DEBUG_BREAKS && defined(_DEBUG)
	if (Index3DId(0, 8, 4, 2) == id) {
		int dbgBreak = 1;
	}
#endif

	auto pOwner = getOwner(id);
	if (pOwner != this) {
		pOwner->makeRefPolygonIfRequired(id);
		return;
	}

	assert(id.blockIdx() == _blockIdx);
	if (_refData._polygons.exists(id))
		return;
	assert(_modelData._polygons.exists(id));
	const auto& temp = _modelData._polygons[id];
	assert(temp.getVertexIds().size() == 4);
	_refData._polygons.findOrAdd(temp, id);
	assert(_refData._polygons.exists(id));
	auto& refFace = _refData._polygons[id];
	refFace._cellIds.clear();
}

void Block::makeRefPolyhedronIfRequired(const Index3DId& id)
{
	auto pOwner = getOwner(id);
	if (pOwner != this) {
		pOwner->makeRefPolyhedronIfRequired(id);
		return;
	}
	assert(id.blockIdx() == _blockIdx);
	if (polyhedronExists(TS_REFERENCE, id))
		return;
	assert(polyhedronExists(TS_REAL, id));
	const auto& temp = _modelData._polyhedra[id];
	assert(temp.getFaceIds().size() == 6);
	_refData._polyhedra.findOrAdd(temp, id);
	assert(polyhedronExists(TS_REFERENCE, id));
}

void Block::setVertexLockType(const Index3DId& vertId, VertexLockType val)
{
	getOwner(vertId)->_vertices[vertId].setLockType(val);
}

VertexLockType Block::getVertexLockType(const Index3DId& vertId) const
{
	return getOwner(vertId)->_vertices[vertId].getLockType();
}

CBoundingBox3Dd Block::getBBox() const
{
	return _corners.getBBox();
}

Index3DId Block::addFace(const MTC::vector<Index3DId>& vertIndices)
{
#if 0 && defined(_DEBUG)
	if (!Polygon::verifyVertsConvexStat(this, vertIndices)) {
		return Index3DId();
}
#endif // _DEBUG

	Polygon newFace(vertIndices);

#if 0 && defined(_DEBUG)
	const auto& edges = newFace.getEdges();
	for (const auto& edge : edges) {
		assert(edge.onPrincipalAxis(this));
	}
#endif // _DEBUG

	Index3DId faceId = addFace(newFace);
	auto& face = _modelData._polygons[faceId];

	return faceId;
}

Index3DId Block::addFace(const MTC::vector<Vector3d>& pts)
{
	MTC::vector<Index3DId> vertIds;
	for (const auto& pt : pts) {
		auto vertId = addVertex(pt);
		vertIds.push_back(vertId);
	}
	if (!Polygon::verifyUniqueStat(vertIds))
		return Index3DId();
	return addFace(vertIds);
}

Index3DId Block::addFace(const Index3D& subBlockIdx, const MTC::vector<Index3DId>& verts)
{
	Index3D ownerBlockIdx = determineOwnerBlockIdx(verts);
	assert(ownerBlockIdx.isValid());
	auto* pOwner = getOwner(ownerBlockIdx);
	assert(pOwner);
	auto faceId = pOwner->addFace(verts);

	return faceId;
}

Index3DId Block::addCell(const Polyhedron& cell)
{
	Index3DId cellId = _modelData._polyhedra.findOrAdd(cell);
	auto& newCell = _modelData._polyhedra[cellId];
	const auto& cellFaceIds = newCell.getFaceIds();

	for (const auto& faceId : cellFaceIds) {
		faceFunc(TS_REAL,faceId, [this, &cellId](Polygon& cellFace) {
			cellFace.addCellId(cellId, 0);
		});
	}
	newCell.orientFaces();
	assert(newCell.isOriented());
	assert(newCell.isConvex());
	assert(newCell.calVolume() > 0);

	return cellId;
}

Index3DId Block::addCell(const MTC::set<Index3DId>& faceIds)
{
	return addCell(Polyhedron(faceIds));
}

Index3DId Block::addCell(const MTC::vector<Index3DId>& faceIds)
{
	MTC::set<Index3DId> faceSet;
	faceSet.insert(faceIds.begin(), faceIds.end());
	Polyhedron hex(faceSet);
	Index3DId cellId = _modelData._polyhedra.findOrAdd(hex);

	return cellId;
}

Index3DId Block::addHexCell(const std::vector<Vector3d>& cellPts)
{
	assert(cellPts.size() == 8);
	MTC::vector<Index3DId> vertIds;
	vertIds.resize(8);
	for (size_t i = 0; i < cellPts.size(); i++) {
		vertIds[i] = getVertexIdOfPoint(cellPts[i]);
	}

	Index3DId subBlockIdx;
	MTC::vector<Index3DId> faceIds;
	faceIds.reserve(6);

	// add left and right
	faceIds.push_back(addFace(subBlockIdx, { vertIds[0], vertIds[4], vertIds[7], vertIds[3] }));
	faceIds.push_back(addFace(subBlockIdx, { vertIds[1], vertIds[2], vertIds[6], vertIds[5] }));

	// add front and back
	faceIds.push_back(addFace(subBlockIdx, { vertIds[0], vertIds[1], vertIds[5], vertIds[4] }));
	faceIds.push_back(addFace(subBlockIdx, { vertIds[2], vertIds[3], vertIds[7], vertIds[6] }));

	// add bottom and top
	faceIds.push_back(addFace(subBlockIdx, { vertIds[0], vertIds[3], vertIds[2], vertIds[1] }));
	faceIds.push_back(addFace(subBlockIdx, { vertIds[4], vertIds[5], vertIds[6], vertIds[7] }));

	const Index3DId polyhedronId = addCell(Polyhedron(faceIds));
	cellFunc(TS_REAL, polyhedronId, [this](Polyhedron& cell) {
		assert(cell.isClosed());
		cell.setTriIndices(_triIndices);
		cell.setEdgeIndices(_edgeIndices);
		});

	return polyhedronId; // SubBlocks are never shared across blocks, so we can drop the block index
}

Index3DId Block::addHexCell(const std::vector<Vector3d>& blockPts, size_t blockDim, const Index3D& subBlockIdx, bool intersectingOnly)
{
	auto vertIds = getSubBlockCornerVertIds(blockPts, blockDim, subBlockIdx);

	if (intersectingOnly) {
		CBoundingBox3Dd bbox;
		for (size_t i = 0; i < 8; i++) {
			bbox.merge(getVertexPoint(vertIds[i]));
		}

		vector<size_t> triIndices;
		bool found = processTris(bbox, triIndices) > 0;

		if (!found) {
			auto sharps = _pVol->getSharpVertIndices();
			for (const auto& vertIdx : sharps) {
#if 0
				if (bbox.contains(getModelMesh()->getVert(vertIdx)._pt, Tolerance::sameDistTol())) {
					found = true;
					break;
				}
#endif
			}
		}
		if (!found)
			return Index3DId();
	}

	MTC::vector<Index3DId> faceIds;
	faceIds.reserve(6);

	// add left and right
	faceIds.push_back(addFace(subBlockIdx, { vertIds[0], vertIds[4], vertIds[7], vertIds[3] }));
	faceIds.push_back(addFace(subBlockIdx, { vertIds[1], vertIds[2], vertIds[6], vertIds[5] }));

	// add front and back
	faceIds.push_back(addFace(subBlockIdx, { vertIds[0], vertIds[1], vertIds[5], vertIds[4] }));
	faceIds.push_back(addFace(subBlockIdx, { vertIds[2], vertIds[3], vertIds[7], vertIds[6] }));

	// add bottom and top
	faceIds.push_back(addFace(subBlockIdx, { vertIds[0], vertIds[3], vertIds[2], vertIds[1] }));
	faceIds.push_back(addFace(subBlockIdx, { vertIds[4], vertIds[5], vertIds[6], vertIds[7] }));

	const Index3DId polyhedronId = addCell(Polyhedron(faceIds));
	cellFunc(TS_REAL,polyhedronId, [this](Polyhedron& cell) {
		assert(cell.isClosed());
		cell.setTriIndices(_triIndices);
		cell.setEdgeIndices(_edgeIndices);
	});

	return polyhedronId; // SubBlocks are never shared across blocks, so we can drop the block index
}

const Block* Block::getOwner(const Index3D& blockIdx) const
{
#if 0 && RUN_MULTI_THREAD && defined(_DEBUG)
	// Test that block indices are within 1 index of the tread index
	Index3D threadIdx = getThreadBlockIdx();

	int bounds = 1;
	assert(inBounds(threadIdx, blockIdx, bounds));
	assert(inBounds(threadIdx, _blockIdx, bounds));

#endif // _DEBUG

	return _pVol->getBlockPtr(blockIdx);
}

Block* Block::getOwner(const Index3D& blockIdx)
{
#if 0 && RUN_MULTI_THREAD && defined(_DEBUG)
	// Test that block indices are within 1 index of the tread index
	Index3D threadIdx = getThreadBlockIdx();

	assert(inBounds(threadIdx, blockIdx, 1));
	assert(inBounds(threadIdx, _blockIdx, 1));

#endif // _DEBUG

	return _pVol->getBlockPtr(blockIdx);
}

Index3DId Block::idOfPoint(const Vector3d& pt) const
{
	// NOTE: Be careful to keep the difference between the _pVertTree indices and the _vertices indices clear. Failure causes NPEs
	auto ownerBlockIdx = determineOwnerBlockIdx(pt);
	auto* pOwner = getOwner(ownerBlockIdx);
	Index3DId result;
	pOwner->_pVertTree->find(pt, result);

	return result;
}

Index3DId Block::addVertex(const Vector3d& pt, const Index3DId& currentId)
{
	Index3DId result = idOfPoint(pt);
	if (result.isValid())
		return result;

	// NOTE: Be careful to keep the difference between the _pVertTree indices and the _vertices indices clear. Failure causes NPEs
	auto ownerBlockIdx = determineOwnerBlockIdx(pt);
	auto* pOwner = getOwner(ownerBlockIdx);

	Vertex vert(pt);
	vert.setPoolOwner(this);
	result = pOwner->_vertices.findOrAdd(vert, currentId);

	// Bounding box is off due to skewing
	pOwner->_pVertTree->add(pt, result);

#ifdef _DEBUG
	assert(idOfPoint(pt) == result);
#endif // _DEBUG

	return result;
}

Index3DId Block::addFace(const Polygon& face)
{
	face.initVertices(getVolume());
	auto ownerBlockIdx = determineOwnerBlockIdx(face);
	auto* pOwner = getOwner(ownerBlockIdx);
	Index3DId result = pOwner->_modelData._polygons.findOrAdd(face);
	auto& newFace = pOwner->_modelData._polygons[result];

#if DEBUG_BREAKS && defined(_DEBUG)
	if (Index3DId(5, 1, 3, 10) == result) {
		int dbgBreak = 1;
	}
#endif

	return result;
}

void Block::addFaceToLookup(const Index3DId& faceId)
{
	auto pOwner = getOwner(faceId);
	pOwner->_modelData._polygons.addToLookup(faceId);
}

bool Block::removeFaceFromLookUp(const Index3DId& faceId)
{
	auto* pOwner = getOwner(faceId);
	return pOwner->_modelData._polygons.removeFromLookup(faceId);
}

Vector3d Block::getVertexPoint(const Index3DId& vertId) const
{
	auto pOwner = getOwner(vertId);
	return pOwner->_vertices[vertId].getPoint();
}

bool Block::write(ostream& out) const
{
	uint8_t version = 0;
	out.write((char*)&version, sizeof(version));

	_blockIdx.write(out);
	_boundBox.write(out);
	_innerBoundBox.write(out);

	out.write((char*)&_blockDim, sizeof(_blockDim));
	vector<Vector3<double>> t = _corners;
	IoUtil::writeVector3(out, t);
	out.write((char*)&_baseIdxVerts, sizeof(_baseIdxVerts));
	out.write((char*)&_baseIdxPolygons, sizeof(_baseIdxPolygons));
	out.write((char*)&_baseIdxPolyhedra, sizeof(_baseIdxPolyhedra));

	_vertices.write(out);
	_modelData._polygons.write(out);
	_modelData._polyhedra.write(out);
	_refData._polygons.write(out);
	_refData._polyhedra.write(out);

	IoUtil::write(out, _edgeIndices);
	IoUtil::write(out, _triIndices);
	return true;
}

bool Block::read(istream& in)
{
	uint8_t version = 0;
	in.read((char*)&version, sizeof(version));

	_blockIdx.read(in);
	_boundBox.read(in);
	_innerBoundBox.read(in);

	in.read((char*)&_blockDim, sizeof(_blockDim));

	vector<Vector3<double>> t;
	IoUtil::readVector3(in, t);
	_corners = t;

	in.read((char*)&_baseIdxVerts, sizeof(_baseIdxVerts));
	in.read((char*)&_baseIdxPolygons, sizeof(_baseIdxPolygons));
	in.read((char*)&_baseIdxPolyhedra, sizeof(_baseIdxPolyhedra));

	_vertices.read(in);
	_modelData._polygons.read(in);
	_modelData._polyhedra.read(in);
	_refData._polygons.read(in);
	_refData._polyhedra.read(in);

	IoUtil::read(in, _edgeIndices);
	IoUtil::read(in, _triIndices);

	return true;
}

bool Block::unload(string& filename)
{
	{
		ofstream out(filename, ofstream::binary);
		if (!out.good()) {
			return false;
		}

		if (out.good()) {
			_filename = filename;
		} else {
			return false;
		}
	}

#if 0
	for (auto subBlockIdx : _subBlocks) {
		_subBlocks.unload(subBlockIdx);
	}
	_subBlocks.clear();
#endif

	return true;
}

bool Block::load()
{
	if (_filename.empty())
		return false;
	ifstream in(_filename, ofstream::binary);
	if (!in.good()) return false;

	size_t size;
	in.read((char*) & size, sizeof(size));
	if (!in.good()) return false;

	_filename.clear();

	return true;
}

bool Block::doPresplits(const BuildCFDParams& params)
{
	bool result = false;
	_modelData._polyhedra.iterateInOrder([this, &params, &result](const Index3DId& id, Polyhedron& cell) {
		if (cell.needToDivideDueToSplitFaces(params)) {
			result = true;
			MTC::set<Index3DId> blockers;
			if (cell.canSplit(blockers)) {
				Vector3d pt = cell.calCentroid();
				PolyhedronSplitter splitter(this, id);
				vector<Index3DId> newCellIds;
				splitter.splitAtPoint(pt);
			} else {
				cell.setNeedsDivideAtCentroid();
			}
		}
	});
	return result;
}

std::string Block::getObjFilePath() const
{
	string path;
	if (filesystem::exists("D:/DarkSky/Projects")) {
		path = "D:/DarkSky/Projects/output/objs/";
	}
	else {
		path = "D:/Users/BobT/Documents/Projects/Code/output/objs/";
	}

	return path;
}

void Block::dumpPolyhedraObj(const MTC::vector<Index3DId>& cellIds, bool includeModel, bool useEdges, bool sharpOnly, const MTC::vector<Vector3d>& pts) const
{
	string path = getObjFilePath();
	for (const auto& cellId : cellIds) {
		stringstream ss;
		ss << path << "cell_" << getLoggerNumericCode() << "_" << cellId.elementId() << ".obj";
		_pVol->writeObj(ss.str(), { cellId }, includeModel, useEdges, sharpOnly, pts);
	}
}

void Block::dumpPolygonObj(std::string& fileName, const MTC::set<Index3DId>& faceIds, const Index3DId& cellId) const
{
	MTC::vector<Index3DId> vecIds;
	vecIds.insert(vecIds.end(), faceIds.begin(), faceIds.end());
	dumpPolygonObj(fileName, vecIds, cellId);
}

void Block::dumpPolygonObj(std::string& fileName, const MTC::vector<Index3DId>& faceIds, const Index3DId& cellId) const
{
	string path = getObjFilePath();
	string filename = path + fileName + ".obj";

	vector<Vector3d> points;
	map<Index3DId, size_t> vertIdToPtMap;
	for (const auto& faceId : faceIds) {
		faceAvailFunc(TS_REAL, faceId, [this, &points, &vertIdToPtMap](const Polygon& face) {
			const MTC::vector<Index3DId>& vertIds = face.getVertexIds();
			for (const auto& vertId : vertIds) {
				auto iter = vertIdToPtMap.find(vertId);
				if (iter == vertIdToPtMap.end()) {
					size_t idx = points.size();
					Vector3d pt = getVertexPoint(vertId);
					points.push_back(pt);
					vertIdToPtMap.insert(make_pair(vertId, idx));
				}
			}
		});
	}

	ofstream out(filename);
	for (const auto& pt : points) {
		out << "v " << pt[0] << " " << pt[1] << " " << pt[2] << "\n";
	}

	for (const auto& faceId : faceIds) {
		faceAvailFunc(TS_REAL, faceId, [this, &out, &vertIdToPtMap, &cellId](const Polygon& face) {
			out << "# " << face.getId() << "\n";
			out << "f";
			MTC::vector<Index3DId> vertIds = face.getOrientedVertexIds(cellId);
			for (const auto& vertId : vertIds) {
				size_t idx = vertIdToPtMap[vertId];
				out << " " << (idx + 1);
			}
			out << "\n";
		});
	}
}

void Block::dumpEdgeObj(std::string& fileName, const MTC::set<Edge>& edges) const
{
	string path = getObjFilePath();
	string filename = path + fileName + ".obj";

	MTC::vector<Vector3d> points;
	MTC::map<Index3DId, size_t> vertIdToPtMap;
	for (const auto& edge : edges) {
		for (int i = 0; i < 2; i++) {
			const auto& vId = edge.getVertexIds()[i];
			auto iter = vertIdToPtMap.find(vId);
			if (iter == vertIdToPtMap.end()) {
				size_t idx = points.size();
				Vector3d pt = getVertexPoint(vId);
				points.push_back(pt);
				vertIdToPtMap.insert(make_pair(vId, idx));
			}			
		}
	}
	ofstream out(filename);
	for (const auto& pt : points) {
		out << "v " << pt[0] << " " << pt[1] << " " << pt[2] << "\n";
	}
	for (const auto& edge : edges) {
		auto vertIds = edge.getVertexIds();
		size_t idx0 = vertIdToPtMap[vertIds[0]] + 1;
		size_t idx1 = vertIdToPtMap[vertIds[1]] + 1;
		out << "l " << idx0 << " " << idx1 << "\n";
	}
}

void Block::dumpOpenCells() const
{
#if DUMP_OPEN_CELL_OBJS
	_modelData._polyhedra.iterateInOrder([this](const Index3DId& id, const Polyhedron& cell) {
		if (!cell.isClosed()) {
			string path;
			if (filesystem::exists("D:/DarkSky/Projects")) {
				path = "D:/DarkSky/Projects/output/objs/";
			}
			else {
				path = "D:/Users/BobT/Documents/Projects/Code/output/objs/";
			}
			stringstream ss;
			ss << path << "cell_" << getLoggerNumericCode() << "_" << cell.getId().elementId() << ".obj";
			_pVol->writeObj(ss.str(), { cell.getId() }, false, false, false);
		}
	});
#endif
}

bool Block::splitRequiredPolyhedra()
{
	bool didSplit = false;
	if (getNeedToSplit().empty())
		return didSplit;

	auto tmp = getNeedToSplit();
	getNeedToSplit().clear();
	for (const auto& cellId : tmp) {
		if (polyhedronExists(TS_REAL, cellId)) {
			PolyhedronSplitter splitter(this, cellId);
			if (splitter.splitIfNeeded())
				didSplit = true;
			else
				assert(!"splitFailed");
			assert(!polyhedronExists(TS_REAL, cellId));
		}
	}

	return didSplit;
}

void Block::imprintTJointVertices()
{
	_modelData._polyhedra.iterateInOrder([this](const Index3DId& cellId, Polyhedron& cell) {
		if (!cell.isClosed()) {

			if (Index3DId(3, 1, 0, 0) == cellId) {
				int dbgBreak = 1;
			}
			makeRefPolyhedronIfRequired(cellId);
			cell.imprintTVertices(this);
			if (!cell.isClosed()) {
				dumpPolyhedraObj({ cellId }, false, false, false);
				assert(cell.isClosed());
			}
		}
	});
}

bool Block::includeFaceInDrawKey(FaceDrawType meshType, const std::vector<Planed>& planes, const Polygon& face) const
{
	bool result = false;
	if (meshType == FT_ALL)
		return true;

	bool intersectsModel = false;
#if 0
	result = face.intersectsModel();
#else
	auto ids = face.getCellIds();
	for (const auto& cellId : ids) {
		if (polyhedronExists(TS_REAL, cellId)) {
			cellFunc(TS_REAL,cellId, [&intersectsModel](const Polyhedron& cell) {
				intersectsModel = cell.intersectsModel();
			});
			if (intersectsModel)
				break;
		}
	}
#endif

	bool isWall = face.isWall();
	for (const auto& pl : planes) {
		if (face.isCoplanar(pl)) {
			isWall = false;
			break;
		}
	}

	bool isBlockBoundary = face.isBlockBoundary();
	bool isModelBoundary = isWall;
	if (isModelBoundary) {
		for (const auto& vertId : face.getVertexIds()) {
			vertexFunc(vertId, [&isModelBoundary](const Vertex& vert) {
				if (vert.getLockType() != VLT_MODEL_MESH) {
					isModelBoundary = false;
				}
				});

			if (!isModelBoundary)
				break;
		}
	}

	result = false;
	switch (meshType) {
		default:
		case FT_ALL:
			result = true;
			break;
		case FT_MODEL_BOUNDARY:
			result = isModelBoundary && !isBlockBoundary;
			break;
		case FT_WALL:
			result = isWall;
			break;
		case FT_BLOCK_BOUNDARY:
			result = !isWall && isBlockBoundary;
			break;

		case FT_BOTTOM:
			result = face.isOnPlane(planes[CFT_BOTTOM]);
			if (result) {
				int dbgBreak = 1;
			}
			break;

		case FT_TOP:
			result = face.isOnPlane(planes[CFT_TOP]);
			break;

		case FT_LEFT:
			result = face.isOnPlane(planes[CFT_LEFT]);
			break;

		case FT_RIGHT:
			result = face.isOnPlane(planes[CFT_RIGHT]);
			break;
		case FT_BACK:
			result = face.isOnPlane(planes[CFT_BACK]);
			break;
		
		case FT_FRONT: 
			result = face.isOnPlane(planes[CFT_FRONT]);
			break;
		
		
	}

	return result;
}

void Block::getBlockTriMesh(FaceDrawType meshType, const std::vector<Planed>& planes, CMeshPtr& pMesh)
{
	if (numFaces(true) == 0)
		return;

	const auto& polys = _modelData._polygons;
	polys.iterateInOrder([this, &pMesh, planes, meshType](const Index3DId& id, const Polygon& face) {
		if (includeFaceInDrawKey(meshType, planes, face)) {
			const auto& vertIds = face.getVertexIds();
			vector<Vector3d> pts;
			pts.reserve(vertIds.size());
			for (const auto& vertId : vertIds) {
				pts.push_back(getVertexPoint(vertId));
			}

			if (pts.size() > 4) {
				Vector3d ctr = face.calCentroid();
				for (size_t idx0 = 0; idx0 < pts.size(); idx0++) {
					size_t idx1 = (idx0 + 1) % pts.size();
					pMesh->addTriangle(ctr, pts[idx0], pts[idx1]);
				}
			} else {
				for (size_t i = 1; i < pts.size() - 1; i++) {
					size_t idx0 = 0;
					size_t idx1 = i;
					size_t idx2 = i + 1;
					pMesh->addTriangle(pts[idx0], pts[idx1], pts[idx2]);
				}
			}
		}
	});
}

void Block::addEdgeToGLPoints(glPointsPtr& points, size_t idx0, size_t idx1)
{
	auto& vals = *points;

	Vector3d pt0 = _corners[idx0];
	vals.push_back((float)pt0[0]);
	vals.push_back((float)pt0[1]);
	vals.push_back((float)pt0[2]);

	Vector3d pt1 = _corners[idx1];
	vals.push_back((float)pt1[0]);
	vals.push_back((float)pt1[1]);
	vals.push_back((float)pt1[2]);
}

void Block::makeEdgeSets(FaceDrawType meshType, const std::vector<Planed>& planes, glPointsPtr& points)
{
	set<Edge> edges;

	const auto& polys = _modelData._polygons;
	polys.iterateInOrder([this, &edges, meshType, planes](const Index3DId& id, const Polygon& face) {
		if (includeFaceInDrawKey(meshType, planes, face)) {
			const auto& vertIds = face.getVertexIds();
			for (size_t i = 0; i < vertIds.size(); i++) {
				size_t j = (i + 1) % vertIds.size();
				Edge edge(vertIds[i], vertIds[j]);
				edges.insert(edge);
			}
		}
	});

	if (!edges.empty()) {
		if (!points)
			points = make_shared< GlPoints>();
		auto& vals = *points;
		for (const auto& edge : edges) {
			const auto* vertIds = edge.getVertexIds();

			Vector3d pt0 = getVertexPoint(vertIds[0]);
			vals.push_back((float)pt0[0]);
			vals.push_back((float)pt0[1]);
			vals.push_back((float)pt0[2]);

			Vector3d pt1 = getVertexPoint(vertIds[1]);
			vals.push_back((float)pt1[0]);
			vals.push_back((float)pt1[1]);
			vals.push_back((float)pt1[2]);
		}
	}
}

bool Block::vertexExists(const Index3DId& id) const
{
	auto pOwner = getOwner(id);
	return pOwner && pOwner->_vertices.exists(id);
}

bool Block::polygonExists(TopolgyState refState, const Index3DId& id) const
{
	auto pOwner = getOwner(id);
	return pOwner && pOwner->data(refState)._polygons.exists(id);
}

bool Block::isPolygonReference(const Polygon* pFace) const
{
	if (!pFace)
		return false;
	const auto& faceId = pFace->getId();
	auto pOwner = getOwner(faceId);
	const auto& data = pOwner->_refData._polygons;
	return data.exists(faceId) && (pFace == &data[faceId]);
}

bool Block::isPolyhedronReference(const Polyhedron* pCell) const
{
	if (!pCell)
		return false;
	const auto& cellId = pCell->getId();
	auto pOwner = getOwner(cellId);
	const auto& data = pOwner->_refData._polyhedra;
	return data.exists(cellId) && (pCell == &data[cellId]);
}

bool Block::allCellsClosed() const
{
	bool result = true;
	_modelData._polyhedra.iterateInOrder([this, &result](const Index3DId& id, const Polyhedron& cell) {
		if (!cell.isClosed())
			result = false;
	});

	return result;
}

bool Block::polyhedronExists(TopolgyState refState, const Index3DId& id) const
{
	auto pOwner = getOwner(id);
	return pOwner && pOwner->data(refState)._polyhedra.exists(id);
}

DFHM::Polygon& Block::getPolygon(TopolgyState refState, const Index3DId& id)
{
	auto pOwner = getOwner(id);
	return pOwner->data(refState)._polygons[id];
}

Polyhedron& Block::getPolyhedron(TopolgyState refState, const Index3DId& id)
{
	auto pOwner = getOwner(id);
	return pOwner->data(refState)._polyhedra[id];
}

void Block::addToSplitStack0(const Index3DId& cellId)
{
	set<Index3DId> blockingIds;

	assert(polyhedronExists(TS_REAL, cellId));
	assert(cellId.blockIdx() == _blockIdx);

	MTC::set<Index3DId> temp;
	auto& cell = _modelData._polyhedra[cellId];
	if (cell.canSplit(temp)) {
		getNeedToSplit().insert(cellId);
		getCantSplitYet().erase(cellId);
	}
	else {
		getCantSplitYet().insert(cellId);
		blockingIds.insert(temp.begin(), temp.end());
	}
	

	while (!blockingIds.empty()) {
		set<Index3DId> blockingIds2;
		for (const auto& cellId : blockingIds) {
			if (!polyhedronExists(TS_REAL, cellId))
				continue;
			auto pOwner = getOwner(cellId);
			MTC::set<Index3DId> temp;
			auto& cell = pOwner->_modelData._polyhedra[cellId];
			if (cell.canSplit(temp)) {
				pOwner->getNeedToSplit().insert(cellId);
				pOwner->getCantSplitYet().erase(cellId);
			}
			else {
				pOwner->getCantSplitYet().insert(cellId);
				blockingIds2.insert(temp.begin(), temp.end());
			}
		}
		blockingIds = blockingIds2;
	}
}

void Block::updateSplitStack()
{
	if (getCantSplitYet().empty())
		return;

	MTC::set<Index3DId> blockingIds;

	auto tmpCantSplit = getCantSplitYet();

	getCantSplitYet().clear();
	for (const auto& cellId : tmpCantSplit) {
		assert(cellId.blockIdx() == _blockIdx);
		MTC::set<Index3DId> temp;
		auto pOwner = getOwner(cellId);
		if (pOwner->_modelData._polyhedra.exists(cellId)) {
			auto& cell = _modelData._polyhedra[cellId];
			if (cell.canSplit(temp)) {
				pOwner->getNeedToSplit().insert(cellId);
			} else {
				pOwner->getCantSplitYet().insert(cellId);
				blockingIds.insert(temp.begin(), temp.end());
			}
		}
	}

	while (!blockingIds.empty()) {
		MTC::set<Index3DId> blockingIds2;
		for (const auto& cellId : blockingIds) {
			auto pOwner = getOwner(cellId);
			if (pOwner->_modelData._polyhedra.exists(cellId)) {
				MTC::set<Index3DId> temp;
				auto& cell = pOwner->_modelData._polyhedra[cellId];
				if (cell.canSplit(temp)) {
					pOwner->getNeedToSplit().insert(cellId);
					pOwner->getCantSplitYet().erase(cellId);
				} else {
					pOwner->getCantSplitYet().insert(cellId);
					blockingIds2.insert(temp.begin(), temp.end());
				}
			}
		}
		blockingIds = blockingIds2;
	}

}

bool Block::hasPendingSplits() const
{
	return !getCantSplitYet().empty() || !getNeedToSplit().empty();
}

void Block::freePolygon(const Index3DId& id, bool requireRefExists)
{
#if CAN_FREE_TESTS_ENABLED
	assert(!isPolygonInUse(id));
#endif

	auto pOwner = getOwner(id);
	if (pOwner) {
		auto& polygons = pOwner->_modelData._polygons;
		auto& refPolygons = pOwner->_refData._polygons;
		if (requireRefExists)
			assert(refPolygons.exists(id));
		polygons.free(id);
	}
}

void Block::freePolyhedron(const Index3DId& id, bool requireRefExists)
{
#if CAN_FREE_TESTS_ENABLED
	assert(!isPolyhedronInUse(id));
#endif

	auto pOwner = getOwner(id);
	if (pOwner) {
		auto& polyhedra = pOwner->_modelData._polyhedra;
		auto& refPolyhedra = pOwner->_refData._polyhedra;
		if (requireRefExists)
			assert(refPolyhedra.exists(id));
		polyhedra.free(id);
	}
}

#ifdef _DEBUG
bool Block::isPolygonInUse(const Index3DId& faceId) const
{
	bool result = false;

	_modelData._polyhedra.iterateInOrder([&result, &faceId](const Index3DId& cellId, const Polyhedron& cell) {
		if (cell.containsFace(faceId)) {
			result = true;
		}
	});

	return result;
	}

bool Block::isPolyhedronInUse(const Index3DId& cellId) const
{
	bool result = false;

	_modelData._polygons.iterateInOrder([&result, &cellId](const Index3DId& faceId, const Polygon& face) {
		if (face.usedByCell(cellId)) {
			result = true;
		}
	});

	return result;
}
#endif // _DEBUG

size_t Block::processEdges(const TriMesh::CMesh::BoundingBox& bbox, vector<size_t>& edgeIndices) const
{
#if 0
	auto pMesh = getModelMesh();
	pMesh->processFoundEdges(_edgeIndices, bbox, edgeIndices);

#if VERIFY_REDUCED_FINDER
	vector<size_t> edgeIndices1;
	pMesh->findEdges(bbox, edgeIndices1);
	assert(edgeIndices.size() == edgeIndices1.size());
#endif
#endif
	return edgeIndices.size();
}

size_t Block::processTris(const TriMesh::CMesh::BoundingBox& bbox, vector<size_t>& triIndices) const
{
#if 0
	auto pMesh = getModelMesh();
	pMesh->processFoundTris(_triIndices, bbox, triIndices);
#if VERIFY_REDUCED_FINDER
	vector<size_t> triIndices1;
	pMesh->findTris(bbox, triIndices1);
	assert(triIndices.size() == triIndices1.size());
#endif
#endif

	return triIndices.size();
}

void Block::pack()
{
#if 0
	for (auto id : _subBlocks) {
		if (id != -1)
			return;
	}

	_subBlocks.clear();
#endif
}

Block::ModelData::ModelData(Block* pBlk)
	: _polygons(pBlk, true)
	, _polyhedra(pBlk, false)
{
}

Block::ModelData::ModelData(Block* pBlk, const ModelData& src)
	: _polygons(pBlk, src._polygons)
	, _polyhedra(pBlk, src._polyhedra)
{
}

//LAMBDA_BLOCK_IMPLS

void Block::vertexFunc(const Index3DId& id, const function<void(const Vertex& obj)>& func) const {
	auto p = getOwner(id); 
	func(p->_vertices[id]);
} 

void Block::vertexFunc(const Index3DId& id, const function<void(Vertex& obj)>& func) {
	auto p = getOwner(id); 
	func(p->_vertices[id]);
} 

void Block::faceFunc(TopolgyState state, const Index3DId& id, const function<void(const Polygon& obj)>& func) const {
	auto p = getOwner(id); 
	if (state == TS_REAL) 
		func(p->_modelData._polygons[id]); 
	else 
		func(p->_refData._polygons[id]);
} 

void Block::faceFunc(TopolgyState state, const Index3DId& id, const function<void(Polygon& obj)>& func) {
	auto p = getOwner(id); 
	if (state == TS_REAL) 
		func(p->_modelData._polygons[id]); 
	else 
		func(p->_refData._polygons[id]);
} 

void Block::cellFunc(TopolgyState state, const Index3DId& id, const function<void(const Polyhedron& obj)>& func) const {
	auto p = getOwner(id); 
	if (state == TS_REAL) 
		func(p->_modelData._polyhedra[id]); 
	else 
		func(p->_refData._polyhedra[id]);
} 

void Block::cellFunc(TopolgyState state, const Index3DId& id, const function<void(Polyhedron& obj)>& func) {
	auto p = getOwner(id); 
	if (state == TS_REAL) 
		func(p->_modelData._polyhedra[id]); 
	else 
		func(p->_refData._polyhedra[id]);
} 

void Block::faceAvailFunc(TopolgyState prefState, const Index3DId& id, const function<void(const Polygon& obj)>& func) const {
	const auto p = getOwner(id); 
	if (prefState == TS_REAL) {
		if (p->_modelData._polygons.exists(id)) {
			func(p->_modelData._polygons[id]);
		} else {
			func(p->_refData._polygons[id]);
		}
	} else {
		if (p->_refData._polygons.exists(id)) {
			func(p->_refData._polygons[id]);
		} else {
			func(p->_modelData._polygons[id]);
		}
	}
} 

void Block::cellAvailFunc(TopolgyState prefState, const Index3DId& id, const function<void(const Polyhedron& obj)>& func) const {
	const auto p = getOwner(id); 
	if (prefState == TS_REAL) {
		if (p->_modelData._polyhedra.exists(id)) {
			func(p->_modelData._polyhedra[id]);
		} else {
			func(p->_refData._polyhedra[id]);
		}
	} else {
		if (p->_refData._polyhedra.exists(id)) {
			func(p->_refData._polyhedra[id]);
		} else {
			func(p->_modelData._polyhedra[id]);
		}
	}
}
