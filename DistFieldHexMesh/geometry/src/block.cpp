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
#include <string>
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
#include <splitter3D.h>
#include <block.h>
#include <volume.h>
#include <logger.h>
#include <gradingOp.h>
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
	, _vertices(this, true, 8*8)
	, _polygons(this, true, 8*6)
	, _polyhedra(this, false, 8)
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
		
#if USE_MULTI_THREAD_CONTAINERS			
	MultiCore::scoped_set_local_heap st(&_heap);
#endif
}

Block::~Block()
{
}

void Block::clear()
{
#if USE_MULTI_THREAD_CONTAINERS			
	MultiCore::scoped_set_local_heap st(&_heap);
#endif

	_needToSplit.clear();

	_vertices.clear();
	_polygons.clear();
	_polyhedra.clear();
}

const SplittingParams& Block::getSplitParams() const
{
	return _pVol->getAppData()->getParams();
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
		assert(!"calSubBlockIndexFromLinear failed.");
		std::string msg = std::string(__FILE__) + ":" + std::to_string(__LINE__) + std::string(" calSubBlockIndexFromLinear failed.");
		throw runtime_error(msg);
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

const std::vector<MeshDataPtr>& Block::getModelMeshData() const
{
	auto& pAppData = _pVol->getAppData();
	return pAppData->getMeshData();
}

void Block::remapBlockIndices(const std::vector<size_t>& idRemap, const Index3D& srcDims)
{
	size_t srcIdx = Volume::calLinearBlockIndex(_blockIdx, srcDims);
	Index3D dstIdx;
	size_t remapedIdx = idRemap[srcIdx];
	if (remapedIdx != -1) {
		dstIdx = _pVol->calBlockIndexFromLinearIndex(remapedIdx);
		_blockIdx = dstIdx;
	}


	_vertices.iterateInOrder([&idRemap, &srcDims](const Index3DId& id, Vertex& vert) {
		vert.remapId(idRemap, srcDims);
	});

	_polygons.iterateInOrder([&idRemap, &srcDims](const Index3DId& faceId, Polygon& face) {
		face.remapId(idRemap, srcDims);
	});

	_polyhedra.iterateInOrder([&idRemap, &srcDims](const Index3DId& cellId, Polyhedron& cell) {
		cell.remapId(idRemap, srcDims);
	});
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
	_polygons.iterateInOrder([&result, includeInner](const Index3DId& id, const Polygon& face) {
		if (includeInner || face.isBlockBoundary())
			result++;
	});
	return result;
}

size_t Block::numPolyhedra() const
{
	return _polyhedra.size();
}

size_t Block::numBytes() const
{
	size_t result = 0;
	// TODO, need a mutex or semaphore because another thread is writing these
	result += _vertices.numBytes();
	result += _polygons.numBytes();
	result += _polyhedra.numBytes();
	return result;
}

bool Block::verifyTopology() const
{
	bool result = true;
	_vertices.iterateInOrder([&result](const Index3DId& id, const Vertex& vert) {
		if (!vert.verifyTopology())
			result = false;
	});

	_polygons.iterateInOrder([&result](const Index3DId& id, const Polygon& face) {
		if (!face.verifyTopology())
			result = false;
	});

	MTC::vector<Index3DId> badCellIds;
	_polyhedra.iterateInOrder([&result, &badCellIds](const Index3DId& id, const Polyhedron& cell) {
		if (!cell.verifyTopology()) {
			badCellIds.push_back(id);
			result = false;
		}
	});

	if (!badCellIds.empty()) {
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
#if 0
	bool result = true;
	_vertices.iterateInOrder([&idx, &result](const Index3DId& id, const Vertex& vert) {
		if (!vert.verifyIndices(idx))
			result = false;
	});

	_polygons.iterateInOrder([&idx, &result](const Index3DId& faceId, const Polygon& face) {
		if (!face.verifyIndices(idx))
			result = false;
	});

	_polyhedra.iterateInOrder([&idx, &result](const Index3DId& faceId, const Polyhedron& cell) {
		if (!cell.verifyIndices(idx))
			result = false;
	});
#endif
	return true;
}

void Block::createBlockCells()
{
	Index3D idx;
	for (idx[0] = 0; idx[0] < _blockDim; idx[0]++) {
		for (idx[1] = 0; idx[1] < _blockDim; idx[1]++) {
			for (idx[2] = 0; idx[2] < _blockDim; idx[2]++) {
				createGradedHexCell(_corners, _blockDim, idx);
			}
		}
	}
}

void Block::createSubBlocksForHexSubBlock(const Vector3d* blockPts, const Index3D& subBlockIdx)
{

	auto blockCornerPts = getCornerPts();	
	auto polyId = createGradedHexCell(blockCornerPts, _blockDim, subBlockIdx);
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

void Block::addPolygonToLookup(const DFHM::Polygon& face)
{
	Index3D ownerBlockIdx = determineOwnerBlockIdx(face);
	assert(ownerBlockIdx.isValid());
	auto* pOwner = getOwner(ownerBlockIdx);
	assert(pOwner);
	assert(pOwner->_polygons.exists(face.getId()));
	pOwner->_polygons.addToLookup(face);
#ifdef _DEBUG
	// The copy in storage must ALWAYS be the most complex copy with imprinted vertices.
	// Replacing a simple face, with a complex face assures edge and vertex fusing
	// If a complex face is replaced with a simpler one, it breaks fusing
	const Polygon* pFace = pOwner->_polygons.get(face);
	if (pFace->getVertexIds().size() >= face.getVertexIds().size())
	{
		assert(!"Trying to replace a complex face with a simpler one is not allowed. Delete the old one first, then add the new one if this is requred.");
	}
#endif // _DEBUG

}

void Block::removePolygonFromLookup(const DFHM::Polygon& face)
{
	Index3D ownerBlockIdx = determineOwnerBlockIdx(face);
	assert(ownerBlockIdx.isValid());
	auto* pOwner = getOwner(ownerBlockIdx);
	assert(pOwner);
	auto faceId = pOwner->_polygons.findId(face);
	if (faceId.isValid()) {
		assert(faceId == face.getId());
		pOwner->_polygons.removeFromLookup(faceId);
	}
}

Index3DId Block::findPolygon(const Polygon& face) const
{
	Index3D ownerBlockIdx = determineOwnerBlockIdx(face);
	assert(ownerBlockIdx.isValid());
	auto* pOwner = getOwner(ownerBlockIdx);
	assert(pOwner);
	auto faceId = pOwner->_polygons.findId(face);

	return faceId;
}

Index3DId Block::addCell(const Polyhedron& cell, const Index3DId& parentCellId)
{
#ifdef _DEBUG
	for (const auto& faceId : cell.getFaceIds()) {
		assert(getBlockIdx().withinRange(faceId));
	}
#endif // _DEBUG

	Index3DId cellId = _polyhedra.findOrAdd(cell);
	auto& newCell = _polyhedra[cellId];
#if USE_CELL_HISTOGRAM
	newCell.setParentId(parentCellId);
#endif
	const auto& cellFaceIds = newCell.getFaceIds();

	for (const auto& faceId : cellFaceIds) {
		faceFunc(faceId, [this, &cellId](Polygon& cellFace) {
			cellFace.addCellId(cellId);
		});
	}
//	newCell.orientFaces();
	if (newCell._triIndices.empty()) {
		if (!parentCellId.isValid() || !polyhedronExists(parentCellId)) {
			const auto& meshData = getModelMeshData();
			newCell.addMeshToTriIndices(meshData);
		} else {
			const auto& parentCell = getPolyhedron(parentCellId);
			newCell.setTriIndices(parentCell);
		}
	}

#if VALIDATION_ON && defined(_DEBUG)
	auto& ctr = newCell.calCentroid();
	assert(newCell.pointInside(ctr));
	assert(newCell.isOriented());
//	assert(newCell.isConvex());
	assert(newCell.calVolume() > 0);
#endif

	return cellId;
}

Index3DId Block::addHexCell(const MTC::vector<Index3DId>& cornerVertIds)
{
	MTC::vector<MTC::vector<Index3DId>> blockFaceIds;
	GradingOp::getCubeFaceVertIds(cornerVertIds, blockFaceIds);
	MTC::vector<Index3DId> faceIds;
	faceIds.reserve(blockFaceIds.size());

	for (const auto& vertIds : blockFaceIds) {
		faceIds.push_back(addPolygon(Polygon(vertIds)));
	}

	const Index3DId polyhedronId = addCell(Polyhedron(faceIds, cornerVertIds), Index3DId());

	return polyhedronId; // SubBlocks are never shared across blocks, so we can drop the block index
}

Index3DId Block::createGradedHexCell(const std::vector<Vector3d>& blockPts, size_t blockDim, const Index3D& subBlockIdx)
{
	auto vertIds = getSubBlockCornerVertIds(blockPts, blockDim, subBlockIdx);
	MTC::vector<MTC::vector<Index3DId>> blockVertIds;
	GradingOp::getCubeFaceVertIds(vertIds, blockVertIds);

	MTC::vector<Index3DId> faceIds;
	faceIds.reserve(6);
	for (const auto& faceVertIds : blockVertIds) {
		faceIds.push_back(addPolygon(Polygon(faceVertIds)));
	}

	const Index3DId polyhedronId = addCell(Polyhedron(faceIds, vertIds), Index3DId());

	return polyhedronId; // SubBlocks are never shared across blocks, so we can drop the block index
}

const Block* Block::getOwner(const Index3D& blockIdx) const
{
#if 1 && defined(_DEBUG)
	// Test that block indices are within 1 index of the tread index
	Index3D threadIdx = getThreadBlockIdx();

	if (threadIdx.isValid()) {
		assert(threadIdx.withinRange(blockIdx));
		assert(threadIdx.withinRange(_blockIdx));
	} else {
		assert(_blockIdx.withinRange(blockIdx));
	}
#endif // _DEBUG


	return blockIdx.isValid() ? _pVol->getBlockPtr(blockIdx) : nullptr;
}

Block* Block::getOwner(const Index3D& blockIdx)
{
#if 1 && defined(_DEBUG)
	// Test that block indices are within 1 index of the thread index
	Index3D threadIdx = getThreadBlockIdx();

	if (threadIdx.isValid()) {
		assert(threadIdx.withinRange(blockIdx));
		assert(threadIdx.withinRange(_blockIdx));
	} else {
		assert(_blockIdx.withinRange(blockIdx));
	}
#endif // _DEBUG

	return blockIdx.isValid() ? _pVol->getBlockPtr(blockIdx) : nullptr;
}

const Block* Block::getOwner(const Edge& edge) const
{
	return getOwner(edge[0]);
}

Block* Block::getOwner(const Edge& edge)
{
	return getOwner(edge[0]);
}

Index3DId Block::addVertex(const Vector3d& pt, const Index3DId& currentId)
{
	Index3DId result = getVertexIdOfPoint(pt);
	if (result.isValid())
		return result;

	// NOTE: Be careful to keep the difference between the _pVertTree indices and the _vertices indices clear. Failure causes NPEs
	auto ownerBlockIdx = determineOwnerBlockIdx(pt);
	auto* pOwner = getOwner(ownerBlockIdx);
	assert(pOwner);

	result = pOwner->_vertices.findOrAdd(Vertex(pt), currentId);

	assert(getVertexIdOfPoint(pt) == result);

	return result;
}

Index3DId Block::addPolygon(const Polygon& face)
{
	assert(Polygon::verifyVertsConvexStat(this, face.getVertexIds()));
	auto ownerBlockIdx = determineOwnerBlockIdx(face);
	auto* pOwner = getOwner(ownerBlockIdx);
	Index3DId result = pOwner->_polygons.findOrAdd(face);
	auto& newFace = pOwner->_polygons[result];

#if DEBUG_BREAKS && defined(_DEBUG)
	if (Index3DId(5, 1, 3, 10) == result) {
		int dbgBreak = 1;
	}
#endif

	return result;
}

const Vector3d& Block::getVertexPoint(const Index3DId& vertId) const
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

	_vertices.write(out);
	_polygons.write(out);
	_polyhedra.write(out);

	return true;
}

void Block::setSupportsReverseLookup(bool val)
{
	_polygons.setSupportsReverseLookup(val);
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

	_vertices.read(in);
	_polygons.read(in);
	_polyhedra.read(in);

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
		ss << path << "cell_" << getLoggerNumericCode(cellId) << ".obj";
		_pVol->writeObj(ss.str(), { cellId }, includeModel, useEdges, sharpOnly, pts);
	}
}

void Block::dumpPolygonObj(const std::string& filename, const MTC::set<Index3DId>& faceIds, const Index3DId& cellId) const
{
	MTC::vector<Index3DId> vecIds;
	vecIds.insert(vecIds.end(), faceIds.begin(), faceIds.end());
	dumpPolygonObj(filename, vecIds, cellId);
}

void Block::dumpPolygonObj(const std::string& filename, const MTC::vector<Index3DId>& faceIds, const Index3DId& cellId) const
{
	string path = getObjFilePath();
	string fullFilename = path + filename + ".obj";

	vector<Vector3d> points;
	map<Index3DId, size_t> vertIdToPtMap;
	for (const auto& faceId : faceIds) {
		faceFunc(faceId, [this, &points, &vertIdToPtMap](const Polygon& face) {
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

	ofstream out(fullFilename);
	for (const auto& pt : points) {
		out << "v " << pt[0] << " " << pt[1] << " " << pt[2] << "\n";
	}

	for (const auto& faceId : faceIds) {
		faceFunc(faceId, [this, &out, &vertIdToPtMap, &cellId](const Polygon& face) {
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

void Block::dumpEdgeObj(std::string& fileName, const MTC::set<EdgeKey>& edges) const
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
	_polyhedra.iterateInOrder([this](const Index3DId& id, const Polyhedron& cell) {
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

bool Block::splitRequiredPolyhedra(size_t splittingPass)
{
	bool didSplit = false;
	if (_needToSplit.empty())
		return didSplit;

	auto needToSplitCopy = _needToSplit;
	_needToSplit.clear();

	for (const auto& cellId : needToSplitCopy) {
		if (polyhedronExists(cellId)) {
			Splitter3D splitter(this, cellId, splittingPass);
			if (splitter.splitAtCenter()) {
				didSplit = true;
				assert(!polyhedronExists(cellId));
			}
		}
	}

	return didSplit;
}

bool Block::includeFaceInDrawKey(FaceDrawType meshType, const std::vector<Planed>& planes, const Polygon& face) const
{
	bool result = false;

	bool isWall = face.isWall();
	bool isInner = face.numCells() == 2;
	if (isInner) {
		int dbgBreak = 1;
	}
	for (const auto& pl : planes) {
		if (face.isCoplanar(pl)) {
			isWall = false;
			break;
		}
	}

	int64_t layerNum = face.getLayerNum();

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
		case FT_INNER:
			result = isInner;
			break;
		case FT_ERROR_WALL:
			result = isWall;
			break;

		case FT_BOTTOM:
			result = face.isCoplanar(planes[CFT_BOTTOM]);
			break;

		case FT_TOP:
			result = face.isCoplanar(planes[CFT_TOP]);
			break;

		case FT_LEFT:
			result = face.isCoplanar(planes[CFT_LEFT]);
			break;

		case FT_RIGHT:
			result = face.isCoplanar(planes[CFT_RIGHT]);
			break;

		case FT_BACK:
			result = face.isCoplanar(planes[CFT_BACK]);
			break;
		case FT_FRONT: 
			result = face.isCoplanar(planes[CFT_FRONT]);
			break;
		
		case FT_MESH_LAYER_0:
			if (layerNum == 0)
				result = true;
			break;
		case FT_MESH_LAYER_1:
			if (layerNum == 1)
				result = true;
			break;
		case FT_MESH_LAYER_2:
			if (layerNum == 2)
				result = true;
			break;
		case FT_MESH_LAYER_3:
			if (layerNum == 3)
				result = true;
			break;
		case FT_MESH_LAYER_4:
			if (layerNum == 4)
				result = true;
			break;
		case FT_MESH_LAYER_5:
			if (layerNum == 5)
				result = true;
			break;
		case FT_MESH_LAYER_6:
			if (layerNum == 6)
				result = true;
			break;
		case FT_MESH_LAYER_7:
			if (layerNum == 7)
				result = true;
			break;
		case FT_MESH_LAYER_8:
			if (layerNum == 8)
				result = true;
			break;
		case FT_MESH_LAYER_9:
			if (layerNum == 9)
				result = true;
			break;

	}

	return result;
}

void Block::GlHexFaces::addTriangle(const Vector3d& pt0, const Vector3d& pt1, const Vector3d& pt2)
{
	Vector3d v0 = pt0 - pt1;
	Vector3d v1 = pt2 - pt1;
	Vector3d n = v0.cross(v1).normalized();

	for (int i = 0; i < 3; i++) {
		_glTriPoints.push_back(pt0[i]);
		_glTriNormals.push_back(n[i]);
	}

	for (int i = 0; i < 3; i++) {
		_glTriPoints.push_back(pt1[i]);
		_glTriNormals.push_back(n[i]);
	}

	for (int i = 0; i < 3; i++) {
		_glTriPoints.push_back(pt2[i]);
		_glTriNormals.push_back(n[i]);
	}

}

void Block::GlHexFaces::addFace(const Block& blk, const Polygon& face)
{
	auto triFunc = [this](const Vector3d& pt0, const Vector3d& pt1, const Vector3d& pt2) {
		Vector3d v0 = pt0 - pt1;
		Vector3d v1 = pt2 - pt1;
		Vector3d n = v0.cross(v1).normalized();

		for (int i = 0; i < 3; i++) {
			_glTriPoints.push_back(pt0[i]);
			_glTriNormals.push_back(n[i]);
		}

		for (int i = 0; i < 3; i++) {
			_glTriPoints.push_back(pt1[i]);
			_glTriNormals.push_back(n[i]);
		}

		for (int i = 0; i < 3; i++) {
			_glTriPoints.push_back(pt2[i]);
			_glTriNormals.push_back(n[i]);
		}
	};

	auto edgeFunc = [this](const Vector3d& pt0, const Vector3d& pt1) {
		for (int i = 0; i < 3; i++)
			_glEdgePoints.push_back(pt0[i]);

		for (int i = 0; i < 3; i++)
			_glEdgePoints.push_back(pt1[i]);
	};

	face.getTriPoints(triFunc, edgeFunc);

}

size_t Block::GlHexFaces::numTriVertices() const
{
	return _glTriPoints.size() / 3;
}

size_t Block::GlHexFaces::numEdgeVertices() const
{
	return _glEdgePoints.size() / 2;
}

void Block::createHexTriMesh(FaceDrawType meshType, const std::vector<Planed>& planes, GlHexFacesPtr& glPolys)
{
	if (numFaces(true) == 0)
		return;

	if (!glPolys)
		glPolys = make_shared<GlHexFaces>();
	const auto& polys = _polygons;
	polys.iterateInOrder([this, &glPolys, planes, meshType](const Index3DId& id, const Polygon& face) {
		if (includeFaceInDrawKey(meshType, planes, face)) {
			glPolys->addFace(*this, face);
		}
	});
}

bool Block::vertexExists(const Index3DId& id) const
{
	auto pOwner = getOwner(id);
	return pOwner && pOwner->_vertices.exists(id);
}

bool Block::polygonExists(const Index3DId& id) const
{
	auto pOwner = getOwner(id);
	return pOwner && pOwner->_polygons.exists(id);
}

bool Block::allCellsClosed() const
{
	bool result = true;
	_polyhedra.iterateInOrder([this, &result](const Index3DId& id, const Polyhedron& cell) {
		if (!cell.isClosed())
			result = false;
	});

	return result;
}

bool Block::polyhedronExists(const Index3DId& id) const
{
	auto pOwner = getOwner(id);
	return pOwner && pOwner->_polyhedra.exists(id);
}

DFHM::Polygon& Block::getPolygon(const Index3DId& id)
{
	auto pOwner = getOwner(id);
	return pOwner->_polygons[id];
}

Polyhedron& Block::getPolyhedron(const Index3DId& id)
{
	auto pOwner = getOwner(id);
	return pOwner->_polyhedra[id];
}

void Block::addToSplitStack(const Index3DId& cellId)
{
	auto pOwner = getOwner(cellId);
	if (pOwner && pOwner->_polyhedra.exists(cellId))
		pOwner->_needToSplit.insert(cellId);
}

void Block::updateSplitStack()
{
	MTC::set<Index3DId> blockingIds;
	const auto& params = getSplitParams();
	for (const auto& cellId : _touchedCellIds) {
		auto pOwner = getOwner(cellId);
		if (pOwner && pOwner->_polyhedra.exists(cellId)) {
			auto& cell = pOwner->_polyhedra[cellId];
			if (cell.isTooComplex(params))
				pOwner->_needToSplit.insert(cellId);
		}
	}
	_touchedCellIds.clear();
}

bool Block::hasPendingSplits() const
{
	return !_touchedCellIds.empty() || !_needToSplit.empty();
}

void Block::resetLayerNums()
{
	_polyhedra.iterateInOrder([](const Index3DId& cellId, Polyhedron& cell) {
		cell.clearLayerNum();
	});
}

void Block::markIncrementLayerNums(int i)
{
	_polyhedra.iterateInOrder([this, i](const Index3DId& cellId, Polyhedron& cell) {
		if (cell.getLayerNum() == i) {
			const auto& adjIds = cell.getAdjacentCells();
			for (const auto& adjId : adjIds) {
				cellFunc(adjId, [i](Polyhedron& adjCell) {
					adjCell.setLayerNumOnNextPass(adjCell.getLayerNum());
				});
			}
		}
	});
}

void Block::setIncrementLayerNums(int i)
{
	_polyhedra.iterateInOrder([this, i](const Index3DId& cellId, Polyhedron& cell) {
		cell.setLayerNum(i + 1, false);
	});
}

void Block::freePolygon(const Index3DId& id)
{
#if CAN_FREE_TESTS_ENABLED
	assert(!isPolygonInUse(id));
#endif

	auto pOwner = getOwner(id);
	if (pOwner) {
		auto& polygons = pOwner->_polygons;
		polygons.free(id);
	}
}

void Block::freePolyhedron(const Index3DId& id)
{
#if CAN_FREE_TESTS_ENABLED
	assert(!isPolyhedronInUse(id));
#endif

	auto pOwner = getOwner(id);
	if (pOwner) {
		auto& polyhedra = pOwner->_polyhedra;
		polyhedra.free(id); // This frees all unused polygons
	}
}

#ifdef _DEBUG
bool Block::isPolygonInUse(const Index3DId& faceId) const
{
	bool result = false;

	_polyhedra.iterateInOrder([&result, &faceId](const Index3DId& cellId, const Polyhedron& cell) {
		if (cell.containsFace(faceId)) {
			result = true;
		}
	});

	return result;
	}

bool Block::isPolyhedronInUse(const Index3DId& cellId) const
{
	bool result = false;

	_polygons.iterateInOrder([&result, &cellId](const Index3DId& faceId, const Polygon& face) {
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

//LAMBDA_BLOCK_IMPLS
void Block::vertexFunc(const Index3DId& id, const function<void(const Vertex& obj)>& func) const {
	auto p = getOwner(id); 
	if (p->_vertices.exists(id)) 
		func(p->_vertices[id]);
} 

void Block::vertexFunc(const Index3DId& id, const function<void(Vertex& obj)>& func) {
	auto p = getOwner(id); 
	if (p->_vertices.exists(id)) 
		func(p->_vertices[id]);
} 

void Block::faceFunc(const Index3DId& id, const function<void(const Polygon& obj)>& func) const {
	auto p = getOwner(id); 
	if (p->_polygons.exists(id)) 
		func(p->_polygons[id]);
} 

void Block::faceFunc(const Index3DId& id, const function<void(Polygon& obj)>& func) {
	auto p = getOwner(id); 
	if (p->_polygons.exists(id)) 
		func(p->_polygons[id]);
} 

void Block::cellFunc(const Index3DId& id, const function<void(const Polyhedron& obj)>& func) const {
	auto p = getOwner(id); 
	if (p->_polyhedra.exists(id)) 
		func(p->_polyhedra[id]);
} 

void Block::cellFunc(const Index3DId& id, const function<void(Polyhedron& obj)>& func) {
	auto p = getOwner(id); 
	if (p->_polyhedra.exists(id)) 
		func(p->_polyhedra[id]);
} 

void Block::edgeFunc(const EdgeKey& key, const function<void(const Edge& obj)>& func) const {
	auto& idx = getBlockIdx(); 
	const Block* p; 
	if (idx.withinRange(key[0])) 
		p = getOwner(key[0]); 
	else if (idx.withinRange(key[1])) 
		p = getOwner(key[1]); 
	if (p) {
		Edge edge(key, p); func(edge);
	}
} 

void Block::edgeFunc(const EdgeKey& key, const function<void(Edge& obj)>& func) {
	auto& idx = getBlockIdx();  
	Block* p; 
	if (idx.withinRange(key[0])) 
		p = getOwner(key[0]); 
	else if (idx.withinRange(key[1])) 
		p = getOwner(key[1]); 
	if (p) {
		Edge edge(key, p); 
		func(edge);
	}
}
