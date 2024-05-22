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
#include <Index3D.h>
#include <splitParams.h>
#include <vertex.h>
#include <edge.h>
#include <polygon.h>
#include <polyhedron.h>
#include <polyhedronSplitter.h>
#include <block.h>
#include <volume.h>
#include <logger.h>

using namespace std;
using namespace TriMesh;
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

Block::Block(Volume* pVol, const Index3D& blockIdx, const vector<Vector3d>& pts)
	: _blockIdx(blockIdx)
	, _pVol(pVol)
	, _modelData(this)
	, _vertices(this, true)
	, _refData(this)
{
	_blockDim = Index3D::getBlockDim();
	assert(pts.size() == 8);
	_corners.resize(8);
	for (size_t i = 0; i < pts.size(); i++) {
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

	// This is close to working, but the full search is finding solutions the partial search is not
	TriMesh::CMeshConstPtr pMesh = getModelMesh();
	pMesh->findEdges(_boundBox, _edgeIndices);
	pMesh->findTris(_boundBox, _triIndices);
}

Block::Block(const Block& src)
	: _blockIdx(src._blockIdx)
	, _pVol(src._pVol)
	, _boundBox(src._boundBox)
	, _innerBoundBox(src._innerBoundBox)
	, _blockDim(src._blockDim)
	, _corners(src._corners)
	, _filename(src._filename)
	, _vertices(this, src._vertices)
	, _modelData(this, src._modelData)
	, _refData(this, src._refData)
{
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

const CMeshPtr& Block::getModelMesh() const
{
	return _pVol->getModelMesh();
}


void Block::getAdjacentBlockIndices(set<Index3D>& indices) const
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
	if (_innerBoundBox.contains(point))
		return _blockIdx;

	auto ratios = invTriLinIterp(point);
	return determineOwnerBlockIdxFromRatios(ratios);
}

Index3D Block::determineOwnerBlockIdx(const Vertex& vert) const
{
	return determineOwnerBlockIdx(vert.getPoint());
}

Index3D Block::determineOwnerBlockIdx(const vector<Vector3d>& points) const
{
	auto volBounds = _pVol->volDim();

	Vector3d ctr(0, 0, 0);
	for (const auto& pt : points) {
		ctr += pt;
	}
	ctr = ctr / points.size();

	return determineOwnerBlockIdx(ctr);
}

Index3D Block::determineOwnerBlockIdx(const vector<Index3DId>& verts) const
{
	vector<Vector3d> pts;
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

size_t Block::pointOpenFoamLabel(const Index3DId& id) const
{
	auto pBlk = getOwner(id);
	return pBlk->_baseIdxVerts + _vertices.getRawIndex(id);
}

size_t Block::faceOpenFoamLabel(const Index3DId& id) const
{
	auto pBlk = getOwner(id);
	return pBlk->_baseIdxPolygons + _modelData._polygons.getRawIndex(id);
}

size_t Block::cellOpenFoamLabel(const Index3DId& id) const
{
	auto pBlk = getOwner(id);
	return pBlk->_baseIdxPolyhedra + _modelData._polyhedra.getRawIndex(id);
}

Index3DId Block::maxCellId(const Index3DId& id0, const Index3DId& id1) const
{
	return (cellOpenFoamLabel(id0) > cellOpenFoamLabel(id1)) ? id0 : id1;
}

size_t Block::numFaces(bool includeInner) const
{
	size_t result = 0;
	_modelData._polygons.iterateInOrder([&result, includeInner](const Index3DId& id, const Polygon& face) {
		if (includeInner || face.isOuter())
			result++;
	});
	return result;
}

size_t Block::numPolyhedra() const
{
	return _modelData._polyhedra.size();
}

bool Block::verifyTopology() const
{
	bool result = true;
	vector<Index3DId> badCellIds;
	_modelData._polyhedra.iterateInOrder([&result, &badCellIds](const Index3DId& id, const Polyhedron& cell) {
		if (!cell.verifyTopology()) {
			badCellIds.push_back(id);
			result = false;
		}
	});
	if (!result) {
		dumpObj(badCellIds, false, false, false);
	}

	return result;
}

void Block::createSubBlocks(TopolgyState refState)
{
	Index3D idx;
	for (idx[0] = 0; idx[0] < _blockDim; idx[0]++) {
		for (idx[1] = 0; idx[1] < _blockDim; idx[1]++) {
			for (idx[2] = 0; idx[2] < _blockDim; idx[2]++) {
				addHexCell(_corners.data(), _blockDim, idx, false);
			}
		}
	}
}

void Block::createSubBlocksForHexSubBlock(const Vector3d* blockPts, const Index3D& subBlockIdx)
{

	auto blockCornerPts = getCornerPts();	
	auto polyId = addHexCell(blockCornerPts.data(), _blockDim, subBlockIdx, false);
}

void Block::addIndexToMap(const Index3D& subBlockIdx, set<Index3D>& subBlockIndices)
{
	subBlockIndices.insert(subBlockIdx);
}

const vector<Vector3d>& Block::getCornerPts() const
{
	return _corners;
}

vector<Vector3d> Block::getSubBlockCornerPts(const Vector3d* blockPts, size_t divs, const Index3D& index) const
{
	vector<Vector3d> result = {
		triLinInterp(blockPts, divs, index + Index3D(0, 0, 0)),
		triLinInterp(blockPts, divs, index + Index3D(1, 0, 0)),
		triLinInterp(blockPts, divs, index + Index3D(1, 1, 0)),
		triLinInterp(blockPts, divs, index + Index3D(0, 1, 0)),

		triLinInterp(blockPts, divs, index + Index3D(0, 0, 1)),
		triLinInterp(blockPts, divs, index + Index3D(1, 0, 1)),
		triLinInterp(blockPts, divs, index + Index3D(1, 1, 1)),
		triLinInterp(blockPts, divs, index + Index3D(0, 1, 1)),
	};

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

Vector3d Block::invTriLinIterp(const Vector3d& pt) const
{
	return invTriLinIterp(_corners.data(), pt);
}

Vector3d Block::invTriLinIterp(const Vector3d* blockPts, const Vector3d& pt) const
{
	const double tol = Tolerance::looseParamTol();
	const double tolSqr = tol * tol;

	// Linearly interpolate based on the bounding box. This is precise for a paralelapiped set of blockPts
	Vector3d vRange = _boundBox.range();
	Vector3d delta = pt - _boundBox.getMin();
	Vector3d result;
	for (size_t i = 0; i < 3; i++) {
		result[i] = delta[i] / vRange[i];
		assert(-tol <= result[i] && result[i] <= 1 + tol);
		if (result[i] < 0)
			result[i] = 0;
		else if (result[i] > 1)
			result[i] = 1;
	}

	Vector3d checkPt = TRI_LERP(blockPts, result[0], result[1], result[2]);

	const double step = 1.0e-12;
	Vector3d vErr = checkPt - pt;
	double errSqr = vErr.squaredNorm();
	while (errSqr > tolSqr) {
		// Compute gradient
		Vector3d grad;
		for (size_t axis = 0; axis < 3; axis++) {
			switch (axis) {
			default:
			case 0:
				checkPt = TRI_LERP(blockPts, result[0] + step, result[1], result[2]);
				break;
			case 1:
				checkPt = TRI_LERP(blockPts, result[0], result[1] + step, result[2]);
				break;
			case 2:
				checkPt = TRI_LERP(blockPts, result[0], result[1], result[2] + step);
				break;
			}
			grad[axis] = ((checkPt[axis]) - pt[axis]) / step;
		}
		grad.normalize();

		Vector3d ratio0 = - step * grad;
		Vector3d testPt0 = TRI_LERP(blockPts, ratio0[0], ratio0[1], ratio0[2] + step);

		Vector3d ratio1 = step * grad;
		Vector3d testPt1 = TRI_LERP(blockPts, ratio1[0], ratio1[1], ratio1[2] + step);

		double y0 = (testPt0 - pt).norm();
		double y1 = (result - pt).norm();
		double y2 = (testPt1 - pt).norm();
		y0 = y0 - y1;
		y1 = y2 - y1;
		double a = (y0 + y1) / (2 * step * step);
		double b = (y1 - y0) / (2 * step);
		if (fabs(a) < tol)
			break;
		double dx = -b / (2 * a);
		result = result + dx * grad;
		checkPt = TRI_LERP(blockPts, result[0], result[1], result[2]);
		vErr = checkPt - pt;
		errSqr = vErr.squaredNorm();
	}
		
	for (int i = 0; i < 3; i++) {
		assert(0 <= result[i] && result[i] <= 1);
	}
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

Index3DId Block::addFace(const vector<Index3DId>& vertIndices)
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

	return faceId;
}
Index3DId Block::addFace(const vector<Vector3d>& pts)
{
	vector<Index3DId> vertIds;
	for (const auto& pt : pts) {
		auto vertId = addVertex(pt);
		vertIds.push_back(vertId);
	}
	if (!Polygon::verifyUniqueStat(vertIds))
		return Index3DId();
	return addFace(vertIds);
}

Index3DId Block::addFace(int axis, const Index3D& subBlockIdx, const vector<Vector3d>& pts)
{
	Index3D ownerBlockIdx = determineOwnerBlockIdx(pts);
	assert(ownerBlockIdx.isValid());
	auto* pOwner = getOwner(ownerBlockIdx);
	assert(pOwner);
	auto faceId = pOwner->addFace(pts);

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
	return cellId;
}

Index3DId Block::addCell(const set<Index3DId>& faceIds)
{
	return addCell(Polyhedron(faceIds));
}

Index3DId Block::addCell(const vector<Index3DId>& faceIds)
{
	set<Index3DId> faceSet;
	faceSet.insert(faceIds.begin(), faceIds.end());
	Index3DId cellId = _modelData._polyhedra.findOrAdd(Polyhedron(faceSet));

	return cellId;
}

Index3DId Block::addHexCell(const Vector3d* blockPts, size_t blockDim, const Index3D& subBlockIdx, bool intersectingOnly)
{
	auto pts = getSubBlockCornerPts(blockPts, blockDim, subBlockIdx);

	CBoundingBox3Dd bbox;
	for (size_t i = 0; i < 8; i++) {
		bbox.merge(pts[i]);
	}

	if (intersectingOnly) {
		vector<size_t> triIndices;
		bool found = processTris(bbox, triIndices) > 0;

		if (!found) {
			auto sharps = _pVol->getSharpVertIndices();
			for (const auto& vertIdx : sharps) {
				if (bbox.contains(getModelMesh()->getVert(vertIdx)._pt)) {
					found = true;
					break;
				}
			}
		}
		if (!found)
			return Index3DId();
	}

	vector<Index3DId> faceIds;
	faceIds.reserve(6);

	// add left and right
	faceIds.push_back(addFace(0, subBlockIdx, { pts[0], pts[4], pts[7], pts[3] }));
	faceIds.push_back(addFace(0, subBlockIdx, { pts[1], pts[2], pts[6], pts[5] }));

	// add front and back
	faceIds.push_back(addFace(1, subBlockIdx, { pts[0], pts[1], pts[5], pts[4] }));
	faceIds.push_back(addFace(1, subBlockIdx, { pts[2], pts[3], pts[7], pts[6] }));

	// add bottom and top
	faceIds.push_back(addFace(2, subBlockIdx, { pts[0], pts[3], pts[2], pts[1] }));
	faceIds.push_back(addFace(2, subBlockIdx, { pts[4], pts[5], pts[6], pts[7] }));

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
	auto ownerBlockIdx = determineOwnerBlockIdx(pt);
	auto* pOwner = getOwner(ownerBlockIdx);
	return pOwner->_vertices.findId(pt);
}

Index3DId Block::addVertex(const Vector3d& pt, const Index3DId& currentId)
{
	auto ownerBlockIdx = determineOwnerBlockIdx(pt);
	auto* pOwner = getOwner(ownerBlockIdx);
	Vertex vert(pt);
	return pOwner->_vertices.findOrAdd(vert, currentId);
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

FixedPt Block::getFixedPoint(const Index3DId& vertId) const
{
	auto pOwner = getOwner(vertId);
	return pOwner->_vertices[vertId].getFixedPt();
}

bool Block::write(ostream& out) const
{
	uint8_t version = 0;
	out.write((char*)&version, sizeof(version));

	_blockIdx.write(out);
	_boundBox.write(out);
	_innerBoundBox.write(out);

	out.write((char*)&_blockDim, sizeof(_blockDim));
	IoUtil::writeVector3(out, _corners);
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

	IoUtil::readVector3(in, _corners);

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
		if (cell.needToSplitDueToSplitFaces(params)) {
			result = true;
			set<Index3DId> blockers;
			if (cell.canSplit(blockers)) {
				Vector3d pt = cell.calCentroid();
				PolyhedronSplitter splitter(this, id);
				vector<Index3DId> newCellIds;
				splitter.splitAtPoint(pt);
			} else {
				cell.setNeedsSplitAtCentroid();
			}
		}
	});
	return result;
}

void Block::dumpObj(const vector<Index3DId>& cellIds, bool includeModel, bool useEdges, bool sharpOnly, const vector<Vector3d>& pts) const
{
	string path;
	if (filesystem::exists("D:/DarkSky/Projects")) {
		path = "D:/DarkSky/Projects/output/objs/";
	}
	else {
		path = "D:/Users/BobT/Documents/Projects/Code/output/objs/";
	}
	for (const auto& cellId : cellIds) {
		cellFunc(TS_REAL,cellId, [this, &path, includeModel, useEdges, sharpOnly, &pts](const Polyhedron& cell) {
			stringstream ss;
			ss << path << "cell_" << getLoggerNumericCode() << "_" << cell.getId().elementId() << ".obj";
			_pVol->writeObj(ss.str(), { cell.getId() }, includeModel, useEdges, sharpOnly, pts);
		});
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
#if LOGGING_ENABLED
	auto pLogger = getLogger();
	auto& out = pLogger->getStream();
	out << "Block[" << _blockIdx << "]::splitRequiredPolyhedra\n";
	Logger::Indent indent;
#endif

	bool didSplit = false;
	if (_needToSplit.empty())
		return didSplit;

	auto tmp = _needToSplit;
	_needToSplit.clear();
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
#if LOGGING_ENABLED
	auto pLogger = getLogger();
	auto& out = pLogger->getStream();
	LOG(out << "\n");
	LOG(out << "*******************************************************************************************\n");
	LOG(out << "Block[" << _blockIdx << "]::imprintTJointVertices *****************************************\n");
	LOG(out << "*******************************************************************************************\n");
	Logger::Indent indent;
#endif

	_modelData._polyhedra.iterateInOrder([this](const Index3DId& cellId, Polyhedron& cell) {
		if (!cell.isClosed()) {

			if (Index3DId(3, 1, 0, 0) == cellId) {
				int dbgBreak = 1;
			}
			makeRefPolyhedronIfRequired(cellId);
			cell.imprintTVertices(this);
			if (!cell.isClosed()) {
				dumpObj({ cellId }, false, false, false);
				assert(cell.isClosed());
			}
		}
	});
}

bool Block::includeFaceInRender(FaceType meshType, const Polygon& face) const
{
	bool result = false;
	if (meshType == FT_ALL)
		return true;
#if 1
	result = face.intersectsModel();
#else
	auto ids = face.getCellIds();
	for (const auto& cellId : ids) {
		if (polyhedronExists(TS_REAL, cellId)) {
			cellFunc(TS_REAL,cellId, [&result](const Polyhedron& cell) {
				result = cell.intersectsModel();
				});
		}
		if (result)
			break;
	}
#endif
	if (!result)
		return false;

	bool isOuter = face.isOuter();
	bool isBlockBoundary = face.isBlockBoundary();

	result = false;
	switch (meshType) {
		default:
		case FT_ALL:
			result = true;
			break;
		case FT_INNER:
			result = !isOuter && !isBlockBoundary;
			break;
		case FT_OUTER:
			result = isOuter;
			break;
		case FT_BLOCK_BOUNDARY:
			result = !isOuter && isBlockBoundary;
			break;
	}

	return result;
}

void Block::getBlockTriMesh(FaceType meshType, CMeshPtr& pMesh)
{
	if (numFaces(true) == 0)
		return;

	_modelData._polygons.iterateInOrder([this, &pMesh, meshType](const Index3DId& id, const Polygon& face) {
		if (includeFaceInRender(meshType, face)) {
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

void Block::makeEdgeSets(FaceType meshType, glPointsPtr& points)
{
	set<Edge> edges;

	_modelData._polygons.iterateInOrder([this, &edges, meshType](const Index3DId& id, const Polygon& face) {
		if (includeFaceInRender(meshType, face)) {
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

Polygon& Block::getPolygon(TopolgyState refState, const Index3DId& id)
{
	auto pOwner = getOwner(id);
	return pOwner->data(refState)._polygons[id];
}

Polyhedron& Block::getPolyhedron(TopolgyState refState, const Index3DId& id)
{
	auto pOwner = getOwner(id);
	return pOwner->data(refState)._polyhedra[id];
}

void Block::addToSplitStack(const set<Index3DId>& cellIds)
{
	set<Index3DId> blockingIds;
	for (const auto& cellId : cellIds) {
		assert(polyhedronExists(TS_REAL, cellId));
		assert(cellId.blockIdx() == _blockIdx);

		set<Index3DId> temp;
		auto& cell = _modelData._polyhedra[cellId];
		if (cell.canSplit(temp)) {
			_needToSplit.insert(cellId);
			_cantSplitYet.erase(cellId);
		} else {
			_cantSplitYet.insert(cellId);
			blockingIds.insert(temp.begin(), temp.end());
		}
	}

	while (!blockingIds.empty()) {
		set<Index3DId> blockingIds2;
		for (const auto& cellId : blockingIds) {
			if (!polyhedronExists(TS_REAL, cellId))
				continue;
			auto pOwner = getOwner(cellId);
			set<Index3DId> temp;
			auto& cell = pOwner->_modelData._polyhedra[cellId];
			if (cell.canSplit(temp)) {
				pOwner->_needToSplit.insert(cellId);
				pOwner->_cantSplitYet.erase(cellId);
			}
			else {
				pOwner->_cantSplitYet.insert(cellId);
				blockingIds2.insert(temp.begin(), temp.end());
			}
		}
		blockingIds = blockingIds2;
	}
}

void Block::updateSplitStack()
{
	if (_cantSplitYet.empty())
		return;

	set<Index3DId> blockingIds;

	auto tmpCantSplit = _cantSplitYet;

	_cantSplitYet.clear();
	for (const auto& cellId : tmpCantSplit) {
		assert(cellId.blockIdx() == _blockIdx);
		set<Index3DId> temp;
		auto pOwner = getOwner(cellId);
		if (pOwner->_modelData._polyhedra.exists(cellId)) {
			auto& cell = _modelData._polyhedra[cellId];
			if (cell.canSplit(temp)) {
				pOwner->_needToSplit.insert(cellId);
			} else {
				pOwner->_cantSplitYet.insert(cellId);
				blockingIds.insert(temp.begin(), temp.end());
			}
		}
	}

	while (!blockingIds.empty()) {
		set<Index3DId> blockingIds2;
		for (const auto& cellId : blockingIds) {
			auto pOwner = getOwner(cellId);
			if (pOwner->_modelData._polyhedra.exists(cellId)) {
				set<Index3DId> temp;
				auto& cell = pOwner->_modelData._polyhedra[cellId];
				if (cell.canSplit(temp)) {
					pOwner->_needToSplit.insert(cellId);
					pOwner->_cantSplitYet.erase(cellId);
				} else {
					pOwner->_cantSplitYet.insert(cellId);
					blockingIds2.insert(temp.begin(), temp.end());
				}
			}
		}
		blockingIds = blockingIds2;
	}

}

bool Block::hasPendingSplits() const
{
	return !_cantSplitYet.empty() || !_needToSplit.empty();
}

void Block::freePolygon(const Index3DId& id)
{
#if CAN_FREE_TESTS_ENABLED
	assert(!isPolygonInUse(id));
#endif

	auto pOwner = getOwner(id);
	if (pOwner) {
#if LOGGING_ENABLED
		auto& out = getLogger()->getStream();
#endif
		LOG(out << Logger::Pad() << "Deleting f" << id << "\n");
		auto& polygons = pOwner->_modelData._polygons;
		auto& refPolygons = pOwner->_refData._polygons;
		assert(refPolygons.exists(id));
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
#if LOGGING_ENABLED
		auto& out = getLogger()->getStream();
#endif
		LOG(out << Logger::Pad() << "Deleting c" << id << "\n");
		auto& polyhedra = pOwner->_modelData._polyhedra;
		auto& refPolyhedra = pOwner->_refData._polyhedra;
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
	auto pMesh = getModelMesh();
	pMesh->processFoundEdges(_edgeIndices, bbox, edgeIndices);
#if VERIFY_REDUCED_FINDER
	vector<size_t> edgeIndices1;
	pMesh->findEdges(bbox, edgeIndices1);
	assert(edgeIndices.size() == edgeIndices1.size());
#endif

	return edgeIndices.size();
}

size_t Block::processTris(const TriMesh::CMesh::BoundingBox& bbox, vector<size_t>& triIndices) const
{
	auto pMesh = getModelMesh();
	pMesh->processFoundTris(_triIndices, bbox, triIndices);
#if VERIFY_REDUCED_FINDER
	vector<size_t> triIndices1;
	pMesh->findTris(bbox, triIndices1);
	assert(triIndices.size() == triIndices1.size());
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
		}
		else {
			func(p->_refData._polygons[id]);
		}
	}
	else {
		if (p->_refData._polygons.exists(id)) {
			func(p->_refData._polygons[id]);
		}
		else {
			func(p->_modelData._polygons[id]);
		}
	}
} 

void Block::cellAvailFunc(TopolgyState prefState, const Index3DId& id, const function<void(const Polyhedron& obj)>& func) const {
	const auto p = getOwner(id); 
	if (prefState == TS_REAL) {
		if (p->_modelData._polyhedra.exists(id)) {
			func(p->_modelData._polyhedra[id]);
		}
		else {
			func(p->_refData._polyhedra[id]);
		}
	}
	else {
		if (p->_refData._polyhedra.exists(id)) {
			func(p->_refData._polyhedra[id]);
		}
		else {
			func(p->_modelData._polyhedra[id]);
		}
	}
}
