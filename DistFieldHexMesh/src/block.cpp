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
#define _USE_MATH_DEFINES
#include <cmath>

#include <Index3D.h>
#include <vertex.h>
#include <edge.h>
#include <polygon.h>
#include <polyhedron.h>
#include <block.h>
#include <volume.h>

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
	, _vertices(this, true)
	, _polygons(this, true)
	, _polyhedra(this, false)
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
	, _polygons(this, src._polygons)
	, _polyhedra(this, src._polyhedra)
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


void Block::getAdjacentBlockIndices(std::set<Index3D>& indices) const
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
	return pBlk->_baseIdxPolygons + _polygons.getRawIndex(id);
}

size_t Block::cellOpenFoamLabel(const Index3DId& id) const
{
	auto pBlk = getOwner(id);
	return pBlk->_baseIdxPolyhedra + _polyhedra.getRawIndex(id);
}

Index3DId Block::maxCellId(const Index3DId& id0, const Index3DId& id1) const
{
	return (cellOpenFoamLabel(id0) > cellOpenFoamLabel(id1)) ? id0 : id1;
}

size_t Block::numFaces(bool includeInner) const
{
	size_t result = 0;
	_polygons.iterateInOrder([&result, includeInner](const Polygon& face) {
		if (includeInner || face.isOuter())
			result++;
	});
	return result;
}

size_t Block::numPolyhedra() const
{
	return _polyhedra.size();
}

bool Block::verifyTopology() const
{
	bool result = true;
#ifdef _DEBUG 
	_polyhedra.iterateInOrder([&result](const Polyhedron& cell) {
		if (cell.getId() == Index3DId(Index3D(2, 0, 0), 0)) {
			int dbgBreak = 1;
		}
		// make sure we get block 3,1,1
		bool pass = cell.verifyTopology();
		if (!pass)
			result = false;
	});
#endif

	return result;
}

vector<Index3DId> Block::createSubBlocks()
{
	vector<Index3DId> newCells;
	Index3D idx;
	for (idx[0] = 0; idx[0] < _blockDim; idx[0]++) {
		for (idx[1] = 0; idx[1] < _blockDim; idx[1]++) {
			for (idx[2] = 0; idx[2] < _blockDim; idx[2]++) {
				auto cellId = addHexCell(_corners.data(), _blockDim, idx, true);
				if (cellId.isValid())
					newCells.push_back(cellId);
			}
		}
	}

	return newCells;
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

const vector<Index3DId>& Block::getFaceVertexIds(const Index3DId& faceId) const
{
	auto pOwner = getOwner(faceId);
	return pOwner->_polygons[faceId].getVertexIds();
}

Index3DId Block::findFace(const std::vector<Index3DId>& vertIndices) const
{
	Polygon tempFace(vertIndices);
	auto ownerIdx = determineOwnerBlockIdx(tempFace);
	return getOwner(ownerIdx)->_polygons.findId(tempFace);
}

Index3DId Block::addFace(const vector<Index3DId>& vertIndices)
{
	if (!Polygon::verifyVertsConvexStat(this, vertIndices)) {
		return Index3DId();
	}

	Polygon newFace(vertIndices);
	auto ownerIdx = determineOwnerBlockIdx(newFace);
	set<Edge> edges;
	newFace.getEdges(edges);
	for (const auto& edge : edges) {
		assert(edge.onPrincipalAxis(this));
	}

	auto* pOwner = getOwner(ownerIdx);
	Index3DId faceId = pOwner->_polygons.findOrAdd(newFace);

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
	auto* pOwner = getOwner(ownerBlockIdx);

	auto faceId = pOwner->addFace(pts);

	return faceId;
}

Index3DId Block::addCell(const Polyhedron& cell)
{
	Index3DId cellId = _polyhedra.findOrAdd(cell);
	const auto& cellFaceIds = _polyhedra[cellId].getFaceIds();

	for (const auto& faceId : cellFaceIds) {
		faceFunc(faceId, [this, &cellId](Polygon& cellFace) {
			// Removed broken face to owner cell linkages.
			auto cellIds = cellFace.getCellIds();
			for (const auto& cellId : cellIds) {
				cellFunc(cellId, [&cellIds, &cellFace](Polyhedron& cell) {
					if (!cell.containsFace(cellFace.getId())) {
						cellFace.removeCellId(cell.getId());
					}
				});
			}

			// Add linkage to the new cell
			cellFace.addCellId(cellId);
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
	Index3DId cellId = _polyhedra.findOrAdd(Polyhedron(faceSet));

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
		vector<CMesh::SearchEntry> triIndices;
		bool found = getModelMesh()->findTris(bbox, triIndices) > 0;

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

	const Index3DId polyhedronId = _polyhedra.findOrAdd(Polyhedron(faceIds));
	assert(polyhedronId.isValid());
	for (const auto& faceId : faceIds) {
		faceFunc(faceId, [&polyhedronId](Polygon& face) {
			face.addCellId(polyhedronId);
		});
	}

	return polyhedronId; // SubBlocks are never shared across blocks, so we can drop the block index
}

const Block* Block::getOwner(const Index3D& blockIdx) const
{
	return _pVol->getBlockPtr(blockIdx);
}

Block* Block::getOwner(const Index3D& blockIdx)
{
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
	auto ownerBlockIdx = determineOwnerBlockIdx(face);
	auto* pOwner = getOwner(ownerBlockIdx);
	auto result = pOwner->_polygons.findOrAdd(face);
	return result;
}

void Block::addFaceToLookup(const Index3DId& faceId)
{
	auto pOwner = getOwner(faceId);
	pOwner->_polygons.addToLookup(faceId);
}

bool Block::removeFaceFromLookUp(const Index3DId& faceId)
{
	auto* pOwner = getOwner(faceId);
	return pOwner->_polygons.removeFromLookup(faceId);
}

Vector3d Block::getVertexPoint(const Index3DId& vertId) const
{
	auto pOwner = getOwner(vertId);
	return pOwner->_vertices[vertId].getPoint();
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

void Block::setNeedsSimpleSplit()
{
	_polyhedra.iterateInOrder([](Polyhedron& cell) {
		cell.setNeedToSplitAtCentroid(true);
	});
}

void Block::setNeedsCurvatureSplit(int divsPerRadius, double maxCurvatureRadius, double sinEdgeAngle)
{
	_polyhedra.iterateInOrder([divsPerRadius, maxCurvatureRadius, sinEdgeAngle](Polyhedron& cell) {
		cell.setNeedToSplitCurvature(divsPerRadius, maxCurvatureRadius, sinEdgeAngle);
	});
}

void Block::splitPolygonsNeedingSplit()
{
	_polygons.iterateInOrder([](Polygon& face) {
		face.splitIfRequred();
	});
}

void Block::splitPolyhedraNeedingSplit()
{
	_polyhedra.iterateInOrder([](Polyhedron& cell) {
		cell.splitIfRequred();
	});
}

void Block::imprintPolyhedraVertices()
{
	_polyhedra.iterateInOrder([](Polyhedron& cell) {
		cell.imprintVertices();
	});
}

size_t Block::splitAllCellsAtCentroid()
{
	size_t count = _polyhedra.size();
	_polyhedra.iterateInOrder([this](Polyhedron& cell) {
		set<Index3DId> newCellIds;
		cell.splitAtCentroid(newCellIds);
	});
	count = _polyhedra.size() - count;
	return count;
}

size_t Block::splitAllCellsAtPoint(const Vector3d& pt)
{
	size_t count = _polyhedra.size();
	_polyhedra.iterateInOrder([this, pt](Polyhedron& cell) {
		// TODO, we do need a cell finder at some point
		set<Index3DId> newCellIds;
		if (cell.contains(pt))
			cell.splitAtPoint(pt, newCellIds);
	});
	count = _polyhedra.size() - count;
	return count;
}

size_t Block::splitAllCellsByCurvature(int divsPerRadius, double maxCurvatureRadius, double sinEdgeAngle)
{
	size_t count = _polyhedra.size();
	_polyhedra.iterateInOrder([this, divsPerRadius, maxCurvatureRadius, sinEdgeAngle](Polyhedron& cell) {
		if (cell.isActive())
			cell.splitByCurvature(divsPerRadius, maxCurvatureRadius, sinEdgeAngle);
	});
	_polyhedra.iterateInOrder([](Polyhedron& cell) {
		if (cell.isActive())
			cell.splitIfTooManyFaceSplits();
	});
	count = _polyhedra.size() - count;
	return count;
}

bool Block::includeFace(FaceType meshType, const Polygon& face) const
{
	bool result = false;
	if (!face.isActive())
		return false;

	for (const auto& cellId : face.getCellIds()) {
		cellFunc(cellId, [&result](const Polyhedron& cell) {
			result = cell.intersectsModel() && cell.isActive();
		});
		if (result)
			break;
	}
	if (!result)
		return false;

	bool isOuter = face.isOuter();
	bool isBlockBoundary = face.isBlockBoundary();

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
		case FT_LAYER_BOUNDARY:
//			result = !isOuter && isLevelBoundary;
			break;
		case FT_BLOCK_BOUNDARY:
			result = !isOuter && isBlockBoundary;
			break;
	}

	return result;
}

CMeshPtr Block::getBlockTriMesh(FaceType meshType)
{
	if (numFaces(true) == 0)
		return nullptr;

	CMesh::BoundingBox bbox = _boundBox;
	double span = bbox.range().norm();
	bbox.grow(0.05 * span);

	CMeshPtr result;
	_polygons.iterateInOrder([this, &result, &bbox, meshType](const Polygon& face) {
		if (includeFace(meshType, face)) {
			if (!result) {
				result = make_shared<CMesh>(bbox);
			}
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
					result->addTriangle(ctr, pts[idx0], pts[idx1]);
				}
			} else {
				for (size_t i = 1; i < pts.size() - 1; i++) {
					size_t idx0 = 0;
					size_t idx1 = i;
					size_t idx2 = i + 1;
					result->addTriangle(pts[idx0], pts[idx1], pts[idx2]);
				}
			}
			result->changed();
		}
	});

//	cout << "Skipped " << skipped << "inner faces\n";
	return result;
}

Block::glPointsPtr Block::makeEdgeSets(FaceType meshType)
{
	set<Edge> edges;

	_polygons.iterateInOrder([this, meshType, &edges](const Polygon& face) {
		if (includeFace(meshType, face)) {
			face.getEdges(edges);
		}
	});

	glPointsPtr result;
	if (!edges.empty()) {
		if (!result)
			result = make_shared< GlPoints>();
		auto& vals = *result;
		vals.clear();
		vals.changed();
		for (const auto& edge : edges) {
			const auto* vertIds = edge.getVertexIds();

			Vector3d pt = getVertexPoint(vertIds[0]);
			vals.push_back((float)pt[0]);
			vals.push_back((float)pt[1]);
			vals.push_back((float)pt[2]);

			pt = getVertexPoint(vertIds[1]);
			vals.push_back((float)pt[0]);
			vals.push_back((float)pt[1]);
			vals.push_back((float)pt[2]);
		}
	}
	return result;
}

bool Block::freePolygon(const Index3DId& id)
{
	auto pOwner = getOwner(id);
	return getOwner(id) && pOwner->_polygons.free(id);
}

bool Block::freePolyhedron(const Index3DId& id)
{
	auto pOwner = getOwner(id);
	return getOwner(id) && pOwner->_polyhedra.free(id);
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

bool Block::polyhedronExists(const Index3DId& id) const
{
	auto pOwner = getOwner(id);
	return pOwner && pOwner->_polyhedra.exists(id);
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
