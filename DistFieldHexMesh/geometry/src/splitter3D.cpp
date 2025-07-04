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

	Copyright Robert R Tipton, 2022, all rights reserved except those granted in prior license statement.

	Dark Sky Innovative Solutions http://darkskyinnovation.com/
*/

#include <defines.h>
#include <assert.h>
#include <tm_lineSegment.h>
#include <tm_ray.h>
#include <tm_math.hpp>
#include <tm_spatialSearch.hpp>
#include <triMesh.h>
#include <triMeshPatch.h>
#include <pool_vector.h>
#include <pool_set.h>
#include <utils.h>
#include <splitParams.h>
#include <polygon.h>
#include <polyhedron.h>
#include <splitter3D.h>
#include <splitter2D.h>
#include <block.h>
#include <tolerances.h>
#include <utils.h>
#include <gradingOp.h>

#ifdef _WIN32
#include <windows.h>
#include <profileapi.h>
#endif // _WIN32

using namespace std;
using namespace DFHM;

thread_local VolumePtr Splitter3D::_pScratchVol;

namespace
{
	static atomic<size_t> numSplits2 = 0;
	static atomic<size_t> numSplits4 = 0;
	static atomic<size_t> numSplitsComplex8 = 0;
}

Splitter3D::Splitter3D(Block* pBlock, const Index3DId& polyhedronId, size_t splitLevel)
	: _pBlock(pBlock)
	, _polyhedronId(polyhedronId)
	, _splitLevel(splitLevel)
	, _params(pBlock->getSplitParams())
{
	if (!_pScratchVol)
		_pScratchVol = _pBlock->getVolume()->createScratchVolume();
	_pScratchBlock = _pScratchVol->getBlockPtr(Index3D(0, 0, 0));

#ifdef _DEBUG
	auto& cell = getPolyhedron(_polyhedronId);
	if (!cell.isClosed()) {
		getBlockPtr()->dumpPolyhedraObj({ cell.getId() }, false, false, false);
		assert(!"Cell not closed");
	}
#endif // _DEBUG
}

Splitter3D::~Splitter3D()
{
	if (_pScratchVol)
		_pScratchVol->clearEntries();

#if !RUN_MULTI_THREAD // testing all blocks when multithreaded will defintely have race conditions
	auto pVol = _pBlock->getVolume();
//	assert(pVol->verifyTopology(false));
#endif // !!RUN_MULTI_THREAD

}

void Splitter3D::reset(const MTC::vector<Index3DId>& tempCellids)
{
	for (auto& cellId : tempCellids) {
		getBlockPtr()->removeFromTouchedCellList(cellId);
	}
	_pScratchBlock->clear();
}

void Splitter3D::dumpSplitStats()
{
	cout << "Num splits 2: " << numSplits2 << "\n";
	cout << "Num splits 4: " << numSplits4 << "\n";
	cout << "Num splits complex 8: " << numSplitsComplex8 << "\n";

	numSplits2 = 0;
	numSplits4 = 0;
	numSplitsComplex8 = 0;
}

void Splitter3D::clearThreadLocal()
{
	_pScratchVol = nullptr;
}

bool Splitter3D::splitComplex()
{
	bool result = false;
	if (!_pBlock->polyhedronExists(_polyhedronId))
		return false;
	try {

#if 0 && defined(_DEBUG)
		getBlockPtr()->getVolume()->writeObj("D:/DarkSky/Projects/output/objs/splitting_cell.obj", { _polyhedronId }, false, false, false);
#endif
		CellType cellType = CT_UNKNOWN;

		{
			auto& parentCell = getPolyhedron(_polyhedronId);
			Utils::Timer tmr(Utils::Timer::TT_splitAtPointInner);

			createHexCellData(parentCell);

			auto& cornerVertIds = parentCell.getCanonicalVertIds();
			vector<Vector3d> cornerPts;
			cornerPts.reserve(cornerVertIds.size());
			for (const auto& id : cornerVertIds)
				cornerPts.push_back(getVertexPoint(id));

			switch (cornerPts.size()) {
			case 8:
				cellType = CT_HEX;
				break;
			default:
				break;
			}
		}

		// DO NOT USE the parentCell centroid! It is at a different location than the parametric center. That results in faces which do 
		// match with the neighbor cells's faces.
		// Now split the parentCell
		Vector3d tuv(0.5, 0.5, 0.5);
		switch (cellType) {
		case CT_HEX: {
#if DEBUGGING_MUTEXES_ENABLED
			static mutex lockMutexPtrMutex, lockMutex;
			shared_ptr<lock_guard<mutex>> pLg;
			{
				auto pVol = getBlockPtr()->getVolume();

				lock_guard lg(lockMutexPtrMutex);
				if (pVol->isCellSelected(_polyhedronId)) {
					pLg = make_shared<lock_guard<mutex>>(lockMutex);
				}
			}
#endif
			result = complexityBisectionHexSplit(_polyhedronId, 0, 8);
			break;
		}
		default:
			result = false;
		}

		finalizeCreatedCells();

		int dbgBreak = 1;
	}
	catch (const runtime_error& err) {
		cout << "Exception thrown: " << err.what() << "\n";
	}
	return result;
}

bool Splitter3D::splitConditional()
{
#ifdef _DEBUG
	if (Index3DId(2, 0, 3, 39) == _polyhedronId || Index3DId(2, 0, 3, 41) == _polyhedronId) {
		int dbgBreak = 1;
	}
#endif // 
	bool result = false;
	if (!_pBlock->polyhedronExists(_polyhedronId))
		return false;
	try {

#if 0 && defined(_DEBUG)
		getBlockPtr()->getVolume()->writeObj("D:/DarkSky/Projects/output/objs/splitting_cell.obj", { _polyhedronId }, false, false, false);
#endif
		CellType cellType = CT_UNKNOWN;

		{
			auto& parentCell = getPolyhedron(_polyhedronId);
			Utils::Timer tmr(Utils::Timer::TT_splitAtPointInner);

			createHexCellData(parentCell);

			auto& cornerVertIds = parentCell.getCanonicalVertIds();
			vector<Vector3d> cornerPts;
			cornerPts.reserve(cornerVertIds.size());
			for (const auto& id : cornerVertIds)
				cornerPts.push_back(getVertexPoint(id));

			switch (cornerPts.size()) {
			case 8:
				cellType = CT_HEX;
				break;
			default:
				break;
			}
		}

		// DO NOT USE the parentCell centroid! It is at a different location than the parametric center. That results in faces which do 
		// match with the neighbor cells's faces.
		// Now split the parentCell
		Vector3d tuv(0.5, 0.5, 0.5);
		switch (cellType) {
		case CT_HEX: {
#ifdef _DEBUG
			if (_polyhedronId == Index3DId(3, 0, 4, 5)) {
				int dbgBreak = 1;
			}
#endif
			result = conditionalBisectionHexSplit(_polyhedronId, 0, 8);
			break;
		}
		default:
			result = false;
		}

		finalizeCreatedCells();

		int dbgBreak = 1;
	} catch (const runtime_error& err) {
		cout << "Exception thrown: " << err.what() << "\n";
	}
	return result;
}

bool Splitter3D::conditionalBisectionHexSplit(const Index3DId& parentId, int testedAxisBits, int numPossibleSplits)
{
	if (_polyhedronId == Index3DId(3, 0, 4, 5) || parentId == Index3DId(3, 0, 4, 5)) {
		int dbgBreak = 1;
	}

	bool wasSplit = false;
	int splitAxis = determineBestConditionalSplitAxis(parentId, testedAxisBits, numPossibleSplits);

	if (splitAxis != -1) {
		MTC::vector<Index3DId> newCellIds;
		bisectHexCell(parentId, splitAxis, newCellIds);

		int axisBit = 1 << splitAxis;
		testedAxisBits |= axisBit;

		if (numPossibleSplits == 8) {
			for (const auto& cellId : newCellIds) {
				conditionalBisectionHexSplit(cellId, testedAxisBits, 4);
			}
		} else if (numPossibleSplits == 4) {
			for (const auto& cellId : newCellIds) {
				conditionalBisectionHexSplit(cellId, testedAxisBits, 2);
			}
		}

		wasSplit = true;
	}

	return wasSplit;
}

int Splitter3D::determineBestConditionalSplitAxis(const Index3DId& parentId, int testedAxisBits, int numPossibleSplits)
{
#if 0 && defined(_DEBUG)
	if (_polyhedronId == Index3DId(3, 0, 4, 5) || parentId == Index3DId(3, 0, 4, 5)) {
		int dbgBreak = 1;
	}
#endif
	const auto& parentCell = getPolyhedron(parentId);
	if (!parentCell.intersectsModel())
		return -1;

	int minIntersections = INT_MAX;
	int bestIntersectionSplitAxis = -1;
	int bestCurvatureSplitAxis = -1;

	// Search for the best split axis
	// If there's a split which creates sub cells where one intersects and one does not (numIntersection == 1), take it as soon as found.
	// If there's a cell which has two intersection splits AND needs a curvature split, record it for later
	// If there're no splits with 1 intersecting subCell AND a curvature split is needed, use the first axis which had a curvature split
	for (int axis = 0; axis < 3; axis++) {
		int axisBit = 1 << axis;
		if ((testedAxisBits & axisBit) == axisBit)
			continue;

		int numIntersections = 0;
		{
			auto scratchCellId = makeScratchCell(parentId);

			Utils::ScopedRestore restore1(_testRun);
			Utils::ScopedRestore restore2(_pBlock);
			_testRun = true;
			_pBlock = _pScratchBlock;

			MTC::vector<Index3DId> newCellIds;
			bisectHexCell(scratchCellId, axis, newCellIds);

			for (size_t cellNum = 0; cellNum < 2; cellNum++) {
				const auto& newCell = getPolyhedron(newCellIds[cellNum]);
				// intersectsModel and needsCurvatureSplit test if _splitLevel is appropriate, so it's not needed here
				if (intersectsModel(newCell)) {
					numIntersections++;
				}
			}

			if (numIntersections < minIntersections) {
				// If neither sub cell intersects, we do not need to split this one
				minIntersections = numIntersections;
				bestIntersectionSplitAxis = axis;
			}

			reset(newCellIds);
		}

		if (numIntersections == 2 && bestCurvatureSplitAxis == -1) {
			const auto& parentCell = getPolyhedron(parentId);
			// intersectsModel and needsCurvatureSplit test if _splitLevel is appropriate, so it's not needed here
			if (needsCurvatureSplit(parentCell, axis)) {
				bestCurvatureSplitAxis = axis;
			}
		}
	}

	if (_splitLevel < _params.numIntersectionDivs) {
		return bestIntersectionSplitAxis;
	} else if (_splitLevel < _params.numCurvatureDivs) {
		if (minIntersections == 1) {
			// If we're doing curvature splits, split where there's any intersection which generates intersecting and non-intersectingcells.
			return bestIntersectionSplitAxis;
		} else if (minIntersections== 2) { // The only way to reach here is there's a cuvature split. We could test if bestCurvatureSplitAxis != -1, but that will be handled by the caller.
			return bestCurvatureSplitAxis;
		}
	}

	return -1;
}

bool Splitter3D::complexityBisectionHexSplit(const Index3DId& parentId, int testedAxisBits, int numPossibleSplits)
{
	if (!getBlockPtr()->doQualitySplits())
		return false;

	bool wasSplit = false;
	int splitAxis = determineBestComplexitySplitAxis(parentId, testedAxisBits, numPossibleSplits);

	if (splitAxis != -1) {
		MTC::vector<Index3DId> newCellIds;
		bisectHexCell(parentId, splitAxis, newCellIds);

		int axisBit = 1 << splitAxis;
		testedAxisBits |= axisBit;

		if (numPossibleSplits == 8) {
			for (const auto& cellId : newCellIds) {
				complexityBisectionHexSplit(cellId, testedAxisBits, 4);
			}
		} else if (numPossibleSplits == 4) {
			for (const auto& cellId : newCellIds) {
				complexityBisectionHexSplit(cellId, testedAxisBits, 2);
			}
		}

		wasSplit = true;
	}

	return wasSplit;
}

int Splitter3D::determineBestComplexitySplitAxis(const Index3DId& parentId, int testedAxisBits, int numPossibleSplits)
{
	if (!getBlockPtr()->doQualitySplits())
		return -1;

	const auto& parentCell = getPolyhedron(parentId);
	if (!parentCell.isTooComplex(_params))
		return -1;


	size_t minFaceSplits = INT_MAX;
	double minOfMaxOrtho = DBL_MAX;
	int bestFaceSplitAxis = -1;
	int bestOrthoSplitAxis = -1;

	for (int axis = 0; axis < 3; axis++) {
		int axisBit = 1 << axis;
		if ((testedAxisBits & axisBit) == axisBit)
			continue;

		auto scratchCellId = makeScratchCell(parentId);

		Utils::ScopedRestore restore1(_testRun);
		Utils::ScopedRestore restore2(_pBlock);
		_testRun = true;
		_pBlock = _pScratchBlock;

		MTC::vector<Index3DId> newCellIds;
		bisectHexCell(scratchCellId, axis, newCellIds);

		size_t totalTooManyFaces = 0;
		double maxOrtho = 0;

		for (size_t cellNum = 0; cellNum < 2; cellNum++) {
			const auto& newCell = getPolyhedron(newCellIds[cellNum]);

			if (newCell.hasTooManFaces(_params)) {
				totalTooManyFaces++;
			}

			auto ortho = newCell.maxOrthogonalityAngleRadians();
			if (ortho > maxOrtho) {
				maxOrtho = ortho;
			}
		}

		// MUST split on face complexity first. If face complexity gets too high, the splitting process degenerates. The is also reduces orthoganlity by itself.
		if (totalTooManyFaces < minFaceSplits) {
			minFaceSplits = totalTooManyFaces;
			bestFaceSplitAxis = axis;
		}

		if (maxOrtho < minOfMaxOrtho) {
			minOfMaxOrtho = maxOrtho;
			bestOrthoSplitAxis = axis;
		}
		reset(newCellIds);
	}

	if (bestFaceSplitAxis == -1) {
		return bestFaceSplitAxis;
	}

	return bestOrthoSplitAxis;
}

bool Splitter3D::intersectsModel(const Polyhedron& testCell) const
{
	return testCell.intersectsModel();
}

bool Splitter3D::needsCurvatureSplit(const Polyhedron& testCell, int axis) const
{
	if (_splitLevel >= _params.numIntersectionDivs && _splitLevel < _params.numCurvatureDivs)
		return testCell.needsCurvatureSplit(_params, axis);
	return false;
}

bool Splitter3D::planeFromPoints(const std::vector<Vector3d>& pts, Planed& pl)
{
	if (pts.size() < 3)
		return false;

	const auto tolSqr = Tolerance::sameDistTolSqr();
	Vector3d norm(0, 0, 0);
	const auto& pt0 = pts[0];
	for (size_t i = 1; i < pts.size() - 2; i++) {
		size_t j = (i + 1) % pts.size();
		const auto& pt1 = pts[i];
		const auto& pt2 = pts[j];
		Vector3d v0 = pt0 - pt1;
		double lSqr0 = v0.squaredNorm();
		if (lSqr0 < tolSqr)
			return false;

		Vector3d v1 = pt2 - pt1;

		double lSqr1 = v0.squaredNorm();
		if (lSqr1 < tolSqr)
			return false;

		v0 /= sqrt(lSqr0);
		v1 /= sqrt(lSqr1);
		Vector3d n = v1.cross(v0);

		norm += n;
	}

	Vector3d origin(0, 0, 0);
	for (const auto& pt : pts) 
		origin += pt;
	
	origin /= pts.size();
	pl = Planed(origin, norm);

	return true;
}

void Splitter3D::bisectHexCell(const Index3DId& parentId, int splitAxis, MTC::vector<Index3DId>& newCellIds)
{
	auto pBlk = getBlockPtr();
	auto& parentCell = getPolyhedron(parentId);

	MTC::vector<MTC::vector<Vector3d>> subCellsPts;
	MTC::vector<Vector3d> splittingFacePts;
	parentCell.makeHexCellPoints(splitAxis, subCellsPts, splittingFacePts);
	MTC::vector<Index3DId> splittingFaceVerts;
	splittingFaceVerts.resize(splittingFacePts.size());
	for (size_t i = 0; i < splittingFacePts.size(); i++)
		splittingFaceVerts[i] = vertId(splittingFacePts[i]);

	auto splittingFaceId = pBlk->addPolygon(Polygon(splittingFaceVerts));

	imprintCellOnFace(splittingFaceId, parentCell); // create all cell edges on the splitting face to create required new vertices
	splitCell(parentCell, splittingFaceId);

	parentCell.detachFaces();
	const auto& faceIds = parentCell.getFaceIds();
	for (const auto& subCellPts : subCellsPts) {
		MTC::vector<Index3DId> subCellVerts;
		subCellVerts.resize(subCellPts.size());
		Vector3d ctr(0, 0, 0);
		for (size_t i = 0; i < subCellPts.size(); i++) {
			ctr += subCellPts[i];
			subCellVerts[i] = vertId(subCellPts[i]);
		}
		ctr /= subCellPts.size();

		MTC::vector<Index3DId> newCellFaceIds;
		newCellFaceIds.push_back(splittingFaceId);

		MTC::vector<MTC::vector<Vector3d>> allCellFacesPts;
		GradingOp::getCubeFacePoints(subCellPts, allCellFacesPts);
		vector<Planed> boundingPlanes;
		for (const auto& cellFacePts : allCellFacesPts) {

			Planed pl;
			if (!planeFromPoints(cellFacePts, pl)) {
				stringstream ss;
				ss << "planeFromPoints failed. " << __FILE__ << "-" << __LINE__;
				throw runtime_error(ss.str());
			}

			// Force all plane normals to point outward from the cell center
			Vector3d v = pl.getOrigin() - ctr;
			if (pl.getNormal().dot(v) < 0)
				pl.reverse();
			boundingPlanes.push_back(pl);
		}

		for (const auto& faceId : faceIds) {
			if (cellBoundsContainsFace(boundingPlanes, faceId)) {
				newCellFaceIds.push_back(faceId);
			}
		}

		// This will add the newCell's Id to all the new cell's faces
		Polyhedron tmp(newCellFaceIds, subCellVerts);
		auto newCellId = pBlk->addCell(tmp, parentId);
		auto& newCell = getPolyhedron(newCellId);

		newCellIds.push_back(newCellId);
		_createdCellIds.insert(newCellId);
	}

	_createdCellIds.erase(parentId);
	getBlockPtr()->freePolyhedron(parentId);
}

void Splitter3D::imprintCellOnFace(const Index3DId& splittingFaceId, const Polyhedron& parentCell)
{
	const auto tol = Tolerance::sameDistTol();

	// Imprint existing vertices on the splitting face.
	// This handles cases where the existing cell has a T-intersection with a splitting face edge
	MTC::set<Index3DId> vertIds;
	parentCell.getVertIds(vertIds);

	auto& face = getPolygon(splittingFaceId);
	auto& faceVertIds = face.getVertexIds();
	for (const auto& imprintVertId : vertIds) {
		for (size_t i = 0; i < faceVertIds.size(); i++) {
			size_t j = (i + 1) % faceVertIds.size();
			EdgeKey ek(faceVertIds[i], faceVertIds[j]);
			bool imprinted;
			edgeFunc(ek, [&imprintVertId, &imprinted](Edge& edge) {
				imprinted = edge.imprintVertex(imprintVertId);
			});

			if (imprinted)
				break;
		}
	}

	// Imprint parentCell edges which intersect a splitting face edge.
	auto edgeKeys = parentCell.getEdgeKeys(false);

	const auto& facePlane = face.calPlane();
	set<Index3DId> iVertIds;
	for (const auto& ek : edgeKeys) {
		const auto& pt0 = getVertexPoint(ek[0]);
		const auto& pt1 = getVertexPoint(ek[1]);
		LineSegment_byrefd seg(pt0, pt1);
		RayHitd hit;
		if (facePlane.intersectLineSegment(seg, hit, tol)) {
			auto iVertId = vertId(hit.hitPt);
			if (!face.containsVertex(iVertId)) {
				iVertIds.insert(iVertId);
			}
		}
	}

	for (const auto& imprintVertId : iVertIds) {
		for (size_t i = 0; i < faceVertIds.size(); i++) {
			size_t j = (i + 1) % faceVertIds.size();
			EdgeKey ek(faceVertIds[i], faceVertIds[j]);
			bool imprinted;
			edgeFunc(ek, [&imprintVertId, &imprinted](Edge& edge) {
				imprinted = edge.imprintVertex(imprintVertId);
			});

			if (imprinted)
				break;
		}
	}
}

void Splitter3D::splitCell(Polyhedron& parentCell, const Index3DId& splittingFaceId)
{
	const auto tol = Tolerance::sameDistTol();

	imprintSplittingFaceVerts(parentCell, splittingFaceId);

	const auto& splittingFace = getPolygon(splittingFaceId);
	const auto splittingEdgeKeys = splittingFace.getEdgeKeys();

	auto faceIds = parentCell.getFaceIds();
	for (const auto& faceId : faceIds) {
		auto& face = getPolygon(faceId);
		const auto& facePlane = face.calPlane();
		for (const auto& ek : splittingEdgeKeys) {
			const auto& pt0 = getVertexPoint(ek[0]);
			if (facePlane.isCoincident(pt0, tol)) {
				const auto& pt1 = getVertexPoint(ek[1]);
				if (facePlane.isCoincident(pt1, tol)) {
					splitFace(face, ek);
				}
			}
		}
	}
}

bool Splitter3D::cellBoundsContainsFace(const std::vector<Planed>& boundingPlanes, const Index3DId& faceId)
{
	const auto tol = Tolerance::sameDistTol();

	const auto& face = getPolygon(faceId);
	const auto& vertIds = face.getVertexIds();
	for (const auto& vertId : vertIds) {
		const auto& pt = getVertexPoint(vertId);
		for (const auto& pl : boundingPlanes) {
			auto dist = pl.distanceToPoint(pt, false);
			if (dist > tol)
				return false;
		}
	}
	return true;
}

void Splitter3D::finalizeCreatedCells()
{
	const auto& model = _pBlock->getModel();

	for (auto& createdCellId : _createdCellIds) {
		auto& createdCell = getPolyhedron(createdCellId);

		// If the parent cell doesn't intersect the model, it's sub cells cannot intersect either
		if (!_intersectsModel)
			createdCell.setIntersectsModel(false);
		else {
			auto subBbox = createdCell.getBoundingBox();
			if (!createdCell._hasSetSearchTree) {
				createdCell._hasSetSearchTree = true;
				if (_hasSetSearchTree) {
					createdCell._pPolySearchTree = _pPolySearchTree;
				} else {
					createdCell._pPolySearchTree = _pBlock->getPolySearchTree();
				}

				if (createdCell._pPolySearchTree) {
					size_t numInTree = createdCell._pPolySearchTree->numInTree();
					if (numInTree == 0)
						createdCell._pPolySearchTree = nullptr;
					else if (numInTree > MAX_SUB_TREE_COUNT) {
						size_t n = createdCell._pPolySearchTree->count(subBbox, createdCell.getRefiner());
						if (numInTree > SUB_TREE_SPLIT_RATIO * n) {
							// Splitting small trees takes time and memory, so only reduce larger ones
							createdCell._pPolySearchTree = createdCell._pPolySearchTree->getSubTree(subBbox, createdCell.getRefiner());
						}
					}
				}
			}
		}
		createdCell.setSplitLevel(_splitLevel + 1);
	}
}

bool Splitter3D::imprintSplittingFaceVerts(const Polyhedron& parentCell, const Index3DId& splittingFaceId)
{
	bool result = false;
	const auto tol = Tolerance::sameDistTol();

	set<Index3DId> faceIds;
	auto& splittingFace = getPolygon(splittingFaceId);
	const auto& vertIds = splittingFace.getVertexIds();

	const auto cellEdgeKeys = parentCell.getEdgeKeys(false);
	for (const auto& ek : cellEdgeKeys) {
		edgeFunc(ek, [&vertIds, &result](Edge& edge) {
			for (const auto& vertId : vertIds) {
				result = edge.imprintVertex(vertId) || result;
			}
		});
	}
	return result;
}

bool Splitter3D::splitFace(Polygon& targetFace, const EdgeKey& toolEdgeKey)
{
	// If the target face aleady contains this edge return
	bool containsToolEdgeKey = false;
	targetFace.iterateEdgeKeys([&toolEdgeKey, &containsToolEdgeKey](const EdgeKey& ek)->bool {
		if (toolEdgeKey == ek)
			containsToolEdgeKey = true;
		return !containsToolEdgeKey;
		});

	if (containsToolEdgeKey)
		return false;

	const auto& vertIds = targetFace.getVertexIds();
	size_t idx[2] = { 0xffffffffffffffff, 0xffffffffffffffff };
	int numIdx = 0;
	for (size_t i = 0; i < vertIds.size(); i++) {
		if (toolEdgeKey.containsVertex(vertIds[i])) {
			if (numIdx < 2 && idx[numIdx] == -1)
				idx[numIdx++] = i;
		}
	}

	if (numIdx != 2)
		return false;

	MTC::vector<Index3DId> newVertIds[2];
	for (int idxNum = 0; idxNum < 2; idxNum++) {
		for (size_t i = 0; i < vertIds.size(); i++) {
			size_t j = (i + idx[idxNum]) % vertIds.size();
			newVertIds[idxNum].push_back(vertIds[j]);
			if (j == idx[1 - idxNum])
				break;
		}
	}

	auto cellIds = targetFace.getCellIds();
	assert(!cellIds.empty());

	auto targetId = targetFace.getId();
	for (int i = 0; i < 2; i++) {
		auto faceId = getBlockPtr()->addPolygon(Polygon(newVertIds[i]));
		for (const auto& cellId : cellIds) {
			auto& cell = getPolyhedron(cellId);
			cell.removeFace(targetId);
			cell.addFace(faceId);
		}
	}

	getBlockPtr()->freePolygon(targetId);
	return true;
}

void Splitter3D::addFaceToLocalEdgeSet(map<EdgeKey, FastBisectionSet<Index3DId>>& localEdgeSet, const Index3DId& faceId) const
{
	auto& face = getPolygon(faceId);
	auto& vertIds = face.getVertexIds();
	for (size_t i = 0; i < vertIds.size(); i++) {
		size_t j = (i + 1) % vertIds.size();
		EdgeKey e(vertIds[i], vertIds[j]);
		auto iter = localEdgeSet.find(e);
		if (iter == localEdgeSet.end())
			iter = localEdgeSet.insert(make_pair(e, FastBisectionSet<Index3DId>())).first;
		iter->second.insert(faceId);
	}
}

void Splitter3D::removeFacefromLocalEdgeSet(map<EdgeKey, FastBisectionSet<Index3DId>>& localEdgeSet, const Index3DId& faceId) const
{
	auto& face = getPolygon(faceId);
	auto& vertIds = face.getVertexIds();
	for (size_t i = 0; i < vertIds.size(); i++) {
		size_t j = (i + 1) % vertIds.size();
		EdgeKey e(vertIds[i], vertIds[j]);
		auto iter = localEdgeSet.find(e);
		if (iter != localEdgeSet.end()) {
			iter->second.erase(faceId);
			if (iter->second.empty())
				localEdgeSet.erase(e);
		}
	}
}

Index3DId Splitter3D::findConnectedFaceId(const map<EdgeKey, FastBisectionSet<Index3DId>>& localEdgeSet, const Index3DId& faceId) const
{
	Index3DId result;
	auto& face = getPolygon(faceId);
	auto& vertIds = face.getVertexIds();
	for (size_t i = 0; i < vertIds.size(); i++) {
		size_t j = (i + 1) % vertIds.size();
		Edge edge(EdgeKey(vertIds[i], vertIds[j]), getBlockPtr());
		auto iter = localEdgeSet.find(edge);
		if (iter != localEdgeSet.end()) {
			auto& pair = *iter;
			auto& faceIds = pair.second;
			if (faceIds.size() == 1) {
				result = *faceIds.begin();
				break;
			}
		}
	}

	return result;
}

Index3DId Splitter3D::makeCellFromHexFaces(const Index3DId& parentId, const Index3DId& splittingFaceId, const MTC::vector<Vector3d>& cornerPts,
	FastBisectionSet<Index3DId>& allCellFaceIds, bool useAllFaces)
{
#ifdef _DEBUG
	if (Index3DId(2, 4, 7, 0) == _polyhedronId) {
		int dbgBreak = 1;
	}
#endif // _DEBUG

	// Failing on skewed cells. Need to use a full topological assembly, not the current normal based one.
	MTC::set<Index3DId> cellFaces;

	if (!useAllFaces) {
		map<EdgeKey, FastBisectionSet<Index3DId>> localEdgeSet;

		addFaceToLocalEdgeSet(localEdgeSet, splittingFaceId);
		for (auto& id : allCellFaceIds) {
			addFaceToLocalEdgeSet(localEdgeSet, id);
		}

		verifyLocalEdgeSet(localEdgeSet, splittingFaceId);

		Index3DId seedVert = getBlockPtr()->getVertexIdOfPoint(cornerPts[0]);
		vertexFunc(seedVert, [this, &allCellFaceIds, &cellFaces, &localEdgeSet](const Vertex& vert) {
			auto& faceIds = vert.getFaceIds();
			for (auto& id : faceIds) {
				if (allCellFaceIds.contains(id)) {
					cellFaces.insert(id);
					removeFacefromLocalEdgeSet(localEdgeSet, id);
					allCellFaceIds.erase(id);
					break;
				}
			}
		});

		bool found;
		do {
			found = false;
			for (auto& faceId : cellFaces) {
				auto nextId = findConnectedFaceId(localEdgeSet, faceId);
				if (nextId.isValid()) {
					cellFaces.insert(nextId);
					removeFacefromLocalEdgeSet(localEdgeSet, nextId);
					allCellFaceIds.erase(nextId);
					found = true;
					break;
				}
			}
		} while (found);
	} else {
		cellFaces.insert(allCellFaceIds.begin(), allCellFaceIds.end());
	}

	cellFaces.insert(splittingFaceId);

	MTC::vector<Index3DId> cornerVertIds;
	for (const auto& pt : cornerPts)
		cornerVertIds.push_back(vertId(pt));

	Polyhedron tmpCell(cellFaces, cornerVertIds);
	auto newCellId = getBlockPtr()->addCell(tmpCell, _polyhedronId);
	auto& newCell = getBlockPtr()->getPolyhedron(newCellId);
	newCell._parentId = parentId;
	if (_testRun) {
		const auto& parentCell = getBlockPtr()->getPolyhedron(parentId);
		newCell._hasSetSearchTree = parentCell._hasSetSearchTree;
		newCell._pPolySearchTree = parentCell._pPolySearchTree;
	} else
		_createdCellIds.insert(newCellId);

#if DEBUG_BREAKS && defined(_DEBUG)
	cellFunc(newCellId, [this](const Polyhedron& cell) {
		if (!cell.isClosed()) {
			stringstream ss;
			ss << "D:/DarkSky/Projects/output/objs/unclosedCell_" << getBlockPtr()->getLoggerNumericCode(cell.getId()) << "_post.obj";
			getBlockPtr()->getVolume()->writeObj(ss.str(), { cell.getId() }, false, false, false);

			// Dump each face to it's own file
			for (auto& id : cell.getFaceIds()) {
				MTC::vector<Vector3d> pts;
				faceFunc(id, [this, &pts](const Polygon& face) {
					for (auto& id : face.getVertexIds()) {
						auto& pt = getBlockPtr()->getVertexPoint(id);
						pts.push_back(pt);
					}
				});
				stringstream ss;
				ss << "D:/DarkSky/Projects/output/objs/unclosedCell_" << getBlockPtr()->getLoggerNumericCode(cell.getId()) << "_face_" << getBlockPtr()->getLoggerNumericCode(id) << "_post.obj";
				getBlockPtr()->getVolume()->writeObj(ss.str(), { pts }, true);

			}
			assert(!"Cell not closed");
		}
	});
#endif
	return newCellId;
}

void Splitter3D::verifyLocalEdgeSet(const map<EdgeKey, FastBisectionSet<Index3DId>>& localEdgeSet, const Index3DId& splittingFaceId) const
{
	set<EdgeKey> unclosedEdges;
	for (auto& pair : localEdgeSet) {
		if (pair.second.size() < 2) {
			unclosedEdges.insert(pair.first);
		}
	}

	if (!unclosedEdges.empty()) {
		vector<vector<Vector3d>> edgePts;
		for (auto& e : unclosedEdges) {
			vector<Vector3d> vec;
			vec.push_back(getVertexPoint(e[0]));
			vec.push_back(getVertexPoint(e[1]));
			edgePts.push_back(vec);
		}

		auto pVol = getBlockPtr()->getVolume();
#if 0
		{
			stringstream ss;
			ss << "D:/DarkSky/Projects/output/objs/_" << getBlockPtr()->getLoggerNumericCode(_polyhedronId) << " badlyImprintedCell_.obj";
			pVol->writeObj(ss.str(), { _polyhedronId }, false, false, false);
			cellFunc(_polyhedronId, [this, pVol](const Polyhedron& cell) {
				auto& faceIds = cell.getFaceIds();
				for (auto& faceId : faceIds) {
					faceFunc(faceId, [this, pVol](const Polygon& face) {
						stringstream ss;
						ss << "D:/DarkSky/Projects/output/objs/_" << getBlockPtr()->getLoggerNumericCode(_polyhedronId) << " f" << getBlockPtr()->getLoggerNumericCode(face.getId()) << " badlyImprintedFace.obj";
						pVol->writeObj(ss.str(), { face.getVertexIds() });
					});
				}
			});
		}

		getBlockPtr()->faceFunc(splittingFaceId, [this, pVol](const Polygon& face) {
			stringstream ss;
			ss << "D:/DarkSky/Projects/output/objs/_" << getBlockPtr()->getLoggerNumericCode(_polyhedronId) << " badlyImprintedSplittingFace.obj";
			pVol->writeObj(ss.str(), { face.getVertexIds() });
		});
#endif
		{
			stringstream ss;
			ss << "D:/DarkSky/Projects/output/objs/_parentCell_" << getBlockPtr()->getLoggerNumericCode(_polyhedronId) << " badlyTestSplit_.obj";
			pVol->writeObj(ss.str(), edgePts, false);
		}
		assert(!"spltting produced laminar edge(s)");
//		string msg = string(__FILE__) + ":" + to_string(__LINE__) + string(" spltting produced laminar edge(s)");
//		throw runtime_error(msg);
	}
}

Index3DId Splitter3D::makeScratchCell(const Index3DId& parentId)
{
	const auto& srcCell = getPolyhedron(parentId);
	const auto& srcFaceIds = srcCell.getFaceIds();
	const auto& srcCanonicalVertIds = srcCell.getCanonicalVertIds();

	MTC::set<Index3DId> newFaceIds;
	for (const auto& srcFaceId : srcFaceIds) {
		const auto& newFaceId = makeScratchFace(srcFaceId);
		newFaceIds.insert(newFaceId);
	}

	MTC::vector<Index3DId> newCanonicalVertIds;
	for (const auto& srcVertId : srcCanonicalVertIds) {
		auto& pt = _pBlock->getVertexPoint(srcVertId);
		auto newVertId = _pScratchBlock->addVertex(pt);
		newCanonicalVertIds.push_back(newVertId);
	}

	Polyhedron tmpCell(newFaceIds, newCanonicalVertIds);
	auto scratchCellId = _pScratchBlock->addCell(tmpCell, Index3DId());

	auto& newCell = _pScratchBlock->getPolyhedron(scratchCellId);
	newCell.copyCaches(srcCell);
#ifdef _DEBUG
	if (!newCell.isClosed()) {
		assert(!"Scratch cell is not closed.");
	}
#endif // _DEBUG

	return scratchCellId;
}

Index3DId Splitter3D::makeScratchFace(const Index3DId& srcFaceId)
{
	Index3DId newFaceId;
	auto& srcFace = getPolygon(srcFaceId);
	const auto& srcVertIds = srcFace.getVertexIds();
	MTC::vector<Index3DId> newVertIds;
	for (const auto& srcVertId : srcVertIds) {
		const auto& pt = _pBlock->getVertexPoint(srcVertId);
		const auto& newVertId = _pScratchBlock->getVertexIdOfPoint(pt);
		newVertIds.push_back(newVertId);
	}

	newFaceId = _pScratchBlock->addPolygon(Polygon(newVertIds));
	auto& newFace = _pScratchBlock->getPolygon(newFaceId);
	newFace.copyCaches(srcFace);

	return newFaceId;
}

void Splitter3D::createHexCellData(const Polyhedron& targetCell)
{
	_intersectsModel = targetCell.intersectsModel();

	if (_intersectsModel) {
		if (targetCell._hasSetSearchTree)
			_pPolySearchTree = targetCell._pPolySearchTree;
		else
			_pPolySearchTree = targetCell.getBlockPtr()->getPolySearchTree();
	} else {
		_pPolySearchTree = nullptr;
	}
	_hasSetSearchTree = true;
}

inline Index3DId Splitter3D::vertId(const Vector3d& pt)
{
	return _pBlock->getVertexIdOfPoint(pt);
}

inline const Vector3d& Splitter3D::getVertexPoint(const  Index3DId& id) const
{
	return _pBlock->getVertexPoint(id);
}

//LAMBDA_CLIENT_IMPLS(Splitter3D)
void Splitter3D::vertexFunc(const Index3DId& id, const function<void(const Vertex& obj)>& func) const {
	const auto p = getBlockPtr(); 
	p->vertexFunc(id, func);
} 

void Splitter3D::vertexFunc(const Index3DId& id, const function<void(Vertex& obj)>& func) {
	auto p = getBlockPtr(); 
	p->vertexFunc(id, func);
} 

void Splitter3D::faceFunc(const Index3DId& id, const function<void(const Polygon& obj)>& func) const {
	const auto p = getBlockPtr(); 
	p->faceFunc(id, func);
} 

void Splitter3D::faceFunc(const Index3DId& id, const function<void(Polygon& obj)>& func) {
	auto p = getBlockPtr(); 
	p->faceFunc(id, func);
} 

void Splitter3D::cellFunc(const Index3DId& id, const function<void(const Polyhedron& obj)>& func) const {
	const auto p = getBlockPtr(); 
	p->cellFunc(id, func);
} 

void Splitter3D::cellFunc(const Index3DId& id, const function<void(Polyhedron& obj)>& func) {
	auto p = getBlockPtr(); 
	p->cellFunc(id, func);
} 

const Vertex& Splitter3D::getVertex(const Index3DId& id) const {
	return getBlockPtr()->getVertex(id);
}  

Vertex& Splitter3D::getVertex(const Index3DId& id) {
	return getBlockPtr()->getVertex(id);
} 

const DFHM::Polygon& Splitter3D::getPolygon(const Index3DId& id) const {
	return getBlockPtr()->getPolygon(id);
}  

DFHM::Polygon& Splitter3D::getPolygon(const Index3DId& id) {
	return getBlockPtr()->getPolygon(id);
} 

const Polyhedron& Splitter3D::getPolyhedron(const Index3DId& id) const {
	return getBlockPtr()->getPolyhedron(id);
}  

Polyhedron& Splitter3D::getPolyhedron(const Index3DId& id) {
	return getBlockPtr()->getPolyhedron(id);
} 

void Splitter3D::edgeFunc(const EdgeKey& key, const function<void(const Edge& obj)>& func) const {
	const auto p = getBlockPtr(); 
	p->edgeFunc(key, func);
} 

void Splitter3D::edgeFunc(const EdgeKey& key, const function<void(Edge& obj)>& func) {
	auto p = getBlockPtr(); 
	p->edgeFunc(key, func);
}
