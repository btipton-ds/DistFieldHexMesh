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

#include <defines.h>
#include <assert.h>
#include <tm_bestFit.h>
#include <tm_lineSegment.h>
#include <tm_ray.h>
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

Splitter3D::Splitter3D(Block* pBlock, const Index3DId& polyhedronId, size_t level)
	: _pBlock(pBlock)
	, _polyhedronId(polyhedronId)
	, _splitLevel(level)
	, _params(pBlock->getSplitParams())
{
	if (!_pScratchVol)
		_pScratchVol = _pBlock->getVolume()->createScratchVolume();
	_pScratchBlock = _pScratchVol->getBlockPtr(Index3D(0, 0, 0));

	cellFunc(_polyhedronId, [this](Polyhedron& cell) {
#ifdef _DEBUG
		if (!cell.isClosed()) {
			getBlockPtr()->dumpPolyhedraObj({ cell.getId() }, false, false, false);
			assert(!"Cell not closed");
		}
#endif // _DEBUG
	});
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

void Splitter3D::reset()
{
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

bool Splitter3D::splitAtCenter()
{
	bool result = false;
	if (!_pBlock->polyhedronExists(_polyhedronId))
		return false;
	try {

#if 0 && defined(_DEBUG)
		getBlockPtr()->getVolume()->writeObj("D:/DarkSky/Projects/output/objs/splitting_cell.obj", { _polyhedronId }, false, false, false);
#endif
		CellType cellType = CT_UNKNOWN;

		cellFunc(_polyhedronId, [this, &cellType](Polyhedron& parentCell) {
			Utils::Timer tmr(Utils::Timer::TT_splitAtPointInner);

			createHexCellData(parentCell);

			switch (_cornerPts.size()) {
			case 8:
				cellType = CT_HEX;
				break;
			default:
				break;
			}
		});

		// DO NOT USE the parentCell centroid! It is at a different location than the parametric center. That results in faces which do 
		// match with the neighbor cells's faces.
		// Now split the parentCell
		Vector3d tuv(0.5, 0.5, 0.5);
		switch (cellType) {
		case CT_HEX:
			result = conditionalBisectionHexSplit(_polyhedronId, tuv, 0, 8);
			break;
		default:
			result = false;
		}
	} catch (const std::runtime_error& err) {
		cout << "Exception thrown: " << err.what() << "\n";
	}
	return result;
}

bool Splitter3D::conditionalBisectionHexSplit(const Index3DId& parentId, const Vector3d& tuv, int ignoreAxisBits, int numPossibleSplits)
{
	bool wasSplit = false;

	if (numPossibleSplits == 8) {
		// Highest priority, split the cell if it's too complex.
		int bestAxis = findBestHexComplexSplitAxis(parentId, tuv, ignoreAxisBits);
		if (bestAxis != -1) {
			MTC::vector<Index3DId> newCellIds;
			bisectHexCell(parentId, tuv, bestAxis, newCellIds);
			for (const auto& cellId : newCellIds) {
				// Now we split the pair of simplified cells
				conditionalBisectionHexSplit(cellId, tuv, ignoreAxisBits, 8);
			}

			return true;
		}
	}

	// isect[] keeps track of intersections in the 8 possible subcells
	bool isect[8];
	clearCellAll(isect);

	if (_splitLevel < _params.numIntersectionDivs)
		doScratchHexIntersectionSplitTests(parentId, tuv, isect, ignoreAxisBits);
	else if (_splitLevel < _params.numCurvatureDivs)
		doScratchHexCurvatureSplitTests(parentId, tuv, isect, ignoreAxisBits);

	int splitAxis = getSplitAxis(parentId, tuv, isect, ignoreAxisBits, numPossibleSplits);
	if (splitAxis != -1) {
#if 0
		string axisStr;
		switch (splitAxis) {
		case 0:
			axisStr = "xAxis";
			break;
		case 1:
			axisStr = "yAxis";
			break;
		case 2:
			axisStr = "zAxis";
			break;
		}

		switch (numPossibleSplits) {
		case 8:
			cout << "\n\n";
			cout << "Split " << axisStr << " :" << ignoreAxisBits << "\n";
			break;
		case 4:
			cout << "        " << axisStr << " :" << ignoreAxisBits << "\n";
			break;
		case 2:
			cout << "          " << axisStr << " :" << ignoreAxisBits << "\n";
			break;
		default:
			cout << "Error********\n";
			break;
		}
#endif
		MTC::vector<Index3DId> newCellIds;
		bisectHexCell(parentId, tuv, splitAxis, newCellIds);
		if (numPossibleSplits == 8) {
			int axisBit = 1 << splitAxis;
			for (const auto& cellId : newCellIds) {
				conditionalBisectionHexSplit(cellId, tuv, ignoreAxisBits | axisBit, 4);
			}
		}
		else if (numPossibleSplits == 4) {
			int axisBit = 1 << splitAxis;
			for (const auto& cellId : newCellIds) {
				conditionalBisectionHexSplit(cellId, tuv, ignoreAxisBits | axisBit, 2);
			}
		}
		wasSplit = true;
	}

	return wasSplit;
}

inline Index3DId Splitter3D::vertId(const Vector3d& pt)
{
	return _pBlock->getVertexIdOfPoint(pt);
}

inline const Vector3d& Splitter3D::getVertexPoint(const  Index3DId& id) const
{
	return _pBlock->getVertexPoint(id);
}

void Splitter3D::clearHexSplitBits(bool isect[8], int splitAxis, size_t j)
{
	switch (splitAxis) {
	case 0:
		if (j == 0)
			clearCell(isect, { 0, 2, 4, 6 }); // front 4 empty
		else
			clearCell(isect, { 1, 3, 5, 7 }); // back 4 empty
		break;
	case 1:
		if (j == 0)
			clearCell(isect, { 0, 1, 4, 5 }); // right 4 empty
		else
			clearCell(isect, { 2, 3, 6, 7 }); // left 4 empty
		break;
	case 2:
		if (j == 0)
			clearCell(isect, { 0, 1, 2, 3 }); // bottom 4 empty
		else
			clearCell(isect, { 4, 5, 6, 7 }); // top 4 empty
		break;
	}
}

int Splitter3D::findBestHexComplexSplitAxis(const Index3DId& parentId, const Vector3d& tuv, int ignoreAxisBits)
{
	FastBisectionSet<Index3DId> faceIds;
	bool isTooComplex = false;
	cellFunc(parentId, [this, &isTooComplex, &faceIds](const Polyhedron& cell) {
		isTooComplex = cell.isTooComplex(_params);
		if (isTooComplex)
			faceIds = cell.getFaceIds();
	});

	if (!isTooComplex)
		return - 1;
#if 0
	stringstream ss;
	ss << "D:/DarkSky/Projects/output/objs/complex_cell_" << getBlockPtr()->getLoggerNumericCode(parentId) << ".obj";
	getBlockPtr()->getVolume()->writeObj(ss.str(), {parentId}, false, false, false);
#endif

	int minimumOfMaxFaces = INT_MAX;
	int bestAxis = -1;
	for (int splitAxis = 0; splitAxis < 3; splitAxis++)
	{
		int axisBit = 1 << splitAxis;
		bool ignore = (ignoreAxisBits & axisBit) == axisBit;
		if (ignore) {
			continue;
		}

		MTC::vector<MTC::vector<Vector3d>> discarded;
		MTC::vector<Vector3d> partingFacePts;
		makeHexCellPoints(_pBlock, parentId, tuv, splitAxis, discarded, partingFacePts);
		Vector3d v0 = partingFacePts[0] - partingFacePts[1];
		Vector3d v1 = partingFacePts[2] - partingFacePts[1];
		Vector3d n = v1.cross(v0);
		Planed splittingPlane(partingFacePts[0], n);

		int numFacesAbove = 0, numFacesBelow = 0, numCrossingFace = 0;
		for (const auto& faceId : faceIds) {
			int numVertsAbove = 0, numVertsBelow = 0;
			faceFunc(faceId, [this, &splittingPlane, &numVertsAbove, &numVertsBelow](const Polygon& face) {
				auto& vertIds = face.getVertexIds();
				for (const auto& vertId : vertIds) {
					const auto& pt = getVertexPoint(vertId);
					auto dist = splittingPlane.distanceToPoint(pt, false);
					if (dist > Tolerance::sameDistTol())
						numVertsAbove++;
					else if (dist < -Tolerance::sameDistTol())
						numVertsBelow++;
				}
			});

			if (numVertsAbove > 0 && numVertsBelow > 0) {
				numFacesAbove++;
				numFacesBelow++;
			}
			if (numVertsAbove > 0 && numVertsBelow == 0)
				numFacesAbove++;
			else if (numVertsBelow > 0 && numVertsAbove == 0)
				numFacesBelow++;
		}

		int maxFaces = numFacesAbove > numFacesBelow ? numFacesAbove : numFacesBelow;
		if (maxFaces < minimumOfMaxFaces) {
			minimumOfMaxFaces = maxFaces;
			bestAxis = splitAxis;
		}
	}

	return bestAxis;
}

void Splitter3D::doScratchHexIntersectionSplitTests(const Index3DId& parentId, const Vector3d& tuv, bool isect[8], int ignoreAxisBits)
{
	// Split the cell with a plane on each axis
	// When one of the binary split cells has no intersections, it's 4 subcells are marked as no intersect
	// When finished, only subcells with intersections are marked true
	for (int i = 0; i < 8; i++)
		isect[i] = true;

	for (int splitAxis = 0; splitAxis < 3; splitAxis++)
	{
		int axisBit = 1 << splitAxis;
		bool ignore = (ignoreAxisBits & axisBit) == axisBit;
		if (ignore) {
			clearHexSplitBits(isect, splitAxis, 0);
			clearHexSplitBits(isect, splitAxis, 1);
			continue;
		}

		Utils::ScopedRestore restore1(_testRun);
		_testRun = true;
		const auto scratchCellId = createScratchCell(parentId);
		MTC::vector<Index3DId> newCellIds;
		makeScratchHexCells(scratchCellId, tuv, splitAxis, newCellIds);
		for (size_t j = 0; j < 2; j++) {
			_pScratchBlock->cellFunc(newCellIds[j], [this, &isect, splitAxis, j](const Polyhedron& cell) {
				if (!cell.intersectsModel()) {
					clearHexSplitBits(isect, splitAxis, j);
				}
			});
		}
		reset();
	}
}

void Splitter3D::doScratchHexCurvatureSplitTests(const Index3DId& parentId, const Vector3d& tuv, bool isect[8], int ignoreAxisBits)
{
	// Split the cell with a plane on each axis
	// When one of the binary split cells has no intersections, it's 4 subcells are marked as no intersect
	// When finished, only subcells with intersections are marked true

	for (int splitAxis = 0; splitAxis < 3; splitAxis++)
	{
		int axisBit = 1 << splitAxis;
		bool ignore = (ignoreAxisBits & axisBit) == axisBit;
		if (ignore) {
			clearHexSplitBits(isect, splitAxis, 0);
			clearHexSplitBits(isect, splitAxis, 1);
			continue;
		}

		cellFunc(parentId, [this, &tuv, &isect, splitAxis](const Polyhedron& cell) {
			if (cell.intersectsModel()) {
				auto pVol = getBlockPtr()->getVolume();
				int orthAxis0 = (splitAxis + 1) % 3;
				int orthAxis1 = (splitAxis + 2) % 3;;

				MTC::vector<MTC::vector<Vector3d>> discarded;
				MTC::vector<Vector3d> facePts0, facePts1;
				makeHexCellPoints(_pBlock, cell.getId(), tuv, orthAxis0, discarded, facePts0);
				makeHexCellPoints(_pBlock, cell.getId(), tuv, orthAxis1, discarded, facePts1);

				double maxCurv0 = cell.calMaxCurvature2D(facePts0, 0);
				double maxCurv1 = cell.calMaxCurvature2D(facePts1, 1);
				double maxCurv = (maxCurv0 > maxCurv1) ? maxCurv0 : maxCurv1;

				if (maxCurv > 0) {
					MTC::vector<Vector3d> splitFacePts;
					makeHexCellPoints(_pBlock, cell.getId(), tuv, splitAxis, discarded, splitFacePts);
#if 1 && defined(_DEBUG)
					makeHexCellPoints(_pBlock, cell.getId(), tuv, splitAxis, discarded, splitFacePts);
					pVol->writeObj("D:/DarkSky/Projects/output/objs/curvatureSplittingPlane.obj", { splitFacePts }, true);
					pVol->writeObj("D:/DarkSky/Projects/output/objs/curvaturePlane0.obj", { facePts0 }, true);
					pVol->writeObj("D:/DarkSky/Projects/output/objs/curvaturePlane1.obj", { facePts1 }, true);
					pVol->writeObj("D:/DarkSky/Projects/output/objs/cell.obj", { cell.getId() }, false, false, false);
#endif

					double radius = 1 / maxCurv;

					double avgSpan = 0;
					for (size_t i = 0; i < splitFacePts.size(); i++) {
						size_t j = (i + 1) / splitFacePts.size();
						avgSpan += (splitFacePts[j] - splitFacePts[i]).norm();
					}
					avgSpan /= splitFacePts.size();
					double R0 = (radius + pow(avgSpan, 2)) / (4 * radius);
					int dbgBreak = 1;

				}
			}

		});
	}
}

int Splitter3D::getSplitAxis(const Index3DId& parentId, const Vector3d& tuv, bool isect[8], int ignoreAxisBits, int numPossibleSplits)
{
	int splitAxis = -1;
	if (allCellsClear(isect))
		return splitAxis;

	bool doSplit = false;
	for (splitAxis = 0; splitAxis < 3; splitAxis++) {
		doSplit = false;

		int axisBit = 1 << splitAxis;
		if ((axisBit & ignoreAxisBits) == axisBit)
			continue;

		switch (splitAxis) {
		case 0:
			if (cellsNotSet(isect, { 0, 2, 4, 6 }) ||
				cellsNotSet(isect, { 1, 3, 5, 7 })) {
				doSplit = true;
			}
			break;
		case 1:
			if (cellsNotSet(isect, { 0, 1, 4, 5 }) ||
				cellsNotSet(isect, { 2, 3, 6, 7 })) {
				doSplit = true;
			}
			break;
		case 2:
			if (cellsNotSet(isect, { 0, 1, 2, 3 }) ||
				cellsNotSet(isect, { 4, 5, 6, 7 })) {
				doSplit = true;
			}
			break;
		}
		if (doSplit)
			break;
	}

	if (!doSplit) {
		if (allCellsSet(isect, numPossibleSplits)) {
			for (splitAxis = 0; splitAxis < 3; splitAxis++) {
				int axisBit = 1 << splitAxis;
				if ((axisBit & ignoreAxisBits) == axisBit)
					continue;
				break;
			}
			doSplit = true;
		}
	}

	if (doSplit)
		return splitAxis;

	return -1;
}

void Splitter3D::bisectHexCell(const Index3DId& parentId, const Vector3d& tuv, int splitAxis, MTC::vector<Index3DId>& newCellIds)
{
	Index3DId testId(2, 0, 3, 0);
#if 0 && defined(_DEBUG)
	if (testId == parentId) {
		int dbgBreak = 1;
	}
#endif
	MTC::vector<MTC::vector<Vector3d>> subCells;
	MTC::vector<Vector3d> splittingFacePts;
	makeHexCellPoints(_pBlock, parentId, tuv, splitAxis, subCells, splittingFacePts);
	MTC::vector<Index3DId> splittingFaceVertIds;
	for (const auto& pt : splittingFacePts)
		splittingFaceVertIds.push_back(vertId(pt));
	auto splittingFaceId = getBlockPtr()->addPolygon(Polygon(splittingFaceVertIds));

	FastBisectionSet<Index3DId> faceIds;
	cellFunc(parentId, [&faceIds](const Polyhedron& cell) {
		faceIds = cell.getFaceIds();
	});

	faceFunc(splittingFaceId, [&faceIds](Polygon& splittingFace) {
		splittingFace.imprintFaces(faceIds);
	});

	FastBisectionSet<Index3DId> allCellFaceIds;
	cellFunc(parentId, [this, parentId, testId, &splittingFaceId, &allCellFaceIds](Polyhedron& cell) {
#if 0 && defined(_DEBUG)
		if (testId == parentId) {
			{
				stringstream ss;
				ss << "splittingFace_" << getBlockPtr()->getLoggerNumericCode(splittingFaceId);
				MTC::vector<Index3DId> ids;
				ids.push_back(splittingFaceId);
				getBlockPtr()->dumpPolygonObj(ss.str(), ids);
			}
			{
				stringstream ss;
				ss << "D:/DarkSky/Projects/output/objs/cell_" << getBlockPtr()->getLoggerNumericCode(cell.getId()) << "_pre.obj";
				getBlockPtr()->getVolume()->writeObj(ss.str(), { cell.getId() }, false, false, false);
			}
		}
#endif
		FastBisectionSet<Index3DId> touched;
		// This is not producing the 4 way split that was expected
		cell.imprintFaceEdges(splittingFaceId, touched);
		for (const auto& id : touched)
			getBlockPtr()->addToTouchedCellList(id);

		allCellFaceIds = cell.getFaceIds();
		cell.detachFaces();
	});

	for (int i = 0; i < 2; i++) {
		Index3DId cellId = makeCellFromHexFaces(splittingFaceId, subCells[i], allCellFaceIds, i == 1);
#if 0 && defined(_DEBUG)
		cellFunc(cellId, [this, &splittingFaceId](const Polyhedron& cell) {
			if (!cell.isClosed()) {
				stringstream ss;
				ss << "D:/DarkSky/Projects/output/objs/cell_" << getBlockPtr()->getLoggerNumericCode(cell.getId()) << "_post.obj";
				getBlockPtr()->getVolume()->writeObj(ss.str(), { cell.getId() }, false, false, false);
			}
		});
#endif
		newCellIds.push_back(cellId);
	}

	getBlockPtr()->freePolyhedron(parentId);

}

Index3DId Splitter3D::makeCellFromHexFaces(const Index3DId& splittingFaceId, const MTC::vector<Vector3d>& cornerPts, FastBisectionSet<Index3DId>& allCellFaceIds, bool useAllFaces)
{
	MTC::set<Index3DId> cellFaces;

	if (!useAllFaces) {
		// This is the first of two passes. It must be on the first cell of the two produced by makeHexPoints
		Planed splittingPlane;
		faceFunc(splittingFaceId, [&splittingPlane](const Polygon& face) {
			splittingPlane = face.calPlane();
		});

#if 1 && defined(_DEBUG)
		Vector3d cellCtr(0, 0, 0);
		for (const auto& pt : cornerPts)
			cellCtr += pt;
		cellCtr /= cornerPts.size();

		// makeHexPoints should assure that the face normal points into the first cell
		assert(splittingPlane.distanceToPoint(cellCtr, false) > 0);
#endif

		for (const auto& faceId : allCellFaceIds) {
			faceFunc(faceId, [this, &splittingPlane, &cellFaces](const Polygon& face) {
				const auto& vertIds = face.getVertexIds();
				for (const auto& vertId : vertIds) {
					auto& pt = getVertexPoint(vertId);
					double dist = splittingPlane.distanceToPoint(pt, false);
					if (dist > Tolerance::sameDistTol()) {
						cellFaces.insert(face.getId());
						break;
					}
				}
			});
		}

		for (const auto& faceId : cellFaces)
			allCellFaceIds.erase(faceId);
	} else {
		cellFaces.insert(allCellFaceIds.begin(), allCellFaceIds.end());
	}

	cellFaces.insert(splittingFaceId);

	MTC::vector<Index3DId> cornerVertIds;
	for (const auto& pt : cornerPts)
		cornerVertIds.push_back(vertId(pt));

	Polyhedron newCell(cellFaces, cornerVertIds);
	auto newCellId = getBlockPtr()->addCell(newCell, _polyhedronId);

#if 1 && defined(_DEBUG)
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

void Splitter3D::makeHexCellPoints(const Block* pBlock, const Index3DId& parentId, const Vector3d& tuv, int axis, MTC::vector<MTC::vector<Vector3d>>& subCells, MTC::vector<Vector3d>& partingFacePts)
{
	MTC::vector<Vector3d> cornerPts;
	if (!_testRun) {
		pBlock->cellFunc(parentId, [this, pBlock, &cornerPts](const Polyhedron& cell) {
			set<Index3DId> cvSet;
			auto& cornerVertIds = cell.getCanonicalVertIds();
			for (const auto& id : cornerVertIds) {
				cornerPts.push_back(pBlock->getVertexPoint(id));
				cvSet.insert(id);
			}
		});
	}

	const auto& cp = (_testRun) ? _cornerPts : cornerPts;

	for (int i = 0; i < 2; i++) {
		double t0 = 0, t1 = 1;
		double u0 = 0, u1 = 1;
		double v0 = 0, v1 = 1;

		switch (axis) {
		case 0:
			t0 = (i == 0) ? 0 : tuv[0];
			t1 = (i == 0) ? tuv[0] : 1;
			break;
		case 1:
			u0 = (i == 0) ? 0 : tuv[1];
			u1 = (i == 0) ? tuv[1] : 1;
			break;
		case 2:
			v0 = (i == 0) ? 0 : tuv[2];
			v1 = (i == 0) ? tuv[2] : 1;
			break;
		}

		MTC::vector<Vector3d> subPts = {
			TRI_LERP(cp, t0, u0, v0),
			TRI_LERP(cp, t1, u0, v0),
			TRI_LERP(cp, t1, u1, v0),
			TRI_LERP(cp, t0, u1, v0),

			TRI_LERP(cp, t0, u0, v1),
			TRI_LERP(cp, t1, u0, v1),
			TRI_LERP(cp, t1, u1, v1),
			TRI_LERP(cp, t0, u1, v1),
		};
		subCells.push_back(subPts);
		if (i == 0) {
			switch (axis) {
			case 0:
				partingFacePts = { subPts[1], subPts[5], subPts[6], subPts[2], };
				break;
			case 1:
				partingFacePts = { subPts[2], subPts[6], subPts[7], subPts[3], };
				break;
			case 2:
				partingFacePts = { subPts[4], subPts[7], subPts[6], subPts[5], };
				break;
			}
		}
	}
}

void Splitter3D::makeScratchHexCells(const Index3DId& parentId, const Vector3d& tuv, int axis, MTC::vector<Index3DId>& newCells)
{
	const double tol = 10 * _distTol; // Sloppier than "exact" match. We just need a "good" match

	MTC::vector<MTC::vector<Vector3d>> subCells;
	MTC::vector<Vector3d> discarded;
	makeHexCellPoints(_pScratchBlock, parentId, tuv, axis, subCells, discarded);

	_pScratchBlock->cellFunc(parentId, [](Polyhedron& cell) {
		cell.detachFaces();
	});

	for (int i = 0; i < 2; i++) {
		const auto& subPts = subCells[i];

		MTC::vector<Index3DId> subCorners;
		for (const auto& pt : subPts)
			subCorners.push_back(_pScratchBlock->getVertexIdOfPoint(pt));

		auto newId = makeScratchHexCell(parentId, subCorners, tol);
		newCells.push_back(newId);
	}
}

Index3DId Splitter3D::createScratchCell(const Index3DId& parentId)
{
	FastBisectionSet<Index3DId> srcFaceIds;
	cellFunc(parentId, [&srcFaceIds](const Polyhedron& srcCell) {
		srcFaceIds = srcCell.getFaceIds();
	});

	MTC::set<Index3DId> newFaceIds;
	for (const auto& srcFaceId : srcFaceIds) {
		const auto& newFaceId = createScratchFace(srcFaceId);
		newFaceIds.insert(newFaceId);
	}

	auto scratchCellId = _pScratchBlock->addCell(Polyhedron(newFaceIds, _cornerVertIds), Index3DId());

	return scratchCellId;
}

Index3DId Splitter3D::createScratchFace(const Index3DId& srcFaceId)
{
	Index3DId newFaceId;
	faceFunc(srcFaceId, [this, &newFaceId](const Polygon& srcFace) {
		const auto& srcVertIds = srcFace.getVertexIds();
		MTC::vector<Index3DId> newVertIds;
		for (const auto& srcVertId : srcVertIds) {
			const auto& pt = _pBlock->getVertexPoint(srcVertId);
			const auto& newVertId = _pScratchBlock->getVertexIdOfPoint(pt);
			newVertIds.push_back(newVertId);
		}

		newFaceId = _pScratchBlock->addPolygon(Polygon(newVertIds));
	});

	return newFaceId;
}

Index3DId Splitter3D::makeScratchHexCell(const Index3DId& parentId, const MTC::vector<Index3DId>& cubeVerts, double tol)
{
	Index3DId newCellId;

	assert(cubeVerts.size() == 8);
	MTC::vector<MTC::vector<Index3DId>> faceVertList;
	GradingOp::getCubeFaceVertIds(cubeVerts, faceVertList);
	assert(faceVertList.size() == 6);

	MTC::set<Index3DId> cellFaceIds;
	for (const auto& faceVerts : faceVertList) {
		assert(faceVerts.size() == 4);
		MTC::set<Index3DId> newFaceIds;
		assert(_testRun);
		// Skip the expensive splitting operations
		auto newFaceId = _pScratchBlock->addPolygon(Polygon(faceVerts));
		cellFaceIds.insert(newFaceId);

	}

	newCellId = _pScratchBlock->addCell(Polyhedron(cellFaceIds, cubeVerts), parentId);

	return newCellId;
}

void Splitter3D::createHexCellData(const Polyhedron& parentCell)
{
	_cornerVertIds = parentCell.getCanonicalVertIds();
	_cornerPts.reserve(_cornerVertIds.size());
	for (const auto& id : _cornerVertIds)
		_cornerPts.push_back(getVertexPoint(id));

	int dbgBreak = 1;
}

inline void Splitter3D::clearCell(bool isect[8], const vector<int>& entries)
{
	for (int i : entries)
		isect[i] = false;
}

inline void Splitter3D::clearCellAll(bool isect[8])
{
	for (int i = 0; i < 8; i++)
		isect[i] = false;
}

inline bool Splitter3D::cellsNotSet(bool isect[8], const vector<int>& entries)
{
	bool result = true;
	for (int i : entries)
		result = result && !isect[i];
	return result;
}

inline bool Splitter3D::cellsSet(bool isect[8], const vector<int>& entries)
{
	bool result = true;
	for (int i : entries)
		result = result && isect[i];
	return result;
}

inline bool Splitter3D::allCellsSet(bool isect[8], int numRequired)
{
	int numSet = 0;
	for (int i = 0; i < 8; i++) {
		if (isect[i])
			numSet++;
	}
	return numSet >= numRequired;
}

bool Splitter3D::allCellsClear(bool isect[8])
{
	int numSet = 0;
	for (int i = 0; i < 8; i++) {
		if (isect[i])
			numSet++;
	}
	return numSet == 0;
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

void Splitter3D::edgeFunc(const EdgeKey& key, const function<void(const Edge& obj)>& func) const {
	const auto p = getBlockPtr(); 
	p->edgeFunc(key, func);
} 

void Splitter3D::edgeFunc(const EdgeKey& key, const function<void(Edge& obj)>& func) {
	auto p = getBlockPtr(); 
	p->edgeFunc(key, func);
}
