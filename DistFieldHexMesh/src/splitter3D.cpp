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

Splitter3D::Splitter3D(Block* pBlock, const Index3DId& polyhedronId, MTC::vector<Index3DId>& localTouched)
	: _pBlock(pBlock)
	, _pSrcBlock(pBlock)
	, _polyhedronId(polyhedronId)
	, _localTouched(localTouched)
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
	assert(pVol->verifyTopology(false));
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
			// DO NOT USE the parentCell centroid! It is at a different location than the parametric center. That results in faces which do 
			// match with the neighbor cells's faces.
			// Now split the parentCell
#if 0 && defined(_DEBUG)
			assert(parentCell.verifyTopology());
			Index3DId testId(6, 4, 5, 0);
			if (testId == _polyhedronId) {
				int dbgBreak = 1;
				_pBlock->dumpPolyhedraObj({ _polyhedronId }, false, false, false);
			}
#endif

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

		Vector3d tuv(0.5, 0.5, 0.5);
		switch (cellType) {
		case CT_HEX:
			result = splitHexCell_8_possible(_polyhedronId, tuv);
			break;
		default:
			result = false;
		}
	} catch (const std::runtime_error& err) {
		cout << "Exception thrown: " << err.what() << "\n";
	}
	return result;
}

inline Index3DId Splitter3D::vertId(const Vector3d& pt)
{
	return _pBlock->getVertexIdOfPoint(pt);
}

inline const Vector3d& Splitter3D::getVertexPoint(const  Index3DId& id) const
{
	return _pBlock->getVertexPoint(id);
}

namespace
{

	inline void clearCell(bool isect[8], const vector<int>& entries)
{
	for (int i : entries)
		isect[i] = false;
}

inline bool cellsNotSet(bool isect[8], const vector<int>& entries)
{
	bool result = true;
	for (int i : entries)
		result = result && !isect[i];
	return result;
}

inline bool cellsSet(bool isect[8], const vector<int>& entries)
{
	bool result = true;
	for (int i : entries)
		result = result && isect[i];
	return result;
}

inline bool allCellsSet(bool isect[8], int numRequired)
{
	int numSet = 0;
	for (int i = 0; i < 8; i++) {
		if (isect[i])
			numSet++;
	}
	return numSet >= numRequired;
}

}

void Splitter3D::performTestHexSplits(const Index3DId& parentId, const Vector3d& tuv, bool isect[8], int ignoreAxisBits)
{
	for (int i = 0; i < 8; i++)
		isect[i] = true;

	for (int splitAxis = 0; splitAxis < 3; splitAxis++)
	{
		int mask = 1 << splitAxis;
		bool ignore = ignoreAxisBits & mask;
		if (ignore)
			continue;

		Utils::ScopedRestore restore0(_pBlock);
		_pBlock = _pScratchBlock;
		Utils::ScopedRestore restore1(_testRun);
		_testRun = true;
		const auto scratchCellId = createScratchCell(parentId);
		MTC::vector<Index3DId> newCellIds;
		makeTestHexCells_2_hexes(scratchCellId, tuv, splitAxis, newCellIds);
		for (size_t j = 0; j < 2; j++) {
			cellFunc(newCellIds[j], [&isect, splitAxis, j](const Polyhedron& cell) {
				if (!cell.intersectsModel()) {
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
				});
		}
		reset();
	}
}

int Splitter3D::getSplitAxis8(const Index3DId& parentId, const Vector3d& tuv)
{
	// Split the cell with a plane on each axis
	// intersects[] keeps track of intersections in the 8 possible subcells
	// When one of the binary split cells has no intersections, it's 4 subcells are marked as no intersect
	// When finished, only subcells with intersections are marked true
	bool isect[8];

	performTestHexSplits(parentId, tuv, isect, 0);

	int splitAxis;
	bool doSplit = false;
	for (splitAxis = 0; splitAxis < 3; splitAxis++) {
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
#if 1
		if (!doSplit) {
			// TODO Do curvature split testing here
			if (allCellsSet(isect, 8))
				doSplit = true;
		}
#endif
		if (doSplit)
			break;
	}

	return splitAxis;
}

bool Splitter3D::splitHexCell_8_possible(const Index3DId& parentId, const Vector3d& tuv)
{
	bool wasSplit = false;

	int splitAxis = getSplitAxis8(parentId, tuv);
	if (splitAxis < 3) {
		MTC::vector<Index3DId> newCellIds;
		splitHexCell_2(parentId, tuv, splitAxis, newCellIds);
		for (const auto& cellId : newCellIds) {
			int ignoreAxisBits = 1 << splitAxis;
			splitHexCell_4_possible(cellId, tuv, ignoreAxisBits);
		}
		wasSplit = true;
	}
	
	return wasSplit;
}

void Splitter3D::splitHexCell_4_possible(const Index3DId& parentId, const Vector3d& tuv, int ignoreAxisBits)
{
	bool isect[8];

	performTestHexSplits(parentId, tuv, isect, ignoreAxisBits);

	int ignoreAxis;
	for (ignoreAxis = 0; ignoreAxis < 3; ignoreAxis++) {
		int mask = 1 << ignoreAxis;
		bool ignore = ignoreAxisBits & mask;
		if (ignore)
			break;
	}

	for (int splitAxis = 0; splitAxis < 3; splitAxis++) {
		bool doSplit = false;
		for (int splitAxis = 0; splitAxis < 3; splitAxis++) {
			bool doSplit = false;
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
		}

		if (doSplit) {
			MTC::vector<Index3DId> newCellIds;
			splitHexCell_2(parentId, tuv, splitAxis, newCellIds);
			for (const auto& cellId : newCellIds) {
				int ignoreAxisBits = 1 << splitAxis;
				splitHexCell_2_possible(cellId, tuv, ignoreAxisBits);
			}
			break;
		}
	}
}

void Splitter3D::splitHexCell_2_possible(const Index3DId& parentId, const Vector3d& tuv, int ignoreAxisBits)
{

}

void Splitter3D::splitHexCell_2(const Index3DId& parentId, const Vector3d& tuv, int splitAxis, MTC::vector<Index3DId>& newCellIds)
{
	Index3DId testId(2, 0, 3, 3);
#if 1 && _DEBUG
	if (testId == parentId) {
		int dbgBreak = 1;
	}
#endif
	auto splittingFaceId = createSplittingHexFace(parentId, tuv, splitAxis);

	FastBisectionSet<Index3DId> allCellFaceIds;
	cellFunc(parentId, [this, parentId, testId, &splittingFaceId, &allCellFaceIds](Polyhedron& cell) {
#if 1 && _DEBUG
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
		cell.imprintFaceEdges(splittingFaceId);
		allCellFaceIds = cell.getFaceIds();
		cell.detachFaces();
		});

	MTC::vector<MTC::vector<Vector3d>> subCells;
	makeHexPoints(parentId, tuv, splitAxis, subCells);

	for (int i = 0; i < 2; i++) {
		Index3DId cellId = makeCellFromFaces(splittingFaceId, subCells[i], allCellFaceIds, i == 1);
#if 1 && defined(_DEBUG)
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

Index3DId Splitter3D::makeCellFromFaces(const Index3DId& splittingFaceId, const MTC::vector<Vector3d>& cornerPts, FastBisectionSet<Index3DId>& allCellFaceIds, bool useAllFaces)
{
	MTC::set<Index3DId> cellFaces;

	if (useAllFaces) {
		cellFaces.insert(allCellFaceIds.begin(), allCellFaceIds.end());
	} else {
		Vector3d cellCtr(0, 0, 0);
		for (const auto& pt : cornerPts)
			cellCtr += pt;
		cellCtr /= cornerPts.size();

		Planed splittingPlane;
		faceFunc(splittingFaceId, [&splittingPlane](const Polygon& face) {
			splittingPlane = face.calPlane();
		});

		for (const auto& faceId : allCellFaceIds) {
			faceFunc(faceId, [this, &splittingPlane, &cellFaces](Polygon& face) {
				auto ctr = face.calCentroid();
				double dist = splittingPlane.distanceToPoint(ctr, false);
				if (dist > 0)
					cellFaces.insert(face.getId());
			});
		}

		for (const auto& faceId : cellFaces)
			allCellFaceIds.erase(faceId);
	}

	cellFaces.insert(splittingFaceId);

	MTC::vector<Index3DId> cornerVertIds;
	for (const auto& pt : cornerPts)
		cornerVertIds.push_back(vertId(pt));

	Polyhedron newCell(cellFaces, cornerVertIds);
	auto newCellId = getBlockPtr()->addCell(newCell, _polyhedronId);

#if 0 && defined(_DEBUG)
	cellFunc(newCellId, [this](const Polyhedron& cell) {
		if (!cell.isClosed()) {
			stringstream ss;
			ss << "D:/DarkSky/Projects/output/objs/unclosedCell_" << getBlockPtr()->getLoggerNumericCode(cell.getId()) << "_post.obj";
			getBlockPtr()->getVolume()->writeObj(ss.str(), { cell.getId() }, false, false, false);
			assert(!"Cell not closed");
		}
	});
#endif
	return newCellId;
}

bool Splitter3D::faceInsideBoundary(const Polygon& face, const MTC::vector<Vector3d>& boundingPts) const
{
	Planed pl = makePlane(boundingPts);
	if (!face.isCoplanar(pl)) {
		return false;
	}

	Splitter2D sp(pl);
	for (size_t i = 0; i < boundingPts.size(); i++) {
		size_t j = (i + 1) % boundingPts.size();
		sp.add3DEdge(boundingPts[i], boundingPts[j]);
	}

	auto& vertIds = face.getVertexIds();
	for (size_t i = 0; i < vertIds.size(); i++) {
		size_t j = (i + 1) % vertIds.size();
		const auto& pt0 = getVertexPoint(vertIds[i]);
		const auto& pt1 = getVertexPoint(vertIds[j]);
		if (!sp.contains3DEdge(pt0, pt1))
			return false;
	}

	return true;
}

Planed Splitter3D::makePlane(const MTC::vector<Vector3d>& boundingPts) const
{
	Vector3d v0 = boundingPts[0] - boundingPts[1];
	Vector3d v1 = boundingPts[2] - boundingPts[1];
	Vector3d n = v1.cross(v0);
	Planed result(boundingPts[0], n);
	return result;
}

void Splitter3D::makeHexPoints(const Index3DId& parentId, const Vector3d& tuv, int axis, MTC::vector<MTC::vector<Vector3d>>& subCells)
{
	MTC::vector<Vector3d> cornerPts;
	if (!_testRun) {
		cellFunc(parentId, [this, &cornerPts](const Polyhedron& cell) {
			auto& cornerVertIds = cell.getCanonicalVertIds();
			for (const auto& id : cornerVertIds)
				cornerPts.push_back(getVertexPoint(id));
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
	}
}

void Splitter3D::makeTestHexCells_2_hexes(const Index3DId& parentId, const Vector3d& tuv, int axis, MTC::vector<Index3DId>& newCells)
{
	const double tol = 10 * _distTol; // Sloppier than "exact" match. We just need a "good" match

#ifdef _DEBUG
	Index3DId testId(6, 0, 3, 0);
	if (parentId == testId) {
		int dbgBreak = 1;
	}
#endif
	MTC::vector<MTC::vector<Vector3d>> subCells;
	makeHexPoints(parentId, tuv, axis, subCells);
	for (int i = 0; i < 2; i++) {
		const auto& subPts = subCells[i];

		MTC::vector<Index3DId> subCorners;
		for (const auto& pt : subPts)
			subCorners.push_back(vertId(pt));

		auto newId = addHexCell(parentId, subCorners, tol);
		newCells.push_back(newId);

#if 0 && defined(_DEBUG)
		stringstream ss;
		ss << "D:/DarkSky/Projects/output/objs/new_cell_" << i << ".obj";
		getBlockPtr()->getVolume()->writeObj(ss.str().c_str(), { newId }, false, false, false);
#endif
	}
}

void Splitter3D::createSplittingHexPoints(const Index3DId& parentId, const Vector3d& tuv, int axis, MTC::vector<Vector3d>& quadPts)
{
	MTC::vector<Vector3d> cornerPts;
	cellFunc(parentId, [this, &cornerPts](const Polyhedron& cell) {
		auto& cornerVerts = cell.getCanonicalVertIds();
		for (const auto& id : cornerVerts)
			cornerPts.push_back(getVertexPoint(id));
	});

	switch (axis) {
	case 0:
		quadPts = {
			TRI_LERP(cornerPts, tuv[0], 0., 0.),
			TRI_LERP(cornerPts, tuv[0], 1., 0.),
			TRI_LERP(cornerPts, tuv[0], 1., 1.),
			TRI_LERP(cornerPts, tuv[0], 0., 1.),
		};
		break;
	case 1:
		quadPts = {
			TRI_LERP(cornerPts, 0., tuv[1], 0.),
			TRI_LERP(cornerPts, 1., tuv[1], 0.),
			TRI_LERP(cornerPts, 1., tuv[1], 1.),
			TRI_LERP(cornerPts, 0., tuv[1], 1.),
		};
		break;
	case 2:
		quadPts = {
			TRI_LERP(cornerPts, 0., 0., tuv[1]),
			TRI_LERP(cornerPts, 1., 0., tuv[1]),
			TRI_LERP(cornerPts, 1., 1., tuv[1]),
			TRI_LERP(cornerPts, 0., 1., tuv[1]),
		};
		break;
	}

}

Index3DId Splitter3D::createSplittingHexFace(const Index3DId& parentId, const Vector3d& tuv, int axis)
{
	const double tol = 10 * _distTol; // Sloppier than "exact" match. We just need a "good" match

#ifdef _DEBUG
	Index3DId testId(6, 0, 3, 0);
	if (parentId == testId) {
		int dbgBreak = 1;
	}
#endif

	MTC::vector<Vector3d> facePts;
	createSplittingHexPoints(parentId, tuv, axis, facePts);

	MTC::vector<Index3DId> faceVerts;
	for (const auto& pt : facePts)
		faceVerts.push_back(vertId(pt));

	auto newFaceId = getBlockPtr()->addPolygon(Polygon(faceVerts));

#if 0 && defined(_DEBUG)
	stringstream ss;
	ss << "D:/DarkSky/Projects/output/objs/new_cell_" << i << ".obj";
	getBlockPtr()->getVolume()->writeObj(ss.str().c_str(), { newId }, false, false, false);
#endif

	return newFaceId;
}

Index3DId Splitter3D::createScratchCell(const Index3DId& parentId)
{
	Utils::ScopedRestore restore(_pBlock);
	_pBlock = _pSrcBlock;

	FastBisectionSet<Index3DId> srcFaceIds;
	cellFunc(parentId, [&srcFaceIds](const Polyhedron& srcCell) {
		srcFaceIds = srcCell.getFaceIds();
	});

	MTC::set<Index3DId> newFaceIds;
	for (const auto& srcFaceId : srcFaceIds) {
		const auto& newFaceId = createScratchFace(srcFaceId);
		newFaceIds.insert(newFaceId);
	}

	MTC::vector<Index3DId> cornerVertIds;
	cornerVertIds.reserve(_cornerPts.size());
	for (const auto& pt : _cornerPts)
		cornerVertIds.push_back(vertId(pt));
	auto scratchCellId = _pScratchBlock->addCell(Polyhedron(newFaceIds, cornerVertIds), Index3DId());

	return scratchCellId;
}

Index3DId Splitter3D::createScratchFace(const Index3DId& srcFaceId)
{
	Utils::ScopedRestore restore(_pBlock);
	_pBlock = _pSrcBlock;

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

Index3DId Splitter3D::addHexCell(const Index3DId& parentId, const MTC::vector<Index3DId>& cubeVerts, double tol)
{
	Index3DId newCellId;

	assert(cubeVerts.size() == 8);
	MTC::vector<MTC::vector<Index3DId>> faceVertList;
	GradingOp::getCubeFaceVertIds(cubeVerts, faceVertList);
	assert(faceVertList.size() == 6);
#ifdef _DEBUG
	Index3DId testId(6, 0, 4, 0);
	if (parentId == testId) {
		int dbgBreak = 1;
	}
#endif
	MTC::set<Index3DId> cellFaceIds;
	for (const auto& faceVerts : faceVertList) {
		assert(faceVerts.size() == 4);
		MTC::set<Index3DId> newFaceIds;
		assert(_testRun);
		// Skip the expensive splitting operations
		auto newFaceId = getBlockPtr()->addPolygon(Polygon(faceVerts));
#ifdef _DEBUG
		Index3DId testId(0, 0, 0, 10);
		if (testId == newFaceId) {
			int dbgBreak = 1;
		}
		faceFunc(newFaceId, [](const Polygon& face) {
			assert(face.getCellIds().size() < 2);
		});
#endif
		cellFaceIds.insert(newFaceId);

	}

	newCellId = _pBlock->addCell(Polyhedron(cellFaceIds, cubeVerts), parentId);
#if 1 && defined(_DEBUG)
	if (testId == _polyhedronId && !_testRun) {
		getBlockPtr()->dumpPolyhedraObj({ 
			_polyhedronId,
			Index3DId(6, 0, 4, 1), 
			Index3DId(6, 0, 3, 1),
			Index3DId(6, 0, 3, 2),
			}, false, false, false);
		int dbgBreak = 1;
	}
#endif

	return newCellId;
}

void Splitter3D::replaceExistingFaces(const Index3DId& existingFaceId, const std::vector<std::vector<Vector3d>>& newFacePoints)
{
	if (!getBlockPtr()->polygonExists(existingFaceId))
		return; // This is a normal condition. The first pass should have created most of the faces for us.

#if 1 && defined(_DEBUG)
	Index3DId testId(6, 0, 3, 0);
	if (existingFaceId == testId) {
		int dbgBreak = 1;
	}
#endif // _DEBUG
	FastBisectionSet<Index3DId> cellIds;
	MTC::vector<EdgeKey> edgeKeys;
	faceFunc(existingFaceId, [&cellIds, &edgeKeys](const Polygon& existingFace) {
		cellIds = existingFace.getCellIds();
		edgeKeys = existingFace.getEdgeKeys();
	});

	for (const auto& cellId : cellIds) {
		cellFunc(cellId, [&existingFaceId](Polyhedron& cell) {
			cell.removeFace(existingFaceId);
		});
	}
	getBlockPtr()->freePolygon(existingFaceId);

	if (cellIds.empty())
		return;

	for (const auto& pts : newFacePoints) {
		vector<Index3DId> vertIds;
		for (const auto& pt : pts)
			vertIds.push_back(vertId(pt));
		auto newFaceId = getBlockPtr()->addPolygon(Polygon(vertIds));
		assert(getBlockPtr()->getBlockIdx().withinRange(newFaceId));
#ifdef _DEBUG
		Index3DId testId(0, 0, 0, 10);
		if (testId == newFaceId) {
			int dbgBreak = 1;
		}
#endif
		for (const auto& cellId : cellIds) {
			cellFunc(cellId, [&newFaceId](Polyhedron& cell) {
				cell.addFace(newFaceId);
			});
		}
	}
}

void Splitter3D::createHexCellData(const Polyhedron& parentCell)
{
	auto& cornerVertIds = parentCell.getCanonicalVertIds();
	_cornerPts.reserve(cornerVertIds.size());
	for (const auto& id : cornerVertIds)
		_cornerPts.push_back(getVertexPoint(id));

	int dbgBreak = 1;
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
