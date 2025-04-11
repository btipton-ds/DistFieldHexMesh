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

Splitter3D::Splitter3D(Block* pBlock, const Index3DId& polyhedronId, size_t splitLevel, size_t subPassNum)
	: _pBlock(pBlock)
	, _polyhedronId(polyhedronId)
	, _splitLevel(splitLevel)
	, _subPassNum(subPassNum)
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

		{
			auto& parentCell = getPolyhedron(_polyhedronId);
			Utils::Timer tmr(Utils::Timer::TT_splitAtPointInner);

			createHexCellData(parentCell);

			switch (_cornerPts.size()) {
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
		case CT_HEX:
			result = conditionalBisectionHexSplit(_polyhedronId, tuv, 0, 8);
			break;
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

bool Splitter3D::conditionalBisectionHexSplit(const Index3DId& parentId, const Vector3d& tuv, int ignoreAxisBits, int numPossibleSplits)
{
	if (Index3DId(2, 4, 7, 0) == _polyhedronId) {
		int dbgBreak = 1;
	}

	if (numPossibleSplits == 8 && doCurvatureSplit(parentId, tuv, ignoreAxisBits)) {
		return true;
	}

	bool wasSplit = false;
	int splitAxis = determineBestSplitAxis(parentId, tuv, ignoreAxisBits);

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

int Splitter3D::determineBestSplitAxis(const Index3DId& parentId, const Vector3d& tuv, int ignoreAxisBits)
{
	const auto& parentCell = getPolyhedron(parentId);

	bool doTooManyFacesSplit = parentCell.hasTooManFaces(_params);
	bool doTooNonOrthogonalSplit = parentCell.maxOrthogonalityAngleRadians() > _params.maxOrthoAngleRadians;
	bool hasIntersection = false;
	if (_subPassNum == 0)
		hasIntersection = parentCell.intersectsModel();

	bool doIntersectionSplit = hasIntersection && _splitLevel < _params.numIntersectionDivs;

	size_t minTooManyFaceCells = INT_MAX;
	double minOfMaxFinalOrthoCells = DBL_MAX;
	int minIntersections = INT_MAX;

	int bestTooManyFacesSplitAxis = -1;
	int bestTooManyOrthoSplitAxis = -1;
	int bestIntersectionSplitAxis = -1;

	for (int axis = 0; axis < 3; axis++) {
		int axisBit = 1 << axis;
		bool ignore = (ignoreAxisBits & axisBit) == axisBit;
		if (ignore)
			continue;

		auto scratchCellId = makeScratchCell(parentId);

		Utils::ScopedRestore restore1(_testRun);
		Utils::ScopedRestore restore2(_pBlock);
		_testRun = true;
		_pBlock = _pScratchBlock;

		MTC::vector<Index3DId> newCellIds;
		bisectHexCell(scratchCellId, tuv, axis, newCellIds);

		size_t totalTooManyFaces = 0;
		double maxOrtho = 0;
		int numIntersections = 0;

		for (size_t cellNum = 0; cellNum < 2; cellNum++) {
			const auto& newCell = getPolyhedron(newCellIds[cellNum]);

			if (doTooManyFacesSplit || doTooNonOrthogonalSplit) {
				if (newCell.hasTooManFaces(_params)) {
					totalTooManyFaces++;
				}

				double ortho = newCell.maxOrthogonalityAngleRadians();
				if (ortho > maxOrtho) {
					maxOrtho = ortho;
				}
			}
			else if (_subPassNum == 0 && newCell.intersectsModel()) {
				numIntersections++;
			}
		}

		if (doTooManyFacesSplit && totalTooManyFaces < minTooManyFaceCells) {
			minTooManyFaceCells = totalTooManyFaces;
			bestTooManyFacesSplitAxis = axis;
		}

		if (doTooNonOrthogonalSplit && maxOrtho < minOfMaxFinalOrthoCells) {
			minOfMaxFinalOrthoCells = maxOrtho;
			bestTooManyOrthoSplitAxis = axis;
		}

		if (_subPassNum == 0) {
			if (numIntersections < minIntersections) {
				// If neither sub cell intersects, we do not need to split this one
				minIntersections = numIntersections;
				bestIntersectionSplitAxis = axis;
			}
		}

		reset(newCellIds);
	}

	int splitAxis = -1;

	if (bestTooManyOrthoSplitAxis != -1)
		splitAxis = bestTooManyOrthoSplitAxis;
	else if (bestTooManyFacesSplitAxis != -1)
		splitAxis = bestTooManyFacesSplitAxis; 
	else if (_subPassNum == 0) {
		if (_splitLevel < _params.numIntersectionDivs) {
			// If we're doing intersection splits, split there's any intersection
			splitAxis = bestIntersectionSplitAxis;
		} else if (_splitLevel < _params.numCurvatureDivs && minIntersections == 1) {
			// If we're doing curvature splits, only split if one side is empty
			splitAxis = bestIntersectionSplitAxis;
		}
	}

	return splitAxis;
}

bool Splitter3D::doCurvatureSplit(const Index3DId& parentId, const Vector3d& tuv, int& ignoreAxisBits)
{
	if (_splitLevel < _params.numIntersectionDivs || _splitLevel >= _params.numCurvatureDivs)
		return false;

	const auto& parentCell = getPolyhedron(parentId);
	if (!parentCell.hasTooMuchCurvature(_params) /* hasTooMuchCurvature tests intersectsModel */)
		return false;

	// Determine all required curvatures splits before doing any other splitting.
	bool axisNeedsSplit[3];
	for (int axis = 0; axis < 3; axis++) {
		double curvature;
		double edgeLenOverChordLen;
		axisNeedsSplit[axis] = parentCell.needCurvatureSplit(_params, curvature, edgeLenOverChordLen, axis);
	}

#if 1 && defined(_DEBUG)
	static mutex mut;
	lock_guard lg(mut);
#endif // _DEBUG

	bool needCurvatureSplit[] = { false, false, false };
	for (int axis = 0; axis < 3; axis++) {
		int orthoAxis0 = (axis + 1) % 3;
		int orthoAxis1 = (axis + 2) % 3;
		needCurvatureSplit[axis] = axisNeedsSplit[orthoAxis0] || axisNeedsSplit[orthoAxis1];
	}

	bool wasSplit = false;
	vector<Index3DId> newCellIds;
	newCellIds.push_back(parentId);
	for (int axis = 0; axis < 3; axis++) {
		if (needCurvatureSplit[axis]) {
			int axisBit = 1 << axis;
			ignoreAxisBits |= axisBit;

			vector<Index3DId> tmp(newCellIds);
			newCellIds.clear();
			for (const auto& id : tmp) {
				bisectHexCell(id, tuv, axis, newCellIds);
			}
		}

		wasSplit = true;
	}

	if (newCellIds.size() == 4) {
		for (const auto& cellId : newCellIds) {
			if (conditionalBisectionHexSplit(cellId, tuv, ignoreAxisBits, 2)) {
				wasSplit = true;
			}
		}
	}
	return wasSplit;
}

void Splitter3D::bisectHexCell(const Index3DId& parentId, const Vector3d& tuv, int splitAxis, MTC::vector<Index3DId>& newCellIds)
{
	Index3DId testId(2, 4, 7, 0);
#if DEBUG_BREAKS && defined(_DEBUG)
	if (testId == parentId) {
		int dbgBreak = 1;
	}
#endif
	auto& parentCell = getPolyhedron(parentId);

	MTC::vector<MTC::vector<Vector3d>> subCells;
	MTC::vector<Vector3d> splittingFacePts;
	parentCell.makeHexCellPoints(splitAxis, tuv, subCells, splittingFacePts);
	MTC::vector<Index3DId> splittingFaceVertIds;
	for (const auto& pt : splittingFacePts)
		splittingFaceVertIds.push_back(vertId(pt));
	auto splittingFaceId = getBlockPtr()->addPolygon(Polygon(splittingFaceVertIds));

#if DEBUG_BREAKS && defined(_DEBUG)
	if (testId == _polyhedronId) {
		stringstream ss;
		ss << "splittingFace_" << getBlockPtr()->getLoggerNumericCode(splittingFaceId);
		MTC::vector<Index3DId> ids;
		ids.push_back(splittingFaceId);
		getBlockPtr()->dumpPolygonObj(ss.str(), ids);
	}
#endif

	auto& splittingFace = getPolygon(splittingFaceId);
	auto n = splittingFace.calUnitNormal();

	const auto& faceIds = parentCell.getFaceIds();

	splittingFace.imprintFaces(faceIds);

	imprintSplittingFace(parentId, splittingFaceId);

	set<Index3DId> allCellFaceIds(faceIds.begin(), faceIds.end());
	parentCell.detachFaces();
	for (int i = 0; i < 2; i++) {
		Index3DId cellId = makeCellFromHexFaces(parentId, splittingFaceId, subCells[i], allCellFaceIds, i == 1);
		newCellIds.push_back(cellId);
	}

	_createdCellIds.erase(parentId);
	getBlockPtr()->freePolyhedron(parentId);
}

void Splitter3D::finalizeCreatedCells()
{
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
					createdCell._pSearchTree = _pSearchSourceTree;
					if (createdCell._pSearchTree /* && createdCell._pSearchTree->numInTree() > 128*/) {
						// Splitting small trees takes time and memory, so only reduce larger ones
						createdCell._pSearchTree = createdCell._pSearchTree->getSubTree(subBbox);
					}
				}
				else
					createdCell._pSearchTree = createdCell.getOurBlockPtr()->getModel().getSubTree(subBbox);
			}
		}
		createdCell.setSplitLevel(_splitLevel + 1);
	}
}

void Splitter3D::imprintSplittingFace(const Index3DId& parentId, const Index3DId& splittingFaceId)
{
	if (!_testRun) {
		int dbgBreak = 1;
	}
	const auto& parentCell = getPolyhedron(parentId);
	const auto cellEdges = parentCell.getEdgeKeys(false);

	set<Index3DId> faceIds;
	auto& splittingFace = getPolygon(splittingFaceId);
	MTC::vector<EdgeKey> splittingEdges = splittingFace.getEdgeKeys();
	Vector3d splitPlaneNormal = splittingFace.calUnitNormal();

	for (auto& edge : cellEdges) {
		FastBisectionSet<Index3DId> edgeFaces;
		edgeFunc(edge, [this, &edgeFaces](const Edge& edge) {
			edgeFaces = edge.getFaceIds();
		});
		for (auto& faceId : edgeFaces) {
			auto& face = getPolygon(faceId);
			auto faceNormal = face.calUnitNormal();
			auto cp = splitPlaneNormal.cross(faceNormal).norm();
			if (cp > Tolerance::paramTol()) {
				faceIds.insert(faceId);
			}
		}
	}

	for (auto& faceId : faceIds) {
		auto& face = getPolygon(faceId);
		for (auto& ek : splittingEdges) {
			face.imprintEdge(ek);
		}
	}

	for (const auto& faceId : faceIds) {
		if (Index3DId(3, 0, 3, 13) == faceId) {
			int dbgBreak = 1;
		}
		assert(getBlockPtr()->polygonExists(faceId));
		MTC::vector<MTC::vector<Vector3d>> splitFacePoints;
		FastBisectionSet<Index3DId> cellIds;

		auto& face = getPolygon(faceId);

		if (Index3DId(5, 2, 7, 0) == _polyhedronId && Index3DId(0, 0, 0, 9) == face.getId()) {
			int dbgBreak = 1;
		}
		Splitter2D sp(face.calPlane());
		cellIds = face.getCellIds();
		face.iterateEdges([this, &sp](const Edge& edge)->bool {
			const auto& pt0 = getVertexPoint(edge[0]);
			const auto& pt1 = getVertexPoint(edge[1]);
			sp.add3DEdge(pt0, pt1);
			return true;
			});

		for (const auto& ek : splittingEdges) {
			const auto& pt0 = getVertexPoint(ek[0]);
			const auto& pt1 = getVertexPoint(ek[1]);
			bool cp0 = face.isCoplanar(pt0);
			bool cp1 = face.isCoplanar(pt1);
			if (cp0 && cp1) {
				sp.add3DEdge(pt0, pt1);
			}
		}
		sp.getFacePoints(splitFacePoints);

		if (splitFacePoints.size() < 2)
			continue;

		set<Index3DId> newFaceIds;
		for (const auto& facePts : splitFacePoints) {
			MTC::vector<Index3DId> vertIds;
			for (const auto& pt : facePts) {
				vertIds.push_back(getBlockPtr()->addVertex(pt));
			}
			auto imprintFaceId = getBlockPtr()->addPolygon(Polygon(vertIds));
			newFaceIds.insert(imprintFaceId);
		}

		if (!newFaceIds.empty()) {
			for (const auto& cellId : cellIds) {
				// This also changes our faces cellIds to point to this
				auto& cell = getPolyhedron(cellId);
				cell.removeFace(faceId);
				for (const auto& imprintFaceId : newFaceIds) {
					cell.addFace(imprintFaceId);
				}

				if (!cell.isClosed()) {
					{
						stringstream ss;
						ss << "D:/DarkSky/Projects/output/objs/imprintFaceEdges_open_cell_" << getBlockPtr()->getLoggerNumericCode(cell.getId()) << "_splittingFace.obj";
						MTC::vector<Vector3d> facePts;
						faceFunc(splittingFaceId, [this, &facePts](const Polygon& splittingFace) {
							auto& fv = splittingFace.getVertexIds();
							for (auto& id : fv) {
								facePts.push_back(getVertexPoint(id));
							}
							});
						getBlockPtr()->getVolume()->writeObj(ss.str(), { facePts }, true);
					}
					{
						stringstream ss;
						ss << "D:/DarkSky/Projects/output/objs/imprintFaceEdges_open_cell_" << getBlockPtr()->getLoggerNumericCode(cell.getId()) << ".obj";
						getBlockPtr()->getVolume()->writeObj(ss.str(), { cell.getId() }, false, false, false);
					}
					assert(!"imprintFaceEdges open cell");
				}
			}

			getBlockPtr()->freePolygon(faceId);
		}
	}
}

void Splitter3D::addFaceToLocalEdgeSet(map<EdgeKey, set<Index3DId>>& localEdgeSet, const Index3DId& faceId) const
{
	auto& face = getPolygon(faceId);
	auto& vertIds = face.getVertexIds();
	for (size_t i = 0; i < vertIds.size(); i++) {
		size_t j = (i + 1) % vertIds.size();
		EdgeKey e(vertIds[i], vertIds[j]);
		auto iter = localEdgeSet.find(e);
		if (iter == localEdgeSet.end())
			iter = localEdgeSet.insert(make_pair(e, set<Index3DId>())).first;
		iter->second.insert(faceId);
	}
}

void Splitter3D::removeFacefromLocalEdgeSet(map<EdgeKey, set<Index3DId>>& localEdgeSet, const Index3DId& faceId) const
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

Index3DId Splitter3D::findConnectedFaceId(const map<EdgeKey, set<Index3DId>>& localEdgeSet, const Index3DId& faceId) const
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
	set<Index3DId>& allCellFaceIds, bool useAllFaces)
{
#ifdef _DEBUG
	if (Index3DId(2, 4, 7, 0) == _polyhedronId) {
		int dbgBreak = 1;
	}
#endif // _DEBUG

	// Failing on skewed cells. Need to use a full topological assembly, not the current normal based one.
	MTC::set<Index3DId> cellFaces;

	if (!useAllFaces) {
		map<EdgeKey, set<Index3DId>> localEdgeSet;

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
	if (_testRun) {
		const auto& parentCell = getBlockPtr()->getPolyhedron(parentId);
		auto& newCell = getBlockPtr()->getPolyhedron(newCellId);
		newCell._hasSetSearchTree = parentCell._hasSetSearchTree;
		newCell._pSearchTree = parentCell._pSearchTree;
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

void Splitter3D::verifyLocalEdgeSet(const map<EdgeKey, set<Index3DId>>& localEdgeSet, const Index3DId& splittingFaceId) const
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

		{
			stringstream ss;
			ss << "D:/DarkSky/Projects/output/objs/_" << getBlockPtr()->getLoggerNumericCode(_polyhedronId) << " badlyImprintedEdges_.obj";
			pVol->writeObj(ss.str(), edgePts, false);
		}
		assert(!"spltting produced laminar edge(s)");
		string msg = string(__FILE__) + ":" + to_string(__LINE__) + string(" spltting produced laminar edge(s)");
		throw runtime_error(msg);
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
		_hasSetSearchTree = targetCell._hasSetSearchTree;
		if (targetCell._hasSetSearchTree)
			_pSearchSourceTree = targetCell._pSearchTree;
		else
			_pSearchSourceTree = targetCell.getBlockPtr()->getModelSearchTree();
	} else {
		_hasSetSearchTree = true;
		_pSearchSourceTree = nullptr;
	}

	_splitLevel = targetCell.getSplitLevel();
	_cornerVertIds = targetCell.getCanonicalVertIds();
	_cornerPts.reserve(_cornerVertIds.size());
	for (const auto& id : _cornerVertIds)
		_cornerPts.push_back(getVertexPoint(id));
	int dbgBreak = 1;
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
