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
#include <splitter.h>
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

thread_local VolumePtr Splitter::_pScratchVol;

namespace
{
	static std::atomic<size_t> numSplits2 = 0;
	static std::atomic<size_t> numSplits4 = 0;
	static std::atomic<size_t> numSplitsComplex8 = 0;
}

void Splitter::reset()
{
	_partingFaceIds.clear();
	_pScratchBlock->clear();
}

void Splitter::dumpSplitStats()
{
	cout << "Num splits 2: " << numSplits2 << "\n";
	cout << "Num splits 4: " << numSplits4 << "\n";
	cout << "Num splits complex 8: " << numSplitsComplex8 << "\n";

	numSplits2 = 0;
	numSplits4 = 0;
	numSplitsComplex8 = 0;
}

void Splitter::clearThreadLocal()
{
	_pScratchVol = nullptr;
}

Splitter::Splitter(Block* pBlock, const Index3DId& polyhedronId, vector<Index3DId>& localTouched)
	: _pBlock(pBlock)
	, _pSrcBlock(pBlock)
	, _polyhedronId(polyhedronId)
	, _localTouched(localTouched)
	, _params(pBlock->getSplitParams())
{
	if (!_pScratchVol)
		_pScratchVol = _pBlock->getVolume()->createScratchVolume();
	_pScratchBlock = _pScratchVol->getBlockPtr(Index3D(0, 0, 0));
}

Splitter::~Splitter()
{
	if (_pScratchVol)
		_pScratchVol->clearEntries();
}

bool Splitter::splitAtCenter()
{
	if (!_pBlock->polyhedronExists(_polyhedronId))
		return false;

	bool result = false;
	CellType cellType = CT_UNKNOWN;

	cellFunc(_polyhedronId, [this, &cellType](Polyhedron& parentCell) {
		// DO NOT USE the parentCell centroid! It is at a different location than the parametric center. That results in faces which do 
		// match with the neighbor cells's faces.
#if 0 && defined(_DEBUG)
		// Now split the parentCell
		Index3DId testId(6, 4, 5, 0);
		if (testId == _polyhedronId) {
			int dbgBreak = 1;
			_pBlock->dumpPolyhedraObj({ _polyhedronId }, false, false, false);
		}
#endif

		Utils::Timer tmr(Utils::Timer::TT_splitAtPointInner);

		parentCell.detachFaces();
		if (parentCell.classify(_cornerPts) == 8) {
			cellType = CT_HEX;
		}
	});

	Vector3d tuv(0.5, 0.5, 0.5);
	switch (cellType) {
	case CT_HEX:
		splitHexCell(tuv);
		result = true;
		break;
	default:
		result = false;
	}

	if (result) {
		fixNewCellFaceSplits();

		// It's been completely split, so this parentCell can be removed
		_pBlock->freePolyhedron(_polyhedronId);
	}
	return result;
}

inline Index3DId Splitter::vertId(const Vector3d& pt)
{
	return _pBlock->getVertexIdOfPoint(pt);
}

inline const Vector3d& Splitter::vertexPoint(const  Index3DId& id) const
{
	return _pBlock->getVertexPoint(id);
}

void Splitter::fixNewCellFaceSplits()
{
	if (_partingFaceIds.empty())
		return;

	MTC::set<EdgeKey> newEdgeKeys;
	for (const auto& partingFaceId : _partingFaceIds) {
		faceFunc(partingFaceId, [&newEdgeKeys](const Polygon& face) {
			auto t = face.getEdgeKeys();
			newEdgeKeys.insert(t.begin(), t.end());
		});
	}
	cellFunc(_polyhedronId, [this, &newEdgeKeys](const Polyhedron& cell) {
		const auto& oldFaceIds = cell.getFaceIds();
		for (const auto& oldFaceId : oldFaceIds) {
			if (_partingFaceIds.contains(oldFaceId))
				return;
			for (const auto& newEdgeKey : newEdgeKeys) {
				splitFaceWithEdge(oldFaceId, newEdgeKey);
			}
		}
	});

	_partingFaceIds.clear();
}

void Splitter::splitFaceWithEdge(const Index3DId& oldFaceId, const EdgeKey& newEdgeKey)
{
	MTC::vector<Index3DId> newSplitIds;
	splitFaceWithEdgeInner(oldFaceId, newEdgeKey, newSplitIds);
	if (!newSplitIds.empty()) {
		faceFunc(oldFaceId, [&newSplitIds](Polygon& face) {
			face.setSplitFaceIds(newSplitIds);
		});
	}
}

void Splitter::splitFaceWithEdgeInner(const Index3DId& oldFaceId, const EdgeKey& newEdgeKey, MTC::vector<Index3DId>& newSplitIds)
{
	faceFunc(oldFaceId, [this, &newEdgeKey, &newSplitIds](const Polygon& oldFace) {
		const auto& splitIds = oldFace.getSplitIds();
		if (splitIds.empty()) {
			const double tol = Tolerance::sameDistTol();
			if (oldFace.usesEdge(newEdgeKey) || !oldFace.isCoplanar(newEdgeKey))
				return;

			Vector3d pt0, pt1;
			edgeFunc(newEdgeKey, [this, &pt0, &pt1](const Edge& newEdge) {
				pt0 = vertexPoint(newEdge.getVertex(0));
				pt1 = vertexPoint(newEdge.getVertex(1));
			});

			Vector3d v0 = oldFace.calUnitNormal();
			Vector3d v1 = pt1 - pt0;
			Vector3d n = v1.cross(v0);
			Planed pl(pt0, n);
			RayHitd hit;

			const auto& vertIds = oldFace.getVertexIds();

			vector<Index3DId> newVertIds;
			vector<size_t> insertIndices;
			for (size_t i = 0; i < vertIds.size(); i++) {
				size_t j = (i + 1) % vertIds.size();
				newVertIds.push_back(vertIds[i]);

				Vector3d fpt0 = vertexPoint(vertIds[i]);
				Vector3d fpt1 = vertexPoint(vertIds[j]);
				LineSegmentd seg(fpt0, fpt1);
				if (pl.intersectLineSegment(seg, hit, tol)) {
					double t = hit.dist / seg.calLength();
					auto id = _pBlock->addVertex(hit.hitPt);
					insertIndices.push_back(newVertIds.size());
					newVertIds.push_back(id);
				}
			}

			assert(insertIndices.size() == 2);

			MTC::vector<Index3DId> faceVerts0, faceVerts1;

			size_t idx = insertIndices[0];
			while (idx != insertIndices[1]) {
				faceVerts0.push_back(newVertIds[idx]);
				idx = (idx + 1) % newVertIds.size();
			}
			faceVerts0.push_back(newVertIds[idx]);

			while (idx != insertIndices[0]) {
				faceVerts1.push_back(newVertIds[idx]);
				idx = (idx + 1) % newVertIds.size();
			}
			faceVerts1.push_back(newVertIds[idx]);

			newSplitIds.push_back(_pBlock->addFace(Polygon(faceVerts0)));
			newSplitIds.push_back(_pBlock->addFace(Polygon(faceVerts1)));

			int dbgBreak = 1;
		} else {
			for (const auto& splitId : splitIds) {
				splitFaceWithEdgeInner(splitId, newEdgeKey, newSplitIds);
			}
			int dbgBreak = 1;
		}
	});
}

void Splitter::splitHexCell(const Vector3d& tuv)
{
#if 0
	splitHexCell8(_polyhedronId, tuv);
#else
	int intersects[] = { 0, 0, 0 };
	for (int i = 0; i < 3; i++)
	{
		Utils::ScopedRestore restore(_pBlock);
		const auto& scratchCellId = createScratchCell(false);
		_pBlock = _pScratchBlock;
		splitHexCell2(scratchCellId, tuv, i);
		_pScratchBlock->iteratePolyhedraInOrder([&scratchCellId , &intersects, i](const Index3DId& cellId, const Polyhedron& cell) {
			if (cell.getId() != scratchCellId && cell.intersectsModel())
				intersects[i]++;
			});
		reset();
	}

	if (intersects[0] == 0) {
		assert((intersects[1] == 0) && (intersects[2] == 0));
		numSplitsComplex8++;
//		splitHexCell8(_polyhedronId, tuv);
	} else if ((intersects[0] == 2) && (intersects[1] == 2) && (intersects[2] == 2)) {
//		splitHexCell8(_polyhedronId, tuv);

	} else if ((intersects[0] == 1) && (intersects[1] == 2) && (intersects[2] == 2)) {
		numSplits2++;
		splitHexCell2(_polyhedronId, tuv, 0);
	} else if ((intersects[0] == 2) && (intersects[1] == 1) && (intersects[2] == 2)) {
		numSplits2++;
		splitHexCell2(_polyhedronId, tuv, 1);
	} else if ((intersects[0] == 2) && (intersects[1] == 2) && (intersects[2] == 1)) {
		numSplits2++;
		splitHexCell2(_polyhedronId, tuv, 2);

	} else if ((intersects[0] == 2) && (intersects[1] == 1) && (intersects[2] == 1)) {
		numSplits4++;
		splitHexCell4(_polyhedronId, tuv, 0);
	} else if ((intersects[0] == 1) && (intersects[1] == 2) && (intersects[2] == 1)) {
		numSplits4++;
		splitHexCell4(_polyhedronId, tuv, 1);
	} else if ((intersects[0] == 1) && (intersects[1] == 1) && (intersects[2] == 2)) {
		numSplits4++;
		splitHexCell4(_polyhedronId, tuv, 2);
	} else {
//		splitHexCell8(_polyhedronId, tuv);
	}
#endif
}

void Splitter::splitHexCell8(const Index3DId& parentId, const Vector3d& tuv)
{
	const double tol = 10 * Tolerance::sameDistTol(); // Sloppier than "exact" match. We just need a "good" match
	cellFunc(parentId, [this, &tuv, &tol](Polyhedron& parentCell) {
		createHexCellData(parentCell);

		for (const auto& facePts : _cellFacePoints) {
			// This aligns the disordered faceIds with their cube face points
			const auto& faceId = findSourceFaceId(parentCell.getId(), facePts, tol);
			assert(faceId.isValid());

			conditionalSplitQuadFaceAtParam(faceId, facePts, 0.5, 0.5);
		}
#if _DEBUG
		for (const auto& faceId : parentCell.getFaceIds()) {
			_pBlock->faceFunc(faceId, [](const Polygon& face) {
				assert(face.isSplit());
				});
		}
#endif

		parentCell.detachFaces(); // This parentCell is about to be deleted, so detach it from all faces using it BEFORE we start attaching new ones
	});

	for (int i = 0; i < 2; i++) {
		double t0 = (i == 0) ? 0 : tuv[0];
		double t1 = (i == 0) ? tuv[0] : 1;
		for (int j = 0; j < 2; j++) {
			double u0 = (j == 0) ? 0 : tuv[1];
			double u1 = (j == 0) ? tuv[1] : 1;
			for (int k = 0; k < 2; k++) {
				double v0 = (k == 0) ? 0 : tuv[2];
				double v1 = (k == 0) ? tuv[2] : 1;

				MTC::vector<Vector3d> subCorners = {
					TRI_LERP(_cornerPts, t0, u0, v0),
					TRI_LERP(_cornerPts, t1, u0, v0),
					TRI_LERP(_cornerPts, t1, u1, v0),
					TRI_LERP(_cornerPts, t0, u1, v0),

					TRI_LERP(_cornerPts, t0, u0, v1),
					TRI_LERP(_cornerPts, t1, u0, v1),
					TRI_LERP(_cornerPts, t1, u1, v1),
					TRI_LERP(_cornerPts, t0, u1, v1),
				};

				addHexCell(parentId, subCorners, tol);
			}
		}
	}
}

void Splitter::splitHexCell2(const Index3DId& parentId, const Vector3d& tuv, int axis)
{
#if 0
	splitHexCell8(parentId, tuv);
#else
	const double tol = 10 * Tolerance::sameDistTol(); // Sloppier than "exact" match. We just need a "good" match

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
			u0 = (i == 0) ? 0 : tuv[0];
			u1 = (i == 0) ? tuv[0] : 1;
			break;
		case 2:
			v0 = (i == 0) ? 0 : tuv[0];
			v1 = (i == 0) ? tuv[0] : 1;
			break;
		}
		MTC::vector<Vector3d> subCorners = {
			TRI_LERP(_cornerPts, t0, u0, v0),
			TRI_LERP(_cornerPts, t1, u0, v0),
			TRI_LERP(_cornerPts, t1, u1, v0),
			TRI_LERP(_cornerPts, t0, u1, v0),

			TRI_LERP(_cornerPts, t0, u0, v1),
			TRI_LERP(_cornerPts, t1, u0, v1),
			TRI_LERP(_cornerPts, t1, u1, v1),
			TRI_LERP(_cornerPts, t0, u1, v1),
		};

		addHexCell(parentId, subCorners, tol);

		if (i == 0) {
			switch (axis) {
			case 0:
				addPartingFace(subCorners, { 1, 2, 6, 5 });
				break;
			case 1:
				addPartingFace(subCorners, { 3, 2, 6, 7 });
				break;
			case 2:
				addPartingFace(subCorners, { 4, 5, 6, 7 });
				break;
			}
		}
	}
#endif
}

void Splitter::addPartingFace(const MTC::vector<Vector3d>& corners, const MTC::vector<size_t>& indices)
{
	MTC::vector<Index3DId> faceIds;
	for (size_t idx : indices)
		faceIds.push_back(vertId(corners[idx]));

	const auto& id = _pBlock->findFace(Polygon(faceIds));
	assert(id.isValid());
	_partingFaceIds.insert(id);

}

void Splitter::splitHexCell4(const Index3DId& parentId, const Vector3d& tuv, int axis)
{
#if 0
	splitHexCell8(parentId, tuv);
#else
	const double tol = 10 * Tolerance::sameDistTol(); // Sloppier than "exact" match. We just need a "good" match

	for (int i = 0; i < 2; i++) {
		for (int j = 0; j < 2; j++) {
			double t0 = 0, t1 = 1;
			double u0 = 0, u1 = 1;
			double v0 = 0, v1 = 1;

			switch (axis) {
			case 0:
				u0 = (i == 0) ? 0 : tuv[1];
				u1 = (i == 0) ? tuv[1] : 1;
				v0 = (j == 0) ? 0 : tuv[2];
				v1 = (j == 0) ? tuv[2] : 1;
				break;
			case 1:
				t0 = (i == 0) ? 0 : tuv[0];
				t1 = (i == 0) ? tuv[0] : 1;
				v0 = (j == 0) ? 0 : tuv[1];
				v1 = (j == 0) ? tuv[1] : 1;
				break;
			case 2:
				t0 = (i == 0) ? 0 : tuv[0];
				t1 = (i == 0) ? tuv[0] : 1;
				u0 = (j == 0) ? 0 : tuv[1];
				u1 = (j == 0) ? tuv[1] : 1;
				break;
			}
			MTC::vector<Vector3d> subCorners = {
				TRI_LERP(_cornerPts, t0, u0, v0),
				TRI_LERP(_cornerPts, t1, u0, v0),
				TRI_LERP(_cornerPts, t1, u1, v0),
				TRI_LERP(_cornerPts, t0, u1, v0),

				TRI_LERP(_cornerPts, t0, u0, v1),
				TRI_LERP(_cornerPts, t1, u0, v1),
				TRI_LERP(_cornerPts, t1, u1, v1),
				TRI_LERP(_cornerPts, t0, u1, v1),
			};

			addHexCell(parentId, subCorners, tol);


		}
	}
#endif
}

Index3DId Splitter::createScratchCell(bool includeSplits)
{
	Utils::ScopedRestore restore(_pBlock);
	_pBlock = _pSrcBlock;

	FastBisectionSet<Index3DId> srcFaceIds;
	cellFunc(_polyhedronId, [&srcFaceIds](const Polyhedron& srcCell) {
		srcFaceIds = srcCell.getFaceIds();
	});

	MTC::set<Index3DId> newFaceIds;
	for (const auto& srcFaceId : srcFaceIds) {
		const auto& newFaceId = createScratchFace(srcFaceId, includeSplits);
		newFaceIds.insert(newFaceId);
	}

	auto scratchCellId = _pScratchBlock->addCell(Polyhedron(newFaceIds), Index3DId());

	return scratchCellId;
}

Index3DId Splitter::createScratchFace(const Index3DId& srcFaceId, bool includeSplits)
{
	Utils::ScopedRestore restore(_pBlock);
	_pBlock = _pSrcBlock;
	Index3DId newFaceId;
	faceFunc(srcFaceId, [this, includeSplits, &newFaceId](const Polygon& srcFace) {
		const auto& srcVertIds = srcFace.getVertexIds();
		MTC::vector<Index3DId> newVertIds;
		for (const auto& srcVertId : srcVertIds) {
			const auto& pt = _pBlock->getVertexPoint(srcVertId);
			const auto& newVertId = _pScratchBlock->getVertexIdOfPoint(pt);
			newVertIds.push_back(newVertId);
		}

		newFaceId = _pScratchBlock->addFace(Polygon(newVertIds));

		if (includeSplits) {
			const auto& srcSplitIds = srcFace.getSplitIds();
			faceFunc(newFaceId, [this, includeSplits, &srcSplitIds](Polygon& newFace) {
				MTC::vector<Index3DId> newSplitIds;
				for (const auto& srcSplitId : srcSplitIds) {
					const auto& newSplitId = createScratchFace(srcSplitId, includeSplits);
					newSplitIds.push_back(newSplitId);
				}

				if (!newSplitIds.empty()) {
					newFace.setSplitFaceIds(newSplitIds);
				}
			});
		}
	});

	return newFaceId;
}

void Splitter::addHexCell(const Index3DId& parentId, const std::vector<Vector3d>& cubePts, double tol)
{
	assert(cubePts.size() == 8);
	std::vector<std::vector<Vector3d>> facePtList;
	GradingOp::getCubeFacePoints(cubePts, facePtList);
	assert(facePtList.size() == 6);

	MTC::set<Index3DId> cellFaceIds;
	for (const auto& facePts : facePtList) {
		assert(facePts.size() == 4);
		Index3DId id = findSourceFaceId(parentId, facePts, tol);
		if (!id.isValid()) {
			// This should only happen for new interior faces
			vector<Index3DId> faceVertIds;
			for (const auto& pt : facePts) {
				faceVertIds.push_back(vertId(pt));
			}
			id = _pBlock->addFace(Polygon(faceVertIds));
		}
		cellFaceIds.insert(id);
	}
	Index3DId newCellId = _pBlock->addCell(Polyhedron(cellFaceIds), parentId);

	cellFunc(newCellId, [this](Polyhedron& newCell) {
		assert(newCell.getNumFaces(true) <= _params.maxCellFaces);
	});
}

Index3DId Splitter::findSourceFaceId(const Index3DId& parentId, const std::vector<Vector3d>& facePts, double tol) const
{
	FastBisectionSet<Index3DId> faceIds;
	cellFunc(parentId, [&faceIds](const Polyhedron& parentCell) {
		faceIds = parentCell.getFaceIds();
	});
	double minErr = DBL_MAX;
	Index3DId result;
	for (const auto& faceId : faceIds) {
		findSourceFaceId_inner(faceId, facePts, minErr, result, tol);
		if (minErr < tol)
			break;
	}
	if (minErr < tol)
		return result;
	return Index3DId();
}

void Splitter::findSourceFaceId_inner(const Index3DId& faceId, const std::vector<Vector3d>& facePts, double& minErr, Index3DId& result, double tol) const
{
	faceFunc(faceId, [this, &minErr, &facePts, &result, tol](const Polygon& face) {
		// Check if the unsplit face is the best match
		auto err = face.calVertexError(facePts);
		if (err < minErr) {
			minErr = err;
			result = face.getId();
			if (minErr < tol)
				return;
		}

		const auto& splitIds = face.getSplitIds();
		for (const auto& splitId : splitIds) {
			findSourceFaceId_inner(splitId, facePts, minErr, result, tol);
			if (minErr < tol)
				return;
		}
	});

}

void Splitter::createHexCellData(const Polyhedron& parentCell)
{
	_cellFacePoints;
	GradingOp::getCubeFacePoints(_cornerPts, _cellFacePoints);

	_cellFaceVertIds.resize(_cellFacePoints.size());
	for (size_t i = 0; i < _cellFacePoints.size(); i++) {
		const auto& facePts = _cellFacePoints[i];
		auto& faceVerts = _cellFaceVertIds[i];
		faceVerts.resize(facePts.size());
		for (size_t j = 0; j < facePts.size(); j++) {
			faceVerts[j] = vertId(facePts[j]);
		}
	}

	_numSplitFaces = 0;
	const auto& faceIds = parentCell.getFaceIds();
	for (const auto& faceId : faceIds) {
		parentCell.faceFunc(faceId, [this](const Polygon& face) {
			if (face.getSplitIds().size() > 1)
				_numSplitFaces++;
		});
	}
}

void Splitter::conditionalSplitQuadFaceAtParam(const Index3DId& faceId, const std::vector<Vector3d>& facePts, double t, double u)
{
	faceFunc(faceId, [this, &facePts, t, u](Polygon& oldFace) {
		if (!oldFace.isSplit()) {
			const auto& id = oldFace.getId();
			const auto& adjCellId = oldFace.getAdjacentCellId(_polyhedronId);
			if (_pBlock->getBlockIdx() == adjCellId)
				_localTouched.push_back(adjCellId);
			else
				_pBlock->addToTouchedCellList(adjCellId);

			MTC::vector<Index3DId> splitIds;
			for (int i = 0; i < 2; i++) {
				double t0 = (i == 0) ? 0 : t;
				double t1 = (i == 0) ? t : 1;
				for (int j = 0; j < 2; j++) {
					double u0 = (j == 0) ? 0 : u;
					double u1 = (j == 0) ? u : 1;

					vector<Index3DId> subFaceVertIds = {
						vertId(BI_LERP<double>(facePts, t0, u0)),
						vertId(BI_LERP<double>(facePts, t1, u0)),
						vertId(BI_LERP<double>(facePts, t1, u1)),
						vertId(BI_LERP<double>(facePts, t0, u1)),
					};

					Polygon face(subFaceVertIds);
					auto newFaceId = _pBlock->addFace(face);
					splitIds.push_back(newFaceId);
				}
				
			}

			oldFace.setSplitFaceIds(splitIds);
		}
	});
}

void Splitter::calHexCellFaceTU(int i, const Vector3d& tuv, double& t, double& u) {
	switch (i) {
	case 0: // bottom
	case 1: // top
		t = tuv[0];
		u = tuv[1];
		break;

	case 2: // back
	case 3: // front
		t = tuv[1];
		u = tuv[2];
		break;

	case 4: // left
	case 5: // right
		t = tuv[0];
		u = tuv[2];
		break;
	}
}

//LAMBDA_CLIENT_IMPLS(Splitter)
void Splitter::vertexFunc(const Index3DId& id, const std::function<void(const Vertex& obj)>& func) const {
	const auto p = getBlockPtr(); 
	p->vertexFunc(id, func);
} 

void Splitter::vertexFunc(const Index3DId& id, const std::function<void(Vertex& obj)>& func) {
	auto p = getBlockPtr(); 
	p->vertexFunc(id, func);
} 

void Splitter::faceFunc(const Index3DId& id, const std::function<void(const Polygon& obj)>& func) const {
	const auto p = getBlockPtr(); 
	p->faceFunc(id, func);
} 

void Splitter::faceFunc(const Index3DId& id, const std::function<void(Polygon& obj)>& func) {
	auto p = getBlockPtr(); 
	p->faceFunc(id, func);
} 

void Splitter::cellFunc(const Index3DId& id, const std::function<void(const Polyhedron& obj)>& func) const {
	const auto p = getBlockPtr(); 
	p->cellFunc(id, func);
} 

void Splitter::cellFunc(const Index3DId& id, const std::function<void(Polyhedron& obj)>& func) {
	auto p = getBlockPtr(); 
	p->cellFunc(id, func);
} 

void Splitter::edgeFunc(const EdgeKey& key, const std::function<void(const Edge& obj)>& func) const {
	const auto p = getBlockPtr(); 
	p->edgeFunc(key, func);
} 

void Splitter::edgeFunc(const EdgeKey& key, const std::function<void(Edge& obj)>& func) {
	auto p = getBlockPtr(); 
	p->edgeFunc(key, func);
}
