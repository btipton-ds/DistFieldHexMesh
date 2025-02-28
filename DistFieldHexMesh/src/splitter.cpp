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
#include <splitParams.h>
#include <polygon.h>
#include <polyhedron.h>
#include <splitter.h>
#include <block.h>
#include <volume.h>
#include <tolerances.h>
#include <utils.h>
#include <gradingOp.h>

#ifdef _WIN32
#include <windows.h>
#include <profileapi.h>
#endif // _WIN32

using namespace std;
using namespace DFHM;

Splitter::Splitter(Block* pBlock, const Index3DId& polyhedronId, vector<Index3DId>& localTouched)
	: _pBlock(pBlock)
	, _polyhedronId(polyhedronId)
	, _localTouched(localTouched)
{
}

bool Splitter::splitAtParamCenter()
{
	bool result = false;
	Vector3d tuv(0.5, 0.5, 0.5);
	result = splitAtParam(tuv);

	return result;
}

bool Splitter::splitAtParam(const Vector3d& tuv)
{
	if (!_pBlock->polyhedronExists(_polyhedronId))
		return false;

	bool result = false;
	_pBlock->cellFunc(_polyhedronId, [this, &tuv, &result](Polyhedron& cell) {
		result = splitAtParamInner(cell, tuv);
	});

	return result;
}

Index3DId Splitter::vertId(const Vector3d& pt)
{
	return _pBlock->getVertexIdOfPoint(pt);
}

bool Splitter::splitAtParamInner(Polyhedron& cell, const Vector3d& tuv)
{
#if 0 && defined(_DEBUG)
	// Now split the cell
	Index3DId testId(6, 4, 5, 0);
	if (testId == _polyhedronId) {
		int dbgBreak = 1;
		_pBlock->dumpPolyhedraObj({ _polyhedronId }, false, false, false);
	}
#endif

	Utils::Timer tmr(Utils::Timer::TT_splitAtPointInner);

	if (cell.classify(_cornerPts) == 8) {
		splitHexCell(cell, tuv);
	}
	
	return true;
}

void Splitter::splitHexCell(Polyhedron& cell, const Vector3d& tuv)
{
	const double tol = 10 * Tolerance::sameDistTol(); // Sloppier than "exact" match. We just need a "good" match
	createHexCellData(cell);
	if (_numSplitFaces > 0) {
		int dbgBreak = 1;
	}

	for (const auto& facePts : _cellFacePoints) {
		// This aligns the disordered faceIds with their cube face points
		const auto& faceId = findSourceFaceId(cell, facePts, tol);
		assert(faceId.isValid());

		conditionalSplitQuadFaceAtParam(faceId, facePts, 0.5, 0.5);
	}
#if _DEBUG
	for (const auto& faceId : cell.getFaceIds()) {
		_pBlock->faceFunc(faceId, [](const Polygon& face) {
			assert(face.isSplit());
		});
	}
#endif

	cell.detachFaces(); // This cell is about to be deleted, so detach it from all faces using it BEFORE we start attaching new ones

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

				addHexCell(cell, subCorners, tol);
			}
		}
	}

	// It's been completely split, so this cell can be removed
	_pBlock->freePolyhedron(_polyhedronId);
}

void Splitter::addHexCell(const Polyhedron& cell, const std::vector<Vector3d>& cubePts, double tol)
{
	assert(cubePts.size() == 8);
	std::vector<std::vector<Vector3d>> facePtList;
	GradingOp::getCubeFacePoints(cubePts, facePtList);
	assert(facePtList.size() == 6);

	MTC::set<Index3DId> cellFaceIds;
	for (const auto& facePts : facePtList) {
		assert(facePts.size() == 4);
		Index3DId id = findSourceFaceId(cell, facePts, tol);
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
	Index3DId newCellId = _pBlock->addCell(Polyhedron(cellFaceIds));

	_pBlock->cellFunc(newCellId, [this, cell](Polyhedron& newCell) {
		assert(newCell.getNumFaces(true) <= _pBlock->getSplitParams().maxCellFaces);
		newCell.setParentId(cell.getId());
		newCell.setSplitLevel(cell.getSplitLevel());
		newCell.setTriIndices(cell);
	});
		

}

Index3DId Splitter::findSourceFaceId(const Polyhedron& cell, const std::vector<Vector3d>& facePts, double tol) const
{
	const auto& faceIds = cell.getFaceIds();
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
	_pBlock->faceFunc(faceId, [this, &minErr, &facePts, &result, tol](const Polygon& face) {
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

void Splitter::createHexCellData(const Polyhedron& cell)
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
	const auto& faceIds = cell.getFaceIds();
	for (const auto& faceId : faceIds) {
		cell.faceFunc(faceId, [this](const Polygon& face) {
			if (face.getSplitIds().size() > 1)
				_numSplitFaces++;
		});
	}
}

void Splitter::conditionalSplitQuadFaceAtParam(const Index3DId& faceId, const std::vector<Vector3d>& facePts, double t, double u)
{
	_pBlock->faceFunc(faceId, [this, &facePts, t, u](Polygon& oldFace) {
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

