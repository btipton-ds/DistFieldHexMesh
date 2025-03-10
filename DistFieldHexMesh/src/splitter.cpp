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

		createHexCellData(parentCell);


		switch (_cornerPts.size()) {
			case 8:
				cellType = CT_HEX;
				break;
			default:
				break;
		}
		parentCell.detachFaces();
	});

	Vector3d tuv(0.5, 0.5, 0.5);
	switch (cellType) {
	case CT_HEX:
		result = splitHexCell(tuv);
		break;
	default:
		result = false;
	}

	if (result) {
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

bool Splitter::splitHexCell(const Vector3d& tuv)
{
	bool wasSplit = false;
#if 0
	wasSplit = splitHexCell8(_polyhedronId, tuv);
#else
	int intersects[] = { 0, 0, 0 };
	for (int i = 0; i < 3; i++)
	{
		Utils::ScopedRestore restore(_pBlock);
		const auto& scratchCellId = createScratchCell();
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
//		wasSplit =splitHexCell8(_polyhedronId, tuv);
	} else if (((intersects[0] == 2) && (intersects[1] == 2) && (intersects[2] == 2)) ||
		       ((intersects[0] == 1) && (intersects[1] == 1) && (intersects[2] == 1))) {
		wasSplit = splitHexCell8(_polyhedronId, tuv);

	} else if ((intersects[0] == 1) && (intersects[1] == 2) && (intersects[2] == 2)) {
		numSplits2++;
		wasSplit = splitHexCell2(_polyhedronId, tuv, 0);
	} else if ((intersects[0] == 2) && (intersects[1] == 1) && (intersects[2] == 2)) {
		numSplits2++;
		wasSplit = splitHexCell2(_polyhedronId, tuv, 1);
	} else if ((intersects[0] == 2) && (intersects[1] == 2) && (intersects[2] == 1)) {
		numSplits2++;
		wasSplit = splitHexCell2(_polyhedronId, tuv, 2);

	} else if ((intersects[0] == 2) && (intersects[1] == 1) && (intersects[2] == 1)) {
		numSplits4++;
		wasSplit = splitHexCell4(_polyhedronId, tuv, 0);
	} else if ((intersects[0] == 1) && (intersects[1] == 2) && (intersects[2] == 1)) {
		numSplits4++;
		wasSplit = splitHexCell4(_polyhedronId, tuv, 1);
	} else if ((intersects[0] == 1) && (intersects[1] == 1) && (intersects[2] == 2)) {
		numSplits4++;
		wasSplit = splitHexCell4(_polyhedronId, tuv, 2);
	} else {
		assert(!"Should never get here");
	}
#endif
	return wasSplit;
}

bool Splitter::splitHexCell8(const Index3DId& parentId, const Vector3d& tuv)
{
	const double tol = 10 * Tolerance::sameDistTol(); // Sloppier than "exact" match. We just need a "good" match

	cellFunc(parentId, [this, &tuv, &tol](Polyhedron& parentCell) {
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

				MTC::vector<Index3DId> subCorners = {
					vertId(TRI_LERP(_cornerPts, t0, u0, v0)),
					vertId(TRI_LERP(_cornerPts, t1, u0, v0)),
					vertId(TRI_LERP(_cornerPts, t1, u1, v0)),
					vertId(TRI_LERP(_cornerPts, t0, u1, v0)),

					vertId(TRI_LERP(_cornerPts, t0, u0, v1)),
					vertId(TRI_LERP(_cornerPts, t1, u0, v1)),
					vertId(TRI_LERP(_cornerPts, t1, u1, v1)),
					vertId(TRI_LERP(_cornerPts, t0, u1, v1)),
				};

				addHexCell(parentId, subCorners, tol);
			}
		}
	}

	return true;
}

bool Splitter::splitHexCell2(const Index3DId& parentId, const Vector3d& tuv, int axis)
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
		MTC::vector<Index3DId> subCorners = {
			vertId(TRI_LERP(_cornerPts, t0, u0, v0)),
			vertId(TRI_LERP(_cornerPts, t1, u0, v0)),
			vertId(TRI_LERP(_cornerPts, t1, u1, v0)),
			vertId(TRI_LERP(_cornerPts, t0, u1, v0)),

			vertId(TRI_LERP(_cornerPts, t0, u0, v1)),
			vertId(TRI_LERP(_cornerPts, t1, u0, v1)),
			vertId(TRI_LERP(_cornerPts, t1, u1, v1)),
			vertId(TRI_LERP(_cornerPts, t0, u1, v1)),
		};

		addHexCell(parentId, subCorners, tol);
	}
#endif
	return true;
}

bool Splitter::splitHexCell4(const Index3DId& parentId, const Vector3d& tuv, int axis)
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
			MTC::vector<Index3DId> subCorners = {
				vertId(TRI_LERP(_cornerPts, t0, u0, v0)),
				vertId(TRI_LERP(_cornerPts, t1, u0, v0)),
				vertId(TRI_LERP(_cornerPts, t1, u1, v0)),
				vertId(TRI_LERP(_cornerPts, t0, u1, v0)),

				vertId(TRI_LERP(_cornerPts, t0, u0, v1)),
				vertId(TRI_LERP(_cornerPts, t1, u0, v1)),
				vertId(TRI_LERP(_cornerPts, t1, u1, v1)),
				vertId(TRI_LERP(_cornerPts, t0, u1, v1)),
			};

			addHexCell(parentId, subCorners, tol);
		}
	}
#endif
	return true;
}

Index3DId Splitter::createScratchCell()
{
	Utils::ScopedRestore restore(_pBlock);
	_pBlock = _pSrcBlock;

	FastBisectionSet<Index3DId> srcFaceIds;
	cellFunc(_polyhedronId, [&srcFaceIds](const Polyhedron& srcCell) {
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

Index3DId Splitter::createScratchFace(const Index3DId& srcFaceId)
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

		newFaceId = _pScratchBlock->addFace(Polygon(newVertIds));

	});

	return newFaceId;
}

void Splitter::addHexCell(const Index3DId& parentId, const std::vector<Index3DId>& cubeVerts, double tol)
{
	assert(cubeVerts.size() == 8);
	std::vector<std::vector<Index3DId>> faceVertList;
	GradingOp::getCubeFaceVertIds(cubeVerts, faceVertList);
	assert(faceVertList.size() == 6);

	MTC::set<Index3DId> cellFaceIds;
	for (const auto& faceVerts : faceVertList) {
		assert(faceVerts.size() == 4);
		MTC::set<Index3DId> newFaceIds;
		createFace(parentId, faceVerts, newFaceIds, tol);
#ifdef _DEBUG
		for (const auto& id : newFaceIds) {
			assert(id.isValid());
		}
#endif // _DEBUG

		cellFaceIds.insert(newFaceIds.begin(), newFaceIds.end());
	}
	Index3DId newCellId = _pBlock->addCell(Polyhedron(cellFaceIds, cubeVerts), parentId);

	cellFunc(newCellId, [this](Polyhedron& newCell) {
		assert(newCell.getNumFaces() <= _params.maxCellFaces);
	});
}

void Splitter::createFace(const Index3DId& parentId, const std::vector<Index3DId>& faceVertIds, MTC::set<Index3DId>& newFaceIds, double tol)
{
	Index3DId result;

	FastBisectionSet<Index3DId> faceIds;
	cellFunc(parentId, [&faceIds](const Polyhedron& parentCell) {
		faceIds = parentCell.getFaceIds();
	});

	for (const auto& faceId : faceIds) {
		faceFunc(faceId, [this, &faceVertIds, &newFaceIds](Polygon& face) {
			Planed oldFacePlane = face.calPlane();
			const auto& oldFaceVertIds = face.getVertexIds();
	
			for (const auto& id : oldFaceVertIds) {
				const auto& pt = vertexPoint(id);
				if (!oldFacePlane.isCoincident(pt, Tolerance::sameDistTol()))
					return; // go to the next face
			}
			// The new face is coplanar with and existing face
			// Need to build the new face(s) to fit the old face
			face.recreateToMatch(faceVertIds, newFaceIds);
		});
	}
}

void Splitter::createHexCellData(const Polyhedron& parentCell)
{
	auto& cornerVertIds = parentCell.getCanonicalVertIds();
	_cornerPts.reserve(cornerVertIds.size());
	for (const auto& id : cornerVertIds)
		_cornerPts.push_back(vertexPoint(id));
	GradingOp::getCubeFaceVertIds(cornerVertIds, _cellFaceVertIds);

	_cellFaceVertIds.resize(_cellFacePoints.size());
	for (size_t i = 0; i < _cellFacePoints.size(); i++) {
		const auto& facePts = _cellFacePoints[i];
		auto& faceVerts = _cellFaceVertIds[i];
		faceVerts.resize(facePts.size());
		for (size_t j = 0; j < facePts.size(); j++) {
			faceVerts[j] = vertId(facePts[j]);
		}
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
