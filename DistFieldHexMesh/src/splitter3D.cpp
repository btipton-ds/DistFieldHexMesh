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
	assert(_pBlock->verifyTopology());

	if (!_pScratchVol)
		_pScratchVol = _pBlock->getVolume()->createScratchVolume();
	_pScratchBlock = _pScratchVol->getBlockPtr(Index3D(0, 0, 0));

	cellFunc(_polyhedronId, [this](Polyhedron& cell) {
#ifdef _DEBUG
		if (!cell.verifyTopology()) {
			assert(!"Bad topology");
		}
#endif // _DEBUG
		_adjacentCellIds = cell.getAdjacentCells();
	});
}

Splitter3D::~Splitter3D()
{
	if (_pScratchVol)
		_pScratchVol->clearEntries();

#ifdef _DEBUG
	bool hasErrors = false;
	for (const auto& cellId : _newCellIds) {
		cellFunc(cellId, [this, &hasErrors](const Polyhedron& cell) {
			bool isValid = cell.verifyTopology();
			if (!isValid) {
				hasErrors = true;
				cout << "Invalid new cell: " << cell.getId() << "," << cell.getNumFaces() << "\n";
			}
		});
	}

	for (const auto& cellId : _adjacentCellIds) {
		cellFunc(cellId, [this, &hasErrors](const Polyhedron& cell) {
			bool isValid = cell.verifyTopology();
			if (!isValid) {
				hasErrors = true;
				cout << "Invalid adj cell: " << cell.getId() << "," << cell.getNumFaces() << "\n";
			}
		});
	}
	if (hasErrors) {
		cout << "\n";
		cout << "_polyhedronId" << _polyhedronId << "\n";
		cout << "********************************************************************\n";
		cout << "********************************************************************\n";
	}
#endif // _DEBUG

#if !RUN_MULTI_THREAD // testing all blocks when multithreaded will defintely have race conditions
	auto pVol = _pBlock->getVolume();
	assert(pVol->verifyTopology(false));
#endif // !!RUN_MULTI_THREAD

}

void Splitter3D::reset()
{
	_pScratchBlock->clear();
	_newCellIds.clear();
}

void Splitter3D::imprintEverything()
{
	if (_testRun)
		return;

	size_t numImprints = 0;
	set<Index3DId> allVertIds;
	for (const auto& cellId : _newCellIds) {
		cellFunc(cellId, [this, &allVertIds](const Polyhedron& cell) {
			for (const auto& faceId : cell.getFaceIds()) {
				faceFunc(faceId, [&allVertIds](const Polygon& face) {
					const auto& temp = face.getVertexIds();
					allVertIds.insert(temp.begin(), temp.end());
				});
			}
		});
	}

	for (const auto& cellId : _newCellIds) {
		cellFunc(cellId, [this, &allVertIds, &numImprints](Polyhedron& cell) {
			for (const auto& faceId : cell.getFaceIds()) {
				if (!_polyhedronId.withinRange(faceId))
					continue;
				faceFunc(faceId, [this, &allVertIds, &numImprints](Polygon& face) {
					auto edgeKeys = face.getEdgeKeys();
					for (const auto& edgeKey : edgeKeys) {
						edgeFunc(edgeKey, [&allVertIds](Edge& edge) {
							edge.imprintVertices(allVertIds);
						});
					}
				});
			}
		});
	}

	for (const auto& cellId : _adjacentCellIds) {
		cellFunc(cellId, [this, &allVertIds, &numImprints](Polyhedron& cell) {
			for (const auto& faceId : cell.getFaceIds()) {
				if (!_polyhedronId.withinRange(faceId))
					continue;
				faceFunc(faceId, [this, &allVertIds, &numImprints](Polygon& face) {
					auto edgeKeys = face.getEdgeKeys();
					for (const auto& edgeKey : edgeKeys) {
						if (_polyhedronId.withinRange(edgeKey[0]) && _polyhedronId.withinRange(edgeKey[1])) {
							edgeFunc(edgeKey, [&allVertIds](Edge& edge) {
								edge.imprintVertices(allVertIds);
							});
						}
					}
					});
			}
		});
	}
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
			result = splitHexCell(tuv);
			break;
		default:
			result = false;
		}

		if (result) {
			_pBlock->freePolyhedron(_polyhedronId);
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

inline const Vector3d& Splitter3D::vertexPoint(const  Index3DId& id) const
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
}

bool Splitter3D::splitHexCell(const Vector3d& tuv)
{
	bool wasSplit = false;

	// Split the cell with a plane on each axis
	// intersects[] keeps track of intersections in the 8 possible subcells
	// When one of the binary split cells has no intersections, it's 4 subcells are marked as no intersect
	// When finished, only subcells with intersections are marked true
	bool isect[] = { 
		true, true, true, true,
		true, true, true, true,
	};

	for (int splitAxis = 0; splitAxis < 3; splitAxis++)
	{
		Utils::ScopedRestore restore0(_pBlock);
		_pBlock = _pScratchBlock;
		Utils::ScopedRestore restore1(_testRun);
		_testRun = true;
		const auto& scratchCellId = createScratchCell();
		MTC::vector<Index3DId> newCellIds;
		splitHexCell2_0(scratchCellId, tuv, splitAxis, newCellIds);
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

	int dbgBreak = 1;

	for (int splitAxis = 0; splitAxis < 3; splitAxis++) {
		bool doSplit = false;
		switch (splitAxis) {
		case 0:
			if ((!isect[0] && !isect[2] && !isect[4] && !isect[6]) ||
				(!isect[1] && !isect[3] && !isect[5] && !isect[7])) {
				doSplit = true;
			}
			break;
		case 1:
			if ((!isect[0] && !isect[1] && !isect[4] && !isect[5]) ||
				(!isect[2] && !isect[3] && !isect[6] && !isect[7])) {
				doSplit = true;
			}
			break;
		case 2:
			if ((!isect[0] && !isect[1] && !isect[2] && !isect[3]) ||
				(!isect[4] && !isect[5] && !isect[6] && !isect[7])) {
				doSplit = true;
			}
			break;
		}

		if (doSplit) {
			MTC::vector<Index3DId> newCellIds;
			splitHexCell2_0(_polyhedronId, tuv, splitAxis, newCellIds);
			wasSplit = true;
			break;
		}
	}

	return wasSplit;
}

bool Splitter3D::splitHexCell2_0(const Index3DId& parentId, const Vector3d& tuv, int axis, MTC::vector<Index3DId>& newCells)
{
	getBlockPtr()->cellFunc(parentId, [](Polyhedron& cell) {
		cell.detachFaces();
	});

	const double tol = 10 * _distTol; // Sloppier than "exact" match. We just need a "good" match
	_newCellIds.clear();

#ifdef _DEBUG
	Index3DId testId(6, 0, 3, 0);
	if (parentId == testId) {
		int dbgBreak = 1;
	}
#endif

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
			TRI_LERP(_cornerPts, t0, u0, v0),
			TRI_LERP(_cornerPts, t1, u0, v0),
			TRI_LERP(_cornerPts, t1, u1, v0),
			TRI_LERP(_cornerPts, t0, u1, v0),

			TRI_LERP(_cornerPts, t0, u0, v1),
			TRI_LERP(_cornerPts, t1, u0, v1),
			TRI_LERP(_cornerPts, t1, u1, v1),
			TRI_LERP(_cornerPts, t0, u1, v1),
		};
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

	imprintEverything();
	return true;
}

Index3DId Splitter3D::createScratchCell()
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
		if (_testRun) {
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
		} else {
			// Use createFace to do the complex face slitting
			createFaces(parentId, faceVerts, newFaceIds, tol);
			if (newFaceIds.empty()) {
				auto id = getBlockPtr()->addPolygon(Polygon(faceVerts));
#ifdef _DEBUG
				faceFunc(id, [](const Polygon& face) {
					assert(face.getCellIds().size() < 2);
				});
#endif
				cellFaceIds.insert(id);
			}
			else {
#ifdef _DEBUG
				for (const auto& id : newFaceIds) {
					assert(id.isValid());
					faceFunc(id, [](const Polygon& face) {
						assert(face.getCellIds().size() < 2);
					});
				}
#endif // _DEBUG
				cellFaceIds.insert(newFaceIds.begin(), newFaceIds.end());
			}
		}
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
	_newCellIds.insert(newCellId);

	return newCellId;
}

void Splitter3D::createFaces(const Index3DId& parentId, const MTC::vector<Index3DId>& newFaceVertIds, MTC::set<Index3DId>& newFaceIds, double tol)
{
	Index3DId result;

	auto existingFaceId = getBlockPtr()->findPolygon(Polygon(newFaceVertIds));
	if (existingFaceId.isValid()) {
		newFaceIds.insert(existingFaceId);
		return;
	}
	FastBisectionSet<Index3DId> oldFaceIds;
	cellFunc(parentId, [&oldFaceIds](const Polyhedron& parentCell) {
		oldFaceIds = parentCell.getFaceIds();
	});

	Vector3d newFaceNorm = Polygon::calUnitNormalStat(getBlockPtr(), newFaceVertIds);
	Vector3d newFaceOrigin = getBlockPtr()->getVertexPoint(newFaceVertIds[0]);
	Planed newFacePlane(newFaceOrigin, newFaceNorm);

	for (const auto& oldFaceId : oldFaceIds) {
#if 0 && defined(_DEBUG)
#define DUMP_EDGES 1
			Index3DId testId(2, 0, 3, 0);
			if (oldFaceId == testId) {
				int dbgBreak = 1;
			}
#endif // _DEBUG

			Planed oldFacePlane;
			faceFunc(oldFaceId, [this, &oldFacePlane](Polygon& oldFace) {
				oldFacePlane = oldFace.calPlane();
				});

			if (!oldFacePlane.isCoincident(newFacePlane.getOrgin(), _distTol))
				continue;
			Vector3d cp = oldFacePlane.getNormal().cross(newFacePlane.getNormal());
			if (cp.squaredNorm() > _paramTolSqr)
				continue;

			Splitter2D splitter2d(oldFacePlane);
			vector<Vector3d> oldBoundaryPoints, newBoundaryPoints;
			faceFunc(oldFaceId, [this, &splitter2d, &newFaceVertIds, &oldBoundaryPoints, &newBoundaryPoints](Polygon& oldFace) {

				const auto& oldVertIds = oldFace.getVertexIds();

				for (size_t i = 0; i < oldVertIds.size(); i++) {
					size_t j = (i + 1) % oldVertIds.size();
					const auto& pt0 = vertexPoint(oldVertIds[i]);
					const auto& pt1 = vertexPoint(oldVertIds[j]);
					splitter2d.add3DEdge(pt0, pt1);

					oldBoundaryPoints.push_back(pt0);
				}

				for (size_t i = 0; i < newFaceVertIds.size(); i++) {
					size_t j = (i + 1) % newFaceVertIds.size();
					const auto& pt0 = vertexPoint(newFaceVertIds[i]);
					const auto& pt1 = vertexPoint(newFaceVertIds[j]);
					splitter2d.add3DEdge(pt0, pt1);

					newBoundaryPoints.push_back(pt0);
				}
				});

#ifdef DUMP_EDGES
			if (oldFaceId == testId) {
				vector<vector<Vector3d>> edgePts;
				splitter2d.getEdgePts(edgePts);
				getBlockPtr()->getVolume()->writeObj("D:/DarkSky/Projects/output/objs/splitEdges.obj", edgePts, false);
				int dbgBreak = 1;
			}
#endif // _DEBUG
			vector<vector<Vector3d>> oldFacePoints, newFacePoints;
			splitter2d.getFacePoints(oldBoundaryPoints, oldFacePoints); // These face points describe the replacement faces for the oldFace
			splitter2d.getFacePoints(newBoundaryPoints, newFacePoints);  // These face points describe the replacement faces for the newFace

#if 0
			for (size_t i = 0; i < oldFacePoints.size(); i++) {
				stringstream ss;
				ss << "D:/DarkSky/Projects/output/objs/oldFacePoints_" << i << ".obj";
				getBlockPtr()->getVolume()->writeObj(ss.str(), { oldFacePoints[i] }, false);
			}
			for (size_t i = 0; i < newFacePoints.size(); i++) {
				stringstream ss;
				ss << "D:/DarkSky/Projects/output/objs/newFacePoints_" << i << ".obj";
				getBlockPtr()->getVolume()->writeObj(ss.str(), { newFacePoints[i] }, false);
			}
#endif

			replaceExistingFaces(oldFaceId, oldFacePoints);

			for (const auto& facePts : newFacePoints) {
				MTC::vector<Index3DId> vertIds;
				for (const auto& pt : facePts) {
					auto id = vertId(pt);
					assert(getBlockPtr()->getBlockIdx().withinRange(id));
					vertIds.push_back(id);
				}

				auto newFaceId = getBlockPtr()->addPolygon(Polygon(vertIds));
				assert(getBlockPtr()->getBlockIdx().withinRange(newFaceId));
#ifdef _DEBUG
				Index3DId testId(0, 0, 0, 10);
				if (testId == newFaceId) {
					int dbgBreak = 1;
				}
#endif
				newFaceIds.insert(newFaceId);
			}
			int dbgBreak = 1;
	}
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
		_cornerPts.push_back(vertexPoint(id));
	GradingOp::getCubeFaceVertIds(cornerVertIds, _cellFaceVertIds);

	_cellFacePoints.resize(_cellFaceVertIds.size());
	for (size_t i = 0; i < _cellFaceVertIds.size(); i++) {
		const auto& faceVerts = _cellFaceVertIds[i];
		auto& facePts = _cellFacePoints[i];
		facePts.resize(faceVerts.size());
		for (size_t j = 0; j < faceVerts.size(); j++) {
			facePts[j] = vertexPoint(faceVerts[j]);
		}
	}

	_oldFaceToNewFaceMap.clear();
	_oldFaceToNewFaceMap.resize(6);
	for (int i = 0; i < 6; i++)
		collectAllPolyhedronFaces(i);

	int dbgBreak = 1;
}

void Splitter3D::collectAllPolyhedronFaces(size_t index)
{
	const auto& facePts(_cellFacePoints[index]);
	assert(facePts.size() >= 3);

	Vector3d v0 = facePts[0] - facePts[1];
	Vector3d v1 = facePts[2] - facePts[1];
	Vector3d n = v1.cross(v0);
	Planed polyPlane(facePts[0], n);

	auto& faceSet = _oldFaceToNewFaceMap[index];
	cellFunc(_polyhedronId, [this, &polyPlane, &faceSet](const Polyhedron& cell) {
		assert(cell.isClosed());
		const auto& faceIds = cell.getFaceIds();
		for (const auto& faceId : faceIds) {
			faceFunc(faceId, [&polyPlane, &faceSet](const Polygon& face) {
				if (face.isCoplanar(polyPlane))
					faceSet.insert(face.getId());
			});
		}
	});

	assert(!faceSet.empty());
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
