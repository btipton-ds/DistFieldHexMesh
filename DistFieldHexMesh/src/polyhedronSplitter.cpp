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
#include <polygonSplitter.h>
#include <polyhedronSplitter.h>
#include <block.h>
#include <volume.h>
#include <tolerances.h>
#include <utils.h>

#ifdef _WIN32
#include <windows.h>
#include <profileapi.h>
#endif // _WIN32

using namespace std;
using namespace DFHM;

PolyhedronSplitter::PolyhedronSplitter(Block* pBlock, const Index3DId& polyhedronId)
	: _pBlock(pBlock)
	, _polyhedronId(polyhedronId)
{
}

bool PolyhedronSplitter::splitIfNeeded()
{
	bool result = false;
	_pBlock->cellAvailFunc(TS_REFERENCE, _polyhedronId, [this, &result](const Polyhedron& cell) {
		set<Index3DId> newCellIds;
		Vector3d ctr;
		Planed plane;
		if (cell.needsDivideAtCentroid()) {
			ctr = cell.calCentroid();
			result = splitAtPoint(ctr);
		} else {
			// TODO This needs to be an adaptive split to match the neighbor
			// We get here when we have to split a neighbor cell prior to splitting a cell.
			ctr = cell.calCentroid();
			result = splitAtPoint(ctr);
		}
	});

	return result;
}

bool PolyhedronSplitter::splitAtPoint(const Vector3d& pt)
{
	if (!_pBlock->polyhedronExists(TS_REAL, _polyhedronId))
		return false;

	_pBlock->makeRefPolyhedronIfRequired(_polyhedronId);

	assert(_pBlock->polyhedronExists(TS_REFERENCE, _polyhedronId));
	Polyhedron& referenceCell = _pBlock->getPolyhedron(TS_REFERENCE, _polyhedronId);

	assert(_pBlock->polyhedronExists(TS_REAL, _polyhedronId));
	Polyhedron& realCell = _pBlock->getPolyhedron(TS_REAL, _polyhedronId);

	bool result = splitAtPointInner(realCell, referenceCell, pt);

	if (result && _pBlock->polyhedronExists(TS_REAL, _polyhedronId)) {
		_pBlock->freePolyhedron(_polyhedronId, true);
	}

	return result;
}

bool PolyhedronSplitter::splitAtPointInner(Polyhedron& realCell, Polyhedron& referanceCell, const Vector3d& pt) const
{
	assert(_pBlock);

#if 0 && defined(_DEBUG)
	// Now split the cell
	if (Index3DId(0, 12, 0, 19) == _polyhedronId) {
		_pBlock->dumpPolyhedraObj({ _polyhedronId }, false, false, false, { pt });
	}
#endif

	Utils::Timer tmr(Utils::Timer::TT_splitAtPointInner);


	MTC::set<Index3DId> cornerVerts;
	referanceCell.getVertIds(cornerVerts);
	assert(cornerVerts.size() == 8);
	const auto& faceIds = referanceCell.getFaceIds();
	assert(faceIds.size() == 6);
	MTC::map<Index3DId, MTC::set<Index3DId>> cornerVertToFaceMap;
	for (const auto& vertId : cornerVerts) {
		cornerVertToFaceMap.insert(make_pair(vertId, set<Index3DId>()));
	}

	// Split all faces which require splitting
	bool pass = true;
	for (const auto& faceId : faceIds) {

		PolygonSplitter splitter(_pBlock, faceId);
		splitter.splitAtPoint(pt);

		faceFunc(TS_REFERENCE, faceId, [this, &cornerVertToFaceMap, &pass](const Polygon& refFace) {
			assert(refFace._splitFaceProductIds.size() == refFace.getVertexIds().size());

			for (const auto& childFaceId : refFace._splitFaceProductIds) {
				if (!_pBlock->polygonExists(TS_REAL, childFaceId)) {
					pass = false;
					break;
				}
				faceFunc(TS_REAL, childFaceId, [&cornerVertToFaceMap](const Polygon& childFace) {
					for (const auto& vertId : childFace.getVertexIds()) {
						auto iter = cornerVertToFaceMap.find(vertId);
						if (iter != cornerVertToFaceMap.end()) {
							iter->second.insert(childFace.getId());
						}
					}
				});
			}
		});

		if (!pass) {
			break;
		}
	}

#if _DEBUG
	if (!pass) {
		_pBlock->dumpPolyhedraObj({ realCell.getId() }, false, false, false);
		return false;
	}
#endif

	Index3DId cellMidId = _pBlock->addVertex(pt);

	for (const auto& pair : cornerVertToFaceMap) {
		auto cornerVertId = pair.first;
		auto splitVertFaces = pair.second;
		assert(splitVertFaces.size() == 3);
		MTC::map<Edge, MTC::set<Index3DId>> fullEdgeToFaceMap;
		for (const auto& splitFaceId : splitVertFaces) {
			assert(_pBlock->polygonExists(TS_REAL, splitFaceId));
			_pBlock->faceAvailFunc(TS_REFERENCE, splitFaceId, [&fullEdgeToFaceMap](const Polygon& splitFace) {
				assert(splitFace.getVertexIds().size() == 4);
				const auto& verts = splitFace.getVertexIds();
				for (size_t i = 0; i < verts.size(); i++) {
					size_t j = (i + 1) % verts.size();
					Edge edge(verts[i], verts[j]);
					auto iter = fullEdgeToFaceMap.find(edge);
					if (iter == fullEdgeToFaceMap.end())
						iter = fullEdgeToFaceMap.insert(make_pair(edge, set<Index3DId>())).first;
					iter->second.insert(splitFace.getId());
				}
			});

#if DEBUG_BREAKS && defined(_DEBUG)
			int dbgBreak = 1;
#endif
		}

		assert(fullEdgeToFaceMap.size() == 9);
		MTC::map<Edge, MTC::set<Index3DId>> edgeToFaceMap;
		for (const auto& pair : fullEdgeToFaceMap) {
			if (pair.second.size() == 2)
				edgeToFaceMap.insert(pair);
		}

		assert(edgeToFaceMap.size() == 3);

		for (const auto& pair : edgeToFaceMap) {
			const auto& edge = pair.first;
			const auto& edgeFaces = pair.second;

			const auto& edgeVert1 = edge.getOtherVert(cornerVertId);

			Index3DId faceVert0, faceVert1;
			auto iter = edgeFaces.begin();
			_pBlock->faceAvailFunc(TS_REFERENCE, *iter++, [&faceVert0](const Polygon& face) {
				assert(face.getVertexIds().size() == 4);
				faceVert0 = face.getVertexIds()[0];
			});
			_pBlock->faceAvailFunc(TS_REFERENCE, *iter, [&faceVert1](const Polygon& face) {
				assert(face.getVertexIds().size() == 4);
				faceVert1 = face.getVertexIds()[0];
			});

			MTC::vector<Index3DId> verts = {
				edgeVert1,
				faceVert0,
				cellMidId,
				faceVert1,
			};

			auto newFaceId = _pBlock->addFace(verts);
			splitVertFaces.insert(newFaceId);
		}


		assert(splitVertFaces.size() == 6);
		for (const auto& faceId : splitVertFaces) {
#if DEBUG_BREAKS && defined(_DEBUG)
			if (Index3DId(5, 5, 1, 45) == faceId) {
				int dbgBreak = 1;
			}
#endif

			_pBlock->faceFunc(TS_REAL, faceId, [this](Polygon& face) {
				face.removeCellId(_polyhedronId);
				face.removeDeadCellIds();
			});
		}
		Polyhedron newCell(splitVertFaces);

		auto newCellId = _pBlock->addCell(newCell);
#if DEBUG_BREAKS && defined(_DEBUG)
		if (Index3DId(5, 5, 0, 45) == newCellId) {
			int dbgBreak = 1;
		}
#endif

		_pBlock->cellFunc(TS_REAL, newCellId, [this, &realCell](Polyhedron& newCell) {
			newCell.setSplitLevel(realCell.getSplitLevel() + 1);
#ifdef _DEBUG
			for (const auto& faceId : newCell.getFaceIds()) {
				faceFunc(TS_REAL, faceId, [this](const Polygon& face) {
					assert(face.cellsOwnThis());
				});
			}
#endif // _DEBUG

			newCell.setTriIndices(realCell.getTriIndices());
			newCell.setEdgeIndices(realCell.getEdgeIndices());
		});
	}

	return true;
}

bool PolyhedronSplitter::createAllModelMeshFaces(const std::vector<TriMesh::PatchPtr>& patches, const BuildCFDParams& params, MTC::set<Index3DId>& modelFaces)
{
	cellFunc(TS_REAL, _polyhedronId, [&](const Polyhedron& cell) {
		for (const auto& pPatch : patches) {
			for (const auto& modelFaceTris : pPatch->getFaces()) {
				MTC::set<Edge> faceEdges;
				createFaceEdgesFromMeshFace(cell, modelFaceTris, params, faceEdges);
				MTC::vector<MTC::vector<Index3DId>> faceVerts;
				if (PolygonSplitter::connectEdges(getBlockPtr(), faceEdges, faceVerts)) {
					Vector3d modelFaceNorm = calModelFaceNormal(modelFaceTris);
					for (auto& verts : faceVerts) {
						Vector3d faceNorm = Polygon::calUnitNormalStat(getBlockPtr(), verts);
						if (faceNorm.dot(modelFaceNorm) < 0)
							std::reverse(verts.begin(), verts.end());

						Index3DId faceId = getBlockPtr()->addFace(verts);
						faceFunc(TS_REAL, faceId, [&modelFaceNorm](const Polygon& face) {
							assert(face.calUnitNormal().dot(modelFaceNorm) > 0);
						});
						modelFaces.insert(faceId);
					}
				}
			}
		}
	});

	return !modelFaces.empty();
}

Vector3d PolyhedronSplitter::calModelFaceNormal(const std::vector<size_t>& modelFaceTris) const
{
	Vector3d normal(-1, 0, 0);
#if 0
	auto pMesh = getBlockPtr()->getModelMesh();
	for (size_t triIdx : modelFaceTris)
		normal += pMesh->triUnitNormal(triIdx);
	normal.normalize();
#endif
	return normal;
}

Vector3d PolyhedronSplitter::choosePrincipalAxis(const Vector3d& v, const Vector3d& skipV)
{
	static const Vector3d xAxis(1, 0, 0);
	static const Vector3d yAxis(0, 1, 0);
	static const Vector3d zAxis(0, 0, 1);
	const double tol = Tolerance::sameDistTol();

	Vector3d result;

	double maxDp = 0, dp;

	dp = xAxis.dot(v);
	if (!tolerantEquals(xAxis, skipV, tol) && fabs(dp) > maxDp) {
		result = xAxis;
		maxDp = fabs(dp);
	}

	dp = yAxis.dot(v);
	if (!tolerantEquals(yAxis, skipV, tol) && fabs(dp) > maxDp) {
		result = yAxis;
		maxDp = fabs(dp);
	}

	dp = zAxis.dot(v);
	if (!tolerantEquals(zAxis, skipV, tol) && fabs(dp) > maxDp) {
		result = zAxis;
		maxDp = fabs(dp);
	}

	return result;
}

bool PolyhedronSplitter::splitWithPlane(const Planed& plane, MTC::set<Index3DId>& newCellIds)
{
	MTC::set<Index3DId> faceIds;
	std::vector<size_t> edgeIndices, triIndices;
	
	cellFunc(TS_REAL, _polyhedronId, [&faceIds, &edgeIndices, &triIndices](const Polyhedron& cell) {
		faceIds = cell.getFaceIds();
		edgeIndices = cell.getEdgeIndices();
		triIndices = cell.getTriIndices();
	});

	for (const auto& faceId : faceIds) {
		Planed facePlane;
		faceFunc(TS_REAL, faceId, [&facePlane](const Polygon& face) {
			facePlane = face.calPlane();
		});

		if (plane.isCoincident(facePlane, Tolerance::planeCoincidentDistTol(), Tolerance::planeCoincidentCrossProductTol()))
			return false;
	}

	set<Edge> splitEdges;
	MTC::set<Index3DId> posFaceIds, negFaceIds;
	for (const auto& faceId : faceIds) {
		MTC::vector<Index3DId> vertIds;
		faceFunc(TS_REAL, faceId, [&vertIds](const Polygon& face) {
			vertIds = face.getVertexIds();
		});

		PolygonSplitter ps(_pBlock, faceId);
		Edge edge = ps.createIntersectionEdge(plane);

		MTC::set<Edge> posEdges, negEdges;
		if (edge.isValid()) {
			splitEdges.insert(edge);
			posEdges.insert(edge);
			negEdges.insert(edge);
		}

		for (size_t i = 0; i < vertIds.size(); i++) {
			size_t j = (i + 1) % vertIds.size();
			Index3DId vertId0 = vertIds[i];
			Index3DId vertId1 = vertIds[j];
			Vector3d pt0 = getBlockPtr()->getVertexPoint(vertId0);
			Vector3d pt1 = getBlockPtr()->getVertexPoint(vertId1);
			double d0 = plane.distanceToPoint(pt0, false);
			double d1 = plane.distanceToPoint(pt1, false);
			if (d0 >= 0 && d1 >= 0) {
				posEdges.insert(Edge(vertId0, vertId1));
			} else if ((d0 >= 0 && d1 < 0) || (d0 < 0 && d1 >= 0)) {
				LineSegmentd seg(pt0, pt1);
				RayHitd hit;
				if (plane.intersectLineSegment(seg, hit, Tolerance::sameDistTol())) {
					Index3DId midId = getBlockPtr()->addVertex(hit.hitPt);
					if (d0 >= 0) {
						posEdges.insert(Edge(vertId0, midId));
						negEdges.insert(Edge(vertId1, midId));
					} else {
						negEdges.insert(Edge(vertId0, midId));
						posEdges.insert(Edge(vertId1, midId));
					}	
				} else
					assert(!"should not be possible");
			} else
				negEdges.insert(Edge(vertId0, vertId1));
		}

		if (!posEdges.empty()) {
			MTC::vector<MTC::vector<Index3DId>> faceVertices;
			if (PolygonSplitter::connectEdges(getBlockPtr(), posEdges, faceVertices)) {
				for (const auto& vertices : faceVertices) {
					Index3DId faceId = getBlockPtr()->addFace(vertices);
					posFaceIds.insert(faceId);
				}
			}
		}
		if (!negEdges.empty()) {
			MTC::vector<MTC::vector<Index3DId>> faceVertices;
			if (PolygonSplitter::connectEdges(getBlockPtr(), negEdges, faceVertices)) {
				for (const auto& vertices : faceVertices) {
					Index3DId faceId = getBlockPtr()->addFace(vertices);
					negFaceIds.insert(faceId);
				}
			}
		}
	}

	MTC::vector<MTC::vector<Index3DId>> faceVertices;
	if (!PolygonSplitter::connectEdges(getBlockPtr(), splitEdges, faceVertices))
		return false;

	for (const auto& vertices : faceVertices) {
		Index3DId splitFaceId = getBlockPtr()->addFace(vertices);
		if (!splitFaceId.isValid())
			return false;
		posFaceIds.insert(splitFaceId);
		negFaceIds.insert(splitFaceId);
	}

	Index3DId cellId;

	cellId = getBlockPtr()->addCell(posFaceIds);
	newCellIds.insert(cellId);

	cellId = getBlockPtr()->addCell(negFaceIds);
	newCellIds.insert(cellId);
	for (const auto& cellId : newCellIds) {
		cellFunc(TS_REAL, cellId, [&edgeIndices, &triIndices](Polyhedron& cell) {
			cell.setEdgeIndices(edgeIndices);
			cell.setTriIndices(triIndices);
		});
	}

	return true;
}

namespace
{
	std::set<Index3DId> testIds = { 
		Index3DId(3, 9, 0, 9),
		Index3DId(0, 12, 0, 9),
		Index3DId(0, 12, 0, 49),
		Index3DId(0, 12, 0, 50),
	};
}

bool PolyhedronSplitter::cutWithModelMesh(const BuildCFDParams& params)
{
	MTC::set<Index3DId> deadCellIds, newCellIds;

	if (testIds.contains(_polyhedronId)) {
		int dbgBreak = 1;
	}
	if (!cutWithModelMeshInner(params, deadCellIds, newCellIds))
		return false;

	for (const auto& newCellId : newCellIds) {
		bool needsSplit = false;
		cellFunc(TS_REAL, newCellId, [&needsSplit](const Polyhedron& newCell) {
			needsSplit = newCell.intersectsModelPrecise();
		});
		if (needsSplit) {
			MTC::set<Index3DId> tmpNewCellIds;
			PolyhedronSplitter ps(getBlockPtr(), newCellId);
			if (!ps.cutWithModelMeshInner(params, deadCellIds, tmpNewCellIds))
				return false;
		}
	}

	for (const auto& cellId : deadCellIds) {
		cellFunc(TS_REAL, cellId, [this](const Polyhedron& cell) {
			for (const auto& faceId : cell.getFaceIds()) {
				if (getBlockPtr()->polygonExists(TS_REAL, faceId)) {
					bool deleteFace = false;
					_pBlock->faceFunc(TS_REAL, faceId, [&cell, &deleteFace](Polygon& face) {
						deleteFace = face.numCells() == 0;
					});
					if (deleteFace) {
						_pBlock->freePolygon(faceId, false);
					}
				}
			}
		});

		_pBlock->freePolyhedron(cellId, false);
	}

	return true;
}

bool PolyhedronSplitter::createModelMeshPatches(const BuildCFDParams& params, std::vector<TriMesh::PatchPtr>& patches,
	std::vector<std::vector<std::vector<size_t>>>& allChains) const
{
#if 0
	auto pMesh = getBlockPtr()->getModelMesh();
	std::vector<size_t> tris;
	MTC::set<Index3DId> faceIds;
	cellFunc(TS_REAL, _polyhedronId, [this, &faceIds, &tris](const Polyhedron& cell) {
		tris = cell.getTriIndices();
		faceIds = cell.getFaceIds();
	});

	if (tris.empty())
		return false;

	double sinSharpAngle = sin(params.getSharpAngleRadians());
	if (!pMesh->createPatches(tris, sinSharpAngle, patches))
		return false;

	for (size_t i = 0; i < patches.size(); i++) {
		auto pPatch = patches[i];
		std::vector<std::vector<size_t>> faceChains;
		for (const auto& sec : pPatch->getSharpEdgeChains()) {
			faceChains.push_back(sec);
		}
		allChains.push_back(faceChains);
	}
#endif
	return true;
}

bool PolyhedronSplitter::splitWithSharpEdgePlanes(std::vector<std::vector<std::vector<size_t>>>& allChains, MTC::set<Index3DId>& newCellIds)
{
	std::vector<Vector3d> piercePoints;
	if (findPiercePoints(allChains, piercePoints)) {
		Planed plane;
		double err;
		if (piercePoints.size() > 2 && bestFitPlane(piercePoints, plane, err) && err < Tolerance::sameDistTol()) {
			PolyhedronSplitter subSplitter(_pBlock, _polyhedronId);
			if (subSplitter.splitWithPlane(plane, newCellIds)) {
				allChains.clear();
				return true;
			}
		}
	}
	return false;
}

bool PolyhedronSplitter::createCellsFromFaces(MTC::set<Index3DId>& faceIds, MTC::set<Index3DId>& newCellIds)
{
	MTC::map<Edge, MTC::set<Index3DId>> edgeFaceMap;
	for (const auto& faceId : faceIds) {
		faceFunc(TS_REAL, faceId, [&edgeFaceMap](const Polygon& face) {
			const auto& verts = face.getVertexIds();
			for (size_t i = 0; i < verts.size(); i++) {
				size_t j = (i + 1) % verts.size();
				auto iter = edgeFaceMap.insert(std::make_pair(Edge(verts[i], verts[j]), MTC::set<Index3DId>())).first;
				iter->second.insert(face.getId());
				assert(iter->second.size() <= 2);
			}
		});
	}

	size_t numNewFaces = 0;
	while (!edgeFaceMap.empty()) {
		auto iter = edgeFaceMap.begin();
		auto edge = iter->first;
		auto faces = iter->second;
		edgeFaceMap.erase(iter);
		MTC::set<Index3DId> cellFaces;
		MTC::vector<Index3DId> stack;
		stack.insert(stack.end(), faces.begin(), faces.end());
		while (!stack.empty() && !edgeFaceMap.empty()) {
			auto faceId = stack.back();
			stack.pop_back();

			faceFunc(TS_REAL, faceId, [&edgeFaceMap, &cellFaces, &stack](const Polygon& face) {
				const auto& verts = face.getVertexIds();
				for (size_t i = 0; i < verts.size(); i++) {
					size_t j = (i + 1) % verts.size();
					auto iter = edgeFaceMap.find(Edge(verts[i], verts[j]));
					if (iter != edgeFaceMap.end()) {
						auto& edgeFaces = iter->second;
						for (const auto& edgeFace : edgeFaces) {
							if (!cellFaces.contains(edgeFace)) {
								cellFaces.insert(edgeFace);
								stack.push_back(edgeFace);
							}
						}

						edgeFaceMap.erase(iter);
					}
				}
			});
		}

		if (!cellFaces.empty()) {
#if 0
			auto newCellId = getBlockPtr()->addCell(cellFaces);
			newCellIds.insert(newCellId);
#else
			MTC::set<Index3DId> tmp;
			if (createConvexCells(cellFaces, tmp)) {
				numNewFaces += tmp.size();
				newCellIds.insert(tmp.begin(), tmp.end());
			}
#endif
		}
	}

	return numNewFaces > 0;
}

bool PolyhedronSplitter::createConvexCells(const MTC::set<Index3DId>& cellFacesIn, MTC::set<Index3DId>& newCellIds)
{
	MTC::set<Index3DId> cellFaces(cellFacesIn);

	Polyhedron cell(cellFaces);

	cell.setId(getBlockPtr(), 10000);
	cell.attachFaces();
	assert(cell.isClosed());
	cell.orientFaces();

	const auto edges = cell.getEdges(false);
	MTC::set<Edge> convexEdges, concaveEdges;
	cell.classifyEdges(convexEdges, concaveEdges);
	if (!concaveEdges.empty()) {
		string filename = "ocf_" + getBlockPtr()->getLoggerNumericCode() + "_" + to_string(_polyhedronId.elementId());
		getBlockPtr()->dumpPolygonObj(filename, cell.getFaceIds(), cell.getId());
		filename = "ce_" + getBlockPtr()->getLoggerNumericCode() + "_" + to_string(cell.getId().elementId());
		getBlockPtr()->dumpEdgeObj(filename, concaveEdges);
		for (const auto& edge : concaveEdges) {
			splitCellAtConcaveEdge(cell, edge, newCellIds);
		}
		cell.detachFaces();
		return true;
	}
	
	cell.detachFaces();
	auto newCellId = getBlockPtr()->addCell(cellFaces);
	newCellIds.insert(newCellId);
	return true;
}

void PolyhedronSplitter::splitCellAtConcaveEdge(const Polyhedron& cell, const Edge& edge, MTC::set<Index3DId>& newCellIds)
{
	assert(!edge.isConvex(getBlockPtr(), cell.getId()));
	double angle = edge.calDihedralAngleRadians(getBlockPtr(), cell.getId());
	if (angle > 3 / 4.0 * M_PI) {
		splitConcaveEdgeSingle(cell, edge, newCellIds);
	} else {
		splitConcaveEdgeDouble(cell, edge, newCellIds);
	}
}

void PolyhedronSplitter::splitConcaveEdgeSingle(const Polyhedron& cell, const Edge& edge, MTC::set<Index3DId>& newCellIds)
{

}

void PolyhedronSplitter::splitConcaveEdgeDouble(const Polyhedron& cell, const Edge& edge, MTC::set<Index3DId>& newCellIds)
{
	const double distTol = Tolerance::sameDistTol();
	auto iter = edge.getFaceIds().begin();

	const auto& faceId0 = *iter++;
	const auto& faceId1 = *iter;

	Vector3d norm0, keepDir0, norm1, keepDir1, edgeDir;
	faceFunc(TS_REAL, faceId0, [this, &cell, &norm0, &keepDir0, &edge, &edgeDir](const Polygon& face) {
		norm0 = face.calOrientedUnitNormal(cell.getId());
		norm0 = choosePrincipalAxis(norm0);
		bool found = false;
		face.iterateEdges([this, &edge, &edgeDir, &found](const Edge& fe) {
			if (edge == fe) {
				edgeDir = fe.calUnitDir(getBlockPtr());
				found = true;
			}
			return !found;
		});
		assert(found);

		keepDir0 = norm0.cross(edgeDir);
	});

	faceFunc(TS_REAL, faceId1, [&cell, &norm0, &norm1, &keepDir1, &edgeDir](const Polygon& face) {
		norm1 = face.calOrientedUnitNormal(cell.getId());
		norm1 = choosePrincipalAxis(norm1, norm0);
		norm1 = (norm1 - norm1.dot(edgeDir) * edgeDir).normalized();
		keepDir1 = norm1.cross(edgeDir);
	});

	norm0 = (norm0 - norm0.dot(edgeDir) * edgeDir).normalized();
	norm1 = (norm1 - norm1.dot(edgeDir) * edgeDir).normalized();
	assert(norm0.cross(norm1).norm() > 0);

	const auto& faceIds = cell.getFaceIds();

	MTC::set<Index3DId> partingFaceIds;
	Index3DId partingFaceId = createPartingFace(cell, edge, norm0, keepDir0);
	partingFaceIds.insert(partingFaceId);
	partingFaceId = createPartingFace(cell, edge, norm1, keepDir1);
	partingFaceIds.insert(partingFaceId);

#if 1
	{
		std::string filename = "ptf_" + getBlockPtr()->getLoggerNumericCode() + "_" + to_string(cell.getId().elementId());
		getBlockPtr()->dumpPolygonObj(filename, partingFaceIds);
		filename = "cellFaces_" + getBlockPtr()->getLoggerNumericCode() + "_" + to_string(cell.getId().elementId());
		getBlockPtr()->dumpPolygonObj(filename, faceIds, cell.getId());
	}
#endif

	MTC::set<Index3DId> availFaceIds;
	for (const auto& faceId : faceIds) {
		// TODO Move this to PolygonSplitter when it's working
		if (faceId == Index3DId(3, 9, 0, 71) || faceId == Index3DId(3, 9, 0, 73)) {
			int dbgBreak = 1;
			std::string filename = "cellFace_" + getBlockPtr()->getLoggerNumericCode() + "_" + to_string(faceId.elementId());
			MTC::vector<Index3DId> faceVec;
			faceVec.push_back(faceId);
			getBlockPtr()->dumpPolygonObj(filename, faceVec, cell.getId());
		}
		MTC::set<Edge> faceEdges;
		MTC::set<Edge> partingEdges;
		faceFunc(TS_REAL, faceId, [this, &faceEdges, &partingFaceIds, &partingEdges](const Polygon& face) {
			const auto& tmp = face.getEdges();
			faceEdges.insert(tmp.begin(), tmp.end());

			for (const auto& partingFaceId : partingFaceIds) {
				faceFunc(TS_REAL, partingFaceId, [this, &face, &faceEdges, &partingEdges](const Polygon& partingFace) {
					partingFace.iterateEdges([&face, &partingEdges](const Edge& pfe) {
						bool isUsed;
						if (face.contains(pfe, isUsed) && !isUsed)
							partingEdges.insert(pfe);
						return true;
					});
				});
			}
		});

		if (partingEdges.empty()) {
			// This face does not need to be split, but needs to be in the list
			availFaceIds.insert(faceId);
			continue;
		}

		if (partingEdges.size() > 1) {
			int dbgBreak = 1;
#if 1
			{
				std::string filename = "pte_" + getBlockPtr()->getLoggerNumericCode() + "_" + to_string(cell.getId().elementId());
				getBlockPtr()->dumpEdgeObj(filename, partingEdges);
			}
#endif
		}

		MTC::set<Index3DId> partingVertIds;
		for (const auto& e : partingEdges) {
			partingVertIds.insert(e.getVertex(0));
			partingVertIds.insert(e.getVertex(1));
		}

		MTC::set<Edge> newEdges;
		for (const auto& fe : faceEdges) {
			auto seg = fe.getSegment(getBlockPtr());
			double len = seg.calLength();
			bool hasSplit = false;
			for (const auto& vId : partingVertIds) {
				Vector3d pt = getBlockPtr()->getVertexPoint(vId);
				double t;
				if (seg.contains(pt, t, distTol)) {
					double interDist = t * len;
					if (distTol < interDist && interDist < len - distTol) {
						hasSplit = true;
						const auto& vertId0 = fe.getVertex(0);
						const auto& vertId1 = fe.getVertex(1);
						const auto& midVert = getBlockPtr()->addVertex(pt);
						newEdges.insert(Edge(vertId0, midVert));
						newEdges.insert(Edge(vertId1, midVert));
					}
				}
			}

			if (!hasSplit)
				newEdges.insert(fe);
		}
		assert(newEdges.size() >= faceEdges.size());
		if (newEdges.size() == faceEdges.size()) {
			availFaceIds.insert(faceId);
#if 1
			{
				std::string filename = "spf_" + getBlockPtr()->getLoggerNumericCode() + "_" + to_string(cell.getId().elementId());
				getBlockPtr()->dumpPolygonObj(filename, availFaceIds);
			}
#endif
		} else {
			newEdges.insert(partingEdges.begin(), partingEdges.end());
#if 1
			{
				std::string filename = "spe_" + getBlockPtr()->getLoggerNumericCode() + "_" + to_string(cell.getId().elementId());
				getBlockPtr()->dumpEdgeObj(filename, newEdges);
			}
#endif
			MTC::vector<MTC::vector<Index3DId>> faceVertices;
			if (PolygonSplitter::connectEdges(getBlockPtr(), newEdges, faceVertices)) {
				for (const auto& verts : faceVertices) {
					auto faceId = getBlockPtr()->addFace(verts);
					availFaceIds.insert(faceId);
#if 1
					{
						std::string filename = "spf_" + getBlockPtr()->getLoggerNumericCode() + "_" + to_string(cell.getId().elementId());
						getBlockPtr()->dumpPolygonObj(filename, availFaceIds);
					}
#endif
				}
			} else
				assert(!"Should not be possible");
		}
	} // end for (const auto& faceId


	for (const auto& partingFaceId : partingFaceIds) {
		MTC::set<Index3DId> faceIds;
		const Index3DId adjFaceId = findAdjacentFaceId(availFaceIds, partingFaceId);
		if (!adjFaceId.isValid())
			continue;

		faceIds.insert(partingFaceId);
		faceIds.insert(adjFaceId);
		availFaceIds.erase(adjFaceId);
		while (addAdjacentFaceToSet(availFaceIds, adjFaceId, faceIds))
			;
	}
}

Index3DId PolyhedronSplitter::findAdjacentFaceId(const MTC::set<Index3DId>& availFaceIds, const Index3DId& adjFaceId) const
{
	Index3DId result;

	return result;
}

bool PolyhedronSplitter::addAdjacentFaceToSet(MTC::set<Index3DId>& availFaceIds, const Index3DId& adjFaceId, MTC::set<Index3DId>& faceIds) const
{
	return false;
}

Index3DId PolyhedronSplitter::createPartingFace(const Polyhedron& cell, const Edge& edge, const Vector3d& norm, const Vector3d& keepDir)
{
	const double tol = Tolerance::sameDistTol();
	MTC::set<Edge> edges;
	edges.insert(edge);
	Vector3d origin = edge.calCenter(getBlockPtr());
	Planed cuttingPlane(origin, norm);
	Planed keepPlane(origin, keepDir);
	for (const auto& faceId : cell.getFaceIds()) {
		PolygonSplitter ps(getBlockPtr(), faceId);
		auto interEdge = ps.createIntersectionEdge(cuttingPlane, keepPlane);
		if (!interEdge.isValid())
			continue;

		Vector3d pt0 = getBlockPtr()->getVertexPoint(interEdge.getVertex(0));
		Vector3d pt1 = getBlockPtr()->getVertexPoint(interEdge.getVertex(1));
		double dist0 = keepPlane.distanceToPoint(pt0, false);
		double dist1 = keepPlane.distanceToPoint(pt1, false);
		if (-tol < dist0 && dist1 > -tol) {
			edges.insert(interEdge);
		}
	}

	Index3DId faceId;
	MTC::vector<MTC::vector<Index3DId>> faceVerts;
	if (PolygonSplitter::connectEdges(getBlockPtr(), edges, faceVerts)) {
		assert(faceVerts.size() == 1);
		faceId = getBlockPtr()->addFace(faceVerts.front());
	}

	return faceId;
}

bool PolyhedronSplitter::cutWithModelMeshInner(const BuildCFDParams& params, MTC::set<Index3DId>& deadCellIds, MTC::set<Index3DId>& newCellIds)
{
	std::vector<TriMesh::PatchPtr> patches;
	std::vector<std::vector<std::vector<size_t>>> allChains;
	if (!createModelMeshPatches(params, patches, allChains))
		return false;

	bool result = false;
	MTC::set<Index3DId> faceIds;
	_pBlock->cellFunc(TS_REAL, _polyhedronId, [&](Polyhedron& cell) {
		faceIds = cell.getFaceIds();

		if (testIds.contains(_polyhedronId)) {
			_pBlock->dumpPolyhedraObj({ _polyhedronId }, true, false, false);
		}

		if (splitWithSharpEdgePlanes(allChains, newCellIds)) {
			deadCellIds.insert(_polyhedronId);
			result = true;
			return;
		}

		cell.detachFaces();
		cutWithPatches(cell, patches, params, newCellIds);

		deadCellIds.insert(_polyhedronId);
		result = true;
	});

	return result;
}

bool PolyhedronSplitter::cutWithPatches(const Polyhedron& realCell, const std::vector<TriMesh::PatchPtr>& patches, const BuildCFDParams& params, MTC::set<Index3DId>& newCellIds)
{

	MTC::set<Index3DId> modelFaces;
	if (!createAllModelMeshFaces(patches, params, modelFaces))
		return false;
	{
		string filename = "amf " + getBlockPtr()->getLoggerNumericCode() + "_" + to_string(_polyhedronId.elementId());
		_pBlock->dumpPolygonObj(filename, modelFaces);
	}

	const auto& faceIds = realCell.getFaceIds();
	MTC::set<Index3DId> newFaceIds;

	for (const auto& faceId : faceIds) {
		PolygonSplitter ps(getBlockPtr(), faceId);
		MTC::set<Index3DId> tmp;
		ps.createTrimmedFacesFromFaces(modelFaces, tmp);
#if 0
		{
			string filename = "anf " + getBlockPtr()->getLoggerNumericCode() + "_face_" + to_string(faceId.elementId());
			_pBlock->dumpPolygonObj(filename, tmp);
	}
#endif
		newFaceIds.insert(tmp.begin(), tmp.end());
	}
	newFaceIds.insert(modelFaces.begin(), modelFaces.end());

	{
		string filename = "anf " + getBlockPtr()->getLoggerNumericCode() + "_" + to_string(_polyhedronId.elementId());
		_pBlock->dumpPolygonObj(filename, newFaceIds);
	}

	if (createCellsFromFaces(newFaceIds, newCellIds)) {
		MTC::vector<Index3DId> ids;
		ids.insert(ids.end(), newCellIds.begin(), newCellIds.end());
		getBlockPtr()->dumpPolyhedraObj(ids, false, false, false);
		newCellIds.erase(_polyhedronId);
		return true;
	}

	return false;
}

void PolyhedronSplitter::createFaceEdgesFromMeshFace(const Polyhedron& realCell, const std::vector<size_t>& modelFaceTris, const BuildCFDParams& params,
	MTC::set<Edge>& faceEdges) const
{
#if 0

	const double sinEdgeAngle = sin(params.getSharpAngleRadians());
	auto pMesh = getBlockPtr()->getModelMesh();

	std::vector<size_t> sharpEdges;
	for (size_t triIdx : modelFaceTris) {
		const auto& tri = pMesh->getTri(triIdx);
		for (int i = 0; i < 3; i++) {
			int j = (i + 1) % 3;
			size_t edgeIdx = pMesh->findEdge(TriMesh::CEdge(tri[i], tri[j]));
			if (pMesh->isEdgeSharp(edgeIdx, sinEdgeAngle)) {
				sharpEdges.push_back(edgeIdx);
			}
		}
	}


	// This complexity is to store the pierce edges for later use.
	MTC::set<Edge> tmpPierceEdges;
	createPierceEdges(realCell, sharpEdges, tmpPierceEdges);
	faceEdges.insert(tmpPierceEdges.begin(), tmpPierceEdges.end());

	auto faceIds = realCell.getFaceIds();
	for (const auto& faceId : faceIds) {
		faceFunc(TS_REAL, faceId, [this, &pMesh, &modelFaceTris, &sharpEdges, &faceEdges](const Polygon& face) {
			const double tol = Tolerance::sameDistTol();
			const auto& vertIds = face.getVertexIds();
			MTC::set<Index3DId> interVerts;
			for (size_t i = 0; i < vertIds.size(); i++) {
				size_t j = (i + 1) % vertIds.size();
				const auto& vertId0 = vertIds[i];
				const auto& vertId1 = vertIds[j];
				Vector3d pt0 = _pBlock->getVertexPoint(vertId0);
				Vector3d pt1 = _pBlock->getVertexPoint(vertId1);

				LineSegmentd seg(pt0, pt1);
				for (size_t triIdx : modelFaceTris) {
					RayHitd hit;
					if (pMesh->intersectsTri(seg, triIdx, tol, hit)) {
						auto vertId = _pBlock->addVertex(hit.hitPt);
						interVerts.insert(vertId);
					}
				}
			}

			if (interVerts.size() == 1) {
				auto pierceVert = createPierceVertex(face, sharpEdges);
				if (pierceVert.isValid()) {
					const auto& vertId0 = *interVerts.begin();
					if (vertId0 != pierceVert)
						faceEdges.insert(Edge(vertId0, pierceVert));
				}
			}
			else if (interVerts.size() == 2) {
				auto iter = interVerts.begin();
				const auto& vertId0 = *iter++;
				const auto& vertId1 = *iter++;
				if (vertId0 != vertId1)
					faceEdges.insert(Edge(vertId0, vertId1));
			}
			});
	}
#endif
}

void PolyhedronSplitter::createPierceEdges(const Polyhedron& realCell, const std::vector<size_t>& sharpEdges, MTC::set<Edge>& pierceEdges) const
{
#if 0

	auto pMesh = _pBlock->getModelMesh();
	const auto& faceIds = realCell.getFaceIds();
	MTC::set<Index3DId> intVerts;
	for (size_t edgeIdx : sharpEdges) {
		const auto& edge = pMesh->getEdge(edgeIdx);
		auto seg = edge.getSeg(pMesh);
		for (const auto& faceId : faceIds) {
			faceFunc(TS_REAL, faceId, [this, &seg, &intVerts](const Polygon& face) {
				RayHitd hit;
				if (face.intersect(seg, hit)) {
					auto vertId = _pBlock->addVertex(hit.hitPt);
					intVerts.insert(vertId);
				}
			});
		}
	}

	if (intVerts.size() == 2) {
		auto iter = intVerts.begin();
		const auto& vert0 = *iter++;
		const auto& vert1 = *iter++;
		Edge edge(vert0, vert1);
		pierceEdges.insert(edge);
	}
	
#endif
}

bool PolyhedronSplitter::findPiercePoint(const Polygon& face, const std::vector<size_t>& pierceChain, Vector3d& pt) const
{
#if 0
	auto pMesh = _pBlock->getModelMesh();

	for (size_t edgeIdx : pierceChain) {
		const auto& triEdge = pMesh->getEdge(edgeIdx);
		auto seg = triEdge.getSeg(pMesh);
		RayHitd hit;
		if (face.intersect(seg, hit)) {
			pt = hit.hitPt;
			return true;
		}
	}
#endif
	return false;
}

bool PolyhedronSplitter::findPiercePoints(const std::vector<std::vector<size_t>>& faceEdgeChains, std::vector<Vector3d>& pts) const
{
	MTC::set<Index3DId> faceIds;
	cellFunc(TS_REAL, _polyhedronId, [&faceIds](const Polyhedron& cell) {
		faceIds = cell.getFaceIds();
	});

	for (const auto& faceId : faceIds) {
		faceAvailFunc(TS_REAL, faceId, [this, &faceEdgeChains, &pts](const Polygon& face) {
			for (const auto& edgeChain : faceEdgeChains) {
				Vector3d pt;
				if (findPiercePoint(face, edgeChain, pt))
					pts.push_back(pt);
			}
		});
	}

	return !pts.empty();
}

bool PolyhedronSplitter::findPiercePoints(const std::vector<std::vector<std::vector<size_t>>>& allFaceEdgeChains, std::vector<Vector3d>& pts) const
{
	for (const auto& faceEdgeChains : allFaceEdgeChains)
		findPiercePoints(faceEdgeChains, pts);

	return !pts.empty();
}

Index3DId PolyhedronSplitter::createPierceVertex(const Polygon& face, const std::vector<size_t>& pierceChain) const
{
	Vector3d pt;
	if (findPiercePoint(face, pierceChain, pt)) {
		Index3DId vertId = _pBlock->addVertex(pt);
		return vertId;
	}

	return Index3DId();
}

bool PolyhedronSplitter::facesFormClosedCell(const MTC::set<Index3DId>& faceIds) const
{
	MTC::map<Edge, size_t> edgeMap;

	for (const auto& faceId : faceIds) {
		_pBlock->faceFunc(TS_REAL, faceId, [&edgeMap](const Polygon& face) {
			const auto& verts = face.getVertexIds();
			for (size_t i = 0; i < verts.size(); i++) {
				size_t j = (i + 1) % verts.size();
				Edge e(verts[i], verts[j]);
				auto iter = edgeMap.find(e);
				if (iter == edgeMap.end()) {
					iter = edgeMap.insert(std::make_pair(e, 0)).first;
				}
				iter->second++;
			}
		});
	}

	for (const auto& iter : edgeMap) {
		if (iter.second != 2)
			return false;
	}

	return true;
}

void PolyhedronSplitter::vertexFunc(const Index3DId& id, const function<void(const Vertex& obj)>& func) const {
	auto p = getBlockPtr(); 
	p->vertexFunc(id, func);
} 

//LAMBDA_CLIENT_IMPLS(PolyhedronSplitter)

void PolyhedronSplitter::vertexFunc(const Index3DId& id, const function<void(Vertex& obj)>& func) {
	auto p = getBlockPtr(); 
	p->vertexFunc(id, func);
} 

void PolyhedronSplitter::faceFunc(TopolgyState state, const Index3DId& id, const function<void(const Polygon& obj)>& func) const {
	auto p = getBlockPtr(); 
	p->faceFunc(state, id, func);
} 

void PolyhedronSplitter::faceFunc(TopolgyState state, const Index3DId& id, const function<void(Polygon& obj)>& func) {
	auto p = getBlockPtr(); 
	p->faceFunc(state, id, func);
} 

void PolyhedronSplitter::cellFunc(TopolgyState state, const Index3DId& id, const function<void(const Polyhedron& obj)>& func) const {
	auto p = getBlockPtr(); 
	p->cellFunc(state, id, func);
} 

void PolyhedronSplitter::cellFunc(TopolgyState state, const Index3DId& id, const function<void(Polyhedron& obj)>& func) {
	auto p = getBlockPtr(); 
	p->cellFunc(state, id, func);
} 

void PolyhedronSplitter::faceAvailFunc(TopolgyState prefState, const Index3DId& id, const function<void(const Polygon& obj)>& func) const {
	auto p = getBlockPtr(); 
	p->faceAvailFunc(prefState, id, func);
} 

void PolyhedronSplitter::cellAvailFunc(TopolgyState prefState, const Index3DId& id, const function<void(const Polyhedron& obj)>& func) const {
	auto p = getBlockPtr(); 
	p->cellAvailFunc(prefState, id, func);
}
