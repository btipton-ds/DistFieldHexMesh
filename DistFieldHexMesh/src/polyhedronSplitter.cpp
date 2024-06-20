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

void PolyhedronSplitter::sortNewFacePoints(const Vector3d& tipPt, const Vector3d& xAxis, const Vector3d& yAxis, MTC::vector<Vector3d>& points) const
{
	if (points.size() > 1) {
		sort(points.begin(), points.end(), [&tipPt, &xAxis, &yAxis](const Vector3d& lhs, const Vector3d& rhs)->bool {
			Vector3d v0 = (lhs - tipPt).normalized();
			Vector3d v1 = (rhs - tipPt).normalized();

			double cosT0 = v0.dot(xAxis);
			double sinT0 = v0.dot(yAxis);
			double theta0 = atan2(sinT0, cosT0);

			double cosT1 = v1.dot(xAxis);
			double sinT1 = v1.dot(yAxis);
			double theta1 = atan2(sinT1, cosT1);

			return theta0 < theta1;
		});
	}
}


bool PolyhedronSplitter::createAllPatchFaces(const std::vector<TriMesh::PatchPtr>& patches, const BuildCFDParams& params, MTC::set<Index3DId>& modelFaces)
{
	cellFunc(TS_REAL, _polyhedronId, [&](const Polyhedron& cell) {
		for (const auto& pPatch : patches) {
			for (const auto& modelFaceTris : pPatch->getFaces()) {
				MTC::set<Edge> faceEdges;
				createFaceEdgesFromMeshFace(cell, modelFaceTris, params, faceEdges);
				MTC::vector<Index3DId> verts;
				if (PolygonSplitter::connectEdges(getBlockPtr(), faceEdges, verts)) {
					Vector3d modelFaceNorm = calModelFaceNormal(modelFaceTris);
					Vector3d faceNorm = Polygon::calUnitNormalStat(getBlockPtr(), verts);
					if (faceNorm.dot(modelFaceNorm) < 0)
						std::reverse(verts.begin(), verts.end());

					Index3DId faceId = getBlockPtr()->addFace(verts);
					modelFaces.insert(faceId);
				}
			}
		}
	});

	return !modelFaces.empty();
}

Vector3d PolyhedronSplitter::calModelFaceNormal(const std::vector<size_t>& modelFaceTris) const
{
	auto pMesh = getBlockPtr()->getModelMesh();
	Vector3d normal(0, 0, 0);
	for (size_t triIdx : modelFaceTris)
		normal += pMesh->triUnitNormal(triIdx);
	normal.normalize();
	return normal;
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
			MTC::vector<Index3DId> vertices;
			if (PolygonSplitter::connectEdges(getBlockPtr(), posEdges, vertices)) {
				Index3DId faceId = getBlockPtr()->addFace(vertices);
				posFaceIds.insert(faceId);
			}
		}
		if (!negEdges.empty()) {
			MTC::vector<Index3DId> vertices;
			if (PolygonSplitter::connectEdges(getBlockPtr(), negEdges, vertices)) {
				Index3DId faceId = getBlockPtr()->addFace(vertices);
				negFaceIds.insert(faceId);
			}
		}
	}

	MTC::vector<Index3DId> vertices;
	if (!PolygonSplitter::connectEdges(getBlockPtr(), splitEdges, vertices))
		return false;

	Index3DId splitFaceId = getBlockPtr()->addFace(vertices);
	if (!splitFaceId.isValid())
		return false;

	posFaceIds.insert(splitFaceId);
	negFaceIds.insert(splitFaceId);

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
		Index3DId(0, 12, 0, 9),
		Index3DId(0, 12, 0, 49),
		Index3DId(0, 12, 0, 50),
	};
}

bool PolyhedronSplitter::cutWithModelMesh(const BuildCFDParams& params)
{
	auto pMesh = _pBlock->getModelMesh();
	std::vector<size_t> tris;
	MTC::set<Index3DId> faceIds;
	_pBlock->cellFunc(TS_REAL, _polyhedronId, [this, &faceIds, &tris](const Polyhedron& cell) {
		tris = cell.getTriIndices();
		faceIds = cell.getFaceIds();
	});

	if (tris.empty())
		return false;

	std::vector<TriMesh::PatchPtr> patches;
	double sinSharpAngle = sin(params.getSharpAngleRadians());
	if (!pMesh->createPatches(tris, sinSharpAngle, patches))
		return false;

	MTC::set<Index3DId> modelFaces;
	if (!createAllPatchFaces(patches, params, modelFaces))
		return false;
	{
		string filename = "amf " + getBlockPtr()->getLoggerNumericCode() + "_" + to_string(_polyhedronId.elementId());
		_pBlock->dumpPolygonObj(filename, modelFaces);
	}

	MTC::vector<Index3DId> newFaceIds;
	for (const auto& faceId : faceIds) {
		PolygonSplitter ps(getBlockPtr(), faceId);
		ps.createTrimmedFacesFromFaces(modelFaces, newFaceIds);
	}

	std::vector<std::vector<std::vector<size_t>>> allChains;
	for (size_t i = 0; i < patches.size(); i++) {
		auto pPatch = patches[i];
		std::vector<std::vector<size_t>> faceChains;
		for (const auto& sec : pPatch->getSharpEdgeChains()) {
			faceChains.push_back(sec);
		}
		allChains.push_back(faceChains);
	}

	MTC::set<Index3DId> deadCellIds, newCellIds;

	if (!cutWithModelMeshInner(params, patches, allChains, deadCellIds, newCellIds))
		return false;

	bool result = false;
	for (const auto& newCellId : newCellIds) {
		MTC::set<Index3DId> tmpNewCellIds;
		PolyhedronSplitter ps(getBlockPtr(), newCellId);
		if (ps.cutWithModelMeshInner(params, patches, allChains, deadCellIds, tmpNewCellIds))
			result = true;
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

bool PolyhedronSplitter::cutWithModelMeshInner(const BuildCFDParams& params, const std::vector<TriMesh::PatchPtr>& patches,
	std::vector<std::vector<std::vector<size_t>>>& allChains, MTC::set<Index3DId>& deadCellIds, MTC::set<Index3DId>& newCellIds)
{
	bool result = false;
	_pBlock->cellFunc(TS_REAL, _polyhedronId, [&](Polyhedron& cell) {
		if (testIds.contains(_polyhedronId)) {
			_pBlock->dumpPolyhedraObj({ _polyhedronId }, true, false, false);
		}

		std::vector<Vector3d> piercePoints;
		if (findPiercePoints(allChains, piercePoints)) {
			Planed plane;
			double err;
			if (piercePoints.size() > 2 && bestFitPlane(piercePoints, plane, err) && err < Tolerance::sameDistTol()) {
				PolyhedronSplitter subSplitter(_pBlock, _polyhedronId);
				if (subSplitter.splitWithPlane(plane, newCellIds)) {
					allChains.clear();
					deadCellIds.insert(_polyhedronId);
					result = true;
					return;
				}
			}
		}

		for (size_t i = 0; i < patches.size(); i++) {
			auto pPatch = patches[i];
			std::vector<Vector3d> piercePoints;
			if (!allChains.empty() && !allChains[i].empty() && findPiercePoints(allChains[i], piercePoints)) {
				Planed plane;
				double err;
				if (piercePoints.size() > 2 && bestFitPlane(piercePoints, plane, err) && err < Tolerance::sameDistTol()) {
					PolyhedronSplitter subSplitter(_pBlock, _polyhedronId);
					if (subSplitter.splitWithPlane(plane, newCellIds)) {
						allChains[i].clear();
						deadCellIds.insert(_polyhedronId);
						result = true;
						return;
					}
				}
			}

			cell.detachFaces();
			cutWithPatch(cell, patches, params, i);
		}

		deadCellIds.insert(_polyhedronId);
		result = true;
	});

	return result;
}

void PolyhedronSplitter::cutWithPatch(const Polyhedron& realCell, const std::vector<TriMesh::PatchPtr>& patches, const BuildCFDParams& params, size_t idx)
{
	auto pMesh = _pBlock->getModelMesh();
	const auto& faceIds = realCell.getFaceIds();
	MTC::vector<MTC::set<Edge>> patchEdges;
	MTC::set<Index3DId> patchFaces, trimFaces, allFaces;
	const auto pPatch = patches[idx];

	MTC::set<Index3DId> skippedVerts;
	createAllPatchFaceEdges_deprecated(realCell, pPatch, params, skippedVerts, patchEdges);

	for (const auto& faceEdges : patchEdges) {
		MTC::vector<Index3DId> interVerts;
		if (PolygonSplitter::connectEdges(_pBlock, faceEdges, interVerts)) {
			Index3DId newFaceId = _pBlock->addFace(interVerts);
			patchFaces.insert(newFaceId);
		}
	}

	if (patchFaces.empty())
		return;

	string filename;
	if (testIds.contains(_polyhedronId) && !patchFaces.empty()) {
		filename = "mpf_" + to_string(idx) + " " + Block::getLoggerNumericCode(_polyhedronId);
		_pBlock->dumpPolygonObj(filename, patchFaces);
	}

	for (const auto& faceId : realCell.getFaceIds()) {
		PolygonSplitter ps(_pBlock, faceId);
		Index3DId newFaceId;
		if (ps.createTrimmedFace(patchEdges, skippedVerts, newFaceId))
			trimFaces.insert(newFaceId);
	}

	if (testIds.contains(_polyhedronId) && !trimFaces.empty()) {
		filename = "trf_" + to_string(idx) + " " + Block::getLoggerNumericCode(_polyhedronId);
		_pBlock->dumpPolygonObj(filename, trimFaces);
	}

	allFaces.insert(trimFaces.begin(), trimFaces.end());
	allFaces.insert(patchFaces.begin(), patchFaces.end());

	if (testIds.contains(_polyhedronId) && !allFaces.empty()) {
		filename = "acf_" + to_string(idx) + " " + Block::getLoggerNumericCode(_polyhedronId);
		_pBlock->dumpPolygonObj(filename, allFaces);
	}

	Index3DId newCellId = _pBlock->addCell(allFaces);
	cellFunc(TS_REAL, newCellId, [&realCell](Polyhedron& newCell) {
		newCell.setEdgeIndices(realCell.getEdgeIndices());
		newCell.setTriIndices(realCell.getTriIndices());
	});
}

void PolyhedronSplitter::createModelFaceInterectionEdges(const Index3DId& faceId, const std::vector<size_t>& modFaceTris, const BuildCFDParams& params,
	MTC::set<IntersectEdge>& faceEdges)
{
	faceFunc(TS_REAL, faceId, [this, &modFaceTris, &params, &faceEdges](const Polygon& face) {
		auto pMesh = getBlockPtr()->getModelMesh();
		const auto& vertIds = face.getVertexIds();
		MTC::set<IntersectVertId> interVerts;
		for (size_t i = 0; i < vertIds.size(); i++) {
			size_t j = (i + 1) % vertIds.size();
			const auto& vertId0 = vertIds[i];
			const auto& vertId1 = vertIds[j];
			Vector3d pt0 = _pBlock->getVertexPoint(vertId0);
			Vector3d pt1 = _pBlock->getVertexPoint(vertId1);

			LineSegmentd seg(pt0, pt1);
			for (size_t triIdx : modFaceTris) {
				RayHitd hit;
				if (pMesh->intersectsTri(seg, triIdx, hit)) {
					auto vertId = _pBlock->addVertex(hit.hitPt);
					interVerts.insert(IntersectVertId(vertId, triIdx));
				}
			}
		}

		MTC::set<Index3DId> pierceVerts;
		const double sinEdgeAngle = sin(params.getSharpAngleRadians());
		for (size_t triIdx : modFaceTris) {
			const auto& tri = pMesh->getTri(triIdx);
			for (int i = 0; i < 3; i++) {
				int j = (i + 1) % 3;
				size_t edgeIdx = pMesh->findEdge(TriMesh::CEdge(tri[i], tri[j]));
				if (edgeIdx != -1) {
					if (pMesh->isEdgeSharp(edgeIdx, sinEdgeAngle)) {
						const auto& edge = pMesh->getEdge(edgeIdx);
						auto seg = edge.getSeg(pMesh);
						RayHitd hit;
						if (face.intersect(seg, hit)) {
							auto pierceVert = getBlockPtr()->addVertex(hit.hitPt);
							pierceVerts.insert(pierceVert);
						}
					}
					const auto& edge = pMesh->getEdge(edgeIdx);
				}
			}
		}

		if (pierceVerts.size() == 2) {
			auto iter = pierceVerts.begin();

			auto pierceVert0 = IntersectVertId(*iter++, -1);
			auto pierceVert1 = IntersectVertId(*iter++, -1);
			faceEdges.insert(IntersectEdge(pierceVert0, pierceVert1));
		} else if (interVerts.size() == 1) {
			assert(pierceVerts.size() == 1);
			auto pierceVert = IntersectVertId(*pierceVerts.begin(), -1);

			if (pierceVert.isValid()) {
				const auto& vertId0 = *interVerts.begin();
				if (vertId0 != pierceVert)
					faceEdges.insert(IntersectEdge(vertId0, pierceVert));
			}
		}
		else if (interVerts.size() == 2) {
			auto iter = interVerts.begin();
			const auto& vertId0 = *iter++;
			const auto& vertId1 = *iter++;
			if (vertId0 != vertId1)
				faceEdges.insert(IntersectEdge(vertId0, vertId1));
		}
	});
}

void PolyhedronSplitter::createAllFaceEdges(const Polyhedron& realCell, const std::vector<size_t>& modFaceTris, const BuildCFDParams& params,
	MTC::set<Index3DId>& skippedVerts, MTC::set<Edge>& faceEdges) const
{
	auto pMesh = getBlockPtr()->getModelMesh();
	const double sinEdgeAngle = sin(params.getSharpAngleRadians());

	std::vector<size_t> sharpEdges;
	for (size_t triIdx : modFaceTris) {
		const auto& tri = pMesh->getTri(triIdx);
		for (int i = 0; i < 3; i++) {
			int j = (i + 1) % 3;
			size_t edgeIdx = pMesh->findEdge(TriMesh::CEdge(tri[i], tri[j]));
			if (pMesh->isEdgeSharp(edgeIdx, sinEdgeAngle)) {
				sharpEdges.push_back(edgeIdx);
			}
		}
	}

	createPierceEdges(realCell, sharpEdges, faceEdges);

	auto faceIds = realCell.getFaceIds();
	for (const auto& faceId : faceIds) {
		faceFunc(TS_REAL, faceId, [this, &pMesh, &modFaceTris, &sharpEdges, &skippedVerts, &faceEdges](const Polygon& face) {
			const auto& vertIds = face.getVertexIds();
			MTC::set<IntersectVertId> interVerts;
			for (size_t i = 0; i < vertIds.size(); i++) {
				size_t j = (i + 1) % vertIds.size();
				const auto& vertId0 = vertIds[i];
				const auto& vertId1 = vertIds[j];
				Vector3d pt0 = _pBlock->getVertexPoint(vertId0);
				Vector3d pt1 = _pBlock->getVertexPoint(vertId1);

				LineSegmentd seg(pt0, pt1);
				for (size_t triIdx : modFaceTris) {
					RayHitd hit;
					if (pMesh->intersectsTri(seg, triIdx, hit)) {
						auto vertId = _pBlock->addVertex(hit.hitPt);
						interVerts.insert(IntersectVertId(vertId, triIdx));
						Vector3d modNorm = pMesh->triUnitNormal(triIdx);
						Vector3d v = pt0 - hit.hitPt;
						if (modNorm.dot(v) < 0)
							skippedVerts.insert(vertId0);
						else
							skippedVerts.insert(vertId1);
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
}

void PolyhedronSplitter::createAllPatchFaceEdges_deprecated(const Polyhedron& realCell, const TriMesh::PatchPtr& pPatch, const BuildCFDParams& params,
	MTC::set<Index3DId>& skippedVerts, MTC::vector<MTC::set<Edge>>& patchEdges) const
{
	const double sinEdgeAngle = sin(params.getSharpAngleRadians());
	auto pMesh = getBlockPtr()->getModelMesh();
	const auto& faces = pPatch->getFaces();
	for (const auto& modelFace : faces) {
		std::vector<size_t> sharpEdges;
		for (size_t triIdx : modelFace) {
			const auto& tri = pMesh->getTri(triIdx);
			for (int i = 0; i < 3; i++) {
				int j = (i + 1) % 3;
				size_t edgeIdx = pMesh->findEdge(TriMesh::CEdge(tri[i], tri[j]));
				if (pMesh->isEdgeSharp(edgeIdx, sinEdgeAngle)) {
					sharpEdges.push_back(edgeIdx);
				}
			}
		}

		MTC::set<Edge> cutEdges;
		createPierceEdges(realCell, sharpEdges, cutEdges);

		auto faceIds = realCell.getFaceIds();
		for (const auto& faceId : faceIds) {
			faceFunc(TS_REAL, faceId, [this, &pMesh, &modelFace, &sharpEdges, &skippedVerts, &cutEdges](const Polygon& face) {
				const auto& vertIds = face.getVertexIds();
				MTC::set<IntersectVertId> interVerts;
				for (size_t i = 0; i < vertIds.size(); i++) {
					size_t j = (i + 1) % vertIds.size();
					const auto& vertId0 = vertIds[i];
					const auto& vertId1 = vertIds[j];
					Vector3d pt0 = _pBlock->getVertexPoint(vertId0);
					Vector3d pt1 = _pBlock->getVertexPoint(vertId1);

					LineSegmentd seg(pt0, pt1);
					for (size_t triIdx : modelFace) {
						RayHitd hit;
						if (pMesh->intersectsTri(seg, triIdx, hit)) {
							auto vertId = _pBlock->addVertex(hit.hitPt);
							interVerts.insert(IntersectVertId(vertId, triIdx));
							Vector3d modNorm = pMesh->triUnitNormal(triIdx);
							Vector3d v = pt0 - hit.hitPt;
							if (modNorm.dot(v) < 0)
								skippedVerts.insert(vertId0);
							else
								skippedVerts.insert(vertId1);
						}
					}
				}

				if (interVerts.size() == 1) {
					auto pierceVert = createPierceVertex(face, sharpEdges);
					if (pierceVert.isValid()) {
						const auto& vertId0 = *interVerts.begin();
						if (vertId0 != pierceVert)
							cutEdges.insert(Edge(vertId0, pierceVert));
					}
				}
				else if (interVerts.size() == 2) {
					auto iter = interVerts.begin();
					const auto& vertId0 = *iter++;
					const auto& vertId1 = *iter++;
					if (vertId0 != vertId1)
						cutEdges.insert(Edge(vertId0, vertId1));
				}
				});
		}

		if (!cutEdges.empty())
			patchEdges.push_back(cutEdges);
	}
}

void PolyhedronSplitter::createFaceEdgesFromMeshFace(const Polyhedron& realCell, const std::vector<size_t>& modelFaceTris, const BuildCFDParams& params,
	MTC::set<Edge>& patchEdges) const
{
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

	createPierceEdges(realCell, sharpEdges, patchEdges);

	auto faceIds = realCell.getFaceIds();
	for (const auto& faceId : faceIds) {
		faceFunc(TS_REAL, faceId, [this, &pMesh, &modelFaceTris, &sharpEdges, &patchEdges](const Polygon& face) {
			const auto& vertIds = face.getVertexIds();
			MTC::set<IntersectVertId> interVerts;
			for (size_t i = 0; i < vertIds.size(); i++) {
				size_t j = (i + 1) % vertIds.size();
				const auto& vertId0 = vertIds[i];
				const auto& vertId1 = vertIds[j];
				Vector3d pt0 = _pBlock->getVertexPoint(vertId0);
				Vector3d pt1 = _pBlock->getVertexPoint(vertId1);

				LineSegmentd seg(pt0, pt1);
				for (size_t triIdx : modelFaceTris) {
					RayHitd hit;
					if (pMesh->intersectsTri(seg, triIdx, hit)) {
						auto vertId = _pBlock->addVertex(hit.hitPt);
						interVerts.insert(IntersectVertId(vertId, triIdx));
					}
				}
			}

			if (interVerts.size() == 1) {
				auto pierceVert = createPierceVertex(face, sharpEdges);
				if (pierceVert.isValid()) {
					const auto& vertId0 = *interVerts.begin();
					if (vertId0 != pierceVert)
						patchEdges.insert(Edge(vertId0, pierceVert));
				}
			}
			else if (interVerts.size() == 2) {
				auto iter = interVerts.begin();
				const auto& vertId0 = *iter++;
				const auto& vertId1 = *iter++;
				if (vertId0 != vertId1)
					patchEdges.insert(Edge(vertId0, vertId1));
			}
			});
	}

}

void PolyhedronSplitter::createPierceEdges(const Polyhedron& realCell, const std::vector<size_t>& sharpEdges, MTC::set<Edge>& pierceEdges) const
{
	auto pMesh = _pBlock->getModelMesh();
	const auto& faceIds = realCell.getFaceIds();
	MTC::set<IntersectVertId> intVerts;
	for (size_t edgeIdx : sharpEdges) {
		const auto& edge = pMesh->getEdge(edgeIdx);
		auto seg = edge.getSeg(pMesh);
		for (const auto& faceId : faceIds) {
			faceFunc(TS_REAL, faceId, [this, &seg, &intVerts](const Polygon& face) {
				RayHitd hit;
				if (face.intersect(seg, hit)) {
					auto vertId = _pBlock->addVertex(hit.hitPt);
					intVerts.insert(IntersectVertId(vertId, hit));
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
	

}

bool PolyhedronSplitter::findPiercePoint(const Polygon& face, const std::vector<size_t>& pierceChain, Vector3d& pt) const
{
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

IntersectVertId PolyhedronSplitter::createPierceVertex(const Polygon& face, const std::vector<size_t>& pierceChain) const
{
	Vector3d pt;
	if (findPiercePoint(face, pierceChain, pt)) {
		Index3DId vertId = _pBlock->addVertex(pt);
		return IntersectVertId(vertId, -1);
	}

	return IntersectVertId();
}

bool PolyhedronSplitter::edgePointOutsidePatch(const Index3DId& vert0, const Index3DId& vert1, const Vector3d& pt, const TriMesh::PatchPtr& pPatch) const
{
	Vector3d pt0 = _pBlock->getVertexPoint(vert0);
	Vector3d pt1 = _pBlock->getVertexPoint(vert1);
	Vector3d v0 = (pt1 - pt0).normalized();
	LineSegment seg(pt0, pt1);
	auto pMesh = _pBlock->getModelMesh();
	double minDist = DBL_MAX;
	for (const auto& face : pPatch->getFaces()) {
		for (size_t triIdx : face) {
			RayHitd hit;
			if (pMesh->intersectsTri(seg, triIdx, hit)) {
				Vector3d v1 = hit.hitPt - pt0;
				double dist = v1.dot(v0);
				auto n = pMesh->triUnitNormal(triIdx);
				double dp = -v1.dot(n);
				if (dp >= -Tolerance::sameDistTol() && dist < minDist) {
					minDist = dist;
				}

			}
		}
	}

	return minDist != DBL_MAX;
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

void PolyhedronSplitter::findSharpVertPierecPoints(size_t vertIdx, MTC::vector<Vector3d>& piercePoints, const BuildCFDParams& params) const
{
	const double sinSharpAngle = sin(params.getSharpAngleRadians());
	auto pMesh = _pBlock->getModelMesh();

	piercePoints.clear();

	set<size_t> availEdges;
	MTC::set<Index3DId> faceIds;
	cellFunc(TS_REAL, _polyhedronId, [&availEdges, &faceIds](const Polyhedron& cell) {
		const auto& edgeIndices = cell.getEdgeIndices();
		availEdges.insert(edgeIndices.begin(), edgeIndices.end());
		faceIds = cell.getFaceIds();
	});

	const auto& vert = pMesh->getVert(vertIdx);
	vector<vector<size_t>> vertIndexLines;

	pMesh->createSharpEdgeVertexLines(vertIdx, availEdges, sinSharpAngle, vertIndexLines);

	// Use real faces instead of the bounding box in case they aren't the same.
	for (const auto& faceId : faceIds) {
		faceFunc(TS_REAL, faceId, [&piercePoints, &vertIndexLines, &pMesh](const Polygon& face) {
			for (const auto& vertIndices : vertIndexLines) {
				// Iterate backwards because the furthest edge should be crossing.
				for (size_t i = vertIndices.size() - 1; i > 0; i--) {
					const auto& pt0 = pMesh->getVert(vertIndices[i])._pt;
					const auto& pt1 = pMesh->getVert(vertIndices[i - 1])._pt;
					LineSegmentd seg(pt0, pt1);
					RayHitd hit;
					if (face.intersect(seg, hit)) {
						piercePoints.push_back(hit.hitPt);
						break;
					}
				}
			}
		});
	}
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
