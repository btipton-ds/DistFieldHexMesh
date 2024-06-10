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

#if defined(_DEBUG)
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

bool PolyhedronSplitter::splitMultipleAtPlane(Block* pBlock, const Planed& plane, MTC::set<Index3DId> targetCellIds, MTC::set<Index3DId>& newCellIds)
{
	bool result = false;

	for (const auto& cellId : targetCellIds) {
		PolyhedronSplitter sp(pBlock, cellId);
		if (sp.splitAtPlane(plane, newCellIds)) {
			result = true;
		}
	}
	return result;
}

bool PolyhedronSplitter::cutAtSharpVerts(const BuildCFDParams& params)
{
	bool result = false;

	auto& cell = _pBlock->getPolyhedron(TS_REAL, _polyhedronId);
	vector<size_t> verts = cell.getSharpVertIndices();

	for (size_t vIdx : verts) {
		if (!cutAtSharpVert(vIdx, params)) {
			result = false;
		}
	}

	if (result && _pBlock->polyhedronExists(TS_REAL, _polyhedronId)) {
		_pBlock->freePolyhedron(_polyhedronId, true);
	}

	return result;
}

bool PolyhedronSplitter::cutAtSharpVert(size_t vertIdx, const BuildCFDParams& params)
{
#if LOGGING_ENABLED
	auto pLogger = getBlockPtr()->getLogger();
	auto& out = pLogger->getStream();
	LOG(out << "cutAtSharpVert c" << _polyhedronId << "\n");
#endif
	if (!_pBlock->polyhedronExists(TS_REAL, _polyhedronId))
		return false;

	_pBlock->makeRefPolyhedronIfRequired(_polyhedronId);

	assert(_pBlock->polyhedronExists(TS_REFERENCE, _polyhedronId));
	Polyhedron& referenceCell = _pBlock->getPolyhedron(TS_REFERENCE, _polyhedronId);

	assert(_pBlock->polyhedronExists(TS_REAL, _polyhedronId));
	Polyhedron& realCell = _pBlock->getPolyhedron(TS_REAL, _polyhedronId);

	bool result = cutAtSharpVertInner(realCell, referenceCell, vertIdx, params);

#if LOGGING_ENABLED
	out.flush();
#endif
	return result;
}

bool PolyhedronSplitter::cutAtSharpVertInner(Polyhedron& realCell, Polyhedron& referanceCell, size_t vertIdx, const BuildCFDParams& params)
{
#if LOGGING_ENABLED
	auto pLogger = getBlockPtr()->getLogger();
	auto& out = pLogger->getStream();
#endif
	// Construct pyramid faces through the sharp vert and pierce points

	auto pMesh = _pBlock->getModelMesh();
	bool isConvex = false;
	LineSegmentd axisSeg;
	if (!pMesh->isVertConvex(vertIdx, isConvex, axisSeg)) {
		assert(!"A sharp vertex should always return true.");
	}
	MTC::vector<Vector3d> piercePoints;
	findSharpVertPierecPoints(vertIdx, piercePoints, params);

#if LOGGING_ENABLED
	LOG(out << "cutAtSharpVertInner, #piercePoints:" << piercePoints.size() << "\n");
	LOG(out << "Hexadral faces \n");
	for (const auto& faceId : realCell.getFaceIds()) {
		LOG(out << "face f" << faceId << "\n");
		_pBlock->faceFunc(TS_REAL, faceId, [&out](const Polygon& face) {
			LOG(out << face);
		});
	}
	LOG(out << "Splitting faces \n");
#endif
	MTC::vector<MTC::vector<Vector3d>> cutFaces;
	MTC::vector<Index3DId> modelFaces;
	if (piercePoints.empty()) {
		// Smooth case like a cone or ogive. Create a 4 sided pyramid on principal axes
	} else {
		// General case, create a parting face
		Vector3d tipPt = pMesh->getVert(vertIdx)._pt;
		Index3DId tipVertId = _pBlock->addVertex(tipPt);

		// Lock the sharp vertex when it's created
		_pBlock->setVertexLockType(tipVertId, VLT_ALL_AXES);

		for (size_t i = 0; i < piercePoints.size(); i++) {
			size_t j = (i + 1) % piercePoints.size();

			Vector3d xAxis = (piercePoints[i] - tipPt).normalized();
			Vector3d v1 = (piercePoints[j] - tipPt).normalized();
			Vector3d yAxis = v1 - xAxis.dot(v1) * xAxis;
			yAxis.normalize();
			Vector3d n = v1.cross(xAxis).normalized();
			Planed modelPlane(tipPt, n, false);
			MTC::vector<Vector3d> modelFacePoints, newPoints, tempPoints;
			if (realCell.createIntersectionFacePoints(modelPlane, tempPoints) > 2) {
				double cosT = v1.dot(xAxis);
				double sinT = v1.dot(yAxis);
				double thetaMax = atan2(sinT, cosT);

				for (const auto& pt : tempPoints) {
					Vector3d v = (pt - tipPt).normalized();

					cosT = v.dot(xAxis);
					sinT = v.dot(yAxis);
					double theta = atan2(sinT, cosT);
					if (theta > 0 && theta < thetaMax)
						newPoints.push_back(pt);
				}

				sortNewFacePoints(tipPt, xAxis, yAxis, newPoints);
				modelFacePoints.push_back(tipPt);
				modelFacePoints.insert(modelFacePoints.end(), piercePoints[i]);
				modelFacePoints.insert(modelFacePoints.end(), newPoints.begin(), newPoints.end());
				modelFacePoints.insert(modelFacePoints.end(), piercePoints[j]);
				auto newFaceId = _pBlock->addFace(modelFacePoints);
				modelFaces.push_back(newFaceId);

			}
		}
	}

#if LOGGING_ENABLED
	for (const auto& id : modelFaces) {
		_pBlock->faceFunc(TS_REAL, id, [&out](const Polygon& face) {
			LOG(out << face);
		});
	}
#endif
	string fileName = "modelFacePoints_" + to_string(_polyhedronId[0]) + "_" + to_string(_polyhedronId[1]) + "_" + to_string(_polyhedronId[2]) + "_" + to_string(_polyhedronId.elementId());
	_pBlock->dumpPolygonObj(fileName, modelFaces);

	_pBlock->dumpPolyhedraObj({ _polyhedronId }, false, false, false, piercePoints);

	MTC::vector<Index3DId> newCellIds;
	splitWithFaces(realCell, modelFaces, newCellIds);

	return false;
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

void PolyhedronSplitter::splitWithFaces(Polyhedron& realCell, const MTC::vector<Index3DId>& imprintFaces, MTC::vector<Index3DId>& newCellIds) const
{
	map<Index3DId, set<Edge>> faceToEdgeMap;
	auto realFaceIds = realCell.getFaceIds();
	for (const auto& realFaceId : realFaceIds) {
		Planed realFacePlane;
		set<Vector3d> realFacePoints;
		faceFunc(TS_REAL, realFaceId, [this, &realFacePlane, &realFacePoints](const Polygon& realFace) {
			realFacePlane = realFace.calPlane();
			for (const auto& id : realFace.getVertexIds()) {
				Vector3d pt = _pBlock->getVertexPoint(id);
				realFacePoints.insert(pt);
			}
		});

		for (const auto& imprintFaceId : imprintFaces) {
			MTC::set<Edge> imprintEdges;
			faceFunc(TS_REAL, imprintFaceId, [this, &imprintEdges](const Polygon& imprintFace) {
				imprintEdges = imprintFace.getEdges();
			});

			for (const auto& imprintEdge : imprintEdges) {
				auto vertIds = imprintEdge.getVertexIds();
				Vector3d pt0 = _pBlock->getVertexPoint(vertIds[0]);
				Vector3d pt1 = _pBlock->getVertexPoint(vertIds[1]);
				double dist0 = realFacePlane.distanceToPoint(pt0);
				double dist1 = realFacePlane.distanceToPoint(pt1);
				if (dist0 < Tolerance::sameDistTol() && dist1 < Tolerance::sameDistTol()) {
					auto iter = faceToEdgeMap.find(realFaceId);
					if (iter == faceToEdgeMap.end()) {
						iter = faceToEdgeMap.insert(make_pair(realFaceId, set<Edge>())).first;
					}
					iter->second.insert(imprintEdge);
				}
			}
		}
	}

	for (const auto& pair : faceToEdgeMap) {
		const auto& faceId = pair.first;
		const auto& edges = pair.second;
		faceFunc(TS_REAL, faceId, [&edges](const Polygon& sourceFace) {
			MTC::vector<Index3DId> newFaces;
			sourceFace.splitWithEdges(edges, newFaces);
		});
	}

	MTC::set<Index3DId> faceIds0, faceIds1;

	// Add common faces for both cells
	faceIds0.insert(imprintFaces.begin(), imprintFaces.end());
	faceIds1.insert(imprintFaces.begin(), imprintFaces.end());


}

bool PolyhedronSplitter::cutAtSharpEdges(const BuildCFDParams& params)
{
	return false;
}

bool PolyhedronSplitter::cutWithModelMesh(const BuildCFDParams& params)
{
	_pBlock->cellFunc(TS_REAL, _polyhedronId, [this](const Polyhedron& cell) {
		auto pMesh = _pBlock->getModelMesh();
		const auto& tris = cell.getTriIndices();
		if (!tris.empty()) {
			for (const auto& faceId : cell.getFaceIds()) {
				_pBlock->faceFunc(TS_REAL, faceId, [&cell](Polygon& face) {
					face.removeCellId(cell.getId());
					assert(face.numCells() < 2);
				});
			}

			std::vector<std::vector<size_t>> patches;
			if (pMesh->createPatches(tris, patches)) {
				for (const auto& patch : patches) {
					cutWithPatch(cell, patch);
				}
			}

			for (const auto& faceId : cell.getFaceIds()) {
				bool deleteFace = false;
				_pBlock->faceFunc(TS_REAL, faceId, [&cell, &deleteFace](Polygon& face) {
					deleteFace = face.numCells() == 0;
				});
				if (deleteFace) {
					_pBlock->freePolygon(faceId, false);
				}
			}

			_pBlock->freePolyhedron(_polyhedronId, false);
		}
	});
	return false;
}

void PolyhedronSplitter::cutWithPatch(const Polyhedron& realCell, const std::vector<size_t>& patch)
{
	auto pMesh = _pBlock->getModelMesh();

	bool intersects = false;
	// Prior operations require that the patch have correctly oriented normals.
	MTC::set<IntersectEdge> edges;
	const auto& faceIds = realCell.getFaceIds();
	for (const auto& faceId : faceIds) {
		_pBlock->faceFunc(TS_REAL, faceId, [this, &patch, &edges, &intersects](Polygon& face) {
			if (face.intersectModelTris(patch, edges))
				intersects = true;
		});
	}

	if (intersects)
		_pBlock->dumpPolyhedraObj({ _polyhedronId }, true, false, false);

	if (edges.size() >= 3) {

		MTC::set<Index3DId> cellVerts, cellVertsToKeep, newFaceIds;
		realCell.getVertIds(cellVerts);
		for (const auto& vertId : cellVerts) {
			Vector3d pt = _pBlock->getVertexPoint(vertId);
			for (const auto& edge : edges) {
				auto ctr0 = pMesh->triCentroid(edge._vert0._triIndex);
				auto norm0 = pMesh->triUnitNormal(edge._vert0._triIndex);
				auto ctr1 = pMesh->triCentroid(edge._vert1._triIndex);
				auto norm1 = pMesh->triUnitNormal(edge._vert1._triIndex);

				auto ctr = (ctr0 + ctr1) * 0.5;
				auto norm = norm0 + norm1;
				norm.normalize();

				Vector3d v = pt - ctr;
				if (v.dot(norm) >= 0)
					cellVertsToKeep.insert(vertId);
			}
		}

		for (const auto& faceId : realCell.getFaceIds()) {
			_pBlock->faceFunc(TS_REAL, faceId, [this, &edges, &cellVertsToKeep, &newFaceIds](Polygon& face) {
				int numVertsInSet = 0;
				const auto& faceVerts = face.getVertexIds();
				for (const auto& vertId : faceVerts) {
					if (cellVertsToKeep.contains(vertId))
						numVertsInSet++;
				}

				if (numVertsInSet == faceVerts.size())
					newFaceIds.insert(face.getId());
				else if (numVertsInSet != 0) {
					PolygonSplitter sp(_pBlock, face.getId());
					Index3DId newFaceId = sp.createTrimmedFace(edges);
					if (newFaceId.isValid())
						newFaceIds.insert(newFaceId);
				}
			});
		}

		MTC::vector<Index3DId> verts;
		if (PolygonSplitter::connectIntersectEdges(_pBlock, edges, verts, true)) {
			Index3DId faceId = _pBlock->addFace(verts);
			newFaceIds.insert(faceId);
			int dbgBreak = 1;
		}

		string filename0 = "_" + to_string(_polyhedronId[0]) + "_" + to_string(_polyhedronId[1]) + "_" + to_string(_polyhedronId[2]) + "_" + to_string(_polyhedronId.elementId()) + "_cutWithPatch";
		_pBlock->dumpPolygonObj(filename0, newFaceIds);

		if (facesFormClosedCell(newFaceIds)) {
			Index3DId cellId = _pBlock->addCell(newFaceIds);
#ifdef _DEBUG
			_pBlock->cellFunc(TS_REAL, cellId, [](const Polyhedron& cell) {
				assert(cell.isClosed());
			});
#endif // _DEBUG
		} else if (newFaceIds.size() > 2) {
			assert(!"cell didn't close as expected.");
		}
	}
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

bool PolyhedronSplitter::splitAtPlane(const Planed& plane, MTC::set<Index3DId>& newCellIds)
{
	if (!_pBlock->polyhedronExists(TS_REAL, _polyhedronId))
		return false;

	_pBlock->makeRefPolyhedronIfRequired(_polyhedronId);

	assert(_pBlock->polyhedronExists(TS_REFERENCE, _polyhedronId));
	Polyhedron& referenceCell = _pBlock->getPolyhedron(TS_REFERENCE, _polyhedronId);

	assert(_pBlock->polyhedronExists(TS_REAL, _polyhedronId));
	Polyhedron& realCell = _pBlock->getPolyhedron(TS_REAL, _polyhedronId);

	bool result = splitAtPlaneInner(realCell, referenceCell, plane, newCellIds);

	if (result && _pBlock->polyhedronExists(TS_REAL, _polyhedronId)) {
		newCellIds.erase(_polyhedronId);
		_pBlock->freePolyhedron(_polyhedronId, true);
	}

	return result;
}

bool PolyhedronSplitter::splitAtPlaneInner(Polyhedron& realCell, Polyhedron& referanceCell, const Planed& plane, MTC::set<Index3DId>& newCellIds)
{
	Index3DId imprintFaceId = realCell.createIntersectionFace(plane);

	if (imprintFaceId.isValid()) {
		imprintFace(realCell, imprintFaceId, newCellIds);
		return true;
	}

	return false;
}

bool PolyhedronSplitter::imprintFace(Polyhedron& realCell, const Index3DId& imprintFaceId, MTC::set<Index3DId>& newCellIds)
{
	// Imprint face is a splitting operation. It creates two cells from one

	set<Index3DId> realFaceIds = realCell.getFaceIds();

	MTC::vector<Index3DId> imprintVertIds;
	faceFunc(TS_REAL, imprintFaceId, [&imprintVertIds](const Polygon& imprintFace) {
		imprintVertIds = imprintFace.getVertexIds();
	});

	MTC::vector<Index3DId> facesToSplit;
	for (const auto& realFaceId : realFaceIds) {
		faceFunc(TS_REAL, realFaceId, [&imprintVertIds, &facesToSplit](const Polygon& realFace) {
			for (const auto& imprintVertId : imprintVertIds) {
				size_t idx = realFace.getImprintIndex(imprintVertId);
				if (idx != -1) {
					facesToSplit.push_back(realFace.getId());
				}
			}
		});
	}

	if (facesToSplit.empty())
		return false;

	_pBlock->dumpPolyhedraObj({ _polyhedronId }, true, false, false);

	MTC::vector<Index3DId> lowerFaceIds({ imprintFaceId }), upperFaceIds({ imprintFaceId });

	for (const auto& faceId : facesToSplit) {
		PolygonSplitter sp(_pBlock, faceId);
		Index3DId lowerFaceId, upperFaceId;
		if (sp.splitWithFace(imprintFaceId, lowerFaceId, upperFaceId)) {
			lowerFaceIds.push_back(lowerFaceId);
			upperFaceIds.push_back(upperFaceId);
		}
	}

	auto lowerCellId = _pBlock->addCell(lowerFaceIds);
	newCellIds.insert(lowerCellId);

	auto upperCellId = _pBlock->addCell(upperFaceIds);
	newCellIds.insert(upperCellId);

	return false;
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
