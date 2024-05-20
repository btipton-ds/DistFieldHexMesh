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
#include <tm_bestFit.h>
#include <splitParams.h>
#include <polygon.h>
#include <polyhedron.h>
#include <polygonSplitter.h>
#include <polyhedronSplitter.h>
#include <block.h>
#include <volume.h>

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
		Plane<double> plane;
		if (cell.needsSplitAtCentroid()) {
			ctr = cell.calCentroid();
			result = splitAtPoint(ctr);
		} else if (cell.needsSplitAtPoint(ctr)) {
			static mutex m;
			lock_guard lg(m);
			cout << "Splitting at point c" << _polyhedronId << " at pt " << ctr << "\n";
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
		_pBlock->freePolyhedron(_polyhedronId);
	}

	return result;
}

bool PolyhedronSplitter::splitAtPointInner(Polyhedron& realCell, Polyhedron& referanceCell, const Vector3d& pt) const
{
#if DEBUG_BREAKS && defined(_DEBUG)
	if (Index3DId(0, 8, 4, 0) == _polyhedronId) {
		int dbgBreak = 1;
	}
#endif

	assert(_pBlock);
#if LOGGING_ENABLED
	auto pLogger = _pBlock->getLogger();
	auto& out = pLogger->getStream();

	LOG(out << Logger::Pad() << "Polyhedron::splitAtPoint realCell: " << realCell);
	LOG(out << Logger::Pad() << "Polyhedron::splitAtPoint refCell : " << referanceCell);
#endif

#if _DEBUG
	double minDist = DBL_MAX;
	for (const auto& faceId : realCell.getFaceIds()) {
		_pBlock->faceFunc(TS_REAL, faceId, [this, &minDist, &pt](const Polygon& face) {
			double d = face.distanceToPoint(pt);
			if (d < minDist)
				minDist = d;
		});
	}

	assert(minDist > Tolerance::sameDistTol());
#endif

#if defined(_DEBUG)
	// Now split the cell
	if (Index3DId(0, 12, 0, 19) == _polyhedronId) {
		_pBlock->dumpObj({ _polyhedronId }, false, false, false, { pt });
	}
#endif

	set<Index3DId> cornerVerts;
	referanceCell.getVertIds(cornerVerts);
	assert(cornerVerts.size() == 8);
	const auto& faceIds = referanceCell.getFaceIds();
	assert(faceIds.size() == 6);
	map<Index3DId, set<Index3DId>> cornerVertToFaceMap;
	for (const auto& vertId : cornerVerts) {
		cornerVertToFaceMap.insert(make_pair(vertId, set<Index3DId>()));
	}

	// Split all faces which require splitting
	bool pass = true;
	for (const auto& faceId : faceIds) {

		PolygonSplitter splitter(_pBlock, faceId);
		splitter.splitAtPoint(pt);

		_pBlock->faceFunc(TS_REFERENCE, faceId, [this, &cornerVertToFaceMap, &pass](const Polygon& refFace) {
			assert(refFace._splitFaceProductIds.size() == refFace.getVertexIds().size());

			for (const auto& childFaceId : refFace._splitFaceProductIds) {
				if (!_pBlock->polygonExists(TS_REAL, childFaceId)) {
					pass = false;
					break;
				}
				_pBlock->faceFunc(TS_REAL, childFaceId, [&cornerVertToFaceMap](const Polygon& childFace) {
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

	if (!pass) {
		_pBlock->dumpObj({ realCell.getId() }, false, false, false);
		return false;
	}

	Index3DId cellMidId = _pBlock->addVertex(pt);

	for (const auto& pair : cornerVertToFaceMap) {
		auto cornerVertId = pair.first;
		auto splitVertFaces = pair.second;
		assert(splitVertFaces.size() == 3);
		map<Edge, set<Index3DId>> fullEdgeToFaceMap;
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
		map<Edge, set<Index3DId>> edgeToFaceMap;
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

			vector<Index3DId> verts = {
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
				_pBlock->faceFunc(TS_REAL, faceId, [this](const Polygon& face) {
					assert(face.cellsOwnThis());
				});
			}
#endif // _DEBUG

			newCell.setTriIndices(realCell.getTriIndices());
		});

#if LOGGING_ENABLED
		{
			Logger::Indent indent;
			_pBlock->cellFunc(TS_REAL, newCellId, [this, &out](const Polyhedron& newCell) {
				LOG(out << Logger::Pad() << "to: " << newCell);
			});
		}
#endif
	}

	LOG(out << Logger::Pad() << "=================================================================================================\n");

	return true;
}

bool PolyhedronSplitter::splitMultipleAtPlane(Block* pBlock, const Plane<double>& plane, set<Index3DId> targetCellIds, set<Index3DId>& newCellIds)
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

bool PolyhedronSplitter::splitAtSharpVerts(const BuildCFDParams& params)
{
	bool result = false;

	auto& cell = _pBlock->getPolyhedron(TS_REAL, _polyhedronId);
	vector<size_t> verts = cell.getSharpVertIndices();

	if (verts.empty())
		return false;
	
	if (verts.size() == 1) {
		return splitAtSharpVertConical(verts[0], params);
	} else if (splitAtSharpVertsLinear(verts, params)) {
		assert(!"subdivide and try again");
		return true;
	}

	return result;
}

bool PolyhedronSplitter::splitAtSharpVertConical(size_t vertIdx, const BuildCFDParams& params)
{
	bool result = false;
	auto pMesh = _pBlock->getModelMesh();

	const auto& vert = pMesh->getVert(vertIdx);

	return result;
}

bool PolyhedronSplitter::splitAtSharpVertsLinear(std::vector<size_t>& verts, const BuildCFDParams& params)
{
	bool result = false;
	auto pMesh = _pBlock->getModelMesh();
	vector<Vector3d> pts;
	Vector3d v0, pt0, pt1;

	for (size_t vertIdx : verts) {
		const auto& pt = pMesh->getVert(vertIdx)._pt;
		pts.push_back(pt);
	}
	double err;
	Ray<double> ray = bestFitLine(pts, err);
	if (err > Tolerance::sameDistTol()) {
		assert(!"Not implemented yet. Need to divide the cell into parts.");
		return false;
	}
	v0 = ray._dir;

	Vector3d orth(1, 0, 0);
	double minDp = fabs(v0.dot(orth));

	double dp = fabs(v0.dot(Vector3d(0, 1, 0)));
	if (dp < minDp)
		orth = Vector3d(0, 1, 0);

	dp = fabs(v0.dot(Vector3d(0, 0, 1)));
	if (dp < minDp)
		orth = Vector3d(0, 0, 1);
	orth = orth - v0.dot(orth) * v0;
	orth.normalize();

	Vector3d orth2 = v0.cross(orth);

	set<Index3DId> nextCellIds;
	if (splitMultipleAtPlane(_pBlock, Plane<double>(pts.front(), orth, false), { _polyhedronId }, nextCellIds))
		result = true;

	if (splitMultipleAtPlane(_pBlock, Plane<double>(pts.front(), orth2, false), nextCellIds, nextCellIds))
		result = true;

	for (const auto& pt : pts) {
		if (splitMultipleAtPlane(_pBlock, Plane<double>(pt, v0, false), { _polyhedronId }, nextCellIds))
			result = true;
	}

	return false;
}

bool PolyhedronSplitter::splitAtSharpEdgeCusps(const BuildCFDParams& params)
{
	double sinEdgeAngle = sin(params.getSharpAngleRadians());
	auto pMesh = _pBlock->getModelMesh();
	vector<size_t> verts;

	{
		auto& cell = _pBlock->getPolyhedron(TS_REAL, _polyhedronId);
		auto bbox = cell.getBoundingBox();
		const auto& sharpVerts = _pBlock->getVolume()->getSharpVertIndices();
		for (size_t vertIdx : sharpVerts) {
			const auto& vert = pMesh->getVert(vertIdx);
			if (bbox.contains(vert._pt)) {
				verts.push_back(vertIdx);
			}
		}

		if (verts.empty())
			return false;
	}

	_pBlock->makeRefPolyhedronIfRequired(_polyhedronId);
	if (verts.size() == 1) {
		size_t vertIdx = verts.front();
		if (_pBlock->polyhedronExists(TS_REFERENCE, _polyhedronId)) {
			Vector3d ctr;
			_pBlock->cellFunc(TS_REFERENCE, _polyhedronId, [&ctr](const Polyhedron& cell) {
				ctr = cell.calCentroid();
			});
			splitAtPoint(ctr);
		} else {
			Vector3d pt = pMesh->getVert(vertIdx)._pt;
			return splitAtPoint(pt);
		}
	}

	int dbgBreak = 1;


	return true;
}

bool PolyhedronSplitter::splitAtSharpEdges(const BuildCFDParams& params)
{
	return false;
}

bool PolyhedronSplitter::splitAtPlane(const Plane<double>& plane, set<Index3DId>& newCellIds)
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
		_pBlock->freePolyhedron(_polyhedronId);
	}

	return result;
}

bool PolyhedronSplitter::splitAtPlaneInner(Polyhedron& realCell, Polyhedron& referanceCell, const Plane<double>& plane, set<Index3DId>& newCellIds)
{
	Index3DId imprintFaceId = realCell.createIntersectionFace(plane);

	if (imprintFaceId.isValid()) {
		imprintFace(realCell, imprintFaceId, newCellIds);
		return true;
	}

	return false;
}

bool PolyhedronSplitter::imprintFace(Polyhedron& realCell, const Index3DId& imprintFaceId, set<Index3DId>& newCellIds)
{
	// Imprint face is a splitting operation. It creates two cells from one

	set<Index3DId> realFaceIds = realCell.getFaceIds();

	vector<Index3DId> imprintVertIds;
	_pBlock->faceFunc(TS_REAL, imprintFaceId, [&imprintVertIds](const Polygon& imprintFace) {
		imprintVertIds = imprintFace.getVertexIds();
	});

	vector<Index3DId> facesToSplit;
	for (const auto& realFaceId : realFaceIds) {
		_pBlock->faceFunc(TS_REAL, realFaceId, [&imprintVertIds, &facesToSplit](const Polygon& realFace) {
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

	_pBlock->dumpObj({ _polyhedronId }, true, false, false);

	vector<Index3DId> lowerFaceIds({ imprintFaceId }), upperFaceIds({ imprintFaceId });

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
