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
#include <splitters.h>
#include <block.h>
#include <polygon.h>

using namespace std;
using namespace DFHM;

PolygonSplitter::PolygonSplitter(Block* pBlock, const Index3DId& polygonId)
	: _pBlock(pBlock)
	, _polygonId(polygonId)
{
}

bool PolygonSplitter::doConditionalSplitAtCentroid()
{
	Vector3d ctr;
	_pBlock->faceAvailFunc(_polygonId, TS_REFERENCE, [&ctr](const Polygon& face) {
		ctr = face.calCentroid();
	});

	return doConditionalSplitAtPoint(ctr);
}

bool PolygonSplitter::doConditionalSplitAtPoint(const Vector3d& pt)
{
	_pBlock->makeRefPolygonIfRequired(_polygonId);

	assert(_pBlock->polygonExists(TS_REFERENCE, _polygonId));
	Polygon& referenceFace = _pBlock->getPolygon(TS_REFERENCE, _polygonId);

	if (!referenceFace._splitFaceProductIds.empty()) {
		return false;
	}

	assert(_pBlock->polygonExists(TS_REAL, _polygonId));
	Polygon& realFace = _pBlock->getPolygon(TS_REAL, _polygonId);

	bool result = doSplitAtPoint(realFace, referenceFace, pt);

	if (_pBlock->polygonExists(TS_REAL, _polygonId)) {
		_pBlock->freePolygon(_polygonId);
	}

	return result;
}

bool PolygonSplitter::doSplitAtPoint(Polygon& realFace, Polygon& referanceFace, const Vector3d& pt) const
{
	if (Index3DId(0, 8, 4, 4) == _polygonId) {
		int dbgBreak = 1;
	}

	assert(_pBlock->isPolygonReference(&referanceFace));
	assert(_pBlock->polygonExists(TS_REAL, _polygonId));
	const double sinAngleTol = sin(Tolerance::angleTol());

#if LOGGING_ENABLED
	auto pLogger = _pBlock->getLogger();
	auto& out = pLogger->getStream();

	LOG(out << Logger::Pad() << "Polygon::splitAtPoint. pre: " << realFace);
#endif

	assert(referanceFace.cellsOwnThis());

	// The code must be operating on the reference face
	const auto& vertexIds = referanceFace.getVertexIds();
	assert(vertexIds.size() == 4);
	vector<Index3DId> edgePtIds;
	edgePtIds.resize(vertexIds.size());
	for (size_t i = 0; i < vertexIds.size(); i++) {
		size_t j = (i + 1) % vertexIds.size();
		Edge edge(vertexIds[i], vertexIds[j]);

		// Be sure to project directly to the edge itself. 
		// DO NOT project to the face followed by the edge, because that can result in two points on the same edge.
		Vector3d edgePt = edge.projectPt(_pBlock, pt);
		bool inBounds;
		double t = edge.paramOfPt(_pBlock, edgePt, inBounds);
		if (inBounds) {
			Index3DId vertId = _pBlock->addVertex(edgePt);
			edgePtIds[i] = vertId;

			referanceFace.addSplitEdgeVert(edge, vertId);
		} else {
			assert(!"Edge point is not in bounds.");
			return false;
		}
	}

	Vector3d facePt = referanceFace.projectPoint(pt);
#if _DEBUG
	if (!referanceFace.containsPoint(facePt)) {
		assert(!"Face point is not in bounds.");
		return false;
	}
#endif

	Index3DId facePtId = _pBlock->addVertex(facePt);

#ifdef _DEBUG
	Vector3d srcNorm = referanceFace.calUnitNormal();
#endif // _DEBUG

	auto cellIds = realFace.getCellIds();
	for (size_t i = 0; i < vertexIds.size(); i++) {
		size_t j = (i + vertexIds.size() - 1) % vertexIds.size();
		auto priorEdgeId = edgePtIds[j];
		auto vertId = vertexIds[i];
		auto nextEdgeId = edgePtIds[i];
		Polygon newFace({ facePtId, priorEdgeId, vertId, nextEdgeId });

#ifdef _DEBUG
		Vector3d newNorm = Polygon::calUnitNormalStat(_pBlock, newFace.getVertexIds());
		double cp = newNorm.cross(srcNorm).norm();
		assert(cp < sinAngleTol);
#endif // _DEBUG

		auto newFaceId = _pBlock->addFace(newFace);
		referanceFace.addToSplitFaceProductIds(newFaceId);
	}

	for (const auto& cellId : cellIds) {
		if (_pBlock->polyhedronExists(TS_REAL, cellId)) {
			size_t splitLevel = realFace.getSplitLevel(cellId);
			const auto& splits = referanceFace._splitFaceProductIds;

			_pBlock->makeRefPolyhedronIfRequired(cellId);
			_pBlock->cellRealFunc(cellId, [this, &splits, splitLevel](Polyhedron& cell) {
				cell.replaceFaces(_polygonId, splits, splitLevel + 1);
			});
		}
	}

#if LOGGING_VERBOSE_ENABLED
	LOG(out << Logger::Pad() << "Polygon::splitAtPoint. pst: " << *this);
#endif

	return true;
}

PolyhedronSplitter::PolyhedronSplitter(Block* pBlock, const Index3DId& polyhedronId)
	: _pBlock(pBlock)
	, _polyhedronId(polyhedronId)
{

}

bool PolyhedronSplitter::doConditionalSplitAtCentroid()
{
	Vector3d ctr;
	_pBlock->cellAvailFunc(_polyhedronId, TS_REFERENCE, [&ctr](const Polyhedron& cell) {
		ctr = cell.calCentroid();
	});

	return doConditionalSplitAtPoint(ctr);
}

bool PolyhedronSplitter::doConditionalSplitAtPoint(const Vector3d& pt)
{
	if (!_pBlock->polyhedronExists(TS_REAL, _polyhedronId))
		return false;

	_pBlock->makeRefPolyhedronIfRequired(_polyhedronId);

	assert(_pBlock->polyhedronExists(TS_REFERENCE, _polyhedronId));
	Polyhedron& referenceCell = _pBlock->getPolyhedron(TS_REFERENCE, _polyhedronId);

	assert(_pBlock->polyhedronExists(TS_REAL, _polyhedronId));
	Polyhedron& realCell = _pBlock->getPolyhedron(TS_REAL, _polyhedronId);

	bool result = doSplitAtPoint(realCell, referenceCell, pt);

	if (_pBlock->polyhedronExists(TS_REAL, _polyhedronId)) {
		_pBlock->freePolyhedron(_polyhedronId);
	}

	return result;
}

bool PolyhedronSplitter::doSplitAtPoint(Polyhedron& realCell, Polyhedron& referanceCell, const Vector3d& pt) const
{
	if (Index3DId(0, 8, 4, 0) == _polyhedronId) {
		int dbgBreak = 1;
	}
	assert(_pBlock);
#if LOGGING_ENABLED
	auto pLogger = _pBlock->getLogger();
	auto& out = pLogger->getStream();

	LOG(out << Logger::Pad() << "Polyhedron::splitAtPoint " << *this);
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
	for (const auto& faceId : faceIds) {

		PolygonSplitter splitter(_pBlock, faceId);
		splitter.doConditionalSplitAtCentroid();

		_pBlock->faceRefFunc(faceId, [this, &cornerVertToFaceMap](const Polygon& refFace) {
			assert(refFace._splitFaceProductIds.size() == refFace.getVertexIds().size());

			for (const auto& childFaceId : refFace._splitFaceProductIds) {
				_pBlock->faceRealFunc(childFaceId, [&cornerVertToFaceMap](const Polygon& childFace) {
					for (const auto& vertId : childFace.getVertexIds()) {
						auto iter = cornerVertToFaceMap.find(vertId);
						if (iter != cornerVertToFaceMap.end()) {
							iter->second.insert(childFace.getId());
						}
					}
					});
			}
		});
	}

	// Now split the cell
	if (false && Index3DId(0, 8, 4, 0) == _polyhedronId) {
		_pBlock->dumpObj({ _polyhedronId });
	}

	Index3DId cellMidId = _pBlock->addVertex(pt);

	for (const auto& pair : cornerVertToFaceMap) {
		auto cornerVertId = pair.first;
		auto splitVertFaces = pair.second;
		assert(splitVertFaces.size() == 3);
		map<Edge, set<Index3DId>> fullEdgeToFaceMap;
		for (const auto& splitFaceId : splitVertFaces) {
			assert(_pBlock->polygonExists(TS_REAL, splitFaceId));
			_pBlock->faceAvailFunc(splitFaceId, TS_REFERENCE, [&fullEdgeToFaceMap](const Polygon& splitFace) {
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

			int dbgBreak = 1;
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
			_pBlock->faceAvailFunc(*iter++, TS_REFERENCE, [&faceVert0](const Polygon& face) {
				assert(face.getVertexIds().size() == 4);
				faceVert0 = face.getVertexIds()[0];
			});
			_pBlock->faceAvailFunc(*iter, TS_REFERENCE, [&faceVert1](const Polygon& face) {
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
			if (Index3DId(5, 5, 1, 45) == faceId) {
				int dbgBreak = 1;
			}
			_pBlock->faceRealFunc(faceId, [this](Polygon& face) {
				face.removeCellId(_polyhedronId);
				face.removeDeadCellIds();
			});
		}
		Polyhedron newCell(splitVertFaces);

		auto newCellId = _pBlock->addCell(newCell);
		if (Index3DId(5, 5, 0, 45) == newCellId) {
			int dbgBreak = 1;
		}

		_pBlock->cellRealFunc(newCellId, [this](const Polyhedron& newCell) {
			for (const auto& faceId : newCell.getFaceIds()) {
				_pBlock->faceRealFunc(faceId, [this](const Polygon& face) {
					assert(face.cellsOwnThis());
				});
			}
		});

#if LOGGING_ENABLED
		{
			Logger::Indent indent;
			cellRealFunc(newCellId, [this, &out](const Polyhedron& newCell) {
				LOG(out << Logger::Pad() << "to: " << newCell);
				});
		}
#endif
	}

	LOG(out << Logger::Pad() << "=================================================================================================\n");

	return true;
}
