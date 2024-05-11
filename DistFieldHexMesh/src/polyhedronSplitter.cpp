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
#include <polygon.h>
#include <polyhedron.h>
#include <polygonSplitter.h>
#include <polyhedronSplitter.h>
#include <block.h>

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
		Vector3d ctr;
		Plane<double> plane;
		if (cell.needsSplitAtCentroid()) {
			ctr = cell.calCentroid();
			result = splitAtPoint(ctr);
		} else if (cell.needsSplitAtPoint(ctr)) {
			result = splitAtPoint(ctr);
		} else if (cell.needsSplitAtPlane(plane))
			result = splitAtPlane(plane);
		else {
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
		_pBlock->dumpObj({ realCell.getId() });
		return false;
	}

#if DEBUG_BREAKS && defined(_DEBUG)
	// Now split the cell
	if (false && Index3DId(0, 8, 4, 0) == _polyhedronId) {
		_pBlock->dumpObj({ _polyhedronId });
	}
#endif

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

bool PolyhedronSplitter::splitAtPlane(const Plane<double>& plane)
{
	if (!_pBlock->polyhedronExists(TS_REAL, _polyhedronId))
		return false;

	_pBlock->makeRefPolyhedronIfRequired(_polyhedronId);

	assert(_pBlock->polyhedronExists(TS_REFERENCE, _polyhedronId));
	Polyhedron& referenceCell = _pBlock->getPolyhedron(TS_REFERENCE, _polyhedronId);

	assert(_pBlock->polyhedronExists(TS_REAL, _polyhedronId));
	Polyhedron& realCell = _pBlock->getPolyhedron(TS_REAL, _polyhedronId);

	bool result = splitAtPlaneInner(realCell, referenceCell, plane);

	if (result && _pBlock->polyhedronExists(TS_REAL, _polyhedronId)) {
		_pBlock->freePolyhedron(_polyhedronId);
	}

	return result;
}

bool PolyhedronSplitter::splitAtPlaneInner(Polyhedron& realCell, Polyhedron& referanceCell, const Plane<double>& plane) const
{

	return false;
}
