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

#include <vector>
#include <map>

#define _USE_MATH_DEFINES
#include <cmath>

#include <vertex.h>
#include <edge.h>
#include <polygon.h>
#include <polyhedron.h>
#include <block.h>
#include <volume.h>
#include <logger.h>

using namespace std;
using namespace DFHM;

Polyhedron::Polyhedron(const set<Index3DId>& faceIds)
	: _faceIds(faceIds)
{
}

Polyhedron::Polyhedron(const vector<Index3DId>& faceIds)
{
	_faceIds.insert(faceIds.begin(), faceIds.end());
}

void Polyhedron::dumpFaces() const
{
	size_t idx = 0;
	for (const auto& faceId : _faceIds) {
		cout << "face[" << idx++ << "]\n";
		faceFunc(faceId, [](const Polygon& face) {
			Vector3d n = face.calUnitNormal();
			auto pBlock = face.getBlockPtr();
			cout << "  n: (" << n[0] << ", " << n[1] << ", " << n[2] << ")\n";
			const auto& vertIds = face.getVertexIds();
			for (const auto& vertId : vertIds) {
				Vector3d pt = pBlock->getVertexPoint(vertId);
				cout << "  p: (" << pt[0] << ", " << pt[1] << ", " << pt[2] << ")\n";
			}
		});
	}
}

bool Polyhedron::unload(ostream& out)
{
	return true;
}

bool Polyhedron::load(istream& out)
{
	return true;
}

bool Polyhedron::operator < (const Polyhedron& rhs) const
{
	assert(!"Polyhdra aren't sorted. Should never call this.");
	return false;
}

void Polyhedron::addFace(const Index3DId& faceId)
{
	if (_faceIds.count(faceId) == 0) {
		_faceIds.insert(faceId);
	}
}

bool Polyhedron::removeFace(const Index3DId& faceId)
{
	if (_faceIds.count(faceId) != 0) {
		_faceIds.erase(faceId);
		return true;
	}

	return false;
}

void Polyhedron::setSourceId(const Index3DId& id)
{
	_referenceEntityId = id;
}

void Polyhedron::getVertIds(set<Index3DId>& vertIds) const
{
	for (const auto& faceId : _faceIds) {
		faceFunc(faceId, [&vertIds](const Polygon& face) {
			const auto vertexIds = face.getVertexIds();
			vertIds.insert(vertexIds.begin(), vertexIds.end());
		});
	}
}

void Polyhedron::getEdges(set<Edge>& edges, bool includeNeighborFaces) const
{
	map<Edge, set<Index3DId>> edgeToFaceMap;
	set<Index3DId> adjCellIds;
	for (const auto& faceId : _faceIds) {
		faceFunc(faceId, [this, &edgeToFaceMap, &faceId, &adjCellIds, includeNeighborFaces](const Polygon& face) {
			if (includeNeighborFaces) {
				auto temp = face.getCellIds();
				adjCellIds.insert(temp.begin(), temp.end());
			}
			set<Edge> edges;
			face.getEdges(edges);
			for (const auto& edge : edges) {
				auto iter = edgeToFaceMap.find(edge);
				if (iter == edgeToFaceMap.end()) {
					set<Index3DId> data;
					iter = edgeToFaceMap.insert(make_pair(edge, data)).first;
				}
				iter->second.insert(faceId);
			}
		});
	}

	if (includeNeighborFaces) {
		adjCellIds.erase(_thisId);
		for (const auto& adjCellId : adjCellIds) {
			set<Index3DId> faceIds;
			getBlockPtr()->cellFunc(adjCellId, [&faceIds](const Polyhedron& cell) {
				faceIds = cell.getFaceIds();
			});
			for (const auto& faceId : faceIds) {
				faceFunc(faceId, [this, &edgeToFaceMap, &faceId](const Polygon& face) {
					set<Edge> edges;
					face.getEdges(edges);
					for (const auto& edge : edges) {
						auto iter = edgeToFaceMap.find(edge);
						if (iter != edgeToFaceMap.end()) {
							iter->second.insert(faceId);
						}
					}
				});
			}
		}

	}

	for (const auto& pair : edgeToFaceMap) {
		edges.insert(Edge(pair.first, pair.second));
	}
}

set<Index3DId> Polyhedron::getAdjacentCells(bool includeCornerCells) const
{
	set<Index3DId> cellIds;

	for (const auto& faceId : _faceIds) {
		faceFunc(faceId, [this, &cellIds](const Polygon& face) {
			const auto temp = face.getCellIds();
			cellIds.insert(temp.begin(), temp.end());
		});
	}

	// TODO implement includeCornerCells case

	cellIds.erase(_thisId);
	return cellIds;
}

// Gets the edges for a vertex which belong to this polyhedron
void Polyhedron::getVertEdges(const Index3DId& vertId, set<Edge>& result, bool includeAdjacentCells) const
{
	set<Edge> cellEdgeSet;
	getEdges(cellEdgeSet, includeAdjacentCells);
	if (includeAdjacentCells) {
		set<Index3DId> adjCells = getAdjacentCells(false);
		for (const auto& adjCellId : adjCells) {
			getBlockPtr()->cellFunc(adjCellId, [&cellEdgeSet](const Polyhedron& adjCell) {
				adjCell.getEdges(cellEdgeSet, true);
			});
		}
	}

	for (const auto& edge : cellEdgeSet) {
		if (edge.containsVertex(vertId))
			result.insert(edge);
	}
}

// Gets the faces for a vertex which belong to this polyhedron
set<Index3DId> Polyhedron::getVertFaces(const Index3DId& vertId) const
{
	set<Index3DId> result;

	set<Edge> vertEdges;
	getVertEdges(vertId, vertEdges, false);

	for (const Edge& edge : vertEdges) {
		edge.getFaceIds(result);
	}

	return result;
}

CBoundingBox3Dd Polyhedron::getBoundingBox() const
{
	CBoundingBox3Dd bbox;
	set<Index3DId> vertIds;
	getVertIds(vertIds);
	for (const auto& vertId : vertIds) {
		bbox.merge(getBlockPtr()->getVertexPoint(vertId));
	}

	return bbox;
}

bool Polyhedron::contains(const Vector3d& pt) const
{
	bool result = true;
	Vector3d ctr = calCentroid();
	for (const auto& faceId : _faceIds) {
		faceFunc(faceId, [this, &pt, &ctr, &result](const Polygon& face) {
			const double tol = Tolerance::sameDistTol();
			Vector3d faceCtr = face.calCentroid();
			Vector3d norm = face.calUnitNormal();

			Vector3d vCtr = ctr - faceCtr;
			vCtr.normalize();
			if (vCtr.dot(norm) > 0)
				norm = -norm;

			Vector3d vTestPt = pt - faceCtr;
			vTestPt.normalize();

			if (norm.dot(vTestPt) > tol)
				result = false;
		});
	}

	return result;
}

Vector3d Polyhedron::calCentroid() const
{
	auto bbox = getBoundingBox();
	Vector3d testCtr = (bbox.getMin() + bbox.getMax()) * 0.5;
	double area = 0;
	Vector3d ctr(0, 0, 0);

	for (const auto& faceId : _faceIds) {
		faceFunc(faceId, [&area, &ctr](const Polygon& face) {
			double faceArea;
			Vector3d faceCtr;
			face.calAreaAndCentroid(faceArea, faceCtr);
			area += faceArea;
			ctr += faceArea * faceCtr;
		});
	}
	ctr /= area;

	Vector3d err = ctr - testCtr;
	return ctr;
}

bool Polyhedron::intersectsModel() const
{
	if (_intersectsModel == IS_UNKNOWN) {
		CBoundingBox3Dd bbox = getBoundingBox();
		auto pTriMesh = getBlockPtr()->getModelMesh();
		auto sharps = getBlockPtr()->getVolume()->getSharpVertIndices();
		for (size_t idx : sharps) {
			if (bbox.contains(pTriMesh->getVert(idx)._pt))
				return true;
		}
		vector<CMesh::SearchEntry> triEntries;
		_intersectsModel = pTriMesh->findTris(bbox, triEntries) > 0 ? IS_TRUE : IS_FALSE;
	}

	return _intersectsModel == IS_TRUE; // Don't test split cells
}

bool Polyhedron::hasSplits() const
{
	const double tolSinAngle = sin(Tolerance::angleTol());

	set<Edge> edges;
	getEdges(edges, false);
	for (const auto& edge : edges) {
		assert(edge.getFaceIds().size() == 2);
		bool faceHasSplits = false;

		auto iter = edge.getFaceIds().begin();
		Index3DId faceId0 = *iter++;
		Index3DId faceId1 = *iter;

		Vector3d norm0, norm1;

		faceFunc(faceId0, [&norm0, &faceHasSplits](const Polygon& face0) {
			if (face0.hasSplitEdges())
				faceHasSplits = true;
			norm0 = face0.calUnitNormal();
		});

		faceFunc(faceId1, [&norm1, &faceHasSplits](const Polygon& face1) {
			if (face1.hasSplitEdges())
				faceHasSplits = true;
			norm1 = face1.calUnitNormal();
		});

		double cp = norm1.cross(norm0).norm();
		if (faceHasSplits || cp < tolSinAngle)
			return true;
	}

	return false;
}

Index3DId Polyhedron::createFace(const std::vector<Index3DId>& vertIds)
{
#if _DEBUG
	set<Edge> edgeSet;
	Polygon::createEdgesStat(vertIds, edgeSet);
	for (const auto& edge : edgeSet) {
		assert(edge.getLength(getBlockPtr()) > Tolerance::sameDistTol());
		assert(edge.onPrincipalAxis(getBlockPtr()));
	}
#endif
	Index3DId result = getBlockPtr()->findFace(vertIds);
	if (!result.isValid())
		result = getBlockPtr()->addFace(vertIds);

	return result;
}

void Polyhedron::createHexahedralFaces(const std::vector<Index3DId>& corners, std::vector<Index3DId>& faceIds)
{
	faceIds.push_back(createFace({ corners[0], corners[1], corners[2], corners[3] }));
	faceIds.push_back(createFace({ corners[4], corners[5], corners[6], corners[7] }));

	faceIds.push_back(createFace({ corners[0], corners[1], corners[5], corners[4] }));
	faceIds.push_back(createFace({ corners[3], corners[2], corners[6], corners[7] }));

	faceIds.push_back(createFace({ corners[1], corners[2], corners[6], corners[5] }));
	faceIds.push_back(createFace({ corners[0], corners[3], corners[7], corners[4] }));
}

void Polyhedron::setPolygonsCellId()
{
	for (const auto& faceId : _faceIds) {
		faceFunc(faceId, [this](Polygon& face) {
			face.addCellId(_thisId);
		});
	}
}

void Polyhedron::fixPartialSplits()
{
/*
	Cases:
	Is already a reference cell, do nothing and return
	If scheduled to split, do nothing and return
	No faces are reference, do nothing and return
	Has reference faces
			Does not have a reference cell, create one now
		else
		    Promote reference faces to this cell
*/
	// Case 1 - already a reference which cannot be changed
	if (isReference())
		return;

	bool hasRefFace = false;
	set<Index3DId> realFaceIds;
	for (const auto& faceId : _faceIds) {
		faceFunc(faceId, [&hasRefFace, &realFaceIds](const Polygon& face) {
			if (face.isReference()) {
				hasRefFace = true;
				const auto& subFaceIds = face._referencingEntityIds;
				realFaceIds.insert(subFaceIds.begin(), subFaceIds.end());
			} else {
				realFaceIds.insert(face.getId());
			}
		});
	}

	// Case 2 - Has no reference faces, return
	if (!hasRefFace)
		return;

	if (!_referenceEntityId.isValid()) {
		for (const auto& faceId : realFaceIds) {
			faceFunc(faceId, [this](Polygon& face) {
				face.removeCellId(_thisId);
			});
		}

		Index3DId dupCellId = getBlockPtr()->addCell(realFaceIds);
		_referencingEntityIds.insert(dupCellId);
		cellFunc(dupCellId, [this](Polyhedron& dupCell) {
			dupCell._referenceEntityId = _thisId;
			for (const auto& faceId : dupCell._faceIds) {
				faceFunc(faceId, [this, &dupCell](Polygon& face) {
					assert(face.usedByCell(dupCell.getId()));
					assert(!face.usedByCell(_thisId));
				});
			}

		});
	} else {
		_faceIds = realFaceIds;
	}
}

Index3DId Polyhedron::duplicateAndPromoteFaces()
{
	Index3DId dbgId(0, 6, 5, 1);
	if (dbgId == _thisId) {
		int dbgBreak = 1;
	}
	Index3DId dupCellId;
	if (_referencingEntityIds.empty()) {
		set<Index3DId> allFaceIds;
		for (const Index3DId& faceId : _faceIds) {
			faceFunc(faceId, [&allFaceIds](const Polygon& face) {
				const auto& refIds = face._referencingEntityIds;
				if (refIds.empty())
					allFaceIds.insert(face.getId());
				else {
					allFaceIds.insert(refIds.begin(), refIds.end());
				}
			});
		}

		dupCellId = getBlockPtr()->addCell(allFaceIds);
		if (dbgId == dupCellId) {
			int dbgBreak = 1;
		}
		_referencingEntityIds.insert(dupCellId);
		cellFunc(dupCellId, [this](Polyhedron& dupCell) {
			dupCell._referenceEntityId = _thisId;
		});

		for (const auto& faceId : allFaceIds) {
			faceFunc(faceId, [this, &dupCellId](Polygon& face) {
				face.removeCellId(_thisId);
				face.addCellId(dupCellId);
			});
		}
	} else {
		set<Index3DId> allFaceIds;
		for (const Index3DId& faceId : _faceIds) {
			faceFunc(faceId, [&allFaceIds](const Polygon& face) {
				const auto& refIds = face._referencingEntityIds;
				if (refIds.empty())
					allFaceIds.insert(face.getId());
				else {
					allFaceIds.insert(refIds.begin(), refIds.end());
				}
			});
		}

		_faceIds = allFaceIds;
	}

	return dupCellId;
}

void Polyhedron::splitIfRequred(int phase)
{
	if (_splitRequired) {
		if (phase == 0) {
			if (hasSplits())
				splitAtCentroid();
		}
		else {
			splitAtCentroid();
		}
	}
}

void Polyhedron::promoteReferencePolygons()
{
	Index3DId dbgId(0, 6, 5, 1);
	if (dbgId == _thisId) {
		int dbgBreak = 1;
	}

	if (isReference())
		return;

	bool hasReferenceFaces = false;
	set<Index3DId> allFaceIds;
	for (const auto& faceId : _faceIds) {
		faceFunc(faceId, [&hasReferenceFaces, &allFaceIds](const Polygon& face) {
			if (face.isReference()) {
				hasReferenceFaces = true;
				const auto& subFaces = face._referencingEntityIds;
				allFaceIds.insert(subFaces.begin(), subFaces.end());
			} else {
				allFaceIds.insert(face.getId());
			}
		});
	}

	if (!hasReferenceFaces)
		return; // Nothing to do

	if (_referenceEntityId.isValid()) {
		// this cell has already been duplicated
		for (const auto& faceId : _faceIds) {
			faceFunc(faceId, [this](Polygon& face) {
				face.removeCellId(_thisId);
			});
		}
		for (const auto& faceId : allFaceIds) {
			faceFunc(faceId, [this](Polygon& face) {
				face.removeCellId(_thisId);
			});
		}
		_faceIds = allFaceIds;
		for (const auto& faceId : _faceIds) {
			faceFunc(faceId, [this](Polygon& face) {
				face.addCellId(_thisId);
			});
		}
	} else {
		duplicateAndPromoteFaces();
	}
}

bool Polyhedron::needToImprintVertices() const
{
	assert(!isReference());

	set<Index3DId> vertIds;
	for (const auto& faceId : _faceIds) {
		faceFunc(faceId, [&vertIds](const Polygon& face) {
			assert(!face.isReference());
			const auto& fvIds = face.getVertexIds();
			vertIds.insert(fvIds.begin(), fvIds.end());
		});
	}

	for (const auto& faceId : _faceIds) {
		for (const auto& vertId : vertIds) {
			set<VertEdgePair> pairs;
			faceFunc(faceId, [this, &vertId, &pairs](const Polygon& face) {
				face.addRequiredImprintPairs(vertId, pairs);
			});

			if (!pairs.empty())
				return true;
		}
	};
	return false;
}

void Polyhedron::imprintVertices()
{
	if (isReference() || !needToImprintVertices())
		return;

	if (!_referenceEntityId.isValid()) {
		Index3DId dupCellId = duplicateAndPromoteFaces();
		cellFunc(dupCellId, [this](Polyhedron& dupCell) {
			dupCell.imprintVertices();
		});
		return; // Done, called recursively
	}

	set<Index3DId> vertIds;
	for (const auto& faceId : _faceIds) {
		faceFunc(faceId, [&vertIds](const Polygon& face) {
			assert(!face.isReference());
			const auto& fvIds = face.getVertexIds();
			vertIds.insert(fvIds.begin(), fvIds.end());
		});
	}

	// We CANNOT modify the primary face.
	// If it's primary, make a duplicate and use that.
	// If this face has a duplicate, use the duplicate.
	// If this face has more than one referencing face, it's been split AND THAT'S an error

	for (const auto& faceId : _faceIds) {
		Index3DId dbgId(Index3D(0, 6, 4), 4);
		if (faceId == dbgId) {
			int dbgBreak = 1;
		}
		set<VertEdgePair> pairs;
		Index3DId splitFaceId = faceId;

		faceFunc(faceId, [this, &splitFaceId](Polygon& face) {
			if (face._referencingEntityIds.size() == 1) {
				// The cell is NOT a reference, but this FACE may be a reference??
				splitFaceId = *face._referencingEntityIds.begin();
			}
			else if (!face._referencingEntityIds.empty()) {
				assert(!"should be only 1 or 0.");
			}
		});

		for (const auto& vertId : vertIds) {
			faceFunc(splitFaceId, [this, &vertId, &pairs](Polygon& splitFace) {
				splitFace.addRequiredImprintPairs(vertId, pairs);
			});
		}

		if (pairs.empty())
			continue;

		if (splitFaceId == faceId) {
			getBlockPtr()->removeFaceFromLookUp(faceId);
			// There is no reference face for the split, so create a duplicate and reference it
			faceFunc(faceId, [this, &splitFaceId](Polygon& face) {
				splitFaceId = addFace(face.getVertexIds());
				face._referencingEntityIds.insert(splitFaceId);
				faceFunc(splitFaceId, [this, &face](Polygon& splitFace) {
					splitFace._referenceEntityId = face.getId();
					splitFace.setCellIds(face.getCellIds());
				});
				face.clearCellIds();
			});
		}

		faceFunc(splitFaceId, [this, &pairs](Polygon& face) {
			while (!pairs.empty()) {
				auto iter = pairs.begin();
				auto vertId = iter->_vertId;
				auto edge = iter->_edge;
				pairs.erase(iter);
				face.imprintVertex(vertId, edge);
			}
		});
	}
}

bool Polyhedron::splitAtCentroid()
{
	auto ctr = calCentroid();

	return splitAtPoint(ctr);
}

void Polyhedron::removeOurIdFromFaces()
{
	for (const auto& faceId : _faceIds) {
		faceFunc(faceId, [this](Polygon& face) {
			face.removeCellId(_thisId);
		});
	}
}

bool Polyhedron::splitAtPoint(const Vector3d& centerPoint)
{
	_splitRequired = false;
	assert(_referencingEntityIds.empty());

	// If it has a reference, use that to determine the split
	// If not, then split this one.
	bool canSplitAll = true;
	for (const auto& faceId : _faceIds) {
		faceFunc(faceId, [this, &centerPoint, &canSplitAll](Polygon& face) {
			set<Index3DId> childFaceIds = face._referencingEntityIds;
			canSplitAll = canSplitAll && !childFaceIds.empty();
		});
	}
	
	if (!canSplitAll)
		return false;

	ostream& out = Logger::get().stream("splitter.log");

	THREAD_SAFE_LOG(out << "Splitting " << *this);

	removeOurIdFromFaces();

	Index3DId cellMidId = getBlockPtr()->addVertex(centerPoint);
	map<Index3DId, set<Index3DId>> vertToFaceMap;
	for (const auto& faceId : _faceIds) {
		faceFunc(faceId, [this, &centerPoint, &vertToFaceMap](Polygon& face) {
			set<Index3DId> childFaceIds = face._referencingEntityIds;
			for (const auto& childFaceId : childFaceIds) {
				faceFunc(childFaceId, [this, &vertToFaceMap](const Polygon& childFace) {
					for (const auto& vertId : childFace.getVertexIds()) {
						auto iter = vertToFaceMap.find(vertId);
						if (iter == vertToFaceMap.end()) {
							iter = vertToFaceMap.insert(make_pair(vertId, set<Index3DId>())).first;
						}
						iter->second.insert(childFace.getId());
					}
				});
			}

		});
	}

	set<Index3DId> corners;
	getVertIds(corners);

	for (const auto& vert : corners) {
		auto vertFaces = vertToFaceMap.find(vert)->second;
		assert(vertFaces.count(_thisId) == 0);
		map<Edge, set<Index3DId>> vertEdgeFaceMap;
		for (const auto& faceId : vertFaces) {
			faceFunc(faceId, [&vert, &vertEdgeFaceMap, &vertFaces](const Polygon& face) {
				set<Edge> faceEdges;
				face.getEdges(faceEdges);
				for (const auto& edge : faceEdges) {
					if (edge.containsVertex(vert)) {
						auto iter = vertEdgeFaceMap.find(edge);
						if (iter == vertEdgeFaceMap.end())
							iter = vertEdgeFaceMap.insert(make_pair(edge, set<Index3DId>())).first;
						iter->second.insert(face.getId());
						vertFaces.insert(face.getId());
					}
				}
			});
		}

		for (const auto& pair : vertEdgeFaceMap) {
			const auto& edge = pair.first;
			const auto& edgeFaces = pair.second;

			const auto& edgeVert1 = edge.getOtherVert(vert);

			if (edgeFaces.size() != 2) {
				assert(!"Should never happen, the edge must have two faces attached");
				return false;
			}
			Index3DId faceVert0, faceVert1;
			auto iter = edgeFaces.begin();
			faceFunc(*iter++, [&faceVert0](const Polygon& face0) { faceVert0 = face0.getVertexIds()[0]; });
			faceFunc(*iter, [&faceVert1](const Polygon& face0) { faceVert1 = face0.getVertexIds()[0]; });

			vector<Index3DId> verts = {
				edgeVert1,
				faceVert0,
				cellMidId,
				faceVert1,
			};

			auto newFaceId = addFace(verts);
			vertFaces.insert(newFaceId);
		}

		Polyhedron newCell(vertFaces);

		auto newCellId = getBlockPtr()->addCell(newCell, true);
		cellFunc(newCellId, [this, &out](Polyhedron& newCell) {
			newCell._referenceEntityId = _thisId;
			Padding::ScopedPad sp;
			THREAD_SAFE_LOG(out << Padding::get() << "to: " << newCell);
		});
		_referencingEntityIds.insert(newCellId);

	}

	THREAD_SAFE_LOG(out << "Post split " << *this << "\n**************************************************************************************\n");

	return true;
}

Index3DId Polyhedron::addFace(const std::vector<Index3DId>& vertIds)
{
	Polygon face(vertIds);
#if _DEBUG
	set<Edge> edges;
	face.getEdges(edges);
	for (const auto& edge : edges) {
		assert(edge.onPrincipalAxis(getBlockPtr()));
	}
#endif

	return getBlockPtr()->addFace(face);
}

void Polyhedron::setNeedToSplitAtCentroid()
{
	if (isReference())
		return;

	_splitRequired = true;
	Vector3d ctr = calCentroid();
	for (const auto& faceId : _faceIds) {
		faceFunc(faceId, [&ctr](Polygon& face) {
			face.setNeedToSplitAtPoint(ctr);
		});
	}
}

void Polyhedron::setNeedToSplitCurvature(int divsPerRadius, double maxCurvatureRadius, double sinEdgeAngle)
{
	CBoundingBox3Dd bbox = getBoundingBox();

	bool needToSplit = false;
#if 1 // Split at sharp vertices
	auto sharpVerts = getBlockPtr()->getVolume()->getSharpVertIndices();
	for (size_t vertIdx : sharpVerts) {
		auto pTriMesh = getBlockPtr()->getModelMesh();
		Vector3d pt = pTriMesh->getVert(vertIdx)._pt;
		if (bbox.contains(pt)) {
			needToSplit = true;
			break;
		}
	}
#endif

	if (!needToSplit) {
		double refRadius = calReferenceSurfaceRadius(bbox, maxCurvatureRadius, sinEdgeAngle);
		if (refRadius > 0 && refRadius < maxCurvatureRadius) {
			double maxLength = refRadius / divsPerRadius;
			auto range = bbox.range();
			double minDim = min(range[0], min(range[1], range[2]));
			if (minDim > maxLength) {
				needToSplit = true;
			}
		}
	}

	if (needToSplit) {
		setNeedToSplitAtCentroid();
	}
}

bool Polyhedron::orderVertEdges(set<Edge>& edgesIn, vector<Edge>& orderedEdges) const
{
	orderedEdges.clear();
	set<Edge> edges(edgesIn);
	while (!edges.empty()) {
		Edge lastEdge;
		if (orderedEdges.empty()) {
			lastEdge = *edges.begin();
			orderedEdges.push_back(lastEdge);
			edges.erase(lastEdge);
		} else {
			lastEdge = orderedEdges.back();
		}

		const auto& edgeFaces = lastEdge.getFaceIds();
		assert(edgeFaces.size() == 2);

		bool found = false;
		for (const auto& otherEdge : edges) {
			const auto& otherEdgeFaces = otherEdge.getFaceIds();

			for (const auto& faceId : otherEdgeFaces) {
				if (edgeFaces.count(faceId) != 0) {
					found = true;
					orderedEdges.push_back(otherEdge);
					edges.erase(otherEdge);
					break;
				}
			}
			if (found)
				break;
		}
		if (!found)
			return false;
	}

	return true;
}

double Polyhedron::calReferenceSurfaceRadius(const CBoundingBox3Dd& bbox, double maxCurvatureRadius, double sinEdgeAngle) const
{
	auto pTriMesh = getBlockPtr()->getModelMesh();
	vector<CMesh::SearchEntry> edgeEntries;
	if (pTriMesh->findEdges(bbox, edgeEntries) > 0) {
		vector<double> edgeRadii;
		for (const auto& edgeEntry : edgeEntries) {
			size_t edgeIdx = edgeEntry.getIndex();
			double edgeCurv = pTriMesh->edgeCurvature(edgeIdx);
			double edgeRad = edgeCurv > 0 ? 1 / edgeCurv : 10000;
			if (edgeRad > 0 && edgeRad < maxCurvatureRadius)
				edgeRadii.push_back(edgeRad);
		}
		if (edgeRadii.empty())
			return 1e6;

		sort(edgeRadii.begin(), edgeRadii.end());
#if 0
		{
			static mutex m;
			lock_guard g(m);
			size_t idx = getBlockPtr()->getVolume()->calLinearBlockIndex(getId());
			cout << "Cell idx: " << idx << "\n";
			for (double r : edgeRadii)
				cout << "r: " << r << "\n";
			cout << "\n";
		}
#endif
		size_t num = std::min((size_t)10, edgeRadii.size());
		if (num == 0)
			return -1;
		double avgRad = 0;
		for (size_t i = 0; i < num; i++)
			avgRad += edgeRadii[i];
		avgRad /= num;
		if (avgRad > 0)
			return avgRad;
		return -1;
	}
	return 0;
}

double Polyhedron::getShortestEdge() const
{
	double minDist = DBL_MAX;
	for (const auto& faceId : _faceIds) {
		faceFunc(faceId, [&minDist](const Polygon& face) {
			double l = face.getShortestEdge();
			if (l < minDist)
				minDist = l;
		});
	}

	return minDist;
}

set<Edge> Polyhedron::createEdgesFromVerts(vector<Index3DId>& vertIds) const
{
	// This function takes vertices and looks for faces which contain exactly 2 of the available verts.
	// The edge may already exist.
	set<Edge> edgeSet;
	for (const auto& faceId : _faceIds) {
		faceFunc(faceId, [this, &edgeSet, &vertIds](const Polygon& face) {
			set<Index3DId> vertsInFace;
			for (const auto& vertId : vertIds) {
				if (face.containsVertex(vertId))
					vertsInFace.insert(vertId);
			}
			if (vertsInFace.size() == 2) {
				auto iter = vertsInFace.begin();
				const Index3DId& vertId0 = *iter++;
				const Index3DId& vertId1 = *iter;
				Edge newEdge(vertId0, vertId1);

				edgeSet.insert(newEdge);
			}
		});
	}

	return edgeSet;
}

bool Polyhedron::orderVertIds(vector<Index3DId>& vertIds) const
{

	vector<Index3DId> result;
	auto newEdgeSet = createEdgesFromVerts(vertIds);

	const Edge& e = *newEdgeSet.begin();
	result.push_back(e.getVertexIds()[0]);
	result.push_back(e.getVertexIds()[1]);

	newEdgeSet.erase(newEdgeSet.begin());
	bool found = true;
	while (found && !newEdgeSet.empty()) {
		found = false;
		const auto& lastVert = result.back();
		for (auto iter = newEdgeSet.begin(); iter != newEdgeSet.end(); iter++) {
			const Edge& e = *iter;
			const auto& otherVert = e.getOtherVert(lastVert);
			if (otherVert.isValid()) {
				result.push_back(otherVert);
				newEdgeSet.erase(iter);
				found = true;
				break;
			}
		}
	}

	if (result.front() == result.back()) {
		result.pop_back();
		if (result.size() == vertIds.size()) {
			vertIds = result;
			return true;
		} else {
			assert(!"Ordered list does not match size of input list.");
		}
	}

	return false;
}

bool Polyhedron::isClosed() const
{
	bool result = true;
	set<Edge> edges;
	getEdges(edges, false);
	for (const auto& edge : edges) {
		if (edge.getFaceIds().size() != 2) {
			result = false;
			break;
		}
	}

	return result;
}

bool Polyhedron::verifyTopology() const
{
	bool valid = true;
#ifdef _DEBUG 

	if (isReference())
		return valid;

	if (!isClosed())
		valid = false;
	for (const auto& faceId : _faceIds) {
		if (getBlockPtr()->polygonExists(faceId)) {
			faceFunc(faceId, [this, &valid](const Polygon& face) {
				bool pass = face.usedByCell(_thisId);
				if (!pass)
					valid = false;
				pass = face.verifyTopology();
				if (!pass)
					valid = false;
				});
		} else
			valid = false;
	}
#endif // _DEBUG 

	return valid;
}

ostream& DFHM::operator << (ostream& out, const Polyhedron& cell)
{
	out << "Cell: c" << cell.getId() << "\n";
	{
		Padding::ScopedPad sp;

		out << Padding::get() << "faceIds: {";
		for (const auto& faceId : cell.getFaceIds()) {
			out << "f" << faceId << " ";
		}
		out << "}\n";

		out << Padding::get() << "referenceEntityId: c" << cell._referenceEntityId << "\n";
		out << Padding::get() << "referencingEntityIds: {";
		for (const auto& refId : cell._referencingEntityIds) {
			out << "c" << refId << " ";
		}
		out << "}\n";
	}

	return out;
}
