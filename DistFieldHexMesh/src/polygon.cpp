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
#include <cmath>
#include <fstream>
#include <tm_math.h>
#include <vertex.h>
#include <edge.h>
#include <polygon.h>
#include <block.h>
#include <logger.h>
#include <volume.h>

using namespace std;
using namespace DFHM;

Polygon::Polygon(const vector<Index3DId>& verts)
	: _vertexIds(verts)
{
}

void Polygon::addVertex(const Index3DId& vertId)
{
	_vertexIds.push_back(vertId);
	_needSort = true;
}

size_t Polygon::getSplitLevel(const Index3DId& cellId) const
{
	auto iter = _cellIds.find(cellId);
	if (iter != _cellIds.end())
		return iter->getSplitLevel();

	return -1;
}

Index3DId Polygon::getAdjacentCellId(const Index3DId& thisCellId) const
{
	Index3DId result;
	if (_cellIds.size() == 2) {
		for (const auto& id : _cellIds) {
			if (id != thisCellId) {
				result = id;
				break;
			}
		}
	}

	return result;
}

bool Polygon::unload(ostream& out, size_t idSelf)
{

	return true;
}

bool Polygon::load(istream& in, size_t idSelf)
{

	return true;
}

void Polygon::sortIds() const
{
	if (_needSort) {
		_sortedIds = _vertexIds;
		sort(_sortedIds.begin(), _sortedIds.end());
		_needSort = false;
	}
}

void Polygon::orient()
{
	Index3DId ownerCellId;
	switch (_cellIds.size()) {
		default:
			return;
		case 1: { // Outer polygon
			ownerCellId = *_cellIds.begin();
			break;
		}
		case 2: { // Inner polygon
			auto iter = _cellIds.begin();
			auto id0 = *iter++;
			auto id1 = *iter;

			ownerCellId = getBlockPtr()->maxCellId(id0, id1);
			break;
		}
	}

	Vector3d norm = calUnitNormal();
	Vector3d faceCtr = calCentroid();
	Vector3d cellCtr;
	cellRealFunc(ownerCellId, [&cellCtr](const Polyhedron& cell) {
		cellCtr = cell.calCentroid();
		});
	Vector3d v = cellCtr - faceCtr;
	if (v.dot(norm) < Tolerance::paramTol()) {
		reverse(_vertexIds.begin(), _vertexIds.end());
	}
}

void Polygon::pack()
{
	_sortedIds.clear();
}

bool Polygon::operator < (const Polygon& rhs) const
{
	sortIds();
	rhs.sortIds();

	if (_sortedIds.size() < rhs._sortedIds.size())
		return true;
	else if (_sortedIds.size() > rhs._sortedIds.size())
		return false;

	for (size_t i = 0; i < _sortedIds.size(); i++) {
		if (_sortedIds[i] < rhs._sortedIds[i])
			return true;
		else if (rhs._sortedIds[i] < _sortedIds[i])
			return false;
	}
	return false;
}

bool Polygon::hasSplitEdges() const
{
	const double tolSinAngle = sin(Tolerance::angleTol());
	for (size_t i = 0; i < _vertexIds.size(); i++) {
		size_t j = (i + 1) % _vertexIds.size();
		size_t k = (j + 1) % _vertexIds.size();

		Vector3d pt0 = getBlockPtr()->getVertexPoint(_vertexIds[i]);
		Vector3d pt1 = getBlockPtr()->getVertexPoint(_vertexIds[j]);
		Vector3d pt2 = getBlockPtr()->getVertexPoint(_vertexIds[k]);

		Vector3d v0 = (pt1 - pt0).normalized();
		Vector3d v1 = (pt2 - pt1).normalized();
		double dp = v1.cross(v0).norm();
		if (dp < tolSinAngle)
			return true;
	}

	return false;
}

bool Polygon::isBlockBoundary() const
{
	if (_cellIds.size() == 2) {
		auto iter0 = _cellIds.begin();
		auto iter1 = iter0++;
		Index3DId id0 = *iter0;
		Index3DId id1 = *iter1;
		return (id0.blockIdx() != id1.blockIdx());
	}
	return false;
}

void Polygon::getEdges(set<Edge>& edgeSet) const
{
	createEdgesStat(_vertexIds, edgeSet, _thisId);
}

bool Polygon::containsPoint(const Vector3d& pt) const
{
	if (!isPointOnPlane(pt))
		return false;

	Vector3d norm = calUnitNormal();
	for (size_t i = 0; i < _vertexIds.size(); i++) {
		size_t j = (i + 1) % _vertexIds.size();

		Vector3d pt0 = getBlockPtr()->getVertexPoint(_vertexIds[i]);
		Vector3d pt1 = getBlockPtr()->getVertexPoint(_vertexIds[j]);
		Vector3d v1 = (pt1 - pt0).normalized();
		Vector3d v0 = pt - pt0;
		v0 = v0 - v0.dot(v1) * v1;
		Vector3d cp = v1.cross(v0);
		double dist = norm.dot(cp);
	
		if (dist < -Tolerance::sameDistTol())
			return false;
	}
	return true;
}

bool Polygon::isPointOnPlane(const Vector3d& pt) const {
	return distFromPlane(pt) < Tolerance::sameDistTol();
}

bool Polygon::containsEdge(const Edge& edge) const
{
	size_t idx0, idx1;
	return containsEdge(edge, idx0, idx1);
}

bool Polygon::containsEdge(const Edge& edge, size_t& idx0, size_t& idx1) const
{
	for (size_t i = 0; i < _vertexIds.size(); i++) {
		size_t j = (i + 1) % _vertexIds.size();
		const auto& vertId0 = _vertexIds[i];
		const auto& vertId1 = _vertexIds[j];
		Edge testEdge(vertId0, vertId1);
		if (testEdge == edge) {
			idx0 = i;
			idx1 = j;
			return true;
		}
	}

	idx0 = idx1 = -1;
	return false;
}

bool Polygon::containsVertex(const Index3DId& vertId) const
{
	for (const auto& id : _vertexIds) {
		if (id == vertId)
			return true;
	}
	return false;
}

void Polygon::createEdgesStat(const vector<Index3DId>& verts, set<Edge>& edgeSet, const Index3DId& polygonId)
{
	for (size_t i = 0; i < verts.size(); i++) {
		size_t j = (i + 1) % verts.size();
		// If the edges is aready in the set, we get the existing one, not the new one?
		set<Index3DId> faceSet;
		if (polygonId.isValid())
			faceSet.insert(polygonId);
		edgeSet.insert(Edge(verts[i], verts[j], faceSet));
	}
}

Vector3d Polygon::calUnitNormalStat(const Block* pBlock, const vector<Index3DId>& vertIds)
{
	Vector3d norm(0, 0, 0);

	for (size_t i = 0; i < vertIds.size() - 2; i++) {
		size_t j = (i + 1) % vertIds.size();
		size_t k = (j + 1) % vertIds.size();

		Vector3d pt0 = pBlock->getVertexPoint(vertIds[i]);
		Vector3d pt1 = pBlock->getVertexPoint(vertIds[j]);
		Vector3d pt2 = pBlock->getVertexPoint(vertIds[k]);

		Vector3d v0 = pt2 - pt1;
		Vector3d v1 = pt0 - pt1;

		Vector3d n = v0.cross(v1);
		norm += n;
	}
	norm.normalize();
	return norm;
}

Vector3d Polygon::calUnitNormal() const
{
	return calUnitNormalStat(getBlockPtr(), _vertexIds);
}

double Polygon::calVertexAngleStat(const Block* pBlock, const vector<Index3DId>& vertIds, size_t idx1)
{
	const size_t sz = vertIds.size();
	if (idx1 < sz) {
		Vector3d norm = calUnitNormalStat(pBlock, vertIds);
		size_t idx0 = (idx1 + sz - 1) % sz;
		size_t idx2 = (idx1 + 1) % sz;

		Vector3d pt0 = pBlock->getVertexPoint(vertIds[idx0]);
		Vector3d pt1 = pBlock->getVertexPoint(vertIds[idx1]);
		Vector3d pt2 = pBlock->getVertexPoint(vertIds[idx2]);

		Vector3d v0 = pt0 - pt1;
		Vector3d v1 = pt2 - pt1;

		double dp = v1.dot(v0);
		double cp = v1.cross(v0).dot(norm);
		double angle = atan2(cp, dp);
		return angle;
	}

	return nanf("");
}

double Polygon::getShortestEdge() const
{
	double minDist = DBL_MAX;
	for (size_t i = 0; i < _vertexIds.size(); i++) {
		size_t j = (i + 1) % _vertexIds.size();
		Vector3d pt0 = getBlockPtr()->getVertexPoint(_vertexIds[i]);
		Vector3d pt1 = getBlockPtr()->getVertexPoint(_vertexIds[1]);
		double l = (pt1 - pt0).norm();
		if (l < minDist)
			minDist = l;
	}
	return minDist;
}

double Polygon::calVertexAngle(size_t idx1) const
{
	return calVertexAngleStat(getBlockPtr(), _vertexIds, idx1);
}

Vector3d Polygon::interpolatePoint(double t, double u) const
{
	assert(_vertexIds.size() == 4);
	Vector3d pts[] = {
		getBlockPtr()->getVertexPoint(_vertexIds[0]),
		getBlockPtr()->getVertexPoint(_vertexIds[1]),
		getBlockPtr()->getVertexPoint(_vertexIds[2]),
		getBlockPtr()->getVertexPoint(_vertexIds[3]),
	};

	return BI_LERP(pts[0], pts[1], pts[2], pts[3], t, u);
}

Vector3d Polygon::calCentroid() const
{
	double area;
	Vector3d ctr;
	calAreaAndCentroid(area, ctr);
	return ctr;
}

double Polygon::distFromPlane(const Vector3d& pt) const
{
	Plane pl(getBlockPtr()->getVertexPoint(_vertexIds[0]), calUnitNormal());
	return pl.distanceToPoint(pt);
}

void Polygon::calAreaAndCentroid(double& area, Vector3d& centroid) const
{
	if (!getBlockPtr()->isPolygonReference(this) && getBlockPtr()->polygonExists(TS_REFERENCE, _thisId)) {
		faceRefFunc(_thisId, [&area, &centroid](const Polygon& self) {
			self.calAreaAndCentroid(area, centroid);
		});
		return;
	}

	area = 0;
	centroid = Vector3d(0, 0, 0);
	Vector3d pt0 = getBlockPtr()->getVertexPoint(_vertexIds[0]);
	for (size_t j = 1; j < _vertexIds.size() - 1; j++) {
		size_t k = j + 1;
		Vector3d pt1 = getBlockPtr()->getVertexPoint(_vertexIds[j]);
		Vector3d pt2 = getBlockPtr()->getVertexPoint(_vertexIds[k]);
		Vector3d v0 = pt0 - pt1;
		Vector3d v1 = pt2 - pt1;
		Vector3d triCtr = (pt0 + pt1 + pt2) * (1.0 / 3.0);
		Vector3d cp = v1.cross(v0);
		double triArea = cp.norm() / 2.0;

		centroid += triCtr * triArea;
		area += triArea;
	}

	centroid /= area;
}

Vector3d Polygon::projectPoint(const Vector3d& pt) const
{
	Vector3d origin = getBlockPtr()->getVertexPoint(_vertexIds[0]); // And point will do
	Vector3d normal = calUnitNormal();
	Plane pl(origin, normal);
	auto result = pl.projectPoint(pt);

	return result;
}

void Polygon::removeCellId(const Index3DId& cellId)
{
	_cellIds.erase(cellId);
}

void Polygon::removeAllCellIds()
{
	_cellIds.clear();
}

void Polygon::addCellId(const Index3DId& cellId, size_t level)
{
	if (Index3DId(4, 6, 1, 0) == cellId) {
		int dbgBreak = 1;
	}
	_cellIds.erase(cellId); // Erase to clear split level and replace with the new one
	_cellIds.insert(CellId_SplitLevel(cellId, level));
#if 1
	if (_cellIds.size() > 2) {
		for (const auto& cellId1 : _cellIds) {
			assert(getBlockPtr()->polyhedronExists(TS_REAL, cellId1));
			cellRealFunc(cellId1, [this](const Polyhedron& cell) {
				assert(cell.containsFace(_thisId));
			});
		}
		assert(_cellIds.size() <= 2);
	}
#endif
}

void Polygon::unlinkFromCell(const Index3DId& cellId)
{
	_cellIds.erase(cellId);
}

bool Polygon::canBeSplitInCell(const Index3DId& cellId) const
{
	const double sinAngleTol = sin(Tolerance::angleTol());

	set<Edge> faceEdges;
	getEdges(faceEdges);
	set<Edge> testEdges;
	cellRealFunc(cellId, [this, &faceEdges, &testEdges](const Polyhedron& cell) {
		set<Edge> cellEdges;
		cell.getEdges(cellEdges, false);
		for (const auto& faceEdge : faceEdges) {
			auto iter = cellEdges.find(faceEdge);
			if (iter != cellEdges.end())
				testEdges.insert(*iter);
		}
	});

	for (const auto& testEdge : testEdges) {
		double cp = testEdge.calSinDihedralAngle(getBlockPtr());
		if (cp < sinAngleTol) // faces are tangent, cannot be split in this cell.
			return false;
	}

	return true;
}

void Polygon::splitAtCentroid(Block* pDstBlock) const
{
	auto ctr = calCentroid();
	splitAtPoint(pDstBlock, ctr);
}

void Polygon::splitAtPoint(Block* pDstBlock, const Vector3d& pt) const
{
	if (Index3DId(0, 5, 5, 8) == _thisId) {
		int dbgBreak = 1;
	}

	assert(getBlockPtr()->isPolygonReference(this));
	const double sinAngleTol = sin(Tolerance::angleTol());

#if LOGGING_ENABLED
	assert(getBlockPtr()->isPolygonReference(this));
	auto pLogger = getBlockPtr()->getLogger();
	auto& out = pLogger->getStream();

	LOG(out << Logger::Pad() << "Polygon::splitRefFaceAtPoint. pre: " << *this);
#endif

	// The code must be operating on the reference face
	assert(_vertexIds.size() == 4);
	vector<Index3DId> edgePtIds;
	edgePtIds.resize(_vertexIds.size());
	for (size_t i = 0; i < _vertexIds.size(); i++) {
		size_t j = (i + 1) % _vertexIds.size();
		Edge edge(_vertexIds[i], _vertexIds[j]);

		// Be sure to project directly to the edge itself. 
		// DO NOT project to the face followed by the edge, because that can result in two points on the same edge.
		Vector3d edgePt = edge.projectPt(getBlockPtr(), pt);
		bool inBounds;
		double t = edge.paramOfPt(getBlockPtr(), edgePt, inBounds);
		if (inBounds) {
			Index3DId vertId = pDstBlock->addVertex(edgePt);
			edgePtIds[i] = vertId;

			pDstBlock->addSplitEdgeVert(edge, vertId);
		}
		else {
			assert(!"Edge point is not in bounds.");
			return;
		}
	}

	Vector3d facePt = projectPoint(pt);
	if (!containsPoint(facePt)) {
		assert(!"Face point is not in bounds.");
		return;
	}

	Index3DId facePtId = pDstBlock->addVertex(facePt);

#ifdef _DEBUG
	Vector3d srcNorm = calUnitNormal();
#endif // _DEBUG

	for (size_t i = 0; i < _vertexIds.size(); i++) {
		size_t j = (i + _vertexIds.size() - 1) % _vertexIds.size();
		auto priorEdgeId = edgePtIds[j];
		auto vertId = _vertexIds[i];
		auto nextEdgeId = edgePtIds[i];
		Polygon newFace({ facePtId, priorEdgeId, vertId, nextEdgeId });
		for (const auto& cellId : _cellIds) {
			newFace.addCellId(cellId, cellId.getSplitLevel() + 1);
		}

#ifdef _DEBUG
		Vector3d newNorm = Polygon::calUnitNormalStat(getBlockPtr(), newFace.getVertexIds());
		double cp = newNorm.cross(srcNorm).norm();
		assert(cp < sinAngleTol);
#endif // _DEBUG

		auto newFaceId = pDstBlock->addFace(newFace);
		addToSplitProductIds(newFaceId);

#if LOGGING_ENABLED
		faceRealFunc(newFaceId, [this, &out](const Polygon& newFace) {
			Logger::Indent indent;
			LOG(out << Logger::Pad() << "to: " << newFace);
		});
#endif
	}

	for (const auto& cellId : _cellIds) {
		if (getBlockPtr()->polyhedronExists(TS_REAL, cellId)) {
			pDstBlock->makeRefPolyhedronIfRequired(cellId);
			size_t splitLevel = getSplitLevel(cellId);
			pDstBlock->cellRealFunc(cellId, [this, splitLevel](Polyhedron& cell) {
				cell.replaceFaces(_thisId, _splitProductIds, splitLevel);
			});
		}
	}

#ifdef _DEBUG
	for (const auto& faceId : _splitProductIds) {
		set<Polygon::CellId_SplitLevel> cellIds;
		faceRealFunc(faceId, [&cellIds](const Polygon& childFace) {
			cellIds = childFace.getCellIds();
		});

		for (const auto& cellId : cellIds) {
			if (getBlockPtr()->polyhedronExists(TS_REAL, cellId)) {
				cellRealFunc(cellId, [this, &faceId](const Polyhedron& cell) {
					assert(cell.containsFace(faceId));
					if (cell.hasTooManySplits()) {
						getOwnerBlockPtr(cell.getId())->dumpObj({ cell.getId() });
					}
				});
			}
		}
	}
#endif // _DEBUG

	LOG(out << Logger::Pad() << "Polygon::splitRefFaceAtPoint. pst: " << *this);
}

void Polygon::addToSplitProductIds(const Index3DId& id) const
{
	assert(getBlockPtr()->isPolygonReference(this));
	Polygon* refSelf = const_cast<Polygon*>(this);
	refSelf->_splitProductIds.insert(id);
}

void Polygon::createSplitEdgeVertMap(std::map<Edge, Index3DId>& result) const
{
	auto pBlk = getBlockPtr();

	set<Index3DId> allCells;
	for (const auto& cellId : _cellIds) {
		if (getBlockPtr()->polyhedronExists(TS_REAL, cellId)) {
			allCells.insert(cellId);
			cellRealFunc(cellId, [&allCells](const Polyhedron& cell) {
				auto tmp = cell.getAdjacentCells(true);
				allCells.insert(tmp.begin(), tmp.end());
			});
		}
	}

	set<Edge> edges;
	getEdges(edges);

	for (const auto& cellId : allCells) {
		auto pOwner = getOwnerBlockPtr(cellId);
		const auto& tmp = pOwner->getSplitEdgeVertMap();
		for (const auto& edge : edges) {
			auto iter = tmp.find(edge);
			if (iter != tmp.end())
				result.insert(*iter);
		}
	}
}

bool Polygon::needToImprintVertices() const
{
	map<Edge, Index3DId> splitEdgeVertMap;
	createSplitEdgeVertMap(splitEdgeVertMap);

	return !splitEdgeVertMap.empty();
}

void Polygon::imprintVertices()
{

#if LOGGING_ENABLED
	auto pLogger = getBlockPtr()->getLogger();
	auto& out = pLogger->getStream();

	LOG(out << Logger::Pad() << "imprintVertices pre:" << *this);
#endif
	auto pBlk = getBlockPtr();

	map<Edge, Index3DId> splitEdgeVertMap;
	createSplitEdgeVertMap(splitEdgeVertMap);

	// Increment down so insertions do not corrupt order
	// the vertex is not in this polygon and lies between i and j
	if (_thisId.isValid())
		pBlk->removeFaceFromLookUp(_thisId);

	for (const auto& pair : splitEdgeVertMap) {
		for (size_t i = 0; i < _vertexIds.size(); i++) {
			size_t j = (i + 1) % _vertexIds.size();
			Edge edge(_vertexIds[i], _vertexIds[j]);
			if (edge == pair.first) {
				const auto& vertId = pair.second;
				LOG(out << Logger::Pad() << "inserting vertex v" << vertId << ", in edge(v" << edge.getVertexIds()[0] << " v" << edge.getVertexIds()[1] << ") before vertex v" << _vertexIds[j] << "\n");
				_vertexIds.insert(_vertexIds.begin() + j, vertId);
				LOG(out << Logger::Pad() << "vertexIds(" << _vertexIds.size() << "): {");
				for (const auto& vertId0 : _vertexIds) {
					LOG(out << "v" << vertId0 << " ");
				}
				LOG(out << "}\n");
				break;
			}
		}
	}

	_needSort = true;
	if (_thisId.isValid())
		pBlk->addFaceToLookup(_thisId);
}

bool Polygon::verifyVertsConvexStat(const Block* pBlock, const vector<Index3DId>& vertIds)
{
	for (size_t i = 0; i < vertIds.size(); i++) {
		double angle = calVertexAngleStat(pBlock, vertIds, i);
		if (angle < 0 && angle > M_PI)
			return false;
	}

	return true;
}

bool Polygon::verifyUniqueStat(const vector<Index3DId>& vertIds)
{
	bool valid = true;

	for (size_t i = 0; i < vertIds.size(); i++) {
		for (size_t j = i + 1; j < vertIds.size(); j++) {
			if (vertIds[i] == vertIds[j])
				valid = false;
		}
	}
	return valid;
}

bool Polygon::addRequiredImprintPairs(const Index3DId& vertId, set<VertEdgePair>& pairs) const
{
	Vector3d pt = getBlockPtr()->getVertexPoint(vertId);
	for (size_t i = 0; i < _vertexIds.size(); i++) {
		size_t j = (i + 1) % _vertexIds.size();
		Edge edge(_vertexIds[i], _vertexIds[j]);
		double t;
		if (!containsVertex(vertId) && edge.isColinearWith(getBlockPtr(), vertId, t) && t > Tolerance::paramTol() && t < 1 - Tolerance::paramTol()) {
			pairs.insert(VertEdgePair(vertId, edge));
		}
	}

	return !pairs.empty();
}

bool Polygon::verifyTopology() const
{
	bool valid = true;
#ifdef _DEBUG 
	if (!verifyUnique())
		valid = false;

	if (_cellIds.size() > 2)
		valid = false;

	set<Edge> edges;
	getEdges(edges);
	for (const auto& edge : edges) {
		auto faceIds = edge.getFaceIds();
		if (faceIds.count(_thisId) == 0) // edge does not link back to this face
			valid = false;
	}

	for (const auto& cellId : _cellIds) {
		if (!getBlockPtr()->polyhedronExists(TS_REAL, cellId))
			valid = false;
	}
#endif
	return valid;
}

ostream& DFHM::operator << (ostream& out, const Polygon& face)
{
	out << "Face: f" << face.getId() << "\n";
	{
		Logger::Indent sp;
		
		out << Logger::Pad() << "vertexIds: (" << face._vertexIds.size() << "): {";
		for (const auto& vertId : face._vertexIds) {
			out << "v" << vertId << " ";
		}
		out << "}\n";

		out << Logger::Pad() << "cellIds: (" << face._cellIds.size() << "): {";
		for (const auto& cellId : face._cellIds) {
			auto sl = face.getSplitLevel(cellId);
			out << "c" << cellId << ".split: " << sl << " ";
		}
		out << "}\n";

		if (!face._splitProductIds.empty()) {
			out << Logger::Pad() << "splitFaceIds: (" << face._splitProductIds.size() << "): {";
			for (const auto& faceId : face._splitProductIds) {
				out << "f" << faceId << " ";
			}
			out << "}\n";
		}
	}

	return out;
}

Polygon::CellId_SplitLevel::CellId_SplitLevel(const Index3DId& cellId, size_t splitLevel)
	: _cellId(cellId)
	, _splitLevel(splitLevel)
{
}

bool Polygon::CellId_SplitLevel::operator < (const CellId_SplitLevel& rhs) const
{
	return _cellId < rhs._cellId;
}

Polygon::CellId_SplitLevel::operator const Index3DId& () const
{
	return _cellId;
}

size_t Polygon::CellId_SplitLevel::getSplitLevel() const
{
	return _splitLevel;
}


//LAMBDA_CLIENT_IMPLS(Polygon)

LAMBDA_CLIENT_IMPLS(Polygon)

