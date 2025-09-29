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

	Copyright Robert R Tipton, 2022, all rights reserved except those granted in prior license statement.

	Dark Sky Innovative Solutions http://darkskyinnovation.com/
*/

#include <vector>
#include <map>

#include <defines.h>
#include <cmath>

#include <tm_lineSegment.h>
#include <tm_lineSegment.hpp>
#include <tm_lineSegment_byref.hpp>
#include <tm_ioUtil.h>
#include <tm_spatialSearch.hpp>
#include <tm_bestFit.h>

#include <pool_vector.h>
#include <pool_map.h>
#include <pool_set.h>
#include <io_utils.h>
#include <splitParams.h>
#include <vertex.h>
#include <edge.h>
#include <gradingOp.h>
#include <polygon.h>
#include <polyhedron.h>
#include <block.h>
#include <polyMesh.h>
#include <volume.h>
#include <logger.h>
#include <splitter3D.h>
#include <tolerances.h>
#include <utils.h>
#include <meshData.h>
#include <model.h>
#include <triMeshIndex.h>
#include <splitter2D.h>

using namespace std;
using namespace DFHM;

Polyhedron::Polyhedron(const MultiCore::set<Index3DId>& faceIds, const MultiCore::vector<Index3DId>& cornerVertIds)
{
	for (const auto& id : faceIds)
		_faceIds.insert(id);
	_canonicalVertices = cornerVertIds;
}

Polyhedron::Polyhedron(const set<Index3DId>& faceIds, const vector<Index3DId>& cornerVertIds)
{
	for (const auto& id : faceIds)
		_faceIds.insert(id);
	_canonicalVertices.clear();
	_canonicalVertices.insert(_canonicalVertices.end(), cornerVertIds.begin(), cornerVertIds.end());
}

Polyhedron::Polyhedron(const MultiCore::vector<Index3DId>& faceIds, const MultiCore::vector<Index3DId>& cornerVertIds)
{
	_faceIds = faceIds;
	_canonicalVertices = cornerVertIds;
}


Polyhedron::Polyhedron(const vector<Index3DId>& faceIds, const vector<Index3DId>& cornerVertIds)
{
	_faceIds = faceIds;
	_canonicalVertices.insert(_canonicalVertices.end(), cornerVertIds.begin(), cornerVertIds.end());
}

Polyhedron::Polyhedron(const Polyhedron& src)
	: ObjectPoolOwnerUser(src)
	, _faceIds(src._faceIds)
	, _canonicalVertices(src._canonicalVertices)
	, _needsSplitAtCentroid(src._needsSplitAtCentroid)
	, _cachedIsClosed(src._cachedIsClosed)
	, _layerNum(src._layerNum)
{
}

const Index3DId& Polyhedron::getId() const
{
	return _thisId;
}

void Polyhedron::setId(const Index3DId& id)
{
	_thisId = id;
}

void Polyhedron::initializeSearchTree() const
{
	if (!_hasSetSearchTree) {
		auto pBlk = getOurBlockPtr();
		_hasSetSearchTree = true;
		if (pBlk->polyhedronExists(_parentId)) {
			const auto& cell = getPolyhedron(_parentId);
			_pPolySearchTree = cell._pPolySearchTree;
		} else
			_pPolySearchTree = pBlk->getPolySearchTree();

		if (_pPolySearchTree) {
			size_t numInTree = _pPolySearchTree->numInTree();
			if (numInTree == 0)
				_pPolySearchTree = nullptr;
			else if (numInTree > MAX_SUB_TREE_COUNT) {
				auto bbox = getBoundingBox();
				size_t n = _pPolySearchTree->count(bbox, getRefiner());
				if (numInTree > SUB_TREE_SPLIT_RATIO * n) {
					_pPolySearchTree = _pPolySearchTree->getSubTree(bbox, getRefiner());
#if ENABLE_MODEL_SEARCH_TREE_VERIFICATION
					const auto& model = getModel();
					vector<PolyMeshIndex> indicesA, indicesB;
					size_t numA = model.findPolys(bbox, getRefiner(), indicesA);
					if (numA != 0) {
						if (_pPolySearchTree) {
							size_t numB = _pPolySearchTree->find(bbox, getRefiner(), indicesB);
							if (numA == numB) {
								size_t numMismatch = 0;
								for (size_t i = 0; i < numA; i++) {
									if (indicesA[i] != indicesB[i]) {
										numMismatch++;
									}
								}
								if (numMismatch != 0) {
									stringstream ss;
									ss << "initializeSearchTree _pPolySearchTree numMismatch != 0" << __FILE__ << "-" << __LINE__;
									throw runtime_error(ss.str());
								}
							}
							else {
								stringstream ss;
								ss << "initializeSearchTree _pPolySearchTree numA != numB" << __FILE__ << "-" << __LINE__;
								throw runtime_error(ss.str());
							}
						}
						else {
							stringstream ss;
							ss << "initializeSearchTree _pPolySearchTree is null" << __FILE__ << "-" << __LINE__;
							throw runtime_error(ss.str());
						}
					}
#endif
				}
			}
		}
	}
}

void Polyhedron::clear()
{
	ObjectPoolOwnerUser::clear();

	MTC::set<Index3DId> deadFaceIds;
	for (const auto& faceId : _faceIds) {
		faceFunc(faceId, [this, &deadFaceIds](Polygon& face) {
			face.removeCellId(getId());
			if (face.numCells() == 0) {
				deadFaceIds.insert(face.getId());
			}
		});
	}

	_faceIds.clear();

	for (const auto& faceId : deadFaceIds) {
		getBlockPtr()->freePolygon(faceId);
	}
}

const PolyMeshSearchTree::Refiner* Polyhedron::getRefiner() const
{
#if ENABLE_REFINER
	return this;
#else
	return nullptr;
#endif
}

bool Polyhedron::entryIntersects(const Model::BOX_TYPE& bbox, const PolyMeshSearchTree::Entry& entry) const
{
#if 0
	return true;
#else
	Trinary result = IS_UNKNOWN;
	auto& model = getModel();
	auto pFace = model.getPolygon(entry.getIndex());

	const auto& modelVerts = pFace->getVertexIds();
	set<Index3DId> idSet;
	for (const auto& id : modelVerts) {
		idSet.insert(id);
	}
	for (const auto& id : idSet) {
		auto& pt = pFace->getVertexPoint(id);
		if (pointInside(pt)) {
			return true;
		}
	}

	vector<pair<const Vector3d*, const Polygon*>> cellTriPts;
#if 1
	createTriPoints(cellTriPts);
#else
	auto& pt = bbox.getMin();
	auto d = bbox.range();
	Vector3d dx(d[0], 0, 0);
	Vector3d dy(0, d[1], 0);
	Vector3d dz(0, 0, d[2]);
	Vector3d pts[]{
		pt,
		pt + dx,
		pt + dx + dy,
		pt + dy,

		pt + dz,
		pt + dx + dz,
		pt + dx + dy + dz,
		pt + dy + dz,
	};

	// Bottom
	cellTriPts.push_back(make_pair(&pts[0], nullptr));
	cellTriPts.push_back(make_pair(&pts[1], nullptr));
	cellTriPts.push_back(make_pair(&pts[2], nullptr));
	cellTriPts.push_back(make_pair(&pts[0], nullptr));
	cellTriPts.push_back(make_pair(&pts[2], nullptr));
	cellTriPts.push_back(make_pair(&pts[3], nullptr));

	// Top
	cellTriPts.push_back(make_pair(&pts[4], nullptr));
	cellTriPts.push_back(make_pair(&pts[5], nullptr));
	cellTriPts.push_back(make_pair(&pts[6], nullptr));
	cellTriPts.push_back(make_pair(&pts[4], nullptr));
	cellTriPts.push_back(make_pair(&pts[6], nullptr));
	cellTriPts.push_back(make_pair(&pts[7], nullptr));

	// Front
	cellTriPts.push_back(make_pair(&pts[0], nullptr));
	cellTriPts.push_back(make_pair(&pts[1], nullptr));
	cellTriPts.push_back(make_pair(&pts[5], nullptr));
	cellTriPts.push_back(make_pair(&pts[0], nullptr));
	cellTriPts.push_back(make_pair(&pts[5], nullptr));
	cellTriPts.push_back(make_pair(&pts[4], nullptr));

	// Back
	cellTriPts.push_back(make_pair(&pts[3], nullptr));
	cellTriPts.push_back(make_pair(&pts[2], nullptr));
	cellTriPts.push_back(make_pair(&pts[6], nullptr));
	cellTriPts.push_back(make_pair(&pts[3], nullptr));
	cellTriPts.push_back(make_pair(&pts[6], nullptr));
	cellTriPts.push_back(make_pair(&pts[7], nullptr));

	// Left
	cellTriPts.push_back(make_pair(&pts[0], nullptr));
	cellTriPts.push_back(make_pair(&pts[3], nullptr));
	cellTriPts.push_back(make_pair(&pts[7], nullptr));
	cellTriPts.push_back(make_pair(&pts[0], nullptr));
	cellTriPts.push_back(make_pair(&pts[7], nullptr));
	cellTriPts.push_back(make_pair(&pts[4], nullptr));

	// Right
	cellTriPts.push_back(make_pair(&pts[1], nullptr));
	cellTriPts.push_back(make_pair(&pts[2], nullptr));
	cellTriPts.push_back(make_pair(&pts[6], nullptr));
	cellTriPts.push_back(make_pair(&pts[1], nullptr));
	cellTriPts.push_back(make_pair(&pts[6], nullptr));
	cellTriPts.push_back(make_pair(&pts[5], nullptr));
#endif

	size_t nTris = cellTriPts.size() / 3;
	auto pMeshTriData = cellTriPts.data();
	pFace->intersectHexMeshTris(nTris, pMeshTriData, result);

	return result == IS_TRUE;
#endif
}

Polyhedron& Polyhedron::operator = (const Polyhedron& rhs)
{
	ObjectPoolOwnerUser::operator=(rhs);
	clearCache();
	auto tmp = _faceIds;
	for (const auto& faceId : tmp) {
		if (getBlockPtr()->polygonExists(faceId)) {
			bool isDetached = false;
			faceFunc(faceId, [this, &isDetached](Polygon& face) {
				face.removeCellId(getId());
				isDetached = face.getCellIds().empty();
			});
			if (isDetached)
				getBlockPtr()->freePolygon(faceId);
		}
	}

	_thisId = rhs._thisId;
	_faceIds = rhs._faceIds;
	_canonicalVertices = rhs._canonicalVertices;
	_layerNum = rhs._layerNum;
	_needsSplitAtCentroid = rhs._needsSplitAtCentroid;
	_hasSetSearchTree = rhs._hasSetSearchTree;
	_pPolySearchTree = rhs._pPolySearchTree;

	copyCaches(rhs);

	return *this;
}

void Polyhedron::copyCaches(const Polyhedron& src)
{
	_needsCurvatureCheck = src._needsCurvatureCheck;

	_cachedIsClosed = src._cachedIsClosed;
	_cachedIntersectsModel = src._cachedIntersectsModel; // Cached value
	_sharpEdgesIntersectModel = src._sharpEdgesIntersectModel; // Cached value

	_cachedMinGap = src._cachedMinGap;
	_cachedComplexityScore = src._cachedComplexityScore;
	_maxOrthogonalityAngleRadians = src._maxOrthogonalityAngleRadians;

	// The axes are cached separately because they are accessed separately when determining best split axis
	_cachedCurvatureHexXYPlane = src._cachedCurvatureHexXYPlane;
	_cachedCurvatureHexYZPlane = src._cachedCurvatureHexYZPlane;
	_cachedCurvatureHexZXPlane = src._cachedCurvatureHexZXPlane;

	_cachedCtr = src._cachedCtr;
	_cachedBBox = src._cachedBBox;
	_hasSetSearchTree = src._hasSetSearchTree;
	_pPolySearchTree = src._pPolySearchTree;

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

MTC::vector<size_t> Polyhedron::getSharpVertIndices() const
{
	MTC::vector<size_t> result;
#if 0
	auto bbox = getBoundingBox();
	auto pMesh = getBlockPtr()->getModelMesh();
	const auto& allVerts = getBlockPtr()->getVolume()->getSharpVertIndices();
	for (size_t vertIdx : allVerts) {
		const auto& pt = pMesh->getVert(vertIdx)._pt;
		if (bbox.contains(pt, Tolerance::sameDistTol())) {
			result.push_back(vertIdx);
		}
	}
#endif
	return result;
}

bool Polyhedron::getSharpEdgeIndices(MTC::vector<size_t>& result, const SplittingParams& params) const
{
	const double sinEdgeAngle = sin(params.getSharpAngleRadians());

	result.clear();

#if 0
	auto pMesh = getBlockPtr()->getModelMesh();
	for (size_t edgeIdx : _edgeIndices) {
		if (pMesh->isEdgeSharp(edgeIdx, sinEdgeAngle)) {
			result.push_back(edgeIdx);
		}
	}
#endif
	return !result.empty();
}

MTC::vector<Index3DId> Polyhedron::getParents() const
{
	MTC::vector<Index3DId> result;
	auto curId = _parentId;
	result.push_back(curId);
	while (curId.isValid()) {
		auto& parent = getPolyhedron(_parentId);
		curId = parent._parentId;
		result.push_back(curId);
	}
	return result;
}


void Polyhedron::makeHexCellHexPoints(int axis, SubCellResults& result) const
{
	auto& cornerVertIds = getCanonicalVertIds();
	if (cornerVertIds.size() != 8) {
		stringstream ss;
		ss << "Polyhedron::makeHexCellHexPoints not hexhedral " << __FILE__ << ":" << __LINE__;
		throw runtime_error(ss.str());
	}

	MTC::vector<Vector3d> cp;
	for (const auto& id : cornerVertIds)
		cp.push_back(getVertexPoint(id));

	for (int i = 0; i < 2; i++) {
		SubCell subCell;

		double t0 = 0, t1 = 1;
		double u0 = 0, u1 = 1;
		double v0 = 0, v1 = 1;

		switch (axis) {
		case 0:
			t0 = (i == 0) ? 0 : 0.5;
			t1 = (i == 0) ? 0.5 : 1;
			break;
		case 1:
			u0 = (i == 0) ? 0 : 0.5;
			u1 = (i == 0) ? 0.5 : 1;
			break;
		case 2:
			v0 = (i == 0) ? 0 : 0.5;
			v1 = (i == 0) ? 0.5 : 1;
			break;
		}

		subCell._cellPoints = {
			TRI_LERP(cp, t0, u0, v0),
			TRI_LERP(cp, t1, u0, v0),
			TRI_LERP(cp, t1, u1, v0),
			TRI_LERP(cp, t0, u1, v0),

			TRI_LERP(cp, t0, u0, v1),
			TRI_LERP(cp, t1, u0, v1),
			TRI_LERP(cp, t1, u1, v1),
			TRI_LERP(cp, t0, u1, v1),
		};
		result._subCells.push_back(subCell);
		const auto& subPts = subCell._cellPoints;

		if (i == 0) {
			switch (axis) {
			case 0:
				result._partingFacePts = { subPts[1], subPts[5], subPts[6], subPts[2], };
				break;
			case 1:
				result._partingFacePts = { subPts[2], subPts[6], subPts[7], subPts[3], };
				break;
			case 2:
				result._partingFacePts = { subPts[4], subPts[7], subPts[6], subPts[5], };
				break;
			}
		}
	}

}

void Polyhedron::makeHexCellWedgePoints(int axis, SplitParity parity, MTC::vector<MTC::vector<Vector3d>>& subCells,
	MTC::vector<Vector3d>& partingFacePts) const
{
	/*
	There are three ways to split a hex into hexes, but 6 ways to split a hex into wedges.
	Visuallizing a single square face, it can be split lower-left to upper-right and upper-left to lower right
	This can be done one each axis resulting 6 possible splits. The choice of split is referred to as parity, which is not quite correct
	but works.
	*/
	MTC::vector<Vector3d> cp;
	getCanonicalPoints(cp);

	if (cp.size() != 8) {
		stringstream ss;
		ss << "Polyhedron::makeHexCellWedgePoints not hexhedral " << __FILE__ << ":" << __LINE__;
		throw runtime_error(ss.str());
	}

	auto addCellPts = [&cp, &subCells](const vector<size_t>& indices) {
		if (indices.size() != 6) {
			stringstream ss;
			ss << "Polyhedron::makeHexCellWedgePoints not wedge " << __FILE__ << ":" << __LINE__;
			throw runtime_error(ss.str());
		}

		MTC::vector<Vector3d> subPts(6);
		for (size_t i = 0; i < 6; i++) {
			subPts[i] = cp[indices[i]];
		}
		subCells.push_back(subPts);
	};

	if (parity == SP_0) {
		switch (axis) {
			case 0:
				addCellPts({ 0,3,7, 1,2,6 });
				addCellPts({ 0,7,4, 1,6,5 });
				partingFacePts = { cp[0], cp[1], cp[6], cp[7] };
				break;
			case 1:
				addCellPts({ 0,1,5, 3,2,6 });
				addCellPts({ 0,5,4, 3,6,7 });
				partingFacePts = { cp[0], cp[5], cp[6], cp[3] };
				break;
			case 2:
				addCellPts({ 0,1,2, 4,5,6 });
				addCellPts({ 0,2,3, 4,6,7 });
				partingFacePts = { cp[0], cp[2], cp[6], cp[4] };
				break;
		}
	}
	else {
		switch (axis) {
			case 0:
				addCellPts({ 4,0,3, 5,1,2 });
				addCellPts({ 4,3,7, 5,2,6 });
				partingFacePts = { cp[4], cp[5], cp[2], cp[3] };
				break;
			case 1:
				addCellPts({ 4,0,1, 7,3,2 });
				addCellPts({ 4,1,5, 7,2,6 });
				partingFacePts = { cp[4], cp[1], cp[2], cp[7] };
				break;
			case 2:
				addCellPts({ 1,2,3, 5,6,7 });
				addCellPts({ 1,3,0, 5,7,4 });
				partingFacePts = { cp[1], cp[5], cp[7], cp[3] };
				break;
			}
	}
}

void Polyhedron::makeHexCellHexFacePoints(int axis, double w, MTC::vector<Vector3d>& facePts) const
{
	MTC::vector<Vector3d> cp;
	getCanonicalPoints(cp);

	facePts.resize(4);
	switch (axis) {
	case 0: {
		Vector3d pts0[] = { cp[0], cp[3], cp[7], cp[4] };
		Vector3d pts1[] = { cp[1], cp[2], cp[6], cp[5] };
		for (size_t i = 0; i < 4; i++)
			facePts[i] = LERP(pts0[i], pts1[i], w);
		break;
	}
	case 1: {
		Vector3d pts0[] = { cp[0], cp[1], cp[5], cp[4] };
		Vector3d pts1[] = { cp[3], cp[2], cp[6], cp[7] };
		for (size_t i = 0; i < 4; i++)
			facePts[i] = LERP(pts0[i], pts1[i], w);
		break;
	}
	case 2: {
		Vector3d pts0[] = { cp[0], cp[1], cp[2], cp[3] };
		Vector3d pts1[] = { cp[4], cp[5], cp[6], cp[7] };
		for (size_t i = 0; i < 4; i++)
			facePts[i] = LERP(pts0[i], pts1[i], w);
		break;
	}
	}
}

void Polyhedron::makeHexCellWedgeFacePoints(int axis, SplitParity parity, MTC::vector<Vector3d>& facePts) const
{
	/*
	There are three ways to split a hex into hexes, but 6 ways to split a hex into wedges.
	Visuallizing a single square face, it can be split lower-left to upper-right and upper-left to lower right
	This can be done one each axis resulting 6 possible splits. The choice of split is referred to as parity, which is not quite correct
	but works.
	*/
	MTC::vector<Vector3d> cp;
	getCanonicalPoints(cp);

	if (parity == SP_0) {
		switch (axis) {
		case 0:
			facePts = { cp[0], cp[1], cp[6], cp[7] };
			break;
		case 1:
			facePts = { cp[0], cp[5], cp[6], cp[3] };
			break;
		case 2:
			facePts = { cp[0], cp[2], cp[6], cp[4] };
			break;
		}
	}
	else {
		switch (axis) {
		case 0:
			facePts = { cp[4], cp[5], cp[2], cp[3] };
			break;
		case 1:
			facePts = { cp[4], cp[1], cp[2], cp[7] };
			break;
		case 2:
			facePts = { cp[1], cp[5], cp[7], cp[3] };
			break;
		}
	}
}

namespace
{
	struct IntersectionRec {
		double t;
		Vector3d norm;
		inline bool operator < (const IntersectionRec& rhs) const
		{
			if (t < rhs.t)
				return true;
			else if (rhs.t < t)
				return false;

			return norm < rhs.norm;
		}
	};
}
bool Polyhedron::createSplittingFacePoints(MTC::vector<Vector3d>& pts) const
{
	auto pTree = getPolySearchTree();
	if (!pTree)
		return false;

#if 1

#else
	auto& model = getModel();
	const auto& bbox = getBoundingBox();
	MTC::set<EdgeKey> edgeKeys;
	getCanonicalHexEdgeKeys(edgeKeys);

	map<EdgeKey, set<IntersectionRec>> edgeToPointMap;
	for (const auto& ek : edgeKeys) {
		edgeFunc(ek, [this, &model, &bbox, &pTree, &edgeToPointMap](const Edge& edge) {
			auto seg = edge.getSegment();

			pTree->traverse(bbox, [this, &model, &edge, &seg, &edgeToPointMap](const PolyMeshIndex& polyIdx)->bool {
				const auto pFace = model.getPolygon(polyIdx);
				RayHitd hp;
				if (pFace && pFace->intersect(seg, hp)) {
					auto curId = getBlockPtr()->findVertexIdOfPoint(hp.hitPt);
					if (!curId.isValid()) {
						EdgeKey ek = edge;
						auto iter = edgeToPointMap.find(ek);
						if (iter == edgeToPointMap.end())
							iter = edgeToPointMap.insert(make_pair(ek, set<IntersectionRec>())).first;

						IntersectionRec ir;
						ir.t = seg.parameterize(hp.hitPt);
						ir.norm = pFace->calUnitNormal();
						iter->second.insert(ir);
					}
				}
				return true;
			});
		});
	}

	{
		static mutex mut;
		lock_guard lg(mut);
		cout << "Cell(" << getId() << ")\n";
		for (const auto& pair : edgeToPointMap) {
			cout << "  Edge(" << pair.first[0] << ", " << pair.first[1] << ")\n";
			const auto& irs = pair.second;
			for (const auto& ir : irs) {
				cout << "    t: " << ir.t << ", n: (" << ir.norm << ")\n";
			}
		}
		cout << "\n";
	}
#endif
	return false;
}

void Polyhedron::write(ostream& out) const
{
	uint8_t version = 0;
	IoUtil::write(out, version);

	IoUtil::writeObj(out, _faceIds);
	IoUtil::writeObj(out, _canonicalVertices);

	_parentId.write(out);

	out.write((char*)&_layerNum, sizeof(_layerNum));
}

void Polyhedron::read(istream& in)
{
	uint8_t version;
	in.read((char*)&version, sizeof(version));

	IoUtil::readObj(in, _faceIds);
	IoUtil::readObj(in, _canonicalVertices);

	_parentId.read(in);

	in.read((char*)&_layerNum, sizeof(_layerNum));
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

void Polyhedron::remapId(const vector<size_t>& idRemap, const Index3D& srcDims)
{
	remap(idRemap, srcDims, _thisId);
	remap(idRemap, srcDims, _faceIds);
	remap(idRemap, srcDims, _canonicalVertices);
}

void Polyhedron::addFace(const Index3DId& faceId)
{
	_faceIds.insert(faceId);
#if ENABLE_FAST_BISECTION_VALIDATION
	assert(_faceIds.isSorted());
#endif
	faceFunc(faceId, [this](Polygon& face) {
		face.addCellId(getId());
	});

	clearCache();
}

void Polyhedron::removeFace(const Index3DId& faceId)
{
	_faceIds.erase(faceId);
	faceFunc(faceId, [this](Polygon& face) {
		face.removeCellId(getId());
	});

	clearCache();
}

size_t Polyhedron::getVertIds(MTC::set<Index3DId>& result) const
{
	for (const auto& faceId : _faceIds) {
		faceFunc(faceId, [this, &result](const Polygon& face) {
			const auto& v = face.getVertexIds();
			result.insert(v.begin(), v.end());
		});
	}

	return result.size();
}

size_t Polyhedron::getNumFaces() const
{
	return _faceIds.size();
}

void Polyhedron::getCanonicalPoints(MTC::vector<Vector3d>& pts) const
{
	pts.clear();
	for (const auto& id : _canonicalVertices) {
		pts.push_back(getVertexPoint(id));
	}
}

void Polyhedron::getCanonicalHexEdgeKeys(MTC::set<EdgeKey>& eks, int ignoreAxis) const
{
	if (ignoreAxis == -1) {
		for (size_t i = 0; i < 4; i++) {
			size_t j = (i + 1) % 4;
			eks.insert(EdgeKey(_canonicalVertices[i], _canonicalVertices[j]));
			eks.insert(EdgeKey(_canonicalVertices[i + 4], _canonicalVertices[j + 4]));
			eks.insert(EdgeKey(_canonicalVertices[i], _canonicalVertices[i + 4]));
		}
	} else {
		switch (ignoreAxis) {
		case 0:
			eks.insert(EdgeKey(_canonicalVertices[0], _canonicalVertices[3]));
			eks.insert(EdgeKey(_canonicalVertices[3], _canonicalVertices[7]));
			eks.insert(EdgeKey(_canonicalVertices[7], _canonicalVertices[4]));
			eks.insert(EdgeKey(_canonicalVertices[4], _canonicalVertices[0]));

			eks.insert(EdgeKey(_canonicalVertices[1], _canonicalVertices[2]));
			eks.insert(EdgeKey(_canonicalVertices[2], _canonicalVertices[6]));
			eks.insert(EdgeKey(_canonicalVertices[6], _canonicalVertices[5]));
			eks.insert(EdgeKey(_canonicalVertices[5], _canonicalVertices[1]));
			break;
		case 1:
			eks.insert(EdgeKey(_canonicalVertices[0], _canonicalVertices[1]));
			eks.insert(EdgeKey(_canonicalVertices[1], _canonicalVertices[5]));
			eks.insert(EdgeKey(_canonicalVertices[5], _canonicalVertices[4]));
			eks.insert(EdgeKey(_canonicalVertices[4], _canonicalVertices[0]));

			eks.insert(EdgeKey(_canonicalVertices[3], _canonicalVertices[2]));
			eks.insert(EdgeKey(_canonicalVertices[2], _canonicalVertices[6]));
			eks.insert(EdgeKey(_canonicalVertices[6], _canonicalVertices[7]));
			eks.insert(EdgeKey(_canonicalVertices[7], _canonicalVertices[3]));
			break;
		case 2:
			eks.insert(EdgeKey(_canonicalVertices[0], _canonicalVertices[1]));
			eks.insert(EdgeKey(_canonicalVertices[1], _canonicalVertices[2]));
			eks.insert(EdgeKey(_canonicalVertices[2], _canonicalVertices[3]));
			eks.insert(EdgeKey(_canonicalVertices[3], _canonicalVertices[0]));

			eks.insert(EdgeKey(_canonicalVertices[0 + 4], _canonicalVertices[1 + 4]));
			eks.insert(EdgeKey(_canonicalVertices[1 + 4], _canonicalVertices[2 + 4]));
			eks.insert(EdgeKey(_canonicalVertices[2 + 4], _canonicalVertices[3 + 4]));
			eks.insert(EdgeKey(_canonicalVertices[3 + 4], _canonicalVertices[0 + 4]));
			break;
		}
	}
}

MTC::set<EdgeKey> Polyhedron::getEdgeKeys(bool includeAdjacentCellFaces) const
{
	MTC::set<EdgeKey> result;
	if (includeAdjacentCellFaces) {
		MTC::set<Index3DId> vertIds;
		getVertIds(vertIds);
		for (const auto& vertId : vertIds) {
			vertexFunc(vertId, [&result](const Vertex& vert) {
				vert.getEdges(result);
			});
		}
	} else {
		for (const auto& faceId : _faceIds) {
			faceFunc(faceId, [&result](const Polygon& face) {
				auto edgeKeys = face.getEdgeKeys();
				result.insert(edgeKeys.begin(), edgeKeys.end());
			});
		}
	}

	return result;
}

void Polyhedron::getAdjacentCells(MTC::set<Index3DId>& result) const
{
	MTC::set<Index3DId> vertIds;
	getVertIds(vertIds);
	for (const auto& vertId : vertIds) {
		vertexFunc(vertId, [&result](const Vertex& vertex) {
			auto cellIds = vertex.getCellIds();
			result.insert(cellIds.begin(), cellIds.end());
		});
	}
	result.erase(getId());
}

// Gets the edges for a vertex which belong to this polyhedron
void Polyhedron::getVertEdges(const Index3DId& vertId, FastBisectionSet<EdgeKey>& result, bool includeAdjacentCells) const
{
	auto cellEdgeSet = getEdgeKeys(includeAdjacentCells);
	if (includeAdjacentCells) {
		MTC::set<Index3DId> adjCells;
		getAdjacentCells(adjCells);
		for (const auto& adjCellId : adjCells) {
			cellFunc(adjCellId, [&cellEdgeSet](const Polyhedron& adjCell) {
				const auto& tmp = adjCell.getEdgeKeys(true);
				cellEdgeSet.insert(tmp.begin(), tmp.end());
			});
		}
	}

	for (const auto& edge : cellEdgeSet) {
		if (edge.containsVertex(vertId))
			result.insert(edge);
	}
}

// Gets the faces for a vertex which belong to this polyhedron
FastBisectionSet<Index3DId> Polyhedron::getVertFaces(const Index3DId& vertId) const
{
	FastBisectionSet<Index3DId> result;

	FastBisectionSet<EdgeKey> vertEdgeKeys;
	getVertEdges(vertId, vertEdgeKeys, false);

	for (const auto& edgeKey : vertEdgeKeys) {
		edgeFunc(edgeKey, [&result](const Edge& edge) {
			auto& tmp = edge.getFaceIds();
			result.insert(tmp.begin(), tmp.end());
		});
	}

	return result;
}

size_t Polyhedron::classify(MTC::vector<Vector3d>& corners) const
{
	corners.clear();

	Vector3d cellCtr = calCentroidApprox();

	int numQuads = 0, numTris = 0;
	MTC::vector<Index3DId> baseFaceVerts, oppFaceVerts;
	for (const auto& faceId : _faceIds) {
		faceFunc(faceId, [this, &numQuads, &numTris, &baseFaceVerts](const Polygon& face) {
			const auto& vertIds = face.getVertexIds();
			if (vertIds.size() == 3)
				numTris++;
			else if (vertIds.size() == 4)
				numQuads++;

			if (baseFaceVerts.empty())
				baseFaceVerts = vertIds;
			});
	}

	if (numQuads == 6 && numTris == 0) {
		// May be hexahedral
		auto edges = getEdgeKeys(false);
		for (size_t i = 0; i < baseFaceVerts.size(); i++) {
			size_t j = (i + 1) % baseFaceVerts.size();
			edges.erase(EdgeKey(baseFaceVerts[i], baseFaceVerts[j]));
		}
		for (size_t i = 0; i < baseFaceVerts.size(); i++) {
			for (const auto& e : edges) {
				if (e.containsVertex(baseFaceVerts[i])) {
					oppFaceVerts.push_back(e.getOtherVert(baseFaceVerts[i]));
				}
			}
		}
		if (baseFaceVerts.size() != 4 || oppFaceVerts.size() != 4)
			return -1;

		vector<Vector3d> pts, oppPts;
		Vector3d faceCtr0, faceCtr1;
		for (int i = 0; i < 4; i++) {
			const Vector3d& pt0 = getVertexPoint(baseFaceVerts[i]);
			pts.push_back(pt0);
			faceCtr0 += pt0;

			const Vector3d& pt1 = getVertexPoint(oppFaceVerts[i]);
			oppPts.push_back(pt1);
			faceCtr1 += pt1;
		}
		faceCtr0 /= 4;
		faceCtr1 /= 4;

		Vector3d v, v0, v1, norm;
		v0 = pts[1] - pts[0];
		v1 = pts[3] - pts[0];
		norm = v1.cross(v0).normalized();
		v = (cellCtr - faceCtr0).normalized();
		if (v.dot(norm) < 0) {
			swap(pts[1], pts[3]);
			swap(oppPts[1], oppPts[3]);
		}

		v0 = oppPts[1] - oppPts[0];
		v1 = oppPts[3] - oppPts[0];
		norm = v1.cross(v0).normalized();
		v = (cellCtr - faceCtr1).normalized();
		if (v.dot(norm) < 0) {
			corners.insert(corners.end(), pts.begin(), pts.end());
			corners.insert(corners.end(), oppPts.begin(), oppPts.end());
		}
	}
	else if (numQuads == 1 && numTris == 4) {
		// May be pyramid
	}
	else if (numQuads == 3 && numTris == 2) {
		// May be tri prism
	}
	else if (numQuads == 3 && numTris == 2) {
		// May be tri prism
	}
	else if (numTris == 4 && numQuads == 0) {
		// Must be tetrahedron
	}
	return corners.size();
}

TopologyState Polyhedron::getTopolgyState() const
{
	if (intersectsModel())
		return TOPST_INTERSECTING;

	MTC::set<Index3DId> vertIds;
	getVertIds(vertIds);

	size_t numSolid = 0, numVoid = 0;
	for (const auto& vertId : vertIds) {
		auto& vert = getVertex(vertId);
		auto vTopSt = vert.getTopolgyState();
		switch (vTopSt) {
			default:
		case TOPST_SOLID:
			numSolid++;
			break;
		case TOPST_VOID:
			numVoid++;
			break;
		}
	}

	if (numSolid == vertIds.size())
		return TOPST_SOLID;
	else if (numVoid == vertIds.size())
		return TOPST_VOID;

	return TOPST_INTERSECTING;
}

const CBoundingBox3Dd& Polyhedron::getBoundingBox() const
{
	if (_cachedBBox.empty()) {
		MTC::set<Index3DId> vertIds;
		getVertIds(vertIds);
		for (const auto& vertId : vertIds) {
			_cachedBBox.merge(getVertexPoint(vertId));
		}
	}

	return _cachedBBox;
}

bool Polyhedron::containsVertex(const Index3DId& vertId) const
{
	MTC::set<Index3DId> vertIds;
	getVertIds(vertIds);

	return vertIds.contains(vertId);
}

bool Polyhedron::isVertexConnectedToCell(const Index3DId& cellId) const
{
	bool result = false;
	cellFunc(cellId, [this, &result](const Polyhedron& otherCell) {
		MTC::set<Index3DId> vertIds;
		getVertIds(vertIds);
		for (const auto& id : vertIds) {
			if (otherCell.containsVertex(id)) {
				result = true;
				break;
			}
		}
	});

	return result;
}

const Vector3d& Polyhedron::calCentroid() const
{
	if (_cachedCtr[0] == DBL_MAX) {
		const auto& approxCtr = calCentroidApprox();
		// Use the  curl algorithm over all triangles
		_cachedCtr = Vector3d(0, 0, 0);
		double vol = 0;
		for (const auto& faceId : _faceIds) {
			faceFunc(faceId, [this, &vol, &approxCtr](const Polygon& face) {
				face.iterateTriangles([this, &vol, &approxCtr](const Index3DId& idx0, const Index3DId& idx1, const Index3DId& idx2)->bool {
					const Vector3d axis(1, 0, 0);
					auto pBlk = getBlockPtr();
					const auto& a = pBlk->getVertexPoint(idx0);
					const auto& b = pBlk->getVertexPoint(idx1);
					const auto& c = pBlk->getVertexPoint(idx2);

					auto v = approxCtr - a;

					Vector3d triCtr = (a + b + c) / 3.0;
					Vector3d norm = (b - a).cross(c - a);
					if (v.dot(norm) > 0)
						norm = -norm;

					double triArea = norm.norm() / 2;
					Vector3d unitNorm = norm.normalized();

					double dp = axis.dot(unitNorm);
					double projArea = triArea * dp;
					vol += projArea * triCtr.dot(axis);
					for (int i = 0; i < 3; i++) {
						double ab = a[i] + b[i];
						double bc = b[i] + c[i];
						double ca = c[i] + a[i];

						_cachedCtr[i] += norm[i] * (ab * ab + bc * bc + ca * ca);
					}
					return true;
				});
			});
		}
		_cachedCtr *= 1 / (vol * 48.0);
	}
	return _cachedCtr;
}

Vector3d Polyhedron::calCentroidApprox() const
{
	Vector3d ctr(0, 0, 0);

	for (const auto& id : _canonicalVertices) {
		ctr += getVertexPoint(id);
	}

	ctr /= _canonicalVertices.size();
	
	return ctr;
}

double Polyhedron::calVolume() const
{
	double vol = 0;
	Vector3d cellCtr = calCentroid();
	for (const auto& faceId : _faceIds) {
		auto& face = getPolygon(faceId);
		const Vector3d zAxis(0, 0, 1);
		Vector3d norm = face.calUnitNormal();
		double area;
		Vector3d faceCtr;
		face.calAreaAndCentroid(area, faceCtr);
		Planed pl;
		calOrientatedPlane(faceId, cellCtr, pl);
		double h = faceCtr.dot(zAxis);
		double dp = pl.getNormal().dot(zAxis);
		double deltaVol = dp * h * area;
		vol += deltaVol;
	}
	return vol;
}

double Polyhedron::calCurvature2D(const SplittingParams& params, const MTC::vector<Vector3d>& polyPoints, size_t step) const
{
	const auto tol = Tolerance::sameDistTol();
	double result = 0;
	Planed pl;
	double err;
	if (bestFitPlane(polyPoints, pl, err)) {
		auto pVol = getOurBlockPtr()->getVolume();

		Splitter2DPtr pSection;
		if (pVol->getSection(pl, pSection)) {
			// Do test separately for assertion that the entry was found, but has no contents
			if (pSection) {
				vector<double> curvatures;
				if (pSection->findCurvaturesInPolygon(polyPoints, curvatures) > 0) {
					for (double c : curvatures) {
						if (c > result)
							result = c;
					}
				}
			}
		} else {
//			cout << "getSection failed to find section in list\n";
		}
	}

	return result;
}

bool Polyhedron::isOriented() const
{
	bool result = true;
	auto edgeKeys = getEdgeKeys(false);
	for (const auto& edgeKey : edgeKeys) {
		edgeFunc(edgeKey, [this, &result](const Edge& edge) {
			if (!edge.isOriented(getId()))
				result = false;
		});
	}

	return result;
}

void Polyhedron::classifyEdges(MTC::set<EdgeKey>& convexEdges, MTC::set<EdgeKey>& concaveEdges) const
{
	auto edgeKeys = getEdgeKeys(false);
	for (const auto& edgeKey : edgeKeys) {
		edgeFunc(edgeKey, [this, &convexEdges, &concaveEdges](const Edge& edge) {
			if (edge.isConvex(getId()))
				convexEdges.insert(edge);
			else
				concaveEdges.insert(edge);
		});
	}
}

bool Polyhedron::isConvex() const
{
#if 0 && defined(_DEBUG)
	if (!isClosed()) {
		assert(!"open cell cannot be tested for convexity");
		return false;
	}
#endif

	bool result = true;
	auto& id = getId();
	set<EdgeKey> testedEdgeKeys;
	for (const auto& faceId : _faceIds) {
		auto& face = getPolygon(faceId);
		face.iterateOrientedEdges([&testedEdgeKeys, &id, &result](const Edge& edge)->bool {
			if (!testedEdgeKeys.contains(edge)) {
				testedEdgeKeys.insert(edge);
				if (!edge.isConvex(id)) {
					result = false;
				}
			}

			return !result;
		}, id);
	}

	return result;
}

bool Polyhedron::pointInside(const Vector3d& pt) const
{
	const auto tol = Tolerance::sameDistTol();
	auto& bbox = getBoundingBox();
	if (!bbox.contains(pt, tol))
		return false;

	auto cellCtr = calCentroid();
	for (const auto& faceId : _faceIds) {
		auto& face = getPolygon(faceId);
		Planed pl;
		calOrientatedPlane(faceId, cellCtr, pl);

		double dist = pl.distanceToPoint(pt, false);
		if (dist > Tolerance::paramTol()) {
			return false;
		}
	}

	return true;
}

bool Polyhedron::segInside(const LineSegment_byrefd& seg) const
{
	bool inside = true;
	auto ctr = calCentroidApprox();
	for (const auto& faceId : _faceIds) {
		faceFunc(faceId, [&ctr, &seg, &inside](const Polygon& face) {
			const auto tol = Tolerance::paramTol();
			auto pl = face.calPlane();
			auto ctrDist = pl.distanceToPoint(ctr, false);
			if (ctrDist > 0)
				pl.reverse(); // Vector points out of the cell

			double dist0 = pl.distanceToPoint(seg._pt0, false);
			double dist1 = pl.distanceToPoint(seg._pt1, false);
			if (dist0 > tol || dist1 > tol) {
				RayHitd hp;
				if (!face.intersect(seg, hp)) {
					inside = false;
				}
			}
		});
		if (!inside)
			break;
	}

	return inside;

}

bool Polyhedron::entryIntersectsModel(const PolyMeshIndex& index) const
{
	const auto& tol = Tolerance::sameDistTol();

	bool result = false;
	auto& model = getModel();
	const auto& modelFace = model.getPolygon(index);
	modelFace->iterateTriangles([this, &result, tol](const Index3DId& id0, const Index3DId& id1, const Index3DId& id2)->bool {
		const Vector3d* pts[3] = {
			&getVertexPoint(id0),
			&getVertexPoint(id1),
			&getVertexPoint(id2),
		};

		for (const auto& faceId : _faceIds) {
			const auto& face = getPolygon(faceId);
			face.iterateTriangles([this, &pts, &result, tol](const Index3DId& id0, const Index3DId& id1, const Index3DId& id2)->bool {
				const Vector3d* facePts[] = {
					&getVertexPoint(id0),
					&getVertexPoint(id1),
					&getVertexPoint(id2),
				};

				if (intersectTriTri(pts, facePts, tol)) {
					result = true;
					return false;
				}

				return true; // false exits the lambda for loop
				});

			if (result)
				break;
		}

		return !result;
	});
	return result;
}

bool Polyhedron::intersectsModel() const
{
#if 0 && ENABLE_DEBUGGING_MUTEXES
	static mutex lockMutexPtrMutex, lockMutex;
	shared_ptr<lock_guard<mutex>> pLg;
	{
		lock_guard lg(lockMutexPtrMutex);
		pLg = make_shared<lock_guard<mutex>>(lockMutex);
	}
#endif

	if (_cachedIntersectsModel == IS_UNKNOWN) {
		for (const auto& id : _faceIds) {
			auto& face = getPolygon(id);
			if (face.intersectsModel()) {
				_cachedIntersectsModel = IS_TRUE;
				return true;
			}
		}

		auto pTree = getPolySearchTree();
		if (pTree) {
			vector<pair<const Vector3d*, const Polygon*>> cellTriPts;
			createTriPoints(cellTriPts);

			auto& model = getModel();
			auto& bbox = getBoundingBox();
			pTree->traverse(bbox, [this, &cellTriPts, &model](const PolyMeshIndex& polyIdx)->bool {
				size_t nTris = cellTriPts.size() / 3;
				auto pMeshTriData = cellTriPts.data();
				auto pModelFace = model.getPolygon(polyIdx);

				pModelFace->intersectHexMeshTris(nTris, pMeshTriData, _cachedIntersectsModel);

				return _cachedIntersectsModel != IS_TRUE;
			});
		} else
			_cachedIntersectsModel = IS_FALSE;
	}
	return _cachedIntersectsModel == IS_TRUE; // Don't test split cells
}

void Polyhedron::createTriPoints(vector<pair<const Vector3d*, const Polygon*>>& cellTriPts) const
{
	for (const auto& id : _faceIds) {
		auto& face = getPolygon(id);
		face.iterateTriangles([&face, &cellTriPts](const Index3DId& id0, const Index3DId& id1, const Index3DId& id2)->bool {
			cellTriPts.push_back(make_pair(&face.getVertexPoint(id0), &face));
			cellTriPts.push_back(make_pair(&face.getVertexPoint(id1), &face));
			cellTriPts.push_back(make_pair(&face.getVertexPoint(id2), &face));

			return true;
		});
	}
}

void Polyhedron::createTriPoints(vector<pair<const Vector3d, const Polygon*>>& cellTriPts) const
{
	for (const auto& id : _faceIds) {
		auto& face = getPolygon(id);
		face.iterateTriangles([&face, &cellTriPts](const Index3DId& id0, const Index3DId& id1, const Index3DId& id2)->bool {
			cellTriPts.push_back(make_pair(face.getVertexPoint(id0), &face));
			cellTriPts.push_back(make_pair(face.getVertexPoint(id1), &face));
			cellTriPts.push_back(make_pair(face.getVertexPoint(id2), &face));

			return true;
		});
	}
}

void Polyhedron::setIntersectsModel(bool val)
{
	_cachedIntersectsModel = val ? IS_TRUE : IS_FALSE;
}

bool Polyhedron::sharpEdgesIntersectModel(const SplittingParams& params) const
{
	if (_sharpEdgesIntersectModel == IS_FALSE)
		return false;

	CBoundingBox3Dd bbox = getBoundingBox();
	const double sinSharpEdgeAngle = sin(params.getSharpAngleRadians());
	MTC::vector<size_t> sharpEdges;
	auto& model = getBlockPtr()->getModel();
	for (auto& pData : model) {
		auto& pMesh = pData->getMesh();
		vector<size_t> edgeIndices;
		if (pMesh->findEdges(bbox, edgeIndices)) {
			for (size_t edgeIdx : edgeIndices) {
				if (pMesh->isEdgeSharp(edgeIdx, sinSharpEdgeAngle))
					sharpEdges.push_back(edgeIdx);
			}
		}

		if (sharpEdges.empty())
			return false;

		_sharpEdgesIntersectModel = IS_FALSE;
		for (const auto& faceId : getFaceIds()) {
			faceFunc(faceId, [this, &sharpEdges, &pMesh](const Polygon& face) {
				size_t numHits = 0;
				for (size_t edgeIdx : sharpEdges) {
					const auto& edge = pMesh->getEdge(edgeIdx);
					auto seg = edge.getSeg(pMesh);
					RayHitd hit;
					if (face.intersect(seg, hit)) {
						numHits += 1;
					}
				}
				if (numHits > 1)
					_sharpEdgesIntersectModel = IS_TRUE;
				});

			if (_sharpEdgesIntersectModel == IS_TRUE)
				return true;
		}
	}
	return _sharpEdgesIntersectModel == IS_TRUE;
}

bool Polyhedron::isTooComplex(const SplittingParams& params) const
{
	if (hasTooManyFaces(params))
		return true;

	if (hasTooManySplits())
		return true;

	if (isTooNonOrthogoal(params))
		return true;
	return false;
}

bool Polyhedron::isTooNonOrthogoal(const SplittingParams& params) const
{
	return maxOrthogonalityAngleRadians() > params.maxOrthoAngleRadians;
}

bool Polyhedron::isInsideSolid(const std::vector<Planed>& boundingPlanes) const
{
	auto pTree = getPolySearchTree();

	size_t numInside = 0, numOutside = 0;
	for (const auto& faceId : _faceIds) {
		auto& face = getPolygon(faceId);

		// Boundary faces are ignored
		bool onBoundary = false;
		for (const auto& bp : boundingPlanes) {
			if (face.isCoplanar(bp)) {
				onBoundary = true;
				break;
			}
		}

		if (onBoundary)
			continue;

		auto state = face.isInsideSolid(pTree);
		switch (state) {
		case IS_TRUE:
			numInside++;
			break;
		case IS_FALSE:
			numOutside++;
			break;
		default:
			break;
		}
	}

	if (numOutside > 0)
		return false;

	if (numInside > 0)
		return true;

	return false;
}

bool Polyhedron::hasTooManySplits() const
{
	switch (getCellType()) {
	case CT_HEX:
		return hasTooManySplits_hex();
	case CT_WEDGE:
		return hasTooManySplits_wedge();
	default:
		break;
	}
	return false;
}

bool Polyhedron::hasTooHighCurvature(const SplittingParams& params) const
{

	for (int orthoAxis = 0; orthoAxis < 3; orthoAxis++) {
		initCurvatureByNormalAxis(params, orthoAxis);
	}
	
	bool result = false;
	for (int splittingPlaneNormalAxis = 0; splittingPlaneNormalAxis < 3; splittingPlaneNormalAxis++) {
		if (needsCurvatureSplit(params, splittingPlaneNormalAxis)) {
			result = true;
			break;
		}
	}

	return result;
}

bool Polyhedron::hasTooManyFaces(const SplittingParams& params) const
{
	Utils::Timer tmr(Utils::Timer::TT_polyhedronTooManyFaces);
	MTC::vector<MTC::set<Index3DId>> planarFaceSet;
	if (_faceIds.size() > params.maxCellFaces)
		return true;
#if 0
	createPlanarFaceSet(planarFaceSet);
	for (const auto& fc : planarFaceSet) {
		if (fc.size() > params.maxCoplanarFaces)
			return true;
	}
#endif

	return false;
}

namespace
{
double chordEdgeLenRatio(double curvature, double minCurvature, double len, double sinWedgeAngle)
{
	double result = 0;
	if (curvature > minCurvature) {
		double rad = 1 / curvature;
		double chordLen = 2 * rad * sinWedgeAngle;
		result = len / chordLen;
	}
	return result;
}
}


bool Polyhedron::needsCurvatureSplit(const SplittingParams& params, int splittingPlaneNormalAxis) const
{
	switch (getCellType()) {
	default:
		return false;
	case CT_HEX:
		return needsCurvatureSplitHex(params, splittingPlaneNormalAxis);
	case CT_WEDGE:
		return needsCurvatureSplitWedge(params, splittingPlaneNormalAxis);
	}
}

bool Polyhedron::needsCurvatureSplitHex(const SplittingParams& params, int splittingPlaneNormalAxis) const
{
	assert(getCellType() == CT_HEX);
	const double minCurvature = 1 / params.maxCuvatureRadius;
	const double wedgeAngle = 2 * M_PI / params.curvatureDivsPerCircumference;
	const double sinWedgeAngle = sin(wedgeAngle / 2);

	if (!intersectsModel())
		return false;

	MTC::vector<Vector3d> cp;
	getCanonicalPoints(cp);

	double maxLenChordRatio = 1;
	double len, c0, c1, val0 = 0, val1 = 0;

	Vector3d tuv0(0.5, 0.5, 0.5), tuv1(0.5, 0.5, 0.5);
	tuv0[splittingPlaneNormalAxis] = 0;
	tuv1[splittingPlaneNormalAxis] = 1;
	Vector3d pt0 = TRI_LERP(cp, tuv0[0], tuv0[1], tuv0[2]);
	Vector3d pt1 = TRI_LERP(cp, tuv1[0], tuv1[1], tuv1[2]);
	Vector3d v = pt1 - pt0;

	len = v.norm();

	int orthoAxis0 = (splittingPlaneNormalAxis + 1) % 3;
	int orthoAxis1 = (splittingPlaneNormalAxis + 2) % 3;

	c0 = getCurvatureByNormalAxis(params, orthoAxis0);
	c1 = getCurvatureByNormalAxis(params, orthoAxis1);
	val0 = chordEdgeLenRatio(c0, minCurvature, len, sinWedgeAngle);
	val1 = chordEdgeLenRatio(c1, minCurvature, len, sinWedgeAngle);

	// TODO Need to test for an oblique model surface intersection. One that might cause a concave edge.
	return (val0 > 1 || val1 > 1);
}

bool Polyhedron::needsCurvatureSplitWedge(const SplittingParams& params, int splittingPlaneNormalAxis) const
{
	assert(getCellType() == CT_WEDGE);
	return false;
}

Vector3d Polyhedron::calSpan() const
{
	MTC::vector<Vector3d> cp;
	getCanonicalPoints(cp);
	Vector3d span;
	for (int axis = 0; axis < 3; axis++) {
		Vector3d a, b;
		switch (axis) {
		default:
		case 0:
			a = 0.25 * (cp[0] + cp[3] + cp[7] + cp[4]);
			b = 0.25 * (cp[1] + cp[2] + cp[6] + cp[5]);
			break;
		case 1:
			a = 0.25 * (cp[0] + cp[1] + cp[5] + cp[4]);
			b = 0.25 * (cp[3] + cp[2] + cp[6] + cp[7]);
			break;
		case 2:
			a = 0.25 * (cp[0] + cp[1] + cp[2] + cp[3]);
			b = 0.25 * (cp[4] + cp[5] + cp[6] + cp[7]);
			break;
		}
		span[axis] = (b - a).norm();
	}
	return span;
}


double Polyhedron::maxOrthogonalityAngleRadians() const
{
	if (_maxOrthogonalityAngleRadians < 0) {
		_maxOrthogonalityAngleRadians = 0;
		auto& cellCtr = calCentroid();
		for (auto& id : _faceIds) {
			faceFunc(id, [this, &cellCtr](const Polygon& face) {
				auto& faceCtr = face.calCentroid();
				auto& faceNorm = face.calUnitNormal();
				Vector3d v = (cellCtr - faceCtr).normalized();
				double dp = fabs(v.dot(faceNorm));
				if (dp > 1.0)
					dp = 1.0;
				double angle = acos(dp);
				if (angle > _maxOrthogonalityAngleRadians)
					_maxOrthogonalityAngleRadians = angle;
			});
		}
	}
	return _maxOrthogonalityAngleRadians;
}

void Polyhedron::initCurvatureByNormalAxis(const SplittingParams& params, int orthoAxis) const
{
	double* pCurvature = nullptr;
	switch (orthoAxis) {
	default: {
		stringstream ss;
		ss << "getCurvatureByNormalAxis bad axis - " << __FILE__ << "-" << __LINE__;
		throw runtime_error(ss.str());
	}
	case 0: // X axis is normal to y z
		pCurvature = &_cachedCurvatureHexYZPlane;
		break;
	case 1: // Y axis is normal to x z
		pCurvature = &_cachedCurvatureHexZXPlane;
		break;
	case 2: // Z Axis is normal to x y
		pCurvature = &_cachedCurvatureHexXYPlane;
		break;
	}
	if (*pCurvature > -1)
		return;

	*pCurvature = 0;
	if (!intersectsModel())
		return;

	const size_t steps = params.curvatureSamples;

	for (int step = 0; step < steps; step++) {
		double w = step / (steps - 1.0);
		MTC::vector<Vector3d> facePts;
		makeHexCellHexFacePoints(orthoAxis, w, facePts);

		double c = calCurvature2D(params, facePts, step);

		// Use the maximum because many samples may be empty/zero.
		if (c > *pCurvature) {
			*pCurvature = c;
		}
	}
}

bool Polyhedron::hasTooManySplits_hex() const
{
	auto& canonicalVertIds = getCanonicalVertIds();

	MTC::set<Index3DId> vertIds;
	getVertIds(vertIds);

	if (canonicalVertIds.size() == vertIds.size())
		return false;

	const auto tol = Tolerance::sameDistTol();

	set<Index3DId> canonicalVertIdSet(canonicalVertIds.begin(), canonicalVertIds.end());
	MTC::vector<Vector3d> testPts;
	for (const auto& id : vertIds) {
		if (!canonicalVertIdSet.contains(id)) {
			testPts.push_back(getVertexPoint(id));
		}
	}

	MTC::vector<Vector3d> cp;
	for (const auto& id : canonicalVertIds) {
		cp.push_back(getVertexPoint(id));
	}

	MTC::vector<MTC::vector<Vector3d>> cellFacePts;
	GradingOp::getHexFacePoints(cp, cellFacePts);

	for (const auto& facePts : cellFacePts) {
		Vector3d v0 = facePts[0] - facePts[1];
		Vector3d v1 = facePts[2] - facePts[1];
		Vector3d norm = v1.cross(v0);
		norm.normalize();
		size_t numInside = 0;
		for (const auto& pt : testPts) {
			size_t numNeg = 0, numPos = 0;
			for (size_t i = 0; i < facePts.size(); i++) {
				size_t j = (i + 1) % facePts.size();

				auto& pt0 = facePts[i];
				auto& pt1 = facePts[j];
				Vector3d vLeg = pt1 - pt0;
				vLeg.normalize();
				Vector3d vPerp = vLeg.cross(norm);
				vPerp.normalize();
				Vector3d v = pt - pt0;
				double dist = v.dot(vPerp);

				if (dist < -tol)
					numNeg++;
				else if (dist > tol)
					numPos++;
			}

			if (numNeg == facePts.size() || numPos == facePts.size())
				numInside++;
		}

		if (numInside > 1)
			return true;
	}

	MTC::set<EdgeKey> edgeKeys;
	getCanonicalHexEdgeKeys(edgeKeys);
	for (const auto& ek : edgeKeys) {
		auto& pt0 = getVertexPoint(ek[0]);
		auto& pt1 = getVertexPoint(ek[1]);

		LineSegment_byrefd seg(pt0, pt1);
		double len = seg.calLength();
		size_t numOnEdge = 0;
		for (const auto& pt : testPts) {
			double t;
			if (seg.contains(pt, t, tol)) {
				double l = t * len;
				if (tol < l && l < len - tol) {
					numOnEdge++;
				}
			}
		}
		if (numOnEdge > 1)
			return true;
	}

	return false;
}

bool Polyhedron::hasTooManySplits_wedge() const
{
	return false;
}

double Polyhedron::getCurvatureByNormalAxis(const SplittingParams& params, int orthoAxis) const
{
	initCurvatureByNormalAxis(params, orthoAxis);

	double* pCurvature = nullptr;

	switch (orthoAxis) {
	default: {
		stringstream ss;
		ss << "getCurvatureByNormalAxis bad axis - " << __FILE__ << "-" << __LINE__;
		throw runtime_error(ss.str());
	}
	case 0: // X axis is normal
		pCurvature = &_cachedCurvatureHexYZPlane;
		break;
	case 1: // Y axis is normal
		pCurvature = &_cachedCurvatureHexZXPlane;
		break;
	case 2: // Z Axis is normal
		pCurvature = &_cachedCurvatureHexXYPlane;
		break;
	}

	return *pCurvature;
}

double Polyhedron::getComplexityScore(const SplittingParams& params) const
{
	if (_cachedComplexityScore < 0) {
		_cachedComplexityScore = 1;
		const double PI_OVER_2 = M_PI * 0.5;
		double x;

		MTC::vector<MTC::set<Index3DId>> planarFaceSet;
		createPlanarFaceSet(planarFaceSet);
		for (const auto& s : planarFaceSet) {
			size_t numSubFaces = s.size();
			double subFaceComplexity = 1;
			if (numSubFaces > params.maxCoplanarFaces) {
				subFaceComplexity = 1 + (numSubFaces - params.maxCoplanarFaces) / (double)params.maxCoplanarFaces;
				int dbgBreak = 1;
			}
			x = pow(subFaceComplexity, params.complexitySubFaceFactor);
			_cachedComplexityScore *= x;
		}

		size_t numFaces = getFaceIds().size();
		double faceComplexity = 1;
		if (numFaces > params.maxCellFaces) {
			faceComplexity = 1 + (numFaces - params.maxCellFaces) / (double)params.maxCellFaces;
			int dbgBreak = 1;
		}

		x = pow(faceComplexity, params.complexityFaceFactor);
		_cachedComplexityScore *= x;

		auto orthoAngle = maxOrthogonalityAngleRadians(); // in the range [0, PI/4]
		double orthoComplexity = 1;
		if (orthoAngle > params.maxOrthoAngleRadians) {
			orthoComplexity = 1 + (orthoAngle - params.maxOrthoAngleRadians) / (PI_OVER_2 - params.maxOrthoAngleRadians);
			int dbgBreak = 1;
		}
		x = pow(orthoComplexity, params.complexityOrthoFactor);
		_cachedComplexityScore *= x;
	}
	
	return _cachedComplexityScore;
}

CellType Polyhedron::getCellType() const
{
	if (_canonicalVertices.size() == 8)
		return CT_HEX;
	else if (_canonicalVertices.size() == 6)
		return CT_WEDGE;

	return CT_UNKNOWN;
}

const Vector3d& Polyhedron::getVertexPoint(const Index3DId& vertId) const
{
	return getOurBlockPtr()->getVertexPoint(vertId);
}

void Polyhedron::setNeedsDivideAtCentroid()
{
	_needsSplitAtCentroid = true;
	addToSplitStack();
}

bool Polyhedron::needsDivideAtCentroid() const
{
	return _needsSplitAtCentroid;
}

bool Polyhedron::setNeedsCleanFaces()
{
	if (getBlockPtr()->polyhedronExists(_thisId)) {
		setNeedsDivideAtCentroid();
		return true;
	}

	return false;
}

bool Polyhedron::containsSharps() const
{
	auto vertIndices = getBlockPtr()->getVolume()->getSharpVertIndices();

	auto bbox = getBoundingBox();
	const auto& model = getBlockPtr()->getModel();
	for (const auto& pData : model) {
		auto pMesh = pData->getMesh();
		for (size_t vertIdx : vertIndices) {
			const auto& pt = pMesh->getVert(vertIdx)._pt;
			if (bbox.contains(pt, Tolerance::sameDistTol()))
				return true;
		}

		vector<size_t> triIndices;
		if (pMesh->findTris(bbox, triIndices) > 0) {
		}
	}
	return false;
}

void Polyhedron::attachFaces()
{
	for (const auto& faceId : _faceIds) {
		faceFunc(faceId, [this](Polygon& face) {
			face.addCellId(getId());
		});
	}
}

void Polyhedron::detachFaces()
{
	for (const auto& faceId : _faceIds) {
		faceFunc(faceId, [this](Polygon& face) {
			face.removeCellId(getId());
		});
	}
}

void Polyhedron::connectVertEdgeTopology()
{
	for (const auto& faceId : _faceIds) {
		faceFunc(faceId, [this](Polygon& face) {
			face.connectVertEdgeTopology();
		});
	}
}

void Polyhedron::disconnectVertEdgeTopology()
{
	for (const auto& faceId : _faceIds) {
		faceFunc(faceId, [this](Polygon& face) {
			face.disconnectVertEdgeTopology();
		});
	}
}

void Polyhedron::addToSplitStack()
{
	getOurBlockPtr()->addToSplitStack(_thisId);
}

#ifdef _DEBUG
namespace
{

bool boxesEqualTol(const CBoundingBox3Dd& a, const CBoundingBox3Dd& b)
{
	const double tol = 1.0e-6;

	const auto& minA = a.getMin();
	const auto& minB = b.getMin();
	const auto& maxA = a.getMax();
	const auto& maxB = b.getMax();
	for (int i = 0; i < 3; i++) {
		if (fabs(minA[i] - minB[i]) > tol)
			return false;
		if (fabs(maxA[i] - maxB[i]) > tol)
			return false;
	}
	return true;
}

}

#endif

bool Polyhedron::setNeedToSplitConditional(size_t splitNum, const SplittingParams& params)
{
#if 0 && ENABLE_DEBUGGING_MUTEXES
	static mutex lockMutexPtrMutex, lockMutex;
	shared_ptr<lock_guard<mutex>> pLg;
	{
		lock_guard lg(lockMutexPtrMutex);
		pLg = make_shared<lock_guard<mutex>>(lockMutex);
	}
#endif

	auto span = calSpan();
	bool canSplit = false;
	for (int i = 0; i < 3; i++) {
		if (span[i] > 2 * params.minEdgeLength) {
			canSplit = true;
			break;
		}
	}

	if (!canSplit) // Cell span in each axis is too small so don't split
		return false;

	if (isTooComplex(params)) {
		setNeedsDivideAtCentroid();
		return true;
	} else if (splitNum < params.numIntersectionDivs && intersectsModel()) {
		setNeedsDivideAtCentroid();
		return true;
	} else if (splitNum < params.numCurvatureDivs && hasTooHighCurvature(params)) {
		setNeedsDivideAtCentroid();
		return true;
	}

#if 0
	if (passNum < params.numSharpEdgeIntersectionDivs && sharpEdgesIntersectModel(params)) {
		setNeedsDivideAtCentroid();
		return true;		
	}

#endif
	return false;
}

double Polyhedron::minGap() const
{
#if 0
	if (_cachedMinGap < 0) {
		_cachedMinGap = DBL_MAX;

		auto pTriMesh = getBlockPtr()->getModelMesh();
		auto bbox = getBoundingBox();
		vector<size_t> triIndices;
		if (pTriMesh->processFoundTris(_triIndices, bbox, triIndices)) {
			for (size_t idx : triIndices) {
				double gap = pTriMesh->triGap(idx);
				if (gap < _cachedMinGap)
					_cachedMinGap = gap;
			}
		}
	}
#endif
	return _cachedMinGap;
}

bool Polyhedron::polygonExists(const Index3DId& id) const
{
	return getBlockPtr()->polygonExists(id);
}

void Polyhedron::calOrientatedPlane(const Index3DId& id, Planed& pl) const
{
	auto ctr = calCentroid();
	return calOrientatedPlane(id, ctr, pl);
}

void Polyhedron::calOrientatedPlane(const Index3DId& id, const Vector3d& cellCtr, Planed& pl) const
{
	const auto& face = getPolygon(id);
	auto faceCtr = face.calCentroid();
	auto v = faceCtr - cellCtr;

	pl = face.calPlane();
	auto dp = v.dot(pl.getNormal());
	auto fdp = fabs(dp);
	if (fdp > 0 && dp / fdp < 0)
		pl.reverse();
}

void Polyhedron::createPlanarFaceSet(MTC::vector<MTC::set<Index3DId>>& planarFaceSet) const
{
	planarFaceSet.clear();
	for (const auto& faceId : _faceIds) {
		Planed thisPlane;
		faceFunc(faceId, [&thisPlane](const Polygon& face) {
			thisPlane = face.calPlane();
		});

		bool added = false;
		for (auto& faceSet : planarFaceSet) {
			for (const auto& testFaceId : faceSet) {
				bool isCoplanar;
				faceFunc(testFaceId, [&thisPlane, &isCoplanar](const Polygon& testFace) {
					isCoplanar = testFace.isCoplanar(thisPlane);
				});
				if (isCoplanar) {
					faceSet.insert(faceId);
					added = true;
					break;
				}
			}
			if (added)
				break;
		}
		if (!added) {
			planarFaceSet.push_back(MTC::set<Index3DId>());
			planarFaceSet.back().insert(faceId);
		}
	}
}

void Polyhedron::clearLayerNum()
{
	_layerNum = -1;
}

void Polyhedron::clearAxisSplitBits()
{
	_axisSplitBits = 0;
}

bool Polyhedron::hasAvailableAxisSplits() const
{
	switch (getCellType()) {
	case CT_HEX:
		return _axisSplitBits < 7;
	default:
		break;
	}

	return false;
}

void Polyhedron::setLayerNum(int32_t val, bool force)
{
	if (_layerNum == -2 || force)
		_layerNum = val;
}

void Polyhedron::setLayerNumOnNextPass(int32_t val)
{
	if (_layerNum == -1)
		_layerNum = -2;
}

inline const Model& Polyhedron::getModel() const
{
	return getOurBlockPtr()->getModel();
}

const std::shared_ptr<const PolyMeshSearchTree> Polyhedron::getPolySearchTree() const
{
	initializeSearchTree();
	return _pPolySearchTree;
}

size_t Polyhedron::getPolyIndices(std::vector<PolyMeshIndex>& indices) const
{
	const auto& model = getModel();

	indices.clear();
	auto& bbox = getBoundingBox();
	auto pClipped = getPolySearchTree();
//	auto pClipped = model.getPolySubTree(bbox);
	size_t result = 0;
	if (pClipped)
		result = pClipped->find(bbox, nullptr, indices);

#if ENABLE_MODEL_SEARCH_TREE_VERIFICATION // This turns on very expensive entity search testing.
	vector<PolyMeshIndex> indicesFull;
	size_t dbgResult = model.findPolys(bbox, nullptr, indicesFull);

	if (result != dbgResult) {
		static mutex mut;
		lock_guard lg(mut);
		stringstream ss;
		ss << "Search trees sizes don't match. indices.size: " << indices.size() << ", indicesFull.size() : " << indicesFull.size() << " " << __FILE__ << " : " << __LINE__;
		throw runtime_error(ss.str());
	}
#endif

	return result;
}

MTC::set<EdgeKey> Polyhedron::createEdgesFromVerts(MTC::vector<Index3DId>& vertIds) const
{
	// This function takes vertices and looks for faces which contain exactly 2 of the available verts.
	// The edge may already exist.
	MTC::set<EdgeKey> edgeSet;
	for (const auto& faceId : _faceIds) {
		faceFunc(faceId, [this, &edgeSet, &vertIds](const Polygon& face) {
			MTC::set<Index3DId> vertsInFace;
			for (const auto& vertId : vertIds) {
				if (face.containsVertex(vertId))
					vertsInFace.insert(vertId);
			}
			if (vertsInFace.size() == 2) {
				auto iter = vertsInFace.begin();
				const Index3DId& vertId0 = *iter++;
				const Index3DId& vertId1 = *iter;
				EdgeKey newEdge(vertId0, vertId1);

				edgeSet.insert(newEdge);
			}
		});
	}

	return edgeSet;
}

bool Polyhedron::orderVertIds(MTC::vector<Index3DId>& vertIds) const
{

	MTC::vector<Index3DId> result;
	auto newEdgeSet = createEdgesFromVerts(vertIds);

	const auto& e = *newEdgeSet.begin();
	result.push_back(e.getVertexIds()[0]);
	result.push_back(e.getVertexIds()[1]);

	newEdgeSet.erase(newEdgeSet.begin());
	bool found = true;
	while (found && !newEdgeSet.empty()) {
		found = false;
		const auto& lastVert = result.back();
		for (auto iter = newEdgeSet.begin(); iter != newEdgeSet.end(); iter++) {
			const auto& e = *iter;
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
	if (_cachedIsClosed == Trinary::IS_UNKNOWN) {
		_cachedIsClosed = Trinary::IS_TRUE;
		auto edgeKeys = getEdgeKeys(false);
		for (const auto& edgeKey : edgeKeys) {
			edgeFunc(edgeKey, [this](const Edge& edge) {
				size_t count = 0;
				const auto& faceIds = edge.getFaceIds();
				for (const auto& id : faceIds) {
					if (_faceIds.contains(id)) {
						count++;
					}
				}
				if (count < 2)
					_cachedIsClosed = Trinary::IS_FALSE;
			});

			if (_cachedIsClosed == Trinary::IS_FALSE)
				break;
		}
	}
	return _cachedIsClosed == Trinary::IS_TRUE;
}

bool Polyhedron::lineSegmentIntersects(const LineSegmentd& seg, MTC::vector<RayHitd>& hits, MTC::vector<Index3DId>& faceIds) const
{
	for (const auto& faceId : _faceIds) {
		faceFunc(faceId, [this, &seg, &hits, &faceIds] (const Polygon& face) {
			RayHitd hit;
			if (face.intersect(seg, hit)) {
				hits.push_back(hit);
				faceIds.push_back(face.getId());
			}
		});
	}
	return !faceIds.empty() && hits.size() == faceIds.size();
}

bool Polyhedron::verifyTopology() const
{
	const auto& params = getBlockPtr()->getSplitParams();
	bool valid = true;

	for (const auto& faceId : _faceIds) {
		if (valid && polygonExists(faceId)) {
			faceFunc(faceId, [this, &valid](const Polygon& face) {
				if (valid && !face.usedByCell(getId())) {
					valid = false;
				}
				if (valid && !face.verifyTopology())
					valid = false;
			});
		}
	}

	if (valid && !isClosed()) {
		valid = false;
	}

#if DUMP_BAD_CELL_OBJS
	if (!valid) {
		getBlockPtr()->dumpPolyhedraObj({ getId() }, false, false, false);
	}
#endif

	return valid;
}

void Polyhedron::clearCache() const
{
	_needsCurvatureCheck = true;

	_cachedIntersectsModel = IS_UNKNOWN; // Cached value
	_sharpEdgesIntersectModel = IS_UNKNOWN;
	_cachedIsClosed = IS_UNKNOWN;
	_cachedCtr = Vector3d(DBL_MAX, DBL_MAX, DBL_MAX);
	_cachedBBox = {};

	_cachedCurvatureHexXYPlane = -1;
	_cachedCurvatureHexYZPlane = -1;
	_cachedCurvatureHexZXPlane = -1;

	_cachedMinGap = -1;
	_maxOrthogonalityAngleRadians = -1;
	_cachedComplexityScore = -1;
}

ostream& DFHM::operator << (ostream& out, const Polyhedron& cell)
{
	auto pBlk = cell.getBlockPtr();

	auto edgeKeys = cell.getEdgeKeys(false);
	bool closed = true;
	for (const auto& edgeKey : edgeKeys) {
		cell.edgeFunc(edgeKey, [&closed](const Edge& edge) {
			if (edge.getFaceIds().size() != 2)
				closed = false;
		});

		if (!closed)
			break;
	}

#if LOGGING_VERBOSE_ENABLED

	out << "Cell: c" << cell.getId() << (closed ? " CLOSED" : " ERROR NOT CLOSED") << "\n";
	{
		Logger::Indent indent;

		out << Logger::Pad() << "faces(" << cell._faceIds.size() << "): {\n";
		for (const auto& faceId : cell._faceIds) {
			pBlk->faceFunc(faceId, [&out](const Polygon& face) {
				Logger::Indent indent;
				out << Logger::Pad() << face << "\n";
			});
		}
		out << Logger::Pad() << "}\n";

		if (!closed) {
			out << Logger::Pad() << "edges(" << edgeKeys.size() << "): {\n";
			for (const auto& edgeKey : edgeKeys) {
				Logger::Indent indent;
				out << Logger::Pad() << edgeKey << "\n";
			}
			out << Logger::Pad() << "}\n";
		}
	}
#else
	out << "Cell: c" << cell.getId() << (closed ? " CLOSED" : " ERROR NOT CLOSED") << "\n";
	{
		Logger::Indent indent;

		out << Logger::Pad() << "faceIds(" << cell._faceIds.size() << "): { ";
		for (const auto& faceId : cell._faceIds) {
			out << "f" << faceId << " ";
		}
		out << "}\n";
	}
#endif

return out;
}

//LAMBDA_CLIENT_IMPLS(Polyhedron)
void Polyhedron::vertexFunc(const Index3DId& id, const std::function<void(const Vertex& obj)>& func) const {
	const auto p = getBlockPtr(); 
	if (p) 
		p->vertexFunc(id, func); 
	else {
		const auto p2 = getPolyMeshPtr(); 
		if (p2) 
			p2->vertexFunc(id, func);
	}
} 

void Polyhedron::vertexFunc(const Index3DId& id, const std::function<void(Vertex& obj)>& func) {
	auto p = getBlockPtr(); 
	if (p) 
		p->vertexFunc(id, func); 
	else {
		auto p2 = getPolyMeshPtr(); 
		if (p2) 
			p2->vertexFunc(id, func);
	}
} 
void Polyhedron::faceFunc(const Index3DId& id, const std::function<void(const Polygon& obj)>& func) const {
	const auto p = getBlockPtr(); 
	if (p) 
		p->faceFunc(id, func); 
	else {
		const auto p2 = getPolyMeshPtr(); 
		if (p2) 
			p2->faceFunc(id, func);
	}
} 

void Polyhedron::faceFunc(const Index3DId& id, const std::function<void(Polygon& obj)>& func) {
	auto p = getBlockPtr(); 
	if (p) 
		p->faceFunc(id, func); 
	else {
		auto p2 = getPolyMeshPtr(); 
		if (p2) 
			p2->faceFunc(id, func);
	}
} 

void Polyhedron::cellFunc(const Index3DId& id, const std::function<void(const Polyhedron& obj)>& func) const {
	const auto p = getBlockPtr(); 
	if (p) 
		p->cellFunc(id, func); 
	else {
		const auto p2 = getPolyMeshPtr(); 
		if (p2) 
			p2->cellFunc(id, func);
	}
} 

void Polyhedron::cellFunc(const Index3DId& id, const std::function<void(Polyhedron& obj)>& func) {
	auto p = getBlockPtr(); 
	if (p) 
		p->cellFunc(id, func); 
	else {
		auto p2 = getPolyMeshPtr(); 
		if (p2) 
			p2->cellFunc(id, func);
	}
} 

const Vertex& Polyhedron::getVertex(const Index3DId& id) const {
	const auto p = getBlockPtr(); 
	if (p) 
		return p->getVertex(id); 
	else {
		const auto p2 = getPolyMeshPtr(); 
		if (p2) 
			return p2->getVertex(id);
	} 
	throw std::runtime_error("Entity does not exist");
}  

Vertex& Polyhedron::getVertex(const Index3DId& id) {
	auto p = getBlockPtr(); 
	if (p) 
		return p->getVertex(id); 
	else {
		auto p2 = getPolyMeshPtr(); 
		if (p2) 
			return p2->getVertex(id);
	} 
	throw std::runtime_error("Entity does not exist");
} 

const DFHM::Polygon& Polyhedron::getPolygon(const Index3DId& id) const {
	const auto p = getBlockPtr(); 
	if (p) 
		return p->getPolygon(id); 
	else {
		const auto p2 = getPolyMeshPtr(); 
		if (p2) 
			return p2->getPolygon(id);
	} 
	throw std::runtime_error("Entity does not exist");
}  

DFHM::Polygon& Polyhedron::getPolygon(const Index3DId& id) {
	auto p = getBlockPtr(); 
	if (p) 
		return p->getPolygon(id); 
	else {
		auto p2 = getPolyMeshPtr(); 
		if (p2) 
			return p2->getPolygon(id);
	} 
	throw std::runtime_error("Entity does not exist");
} 

const Polyhedron& Polyhedron::getPolyhedron(const Index3DId& id) const {
	const auto p = getBlockPtr(); 
	if (p) 
		return p->getPolyhedron(id); 
	else {
		const auto p2 = getPolyMeshPtr(); 
		if (p2) 
			return p2->getPolyhedron(id);
	} 
	throw std::runtime_error("Entity does not exist");
}  

Polyhedron& Polyhedron::getPolyhedron(const Index3DId& id) {
	auto p = getBlockPtr(); 
	if (p) 
		return p->getPolyhedron(id); 
	else {
		auto p2 = getPolyMeshPtr(); 
		if (p2) 
			return p2->getPolyhedron(id);
	} 
	throw std::runtime_error("Entity does not exist");
} 

void Polyhedron::edgeFunc(const EdgeKey& key, const std::function<void(const Edge& obj)>& func) const {
	const auto p = getBlockPtr(); 
	if (p) 
		p->edgeFunc(key, func); 
	else {
		const auto p2 = getPolyMeshPtr(); if (p2) p2->edgeFunc(key, func);
	}
} 

void Polyhedron::edgeFunc(const EdgeKey& key, const std::function<void(Edge& obj)>& func) {
	auto p = getBlockPtr(); 
	if (p) 
		p->edgeFunc(key, func); 
	else {
		auto p2 = getPolyMeshPtr(); if (p2) p2->edgeFunc(key, func);
	}
}
