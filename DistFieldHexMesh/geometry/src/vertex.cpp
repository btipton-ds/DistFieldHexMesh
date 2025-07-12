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

#include <tolerances.h>
#include <fixedPoint.h>
#include <io_utils.h>
#include <vertex.h>
#include <edge.h>
#include <polygon.h>
#include <block.h>
#include <polyMesh.h>
#include <polygon.h>
#include <tm_spatialSearch.hpp>

using namespace std;
using namespace DFHM;

Vertex::Vertex(const Vertex& src)
	: ObjectPoolOwnerUser(src)
	, _pt(src._pt)
	, _lockType(src._lockType)
{
}

const Index3DId& Vertex::getId() const
{
	return _thisId;
}

void Vertex::setId(const Index3DId& id)
{
	_thisId = id;
}

void Vertex::getConnectedVertexIds(MTC::set<Index3DId>& ids) const
{
	for (const auto& faceId : _faceIds) {
		const auto& face = getPolygon(faceId);
		const auto& vertIds = face.getVertexIds();
		for (size_t i = 0; i < vertIds.size(); i++) {
			const auto& thisId = vertIds[i];
			if (thisId == getId()) {
				size_t j = (i + 1) % vertIds.size();
				ids.insert(vertIds[j]);
				size_t k = (i + vertIds.size() - 1) % vertIds.size();
				ids.insert(vertIds[k]);
			}
		}
	}
}

void Vertex::remapId(const std::vector<size_t>& idRemap, const Index3D& srcDims)
{
	remap(idRemap, srcDims, _thisId);
	remap(idRemap, srcDims, _faceIds);
}

Vertex& Vertex::operator = (const Vertex& rhs)
{
	ObjectPoolOwnerUser::operator= (rhs);
	_thisId = rhs._thisId;
	_pt = rhs._pt;
	_lockType = rhs._lockType;
	_faceIds = rhs._faceIds;
	_cachedSurfaceNormal = Vector3d(DBL_MAX, DBL_MAX, DBL_MAX);

	return *this;
}

void Vertex::getEdges(MTC::set<EdgeKey>& edges) const
{
	MTC::set<Index3DId> conVerts;
	getConnectedVertexIds(conVerts);
	for (const auto& otherId : conVerts) {
		edges.insert(EdgeKey(getId(), otherId));
	}
}

MTC::set<Index3DId> Vertex::getCellIds() const
{
	MTC::set<Index3DId> result;
	auto& faceIds = getFaceIds();

	for (const auto& faceId : faceIds) {
		faceFunc(faceId, [&result](const Polygon& face) {
			const auto& cellIds = face.getCellIds();
			result.insert(cellIds.begin(), cellIds.end());
		});
	}

	return result;
}

const Vector3d& Vertex::calSurfaceNormal() const
{
	if (_cachedSurfaceNormal[0] == DBL_MAX) {
		_cachedSurfaceNormal = Vector3d(0, 0, 0);
		for (const auto& id : _faceIds) {
			faceFunc(id, [this](const Polygon& face) {
				const auto& n = face.calUnitNormal();
				_cachedSurfaceNormal += n;
			});
		}
		_cachedSurfaceNormal.normalize();
	}
	return _cachedSurfaceNormal;
}

void Vertex::markSolidAndIntersecting()
{
	auto tol = Tolerance::sameDistTol();

	const auto& model = getOurBlockPtr()->getModel();
	auto pTree = model.getPolySearchTree();
	if (!pTree)
		return;

	MTC::set<Index3DId> conVerts;
	getConnectedVertexIds(conVerts);
	for (const auto& otherVertId : conVerts) {
		const auto& otherVert = getVertex(otherVertId);
		const auto& otherPt = otherVert._pt;

		Vector3d dir = otherPt - _pt;
		dir.normalize();
		Rayd ray(_pt, dir);
		vector<MultiPolyMeshRayHit> hitsOnSolidModel;
		pTree->biDirRayCastTraverse(ray, [this, &model, &hitsOnSolidModel](const Rayd& ray, const PolyMeshIndex& idx)->bool {
			if (model.isClosed(idx)) {
				auto pFace = model.getPolygon(idx);
				RayHitd rh;
				if (pFace->intersect(ray, rh)) {
					hitsOnSolidModel.push_back(MultiPolyMeshRayHit(idx, rh));
				}
			}
			return true;
		}, tol);

		if (!hitsOnSolidModel.empty()) {
			sort(hitsOnSolidModel.begin(), hitsOnSolidModel.end(), [](const MultiPolyMeshRayHit& lhs, const MultiPolyMeshRayHit& rhs)->bool {
				return lhs.getDist() < rhs.getDist();
			});

			const auto& closestHitOnSolidModel = hitsOnSolidModel.front();
			auto pModelFace = model.getPolygon(closestHitOnSolidModel);
			Vector3d norm = pModelFace->calUnitNormal();
			Vector3d v = _pt - closestHitOnSolidModel.getPoint();
			double dist = v.dot(norm);

			if (dist < -tol)
				_topologyState = TOPST_SOLID;
			else if (dist > tol)
				_topologyState = TOPST_VOID;
			else
				_topologyState = TOPST_INTERSECTING;

			return;
		}
	}
}

void Vertex::dumpIntersectionObj(const Rayd& ray, const std::vector<PolyMeshIndex>& hits) const
{
	if (!hits.empty()) {
		auto path = getOurBlockPtr()->getObjFilePath();
		string filename(path + "/vertex_intersect.obj");
		ofstream out(filename);

		map<Vector3d, size_t> ptToIndexMap;
		vector<Vector3d> pts;
		vector<vector<size_t>> allFaceIndices;

		pts.push_back(ray._origin);
		ptToIndexMap[pts.back()] = pts.size() - 1;

		pts.push_back(ray._origin + ray._dir * 50);
		ptToIndexMap[pts.back()] = pts.size() - 1;

		auto& model = getOurBlockPtr()->getModel();

		for (const auto& idx : hits) {
			auto pFace = model.getPolygon(idx);
			vector<size_t> faceIndices;
			const auto& faceVertIds = pFace->getVertexIds();
			for (const auto& id : faceVertIds) {
				const auto& pt = pFace->getVertex(id).getPoint();
				auto iter = ptToIndexMap.find(pt);
				if (iter == ptToIndexMap.end()) {
					iter = ptToIndexMap.insert(make_pair(pt, pts.size())).first;
					pts.push_back(pt);
				}
				size_t idx = iter->second;
				faceIndices.push_back(idx);
			}
			allFaceIndices.push_back(faceIndices);
		}

		out << "Points\n";
		for (const auto& pt : pts) {
			out << "v " << pt[0] << " " << pt[1] << " " << pt[2] << "\n";
		}

		out << "Ray\n";
		out << "l " << 1 << " " << 2 << "\n";

		out << "Faces\n";
		for (const auto& faceIndices : allFaceIndices) {
			out << "f";
			for (size_t idx : faceIndices) {
				out << " " << (idx + 1);
			}
			out << "\n";
		}

	}
}

void Vertex::markOthersVoid()
{
	if (_topologyState == TOPST_UNKNOWN) {
		_topologyState = TOPST_VOID;
	}
}

void Vertex::write(std::ostream& out) const
{
	uint8_t version = 0;
	out.write((char*)&version, sizeof(version));

	writeVector3(out, _pt);

	out.write((char*)&_lockType, sizeof(_lockType));
	IoUtil::writeObj(out, _faceIds);
}

void Vertex::read(std::istream& in)
{
	uint8_t version;
	in.read((char*)&version, sizeof(version));

	readVector3(in, _pt);

	in.read((char*)&_lockType, sizeof(_lockType));
	IoUtil::readObj(in, _faceIds);
}

bool Vertex::verifyTopology() const
{
	bool result = true;
	for (const auto& id : _faceIds) {
		faceFunc(id, [this, &result](const Polygon& face) {
			if (!face.containsVertex(getId())) {
				result = false;
			}
		});
		if (!result)
			break;
	}
	return result;
}

CBoundingBox3Dd Vertex::calBBox(const Vector3d& pt)
{
	CBoundingBox3Dd result(pt, pt);
	result.grow(Tolerance::sameDistTol() / 2.0);

	return result;
}

inline int64_t Vertex::scaleToSearch()
{
	const int64_t micronInv = 1000000;
	return 100 * micronInv; // 1/100 micron
}

inline int64_t Vertex::scaleToSearch(double v)
{
	// Rounding is REQUIRED for correct fusing of highly divided edges. DO NOT REMOVE the "+ 0.5" without a lot of verification.
	return (int64_t)(v * scaleToSearch() + 0.5);
}

inline Vector3<int64_t> Vertex::scaleToSearch(const Vector3d& pt)
{
	return Vector3<int64_t>(scaleToSearch(pt[0]), scaleToSearch(pt[1]), scaleToSearch(pt[2]));
}

const bool Vertex::operator < (const Vertex& rhs) const
{
	/*
		this is an innefficient implementation of Edelsbrunner's and Mucke's simulation of simplicity - https://arxiv.org/abs/math/9410209.

		Their approach was to simulate floating point numbers with fixed point integers. This avoids tolerancing errors when doing <, ==, > testing.
		It effectively "rounds" each floating point value to a scaled integer value.

		The original implementation stores the fixed point value and all math is done in that form. That makes floating point math extremely tricky to overflows and underflows.

		This approach stores a double precision value of the point, and only converts it for comparisons. It's a bit time consuming, but it solves vertex sharing across
		blocks which is a key to high performance multithreading.

	*/

	Vector3<int64_t> iPt(scaleToSearch(_pt)), iRhsPt(scaleToSearch(rhs._pt));
	return iPt < iRhsPt;
}

ostream& DFHM::operator << (ostream& out, const Vertex& vert)
{
	out << "Vertex " << vert.getId();
	return out;
}

//LAMBDA_CLIENT_IMPLS(Vertex)
LAMBDA_CLIENT_IMPLS(Vertex)
