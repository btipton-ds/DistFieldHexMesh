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

#include <tm_spatialSearch.hpp>
#include <tolerances.h>
#include <fixedPoint.h>
#include <io_utils.h>
#include <vertex.h>
#include <edge.h>
#include <polygon.h>
#include <block.h>
#include <polyMesh.h>
#include <volume.h>
#include <debugMeshData.h>
#include <splitParams.h>
#include <rayModelIntersector.h>

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
	_topologyState = rhs._topologyState;
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

void Vertex::getCellIds(MTC::set<Index3DId>& result) const
{
	auto& faceIds = getFaceIds();

	for (const auto& faceId : faceIds) {
		faceFunc(faceId, [&result](const Polygon& face) {
			const auto& cellIds = face.getCellIds();
			result.insert(cellIds.begin(), cellIds.end());
		});
	}
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

bool Vertex::isOnSymPlane(const SplittingParams& params)
{
	auto const tol = Tolerance::sameDistTol();

	Vector3d origin(0, 0, 0);
	if (params.symXAxis) {
		Planed pl(origin, Vector3d(1, 0, 0));
		return pl.isCoincident(_pt, tol);
	}
	if (params.symYAxis) {
		Planed pl(origin, Vector3d(0, 1, 0));
		return pl.isCoincident(_pt, tol);
	}
	if (params.symZAxis) {
		Planed pl(origin, Vector3d(0, 0, 1));
		return pl.isCoincident(_pt, tol);
	}

	return false;
}

void Vertex::markTopologyState()
{
	if (_topologyState == TOPST_UNKNOWN) {
		_topologyState = TOPST_VOID;
		const auto tol = Tolerance::sameDistTol();
		const auto tolFloat = Tolerance::sameDistTolFloat();

		auto& params = getOurBlockPtr()->getSplitParams();
		RayModelIntersector intersector(params, getOurBlockPtr()->getModel(), *this);

		const double randScale = 0.01; // Random 1 cm variation

		// Need a random direction to shoot the ray. May as well use the centroid of the first cell.
		const Vector3d globalX(1, 0, 0), globalY(0, 1, 0), globalZ(0, 0, 1);
		double az = 2 * M_PI * (std::rand() / (double)RAND_MAX);
		double cosAz = cos(az);
		double sinAz = sin(az);

		double el = (-0.5 + (std::rand() / (double)RAND_MAX)) * M_PI;
		double cosEl = cos(el);
		double sinEl = sin(el);

		Vector3d dir(cosAz * cosEl, sinAz * cosEl, sinEl);

		// We require a deterministic, semirandom sequence. So use rand, but always in the same order. 
		// This effectively acts as an infinite table lookup
		std::srand(31415); 

		bool goodCast = true;
		int count = 0;
		do {
			if (goodCast == false) {
				count++;
				if (count > 10) {
					int dbgBreak = 1;
				}
				cout << "Regenerating cast dir " << count << "\n";
				az = 2 * M_PI * (std::rand() / (double)RAND_MAX);
				cosAz = cos(az);
				sinAz = sin(az);

				el = (-0.5 + (std::rand() / (double)RAND_MAX)) * M_PI;
				cosEl = cos(el);
				sinEl = sin(el);

				dir = Vector3d(cosAz * cosEl, sinAz * cosEl, sinEl);
			}

			// Assure the vector points away from all symmetry planes to avoid symmetry issues
			if (params.symXAxis && dir.dot(globalX) < 0) {
				dir[0] = -dir[0]; // Reverse X component
			}
			if (params.symYAxis && dir.dot(globalY) < 0) {
				dir[1] = -dir[1]; // Reverse Y component
			}
			if (params.symZAxis && dir.dot(globalZ) < 0) {
				dir[2] = -dir[2]; // Reverse Z component
			}
			dir.normalize();

			Rayd ray(_pt, dir);
			vector<RayModelIntersector::SolidHitRec> hitsOnSolidModel;
			goodCast = intersector.castRay(ray, hitsOnSolidModel);

			if (goodCast) {
				if (hitsOnSolidModel.empty()) {
					_topologyState = TOPST_VOID;
				} else {
					if (hitsOnSolidModel.size() == 1) {
						// a single hit is unambiguously inside
						_topologyState = TOPST_SOLID;
					} else {
						int parity = hitsOnSolidModel.size() % 2;
						if (parity == 1) {
							_topologyState = TOPST_SOLID;
						}
						else {
							_topologyState = TOPST_VOID;
						}
					}
				}
			}
		} while (!goodCast);
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

void Vertex::write(std::ostream& out) const
{
	uint8_t version = 0;
	IoUtil::write(out, version);

	IoUtil::write(out, _pt);

	int lt = (int)_lockType;
	IoUtil::write(out, lt);

	int ts = (int)_topologyState;
	IoUtil::write(out, ts);

	IoUtil::writeObj(out, _faceIds);
}

void Vertex::read(std::istream& in)
{
	uint8_t version;
	IoUtil::read(in, version);

	IoUtil::read(in, _pt);

	int lt;
	IoUtil::read(in, lt);
	_lockType = (VertexLockType)lt;

	int ts;
	IoUtil::read(in, ts);
	_topologyState = (TopologyState) ts;

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

int64_t Vertex::scaleToSearch()
{
	const int64_t micronInv = 1000000;
	return 100 * micronInv; // 1/100 micron
}

int64_t Vertex::scaleToSearch(double v)
{
	// Rounding is REQUIRED for correct fusing of highly divided edges. DO NOT REMOVE the "+ 0.5" without a lot of verification.
	return (int64_t)(v * scaleToSearch() + 0.5);
}

Vector3<int64_t> Vertex::scaleToSearch(const Vector3d& pt)
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
