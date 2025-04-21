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

#include <vector>

#include <tm_ioUtil.h>
#include <tm_spatialSearch.hpp>
#include <tm_math.h>

#include <tolerances.h>
#include <index3D.h>
#include <splitParams.h>
#include <appData.h>
#include <vertex.h>
#include <edge.h>
#include <polygon.h>
#include <polyMesh.h>
#include <logger.h>
#include <appData.h>

using namespace std;
using namespace DFHM;

#define MEM_STORE_SCALE 10

PolyMesh::PolyMesh()
: _vertices(this, true, MEM_STORE_SCALE * 8 * 8)
, _polygons(this, true, MEM_STORE_SCALE * 8 * 6)
{

}

PolyMesh::PolyMesh(const TriMesh::CMeshPtr& srcMesh)
	: PolyMesh()
{
	size_t nTris = srcMesh->numTris();
	for (size_t i = 0; i < nTris; i++) {
		auto triIndices = srcMesh->getTri(i);
		Polygon newFace;
		for (int i = 0; i < 3; i++) {
			const auto& pt = srcMesh->getVert(triIndices[i])._pt;
			auto vertId = _vertices.findOrAdd(pt);
			newFace.addVertex(vertId);
		}
		_polygons.findOrAdd(newFace);
	}
}

PolyMesh::~PolyMesh()
{

}

const Index3D& PolyMesh::getBlockIdx() const
{
	static Index3D result(0, 0, 0);
	return result;
}

Volume* PolyMesh::getVolume()
{
	return nullptr;
}

const Volume* PolyMesh::getVolume() const
{
	return nullptr;
}

const Block* PolyMesh::getOwner(const Index3D& blockIdx) const
{
	return nullptr;
}

Block* PolyMesh::getOwner(const Index3D& blockIdx)
{
	return nullptr;
}

const PolyMesh* PolyMesh::getPolyMeshPtr() const
{
	return this;
}

PolyMesh* PolyMesh::getPolyMeshPtr()
{
	return this;
}

const Vector3d& PolyMesh::getVertexPoint(const Index3DId& id) const
{
	return _vertices[id];
}

void PolyMesh::makeQuads()
{
	vector<EdgeKey> edges;
	_polygons.iterateInOrder([&edges](const Index3DId& faceId, const Polygon& face)->bool {
		face.iterateEdges([&edges](const Edge& edge) {
			edges.push_back(edge);
			return true;
		});
		return true;
	});

	sort(edges.begin(), edges.end(), [this](const EdgeKey& lhs, const EdgeKey& rhs) {
		double l0, l1;
		edgeFunc(lhs, [&l0](const Edge& edge) {
			l0 = edge.calLength();
		});
		edgeFunc(rhs, [&l1](const Edge& edge) {
			l1 = edge.calLength();
		});

		return l0 > l1;
	});

	for (const auto& ek : edges) {
		edgeFunc(ek, [this](const Edge& edge) {
			mergeToQuad(edge);
		});
	}
	int dbgBreak = 1;
}

void PolyMesh::calCurvatures()
{
	_pointCurvatures.clear();

	map<EdgeKey, double> edges;
	_polygons.iterateInOrder([&edges](const Index3DId& faceId, const Polygon& face)->bool {
		face.iterateEdges([&edges](const Edge& edge) {
			edges.insert(make_pair(edge, 0));
			return true;
		});
		return true;
	});

	for (auto& pair : edges) {
		edgeFunc(pair.first, [&pair](const Edge& edge) {
			pair.second = edge.calCurvature();
		});
	}

	for (auto& pair : edges) {
		const auto& ek = pair.first;
		for (int i = 0; i < 2; i++) {
			const auto& vId = ek[i];
			auto iter = _pointCurvatures.find(vId);
			if (iter == _pointCurvatures.end())
				iter = _pointCurvatures.insert(make_pair(vId, CurvRec())).first;
			iter->second.curvature += pair.second;
			iter->second.count++;
		}
	}

	for (auto& pair : _pointCurvatures) {
		pair.second.curvature /= pair.second.count;
	}
}

void PolyMesh::mergeToQuad(const Edge& edge)
{
	const double MAX_CP = 0.02;
	const double MAX_CP_SQR = MAX_CP * MAX_CP;

	auto& faceIds = edge.getFaceIds();
	if (faceIds.size() != 2)
		return;

	auto iter = faceIds.begin();
	auto faceId0 = *iter++;
	auto faceId1 = *iter;

	if (!_polygons.exists(faceId0) || !_polygons.exists(faceId1))
		return;

	auto& face0 = getPolygon(faceId0);
	auto& face1 = getPolygon(faceId1);

	const auto& vertIds0 = face0.getVertexIds();
	const auto& vertIds1 = face1.getVertexIds();

	if (vertIds0.size() != 3 || vertIds1.size() != 3)
		return;

	if (!isLongestEdge(face0, edge) || !isLongestEdge(face1, edge))
		return;

	// Check for close to coplanar
	auto ncLin0 = PolygonSearchKey::makeNonColinearVertexIds(this, vertIds0);
	auto norm0 = Polygon::calUnitNormalStat(this, ncLin0);

	auto ncLin1 = PolygonSearchKey::makeNonColinearVertexIds(this, vertIds1);
	auto norm1 = Polygon::calUnitNormalStat(this, ncLin1);
	auto cp = norm0.cross(norm1).squaredNorm();
	if (cp > MAX_CP_SQR)
		return;

	Index3DId otherId;
	for (const auto& id : vertIds1) {
		if (!edge.containsVertex(id)) {
			otherId = id;
			break;
		}
	}
	assert(otherId.isValid());

	MTC::vector<Index3DId> newVertIds;
	for (size_t i = 0; i < vertIds0.size(); i++) {
		size_t j = (i + 1) % vertIds0.size();
		newVertIds.push_back(vertIds0[i]);
		if (edge.containsVertex(vertIds0[i]) && edge.containsVertex(vertIds0[j])) {
			newVertIds.push_back(otherId);
		}
	}

	removeFace(faceId0);
	removeFace(faceId1);
	Polygon newFace(newVertIds);
	_polygons.findOrAdd(newFace);

}

void PolyMesh::removeFace(const Index3DId& id)
{
	_polygons[id].disconnectVertEdgeTopology();
	_polygons.removeFromLookup(id);
	_polygons.free(id);
}

bool PolyMesh::isLongestEdge(const Polygon& face, const Edge& edge) const
{
	bool result = true;
	auto l0 = edge.calLength();
	face.iterateEdges([&edge, l0, &result](const Edge& edge1) {
		if (edge != edge1) {
			if (edge.calLength() > l0) {
				result = false;
			}
		}
		return result;
	});
	return result;
}

#define FUNC_IMPL(NAME, KEY, MEMBER_NAME, CONST, CLASS) \
void PolyMesh::NAME##Func(const KEY& id, const function<void(CONST CLASS& obj)>& func) CONST \
{ \
	func(MEMBER_NAME[id]); \
}

#define POINTER_FUNC_IMPL(KEY, MEMBER_NAME, CONST, CLASS) \
CONST DFHM::CLASS& PolyMesh::get##CLASS(const KEY& id) CONST \
{ \
	return MEMBER_NAME[id]; \
}

#define IMPLS(NAME, KEY, MEMBER_NAME, CLASS) \
FUNC_IMPL(NAME, KEY, MEMBER_NAME, const, CLASS) \
FUNC_IMPL(NAME, KEY, MEMBER_NAME, , CLASS) \
POINTER_FUNC_IMPL(KEY, MEMBER_NAME, const, CLASS) \
POINTER_FUNC_IMPL(KEY, MEMBER_NAME, , CLASS) 

#define EDGE_IMPL(NAME, KEY, CONST, CLASS) \
void PolyMesh::NAME##Func(const KEY& key, const function<void(CONST CLASS& obj)>& func) CONST \
{ \
	CLASS edge(key, this);\
	func(edge);\
}

#define EDGE_IMPLS(NAME, KEY, CLASS) \
EDGE_IMPL(NAME, KEY, const, CLASS) \
EDGE_IMPL(NAME, KEY, , CLASS)

#if 0
IMPLS(vertex, Index3DId, _vertices, Vertex)
IMPLS(face, Index3DId, _polygons, Polygon)
EDGE_IMPLS(edge, EdgeKey, Edge)
#endif

void PolyMesh::vertexFunc(const Index3DId& id, const function<void(const Vertex& obj)>& func) const {
	func(_vertices[id]);
} 

void PolyMesh::vertexFunc(const Index3DId& id, const function<void(Vertex& obj)>& func) {
	func(_vertices[id]);
} 

const DFHM::Vertex& PolyMesh::getVertex(const Index3DId& id) const {
	return _vertices[id];
}  

DFHM::Vertex& PolyMesh::getVertex(const Index3DId& id) {
	return _vertices[id];
}

void PolyMesh::faceFunc(const Index3DId& id, const function<void(const Polygon& obj)>& func) const {
	func(_polygons[id]);
} 

void PolyMesh::faceFunc(const Index3DId& id, const function<void(Polygon& obj)>& func) {
	func(_polygons[id]);
} 

const DFHM::Polygon& PolyMesh::getPolygon(const Index3DId& id) const {
	return _polygons[id];
}  

DFHM::Polygon& PolyMesh::getPolygon(const Index3DId& id) {
	return _polygons[id];
}

void PolyMesh::edgeFunc(const EdgeKey& key, const function<void(const Edge& obj)>& func) const {
	Edge edge(key, this); 
	func(edge);
} 

void PolyMesh::edgeFunc(const EdgeKey& key, const function<void(Edge& obj)>& func) {
	Edge edge(key, this);
	func(edge);
}


void PolyMesh::cellFunc(const Index3DId& key, const std::function<void(const Polyhedron& obj)>& func) const
{
	throw runtime_error("Not implemented");
}

void PolyMesh::cellFunc(const Index3DId& key, const std::function<void(Polyhedron& obj)>& func)
{
	throw runtime_error("Not implemented");
}

const Polyhedron& PolyMesh::getPolyhedron(const Index3DId& id) const
{
	throw runtime_error("Not implemented");
}

Polyhedron& PolyMesh::getPolyhedron(const Index3DId& id)
{
	throw runtime_error("Not implemented");
}
