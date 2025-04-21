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

const PolyMesh* PolyMesh::getOwnerAsPolyMesh() const
{
	return this;
}

PolyMesh* PolyMesh::getOwnerAsPolyMesh()
{
	return this;
}

#define FUNC_IMPL(NAME, KEY, MEMBER_NAME, CONST, CLASS) \
void PolyMesh::NAME##Func(const KEY& id, const function<void(CONST CLASS& obj)>& func) CONST \
{ \
	auto p = getOwnerAsPolyMesh(); \
	if (p->MEMBER_NAME.exists(id)) \
		func(p->MEMBER_NAME[id]); \
}

#define POINTER_FUNC_IMPL(KEY, MEMBER_NAME, CONST, CLASS) \
CONST DFHM::CLASS& PolyMesh::get##CLASS(const KEY& id) CONST \
{ \
	auto p = getOwnerAsPolyMesh(); \
	if (p->MEMBER_NAME.exists(id)) \
		return p->MEMBER_NAME[id]; \
	throw runtime_error("Object doesn't exit"); \
}

#define IMPLS(NAME, KEY, MEMBER_NAME, CLASS) \
FUNC_IMPL(NAME, KEY, MEMBER_NAME, const, CLASS) \
FUNC_IMPL(NAME, KEY, MEMBER_NAME, , CLASS) \
POINTER_FUNC_IMPL(KEY, MEMBER_NAME, const, CLASS) \
POINTER_FUNC_IMPL(KEY, MEMBER_NAME, , CLASS) 

#define EDGE_IMPL(NAME, KEY, CONST, CLASS) \
void PolyMesh::NAME##Func(const KEY& key, const function<void(CONST CLASS& obj)>& func) CONST \
{ \
	auto& idx = getBlockIdx();\
	CONST auto* p = getOwnerAsPolyMesh();\
	if (p) {\
		Edge edge(key, p);\
		func(edge);\
	}\
}

#define EDGE_IMPLS(NAME, KEY, CLASS) \
EDGE_IMPL(NAME, KEY, const, CLASS) \
EDGE_IMPL(NAME, KEY, , CLASS)


IMPLS(vertex, Index3DId, _vertices, Vertex)
IMPLS(face, Index3DId, _polygons, Polygon)
EDGE_IMPLS(edge, EdgeKey, Edge)

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
