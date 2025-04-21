#pragma once

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

#define LAMBDA_FUNC_DECL(NAME, KEY, CONST, CLASS) \
void NAME##Func(const KEY& key, const std::function<void(CONST CLASS& obj)>& func) CONST;

#define LAMBDA_POINTER_FUNC_DECL(NAME, KEY, CONST, CLASS) \
CONST CLASS& get##NAME(const KEY& id) CONST;

#define LAMBDA_CLIENT_FUNC_DECL(NAME, CONST, CLASS) \
void NAME##Func(const Index3DId& id, const std::function<void(CONST CLASS& obj)>& func) CONST;

#define GET_CLIENT_FUNC_DECL(NAME, CONST, CLASS) \
CONST CLASS& get##NAME(const Index3DId& id) CONST;

#define LAMBDA_POLYMESH_FUNC_IMPL(NAME, KEY, MEMBER_NAME, CONST, CLASS) \
void PolyMesh::NAME##Func(const KEY& id, const function<void(CONST CLASS& obj)>& func) CONST \
{ \
	auto p = getOwnerAsPolyMesh(); \
	if (p->MEMBER_NAME.exists(id)) \
		func(p->MEMBER_NAME[id]); \
}

#define LAMBDA_CLIENT_FUNC_IMPL(CLASS, NAME, CONST, CLASS2) \
void CLASS::NAME##Func(const Index3DId& id, const std::function<void(CONST CLASS2& obj)>& func) CONST \
{ \
	CONST auto p = getBlockPtr(); \
	if (p) \
		p->NAME##Func(id, func); \
	else { \
		CONST auto p2 = getPolyMeshPtr(); \
		if (p2) \
			p2->NAME##Func(id, func); \
	} \
}

#define GET_CLIENT_FUNC_IMPL(CLASS, NAME, CONST, CLASS2) \
CONST CLASS2& CLASS::get##NAME(const Index3DId& id) CONST \
{ \
	CONST auto p = getBlockPtr(); \
	if (p) \
		return p->get##NAME(id); \
	else { \
		CONST auto p2 = getPolyMeshPtr(); \
		if (p2) \
			return p2->get##NAME(id); \
	} \
	throw std::runtime_error("Entity does not exist"); \
}

#define LAMBDA_CLIENT_FUNC_EDGE_DECL(NAME, CONST) \
void NAME##Func(const EdgeKey& key, const std::function<void(CONST Edge& obj)>& func) CONST;

#define LAMBDA_FUNC_EDGE_IMPL(NAME, CONST) \
void Block::NAME##Func(const EdgeKey& key, const function<void(CONST Edge& obj)>& func) CONST \
{ \
	auto& idx = getBlockIdx();\
	CONST Block* p;\
	if (idx.withinRange(key[0]))\
		p = getOwner(key[0]);\
	else if (idx.withinRange(key[1]))\
		p = getOwner(key[1]);\
	if (p) {\
		Edge edge(key, p);\
		func(edge);\
	}\
}

#define LAMBDA_CLIENT_FUNC_EDGE_IMPL(CLASS, NAME, CONST) \
void CLASS::NAME##Func(const EdgeKey& key, const std::function<void(CONST Edge& obj)>& func) CONST \
{ \
	CONST auto p = getBlockPtr(); \
	p->NAME##Func(key, func); \
}

/****************************************************************************************************/

#define LAMBDA_BLOCK_DECLS(name, KEY, CLASS) \
LAMBDA_FUNC_DECL(name, KEY, const, CLASS) \
LAMBDA_FUNC_DECL(name, KEY, , CLASS) \
LAMBDA_POINTER_FUNC_DECL(CLASS, KEY, const, CLASS) \
LAMBDA_POINTER_FUNC_DECL(CLASS, KEY, ,CLASS) 

#define LAMBDA_CLIENT_FUNC_DUAL_DECLS(NAME, CLASS) \
LAMBDA_CLIENT_FUNC_DECL(NAME, const, CLASS) \
LAMBDA_CLIENT_FUNC_DECL(NAME, , CLASS)

#define GET_CLIENT_FUNC_DUAL_DECL(NAME) \
GET_CLIENT_FUNC_DECL(NAME, const, NAME) \
GET_CLIENT_FUNC_DECL(NAME, , NAME)

#define LAMBDA_CLIENT_DECLS \
LAMBDA_CLIENT_FUNC_DUAL_DECLS(vertex, Vertex) \
LAMBDA_CLIENT_FUNC_DUAL_DECLS(face, Polygon) \
LAMBDA_CLIENT_FUNC_DUAL_DECLS(cell, Polyhedron) \
GET_CLIENT_FUNC_DUAL_DECL(Vertex) \
GET_CLIENT_FUNC_DUAL_DECL(Polygon) \
GET_CLIENT_FUNC_DUAL_DECL(Polyhedron) \
LAMBDA_CLIENT_FUNC_EDGE_DECL(edge, const) \
LAMBDA_CLIENT_FUNC_EDGE_DECL(edge, )


#define LAMBDA_POLYMESH_IMPLS(NAME, KEY, MEMBER_NAME, CLASS) \
LAMBDA_POLYMESH_FUNC_IMPL(NAME, KEY, MEMBER_NAME, const, CLASS) \
LAMBDA_POLYMESH_FUNC_IMPL(NAME, KEY, MEMBER_NAME, , CLASS)

#define LAMBDA_POLYMESH_EDGE_IMPLS(NAME, CLASS) \
LAMBDA_POLYMESH_FUNC_EDGE_IMPL(NAME, const, CLASS) \
LAMBDA_POLYMESH_FUNC_EDGE_IMPL(NAME, , CLASS)

#define LAMBDA_CLIENT_FUNC_DUAL_IMPLS(CLASS, NAME, CLASS2) \
LAMBDA_CLIENT_FUNC_IMPL(CLASS, NAME, const, CLASS2) \
LAMBDA_CLIENT_FUNC_IMPL(CLASS, NAME, , CLASS2)

#define GET_CLIENT_FUNC_DUAL_IMPLS(CLASS, NAME, CLASS2) \
GET_CLIENT_FUNC_IMPL(CLASS, NAME, const, CLASS2) \
GET_CLIENT_FUNC_IMPL(CLASS, NAME, , CLASS2)

#define LAMBDA_CLIENT_IMPLS(CLASS) \
LAMBDA_CLIENT_FUNC_DUAL_IMPLS(CLASS, vertex, Vertex) \
LAMBDA_CLIENT_FUNC_DUAL_IMPLS(CLASS, face, Polygon) \
LAMBDA_CLIENT_FUNC_DUAL_IMPLS(CLASS, cell, Polyhedron) \
GET_CLIENT_FUNC_DUAL_IMPLS(CLASS, Vertex, Vertex) \
GET_CLIENT_FUNC_DUAL_IMPLS(CLASS, Polygon, DFHM::Polygon) \
GET_CLIENT_FUNC_DUAL_IMPLS(CLASS, Polyhedron, Polyhedron) \
LAMBDA_CLIENT_FUNC_EDGE_IMPL(CLASS, edge, const) \
LAMBDA_CLIENT_FUNC_EDGE_IMPL(CLASS, edge, ) 

