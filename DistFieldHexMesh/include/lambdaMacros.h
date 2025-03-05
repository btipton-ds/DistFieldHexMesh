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

#define LAMBDA_FUNC_DECL(NAME, CONST, CLASS) \
void NAME##Func(const Index3DId& id, const std::function<void(CONST CLASS& obj)>& func) CONST;

#define LAMBDA_CLIENT_FUNC_DECL(NAME, CONST, CLASS) \
void NAME##Func(const Index3DId& id, const std::function<void(CONST CLASS& obj)>& func) CONST;

#define LAMBDA_FUNC_IMPL(NAME, MEMBER_NAME, CONST, CLASS) \
void Block::NAME##Func(const Index3DId& id, const function<void(CONST CLASS& obj)>& func) CONST \
{ \
	auto p = getOwner(id); \
	if (p->MEMBER_NAME.exists(id)) \
		func(p->MEMBER_NAME[id]); \
}

#define LAMBDA_CLIENT_FUNC_IMPL(CLASS, NAME, CONST, CLASS2) \
void CLASS::NAME##Func(const Index3DId& id, const std::function<void(CONST CLASS2& obj)>& func) CONST \
{ \
	CONST auto p = getBlockPtr(); \
	p->NAME##Func(id, func); \
}





#define LAMBDA_FUNC_EDGE_DECL(NAME, CONST) \
void NAME##Func(const EdgeKey& key, const std::function<void(CONST Edge& obj)>& func) CONST;

#define LAMBDA_CLIENT_FUNC_EDGE_DECL(NAME, CONST) \
void NAME##Func(const EdgeKey& key, const std::function<void(CONST Edge& obj)>& func) CONST;

#define LAMBDA_FUNC_EDGE_IMPL(NAME, CONST) \
void Block::NAME##Func(const EdgeKey& key, const function<void(CONST Edge& obj)>& func) CONST \
{ \
	auto p = getOwner(key.getVertex(0)); \
	if (p) { \
		auto pEdge = p->getEdge(key);\
		if (pEdge) \
			func(*pEdge); \
	}\
}

#define LAMBDA_CLIENT_FUNC_EDGE_IMPL(CLASS, NAME, CONST) \
void CLASS::NAME##Func(const EdgeKey& key, const std::function<void(CONST Edge& obj)>& func) CONST \
{ \
	CONST auto p = getBlockPtr(); \
	p->NAME##Func(key, func); \
}

/****************************************************************************************************/

#define LAMBDA_BLOCK_DECLS \
LAMBDA_FUNC_DECL(vertex, const, Vertex) \
LAMBDA_FUNC_DECL(vertex, , Vertex) \
LAMBDA_FUNC_DECL(face, const, Polygon) \
LAMBDA_FUNC_DECL(face, , Polygon) \
LAMBDA_FUNC_DECL(cell, const, Polyhedron) \
LAMBDA_FUNC_DECL(cell, , Polyhedron) \
LAMBDA_FUNC_EDGE_DECL(edge, const) \
LAMBDA_FUNC_EDGE_DECL(edge, )

#define LAMBDA_CLIENT_DECLS \
LAMBDA_CLIENT_FUNC_DECL(vertex, const, Vertex) \
LAMBDA_CLIENT_FUNC_DECL(vertex, , Vertex) \
LAMBDA_CLIENT_FUNC_DECL(face, const, Polygon) \
LAMBDA_CLIENT_FUNC_DECL(face, , Polygon) \
LAMBDA_CLIENT_FUNC_DECL(cell, const, Polyhedron) \
LAMBDA_CLIENT_FUNC_DECL(cell, , Polyhedron) \
LAMBDA_CLIENT_FUNC_EDGE_DECL(edge, const) \
LAMBDA_CLIENT_FUNC_EDGE_DECL(edge, )

#define LAMBDA_BLOCK_IMPLS \
LAMBDA_FUNC_IMPL(vertex, _vertices, const, Vertex) \
LAMBDA_FUNC_IMPL(vertex, _vertices, , Vertex) \
LAMBDA_FUNC_IMPL(face, _polygons, const, Polygon) \
LAMBDA_FUNC_IMPL(face, _polygons, , Polygon) \
LAMBDA_FUNC_IMPL(cell, _polyhedra, const, Polyhedron) \
LAMBDA_FUNC_IMPL(cell, _polyhedra, , Polyhedron) \
LAMBDA_FUNC_EDGE_IMPL(edge, const) \
LAMBDA_FUNC_EDGE_IMPL(edge, )

#define LAMBDA_CLIENT_IMPLS(CLASS) \
LAMBDA_CLIENT_FUNC_IMPL(CLASS, vertex, const, Vertex) \
LAMBDA_CLIENT_FUNC_IMPL(CLASS, vertex, , Vertex) \
LAMBDA_CLIENT_FUNC_IMPL(CLASS, face, const, Polygon) \
LAMBDA_CLIENT_FUNC_IMPL(CLASS, face, , Polygon) \
LAMBDA_CLIENT_FUNC_IMPL(CLASS, cell, const, Polyhedron) \
LAMBDA_CLIENT_FUNC_IMPL(CLASS, cell, , Polyhedron) \
LAMBDA_CLIENT_FUNC_EDGE_IMPL(CLASS, edge, const) \
LAMBDA_CLIENT_FUNC_EDGE_IMPL(CLASS, edge, ) 

