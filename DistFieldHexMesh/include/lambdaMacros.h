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
void NAME##Func(const Index3DId& id, std::function<void(CONST CLASS& obj)> func) CONST;


#define LAMBDA_FUNC_REF_DECL(NAME, CLASS) \
void NAME##RefFunc(const Index3DId& id, std::function<void(const CLASS& obj)> func) const;

#define LAMBDA_FUNC_IMPL_0(NAME, MEMBER_NAME, CONST, CLASS) \
void Block::NAME##Func(const Index3DId& id, function<void(CONST CLASS& obj)> func) CONST \
{ \
	auto p = getOwner(id); \
	func(p->MEMBER_NAME[id]); \
}

#define LAMBDA_FUNC_IMPL_1(NAME, MEMBER_NAME, CLASS) \
void Block::NAME##Func(const Index3DId& id, function<void(CLASS& obj)> func) \
{ \
	auto p = getOwner(id); \
	func(p->_modelData.MEMBER_NAME[id]); \
} \
void Block::NAME##Func(const Index3DId& id, function<void(const CLASS& obj)> func) const \
{ \
	const auto p = getOwner(id); \
	if (p->_refData.MEMBER_NAME.exists(id)) \
		func(p->_refData.MEMBER_NAME[id]); \
	else \
		func(p->_modelData.MEMBER_NAME[id]); \
}

#define LAMBDA_FUNC_REF_IMPL(NAME, MEMBER_NAME, CLASS) \
void Block::NAME##RefFunc(const Index3DId& id, function<void(const CLASS& obj)> func) const \
{ \
	const auto p = getOwner(id); \
	if (p->_refData.MEMBER_NAME.exists(id)) \
		func(p->_refData.MEMBER_NAME[id]); \
}

#define LAMBDA_FUNC_SET_DECL(NAME, CLASS) \
LAMBDA_FUNC_DECL(NAME, const, CLASS) \
LAMBDA_FUNC_DECL(NAME,, CLASS)

#define LAMBDA_FUNC_SET_REF_DECL(NAME, CLASS) \
LAMBDA_FUNC_DECL(NAME, const, CLASS) \
LAMBDA_FUNC_DECL(NAME,, CLASS) \
LAMBDA_FUNC_REF_DECL(NAME, CLASS)

#define LAMBDA_FUNC_SET_IMPL(NAME, MEMBER_NAME, CLASS) \
LAMBDA_FUNC_IMPL_0(NAME, MEMBER_NAME, const, CLASS) \
LAMBDA_FUNC_IMPL_0(NAME, MEMBER_NAME, , CLASS)

#define LAMBDA_FUNC_SET_REF_IMPL(NAME, MEMBER_NAME, CLASS) \
LAMBDA_FUNC_IMPL_1(NAME, MEMBER_NAME, CLASS) \
LAMBDA_FUNC_REF_IMPL(NAME, MEMBER_NAME, CLASS)

/****************************************************************************************************/
#define LAMBDA_CLIENT_FUNC_DECL(NAME, CONST, CLASS) \
void NAME##Func(const Index3DId& id, std::function<void(CONST CLASS& obj)> func) CONST;

#define LAMBDA_CLIENT_FUNC_REF_DECL(NAME, CLASS) \
void NAME##RefFunc(const Index3DId& id, std::function<void(const CLASS& obj)> func) const;

#define LAMBDA_CLIENT_FUNC_SET_DECL(NAME, CLASS) \
LAMBDA_CLIENT_FUNC_DECL(NAME, const, CLASS) \
LAMBDA_CLIENT_FUNC_DECL(NAME,, CLASS)

#define LAMBDA_CLIENT_FUNC_SET_REF_DECL(NAME, CLASS) \
LAMBDA_CLIENT_FUNC_DECL(NAME, const, CLASS) \
LAMBDA_CLIENT_FUNC_DECL(NAME,, CLASS) \
LAMBDA_CLIENT_FUNC_REF_DECL(NAME, CLASS)

#define LAMBDA_CLIENT_FUNC_IMPL(CLASS, NAME, CONST, CLASS2) \
void CLASS::NAME##Func(const Index3DId& id, std::function<void(CONST CLASS2& obj)> func) CONST \
{ \
	getBlockPtr()->NAME##Func(id, func); \
}

#define LAMBDA_CLIENT_FUNC_REF_IMPL(CLASS, NAME, CLASS2) \
void CLASS::NAME##RefFunc(const Index3DId& id, std::function<void(const CLASS2& obj)> func) const \
{ \
	getBlockPtr()->NAME##RefFunc(id, func); \
}

#define LAMBDA_CLIENT_FUNC_SET_IMPL(CLASS, NAME, CLASS2) \
LAMBDA_CLIENT_FUNC_IMPL(CLASS, NAME, const, CLASS2) \
LAMBDA_CLIENT_FUNC_IMPL(CLASS, NAME, , CLASS2)

#define LAMBDA_CLIENT_FUNC_SET_REF_IMPL(CLASS, NAME, CLASS2) \
LAMBDA_CLIENT_FUNC_IMPL(CLASS, NAME, const, CLASS2) \
LAMBDA_CLIENT_FUNC_IMPL(CLASS, NAME, , CLASS2) \
LAMBDA_CLIENT_FUNC_REF_IMPL(CLASS, NAME, CLASS2)


#define LAMBDA_BLOCK_DECLS \
LAMBDA_FUNC_SET_DECL(vertex, Vertex) \
LAMBDA_FUNC_SET_REF_DECL(face, Polygon) \
LAMBDA_FUNC_SET_REF_DECL(cell, Polyhedron)

#define LAMBDA_BLOCK_IMPLS \
LAMBDA_FUNC_SET_IMPL(vertex, _vertices, Vertex) \
LAMBDA_FUNC_SET_REF_IMPL(face, _polygons, Polygon) \
LAMBDA_FUNC_SET_REF_IMPL(cell, _polyhedra, Polyhedron)

#define LAMBDA_CLIENT_DECLS \
LAMBDA_CLIENT_FUNC_SET_DECL(vertex, Vertex) \
LAMBDA_CLIENT_FUNC_SET_REF_DECL(face, Polygon) \
LAMBDA_CLIENT_FUNC_SET_REF_DECL(cell, Polyhedron)

#define LAMBDA_CLIENT_IMPLS(CLASS) \
LAMBDA_CLIENT_FUNC_SET_IMPL(CLASS, vertex, Vertex) \
LAMBDA_CLIENT_FUNC_SET_REF_IMPL(CLASS, face, Polygon) \
LAMBDA_CLIENT_FUNC_SET_REF_IMPL(CLASS, cell, Polyhedron) \
