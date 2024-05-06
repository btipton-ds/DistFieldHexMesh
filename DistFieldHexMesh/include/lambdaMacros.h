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

#define LAMBDA_REAL_FUNC_DECL(NAME, CONST, CLASS) \
void NAME##RealFunc(const Index3DId& id, const std::function<void(CONST CLASS& obj)>& func) CONST;

#define LAMBDA_CLIENT_REAL_FUNC_DECL(NAME, CONST, CLASS) \
void NAME##RealFunc(const Index3DId& id, const std::function<void(CONST CLASS& obj)>& func) CONST;

#define LAMBDA_REAL_FUNC_IMPL_0(NAME, MEMBER_NAME, CONST, CLASS) \
void Block::NAME##RealFunc(const Index3DId& id, const function<void(CONST CLASS& obj)>& func) CONST \
{ \
	auto p = getOwner(id); \
	func(p->MEMBER_NAME[id]); \
}

#define LAMBDA_REAL_FUNC_IMPL_1(NAME, MEMBER_NAME, CONST, CLASS) \
void Block::NAME##RealFunc(const Index3DId& id, const function<void(CONST CLASS& obj)>& func) CONST \
{ \
	auto p = getOwner(id); \
	func(p->_modelData.MEMBER_NAME[id]); \
}

#define LAMBDA_CLIENT_REAL_FUNC_IMPL(CLASS, NAME, MEMBER_NAME, CONST, CLASS2) \
void CLASS::NAME##RealFunc(const Index3DId& id, const std::function<void(CONST CLASS2& obj)>& func) CONST \
{ \
	auto p = getBlockPtr(); \
	p->NAME##RealFunc(id, func); \
}

#define LAMBDA_REF_FUNC_DECL(NAME, CLASS) \
void NAME##RefFunc(const Index3DId& id, const std::function<void(const CLASS& obj)>& func) const;

#define LAMBDA_CLIENT_REF_FUNC_DECL(NAME, CLASS) \
void NAME##RefFunc(const Index3DId& id, const std::function<void(const CLASS& obj)>& func) const;

#define LAMBDA_REF_FUNC_IMPL(NAME, MEMBER_NAME, CLASS) \
void Block::NAME##RefFunc(const Index3DId& id, const function<void(const CLASS& obj)>& func) const \
{ \
	auto p = getOwner(id); \
	func(p->_refData.MEMBER_NAME[id]); \
}

#define LAMBDA_CLIENT_REF_FUNC_IMPL(CLASS, NAME, MEMBER_NAME, CLASS2) \
void CLASS::NAME##RefFunc(const Index3DId& id, const std::function<void(const CLASS2& obj)>& func) const \
{ \
	auto p = getBlockPtr(); \
	p->NAME##RefFunc(id, func); \
}

#define LAMBDA_AVAIL_FUNC_DECL(NAME, CLASS) \
void NAME##AvailFunc(const Index3DId& id, TopolgyState prefState, const std::function<void(const CLASS& obj)>& func) const;

#define LAMBDA_CLIENT_AVAIL_FUNC_DECL(NAME, CLASS) \
void NAME##AvailFunc(const Index3DId& id, TopolgyState prefState, const std::function<void(const CLASS& obj)>& func) const;

#define LAMBDA_AVAIL_FUNC_IMPL(NAME, MEMBER_NAME, CLASS) \
void Block::NAME##AvailFunc(const Index3DId& id, TopolgyState prefState, const function<void(const CLASS& obj)>& func) const \
{ \
	const auto p = getOwner(id); \
	if (prefState == TS_REAL) {\
		if (p->_modelData.MEMBER_NAME.exists(id)) {\
			func(p->_modelData.MEMBER_NAME[id]); \
		} else { \
			func(p->_refData.MEMBER_NAME[id]); \
		} \
	} else { \
		if (p->_refData.MEMBER_NAME.exists(id)) {\
			func(p->_refData.MEMBER_NAME[id]); \
		} else {\
			func(p->_modelData.MEMBER_NAME[id]); \
		} \
	} \
}

#define LAMBDA_CLIENT_AVAIL_FUNC_IMPL(CLASS, NAME, MEMBER_NAME, CLASS2) \
void CLASS::NAME##AvailFunc(const Index3DId& id, TopolgyState prefState, const std::function<void(const CLASS2& obj)>& func) const \
{ \
	auto p = getBlockPtr(); \
	p->NAME##AvailFunc(id, prefState, func); \
}

/****************************************************************************************************/

#define LAMBDA_BLOCK_DECLS \
LAMBDA_REAL_FUNC_DECL(vertex, const, Vertex) \
LAMBDA_REAL_FUNC_DECL(vertex, , Vertex) \
LAMBDA_REAL_FUNC_DECL(face, const, Polygon) \
LAMBDA_REAL_FUNC_DECL(face, , Polygon) \
LAMBDA_REAL_FUNC_DECL(cell, const, Polyhedron) \
LAMBDA_REAL_FUNC_DECL(cell, , Polyhedron) \
LAMBDA_REF_FUNC_DECL(face, Polygon) \
LAMBDA_REF_FUNC_DECL(cell, Polyhedron) \
LAMBDA_AVAIL_FUNC_DECL(face, Polygon) \
LAMBDA_AVAIL_FUNC_DECL(cell, Polyhedron) \

#define LAMBDA_CLIENT_DECLS \
LAMBDA_CLIENT_REAL_FUNC_DECL(vertex, const, Vertex) \
LAMBDA_CLIENT_REAL_FUNC_DECL(vertex, , Vertex) \
LAMBDA_CLIENT_REAL_FUNC_DECL(face, const, Polygon) \
LAMBDA_CLIENT_REAL_FUNC_DECL(face, , Polygon) \
LAMBDA_CLIENT_REAL_FUNC_DECL(cell, const, Polyhedron) \
LAMBDA_CLIENT_REAL_FUNC_DECL(cell, , Polyhedron) \
LAMBDA_CLIENT_REF_FUNC_DECL(face, Polygon) \
LAMBDA_CLIENT_REF_FUNC_DECL(cell, Polyhedron) \
LAMBDA_CLIENT_AVAIL_FUNC_DECL(face, Polygon) \
LAMBDA_CLIENT_AVAIL_FUNC_DECL(cell, Polyhedron) \

#define LAMBDA_BLOCK_IMPLS \
LAMBDA_REAL_FUNC_IMPL_0(vertex, _vertices, const, Vertex) \
LAMBDA_REAL_FUNC_IMPL_0(vertex, _vertices, , Vertex) \
LAMBDA_REAL_FUNC_IMPL_1(face, _polygons, const, Polygon) \
LAMBDA_REAL_FUNC_IMPL_1(face, _polygons, , Polygon) \
LAMBDA_REAL_FUNC_IMPL_1(cell, _polyhedra, const, Polyhedron) \
LAMBDA_REAL_FUNC_IMPL_1(cell, _polyhedra, , Polyhedron) \
LAMBDA_REF_FUNC_IMPL(face, _polygons, Polygon) \
LAMBDA_REF_FUNC_IMPL(cell, _polyhedra, Polyhedron) \
LAMBDA_AVAIL_FUNC_IMPL(face, _polygons, Polygon) \
LAMBDA_AVAIL_FUNC_IMPL(cell, _polyhedra, Polyhedron) \

#define LAMBDA_CLIENT_IMPLS(CLASS) \
LAMBDA_CLIENT_REAL_FUNC_IMPL(CLASS, vertex, _vertices, const, Vertex) \
LAMBDA_CLIENT_REAL_FUNC_IMPL(CLASS, vertex, _vertices, , Vertex) \
LAMBDA_CLIENT_REAL_FUNC_IMPL(CLASS, face, _polygons, const, Polygon) \
LAMBDA_CLIENT_REAL_FUNC_IMPL(CLASS, face, _polygons, , Polygon) \
LAMBDA_CLIENT_REAL_FUNC_IMPL(CLASS, cell, _polyhedra, const, Polyhedron) \
LAMBDA_CLIENT_REAL_FUNC_IMPL(CLASS, cell, _polyhedra, , Polyhedron) \
LAMBDA_CLIENT_REF_FUNC_IMPL(CLASS, face, _polygons, Polygon) \
LAMBDA_CLIENT_REF_FUNC_IMPL(CLASS, cell, _polyhedra, Polyhedron) \
LAMBDA_CLIENT_AVAIL_FUNC_IMPL(CLASS, face, _polygons, Polygon) \
LAMBDA_CLIENT_AVAIL_FUNC_IMPL(CLASS, cell, _polyhedra, Polyhedron) \

