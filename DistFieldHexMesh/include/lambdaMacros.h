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

#define LAMBDA_FUNC_DECL(NAME, CONST) \
template<class LAMBDA> \
void NAME##Func(const Index3DId& id, LAMBDA func) CONST;

#define LAMBDA_FUNC_REF_DECL(NAME, CONST) \
template<class LAMBDA> \
void NAME##RefFunc(const Index3DId& id, LAMBDA func) CONST;

#define LAMBDA_FUNC_IMPL_0(NAME, MEMBER_NAME, CONST) \
template<class LAMBDA> \
void Block::NAME##Func(const Index3DId& id, LAMBDA func) CONST \
{ \
	func(getOwner(id)->MEMBER_NAME[id]); \
}

#define LAMBDA_FUNC_IMPL_1(NAME, MEMBER_NAME, CONST) \
template<class LAMBDA> \
void Block::NAME##Func(const Index3DId& id, LAMBDA func) CONST \
{ \
	CONST auto p = getOwner(id); \
	if (p->_modelData.MEMBER_NAME.exists(id)) \
		func(p->_modelData.MEMBER_NAME[id]); \
	else \
		func(p->_refData.MEMBER_NAME[id]); \
}

#define LAMBDA_FUNC_REF_IMPL(NAME, MEMBER_NAME, CONST) \
template<class LAMBDA> \
void Block::NAME##RefFunc(const Index3DId& id, LAMBDA func) CONST \
{ \
	func(getOwner(id)->_refData.MEMBER_NAME[id]); \
}

#define LAMBDA_FUNC_SET_DECL(NAME) \
LAMBDA_FUNC_DECL(NAME, const) \
LAMBDA_FUNC_DECL(NAME,)

#define LAMBDA_FUNC_SET_REF_DECL(NAME) \
LAMBDA_FUNC_DECL(NAME, const) \
LAMBDA_FUNC_DECL(NAME,) \
LAMBDA_FUNC_REF_DECL(NAME, const) \
LAMBDA_FUNC_REF_DECL(NAME,)

#define LAMBDA_FUNC_SET_IMPL(NAME, MEMBER_NAME) \
LAMBDA_FUNC_IMPL_0(NAME, MEMBER_NAME, const) \
LAMBDA_FUNC_IMPL_0(NAME, MEMBER_NAME, )

#define LAMBDA_FUNC_SET_REF_IMPL(NAME, MEMBER_NAME) \
LAMBDA_FUNC_IMPL_1(NAME, MEMBER_NAME, const) \
LAMBDA_FUNC_IMPL_1(NAME, MEMBER_NAME, ) \
LAMBDA_FUNC_REF_IMPL(NAME, MEMBER_NAME, const) \
LAMBDA_FUNC_REF_IMPL(NAME, MEMBER_NAME, )

/****************************************************************************************************/
#define LAMBDA_CLIENT_FUNC_DECL(NAME, CONST) \
template<class LAMBDA> \
void NAME##Func(const Index3DId& id, LAMBDA func) CONST;

#define LAMBDA_CLIENT_FUNC_SET_DECL(NAME) \
LAMBDA_CLIENT_FUNC_DECL(NAME, const) \
LAMBDA_CLIENT_FUNC_DECL(NAME,)

#define LAMBDA_CLIENT_FUNC_SET_REF_DECL(NAME) \
LAMBDA_CLIENT_FUNC_DECL(NAME, const) \
LAMBDA_CLIENT_FUNC_DECL(NAME,)

#define LAMBDA_CLIENT_FUNC_IMPL(CLASS, NAME, CONST) \
template<class LAMBDA> \
void CLASS::NAME##Func(const Index3DId& id, LAMBDA func) CONST \
{ \
	getBlockPtr()->NAME##Func(id, func); \
}

#define LAMBDA_CLIENT_FUNC_SET_IMPL(CLASS, NAME) \
LAMBDA_CLIENT_FUNC_IMPL(CLASS, NAME, const) \
LAMBDA_CLIENT_FUNC_IMPL(CLASS, NAME, )

#define LAMBDA_CLIENT_FUNC_SET_REF_IMPL(CLASS, NAME) \
LAMBDA_CLIENT_FUNC_IMPL(CLASS, NAME, const) \
LAMBDA_CLIENT_FUNC_IMPL(CLASS, NAME, )


#define LAMBDA_BLOCK_DECLS \
LAMBDA_FUNC_SET_DECL(vertex); \
LAMBDA_FUNC_SET_REF_DECL(face); \
LAMBDA_FUNC_SET_REF_DECL(cell);

#define LAMBDA_BLOCK_IMPLS \
LAMBDA_FUNC_SET_IMPL(vertex, _vertices) \
LAMBDA_FUNC_SET_REF_IMPL(face, _polygons) \
LAMBDA_FUNC_SET_REF_IMPL(cell, _polyhedra)

#define LAMBDA_CLIENT_DECLS \
LAMBDA_CLIENT_FUNC_SET_DECL(vertex); \
LAMBDA_CLIENT_FUNC_SET_REF_DECL(face); \
LAMBDA_CLIENT_FUNC_SET_REF_DECL(cell);

#define LAMBDA_CLIENT_IMPLS(CLASS) \
LAMBDA_CLIENT_FUNC_SET_IMPL(CLASS, vertex) \
LAMBDA_CLIENT_FUNC_SET_REF_IMPL(CLASS, face) \
LAMBDA_CLIENT_FUNC_SET_REF_IMPL(CLASS, cell) \
