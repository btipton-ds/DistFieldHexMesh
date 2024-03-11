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

#define LAMBDA_FUNC_IMPL(NAME, MEMBER_NAME, CONST) \
template<class LAMBDA> \
void Block::NAME##Func(const Index3DId& id, LAMBDA func) CONST \
{ \
	func(getOwner(id)->MEMBER_NAME[id]); \
}

#define LAMBDA_FUNC_PAIR_DECL(NAME) \
LAMBDA_FUNC_DECL(NAME, const) \
LAMBDA_FUNC_DECL(NAME,) \
LAMBDA_FUNC_DECL(NAME##Const, const)

#define LAMBDA_FUNC_PAIR_IMPL(NAME, MEMBER_NAME) \
LAMBDA_FUNC_IMPL(NAME, MEMBER_NAME, const) \
LAMBDA_FUNC_IMPL(NAME, MEMBER_NAME, ) \
LAMBDA_FUNC_IMPL(NAME##Const, MEMBER_NAME, const) \

/****************************************************************************************************/
#define LAMBDA_CLIENT_FUNC_DECL(NAME, CONST) \
template<class LAMBDA> \
void NAME##Func(const Index3DId& id, LAMBDA func) CONST;

#define LAMBDA_CLIENT_FUNC_PAIR_DECL(NAME) \
LAMBDA_CLIENT_FUNC_DECL(NAME, const) \
LAMBDA_CLIENT_FUNC_DECL(NAME,) \
LAMBDA_CLIENT_FUNC_DECL(NAME##Const, const)

#define CLIENT_LAMBDA_FUNC_IMPL(CLASS, NAME, CONST) \
template<class LAMBDA> \
void CLASS::NAME##Func(const Index3DId& id, LAMBDA func) CONST \
{ \
	getBlockPtr()->NAME##Func(id, func); \
}

#define CLIENT_LAMBDA_FUNC_PAIR_IMPL(CLASS, NAME) \
CLIENT_LAMBDA_FUNC_IMPL(CLASS, NAME, const) \
CLIENT_LAMBDA_FUNC_IMPL(CLASS, NAME, ) \
CLIENT_LAMBDA_FUNC_IMPL(CLASS, NAME##Const, const)
