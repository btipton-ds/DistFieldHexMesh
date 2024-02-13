#pragma once

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
