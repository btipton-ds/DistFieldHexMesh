#pragma once

#define LAMBDA_FUNC_DECL(NAME, CONST) \
template<class LAMBDA> \
void NAME##Func(const Index3DId& id, LAMBDA func) CONST;

#define LAMBDA_FUNC_IMPL(NAME, GET_OWNER_FUNC, MEMBER_NAME, CONST) \
template<class LAMBDA> \
void Block::NAME##Func(const Index3DId& id, LAMBDA func) CONST \
{ \
	auto pOwner = GET_OWNER_FUNC(id); \
	auto& obj = pOwner->MEMBER_NAME[id]; \
	func(obj); \
}

#define LAMBDA_FUNC_PAIR_DECL(NAME) \
LAMBDA_FUNC_DECL(NAME, const) \
LAMBDA_FUNC_DECL(NAME,) \
LAMBDA_FUNC_DECL(NAME##Out, const)

#define LAMBDA_FUNC_PAIR_IMPL(NAME, MEMBER_NAME) \
LAMBDA_FUNC_IMPL(NAME, getOwner, MEMBER_NAME, const) \
LAMBDA_FUNC_IMPL(NAME, getOwner, MEMBER_NAME, ) \
LAMBDA_FUNC_IMPL(NAME##Out, getOutBlockPtr, MEMBER_NAME, const)

#define CLIENT_LAMBDA_FUNC_IMPL(CLASS, NAME, CONST) \
template<class LAMBDA> \
void CLASS::NAME##Func(const Index3DId& id, LAMBDA func) CONST \
{ \
	getBlockPtr()->NAME##Func(id, func); \
}

#define CLIENT_LAMBDA_FUNC_PAIR_IMPL(CLASS, NAME) \
CLIENT_LAMBDA_FUNC_IMPL(CLASS, NAME, const) \
CLIENT_LAMBDA_FUNC_IMPL(CLASS, NAME, ) \
CLIENT_LAMBDA_FUNC_IMPL(CLASS, NAME##Out, const)
