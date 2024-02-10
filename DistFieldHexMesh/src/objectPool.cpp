
#include <objectPool.h>
#include <vertex.h>
#include <polygon.h>
#include <polyhedron.h>
#include <block.h>
#include <volume.h>

using namespace DFHM;

#define DECL_THREAD_LOCAL(CLASS_NAME) thread_local const CLASS_NAME* ObjectPool<CLASS_NAME>::_tl_pCompareObj = nullptr

DECL_THREAD_LOCAL(Vertex);
DECL_THREAD_LOCAL(Polygon);
DECL_THREAD_LOCAL(Polyhedron);

ObjectPoolOwnerUser::ObjectPoolOwnerUser(const ObjectPoolOwnerUser& src)
{
	_pPoolOwner = src._pPoolOwner;
	_thisId = src._thisId;
}

ObjectPoolOwnerUser::ObjectPoolOwnerUser(const ObjectPoolOwner* poolOwner, size_t id)
{
	_pPoolOwner = const_cast<ObjectPoolOwner*> (poolOwner);
	_thisId = Index3DId(poolOwner->getBlockIdx(), id);
}

ObjectPoolOwnerUser& ObjectPoolOwnerUser::operator = (const ObjectPoolOwnerUser& rhs)
{
	_pPoolOwner = rhs._pPoolOwner;
	_thisId = rhs._thisId;

	return *this;
}

const Block* ObjectPoolOwnerUser::getBlockPtr() const
{
	return const_cast<const Block*>(dynamic_cast<Block*>(_pPoolOwner));
}

Block* ObjectPoolOwnerUser::getOutBlockPtr(const Index3DId& blockIdx) const
{
	auto pOwner = getBlockPtr();
	if (pOwner)
		return pOwner->getVolume()->getOutBlockPtr(blockIdx);
	return nullptr;
}

void ObjectPoolOwnerUser::setId(const ObjectPoolOwner* poolOwner, size_t id)
{
	_pPoolOwner = const_cast<ObjectPoolOwner*> (poolOwner);
	_thisId = Index3DId(poolOwner->getBlockIdx(), id);
}

MutexType& ObjectPoolOwnerUser::getMutex() const
{
	return _mutex;
}
