
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
	auto pBlock =  dynamic_cast<const Block*>(_pPoolOwner);
	return pBlock->getOwner(_thisId);
}

Block* ObjectPoolOwnerUser::getBlockPtr()
{
	auto pOwner = dynamic_cast<Block*>(_pPoolOwner);
	if (pOwner)
		return pOwner->getVolume()->getBlockPtr(_thisId);
	return nullptr;
}

void ObjectPoolOwnerUser::setId(const ObjectPoolOwner* poolOwner, size_t id)
{
	_pPoolOwner = const_cast<ObjectPoolOwner*> (poolOwner);
	_thisId = Index3DId(poolOwner->getBlockIdx(), id);
}
