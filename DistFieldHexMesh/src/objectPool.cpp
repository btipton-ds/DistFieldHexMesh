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

#include <string>
#include <sstream>
#include <objectPool.h>
#include <vertex.h>
#include <polygon.h>
#include <polyhedron.h>
#include <block.h>
#include <volume.h>
#include <logger.h>

using namespace std;
using namespace DFHM;

#define DECL_THREAD_LOCAL(CLASS_NAME)  template<>\
thread_local const CLASS_NAME* ObjectPool<CLASS_NAME>::_tl_pCompareObj = nullptr

DECL_THREAD_LOCAL(Vertex);
DECL_THREAD_LOCAL(Polygon);
DECL_THREAD_LOCAL(Polyhedron);

ObjectPoolOwnerUser::ObjectPoolOwnerUser(const ObjectPoolOwnerUser& src)
	: _pPoolOwner(src._pPoolOwner)
	, _thisId(src._thisId)
{
}

ObjectPoolOwnerUser::ObjectPoolOwnerUser(const ObjectPoolOwner* poolOwner, size_t id)
	: _pPoolOwner(const_cast<ObjectPoolOwner*> (poolOwner))
	, _thisId(poolOwner->getBlockIdx(), id)
{
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

const Index3DId& ObjectPoolOwnerUser::getId() const
{
	return _thisId;
}

shared_ptr<Logger> ObjectPoolOwner::getLogger() const
{
	if (_filename.empty()) {
		Index3D idx = getBlockIdx();
		stringstream ss;
		ss << "block_" << getLoggerNumericCode() << ".log";
		_filename = ss.str();
	}

	return Logger::get(_filename);
}

string ObjectPoolOwner::getLoggerNumericCode() const
{
	Index3D idx = getBlockIdx();
	stringstream ss;
	ss << idx[0] << "_" << idx[1] << "_" << idx[2];
	return ss.str();
}
