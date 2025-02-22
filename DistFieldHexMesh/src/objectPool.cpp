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
#include <pool_map.hpp>
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

ObjectPoolOwnerUser::~ObjectPoolOwnerUser()
{
	clear();
}

void ObjectPoolOwnerUser::clear()
{
}

ObjectPoolOwnerUser& ObjectPoolOwnerUser::operator = (const ObjectPoolOwnerUser& rhs)
{
	clear();
	_pPoolOwner = rhs._pPoolOwner;
	_thisId = rhs._thisId;

	return *this;
}

void ObjectPoolOwnerUser::setPoolOwner(ObjectPoolOwner* pPoolOwner)
{
	_pPoolOwner = pPoolOwner;
}

const Block* ObjectPoolOwnerUser::getOurBlockPtr() const
{
	return dynamic_cast<const Block*>(_pPoolOwner);
}

Block* ObjectPoolOwnerUser::getOurBlockPtr()
{
	return dynamic_cast<Block*>(_pPoolOwner);
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

const Block* ObjectPoolOwnerUser::getOwnerBlockPtr(const Index3D& idx) const
{
	auto p = getBlockPtr();
	if (p)
		return p->getOwner(idx);
	return nullptr;
}

Block* ObjectPoolOwnerUser::getOwnerBlockPtr(const Index3D& idx)
{
	auto p = getBlockPtr();
	if (p)
		return p->getOwner(idx);
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

void ObjectPoolOwnerUser::remapId(const std::vector<size_t>& idRemap, const Index3D& srcDims)
{
	auto pBlk = getOurBlockPtr();
	auto pVol = pBlk->getVolume();
	size_t srcIdx = Volume::calLinearBlockIndex(_thisId, srcDims);
	Index3D dstIdx;
	if (idRemap[srcIdx] != -1) {
		dstIdx = pVol->calBlockIndexFromLinearIndex(idRemap[srcIdx]);
		size_t elementId = _thisId.elementId();
		_thisId = Index3DId(dstIdx, elementId);
	}
}

bool ObjectPoolOwnerUser::verifyIndices(const Index3D& idx) const
{
	return _thisId.blockIdx() == idx;
}

void ObjectPoolOwnerUser::remap(const std::vector<size_t>& idRemap, const Index3D& srcDims, FastBisectionSet<Index3DId>& vals)
{
	FastBisectionSet<Index3DId> tmp(vals);
	vals.clear();

	Index3D dstIdx;
	auto pBlk = getOurBlockPtr();
	auto pVol = pBlk->getVolume();

	for (auto& v : tmp.asVector()) {
		size_t srcIdx = Volume::calLinearBlockIndex(_thisId, srcDims);
		if (idRemap[srcIdx] != -1) {
			dstIdx = pVol->calBlockIndexFromLinearIndex(idRemap[srcIdx]);
			vals.insert(Index3DId(dstIdx, v.elementId()));
		}
		else
			vals.insert(v);
	}
}

void ObjectPoolOwnerUser::remap(const std::vector<size_t>& idRemap, const Index3D& srcDims, MTC::set<Index3DId>& vals)
{
	MTC::set<Index3DId> tmp(vals);
	vals.clear();

	Index3D dstIdx;
	auto pBlk = getOurBlockPtr();
	auto pVol = pBlk->getVolume();

	for (auto& v : tmp) {
		size_t srcIdx = Volume::calLinearBlockIndex(_thisId, srcDims);
		if (idRemap[srcIdx] != -1) {
			dstIdx = pVol->calBlockIndexFromLinearIndex(idRemap[srcIdx]);
			vals.insert(Index3DId(dstIdx, v.elementId()));
		} else
			vals.insert(v);
	}
}

void ObjectPoolOwnerUser::remap(const std::vector<size_t>& idRemap, const Index3D& srcDims, MTC::vector<Index3DId>& vals)
{
	Index3D dstIdx;
	auto pBlk = getOurBlockPtr();
	auto pVol = pBlk->getVolume();

	for (size_t i = 0; i < vals.size(); i++) {
		const auto& v = vals[i];
		size_t srcIdx = Volume::calLinearBlockIndex(_thisId, srcDims);
		if (idRemap[srcIdx] != -1) {
			dstIdx = pVol->calBlockIndexFromLinearIndex(idRemap[srcIdx]);
			vals[i] = Index3DId(dstIdx, vals[i].elementId());
		}
	}
}

bool ObjectPoolOwnerUser::isOwnerBeingDestroyed() const
{
	return !_pPoolOwner || _pPoolOwner->_isBeingDestroyed;
}

namespace
{
	static thread_local Index3D s_blockIdx;
}

void ObjectPoolOwner::setThreadBlockIdx(const Index3D& blockIdx)
{
#if RUN_MULTI_THREAD
	s_blockIdx = blockIdx;
#endif
}

const Index3D& ObjectPoolOwner::getThreadBlockIdx() const
{
#if RUN_MULTI_THREAD
	return s_blockIdx;
#else
	return getBlockIdx();
#endif
}

shared_ptr<Logger> ObjectPoolOwner::getLogger() const
{
#if RUN_MULTI_THREAD
	stringstream ss;
	ss << "block_" << getLoggerNumericCode() << ".log";
	string filename = ss.str();

	return Logger::get(filename);
#else
	if (_filename.empty()) {
		stringstream ss;
		ss << "block_" << getLoggerNumericCode() << ".log";
		_filename = ss.str();
	}

	return Logger::get(_filename);
#endif
}

string ObjectPoolOwner::getLoggerNumericCode() const
{
	Index3D idx;
#if RUN_MULTI_THREAD
	idx = getBlockIdx();
#else
	idx = s_blockIdx;
#endif
	return getLoggerNumericCode(idx);
}

string ObjectPoolOwner::getLoggerNumericCode(const Index3D& idx)
{
	stringstream ss;
	ss << idx[0] << "_" << idx[1] << "_" << idx[2];
	return ss.str();
}

string ObjectPoolOwner::getLoggerNumericCode(const Index3DId& id)
{
	stringstream ss;
	ss << id[0] << "_" << id[1] << "_" << id[2] << "_" << id.elementId();
	return ss.str();
}
