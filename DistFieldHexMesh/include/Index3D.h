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

	Copyright Robert R Tipton, 2022, all rights reserved except those granted in prior license statement.

	Dark Sky Innovative Solutions http://darkskyinnovation.com/
*/

#include <tm_vector3.h>
#include <enums.h>

namespace DFHM {

class Volume;

using Index3DBaseType = uint8_t; // This is large enough for 256 x 256 x 256 block
using Index3DLongType = uint32_t;

// Base class with protected constructors prevents accidental swapping of Index3D and Index3DId
class Index3DBase
{
public:
	bool isUserFlagSet(Index3DBaseType bit) const;
	void setUserFlag(Index3DBaseType bit, bool val) const;

	static void setBlockDim(Index3DBaseType val);
	static Index3DBaseType getBlockDim();

	bool operator < (const Index3DBase& rhs) const;

	template<class T>
	bool operator == (const T& rhs) const;
	template<class T>
	bool operator != (const T& rhs) const;

	Index3DBase operator + (const Index3DBase& rhs) const;
	Index3DBase& operator += (const Index3DBase& rhs);
	Index3DBase operator - (const Index3DBase& rhs) const;
	Index3DBase& operator -= (const Index3DBase& rhs);

	const Index3DBaseType& operator[](int idx) const;
	Index3DBaseType& operator[](int idx);

	bool isValid() const;
	bool isInBounds(const Index3DBase& threadIdx, const Index3DBase& blockIdx, int range);
	bool isInBounds(size_t bound) const;
	void clampInBounds(size_t bound);

	// This checks if each entry is within +/- range of the this index. Used to verify thread safety between blocks.
	bool withinRange(const Index3DBase& blockIdx, int range = 1) const;

	size_t getLinearIdx(const Volume* vol) const;

	template<class BOUND_TYPE>
	bool isInBounds(const Vector3<BOUND_TYPE>& bounds) const;
	template<class BOUND_TYPE>
	void clampInBounds(const Vector3<BOUND_TYPE>& bounds);

	bool isInBounds(const Index3DBase& bounds) const;
	void clampInBounds(const Index3DBase& bounds);

	void write(std::ostream& out) const;
	void read(std::istream& in);

protected:
	Index3DBase();
	Index3DBase(const Index3DBase& src) = default;
	Index3DBase(const Vector3<Index3DBaseType>& src);
	Index3DBase(size_t i, size_t j, size_t k);
	Index3DBase(const Vector3i& src);

private:
	static Index3DBaseType s_blockDim;
	union {
		struct {
			Index3DBaseType _vals[3];
		};
		Index3DLongType _iVal = 0;
	};
	mutable Index3DBaseType _flags;
};

inline void Index3DBase::setBlockDim(Index3DBaseType val)
{
	s_blockDim = val;
}

inline Index3DBaseType Index3DBase::getBlockDim()
{
	return s_blockDim;
}

inline Index3DBase::Index3DBase(size_t i, size_t j, size_t k)
{
	_vals[0] = (Index3DBaseType)i;
	_vals[1] = (Index3DBaseType)j;
	_vals[2] = (Index3DBaseType)k;
	_flags = 0;
}

inline Index3DBase::Index3DBase(const Vector3i& src)
{
	_vals[0] = (Index3DBaseType)src[0];
	_vals[1] = (Index3DBaseType)src[1];
	_vals[2] = (Index3DBaseType)src[2];
	_flags = 0;
}

inline bool Index3DBase::isValid() const
{
	return _vals[0] != 0xff && _vals[1] != 0xff && _vals[2] != 0xff;
}

inline bool Index3DBase::isInBounds(size_t bound) const
{
	auto tBound = (Index3DBaseType)(bound);
	return _vals[0] < tBound && _vals[1] < tBound && _vals[2] < tBound;
}

template<class BOUND_TYPE>
inline bool Index3DBase::isInBounds(const Vector3<BOUND_TYPE>& bounds) const
{
	return 
		_vals[0] < (Index3DBaseType)bounds[0] && 
		_vals[1] < (Index3DBaseType)bounds[1] && 
		_vals[2] < (Index3DBaseType)bounds[2];
}

inline bool Index3DBase::isInBounds(const Index3DBase& bounds) const
{
	return 
		_vals[0] < bounds._vals[0] && 
		_vals[1] < bounds._vals[1] && 
		_vals[2] < bounds._vals[2];
}

template<class BOUND_TYPE>
void Index3DBase::clampInBounds(const Vector3<BOUND_TYPE>& bounds)
{
	for (int i = 0; i < 3; i++) {
		if (_vals[i] >= bounds[i])
			_vals[i] = bounds[i] - 1;
	}
}

inline bool Index3DBase::operator < (const Index3DBase& rhs) const
{
	return _iVal < rhs._iVal;
}

inline const Index3DBaseType& Index3DBase::operator[](int idx) const
{
	return _vals[idx];
}

inline Index3DBaseType& Index3DBase::operator[](int idx)
{
	return _vals[idx];
}

class Index3D : public Index3DBase
{
public:
	Index3D() = default;
	Index3D(const Index3D& src) = default;
	Index3D(const Index3DBase& src);
	Index3D(size_t i, size_t j, size_t k);
	Index3D(const Vector3i& src);
};

inline Index3D::Index3D(const Index3DBase& src)
	: Index3DBase(src)
{
}

inline Index3D::Index3D(size_t i, size_t j, size_t k)
	: Index3DBase(i, j, k)
{
}

inline Index3D::Index3D(const Vector3i& src)
	: Index3DBase(src)
{
}

inline bool Index3DBase::withinRange(const Index3DBase& blockIdx, int range) const
{
#if VALIDATION_ON
	for (int i = 0; i < 3; i++) {
		int v0 = _vals[i];
		int v1 = blockIdx[i];
		if (abs(v1 - v0) > range)
			return false;
	}
#endif

	return true;
}

template<class T>
inline bool Index3DBase::operator == (const T& rhs) const
{
	return _iVal == rhs._iVal;
}

template<class T>
inline bool Index3DBase::operator != (const T& rhs) const
{
	return !operator == (rhs);
}

class Index3DId : public Index3DBase
{
public:
	Index3DId() = default;
	Index3DId(Index3DBaseType i, Index3DBaseType j, Index3DBaseType k, size_t id);
	Index3DId(const Index3DId& src) = default;
	Index3DId(const Index3DBase& src, size_t elementId);

	bool operator < (const Index3DId& rhs) const;
	bool operator == (const Index3DId& rhs) const;
	bool operator != (const Index3DId& rhs) const;

	bool isValid() const;
	size_t elementId() const;
	Index3D blockIdx() const;

	void write(std::ostream& out) const;
	void read(std::istream& in);

private:
	size_t _elementId = -1;
};

inline Index3DId::Index3DId(const Index3DBase& src, size_t elementId)
	: Index3DBase(src)
	, _elementId(elementId)
{
}

inline Index3DId::Index3DId(Index3DBaseType i, Index3DBaseType j, Index3DBaseType k, size_t id)
	: Index3DBase(i, j, k)
	, _elementId(id)
{
}

inline bool Index3DId::operator < (const Index3DId& rhs) const
{
	if (Index3DBase::operator < (rhs))
		return true;
	else if (rhs.Index3DBase::operator<(*this))
		return false;

	return _elementId < rhs._elementId;
}

inline bool Index3DId::operator == (const Index3DId& rhs) const
{
	return (_elementId == rhs._elementId) && Index3DBase::operator ==(rhs);
}

inline bool Index3DId::operator != (const Index3DId& rhs) const
{
	return !Index3DId::operator ==(rhs);
}

inline bool Index3DId::isValid() const
{
	return (_elementId != -1) && Index3DBase::isValid();
}

inline size_t Index3DId::elementId() const
{
	return _elementId;
}

inline Index3D Index3DId::blockIdx() const
{
	return Index3D(*this);
}

std::ostream& operator << (std::ostream& out, const Index3DId& id);
std::ostream& operator << (std::ostream& out, const Index3D& idx);

}