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

#include <tm_vector3.h>
#include <enums.h>

namespace DFHM {

class Volume;

using Index3DBaseType = unsigned short; // This is large enough for 65536 x 65536 x 65536 block

// Base class with protected constructors prevents accidental swapping of Index3D and Index3DId
class Index3DBase : public Vector3<Index3DBaseType>
{
protected:
	Index3DBase() = default;
	Index3DBase(const Index3DBase& src) = default;
	Index3DBase(const Vector3<Index3DBaseType>& src);
	Index3DBase(size_t i, size_t j, size_t k);
	Index3DBase(const Vector3i& src);

public:
	static void setBlockDim(size_t val);
	static size_t getBlockDim();

	Index3DBase operator + (const Index3DBase& rhs) const;
	bool operator < (const Index3DBase& rhs) const;

	template<class T>
	bool operator == (const T& rhs) const;
	template<class T>
	bool operator != (const T& rhs) const;

	bool isValid() const;
	bool isInBounds(size_t bound) const;
	void clampInBounds(size_t bound);

	size_t getLinearIdx(const Volume* vol) const;

	template<class BOUND_TYPE>
	bool isInBounds(const Vector3<BOUND_TYPE>& bounds) const;
	template<class BOUND_TYPE>
	void clampInBounds(const Vector3<BOUND_TYPE>& bounds);

private:
	static Index3DBaseType s_blockDim;
	mutable size_t _linearIdx = -1;
};

inline void Index3DBase::setBlockDim(size_t val)
{
	s_blockDim = (Index3DBaseType)val;
}

inline size_t Index3DBase::getBlockDim()
{
	return s_blockDim;
}

inline Index3DBase::Index3DBase(const Vector3<Index3DBaseType>& src)
	: Vector3<Index3DBaseType>(src)
{
}

inline Index3DBase::Index3DBase(size_t i, size_t j, size_t k)
	: Vector3<Index3DBaseType>((Index3DBaseType)i, (Index3DBaseType)j, (Index3DBaseType)k)
{
}

inline Index3DBase::Index3DBase(const Vector3i& src)
	: Vector3<Index3DBaseType>((Index3DBaseType)src[0], (Index3DBaseType)src[1], (Index3DBaseType)src[2])
{
}

inline Index3DBase Index3DBase::operator + (const Index3DBase& rhs) const
{
	Vector3<Index3DBaseType> temp(*this);
	temp += rhs;
	return temp;
}

inline bool Index3DBase::isValid() const
{
	const Index3DBaseType t = 0xffff;
	return (*this)[0] != t && (*this)[1] != t && (*this)[1] != t;
}

inline bool Index3DBase::isInBounds(size_t bound) const
{
	auto tBound = (Index3DBaseType)(bound);
	return (*this)[0] < tBound && (*this)[1] < tBound && (*this)[2] < tBound;
}

template<class BOUND_TYPE>
inline bool Index3DBase::isInBounds(const Vector3<BOUND_TYPE>& bounds) const
{
	return (*this)[0] < (Index3DBaseType)bounds[0] && (*this)[1] < (Index3DBaseType)bounds[1] && (*this)[2] < (Index3DBaseType)bounds[2];
}

template<class BOUND_TYPE>
void Index3DBase::clampInBounds(const Vector3<BOUND_TYPE>& bounds)
{
	for (int i = 0; i < 3; i++) {
		if ((*this)[i] >= bounds[i])
			(*this)[i] = bounds[i] - 1;
	}
}

class Index3D : public Index3DBase
{
public:
	Index3D() = default;
	Index3D(const Index3D& src) = default;
	Index3D(const Vector3<Index3DBaseType>& src);
	Index3D(size_t i, size_t j, size_t k);
	Index3D(const Vector3i& src);
};

inline Index3D::Index3D(const Vector3<Index3DBaseType>& src)
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

template<class T>
inline bool Index3DBase::operator == (const T& rhs) const
{
	return Vector3<Index3DBaseType>::operator ==(rhs);
}

template<class T>
inline bool Index3DBase::operator != (const T& rhs) const
{
	return !operator==(rhs);
}

class Index3DId : public Index3DBase
{
public:
	static void setBlockDim(size_t val);
	static size_t getBlockDim();

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

private:
	size_t _elementId = -1;
};

inline Index3DId::Index3DId(const Index3DBase& src, size_t elementId)
	: Index3DBase(src)
	, _elementId(elementId)
{
}

inline bool Index3DId::operator < (const Index3DId& rhs) const
{
	if (_elementId < rhs._elementId)
		return true;
	else if (_elementId > rhs._elementId)
		return false;
	
	return Index3DBase::operator <(rhs);
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