#pragma once

#include <tm_vector3.h>

namespace DFHM {

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

	enum Dir {
		Positive = 1,
		Negetive = -1
	};

	Index3DBase operator + (const Index3DBase& rhs) const;
	bool operator < (const Index3DBase& rhs) const;

	template<class T>
	bool operator == (const T& rhs) const;
	template<class T>
	bool operator != (const T& rhs) const;

	bool isValid() const;
	bool isInBounds(size_t bound) const;

	template<class BOUND_TYPE>
	bool isInBounds(const Vector3<BOUND_TYPE>& bounds) const;

private:
	static Index3DBaseType s_blockDim;
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

	enum Dir {
		Positive = 1,
		Negetive = -1
	};

	Index3DId() = default;
	Index3DId(const Index3DId& src) = default;
	Index3DId(const Index3DBase& src, size_t elementId);
	Index3DId(const Vector3<Index3DBaseType>& src);
	Index3DId(size_t i, size_t j, size_t k);
	Index3DId(const Vector3i& src);

	bool operator < (const Index3DId& rhs) const;
	bool operator == (const Index3DId& rhs) const;
	bool operator != (const Index3DId& rhs) const;

	bool isValid() const;
	size_t elementId() const;

private:
	size_t _elementId = -1;
};

inline Index3DId::Index3DId(const Index3DBase& src, size_t elementId)
	: Index3DBase(src)
	, _elementId(elementId)
{
}

inline Index3DId::Index3DId(const Vector3<Index3DBaseType>& src)
	: Index3DBase(src)
{
}

inline Index3DId::Index3DId(size_t i, size_t j, size_t k)
	: Index3DBase(i, j, k)
{
}

inline Index3DId::Index3DId(const Vector3i& src)
	: Index3DBase(src)
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

}