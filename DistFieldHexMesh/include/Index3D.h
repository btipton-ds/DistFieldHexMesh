#pragma once

#include <tm_vector3.h>

namespace DFHM {

using Index3DBaseType = unsigned short; // This is large enough for 65536 x 65536 x 65536 block

class Index3D : public Vector3<Index3DBaseType>
{
public:
	enum Dir {
		Positive = 1,
		Negetive = -1
	};

	Index3D() = default;
	Index3D(const Index3D& src) = default;
	Index3D(const Vector3<Index3DBaseType>& src);
	Index3D(size_t i, size_t j, size_t k);
	Index3D(const Vector3i& src);

	Index3D operator + (const Index3D& rhs) const;
	bool operator < (const Index3D& rhs) const;
	bool isInBounds(const Index3D& bounds) const;

	template<class BOUND_TYPE>
	inline bool isInBounds(BOUND_TYPE bound) const
	{
		Index3DBaseType tBound = (Index3DBaseType)bound;
		return (*this)[0] < tBound && (*this)[1] < tBound && (*this)[2] < tBound;
	}
};

inline Index3D::Index3D(const Vector3<Index3DBaseType>& src)
	: Vector3<Index3DBaseType>(src)
{
}

inline Index3D::Index3D(size_t i, size_t j, size_t k)
	: Vector3<Index3DBaseType>((Index3DBaseType)i, (Index3DBaseType)j, (Index3DBaseType)k)
{
}

inline Index3D::Index3D(const Vector3i& src)
	: Vector3<Index3DBaseType>((Index3DBaseType)src[0], (Index3DBaseType)src[1], (Index3DBaseType)src[2])
{
}

inline Index3D Index3D::operator + (const Index3D& rhs) const
{
	Vector3<Index3DBaseType> temp(*this);
	temp += rhs;
	return temp;
}

inline bool Index3D::isInBounds(const Index3D& bounds) const
{
	return (*this)[0] < bounds[0] && (*this)[1] < bounds[1] && (*this)[2] < bounds[2];
}

inline bool Index3D::operator < (const Index3D& rhs) const
{
	for (size_t i = 0; i < 3; i++) {
		if ((*this)[i] < rhs[i])
			return true;
		else if ((*this)[i] > rhs[i])
			return false;
	}
	return false;
}

}