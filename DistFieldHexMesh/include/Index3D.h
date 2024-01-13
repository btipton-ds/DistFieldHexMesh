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

	Index3D adjX(Dir dir) const;
	Index3D adjY(Dir dir) const;
	Index3D adjZ(Dir dir) const;
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

inline Index3D Index3D::adjX(Dir dir) const
{
	return *this + Index3D(dir, 0, 0);
}

inline Index3D Index3D::adjY(Dir dir) const
{
	return *this + Index3D(0, dir, 0);
}

inline Index3D Index3D::adjZ(Dir dir) const
{
	return *this + Index3D(0, 0, dir);
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