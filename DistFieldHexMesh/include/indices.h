#pragma once

#include <tm_vector3.h>

namespace DFHM {

class Index3D : public Vector3i
{
public:
	enum Dir {
		Positive = 1,
		Negetive = -1
	};

	Index3D() = default;
	Index3D(const Index3D& src) = default;
	Index3D(size_t i, size_t j, size_t k);
	Index3D(const Vector3i& src);

	Index3D operator + (const Index3D& rhs) const;
	bool operator < (const Index3D& rhs) const;

	Index3D adjX(Dir dir) const;
	Index3D adjY(Dir dir) const;
	Index3D adjZ(Dir dir) const;
};

inline Index3D::Index3D(size_t i, size_t j, size_t k)
	: Vector3i(i, j, k)
{
}

inline Index3D::Index3D(const Vector3i& src)
	: Vector3i(src)
{
}

inline Index3D Index3D::operator + (const Index3D& rhs) const
{
	Vector3i temp(*this);
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

class UniversalIndex3D {
public:
	UniversalIndex3D() = default;
	UniversalIndex3D(const UniversalIndex3D& src) = default;
	UniversalIndex3D(const Index3D& blockIdx, const Index3D& cellIdx = Index3D());

	const Index3D& blockIdx() const;
	const Index3D& cellIdx() const;

	bool operator < (const UniversalIndex3D& rhs) const;
private:
	Index3D _blockIdx, _cellIdx;
};

inline UniversalIndex3D::UniversalIndex3D(const Index3D& blockIdx, const Index3D& cellIdx)
	: _blockIdx(blockIdx)
	, _cellIdx(cellIdx)
{
}

inline const Index3D& UniversalIndex3D::blockIdx() const
{
	return _blockIdx;
}

inline const Index3D& UniversalIndex3D::cellIdx() const
{
	return _cellIdx;
}

inline bool UniversalIndex3D::operator < (const UniversalIndex3D& rhs) const
{
	if (_blockIdx < rhs._blockIdx)
		return true;
	else if (rhs._blockIdx < _blockIdx)
		return false;
}

}