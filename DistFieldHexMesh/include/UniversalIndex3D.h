#pragma once

#include <Index3D.h>

namespace DFHM {

class UniversalIndex3D {
public:
	UniversalIndex3D() = default;
	UniversalIndex3D(const UniversalIndex3D& src) = default;
	UniversalIndex3D(const Vector3i& blockIdx, size_t cellId = -1);

	Vector3i blockIdx() const;
	size_t cellId() const;

	bool operator < (const UniversalIndex3D& rhs) const;
private:
	inline static bool isLess(const Vector3<unsigned int>& lhs, const Vector3<unsigned int>& rhs)
	{
		for (int i = 0; i < 3; i++) {
			if (lhs[i] < rhs[i])
				return true;
			else if (rhs[i] < lhs[i])
				return false;
		}
		return false;
	}

	Vector3<unsigned int> _blockIdx;
	size_t _cellId;
};

inline UniversalIndex3D::UniversalIndex3D(const Vector3i& blockIdx, size_t cellId)
	: _blockIdx((unsigned int)blockIdx[0], (unsigned int)blockIdx[1], (unsigned int)blockIdx[2])
	, _cellId(cellId)
{
}

inline Vector3i UniversalIndex3D::blockIdx() const
{
	return Vector3i(_blockIdx[0], _blockIdx[1], _blockIdx[2]);
}

inline size_t UniversalIndex3D::cellId() const
{
	return _cellId;
}

inline bool UniversalIndex3D::operator < (const UniversalIndex3D& rhs) const
{
	if (isLess(_blockIdx, rhs._blockIdx))
		return true;
	else if (rhs.isLess(_blockIdx, _blockIdx))
		return false;

	return _cellId < rhs._cellId;
}

}