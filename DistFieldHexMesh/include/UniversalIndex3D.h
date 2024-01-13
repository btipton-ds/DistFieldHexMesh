#pragma once

#include <Index3D.h>

namespace DFHM {

class Volume;

class UniversalIndex3D {
public:
	UniversalIndex3D() = default;
	UniversalIndex3D(const UniversalIndex3D& src) = default;
	UniversalIndex3D(const Index3D& blockIdx, size_t cellId = -1);

	const Index3D& blockIdx() const;
	size_t cellId() const;

	bool isValid() const;

	bool operator < (const UniversalIndex3D& rhs) const;
	bool operator == (const UniversalIndex3D& rhs) const;
	bool operator != (const UniversalIndex3D& rhs) const;

private:
	Index3D _blockIdx = Index3D(-1, -1, -1);
	size_t _cellId = -1;
};

inline UniversalIndex3D::UniversalIndex3D(const Index3D& blockIdx, size_t cellId)
	: _blockIdx(blockIdx)
	, _cellId(cellId)
{
}

inline const Index3D& UniversalIndex3D::blockIdx() const
{
	return _blockIdx;
}

inline size_t UniversalIndex3D::cellId() const
{
	return _cellId;
}

inline bool UniversalIndex3D::isValid() const
{
	return _cellId != -1 && _blockIdx[0] != -1 && _blockIdx[1] != -1 && _blockIdx[2] != -1;
}

inline bool UniversalIndex3D::operator < (const UniversalIndex3D& rhs) const
{
	if (_blockIdx < rhs._blockIdx)
		return true;
	else if (rhs._blockIdx < _blockIdx)
		return false;

	return _cellId < rhs._cellId;
}

inline bool UniversalIndex3D::operator == (const UniversalIndex3D& rhs) const
{
	return !(operator != (rhs));
}

inline bool UniversalIndex3D::operator != (const UniversalIndex3D& rhs) const
{
	return (*this < rhs) || (rhs < *this);
}


}