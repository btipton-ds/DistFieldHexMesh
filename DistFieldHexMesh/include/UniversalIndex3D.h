#pragma once

#include <Index3D.h>

namespace DFHM {

class Volume;

class UniversalIndex3D {
public:
	UniversalIndex3D() = default;
	UniversalIndex3D(const UniversalIndex3D& src) = default;
	UniversalIndex3D(const Index3D& blockIdx, size_t subBlockId = -1);

	const Index3D& blockIdx() const;
	size_t subBlockId() const;

	bool isValid() const;

	bool operator < (const UniversalIndex3D& rhs) const;
	bool operator == (const UniversalIndex3D& rhs) const;
	bool operator != (const UniversalIndex3D& rhs) const;

private:
	Index3D _blockIdx = Index3D(-1, -1, -1);
	size_t _subBlockId = -1;
};

inline UniversalIndex3D::UniversalIndex3D(const Index3D& blockIdx, size_t subBlockId)
	: _blockIdx(blockIdx)
	, _subBlockId(subBlockId)
{
}

inline const Index3D& UniversalIndex3D::blockIdx() const
{
	return _blockIdx;
}

inline size_t UniversalIndex3D::subBlockId() const
{
	return _subBlockId;
}

inline bool UniversalIndex3D::isValid() const
{
	return _subBlockId != -1 && _blockIdx[0] != -1 && _blockIdx[1] != -1 && _blockIdx[2] != -1;
}

inline bool UniversalIndex3D::operator < (const UniversalIndex3D& rhs) const
{
	if (_blockIdx < rhs._blockIdx)
		return true;
	else if (rhs._blockIdx < _blockIdx)
		return false;

	return _subBlockId < rhs._subBlockId;
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