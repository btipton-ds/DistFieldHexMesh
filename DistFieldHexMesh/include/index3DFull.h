#pragma once

#include <Index3D.h>

namespace DFHM {

class Volume;

class Index3DFull {
public:
	Index3DFull() = default;
	Index3DFull(const Index3DFull& src) = default;
	Index3DFull(const Index3D& blockIdx, size_t subBlockId = -1);

	const Index3D& blockIdx() const;
	size_t subBlockId() const;

	bool isValid() const;

	bool operator < (const Index3DFull& rhs) const;
	bool operator == (const Index3DFull& rhs) const;
	bool operator != (const Index3DFull& rhs) const;

private:
	Index3D _blockIdx = Index3D(-1, -1, -1);
	size_t _subBlockId = -1;
};

inline Index3DFull::Index3DFull(const Index3D& blockIdx, size_t subBlockId)
	: _blockIdx(blockIdx)
	, _subBlockId(subBlockId)
{
}

inline const Index3D& Index3DFull::blockIdx() const
{
	return _blockIdx;
}

inline size_t Index3DFull::subBlockId() const
{
	return _subBlockId;
}

inline bool Index3DFull::isValid() const
{
	return _subBlockId != -1 && _blockIdx[0] != -1 && _blockIdx[1] != -1 && _blockIdx[2] != -1;
}

inline bool Index3DFull::operator < (const Index3DFull& rhs) const
{
	if (_blockIdx < rhs._blockIdx)
		return true;
	else if (rhs._blockIdx < _blockIdx)
		return false;

	return _subBlockId < rhs._subBlockId;
}

inline bool Index3DFull::operator == (const Index3DFull& rhs) const
{
	return !(operator != (rhs));
}

inline bool Index3DFull::operator != (const Index3DFull& rhs) const
{
	return (*this < rhs) || (rhs < *this);
}


}