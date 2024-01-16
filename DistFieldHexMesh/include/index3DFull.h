#pragma once

#include <Index3D.h>

namespace DFHM {

class Volume;

class Index3DFull {
public:
	Index3DFull() = default;
	Index3DFull(const Index3DFull& src) = default;
	Index3DFull(const Index3D& blockIdx, size_t elementId = -1);
	Index3DFull(const Index3D& blockIdx, const Index3D& subBlockIdx, size_t elementId = -1);

	const Index3D& blockIdx() const;
	const Index3D& subBlockIdx() const;
	size_t elementId() const;

	bool isValid() const;

	Index3DFull& operator += (const Index3D& idx);
	Index3DFull& operator -= (const Index3D& idx);
	Index3DFull operator + (const Index3D& idx) const;
	Index3DFull operator - (const Index3D& idx) const;

	bool operator < (const Index3DFull& rhs) const;
	bool operator == (const Index3DFull& rhs) const;
	bool operator != (const Index3DFull& rhs) const;

private:
	Index3D _blockIdx = Index3D(-1, -1, -1);
	Index3D _subBlockIdx = Index3D(-1, -1, -1);
	size_t _elementId = -1;
};

inline Index3DFull::Index3DFull(const Index3D& blockIdx, size_t elementId)
	: _blockIdx(blockIdx)
	, _elementId(elementId)
{
}

inline Index3DFull::Index3DFull(const Index3D& blockIdx, const Index3D& subBlockIdx, size_t elementId)
	: _blockIdx(blockIdx)
	, _subBlockIdx(subBlockIdx)
	, _elementId(elementId)
{
}

inline const Index3D& Index3DFull::blockIdx() const
{
	return _blockIdx;
}

inline size_t Index3DFull::elementId() const
{
	return _elementId;
}

inline bool Index3DFull::isValid() const
{
	return _elementId != -1 && _blockIdx[0] != -1 && _blockIdx[1] != -1 && _blockIdx[2] != -1;
}

inline Index3DFull& Index3DFull::operator += (const Index3D& idx)
{
	for (int i = 0; i < 3; i++) {
		size_t temp = _subBlockIdx[i] + idx[i];
		_subBlockIdx[i] = (Index3DBaseType) (temp % Index3D::getBlockDim());
		_blockIdx[i] = (Index3DBaseType) (temp / Index3D::getBlockDim());
	}

	return *this;
}

inline Index3DFull& Index3DFull::operator -= (const Index3D& idx)
{
	for (int i = 0; i < 3; i++) {
		size_t temp = _subBlockIdx[i] - idx[i];
		_subBlockIdx[i] = (Index3DBaseType)(temp % Index3D::getBlockDim());
		_blockIdx[i] = (Index3DBaseType)(temp / Index3D::getBlockDim());
	}

	return *this;
}

inline Index3DFull Index3DFull::operator + (const Index3D& idx) const
{
	Index3DFull result(*this);
	result += idx;
	return result;
}

inline Index3DFull Index3DFull::operator - (const Index3D& idx) const
{
	Index3DFull result(*this);
	result -= idx;
	return result;
}

inline bool Index3DFull::operator < (const Index3DFull& rhs) const
{
	if (_blockIdx < rhs._blockIdx)
		return true;
	else if (rhs._blockIdx < _blockIdx)
		return false;

	if (_subBlockIdx < rhs._subBlockIdx)
		return true;
	else if (rhs._subBlockIdx < _subBlockIdx)
		return false;

	return _elementId < rhs._elementId;
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