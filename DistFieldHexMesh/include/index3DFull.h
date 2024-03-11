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

#include <index3D.h>

namespace DFHM {

class Volume;

class Index3DFullBase {
protected:
	Index3DFullBase() = default;
	Index3DFullBase(const Index3DFullBase& src) = default;
	Index3DFullBase(const Index3D& blockIdx);
	Index3DFullBase(const Index3D& blockIdx, const Index3D& subBlockIdx);

public:
	const Index3D& blockIdx() const;
	const Index3D& subBlockIdx() const;

	bool isValid() const;

	Index3DFullBase& operator += (const Index3D& idx);
	Index3DFullBase& operator -= (const Index3D& idx);
	Index3DFullBase operator + (const Index3D& idx) const;
	Index3DFullBase operator - (const Index3D& idx) const;

	bool operator < (const Index3DFullBase& rhs) const;
	bool operator == (const Index3DFullBase& rhs) const;
	bool operator != (const Index3DFullBase& rhs) const;

private:
	Index3D _blockIdx = Index3D(-1, -1, -1);
	Index3D _subBlockIdx = Index3D(-1, -1, -1);
};

inline Index3DFullBase::Index3DFullBase(const Index3D& blockIdx)
	: _blockIdx(blockIdx)
{
}

inline Index3DFullBase::Index3DFullBase(const Index3D& blockIdx, const Index3D& subBlockIdx)
	: _blockIdx(blockIdx)
	, _subBlockIdx(subBlockIdx)
{
}

inline const Index3D& Index3DFullBase::blockIdx() const
{
	return _blockIdx;
}


inline bool Index3DFullBase::isValid() const
{
	return _blockIdx.isValid() && _subBlockIdx.isValid();
}

inline Index3DFullBase& Index3DFullBase::operator += (const Index3D& idx)
{
	for (int i = 0; i < 3; i++) {
		size_t temp = _subBlockIdx[i] + idx[i];
		_subBlockIdx[i] = (Index3DBaseType) (temp % Index3D::getBlockDim());
		_blockIdx[i] = (Index3DBaseType) (temp / Index3D::getBlockDim());
	}

	return *this;
}

inline Index3DFullBase& Index3DFullBase::operator -= (const Index3D& idx)
{
	for (int i = 0; i < 3; i++) {
		size_t temp = _subBlockIdx[i] - idx[i];
		_subBlockIdx[i] = (Index3DBaseType)(temp % Index3D::getBlockDim());
		_blockIdx[i] = (Index3DBaseType)(temp / Index3D::getBlockDim());
	}

	return *this;
}

inline Index3DFullBase Index3DFullBase::operator + (const Index3D& idx) const
{
	Index3DFullBase result(*this);
	result += idx;
	return result;
}

inline Index3DFullBase Index3DFullBase::operator - (const Index3D& idx) const
{
	Index3DFullBase result(*this);
	result -= idx;
	return result;
}

inline bool Index3DFullBase::operator < (const Index3DFullBase& rhs) const
{
	if (_blockIdx < rhs._blockIdx)
		return true;
	else if (rhs._blockIdx < _blockIdx)
		return false;

	if (_subBlockIdx < rhs._subBlockIdx)
		return true;
	else if (rhs._subBlockIdx < _subBlockIdx)
		return false;

	return false;
}

inline bool Index3DFullBase::operator == (const Index3DFullBase& rhs) const
{
	return !(*this < rhs) && !(rhs < *this);
}

inline bool Index3DFullBase::operator != (const Index3DFullBase& rhs) const
{
	return !operator== (rhs);
}

class Index3DFull : public Index3DFullBase {
public:
	Index3DFull() = default;
	Index3DFull(const Index3DFull& src) = default;
	Index3DFull(const Index3D& blockIdx);
	Index3DFull(const Index3D& blockIdx, const Index3D& subBlockIdx);
};

inline Index3DFull::Index3DFull(const Index3D& blockIdx)
	:Index3DFullBase(blockIdx)
{
}

inline Index3DFull::Index3DFull(const Index3D& blockIdx, const Index3D& subBlockIdx)
	: Index3DFullBase(blockIdx, subBlockIdx)
{
}

class Index3DIdFull : public Index3DFullBase {
public:
	Index3DIdFull() = default;
	Index3DIdFull(const Index3DIdFull& src) = default;
	explicit Index3DIdFull(const Index3D& blockIdx, size_t elementId);
	explicit Index3DIdFull(const Index3D& blockIdx, const Index3D& subBlockIdx, size_t elementId);

	bool isValid() const;
	size_t elementId() const;

	bool operator < (const Index3DIdFull& rhs) const;
	bool operator == (const Index3DIdFull& rhs) const;
	bool operator != (const Index3DIdFull& rhs) const;

private:
	size_t _elementId = -1;
};

inline Index3DIdFull::Index3DIdFull(const Index3D& blockIdx, size_t elementId)
	: Index3DFullBase(blockIdx)
	, _elementId(elementId)
{
}

inline Index3DIdFull::Index3DIdFull(const Index3D& blockIdx, const Index3D& subBlockIdx, size_t elementId)
	: Index3DFullBase(blockIdx, subBlockIdx)
	, _elementId(elementId)
{
}

inline bool Index3DIdFull::isValid() const
{
	return (_elementId != -1) && Index3DFullBase::isValid();
}

inline size_t Index3DIdFull::elementId() const
{
	return _elementId;
}

inline bool Index3DIdFull::operator < (const Index3DIdFull& rhs) const
{
	if (_elementId < rhs._elementId)
		return true;
	else if (_elementId > rhs._elementId)
		return false;

	return Index3DFullBase::operator<(rhs);
}

inline bool Index3DIdFull::operator == (const Index3DIdFull& rhs) const
{
	return (_elementId == rhs._elementId) && Index3DFullBase::operator==(rhs);
}

inline bool Index3DIdFull::operator != (const Index3DIdFull& rhs) const
{
	return !Index3DIdFull::operator==(rhs);
}

}