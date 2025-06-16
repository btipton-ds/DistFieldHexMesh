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

	Copyright Robert R Tipton, 2022, all rights reserved except those granted in prior license statement.

	Dark Sky Innovative Solutions http://darkskyinnovation.com/
*/

#include <index3D.h>
#include <volume.h>

using namespace std;
using namespace DFHM;

Index3DBaseType Index3DBase::s_blockDim = 8;

Index3DBase::Index3DBase()
{
//	assert(sizeof(Index3DBase) == 8);
	_iVal = 0;
	_vals[0] = 0xff;
	_vals[1] = 0xff;
	_vals[2] = 0xff;
	_flags = 0;
//	assert(_iVal == getMask());
}

Index3DBase& Index3DBase::operator += (const Index3DBase& rhs)
{
	_vals[0] += rhs._vals[0];
	_vals[1] += rhs._vals[1];
	_vals[2] += rhs._vals[2];

	return *this;
}


Index3DBase Index3DBase::operator + (const Index3DBase& rhs) const
{
	Index3DBase temp(*this);
	temp += rhs;
	return temp;
}

Index3DBase& Index3DBase::operator -= (const Index3DBase& rhs)
{
	_vals[0] -= rhs._vals[0];
	_vals[1] -= rhs._vals[1];
	_vals[2] -= rhs._vals[2];

	return *this;
}

Index3DBase Index3DBase::operator - (const Index3DBase& rhs) const
{
	Index3DBase temp(*this);
	temp -= rhs;
	return temp;
}

void Index3DBase::clampInBounds(size_t bound)
{
	for (size_t i = 0; i < 3; i++) {
		if (_vals[i] >= bound)
			_vals[i] = (Index3DBaseType) (bound - 1);
	}
}

void Index3DBase::clampInBounds(const Index3DBase& bounds)
{
	for (size_t i = 0; i < 3; i++) {
		if (_vals[i] >= bounds._vals[i])
			_vals[i] = bounds._vals[i];
	}
}

void Index3DBase::write(std::ostream& out) const
{
	uint8_t version = 0;
	out.write((char*)&version, sizeof(version));
	out.write((char*)&_iVal, sizeof(_iVal));
}

void Index3DBase::read(std::istream& in)
{
	uint8_t version = 0;
	in.read((char*)&version, sizeof(version));
	in.read((char*)&_iVal, sizeof(_iVal));
}

ostream& DFHM::operator << (ostream& out, const Index3DId& id)
{
	if (id.isValid())
		out << "[(" << (int)id[0] << " " << (int)id[1] << " " << (int)id[2] << "): " << id.elementId() << "]";
	else
		out << "[(***): *]";
	return out;
}

ostream& DFHM::operator << (ostream& out, const Index3D& idx)
{
	if (idx.isValid())
		out << "(" << (int)idx[0] << " " << (int)idx[1] << " " << (int)idx[2] << ")";
	else
		out << "(***)";

	return out;
}

bool Index3DBase::isUserFlagSet(Index3DBaseType bit) const
{
	return (_flags & bit) == bit;
}

void Index3DBase::setUserFlag(Index3DBaseType bit, bool val) const
{
	if (val)
		_flags = _flags | bit;
	else
		_flags = _flags & ~bit;
}

void Index3DId::write(std::ostream& out) const
{
	Index3DBase::write(out);

	uint8_t version = 0;
	out.write((char*)&version, sizeof(version));
	out.write((char*)&_elementId, sizeof(_elementId));
}

void Index3DId::read(std::istream& in)
{
	Index3DBase::read(in);
	uint8_t version = 0;
	in.read((char*)&version, sizeof(version));
	in.read((char*)&_elementId, sizeof(_elementId));
}
