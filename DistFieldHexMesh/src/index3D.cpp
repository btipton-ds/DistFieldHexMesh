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
#include <volume.h>

using namespace std;
using namespace DFHM;

Index3DBaseType Index3DBase::s_blockDim = 8;

Index3DBase::Index3DBase()
{
	_vals[0] = -1;
	_vals[1] = -1;
	_vals[2] = -1;
	_vals[3] = -1;
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
		out << "[(" << id[0] << " " << id[1] << " " << id[2] << "): " << id.elementId() << "]";
	else
		out << "[(***): *]";
	return out;
}

ostream& DFHM::operator << (ostream& out, const Index3D& idx)
{
	if (idx.isValid())
		out << "(" << idx[0] << " " << idx[1] << " " << idx[2] << ")";
	else
		out << "(***)";

	return out;
}

bool Index3DId::isUserFlagSet(uint32_t bit) const
{
	return (_userFlags & bit) == bit;
}

void Index3DId::setUserFlag(uint32_t bit, bool val) const
{
	if (val)
		_userFlags = _userFlags | bit;
	else
		_userFlags = _userFlags & ~bit;
}

void Index3DId::write(std::ostream& out) const
{
	Index3DBase::write(out);

	uint8_t version = 1;
	out.write((char*)&version, sizeof(version));
	out.write((char*)&_elementId, sizeof(_elementId));
	out.write((char*)&_userFlags, sizeof(_userFlags));
}

void Index3DId::read(std::istream& in)
{
	Index3DBase::read(in);
	uint8_t version = 0;
	in.read((char*)&version, sizeof(version));
	in.read((char*)&_elementId, sizeof(_elementId));
	if (version == 0) {
		size_t deprecated;
		in.read((char*)&deprecated, sizeof(deprecated));
	}
	in.read((char*)&_userFlags, sizeof(_userFlags));
}
