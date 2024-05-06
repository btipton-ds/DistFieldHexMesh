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

#include <vertex.h>
#include <edge.h>
#include <polygon.h>
#include <block.h>

using namespace std;
using namespace DFHM;

bool FixedPt::operator < (const FixedPt& rhs) const
{
	for (size_t idx = 0; idx < 3; idx++) {
		if ((*this)[idx] < rhs[idx])
			return true;
		else if (rhs[idx] < (*this)[idx])
			return false;
	}

	return false;
}

Vertex::Vertex(const Vertex& src)
#if 0
	: _lockType(src._lockType)
	, _lockIdx(src._lockIdx)
	, _pt(src._pt)
	, _searchPt(src._searchPt)
#else
	: _pt(src._pt)
	, _searchPt(src._searchPt)
#endif
{
}

Vertex& Vertex::operator = (const Vertex& rhs)
{
#if 0
	_lockType = rhs._lockType;
#endif
	_pt = rhs._pt;
	_searchPt = rhs._searchPt;

	return *this;
}

void Vertex::write(std::ostream& out) const
{
	uint8_t version = 0;
	out.write((char*)&version, sizeof(version));

#if 0
	out.write((char*)&_lockType, sizeof(_lockType));
	out.write((char*)&_lockIdx, sizeof(_lockIdx));
#endif

#if USE_FIXED_PT
	FixedPt _pt; // Fixed point representation of a double precisions point
#else
	writeVector3(out, _pt);
	writeVector3(out, _searchPt);
#endif

}

void Vertex::read(std::istream& in)
{
	uint8_t version;
	in.read((char*)&version, sizeof(version));

#if 0
	in.read((char*)&_lockType, sizeof(_lockType));
	in.read((char*)&_lockIdx, sizeof(_lockIdx));
#endif
#if USE_FIXED_PT
	FixedPt _pt; // Fixed point representation of a double precisions point
#else
	readVector3(in, _pt);
	readVector3(in, _searchPt);
#endif
}

const bool Vertex::operator < (const Vertex& rhs) const
{
#if USE_FIXED_PT
	return _pt < rhs._pt;
#else
	return _searchPt < rhs._searchPt;
#endif
}

ostream& DFHM::operator << (ostream& out, const Vertex& vert)
{
	out << "Vertex " << vert.getId();
	return out;
}
