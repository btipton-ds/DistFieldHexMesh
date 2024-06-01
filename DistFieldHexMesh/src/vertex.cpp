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

#include <tolerances.h>
#include <vertex.h>
#include <edge.h>
#include <polygon.h>
#include <block.h>
#include <fixedPoint.h>

using namespace std;
using namespace DFHM;

Vertex::Vertex(const Vertex& src)
	: _pt(src._pt)
	, _lockType(src._lockType)
{
}

Vertex& Vertex::operator = (const Vertex& rhs)
{
	_pt = rhs._pt;
	_lockType = rhs._lockType;

	return *this;
}

void Vertex::write(std::ostream& out) const
{
	uint8_t version = 2;
	out.write((char*)&version, sizeof(version));

	writeVector3(out, _pt);

	out.write((char*)&_lockType, sizeof(_lockType));
}

void Vertex::read(std::istream& in)
{
	uint8_t version;
	in.read((char*)&version, sizeof(version));

	readVector3(in, _pt);
	if (version < 2) {
		FixedPt deprecatedPt;
		readVector3(in, deprecatedPt);
	}

	if (version >= 1) {
		in.read((char*)&_lockType, sizeof(_lockType));
	}
}

CBoundingBox3Dd Vertex::calBBox(const Vector3d& pt)
{
	CBoundingBox3Dd result(pt, pt);
	result.grow(Tolerance::sameDistTol() / 2.0);

	return result;
}

const bool Vertex::operator < (const Vertex& rhs) const
{
	return _pt < rhs._pt;
}

ostream& DFHM::operator << (ostream& out, const Vertex& vert)
{
	out << "Vertex " << vert.getId();
	return out;
}
