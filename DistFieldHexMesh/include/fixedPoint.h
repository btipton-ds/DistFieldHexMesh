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

#include <tm_lineSegment.hpp>

namespace DFHM {

class FixedPt : public Vector3<int>
{
public:
	static int fromDbl(double val);
	static FixedPt fromDbl(const Vector3d& src);
	static double toDbl(int iVal);
	static Vector3d toDbl(const FixedPt& src);
	static double getFixedScale();

	FixedPt() = default;
	FixedPt(const FixedPt& src) = default;
	FixedPt(const Vector3d& pt);

	bool operator < (const FixedPt& rhs) const;
	bool operator == (const FixedPt& rhs) const;

};

using LineSegmentFixed = LineSegment<FixedPt>;
using LineSegmentFixedSet = std::set < LineSegmentFixed, decltype([](const LineSegmentFixed& lhs, const LineSegmentFixed& rhs)->bool {
	for (int i = 0; i < 2; i++) {
		if (lhs._pts[i] < rhs._pts[i])
			return true;
		else if (rhs._pts[i] < lhs._pts[i])
			return false;
	}
	return false;
	}) > ;

inline FixedPt::FixedPt(const Vector3d& pt)
	: Vector3<int>(fromDbl(pt[0]), fromDbl(pt[1]), fromDbl(pt[2]))
{
}

inline double FixedPt::getFixedScale()
{
	return 1000.0; // +/- 25 m volume
}

inline int FixedPt::fromDbl(double val)
{
	double r = val / getFixedScale();
	assert(fabs(r) < 1.0);
	return (int)(r * INT_MAX);
}

inline FixedPt FixedPt::fromDbl(const Vector3d& src)
{
	return FixedPt(src);
}

inline double FixedPt::toDbl(int iVal)
{
	return (iVal / (double)INT_MAX) * getFixedScale();
}

inline Vector3d FixedPt::toDbl(const FixedPt& src)
{
	return Vector3d(toDbl(src[0]), toDbl(src[1]), toDbl(src[2]));
}

}