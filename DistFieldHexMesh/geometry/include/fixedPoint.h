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

	Copyright Robert R Tipton, 2022, all rights reserved except those granted in prior license statement.

	Dark Sky Innovative Solutions http://darkskyinnovation.com/
*/

#include <tm_lineSegment.hpp>
#include <index3D.h>

namespace DFHM {

#if 1
#define FIXED_PT_SCALAR_TYPE int
#define FIXED_PT_INT_MAX INT_MAX
#else
#define FIXED_PT_SCALAR_TYPE int64_t
#define FIXED_PT_INT_MAX LLONG_MAX
#endif

	class FixedPt : public Vector3<FIXED_PT_SCALAR_TYPE>
{
public:
	static FIXED_PT_SCALAR_TYPE fromDbl(double val);
	static FixedPt fromDbl(const Vector3d& src);
	static double toDbl(FIXED_PT_SCALAR_TYPE iVal);
	static Vector3d toDbl(const FixedPt& src);
	static double getFixedScale();

	FixedPt() = default;
	FixedPt(const FixedPt& src) = default;
	FixedPt(const Vector3d& pt);

	bool operator < (const FixedPt& rhs) const;
	bool operator == (const FixedPt& rhs) const;

};

using LineSegmentFixed = LineSegment<FixedPt>;

inline FixedPt::FixedPt(const Vector3d& pt)
	: Vector3<FIXED_PT_SCALAR_TYPE>(fromDbl(pt[0]), fromDbl(pt[1]), fromDbl(pt[2]))
{
}

inline double FixedPt::getFixedScale()
{
	return 1000.0; // +/- 25 m volume
}

inline FIXED_PT_SCALAR_TYPE FixedPt::fromDbl(double val)
{
	double r = val / getFixedScale();
	assert(fabs(r) < 1.0);
	return (FIXED_PT_SCALAR_TYPE)(r * FIXED_PT_INT_MAX); // LLONG_MAX
}

inline FixedPt FixedPt::fromDbl(const Vector3d& src)
{
	return FixedPt(src);
}

inline double FixedPt::toDbl(FIXED_PT_SCALAR_TYPE iVal)
{
	return (iVal / (double)FIXED_PT_INT_MAX) * getFixedScale();
}

inline Vector3d FixedPt::toDbl(const FixedPt& src)
{
	return Vector3d(toDbl(src[0]), toDbl(src[1]), toDbl(src[2]));
}

}
