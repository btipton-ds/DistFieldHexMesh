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

#include <defines.h>
#include <Vector2d.h>

namespace DFHM {

class LineSegment2d {
public:
	LineSegment2d(const Vector2d& pt0 = Vector2d(0, 0), const Vector2d& pt1 = Vector2d(0, 0));
	LineSegment2d(const LineSegment2d& rhs) = default;

	bool project(const Vector2d& pt, double& t) const;
	bool intersectRay(const LineSegment2d& ray, Vector2d& iPt) const;
	bool intersectionInBounds(const LineSegment2d& other, double& t) const;
	const Vector2d& operator[](size_t idx) const;
	Vector2d& operator[](size_t idx);
	Vector2d interpolate(double t) const;
	double length() const;
	Vector2d dir() const;
	double distToPoint(const Vector2d& pt) const;
	double distToPointSqr(const Vector2d& pt) const;

private:
	std::vector<Vector2d> _pts;
};

inline LineSegment2d::LineSegment2d(const Vector2d& pt0, const Vector2d& pt1)
{
	_pts = { pt0, pt1 };
}

inline const Vector2d& LineSegment2d::operator[](size_t idx) const
{
	return _pts[idx];
}

inline Vector2d& LineSegment2d::operator[](size_t idx)
{
	return _pts[idx];
}

}