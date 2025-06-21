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

#include <lineSegment2D.h>
#include <tolerances.h>

using namespace DFHM;

bool LineSegment2d::project(const Vector2d& pt, double& t) const
{
	const double tolSqr = Tolerance::sameDistTolSqr();

	Vector2d dir = (_pts[1] - _pts[0]);
	auto len = dir.norm();
	dir /= len;

	Vector2d v = pt - _pts[0];
	auto dp = v.dot(dir);
	t = dp / len;
	v = v - dir * dp;
	auto errSqr = v.squaredNorm();
	return errSqr < tolSqr;
}

bool LineSegment2d::intersectRay(const LineSegment2d& ray, Vector2d& iPt) const
{
	const double tol = Tolerance::sameDistTol();
	const double paramTol = Tolerance::paramTol();

	Vector2d segOrigin = _pts[0];
	Vector2d xAxis = (_pts[1] - _pts[0]);
	double ourLenSqr = xAxis.squaredNorm();
	if (ourLenSqr < Tolerance::sameDistTolSqr())
		return false; // Degenerate segment

	double ourLen = sqrt(ourLenSqr);
	xAxis /= ourLen;

	Vector2d yAxis(-xAxis[1], xAxis[0]);

	Vector2d localRayStart(xAxis.dot(ray._pts[0] - segOrigin), yAxis.dot(ray._pts[0] - segOrigin));
	Vector2d localRayTip(xAxis.dot(ray._pts[1] - segOrigin), yAxis.dot(ray._pts[1] - segOrigin));
	Vector2d localRayDir = (localRayTip - localRayStart).normalized();

	double x;
	double x0 = localRayStart[0];
	double y0 = localRayStart[1];
	if (fabs(localRayDir[0]) < paramTol) {
		// Vertical ray
		x = x0;
	} else {
		// y0 = x0 * slope + b
		double slope = localRayDir[1] / localRayDir[0];
		if (fabs(slope) > paramTol) {
			double b = y0 - x0 * slope;
			x = -b / slope;
		} else {
			return false; // This is an infinite number intersections or no intersection. Let another seg handle this case.
		}
	}

	if (-tol < x && x < ourLen + tol) {
		iPt = segOrigin + x * xAxis;
		return true;
	}
	return false;
}

bool LineSegment2d::intersectionInBounds(const LineSegment2d& other, double& t) const
{
	const double tol = Tolerance::paramTol();
	double tOther = -1;
	t = -1;

	for (int i = 0; i < 2; i++) {
		for (int j = 0; j < 2; j++) {
			if ((_pts[i] - other._pts[j]).squaredNorm() < Tolerance::sameDistTolSqr()) {
				// The line segments share a point
				t = (i == 0) ? 0 : 1;
				tOther = (j == 0) ? 0 : 1;
				return true;
			}
		}
	}

	Vector2d xAxis = _pts[1] - _pts[0];
	double len0 = xAxis.norm();
	if (len0 < tol)
		return  false;
	xAxis /= len0;

	auto vOther = other[1] - other[0];
	auto lenOther = vOther.norm();
	if (lenOther < tol)
		return  false;
	vOther /= lenOther;

	Vector2d yAxis(-xAxis[1], xAxis[0]);

	auto v0 = other[0] - _pts[0];
	auto v1 = other[1] - _pts[0];

	double x0 = xAxis.dot(v0);
	double y0 = yAxis.dot(v0);

	double x1 = xAxis.dot(v1);
	double y1 = yAxis.dot(v1);

	double dx = x1 - x0;
	double dy = y1 - y0;

	if (fabs(dy) < Tolerance::sameDistTol()) {
		// Horizontal line case
		if (fabs(y0) >= tol && fabs(y1) >= tol) {
			return false; // They are parallel but not colinear, not intersecting
		}

		double t00 = x0 / len0;
		double t01 = x1 / len0;

		// AND both of the test seg's points fall outside our range
		if ((t00 <= tol && t01 <= tol) || (t00 >= 1 - tol && t01 >= 1 - tol)) {
			// Both are less than zero or both are greater than 1
			return false;
		}

		if (-tol < t00 && t00 < 1 + tol) {
			t = t00;
		}
		else {
			t = t01;
		}
	}
	else {
		double slope = DBL_MAX;;
		if (fabs(dx) >= tol)
			slope = dy / dx;

		if (fabs(slope) > 1.0e6) {
			// Vertical line case
			double sign = fabs(y0) > 0 ? y1 / y0 : (fabs(y1) > 0 ? y0 / y1 : 0);

			if (sign <= 0) {
				// crosses x axis
				double x = 0.5 * (x0 + x1);
				t = x / len0;
			}
			else
				return false;
		}
		else {
			double b = y0 - slope * x0;
			//			double xIntercept = -b / slope;
			t = (-b / slope) / len0;
		}

		if (-tol < t && t < 1 + tol) {
			if (t < 0)
				t = 0;
			else if (t > 1)
				t = 1;
		}
		else
			return false;

		auto iPt = _pts[0] + xAxis * len0 * t;

		auto v = iPt - other[0];
		auto distOther = vOther.dot(v);
		tOther = distOther / lenOther;

		if (-tol < tOther && tOther < 1 + tol) {
#if VALIDATION_ON && defined(_DEBUG)
			v = v - vOther * distOther;
			assert(v.squaredNorm() < Tolerance::sameDistTolSqr());
			auto iPtOther = other[0] + vOther * lenOther * tOther;
			assert((iPtOther - iPt).squaredNorm() < Tolerance::sameDistTolSqr());
#endif
			return true;
		}

		return false;
	}

	return false;
}

Vector2d LineSegment2d::interpolate(double t) const
{
	const double tol = Tolerance::paramTol();
	assert(t >= -tol && t <= 1 + tol);
	if (t < 0)
		t = 0;
	else if (t > 1)
		t = 1;
	Vector2d v = _pts[1] - _pts[0];
	Vector2d x = _pts[0] + v * t;
	return x;
}

Vector2d LineSegment2d::dir() const
{
	Vector2d result = (_pts[1] - _pts[0]);
	auto l = result.norm();
	if (l > 0)
		result / l;
	return result;
}

double LineSegment2d::length() const
{
	return (_pts[1] - _pts[0]).norm();
}
