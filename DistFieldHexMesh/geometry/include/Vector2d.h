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

#include <defines.h>
#undef Success
#include <Eigen/Dense>
#include <Eigen/src/Core/Matrix.h>
#include <fastBisectionSet.h>

namespace DFHM {

class Vector2d : public Eigen::Vector2d
{
public:
	Vector2d() = default;
	Vector2d(const Vector2d& src) = default;
	Vector2d(const Eigen::Vector2d& src);
	Vector2d(double x, double y);

	Eigen::Matrix<int64_t, 2, 1> asIntVec() const;

	bool operator < (const Vector2d& rhs) const;
	Vector2d operator -(const Vector2d& rhs) const;
	Vector2d operator +(const Vector2d& rhs) const;
	Vector2d operator *(double rhs) const;
};

inline Vector2d::Vector2d(double x, double y)
	: Eigen::Vector2d(x, y)
{
}

inline Vector2d::Vector2d(const Eigen::Vector2d& rhs)
	: Eigen::Vector2d(rhs)
{
}

inline Vector2d Vector2d::operator -(const Vector2d& rhs) const
{
	auto x = Eigen::Vector2d::operator-(rhs);
	return Vector2d(x);
}

inline Vector2d Vector2d::operator +(const Vector2d& rhs) const
{
	auto x = Eigen::Vector2d::operator+(rhs);
	return Vector2d(x);
}

inline Vector2d Vector2d::operator *(double rhs) const
{
	auto x = Eigen::Vector2d::operator*(rhs);
	return Vector2d(x);
}

inline Vector2d operator * (double t, const Vector2d& v)
{
	return v * t;
}

}