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

#include <splitter2D.h>
#include <tm_lineSegment.hpp>
#include <tolerances.h>

using namespace std;
using namespace DFHM;

class LineSegment2d {
public:
	using Vector2Dd = Splitter2D::Vector2Dd;

	LineSegment2d(const Vector2Dd& pt0 = Vector2Dd(0, 0), const Vector2Dd& pt1 = Vector2Dd(0, 0));
	LineSegment2d(const LineSegment2d& rhs) = default;
	bool intersect(const LineSegment2d& other, double& t) const;
	const Vector2Dd& operator[](size_t idx) const;
	Vector2Dd& operator[](size_t idx);
	Vector2Dd eval(double t);

private:
	vector<Vector2Dd> _vals;
};

Splitter2D::Splitter2D(const Planed& plane)
	: _plane(plane)
{

}

void Splitter2D::add3DEdge(const Vector3d& pt3D0, const Vector3d& pt3D1)
{
	size_t idx0 = addPoint(project(pt3D0));
	size_t idx1 = addPoint(project(pt3D1));
	Edge2D e(idx0, idx1);
	splitExisting(e);

	set<Edge2D> subs;
	for (const auto& e1 : _edges) {
		split(e, e1, subs);
	}

	if (subs.empty()) {
		_edges.insert(e);
	} else {
		_edges.insert(subs.begin(), subs.end());
	}
}

void Splitter2D::splitExisting(const Edge2D& e0)
{
	for (const auto& e1 : _edges) {
		set<Edge2D> subs;
		if (split(e1, e0, subs)) {
			_edges.erase(e1);
			for (const auto& s : subs) {
				_edges.insert(s);
			}
			break;
		}
	}
}

bool Splitter2D::split(const Edge2D& e0, const Edge2D& e1, std::set<Edge2D>& result)
{
	LineSegment2d seg0(getPoint(e0._indices[0]), getPoint(e0._indices[1]));
	LineSegment2d seg1(getPoint(e1._indices[0]), getPoint(e1._indices[1]));
	double t;
	if (seg0.intersect(seg1, t)) {
		Vector2Dd pt0 = seg0[0];
		Vector2Dd pt1 = seg0.eval(t);
		Vector2Dd pt2 = seg0[1];
		result.insert(Edge2D(addPoint(pt0), addPoint(pt1)));
		result.insert(Edge2D(addPoint(pt1), addPoint(pt2)));
	}

	return !result.empty();
}

Splitter2D::Vector2Dd Splitter2D::project(const Vector3d& pt) const
{
	Vector3d xAxis = _plane.getXRef();
	Vector3d yAxis = _plane.getNormal().cross(xAxis);
	Vector3d v = pt - _plane.getOrgin();
	double x = v.dot(xAxis);
	double y = v.dot(yAxis);
	Vector2Dd result(x, y);

	return result;
}

inline const Splitter2D::Vector2Dd& Splitter2D::getPoint(size_t idx) const
{
	return _pts[idx];
}

size_t Splitter2D::addPoint(const Vector2Dd& pt)
{
	auto iter = _ptToIndexMap.find(pt);
	if (iter == _ptToIndexMap.end()) {
		size_t idx = _pts.size();
		_pts.push_back(pt);
		iter = _ptToIndexMap.insert(make_pair(pt, idx)).first;
	}
	return iter->second;
}

Splitter2D::Vector2Dd::Vector2Dd(double x, double y)
	: Eigen::Vector2d(x, y)
{
}

Splitter2D::Vector2Dd::Vector2Dd(const Eigen::Vector2d& rhs)
	: Eigen::Vector2d(rhs)
{
}

Splitter2D::Vector2Dd Splitter2D::Vector2Dd::operator -(const Vector2Dd& rhs) const
{
	auto x = Eigen::Vector2d::operator-(rhs);
	return Vector2Dd(x);
}

Splitter2D::Vector2Dd Splitter2D::Vector2Dd::operator +(const Vector2Dd& rhs) const
{
	auto x = Eigen::Vector2d::operator+(rhs);
	return Vector2Dd(x);
}

Splitter2D::Vector2Dd Splitter2D::Vector2Dd::operator *(double rhs) const
{
	auto x = Eigen::Vector2d::operator*(rhs);
	return Vector2Dd(x);
}

bool Splitter2D::Vector2Dd::operator < (const auto& rhs) const
{
	for (int i = 0; i < 2; i++) {
		if ((*this)[i] < rhs[i])
			return true;
		else if ((rhs)[i] < (*this)[i])
			return false;
	}
	return false;
}


Splitter2D::Edge2D::Edge2D(size_t i0, size_t i1)

{
	_indices[0] = (i0 < i1) ? i0 : i1;
	_indices[1] = (i0 < i1) ? i1 : i0;
}

bool Splitter2D::Edge2D::operator < (const Edge2D& rhs) const
{
	for (int i = 0; i < 2; i++) {
		if (_indices[i] < rhs._indices[i])
			return true;
		else if(_indices[i] > rhs._indices[i])
			return false;
	}
	return false;
}

inline LineSegment2d::LineSegment2d(const Vector2Dd& pt0, const Vector2Dd& pt1)
{
	_vals = { pt0, pt1 };
}

bool LineSegment2d::intersect(const LineSegment2d& other, double& t) const
{
	const double tol = Tolerance::paramTol();
	const double MIN_COS_ANGLE = 1.0e-8;

	Vector2Dd xAxis = _vals[1] - _vals[0];
	double len0 = xAxis.norm();
	if (len0 < tol)
		return  false;
	xAxis /= len0;

	Vector2Dd yAxis(-xAxis[1], xAxis[0]);

	Vector2Dd dir = other[1] - other[0];
	double len1 = dir.norm();
	if (len1 < tol)
		return false;
	dir /= len1;

	auto dp = dir.dot(yAxis);
	if (fabs(dp) < MIN_COS_ANGLE)
		return false; // dir is perpendicular to yAxis, so parallel to xAxis

	Vector2Dd v = other[0] - _vals[0];

	auto h = v.dot(yAxis);
	auto dist0 = -h / dp;
	Vector2Dd hitPt = other[0] +  dir * dist0;

	v = hitPt - _vals[0];
	auto dist1 = v.norm();
	t = dist1 / len0;

#if 1 // Verification code
	Vector2Dd vTest = hitPt - _vals[0];
	double testDist = vTest.dot(yAxis);
	if (fabs(testDist) > tol) {
		assert(!"Point not on seg0");
	}

	Vector2Dd vx = hitPt - other[0];
	vx = vx - dir * vx.dot(dir);
	if (vx.norm() > tol) {
		assert(!"Point not on seg1");
	}

#endif
	
	return tol < t && t < 1 - tol;
}

inline const LineSegment2d::Vector2Dd& LineSegment2d::operator[](size_t idx) const
{
	return _vals[idx];
}

inline LineSegment2d::Vector2Dd& LineSegment2d::operator[](size_t idx)
{
	return _vals[idx];
}

LineSegment2d::Vector2Dd LineSegment2d::eval(double t)
{
	const double tol = Tolerance::paramTol();
	assert(t >= -tol && t <= 1 + tol);
	if (t < 0)
		t = 0;
	else if (t > 1)
		t = 1;
	Vector2Dd v = _vals[1] - _vals[0];
	Vector2Dd x = _vals[0] + v * t;
	return x;
}

