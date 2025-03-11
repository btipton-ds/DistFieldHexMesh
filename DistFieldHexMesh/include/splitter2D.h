#pragma once
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

#include <set>
#include <map>

#include <defines.h>
#include <vector>
#include <map>
#include <set>
#include <tm_vector3.h>
#include <tm_plane.h>
#include <Eigen/src/Core/Matrix.h>

namespace DFHM {

class Splitter2D {
public:
	class Vector2Dd : public Eigen::Vector2d
	{
	public:
		Vector2Dd() = default;
		Vector2Dd(const Vector2Dd& src) = default;
		Vector2Dd(const Eigen::Vector2d& src);
		Vector2Dd(double x, double y);
		bool operator < (const auto& rhs) const;
		Vector2Dd operator -(const Vector2Dd& rhs) const;
		Vector2Dd operator +(const Vector2Dd& rhs) const;
		Vector2Dd operator *(double rhs) const;
	};

	struct Edge2D {
		Edge2D(size_t i0 = -1, size_t i1 = -1);
		bool operator < (const Edge2D& rhs) const;
		size_t _indices[2];
	};

	Splitter2D(const Planed& plane);
	void add3DEdge(const Vector3d& pt0, const Vector3d& pt1);

private:
	Vector2Dd project(const Vector3d& pt) const;
	size_t addPoint(const Vector2Dd& pt);
	const Vector2Dd& getPoint(size_t idx) const;
	void splitExisting(const Edge2D& edge);
	bool split(const Edge2D& e0, const Edge2D& e1, std::set<Edge2D>& result);

	std::vector<Vector2Dd> _pts;
	std::map<Vector2Dd, size_t> _ptToIndexMap;
	std::set<Edge2D> _edges;

	Planed _plane;
};

}
