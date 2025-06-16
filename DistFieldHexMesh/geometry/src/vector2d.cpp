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

#include <vector2d.h>
#include <tolerances.h>
#include <vertex.h>

using namespace DFHM;

Eigen::Matrix<int64_t, 2, 1> Vector2d::asIntVec() const
{
	int64_t iVal0 = Vertex::scaleToSearch((*this)[0]);
	int64_t iVal1 = Vertex::scaleToSearch((*this)[1]);
	Eigen::Matrix<int64_t, 2, 1> result(iVal0, iVal1);
	return result;
}

bool Vector2d::operator < (const Vector2d& rhs) const
{
	int64_t scale = Vertex::scaleToSearch();
	auto pt0 = asIntVec();
	auto pt1 = rhs.asIntVec();
	for (int i = 0; i < 2; i++) {
		if (pt0[i] < pt1[i])
			return true;
		else if (pt1[i] < pt0[i])
			return false;
	}
	return false;
}
