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
#include <tm_vector3.h>
#include <index3D.h>

namespace DFHM {


struct GradingRec {
	Index3D divs = Index3D(0, 0, 0);
	Vector3d grading;
	void calGradingFactors(int axis, double& scale, double& growFactor) const;
};

inline void GradingRec::calGradingFactors(int axis, double& scale, double& growFactor) const
{
	scale = 1;
	growFactor = 1;
	if (grading[axis] > 0 && fabs(grading[axis] - 1) > 1.0e-6) {
		growFactor = pow(grading[axis], 1.0 / (divs[axis] - 1));
		double l = 0, k = 1;
		for (size_t i = 0; i < divs[axis]; i++) {
			l += k;
			k *= growFactor;
		}
		l = l / divs[axis];
		scale = 1.0 / l;
	}

}


}
