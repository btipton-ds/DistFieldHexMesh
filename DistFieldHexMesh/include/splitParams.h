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
#include <cmath>

namespace DFHM {

struct BuildCFDParams {
	inline size_t numConditionalPasses() const
	{
		size_t result = 0;

		if (numIntersectionDivs > result)
			result = numIntersectionDivs;

		if (numSharpVertDivs > result)
			result = numSharpVertDivs;

		if (numCurvatureDivs > result)
			result = numCurvatureDivs;

		if (numSharpEdgeIntersectionDivs > result)
			result = numSharpEdgeIntersectionDivs;

		return result;
	}
	double getSharpAngleRadians() const;
	double getSharpAngleDegrees() const;

	bool uniformRatio = false;
	bool splitAtSharpVerts = true;
	size_t minBlocksPerSide = 6;
	size_t numBlockDivs = 0;
	size_t numSimpleDivs = 0;
	size_t numIntersectionDivs = 1;
	size_t numSharpVertDivs = 0;
	size_t numSharpEdgeIntersectionDivs = 0;
	size_t numCurvatureDivs = 0;
	size_t divsPerCurvatureRadius = 2;
	size_t divsPerGapCurvatureRadius = 4;
	size_t maxCellFaces = 12;
	double maxGapSize = 0.01; // 10 mm
	double maxCurvatureRadius_meters = 1.0; // 1m
	double sharpAngle_degrees = SHARP_EDGE_ANGLE_DEGREES;
	double minSplitEdgeLengthCurvature_meters = 0.001;  //  1 mm
	double minSplitEdgeLengthGapCurvature_meters = 0.001;  //  1 mm
};

}
