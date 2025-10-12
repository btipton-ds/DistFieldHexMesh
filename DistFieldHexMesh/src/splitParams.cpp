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

#include <splitParams.h>

#include "io_utils.h"

using namespace DFHM;

SplittingParams::SplittingParams()
{
	maxOrthoAngleRadians = 70.0 / 180.0 * M_PI;
	maxLocalConcavityCrossProduct = -sin(7.5 / 180.0 * M_PI); // Negative angle is concave. Small local concavity is allowed when simplifying concave "fan" tesselations
}

size_t SplittingParams::numConditionalPasses() const
{
	size_t result = 0;

	if (numIntersectionDivs > result)
		result = numIntersectionDivs;

	if (numCurvatureDivs > result)
		result = numCurvatureDivs;

	if (numGapDivs > result)
		result = numGapDivs;

	return result;
}

double SplittingParams::getSharpAngleRadians() const
{
	return sharpAngle_degrees / 180.0 * M_PI;
}

double SplittingParams::getSharpAngleDegrees() const
{
	return sharpAngle_degrees;
}

double SplittingParams::getSinSharpAngle() const
{
	return sin(getSharpAngleRadians());
}

std::vector<Vector3d> SplittingParams::getVolBounds() const
{
	std::vector<Vector3d> result = {
		Vector3d(xMin, yMin, zMin),
		Vector3d(xMax, yMin, zMin),
		Vector3d(xMax, yMax, zMin),
		Vector3d(xMin, yMax, zMin),

		Vector3d(xMin, yMin, zMax),
		Vector3d(xMax, yMin, zMax),
		Vector3d(xMax, yMax, zMax),
		Vector3d(xMin, yMax, zMax),
	};

	return result;
}

void SplittingParams::read(std::istream& in)
{
	uint8_t version;
	IoUtil::read(in, version);

	IoUtil::read(in, symXAxis);
	IoUtil::read(in, symYAxis);
	IoUtil::read(in, symZAxis);

	IoUtil::read(in, numSimpleDivs);
	IoUtil::read(in, numIntersectionDivs);
	IoUtil::read(in, numCurvatureDivs);
	if (version > 0)
		IoUtil::read(in, numGapDivs);
	IoUtil::read(in, curvatureDivsPerCircumference);
	IoUtil::read(in, maxCellFaces);

	IoUtil::read(in, baseBoxOffset);
	IoUtil::read(in, xRotationDeg);
	IoUtil::read(in, yRotationDeg);
	IoUtil::read(in, zRotationDeg);

	IoUtil::read(in, xMin);
	IoUtil::read(in, xMax);
	IoUtil::read(in, yMin);
	IoUtil::read(in, yMax);
	IoUtil::read(in, zMin);
	IoUtil::read(in, zMax);

	IoUtil::read(in, maxGapSize);
	IoUtil::read(in, sharpAngle_degrees);
	IoUtil::read(in, minEdgeLength);

	IoUtil::read(in, xMinDivs);
	IoUtil::read(in, xMaxDivs);
	IoUtil::read(in, yMinDivs);
	IoUtil::read(in, yMaxDivs);
	IoUtil::read(in, zMinDivs);
	IoUtil::read(in, zMaxDivs);

	IoUtil::read(in, xMinGrading);
	IoUtil::read(in, xMaxGrading);
	IoUtil::read(in, yMinGrading);
	IoUtil::read(in, yMaxGrading);
	IoUtil::read(in, zMinGrading);
	IoUtil::read(in, zMaxGrading);

	IoUtil::read(in, dims.data(), 3);

	volDivs.read(in);
}

void SplittingParams::write(std::ostream& out) const
{
	uint8_t version = 1;
	IoUtil::write(out, version);

	IoUtil::write(out, symXAxis);
	IoUtil::write(out, symYAxis);
	IoUtil::write(out, symZAxis);

	IoUtil::write(out, numSimpleDivs);
	IoUtil::write(out, numIntersectionDivs);
	IoUtil::write(out, numCurvatureDivs);
	IoUtil::write(out, numGapDivs);
	IoUtil::write(out, curvatureDivsPerCircumference);
	IoUtil::write(out, maxCellFaces);

	IoUtil::write(out, baseBoxOffset);
	IoUtil::write(out, xRotationDeg);
	IoUtil::write(out, yRotationDeg);
	IoUtil::write(out, zRotationDeg);

	IoUtil::write(out, xMin);
	IoUtil::write(out, xMax);
	IoUtil::write(out, yMin);
	IoUtil::write(out, yMax);
	IoUtil::write(out, zMin);
	IoUtil::write(out, zMax);

	IoUtil::write(out, maxGapSize);
	IoUtil::write(out, sharpAngle_degrees);
	IoUtil::write(out, minEdgeLength);

	IoUtil::write(out, xMinDivs);
	IoUtil::write(out, xMaxDivs);
	IoUtil::write(out, yMinDivs);
	IoUtil::write(out, yMaxDivs);
	IoUtil::write(out, zMinDivs);
	IoUtil::write(out, zMaxDivs);

	IoUtil::write(out, xMinGrading);
	IoUtil::write(out, xMaxGrading);
	IoUtil::write(out, yMinGrading);
	IoUtil::write(out, yMaxGrading);
	IoUtil::write(out, zMinGrading);
	IoUtil::write(out, zMaxGrading);
	IoUtil::write(out, dims.data(), 3);

	volDivs.write(out);
}

