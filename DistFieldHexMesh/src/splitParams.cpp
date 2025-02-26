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

#include <splitParams.h>

using namespace DFHM;

size_t SplittingParams::numConditionalPasses() const
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
	size_t version;
	in.read((char*)&version, sizeof(size_t));

	in.read((char*)&uniformRatio, sizeof(uniformRatio));
	in.read((char*)&splitAtSharpVerts, sizeof(splitAtSharpVerts));

	in.read((char*)&symXAxis, sizeof(symXAxis));
	in.read((char*)&symYAxis, sizeof(symYAxis));
	in.read((char*)&symZAxis, sizeof(symZAxis));

	if (version <= 2) {
		size_t dummy;
		in.read((char*)&dummy, sizeof(dummy));
		in.read((char*)&dummy, sizeof(dummy));
	}
	in.read((char*)&numSimpleDivs, sizeof(numSimpleDivs));
	in.read((char*)&numIntersectionDivs, sizeof(numIntersectionDivs));
	in.read((char*)&numSharpVertDivs, sizeof(numSharpVertDivs));
	in.read((char*)&numSharpEdgeIntersectionDivs, sizeof(numSharpEdgeIntersectionDivs));
	in.read((char*)&numCurvatureDivs, sizeof(numCurvatureDivs));
	in.read((char*)&divsPerCurvatureRadius, sizeof(divsPerCurvatureRadius));
	in.read((char*)&divsPerGapCurvatureRadius, sizeof(divsPerGapCurvatureRadius));
	in.read((char*)&maxCellFaces, sizeof(maxCellFaces));
	in.read((char*)&baseBoxOffset, sizeof(baseBoxOffset));
	in.read((char*)&xRotationDeg, sizeof(xRotationDeg));
	in.read((char*)&yRotationDeg, sizeof(yRotationDeg));
	in.read((char*)&zRotationDeg, sizeof(zRotationDeg));
	if (version == 0) {
		double x, y, z;
		in.read((char*)&x, sizeof(x));
		in.read((char*)&y, sizeof(y));
		in.read((char*)&z, sizeof(z));
		dims = Vector3d(x, y, z);
	}
	in.read((char*)&xMin, sizeof(xMin));
	in.read((char*)&xMax, sizeof(xMax));
	in.read((char*)&yMin, sizeof(yMin));
	in.read((char*)&yMax, sizeof(yMax));
	in.read((char*)&zMin, sizeof(zMin));
	in.read((char*)&zMax, sizeof(zMax));
	in.read((char*)&maxGapSize, sizeof(maxGapSize));
	in.read((char*)&maxCurvatureRadius_meters, sizeof(maxCurvatureRadius_meters));
	in.read((char*)&sharpAngle_degrees, sizeof(sharpAngle_degrees));
	in.read((char*)&minSplitEdgeLengthCurvature_meters, sizeof(minSplitEdgeLengthCurvature_meters));
	in.read((char*)&minSplitEdgeLengthGapCurvature_meters, sizeof(minSplitEdgeLengthGapCurvature_meters));

	if (version > 0) {
		if (version < 2) {
			size_t tmp;
			in.read((char*)&tmp, sizeof(tmp));
			xMinDivs = (Index3DBaseType)tmp;
			in.read((char*)&tmp, sizeof(tmp));
			xMaxDivs = (Index3DBaseType)tmp;
			in.read((char*)&tmp, sizeof(tmp));
			yMinDivs = (Index3DBaseType)tmp;
			in.read((char*)&tmp, sizeof(tmp));
			yMaxDivs = (Index3DBaseType)tmp;
			in.read((char*)&tmp, sizeof(tmp));
			zMinDivs = (Index3DBaseType)tmp;
			in.read((char*)&tmp, sizeof(tmp));
			zMaxDivs = (Index3DBaseType)tmp;
		} else {
			in.read((char*)&xMinDivs, sizeof(xMinDivs));
			in.read((char*)&xMaxDivs, sizeof(xMaxDivs));
			in.read((char*)&yMinDivs, sizeof(yMinDivs));
			in.read((char*)&yMaxDivs, sizeof(yMaxDivs));
			in.read((char*)&zMinDivs, sizeof(zMinDivs));
			in.read((char*)&zMaxDivs, sizeof(zMaxDivs));
		}

		in.read((char*)&xMinGrading, sizeof(double));
		in.read((char*)&xMaxGrading, sizeof(double));
		in.read((char*)&yMinGrading, sizeof(double));
		in.read((char*)&yMaxGrading, sizeof(double));
		in.read((char*)&zMinGrading, sizeof(double));
		in.read((char*)&zMaxGrading, sizeof(double));

		in.read((char*)dims.data(), 3 * sizeof(double));
		volDivs.read(in);
	}
}

void SplittingParams::write(std::ostream& out) const
{
	size_t version = 3;
	out.write((char*)&version, sizeof(size_t));

	out.write((char*)&uniformRatio, sizeof(uniformRatio));
	out.write((char*)&splitAtSharpVerts, sizeof(splitAtSharpVerts));

	out.write((char*)&symXAxis, sizeof(symXAxis));
	out.write((char*)&symYAxis, sizeof(symYAxis));
	out.write((char*)&symZAxis, sizeof(symZAxis));

	out.write((char*)&numSimpleDivs, sizeof(numSimpleDivs));
	out.write((char*)&numIntersectionDivs, sizeof(numIntersectionDivs));
	out.write((char*)&numSharpVertDivs, sizeof(numSharpVertDivs));
	out.write((char*)&numSharpEdgeIntersectionDivs, sizeof(numSharpEdgeIntersectionDivs));
	out.write((char*)&numCurvatureDivs, sizeof(numCurvatureDivs));
	out.write((char*)&divsPerCurvatureRadius, sizeof(divsPerCurvatureRadius));
	out.write((char*)&divsPerGapCurvatureRadius, sizeof(divsPerGapCurvatureRadius));
	out.write((char*)&maxCellFaces, sizeof(maxCellFaces));

	out.write((char*)&baseBoxOffset, sizeof(baseBoxOffset));
	out.write((char*)&xRotationDeg, sizeof(xRotationDeg));
	out.write((char*)&yRotationDeg, sizeof(yRotationDeg));
	out.write((char*)&zRotationDeg, sizeof(zRotationDeg));

	out.write((char*)&xMin, sizeof(xMin));
	out.write((char*)&xMax, sizeof(xMax));
	out.write((char*)&yMin, sizeof(yMin));
	out.write((char*)&yMax, sizeof(yMax));
	out.write((char*)&zMin, sizeof(zMin));
	out.write((char*)&zMax, sizeof(zMax));
	out.write((char*)&maxGapSize, sizeof(maxGapSize));
	out.write((char*)&maxCurvatureRadius_meters, sizeof(maxCurvatureRadius_meters));
	out.write((char*)&sharpAngle_degrees, sizeof(sharpAngle_degrees));
	out.write((char*)&minSplitEdgeLengthCurvature_meters, sizeof(minSplitEdgeLengthCurvature_meters));
	out.write((char*)&minSplitEdgeLengthGapCurvature_meters, sizeof(minSplitEdgeLengthGapCurvature_meters));

	out.write((char*)&xMinDivs, sizeof(xMinDivs));
	out.write((char*)&xMaxDivs, sizeof(xMaxDivs));
	out.write((char*)&yMinDivs, sizeof(yMinDivs));
	out.write((char*)&yMaxDivs, sizeof(yMaxDivs));
	out.write((char*)&zMinDivs, sizeof(zMinDivs));
	out.write((char*)&zMaxDivs, sizeof(zMaxDivs));

	out.write((char*)&xMinGrading, sizeof(double));
	out.write((char*)&xMaxGrading, sizeof(double));
	out.write((char*)&yMinGrading, sizeof(double));
	out.write((char*)&yMaxGrading, sizeof(double));
	out.write((char*)&zMinGrading, sizeof(double));
	out.write((char*)&zMaxGrading, sizeof(double));

	out.write((char*)dims.data(), 3 * sizeof(double));
	volDivs.write(out);
}

