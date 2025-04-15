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
#include <vector>
#include <iostream>
#include <index3D.h>

namespace DFHM {

struct SplittingParams {
	size_t numConditionalPasses() const;
	double getSharpAngleRadians() const;
	double getSharpAngleDegrees() const;
	double getSinSharpAngle() const;
	std::vector<Vector3d> getVolBounds() const;

	void read(std::istream& in);
	void write(std::ostream& out) const;

	bool uniformRatio = false;
	bool splitAtSharpVerts = true;

	bool symXAxis = false;
	bool symYAxis = true;
	bool symZAxis = false;

	size_t numSimpleDivs = 0;
	size_t numIntersectionDivs = 0;
	size_t numSharpVertDivs = 0;
	size_t numSharpEdgeIntersectionDivs = 0;
	size_t numCurvatureDivs = 1;
	size_t curvatureDivsPerCircumference = 18;
	size_t divsPerGapCurvatureRadius = 4;
	size_t maxCoplanarFaces = 4;
	size_t maxCellFaces = 12;

	Index3DBaseType xMinDivs = 2;
	Index3DBaseType xMaxDivs = 2;
	Index3DBaseType yMinDivs = 2;
	Index3DBaseType yMaxDivs = 2;
	Index3DBaseType zMinDivs = 2;
	Index3DBaseType zMaxDivs = 2;

	double baseBoxOffset = 2;
	double xRotationDeg = 0;
	double yRotationDeg = 45;
	double zRotationDeg = 0;
	double xMin = DBL_MAX, xMax = 1;
	double yMin = DBL_MAX, yMax = 1;
	double zMin = DBL_MAX, zMax = 1;
	double maxGapSize = 0.01; // 10 mm
	double ignoreCurvatureRadius_meters = 0.3; // ignore chordal divisions > ~10 in
	double sharpAngle_degrees = SHARP_EDGE_ANGLE_DEGREES;
	double sharpRadius = 0.001; // 1 mm
	double maxRadius = 100; // m
	double minSplitEdgeLengthCurvature_meters = 0.001;  //  1 mm
	double minSplitEdgeLengthGapCurvature_meters = 0.001;  //  1 mm
	double maxOrthoAngleRadians = 70.0 / 180.0 * M_PI;

	// These are used only to determine which cell is most complex so it gets split first
	double complexityFaceFactor = 1;
	double complexitySubFaceFactor = 1.5;
	double complexityOrthoFactor = 1.25;

	double xMinGrading = 5;
	double xMaxGrading = 5;
	double yMinGrading = 5;
	double yMaxGrading = 5;
	double zMinGrading = 5;
	double zMaxGrading = 5;

	Vector3d dims = Vector3d(2, 1, 1);
	Index3D volDivs = Index3D(1, 1, 1);
};

}
