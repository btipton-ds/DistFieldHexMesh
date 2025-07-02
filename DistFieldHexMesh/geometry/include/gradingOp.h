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

	Copyright Robert R Tipton, 2022, all rights reserved except those granted in prior license statement.

	Dark Sky Innovative Solutions http://darkskyinnovation.com/
*/

#include <defines.h>
#include <tm_vector3.h>
#include <pool_vector.h>
#include <index3D.h>
#include <enums.h>

namespace DFHM {

class Block;
class Volume;
struct SplittingParams;

class GradingOp {
public:
	static const MTC::vector<MTC::vector<size_t>>&getHexFaceIndices();
	static void getHexFacePoints(const MTC::vector<Vector3d>&cornerPts, MTC::vector<MTC::vector<Vector3d>>&facePts);
	static void getHexFaceVertIds(const MTC::vector<Index3DId>&cornerVerts, MTC::vector<MTC::vector<Index3DId>>&faceVerts);

	static const MTC::vector<MTC::vector<size_t>>& getWedgeFaceIndices();
	static void getWedgeFacePoints(const MTC::vector<Vector3d>& cornerPts, MTC::vector<MTC::vector<Vector3d>>& facePts);
	static void getWedgeFaceVertIds(const MTC::vector<Index3DId>& cornerVerts, MTC::vector<MTC::vector<Index3DId>>& faceVerts);

	GradingOp(Block* pBlk, const SplittingParams& params, const Index3D& divs = Index3D(0, 0, 0), const Vector3d& grading = Vector3d(1, 1, 1));
	void calGradingFactors(int axis, double& scale, double& growFactor) const;

	const Index3D& getDivs() const;
	void setDivs(const Index3D& divs);

	const Vector3d& getGrading() const;
	void setGrading(const Vector3d& grading);

	void createGradedCells() const;
	void createGradedCell(HexFaceType ft0, HexFaceType ft1, HexFaceType ft2) const;
private:
	void createGradedCellOnFace(HexFaceType ft0) const;
	void createGradedCellOnEdge(HexFaceType ft0, HexFaceType ft1) const;
	void createGradedCellOnCorner(HexFaceType ft0, HexFaceType ft1, HexFaceType ft2) const;

	Block* _pBlk;
	const SplittingParams& _params;
	Index3D _divs;
	Vector3d _grading;

};

inline const Index3D& GradingOp::getDivs() const
{
	return _divs;
}

inline void GradingOp::setDivs(const Index3D& divs)
{
	_divs = divs;
}

inline const Vector3d& GradingOp::getGrading() const
{
	_grading;
}

inline void GradingOp::setGrading(const Vector3d& grading)
{
	_grading = grading;
}

}
