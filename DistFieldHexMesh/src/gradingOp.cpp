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
#include <vector>

#include <tm_math.h>
#include <tm_math.hpp>

#include <gradingOp.h>
#include <block.h>
#include <splitParams.h>
#include <volume.h>

using namespace std;
using namespace DFHM;

GradingOp::GradingOp(Block* pBlk, const BuildCFDParams& params, const Index3D& _divs, const Vector3d& grading)
    : _pBlk(pBlk)
    , _params(params)
    , _divs(_divs)
    , _grading(grading)
{
}

void GradingOp::calGradingFactors(int axis, double& scale, double& growFactor) const
{
	scale = 1;
	growFactor = 1;
	if (_grading[axis] > 0 && fabs(_grading[axis] - 1) > 1.0e-6) {
		growFactor = pow(_grading[axis], 1.0 / (_divs[axis] - 1));
		double l = 0, k = 1;
		for (size_t i = 0; i < _divs[axis]; i++) {
			l += k;
			k *= growFactor;
		}
		l = l / _divs[axis];
		scale = 1.0 / l;
	}
}

void GradingOp::createGradedCell(CubeFaceType ft0, CubeFaceType ft1, CubeFaceType ft2) const
{

    if (ft2 == CFT_UNDEFINED) {
        if (ft1 == CFT_UNDEFINED)
            createGradedCells();
        else
            createGradedCellOnEdge(ft0, ft1);
    } else {
        createGradedCellOnCorner(ft0, ft1, ft2);
    }
}

void GradingOp::createGradedCellOnFace(CubeFaceType ft0) const
{
    Vector3d pts[8];
    const auto& cPts = _pBlk->getCornerPts();
	switch (ft0) {
    case CFT_BACK:
        pts[0] = pts[1] = cPts[0];
        pts[3] = pts[2] = cPts[3];
        pts[7] = pts[6] = cPts[7];
        pts[4] = pts[5] = cPts[4];
        pts[0][0] = _params.xMin;
        pts[3][0] = _params.xMin;
        pts[7][0] = _params.xMin;
        pts[4][0] = _params.xMin;
        break;
    case CFT_FRONT:

        pts[0] = pts[1] = cPts[1];
        pts[3] = pts[2] = cPts[2];
        pts[7] = pts[6] = cPts[6];
        pts[4] = pts[5] = cPts[5];
        pts[1][0] = _params.xMax;
        pts[2][0] = _params.xMax;
        pts[6][0] = _params.xMax;
        pts[5][0] = _params.xMax;
        break;
    case CFT_BOTTOM:
        pts[4] = pts[0] = cPts[0];
        pts[5] = pts[1] = cPts[1];
        pts[6] = pts[2] = cPts[2];
        pts[7] = pts[3] = cPts[3];
        pts[0][2] = _params.zMin;
        pts[1][2] = _params.zMin;
        pts[2][2] = _params.zMin;
        pts[3][2] = _params.zMin;
        break;
    case CFT_TOP:

        pts[4] = pts[0] = cPts[4];
        pts[5] = pts[1] = cPts[5];
        pts[6] = pts[2] = cPts[6];
        pts[7] = pts[3] = cPts[7];
        pts[4][2] = _params.zMax;
        pts[5][2] = _params.zMax;
        pts[6][2] = _params.zMax;
        pts[7][2] = _params.zMax;
        break;
    case CFT_LEFT:

        pts[2] = pts[1] = cPts[2];
        pts[3] = pts[0] = cPts[3];
        pts[7] = pts[4] = cPts[7];
        pts[6] = pts[5] = cPts[6];
        pts[1][1] = _params.yMin;
        pts[0][1] = _params.yMin;
        pts[4][1] = _params.yMin;
        pts[5][1] = _params.yMin;
        break;
    case CFT_RIGHT:

        pts[3] = pts[0] = cPts[3];
        pts[2] = pts[1] = cPts[2];
        pts[7] = pts[4] = cPts[7];
        pts[6] = pts[5] = cPts[6];
        pts[2][1] = _params.yMax;
        pts[3][1] = _params.yMax;
        pts[7][1] = _params.yMax;
        pts[6][1] = _params.yMax;
        break;
    }

//    createGradedCells(pts);
}

void GradingOp::createGradedCells() const {
    if (_divs[0] == 0)
        return;

    double xScale, yScale, zScale;
    double xGrading, yGrading, zGrading;
    calGradingFactors(0, xScale, xGrading);
    calGradingFactors(1, yScale, yGrading);
    calGradingFactors(2, zScale, zGrading);

    const auto cPts = _pBlk->getCornerPts().data();
    double kx = 1;
    double t0 = 0;
    for (size_t i = 0; i < _divs[0]; i++) {
        double t1 = t0 + 1.0 / (double)_divs[0] * kx * xScale;
        kx *= xGrading;

        double ky = 1;
        double u0 = 0;
        for (size_t j = 0; j < _divs[1]; j++) {
            double u1 = u0 + 1.0 / (double)_divs[1] * ky * yScale;
            ky *= yGrading;

            double kz = 1;
            double v0 = 0;
            for (size_t k = 0; k < _divs[2]; k++) {
                double v1 = v0 + 1.0 / (double)_divs[2] * kz * zScale;
                kz *= zGrading;

                Index3D blkIdx(i, j, k);

                vector<Vector3d> gPts;
                gPts.resize(8);
                gPts[0] = TRI_LERP(cPts, t0, u0, v0);
                gPts[1] = TRI_LERP(cPts, t1, u0, v0);
                gPts[2] = TRI_LERP(cPts, t1, u1, v0);
                gPts[3] = TRI_LERP(cPts, t0, u1, v0);
                gPts[4] = TRI_LERP(cPts, t0, u0, v1);
                gPts[5] = TRI_LERP(cPts, t1, u0, v1);
                gPts[6] = TRI_LERP(cPts, t1, u1, v1);
                gPts[7] = TRI_LERP(cPts, t0, u1, v1);
                _pBlk->addHexCell(gPts);

                v0 = v1;
            }
            u0 = u1;
        }
        t0 = t1;
    }
}

void GradingOp::createGradedCellOnEdge(CubeFaceType ft0, CubeFaceType ft1) const
{

}

void GradingOp::createGradedCellOnCorner(CubeFaceType ft0, CubeFaceType ft1, CubeFaceType ft2) const
{

}
