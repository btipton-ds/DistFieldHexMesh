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

#include <rayModelIntersector.h>
#include <tm_spatialSearch.hpp>
#include <triMesh.h>

#include <tolerances.h>
#include <splitParams.h>
#include <model.h>
#include <vertex.h>
#include <polygon.h>

using namespace std;
using namespace DFHM;

RayModelIntersector::RayModelIntersector(const SplittingParams& params, const Model& model, Vertex& vert)
	: _model(model)
	, _params(params)
	, _vert(vert)
	, _tol(Tolerance::sameDistTol())
{
	_pTree = model.getPolySearchTree();
}

bool RayModelIntersector::castRay(const Rayd& ray, vector<SolidHitRec>& hitsOnSolidModel) {
	bool result = true;
	_pTree->biDirRayCastTraverse(ray, [this, &hitsOnSolidModel, &result](const Rayd& ray, const PolyMeshIndex& idx)->bool {
		if (_model.isClosed(idx)) {
			auto pFace = _model.getPolygon(idx);
			RayHitd rh;
			if (pFace->intersect(ray, rh)) {
				if (fabs(rh.dist) < _tol) {
					if (_vert.isOnSymPlane(_params)) {
						return true; // skip 
					}
					_vert.setTopologyState(TOPST_INTERSECTING);
					return false;
				}
				if (rh.dist > 0) {
					auto& norm = pFace->calUnitNormal();
					auto dp = norm.dot(ray._dir);
					// TODO, it may be advisable to discard any raycast where the ray is close to parallel to a hit face.
					// This condition produces a lot of singularities.
					if (fabs(dp) < 0.1) {
						result = false;
					} else {
						bool positiveCrossing = dp >= 0;
						bool rayStartsOnSymPlaneAndAimedInwards = rh.dist < _tol && pFace->isOnSymmetryPlane() && !positiveCrossing;
						if (!rayStartsOnSymPlaneAndAimedInwards)
							hitsOnSolidModel.push_back(SolidHitRec(rh.dist, positiveCrossing ? 1 : 0, idx));
					}
				}
			}
		}
		return result;
	}, _tol);

	if (hitsOnSolidModel.size() > 2) {
		sort(hitsOnSolidModel.begin(), hitsOnSolidModel.end());
		size_t i = 0;
		size_t j = 1;
		while (j < hitsOnSolidModel.size()) {
			const auto& a = hitsOnSolidModel[i];
			const auto& b = hitsOnSolidModel[j];
			double dist = b._dist - a._dist;
			if (fabs(dist) < _tol) {
				assert(a._positiveCrossing == b._positiveCrossing);
				auto iter = hitsOnSolidModel.begin();
				hitsOnSolidModel.erase(iter + j);
			} else {
				i++;
				j++;
			}
		}
	}
	return result;
}

