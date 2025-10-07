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
#include <tm_spatialSearch.h>
#include <polyMeshIndex.h>
#include <triMeshTypeDefs.h>


namespace DFHM {

class Vertex;
class Model;
class SplittingParams;

class RayModelIntersector {
public:
	struct SolidHitRec {
		SolidHitRec(double dist, int positiveCrossing, const PolyMeshIndex& idx);
		bool operator<(const SolidHitRec& rhs) const;

		double _dist;
		int _positiveCrossing;
		PolyMeshIndex _polyMeshIdx;
	};

public:
	RayModelIntersector(const SplittingParams& params, const Model& model, Vertex& vert);

	void castRay(const Rayd& ray, std::vector<SolidHitRec>& hitsOnSolidModel);

private:
	Vertex& _vert;

	const double _tol;
	const SplittingParams& _params;
	const Model& _model;
	std::shared_ptr<const PolyMeshSearchTree> _pTree;

};

inline RayModelIntersector::SolidHitRec::SolidHitRec(double dist, int positiveCrossing, const PolyMeshIndex& idx)
	: _dist(dist)
	, _positiveCrossing(positiveCrossing)
	, _polyMeshIdx(idx)
{
}

inline bool RayModelIntersector::SolidHitRec::operator<(const SolidHitRec& rhs) const
{
	return (_dist != rhs._dist) ? _dist < rhs._dist : _positiveCrossing < rhs._positiveCrossing;
}


}