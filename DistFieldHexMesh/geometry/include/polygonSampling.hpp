
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

#include <polygon.h>

namespace DFHM
{

template<class FUNC>
void Polygon::sampleSpacedPoints(double gridSpacing, const FUNC& func) const
{
#if 0
	static std::mutex mut;
	std::lock_guard<std::mutex> lg(mut);
#endif
	auto& vertIds = getNonColinearVertexIds();
	if (vertIds.size() == 4) {
		sampleSpacedPointsQuad(vertIds, gridSpacing, func);
	}
	else {
		sampleSpacedPointsGeneral(gridSpacing, func);
	}
}

template<class FUNC>
void Polygon::sampleSpacedPointsQuad(const MTC::vector<Index3DId>& vertIds, double gridSpacing, const FUNC& func) const
{
	const Vector3d* pts[] = {
		&getVertexPoint(vertIds[0]),
		&getVertexPoint(vertIds[1]),
		&getVertexPoint(vertIds[2]),
		&getVertexPoint(vertIds[3]),
	};

	double width = 0.5 * ((*pts[1] - *pts[0]).norm() + (*pts[2] - *pts[3]).norm());
	double height = 0.5 * ((*pts[3] - *pts[0]).norm() + (*pts[2] - *pts[1]).norm());

	size_t numX = (size_t)(width / gridSpacing + 0.5);
	size_t numY = (size_t)(height / gridSpacing + 0.5);
	Vector3d pt;
	if (numX < 2 && numY < 2) {
		func(calCentroid());
	}
	else if (numX < 2) {
		double t = 0.5;
		for (size_t j = 0; j < numY; j++) {
			double u = j / (numY - 1.0);
			pt = BI_LERP(*pts[0], *pts[1], *pts[2], *pts[3], t, u);
			func(pt);
		}
	}
	else if (numY < 2) {
		double u = 0.5;
		for (size_t i = 0; i < numX; i++) {
			double t = i / (numX - 1.0);
			pt = BI_LERP(*pts[0], *pts[1], *pts[2], *pts[3], t, u);
			func(pt);
		}
	}
	else {

		for (size_t i = 0; i < numX; i++) {
			double t = i / (numX - 1.0);
			Vector3d tpt0 = LERP(*pts[0], *pts[1], t);
			Vector3d tpt1 = LERP(*pts[3], *pts[2], t);
			for (size_t j = 0; j < numY; j++) {
				double u = j / (numY - 1.0);
				auto len1 = (tpt1 - tpt0).norm();
				size_t numY = (size_t)(len1 / gridSpacing + 0.5);
				if (numY < 2) {
					Vector3d pt = (tpt1 - tpt0) * 0.5;
					func(pt);
				}
				else {
					for (size_t j = 0; j < numY; j++) {
						double u = j / (numY - 1.0);
						Vector3d pt = LERP(tpt0, tpt1, u);
						func(pt);
					}
				}

			}
		}
	}
}

template<class FUNC>
void Polygon::sampleSpacedPointsGeneral(double gridSpacing, const FUNC& func) const
{
	std::set<Vector3<int64_t>> pointToIndexSet;
	iterateTrianglePts([gridSpacing, &func, &pointToIndexSet](const Vector3d& pt0, const Vector3d& pt1, const Vector3d& pt2)->bool {
		const Vector3d* pts[] = { &pt0, &pt1, &pt2 };
		double minLen = DBL_MAX, len;
		int apexIdx = -1;
		for (int i = 0; i < 3; i++) {
			int j = (i + 1) % 3;
			Vector3d v = *pts[j] - *pts[i];
			len = v.norm();
			if (len < minLen) {
				minLen = len;
				apexIdx = (j + 1) % 3; // The index opposite the shortest leg
			}
		}

		const auto& origin = *pts[apexIdx];
		Vector3d midShort = (*pts[(apexIdx + 1) % 3] + *pts[(apexIdx + 2) % 3]) * 0.5;
		len = (midShort - origin).norm();
		size_t numX = (size_t)(len / gridSpacing + 0.5);

		if (numX < 2) {
			Vector3d ctr = (pt0 + pt1 + pt2) / 3;
			auto ctrI = Vertex::scaleToSearch(ctr);
			if (pointToIndexSet.count(ctrI) == 0) {
				pointToIndexSet.insert(ctrI);
				func(ctr);
			}
		}
		else if (minLen < gridSpacing) {
			for (size_t i = 0; i < numX; i++) {
				double t = i / (numX - 1.0);
				Vector3d pt = LERP(origin, midShort, t);
				auto ptI = Vertex::scaleToSearch(pt);
				if (pointToIndexSet.count(ptI) == 0) {
					pointToIndexSet.insert(ptI);
					func(pt);
				}
			}
		}
		else {
			for (size_t i = 0; i < numX; i++) {
				double t = i / (numX - 1.0);
				Vector3d tpt0 = LERP(*pts[0], *pts[1], t);
				Vector3d tpt1 = LERP(*pts[0], *pts[2], t);
				auto len1 = (tpt1 - tpt0).norm();
				size_t numY = (size_t)(len1 / gridSpacing + 0.5);
				if (numY < 2) {
					Vector3d pt = LERP(tpt0, tpt1, 0.5);
					auto ptI = Vertex::scaleToSearch(pt);
					if (pointToIndexSet.count(ptI) == 0) {
						pointToIndexSet.insert(ptI);
						func(pt);
					}
				}
				else {
					for (size_t j = 0; j < numY; j++) {
						double u = j / (numY - 1.0);
						Vector3d pt = LERP(tpt0, tpt1, u);
						auto ptI = Vertex::scaleToSearch(pt);
						if (pointToIndexSet.count(ptI) == 0) {
							pointToIndexSet.insert(ptI);
							func(pt);
						}
					}
				}
			}

		}

		return true;
	});
}

template<class FUNC>
void Polygon::sampleSpacedQuads(double gridSpacing, const FUNC& func) const
{
#if 0
	static std::mutex mut;
	std::lock_guard<std::mutex> lg(mut);
#endif
	auto& vertIds = getNonColinearVertexIds();
	if (vertIds.size() == 4) {
		sampleSpacedQuadsQuad(vertIds, gridSpacing, func);
	} else {
//		sampleSpacedQuadsGeneral(gridSpacing, func);
	}
}

template<class FUNC>
void Polygon::sampleSpacedQuadsQuad(const MTC::vector<Index3DId>& vertIds, double gridSpacing, const FUNC& func) const
{
	const Vector3d pts[] = {
		getVertexPoint(vertIds[0]),
		getVertexPoint(vertIds[1]),
		getVertexPoint(vertIds[2]),
		getVertexPoint(vertIds[3]),
	};

	double width = 0.5 * ((pts[1] - pts[0]).norm() + (pts[2] - pts[3]).norm());
	double height = 0.5 * ((pts[3] - pts[0]).norm() + (pts[2] - pts[1]).norm());

	size_t numX = (size_t)(width / gridSpacing + 0.5);
	size_t numY = (size_t)(height / gridSpacing + 0.5);
	Vector3d gridPts[4];
	Vector2d tu0(0, 0), tu1(1, 1);

	if (numX < 2 && numY < 2) {
		func(4, pts);
	} else if (numX < 2) {
		tu0[0] = 0;
		tu1[0] = 1;
		for (size_t j = 0; j < numY; j++) {
			tu0[1] = j / (double)numY;
			tu1[1] = (j + 1) / (double)numY;
			gridPts[0] = BI_LERP(pts[0], pts[1], pts[2], pts[3], tu0[0], tu0[1]);
			gridPts[1] = BI_LERP(pts[0], pts[1], pts[2], pts[3], tu1[0], tu0[1]);
			gridPts[2] = BI_LERP(pts[0], pts[1], pts[2], pts[3], tu1[0], tu1[1]);
			gridPts[3] = BI_LERP(pts[0], pts[1], pts[2], pts[3], tu0[0], tu1[1]);
			func(4, gridPts);
		}
	} else if (numY < 2) {
		tu0[1] = 0;
		tu1[1] = 1;
		for (size_t i = 0; i < numX; i++) {
			tu0[0] = i / (double)numX;
			tu1[0] = (i + 1) / (double)numX;
			gridPts[0] = BI_LERP(pts[0], pts[1], pts[2], pts[3], tu0[0], tu0[1]);
			gridPts[1] = BI_LERP(pts[0], pts[1], pts[2], pts[3], tu1[0], tu0[1]);
			gridPts[2] = BI_LERP(pts[0], pts[1], pts[2], pts[3], tu1[0], tu1[1]);
			gridPts[3] = BI_LERP(pts[0], pts[1], pts[2], pts[3], tu0[0], tu1[1]);
			func(4, gridPts);
		}
	} else {
		for (size_t i = 0; i < numX; i++) {
			tu0[0] = i / (double)numX;
			tu1[0] = (i + 1) / (double)numX;

			for (size_t j = 0; j < numY; j++) {
				tu0[1] = j / (double)numY;
				tu1[1] = (j + 1) / (double)numY;
				gridPts[0] = BI_LERP(pts[0], pts[1], pts[2], pts[3], tu0[0], tu0[1]);
				gridPts[1] = BI_LERP(pts[0], pts[1], pts[2], pts[3], tu1[0], tu0[1]);
				gridPts[2] = BI_LERP(pts[0], pts[1], pts[2], pts[3], tu1[0], tu1[1]);
				gridPts[3] = BI_LERP(pts[0], pts[1], pts[2], pts[3], tu0[0], tu1[1]);
				func(4, gridPts);
			}
		}
	}
}

template<class FUNC>
void Polygon::sampleSpacedQuadsGeneral(double gridSpacing, const FUNC& func) const
{
	std::set<Vector3<int64_t>> pointToIndexSet;
	iterateTrianglePts([gridSpacing, &func, &pointToIndexSet](const Vector3d& pt0, const Vector3d& pt1, const Vector3d& pt2)->bool {
		const Vector3d* pts[] = { &pt0, &pt1, &pt2 };
		double minLen = DBL_MAX, len;
		int apexIdx = -1;
		for (int i = 0; i < 3; i++) {
			int j = (i + 1) % 3;
			Vector3d v = *pts[j] - *pts[i];
			len = v.norm();
			if (len < minLen) {
				minLen = len;
				apexIdx = (j + 1) % 3; // The index opposite the shortest leg
			}
		}

		const auto& origin = *pts[apexIdx];
		Vector3d midShort = (*pts[(apexIdx + 1) % 3] + *pts[(apexIdx + 2) % 3]) * 0.5;
		len = (midShort - origin).norm();
		size_t numX = (size_t)(len / gridSpacing + 0.5);

		if (numX < 2) {
			Vector3d ctr = (pt0 + pt1 + pt2) / 3;
			auto ctrI = Vertex::scaleToSearch(ctr);
			if (pointToIndexSet.count(ctrI) == 0) {
				pointToIndexSet.insert(ctrI);
				func(ctr);
			}
		}
		else if (minLen < gridSpacing) {
			for (size_t i = 0; i < numX; i++) {
				double t = i / (numX - 1.0);
				Vector3d pt = LERP(origin, midShort, t);
				auto ptI = Vertex::scaleToSearch(pt);
				if (pointToIndexSet.count(ptI) == 0) {
					pointToIndexSet.insert(ptI);
					func(pt);
				}
			}
		}
		else {
			for (size_t i = 0; i < numX; i++) {
				double t = i / (numX - 1.0);
				Vector3d tpt0 = LERP(*pts[0], *pts[1], t);
				Vector3d tpt1 = LERP(*pts[0], *pts[2], t);
				auto len1 = (tpt1 - tpt0).norm();
				size_t numY = (size_t)(len1 / gridSpacing + 0.5);
				if (numY < 2) {
					Vector3d pt = LERP(tpt0, tpt1, 0.5);
					auto ptI = Vertex::scaleToSearch(pt);
					if (pointToIndexSet.count(ptI) == 0) {
						pointToIndexSet.insert(ptI);
						func(pt);
					}
				}
				else {
					for (size_t j = 0; j < numY; j++) {
						double u = j / (numY - 1.0);
						Vector3d pt = LERP(tpt0, tpt1, u);
						auto ptI = Vertex::scaleToSearch(pt);
						if (pointToIndexSet.count(ptI) == 0) {
							pointToIndexSet.insert(ptI);
							func(pt);
						}
					}
				}
			}

		}

		return true;
		});
}

}