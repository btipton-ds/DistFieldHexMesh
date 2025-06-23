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
#include <fstream>
#include <splitter2D.h>
#include <tm_lineSegment.hpp>
#include <tolerances.h>
#include <vertex.h>
#include <splitParams.h>
#include <polygon.h>

using namespace std;
using namespace DFHM;

Splitter2D::Splitter2D(const Planed& plane)
	: _plane(plane)
{
	_xAxis = _plane.getXRef();
	_yAxis = _plane.getNormal().cross(_xAxis);
}

Splitter2D::Splitter2D(const MTC::vector<Vector3d>& polyPoints)
{
	initFromPoints(polyPoints);
}

Splitter2D::Splitter2D(const Polygon& face)
{
	const auto& vertIds = face.getNonColinearVertexIds();
	vector<Vector3d> polyPts;
	for (const auto& id : vertIds) {
		polyPts.push_back(face.getVertexPoint(id));
	}

#ifdef _DEBUG
	if (Index3DId(0, 0, 0, 106855) == face.getId()) {
		int dbgBreak = 1;
	}
#endif
	initFromPoints(polyPts);
}

bool Splitter2D::pointInPolygon(const Vector2d& pt, const vector<Vector2d>& poly2D) const
{
	const auto tol = Tolerance::sameDistTol();

	for (size_t i = 0; i < poly2D.size(); i++) {
		size_t j = (i + 1) % poly2D.size();
		auto& pt0 = poly2D[i];
		auto& pt1 = poly2D[j];

		Vector2d legVec = pt1 - pt0;
		legVec.normalize();
		Vector2d legPerp(-legVec[1], legVec[0]);
		Vector2d v = pt - pt0;
		double dist = v.dot(legPerp);
		if (dist < -tol)
			return false;
	}

	return true;
}

size_t Splitter2D::findPtIndex(const Vector2d& pt) const
{
	auto iter = _ptToIndexMap.find(FixedPt::fromDbl(pt[0]));
	if (iter != _ptToIndexMap.end()) {
		auto& subMap = iter->second;
		auto iter2 = subMap.find(FixedPt::fromDbl(pt[1]));
		if (iter2 != subMap.end())
			return iter2->second;
	}
	return -1;
}

size_t Splitter2D::addPoint(const Vector2d& pt)
{
	auto valXi = FixedPt::fromDbl(pt[0]);
	auto valYi = FixedPt::fromDbl(pt[1]);

	auto iter = _ptToIndexMap.find(valXi);
	if (iter == _ptToIndexMap.end())
		iter = _ptToIndexMap.insert(make_pair(valXi, std::map<FIXED_PT_SCALAR_TYPE, size_t>())).first;
	auto& subMap = iter->second;
	auto iter2 = subMap.find(valYi);
	if (iter2 == subMap.end()) {
		size_t idx = _pts.size();
		_pts.push_back(pt);
		iter2 = subMap.insert(make_pair(valYi, idx)).first;
	}
	return iter2->second;
}

void Splitter2D::initFromPoints(const MTC::vector<Vector3d>& polyPoints)
{
	if (polyPoints.size() < 3) {
		throw runtime_error("Splitter2D::Splitter2D(). Insufficient points to define plane");
	}

	Vector3d origin(0, 0, 0);
	if (polyPoints.size() == 4) {
		origin = BI_LERP(polyPoints, 0.5, 0.5);
		_xAxis = BI_LERP(polyPoints, 1.0, 0.5) - BI_LERP(polyPoints, 0.0, 0.5);
		_xAxis.normalize();
		Vector3d v = polyPoints[2] - polyPoints[0];
		Vector3d n = _xAxis.cross(v);
		_plane = Planed(origin, n);
		_plane.setXRef(_xAxis);
		_yAxis = _plane.getNormal().cross(_xAxis);
	} else {
		for (const auto& pt : polyPoints)
			origin += pt;
		origin /= polyPoints.size();

		const auto& pt0 = polyPoints[0];
		const auto& pt1 = polyPoints[1];
		const auto& pt2 = polyPoints[2];

		Vector3d v0 = pt0 - pt1;
		_xAxis = -v0.normalized();
		Vector3d v1 = pt2 - pt1;
		Vector3d n = v1.cross(v0);
		_plane = Planed(origin, n);
		_plane.setXRef(_xAxis);
		_yAxis = _plane.getNormal().cross(_xAxis);
	}

	assert(fabs(_plane.getNormal().dot(_xAxis)) < Tolerance::paramTol());
	assert(fabs(_plane.getNormal().dot(_yAxis)) < Tolerance::paramTol());
	assert(fabs(_xAxis.dot(_yAxis)) < Tolerance::paramTol());

	const auto tol = .1; // Extremely loose tolerance since these points define the plane.
	for (size_t i = 0; i < polyPoints.size(); i++) {
		size_t j = (i + 1) % polyPoints.size();
		Vector2d p0, p1;
		if (project(polyPoints[i], p0, tol) && project(polyPoints[j], p1, tol)) {
			size_t idx0 = addPoint(p0);
			size_t idx1 = addPoint(p1);
			Edge2D e(idx0, idx1);

			_boundaryIndices.push_back(idx0);
			_boundaryEdges.insert(e);
		} else {
			assert(!"project(polyPoints) failed");
		}

	}
}

void Splitter2D::addEdge(const Edge2D& edge, bool split)
{
	if (edge[0] == edge[1])
		return;

	if (!split) {
		_edges.insert(edge);
		return;
	}
	splitExisting(edge);

	set<Edge2D> subs;
	splitWithAllPoints(edge, subs);
#if 0
	for (const auto& e1 : _edges) {
		split(edge, e1, subs);
	}
#endif

	if (subs.empty()) {
		if (edge[0] != edge[1])
			_edges.insert(edge);
	}
	else {
		for (auto& s : subs) {
			if (s[0] != s[1])
				_edges.insert(s);
		}
	}
}

void Splitter2D::addEdge(const Vector2d& pt0, const Vector2d& pt1, bool split)
{
	size_t idx0 = addPoint(pt0);
	size_t idx1 = addPoint(pt1);
	Edge2D edge(idx0, idx1);
	addEdge(edge, split);
}

void Splitter2D::add3DEdge(const Vector3d& pt3D0, const Vector3d& pt3D1)
{
	const auto tol = Tolerance::sameDistTol();
	Vector2d p0, p1;
	if (project(pt3D0, p0, tol) && project(pt3D1, p1, tol)) {
		addEdge(p0, p1, true);
	}
}

void Splitter2D::add3DTriEdges(const Vector3d pts[3], bool split)
{
	const Vector3d* pPts[] = {
		&pts[0],
		&pts[1],
		&pts[2],
	};

	return add3DTriEdges(pPts, split);
}

void Splitter2D::add3DTriEdges(const Vector3d* pts[3], bool split)
{
	Vector2d pt0, pt1;
	if (calIntersectionTriPts(pts, pt0, pt1)) {
		addEdge(pt0, pt1, split);
	}
}

void Splitter2D::addFaceEdges(const MTC::vector<const Vector3d*>& polyPoints, bool split) {
	const auto distTol = Tolerance::sameDistTol();
	const auto distTolSqr = Tolerance::sameDistTolSqr();
	const auto tol = Tolerance::paramTol();

	vector<Vector2d> iPts;
	set<size_t> usedIndices;
	for (size_t i = 0; i < polyPoints.size(); i++) {
		size_t j = (i + 1) % polyPoints.size();
		size_t k = (i + 2) % polyPoints.size();

		Vector2d pt02d, pt12d;
		auto& pt0 = *polyPoints[i];
		auto& pt1 = *polyPoints[j];
#if 1
		auto& pt2 = *polyPoints[k];

		auto dist0 = _plane.distanceToPoint(pt0, false);
		auto dist1 = _plane.distanceToPoint(pt1, false);
		auto dist2 = _plane.distanceToPoint(pt2, false);

		if (fabs(dist0) < distTol && fabs(dist1) < distTol) {
			// The entire segment lies in the plane

			if (!usedIndices.contains(i) && project(pt0, pt02d, distTol)) {
				usedIndices.insert(i);
				iPts.push_back(pt02d);
			}

			if (!usedIndices.contains(j) && project(pt1, pt12d, distTol)) {
				usedIndices.insert(j);
				iPts.push_back(pt12d);
			}
		} else if (dist0 * dist1 < 0) {
			if (!usedIndices.contains(i)) {
				// There is a clear crossing of vertI and vertJ;
				LineSegment_byrefd seg(pt0, pt1);
				RayHitd hp;
				if (_plane.intersectLineSegment(seg, hp, tol)) {
					if (project(hp.hitPt, pt02d, distTol)) {
						usedIndices.insert(i);
						iPts.push_back(pt02d);
					}
				}
			}
		} else if (fabs(dist1) < distTol && dist0 * dist2 < 0 && !usedIndices.contains(j)) {
			// The mid point lies on the plane, but pt0 and pt2 cross
			if (project(pt1, pt12d, distTol)) {
				usedIndices.insert(j);
				iPts.push_back(pt12d);
			}
		}
#else
		if (_plane.isCoincident(pt0, distTol) && _plane.isCoincident(pt1, distTol)) {
			if (project(pt0, pt02d, distTol) && project(pt1, pt12d, distTol)) {
				addEdge(pt02d, pt12d, split);
				continue;
			}
		} else {
			LineSegment_byrefd seg(pt0, pt1);
			RayHitd hp;
			if (_plane.intersectLineSegment(seg, hp, tol)) {
				if (project(hp.hitPt, pt02d, distTol)) {
					iPts.push_back(pt02d);
				}
			}
		}
#endif
	}

	if (iPts.size() < 2)
		return;

	// sort all the points into ascending order along the line of intersection.
	const auto& pt0 = iPts[0];
	Vector2d v;
	double len = -1;
	for (size_t i = 1; i < iPts.size(); i++) {
		v = iPts[i] - pt0;
		len = v.norm();
		if (len > Tolerance::paramTol()) {
			v /= len;
			break;
		}
	}

	if (len < Tolerance::sameDistTol())
		return;

	if (iPts.size() > 2) {
		sort(iPts.begin(), iPts.end(), [&pt0, &v](const Vector2d& lhs, const Vector2d& rhs)->bool {
			double lhsDist = (lhs - pt0).dot(v);
			double rhsDist = (rhs - pt0).dot(v);
			return lhsDist < rhsDist;
		});

		for (size_t i = iPts.size() - 2; i != -1; i--) {
			size_t j = (i + 1) % iPts.size();
			const auto& pt0 = iPts[i];
			const auto& pt1 = iPts[j];
			auto l = (pt1 - pt0).squaredNorm();
			if (l < Tolerance::sameDistTolSqr()) {
				iPts.erase(iPts.begin() + j);
			}
		}
	}

	if (!iPts.empty() && iPts.size() % 2 == 0) {
		for (size_t i = 0; i < iPts.size() / 2; i++) {
			size_t j = (i + 1) % iPts.size();
			addEdge(iPts[i], iPts[j], split);
		}
	}
}

bool Splitter2D::calIntersectionTriPts(const Vector3d* const* triPts, Vector2d& pt0, Vector2d& pt1) const
{
	const auto tol = Tolerance::sameDistTol();
	const auto tolSqr = tol * tol;

	int numInBounds = 0;
	Vector2d iPts[3];
	int count = 0;
	for (int i = 0; i < 3; i++) {
		int j = (i + 1) % 3;
		LineSegmentd triSeg(*triPts[i], *triPts[j]);
		RayHitd hit;
		if (_plane.intersectLineSegment(triSeg, hit, Tolerance::sameDistTol())) {
			Vector2d pt;
			if (project(hit.hitPt, pt, tol)) {
				iPts[count++] = pt;
			}
		}
	}

	if (count == 2) {
		pt0 = iPts[0];
		pt1 = iPts[1];
		if ((pt1 - pt0).squaredNorm() > tolSqr) {
			LineSegment2d boundarySeg(pt0, pt1);
			if (segIntersectsBoundary(boundarySeg)) {
				return true;
			}
		}
	}

	return false;
}

void Splitter2D::imprint3DPoint(const Vector3d& pt3D0)
{
	const auto tol = Tolerance::sameDistTol();
	Vector2d pt;
	if (!project(pt3D0, pt, tol))
		return;

	auto idx = findPtIndex(pt);
	if (idx != -1)
		return;
	for (const auto& e : _edges) {
		const auto& pt0 = _pts[e[0]];
		const auto& pt1 = _pts[e[1]];
		LineSegment2d seg(pt0, pt1);
		double t;
		if (seg.project(pt, t)) {
			size_t idx0 = e[0];
			size_t idx1 = addPoint(pt);
			size_t idx2 = e[1];
			_edges.erase(e);
			_edges.insert(Edge2D(idx0, idx1));
			_edges.insert(Edge2D(idx1, idx2));
			break;
		}
	}
	idx = addPoint(pt);
}

void Splitter2D::cleanMap(map<size_t, set<size_t>>& map, size_t indexToRemove)
{
	set<size_t> dead;
	for (auto& pair : map) {
		pair.second.erase(indexToRemove);
		if (pair.second.empty())
			dead.insert(pair.first);
	}

	for (size_t idx : dead)
		map.erase(idx);
}

size_t Splitter2D::getFacePoints(vector<vector<Vector3d>>& facePoints)
{
	vector<vector<Vector3d>> discard;
	getLoops(discard, facePoints);
	return facePoints.size();
}

void Splitter2D::getEdgePts(vector<vector<Vector3d>>& edgePts) const
{
	for (const auto& e : _edges) {
		vector<Vector3d> pts;
		for (int i = 0; i < 2; i++) {
			Vector3d pt(unproject(e[i]));
			pts.push_back(pt);
		}
		edgePts.push_back(pts);
	}
}

void Splitter2D::getLoops(vector<vector<Vector3d>>& polylines, vector<vector<Vector3d>>& loops) const
{
	vector<vector<Vector2d>> polylines2D, loops2d;
	getLoops(polylines2D, loops2d);

	for (const auto& pl2 : polylines2D) {
		vector<Vector3d> pl3;
		for (const auto& pt2 : pl2) {
			pl3.push_back(unproject(pt2));
		}
		polylines.push_back(pl3);
	}

	for (const auto& pl2 : loops2d) {
		vector<Vector3d> pl3;
		for (const auto& pt2 : pl2) {
			pl3.push_back(unproject(pt2));
		}
		loops.push_back(pl3);
	}
}

void Splitter2D::createPointPointMap(POINT_MAP_TYPE& ptMap) const
{
	ptMap.resize(_pts.size());
	for (const auto& e : _edges) {
		ptMap[e[0]].insert(e[1]);
		ptMap[e[1]].insert(e[0]);
	}
}

void Splitter2D::createPointPointMap(std::map<size_t, std::set<size_t>>& pointToPointsMap) const {
	for (const auto& edge : _edges) {
		auto iter = pointToPointsMap.find(edge[0]);
		if (iter == pointToPointsMap.end())
			iter = pointToPointsMap.insert(make_pair(edge[0], set<size_t>())).first;
		iter->second.insert(edge[1]);

		iter = pointToPointsMap.find(edge[1]);
		if (iter == pointToPointsMap.end())
			iter = pointToPointsMap.insert(make_pair(edge[1], set<size_t>())).first;
		iter->second.insert(edge[0]);
	}
}

void Splitter2D::createEdgeUsageMap(const POINT_MAP_TYPE& ptMap, map<Edge2D, size_t>& edgeUsage) const
{
	edgeUsage.clear();
	for (const auto& e : _edges) {
		const auto& set0 = ptMap[e[0]];
		const auto& set1 = ptMap[e[1]];

		size_t numUsages = 1;
		if (set0.size() > 2 && set1.size() > 2)
			numUsages = 2;
		edgeUsage.insert(make_pair(e, numUsages));
	}
}

void Splitter2D::removePolylineFromMaps(const Polyline& pl, POINT_MAP_TYPE& ptMap, map<Edge2D, size_t>& edgeUsage) const
{
	vector<size_t> indices;
	pl.createVector(indices, vector<Vector2d>());
	size_t num = pl._isClosed ? indices.size() : indices.size() - 1;
	for (size_t i = 0; i < num; i++) {
		size_t j = (i + 1) % indices.size();
		Edge2D e(indices[i], indices[j]);
		auto iter = edgeUsage.find(e);
		if (iter == edgeUsage.end())
			continue;

		iter->second--;
		if (iter->second == 0) {
			// Edge is no longer in use

			auto& conP0 = ptMap[e[0]];
			conP0.erase(e[1]);

			auto& conP1 = ptMap[e[1]];
			conP1.erase(e[0]);

			edgeUsage.erase(iter);
		}
	}
}

size_t Splitter2D::getLoopSeedIndex(const POINT_MAP_TYPE& ptMap) const
{
	size_t maxConnections = 0, result = -1;
	for (size_t i = 0; i < ptMap.size(); i++) {
		const auto& conP = ptMap[i];
		if (conP.size() > maxConnections) {
			maxConnections = conP.size();
			result = i;
		}
	}

	return result;
}

size_t Splitter2D::getSpurSeedIndex(const POINT_MAP_TYPE& ptMap) const
{
	size_t result = -1;
	for (size_t i = 0; i < ptMap.size(); i++) {
		const auto& conP = ptMap[i];
		if (conP.size() == 1) {
			result = i;
			break;
		}
	}

	return result;
}

void Splitter2D::getLoops(vector<vector<Vector2d>>& polylines, vector<vector<Vector2d>>& loops) const
{
	vector<Polyline> mixedLoops;
	getPolylines(mixedLoops);
	for (const auto& loop : mixedLoops) {
		vector<Vector2d> pts;
		for (const size_t idx : loop)
			pts.push_back(_pts[idx]);

		if (loop._isClosed) {
			assert(loop.size() >= 3);
			// To be fully deterministic, make the new face normal in the same direction as the splitter plane.
			Vector2d v0 = pts[1] - pts[0];
			Vector2d v1 = pts[2] - pts[1];
			double cp = v0[0] * v1[1] - v0[1] * v1[0];
			if (cp < 0)
				reverse(pts.begin(), pts.end());

			loops.push_back(pts);
		} else {
			polylines.push_back(pts);
		}
	}
}

size_t Splitter2D::getPolylines(vector<Polyline>& polylines) const
{
	POINT_MAP_TYPE ptMap;
	map<Edge2D, size_t> edgeUsage;
	createPointPointMap(ptMap);
	if (ptMap.empty())
		return 0;

	createEdgeUsageMap(ptMap, edgeUsage);

	vector<Polyline> pls;
	while (createPolylines(ptMap, edgeUsage, pls) > 0) {
		// We're getting spurs that duplicate spurs which were already created.
		// Need to check for spurs every time a loop is removed in case it leaves and orphan edge.
		// The duplicate remover should deal with it.
		polylines.insert(polylines.end(), pls.begin(), pls.end());
		pls.clear();
	}

	return polylines.size();
}

size_t Splitter2D::getCurvatures(const SplittingParams& params, vector<double>& curvatures) const
{
	const double sharpAngleRadians = params.getSharpAngleRadians();
	const double sharpRadius = params.sharpRadius;
	const double minRatio = 1 / 25.0;

	if (_edges.empty())
		return 0;

	curvatures.clear();
	vector<Polyline> pls;
	if (getPolylines(pls)) {
		size_t max = 0;
		for (const auto& pl : pls) {
			if (pl.size() > max)
				max = pl.size();
		}
#if 0
		if (max > 50) {
			static mutex mut;
			lock_guard lg(mut);
			writeObj("D:/DarkSky/Projects/output/objs/getCurvatures");
			writePolylinesObj("D:/DarkSky/Projects/output/objs/getCurvatures_pls");
			int dbgBreak = 1;
		}
#endif
		for (const auto& pl : pls) {
			vector<size_t> indices;
			size_t numIndices = pl.createVector(indices, _pts);
			size_t start = 0, stop = numIndices;
			if (!pl._isClosed) {
				start = 1;
				stop = numIndices - 1;
				if (insideBoundary(_pts[indices.front()])) {
					curvatures.push_back(1 / sharpRadius);
				}
			}

			for (size_t j = start; j < stop; j++) {
				size_t i = (j + numIndices - 1) % numIndices;
				size_t k = (j + 1) % numIndices;

				size_t idx0 = indices[i];
				size_t idx1 = indices[j];
				size_t idx2 = indices[k];

				const auto& pt0 = _pts[idx0];
				const auto& pt1 = _pts[idx1];
				const auto& pt2 = _pts[idx2];
				if (!insideBoundary(pt1))
					continue;

				if (isColinear(idx0, idx1, idx2)) {
					curvatures.push_back(0);
					continue;
				}

				Vector2d v0 = (pt1 - pt0);
				auto v1 = (pt2 - pt1);
				auto l0 = v0.norm();
				auto l1 = v1.norm();
				if (l0 < Tolerance::paramTol() || l1 < Tolerance::paramTol()) {
					continue;
				}

				v0 /= l0;
				v1 /= l1;
				double cp = fabs(v0[0] * v1[1] - v0[1] * v1[0]);
				if (cp < Tolerance::paramTol()) {
					curvatures.push_back(0);
					continue;
				}
				double radius = 0; // Zero curvature

				Vector2d perpV0 = Vector2d(-v0[1], v0[0]);
				Vector2d perpV1 = Vector2d(-v1[1], v1[0]);

				double cosTheta = v1.dot(v0);
				double sinTheta = v1.dot(perpV0);
				double theta = atan2(sinTheta, cosTheta);
				if (fabs(theta) > sharpAngleRadians) {
					auto thetaDeg = theta * 180 / M_PI;
					radius = sharpRadius;
				}
				else {
					auto mid0 = (pt0 + pt1) * 0.5;
					auto mid1 = (pt1 + pt2) * 0.5;
					LineSegment2d seg0(mid0, perpV0);
					LineSegment2d seg1(mid1, perpV1);
					double t;
					if (seg0.intersectionInBounds(seg1, t)) {
						Vector2d pt = seg0.interpolate(t);
						radius = (pt0 - pt).norm();
						if (radius < sharpRadius) {
							radius = sharpRadius;
						}
					}
				}

				if (radius > 0) {
					curvatures.push_back(1 / radius);
				}
			}

			if (!pl._isClosed) {
				if (insideBoundary(_pts[indices.back()])) {
					curvatures.push_back(1 / sharpRadius);
				}
			}
		}
	}

	if (!curvatures.empty()) {
		int dbgBreak = 1;
	}

	return curvatures.size();
}

size_t Splitter2D::findCurvaturesInPolygon(const std::vector<Vector3d>& polygon, std::vector<double>& curvatures) const
{
	const auto tol = Tolerance::sameDistTol();

	if (_curvatures.empty())
		return 0;

	vector<Vector2d> poly2D;

	double xMin = DBL_MAX, xMax = -DBL_MAX;
	double yMin = DBL_MAX, yMax = -DBL_MAX;

	for (auto& pt : polygon) {
		Vector2d projPt;
		auto success = project(pt, projPt, tol);
		assert(success);

		if (projPt[0] < xMin)
			xMin = projPt[0];
		if (projPt[0] > xMax)
			xMax = projPt[0];

		if (projPt[1] < yMin)
			yMin = projPt[1];
		if (projPt[1] > yMax)
			yMax = projPt[1];

		poly2D.push_back(projPt);
	}

	auto iterXBegin = _ptToIndexMap.lower_bound(FixedPt::fromDbl(xMin - tol));
	auto iterXEnd = _ptToIndexMap.upper_bound(FixedPt::fromDbl(xMax + tol));

	vector<size_t> tmpIndices;
	for (auto iterX = iterXBegin; iterX != iterXEnd; iterX++) {
		auto& subMap = iterX->second;

		auto iterYBegin = subMap.lower_bound(FixedPt::fromDbl(yMin - tol));
		auto iterYEnd = subMap.upper_bound(FixedPt::fromDbl(yMax + tol));

		for (auto iterY = iterYBegin; iterY != iterYEnd; iterY++) {
			size_t idx = iterY->second;
			if (_curvatures[idx] > 0) {
				tmpIndices.push_back(idx);
			}
		}
	}

	for (const size_t idx : tmpIndices) {
		if (pointInPolygon(_pts[idx], poly2D)) {
			auto curv = _curvatures[idx];
			if (curv > 0)
				curvatures.push_back(curv);
		}
	}
	

	return curvatures.size();
}

void Splitter2D::initCurvatures(const SplittingParams& params)
{
	if (!_pts.empty() && _curvatureEdges.empty()) {
		map<size_t, set<size_t>> pointToPointsMap;
		createPointPointMap(pointToPointsMap);

		calCurvaturesAndRadPoints(params, pointToPointsMap);

		set<size_t> badIndices;
		const double maxCurvatureRatio = 0.04;
		calBadCurvatureIndices(pointToPointsMap, maxCurvatureRatio, badIndices);
#if 1
		for (size_t i = 0; i < 3; i++) {
			if (badIndices.empty())
				break;

			removedBadIndices(badIndices, pointToPointsMap);
			calCurvaturesAndRadPoints(params, pointToPointsMap);

			calBadCurvatureIndices(pointToPointsMap, maxCurvatureRatio, badIndices);
		}
#endif
		for (const auto& pair : pointToPointsMap) {
			size_t idx0 = pair.first;
			for (size_t idx1 : pair.second) {
				_curvatureEdges.insert(Edge2D(idx0, idx1));
			}
		}
	}
}

void Splitter2D::getPointCurvatures(const SplittingParams& params, std::vector<Vector3d>& points, std::vector<double>& curvatures,
	std::vector<Vector3d>& radiusSegs, std::vector<double>& radiusCurvatures)
{
	initCurvatures(params);
	// Don't clear points etc. because the caller accumulates them
	set<size_t> usedPtIndices;
	for (const auto& e : _curvatureEdges) {
		size_t idx0 = e[0];
		size_t idx1 = e[1];

		usedPtIndices.insert(idx0);
		usedPtIndices.insert(idx1);

		points.push_back(unproject(idx0));
		points.push_back(unproject(idx1));

		curvatures.push_back(_curvatures[idx0]);
		curvatures.push_back(_curvatures[idx1]);
	}

	for (size_t ptIdx : usedPtIndices) {
		auto v = _radiusPts[ptIdx] - _pts[ptIdx];
		auto l = v.norm();
		if (l > 10 * Tolerance::sameDistTol()) {
			radiusSegs.push_back(unproject(_pts[ptIdx]));
			radiusCurvatures.push_back(_curvatures[ptIdx]);

			radiusSegs.push_back(unproject(_radiusPts[ptIdx]));
			radiusCurvatures.push_back(_curvatures[ptIdx]);
		}
	}
}

void Splitter2D::calCurvaturesAndRadPoints(const SplittingParams& params, std::map<size_t, std::set<size_t>>& pointToPointsMap)
{
	const auto tol = Tolerance::sameDistTol();
	const auto assertTol = 10 * tol;
	const auto sharpRadius = params.sharpRadius;
	const auto sharpAngleRadians = params.getSharpAngleRadians();

	_curvatures.resize(_pts.size());
	_radiusPts.resize(_pts.size());
	for (size_t i = 0; i < _pts.size(); i++) {
		_curvatures[i] = 0;
		_radiusPts[i] = _pts[i];
	}

	for (const auto& pair : pointToPointsMap) {
		size_t ctrIdx = pair.first;
		_radiusPts[ctrIdx] = _pts[ctrIdx];
		const auto& connectedIndices = pair.second;
		if (connectedIndices.size() != 2) {
			_curvatures[ctrIdx] = 1 / sharpRadius;
		}
		else {
			auto iter = connectedIndices.begin();
			size_t idx0 = *iter++;
			size_t idx1 = *iter;

			const auto& pt0 = _pts[idx0];
			const auto& ctrPt = _pts[ctrIdx];
			const auto& pt1 = _pts[idx1];

			if (isColinear(idx0, ctrIdx, idx1)) {
				_curvatures[ctrIdx] = 0;
				continue;
			}

			Vector2d v0 = (ctrPt - pt0);
			auto v1 = (pt1 - ctrPt);
			auto l0 = v0.norm();
			auto l1 = v1.norm();
#if 0
			if (l0 < Tolerance::paramTol() || l1 < Tolerance::paramTol()) {
				_curvatures[ctrIdx] = 0;
				continue;
			}
#endif

			v0 /= l0;
			v1 /= l1;
			double cp = fabs(v0[0] * v1[1] - v0[1] * v1[0]);
			if (cp < Tolerance::paramTol()) {
				_curvatures[ctrIdx] = 0;
				continue;
			}
			if (ctrIdx == 350) {
				int dbgBreak = 1;
			}
			double radius = sharpRadius; // Zero curvature

			auto mid = (pt0 + pt1) * 0.5;
			auto mid0 = (pt0 + ctrPt) * 0.5;
			auto mid1 = (ctrPt + pt1) * 0.5;

			Vector2d vMid = mid - ctrPt;
			Vector2d perpV0 = vMid - vMid.dot(v0) * v0;
			perpV0.normalize();

			Vector2d perpV1 = vMid - vMid.dot(v1) * v1;
			perpV1.normalize();

			double cosTheta = v0.dot(v1);
			double sinTheta = perpV0.dot(v1);
			double theta = atan2(sinTheta, cosTheta);
			if (fabs(theta) < sharpAngleRadians) {
				LineSegment2d seg0(mid0, mid0 + perpV0);
				LineSegment2d seg1(mid1, mid1 + perpV1);
				Vector2d arcCtr;
				if (seg0.intersectRay(seg1, arcCtr)) {
#if 1 && defined(_DEBUG)
					{
						auto dir0 = seg0.dir();
						auto& origin = seg0[0];
						Vector2d v = arcCtr - origin;
						v = v - v.dot(dir0) * dir0;
						auto dist = v.norm();
						assert(dist < assertTol);
					}
					{
						auto dir1 = seg1.dir();
						auto& origin = seg1[0];
						Vector2d v = arcCtr - origin;
						v = v - v.dot(dir1) * dir1;
						auto dist = v.norm();
						assert(dist < assertTol);
					}
#endif
					Vector2d vRad = arcCtr - ctrPt;
					radius = vRad.norm();
					double renderRatio = 1;
					if (radius > 0.25)
						renderRatio = 0.25 / radius;

#if 0
					assert(fabs((arcCtr - pt0).norm() - radius) < assertTol);
					assert(fabs((arcCtr - ctrPt).norm() - radius) < assertTol);
					assert(fabs((arcCtr - pt1).norm() - radius) < assertTol);
#endif
					if (radius < sharpRadius)
						radius = sharpRadius;
					else if (radius > params.maxCuvatureRadius)
						radius = -1;
					else
						_radiusPts[ctrIdx] = ctrPt + vRad * renderRatio;
				} else {
					radius = -1;
				}
			}

			if (radius <= 0) {
				_radiusPts[ctrIdx] = _pts[ctrIdx];
				_curvatures[ctrIdx] = 0;
			} else
				_curvatures[ctrIdx] = 1 / radius;
		}
	}

}

void Splitter2D::calBadCurvatureIndices(const std::map<size_t, std::set<size_t>>& pointToPointsMap, double maxCurvatureRatio, std::set<size_t>& badIndices) const
{
	const double minCurvature = 0.001;
	badIndices.clear();
	for (const auto& pair : pointToPointsMap) {
		size_t ctrIdx = pair.first;
		const auto& connectedIndices = pair.second;
		if (connectedIndices.size() == 2) {
			auto iter = connectedIndices.begin();
			size_t idx0 = *iter++;
			size_t idx1 = *iter;

#if 1
			auto curvCtr = _curvatures[ctrIdx];
			auto curv0 = _curvatures[idx0];
			auto curv1 = _curvatures[idx1];
			if (curvCtr < curv0 && curvCtr < curv1) {
				const auto& pt0 = _pts[idx0];
				const auto& ctrPt = _pts[ctrIdx];
				const auto& pt1 = _pts[idx1];

				Vector2d vCtr = _radiusPts[ctrIdx] - ctrPt;
				Vector2d v0 = _radiusPts[idx0] - pt0;
				Vector2d v1 = _radiusPts[idx1] - pt1;
				if (v0.dot(v1) > 0 && vCtr.dot(v0 + v1) < 0) {
					badIndices.insert(ctrIdx);
					continue;
				}

				auto avgCurv = (curv0 + curv1) / 2;
				if (curvCtr < 0.05 * avgCurv) {
					badIndices.insert(ctrIdx);
					continue;
				}
#if 1
				auto l0 = (ctrPt - pt0).norm();
				auto l1 = (ctrPt - pt1).norm();
				auto l = l0 + l1;
				double ratio;
				if (l0 < l1)
					ratio = l0 / l;
				else
					ratio = l1 / l;
				if (ratio < maxCurvatureRatio)
					badIndices.insert(ctrIdx);
			}
#endif
#else
			const auto& pt0 = _pts[idx0];
			const auto& ctrPt = _pts[ctrIdx];
			const auto& pt1 = _pts[idx1];

			auto l0 = (ctrPt - pt0).norm();
			auto l1 = (ctrPt - pt1).norm();

			auto cur0 = _curvatures[idx0];
			auto curCtr = _curvatures[ctrIdx];
			auto cur1 = _curvatures[idx1];

			if (cur0 > 0 && curCtr > 0 && cur1 > 0) {
				//					auto rad0 = 1 / (cur0 + curCtr);
				//					auto rad1 = 1 / (cur1 + curCtr);
				auto angle0 = l0 * (cur0 + curCtr) / 2;
				auto angle1 = l1 * (cur1 + curCtr) / 2;
				if (2 * angle0 < minArcAngleRadians || 2 * angle1 < minArcAngleRadians) {
					badIndices.insert(ctrIdx);
				}
			}
#endif
		}
	}
}

void Splitter2D::removedBadIndices(const std::set<size_t>& badIndices, std::map<size_t, std::set<size_t>>& pointToPointsMap) const
{
	for (size_t deadIdx : badIndices) {
		auto iterDeadIdx = pointToPointsMap.find(deadIdx);
		auto& connected = iterDeadIdx->second;
		if (connected.size() == 2) {
			auto adjIter = connected.begin();
			size_t adjIdx0 = *adjIter++;
			size_t adjIdx1 = *adjIter;

			auto iter0 = pointToPointsMap.find(adjIdx0);
			auto iter1 = pointToPointsMap.find(adjIdx1);

			iter0->second.erase(deadIdx);
			iter0->second.insert(adjIdx1);

			iter1->second.erase(deadIdx);
			iter1->second.insert(adjIdx0);

			pointToPointsMap.erase(iterDeadIdx);
		}
	}
}

size_t Splitter2D::getGaps(vector<double>& curvatures) const
{
	return 0;
}

bool Splitter2D::intersectsTriPoints(const Vector3d* const* triPts) const
{
	Vector2d iPt0, iPt1;
	if (calIntersectionTriPts(triPts, iPt0, iPt1)) {
		LineSegment2d testSeg(iPt0, iPt1);
		if (insideBoundary(testSeg[0]))
			return true;

		for (size_t i = 0; i < _boundaryIndices.size(); i++) {

			size_t j = (i + 1) % _boundaryIndices.size();
			auto& bPt0 = _pts[_boundaryIndices[i]];
			auto& bPt1 = _pts[_boundaryIndices[j]];

			double t;
			LineSegment2d seg(bPt0, bPt1);
			if (seg.intersectionInBounds(testSeg, t)) {
				return true;
			}
		}
	}

	return false;
}

bool Splitter2D::intersectWithRay(const Rayd& ray, std::vector<LineSegmentd>& segs) const
{
	const auto tol = Tolerance::sameDistTol();
	Vector2d rayOrigin2d, rayPt2d;
	if (!project(ray._origin, rayOrigin2d, tol) || !project(ray._origin + ray._dir, rayPt2d, tol))
		return false;

	LineSegment2d seg2d(rayOrigin2d, rayPt2d);
	set<Vector2d> ptSet;
	for (const auto& e : _boundaryEdges) {
		LineSegment2d eSeg(_pts[e[0]], _pts[e[1]]);
		Vector2d pt;
		if (seg2d.intersectRay(eSeg, pt)) {
			if (insideBoundary(pt)) {
				ptSet.insert(pt);
			}
		}
	}
	
	if (ptSet.empty())
		return false;


	if (ptSet.size() % 2 != 0)
		return false;

	vector<Vector2d> pts(ptSet.begin(), ptSet.end());

	Vector2d dir = (rayPt2d - rayOrigin2d).normalized();
	sort(pts.begin(), pts.end(), [&rayOrigin2d, &rayPt2d, &dir](const Vector2d& lhs, const Vector2d& rhs)->bool {
		auto lhDist = (lhs - rayOrigin2d).dot(dir);
		auto rhDist = (rhs - rayOrigin2d).dot(dir);
		return lhDist < rhDist;
	});

	for (size_t i = 0; i < pts.size() / 2; i++) {
		const auto pt0 = unproject(pts[2 * i]);
		const auto pt1 = unproject(pts[2 * i + 1]);
		LineSegmentd seg(pt0, pt1);
		segs.push_back(seg);
	}
	return true;
}

bool Splitter2D::intersectWithSeg(const LineSegmentd& seg) const
{
	const auto tol = Tolerance::sameDistTol();
	Vector2d pt0, pt1;

	bool projValid = project(seg._pt0, pt0, tol);
	assert (projValid);

	if (insideBoundary(pt0))
		return true;

	projValid = project(seg._pt1, pt1, tol);
	assert(projValid);

	if (insideBoundary(pt1))
		return true;

	LineSegment2d seg2d(pt0, pt1);
	for (const auto& e : _boundaryEdges) {
		LineSegment2d testSeg2d(_pts[e[0]], _pts[e[1]]);
		double t;
		if (testSeg2d.intersectionInBounds(seg2d, t)) {
			return true;
		}
	}
	return false;
}

bool Splitter2D::segIntersectsBoundary(const LineSegment2d& testSeg) const
{
	if (insideBoundary(testSeg[0]) || insideBoundary(testSeg[1])) { // An edge for intersecting triangle does not have all tri points checked. Must check both points
		return true;
	}

	for (int i = 0; i < _boundaryIndices.size(); i++) {
		int j = (i + 1) % _boundaryIndices.size();
		const auto& segPt0 = _pts[_boundaryIndices[i]];
		const auto& segPt1 = _pts[_boundaryIndices[j]];

		LineSegment2d seg(segPt0, segPt1);
		double t;
		if (seg.intersectionInBounds(testSeg, t)) {
			return true;
		}
	}

	return false;
}

void Splitter2D::writeObj(const string& filenameRoot) const
{
	string str = filenameRoot + ".obj";
	ofstream out(str);

	out << "#Vertices " << _pts.size() << "\n";
	for (const auto& pt : _pts) {
		auto pt2 = unproject(pt);
		out << "v " << pt2[0] << " " << pt2[1] << " " << pt2[2] << "\n";
	}

	out << "#Edges " << _edges.size() << "\n";
	for (auto& edge : _edges) {
		out << "l " << (edge[0] + 1) << " " << (edge[1] + 1) << "\n";
	}
}

void Splitter2D::writeBoundaryEdgesObj(const string& filename) const
{
	ofstream out(filename);
	out << "#Vertices " << _pts.size() << "\n";
	for (const auto& pt : _pts) {
		auto pt2 = unproject(pt);
		out << "v " << pt2[0] << " " << pt2[1] << " " << pt2[2] << "\n";
	}

	out << "#Edges " << _boundaryEdges.size() << "\n";
	for (const auto& e : _boundaryEdges) {
		out << "l " << (e[0] + 1) << " " << (e[1] + 1) << "\n";
	}
}

void Splitter2D::writePolylinesObj(const string& filenameRoot) const
{
	vector<Polyline> pls;
	if (getPolylines(pls)) {
		size_t n = 0;
		for (const auto& pl : pls) {
			string str = filenameRoot + to_string(n++) + ".obj";
			ofstream out(str);
			out << "#Vertices " << _pts.size() << "\n";
			for (const auto& pt : _pts) {
				auto pt2 = unproject(pt);
				out << "v " << pt2[0] << " " << pt2[1] << " " << pt2[2] << "\n";
			}

			out << "#Edges " << _edges.size() << "\n";
			vector<size_t> indices;
			pl.createVector(indices, vector<Vector2d>());
			for (size_t i = 0; i < indices.size() - 1; i++) {
				size_t j = (i + 1) % indices.size();
				out << "l " << (indices[i] + 1) << " " << (indices[j] + 1) << "\n";
			}
		}
	}
}

void Splitter2D::removeColinearVertsFromVertexLoop(Polyline& pl) const
{
	const double tol = Tolerance::looseParamTol();
	list<size_t> tmp = pl;
	pl.clear();
	size_t i = -1, j = -1, k = 0;
	for (auto iter = tmp.begin(); iter != tmp.end(); iter++) {
		if (i < tmp.size() && j < tmp.size()) {
			auto& pt0 = _pts[i];
			auto& pt1 = _pts[j];
			auto& pt2 = _pts[k];
			Vector2d v0 = (pt1 - pt0).normalized();
			Vector2d v1 = (pt2 - pt1).normalized();
			auto cp = fabs(v1[0] * v0[1] - v1[1] * v0[0]); // v1.cross(v0).norm();
			if (cp > tol)
				pl.push_back(*iter);
		} else
			pl.push_back(*iter);

		i = j;
		j = k;
		k = *iter;
	}

}

bool Splitter2D::insideBoundary(const vector<Vector2d>& testFacePts) const
{
	for (const auto& testPt : testFacePts) {
		if (!insideBoundary(testPt))
			return false;
	}
	return true;
}
bool Splitter2D::insideBoundary(const Vector2d& testPt) const
{
	// Check if the point lies on the positive side of all edges
	for (size_t i = 0; i < _boundaryIndices.size(); i++) {
		auto& pt0 = _pts[_boundaryIndices[i]];
		Vector2d v = testPt - pt0;
		if (v.squaredNorm() < Tolerance::sameDistTolSqr())
			return true; // Points coincident

		size_t j = (i + 1) % _boundaryIndices.size();
		auto& pt1 = _pts[_boundaryIndices[j]];
		Vector2d xAxis = pt1 - pt0;
		xAxis.normalize();
		Vector2d yAxis(-xAxis[1], xAxis[0]);
		double dist = v.dot(yAxis);
		if (dist < -Tolerance::sameDistTol())
			return false;
	}
	return true;
}

Vector3d Splitter2D::calNormal(size_t idx0, size_t idx1, size_t idx2) const
{
	Vector3d pt0 = unproject(idx0);
	Vector3d pt1 = unproject(idx1);
	Vector3d pt2 = unproject(idx2);

	Vector3d v0 = pt0 - pt1;
	Vector3d v1 = pt2 - pt1;
	return v1.cross(v0);
}

bool Splitter2D::isColinear(size_t i, size_t j, size_t k) const
{
	const auto& pt0 = _pts[i];
	const auto& pt1 = _pts[j];
	const auto& pt2 = _pts[k];

	Vector2d v0 = pt2 - pt0;
	v0.normalize();
	Vector2d v1 = pt1 - pt0;
	double dp = v1.dot(v0);
	v1 = v1 - v0 * dp;
	double distSqr = v1.squaredNorm();
	return distSqr < Tolerance::sameDistTolSqr();
}

Vector2d Splitter2D::calTurningUnitVector(size_t idx0, size_t idx1, size_t idx2) const
{
/*
If the vectors form a straight line, the turning vector is [1,0]
If it turns left, it's (cos(theta), sin(theta)
If it turns right, it's (cos(theta), -sin(theta)
The maximum result[1] is the sharpest left turn, assuming result[0] > 0
*/
	const auto& pt0 = _pts[idx0];
	const auto& pt1 = _pts[idx1];
	const auto& pt2 = _pts[idx2];

	Vector2d xAxis = (pt1 - pt0).normalized();
	Vector2d yAxis(-xAxis[1], xAxis[0]);
	Vector2d v1 = (pt2 - pt1).normalized();
	Vector2d result(v1.dot(xAxis), v1.dot(yAxis));
	result.normalize();
	return result;
}

bool Splitter2D::PolylineNode::sameLoop(const vector<size_t>& A, const vector<size_t>& B)
{
	// test if r is the same as this loop, but in a different orientation
	if (A.size() != B.size())
		return false;

	set<size_t> a, b;
	a.insert(A.begin(), A.end());
	b.insert(B.begin(), B.end());

	bool match = true;
	auto iterA = a.begin();
	auto iterB = b.begin();
	while (iterA != a.end() && iterB != b.end()) {
		if (*iterA++ != *iterB++) {
			match = false;
			break;
		}
	}

	return (iterA == a.end() && iterB == b.end() && match);
}

inline size_t Splitter2D::PolylineNode::getIdx() const
{
	return _idx;
}

inline void Splitter2D::PolylineNode::setIdx(size_t val)
{
	_idx = val;
}

size_t Splitter2D::PolylineNode::getIndices(vector<size_t>& indices) const
{
	indices.clear();
	auto* pPrior = this;
	while (pPrior) {
		indices.push_back(pPrior->_idx);
		pPrior = pPrior->_pPrior;
	}

	return indices.size();
}

size_t Splitter2D::PolylineNode::getIndices(set<size_t>& indices) const
{
	indices.clear();
	auto* pPrior = this;
	while (pPrior) {
		indices.insert(pPrior->_idx);
		pPrior = pPrior->_pPrior;
	}

	return indices.size();
}

void Splitter2D::PolylineNode::extend(POINT_MAP_TYPE& ptMap, bool terminateAtBranch, vector<vector<size_t>>& results) {

	const auto& pCon = ptMap[_idx];
	bool extended = false;
	if (terminateAtBranch && pCon.size() > 2) {
		vector<size_t> r;
		getIndices(r);
		results.push_back(r);
		return;
	} else {
		for (size_t nextIdx : pCon) {
			if (!contains(nextIdx)) {
				PolylineNode n2;
				n2._pPrior = this;
				n2.setIdx(nextIdx);
				n2.extend(ptMap, terminateAtBranch, results);
				extended = true;
			}
		}
	}

	if (!extended) {
		// NextIdx is already in the list, so we can't use it again.
		vector<size_t> r;
		getIndices(r);
		size_t maxLen = 0;
		for (size_t idx = results.size() - 1; idx != -1; idx--) {
			if (results[idx].size() > maxLen)
				maxLen = results[idx].size();
			if (results[idx].size() > r.size()) {
				results.erase(results.begin() + idx);
			}
			else if (sameLoop(results[idx], r)) {
				r.clear(); // We already have this loop
				break;
			}
		}

		if (!r.empty() && (results.empty() || r.size() <= maxLen)) {
			results.push_back(r);
		}
	}
	
}

bool Splitter2D::PolylineNode::contains(size_t idx) const
{
	auto p = this;
	while (p) {
		if (p->_idx == idx)
			return true;
		p = p->_pPrior;
	}
	return false;
}

size_t Splitter2D::createPolylines(POINT_MAP_TYPE& ptMap, map<Edge2D, size_t>& edgeUsage, vector<Polyline>& polylines) const
{
	polylines.clear();
	PolylineNode n;

	bool isLoop = false;
	n.setIdx(getSpurSeedIndex(ptMap));

	if (n.getIdx() == -1) {
		isLoop = true;
		n.setIdx(getLoopSeedIndex(ptMap));
	}
	if (n.getIdx() == -1)
		return 0;

	vector<vector<size_t>> tmp, allPolylineIndices;
	n.extend(ptMap, !isLoop, tmp);
	sort(tmp.begin(), tmp.end(), [](const vector<size_t>& lhs, const vector<size_t>& rhs) {
		return lhs.size() < rhs.size();
		});

	for (const auto& poly : tmp) {
		vector<size_t> verifiedPoly;
		for (size_t idx : poly) {
			const auto& conP = ptMap[idx];
			if (!conP.empty()) {
				verifiedPoly.push_back(idx);
			}
		}
		if (verifiedPoly.size() == poly.size()) {
			allPolylineIndices.push_back(verifiedPoly);
		}
	}

	for (const auto& indices : allPolylineIndices) {
		Polyline pl;
		pl.insert(pl.end(), indices.begin(), indices.end());
		const auto& conP = ptMap[pl.lastIdx()];
		pl._isClosed = pl.size() > 2 && conP.contains(pl.firstIdx());
		if (isLoop)
			assert(pl._isClosed);
		else
			assert(!pl._isClosed);

		removePolylineFromMaps(pl, ptMap, edgeUsage);

		polylines.push_back(pl);
	}

	return polylines.size();
}

void Splitter2D::splitExisting(const Edge2D& e0)
{
	for (const auto& e1 : _edges) {
		set<Edge2D> subs;
		if (split(e1, e0, subs)) {
			_edges.erase(e1);
			for (const auto& s : subs) {
				if (s[0] != s[1])
					_edges.insert(s);
			}
			break;
		}
	}
}

bool Splitter2D::split(const Edge2D& e0, const Edge2D& e1, set<Edge2D>& result)
{
	const double tol = 1.0e-5;
	for (int i = 0; i < 2; i++) {
		for (int j = 0; j < 2; j++) {
			if (e0[i] == e1[j]) {
				return false;
			}
		}
	}

	LineSegment2d seg0(getPoint(e0[0]), getPoint(e0[1]));
	LineSegment2d seg1(getPoint(e1[0]), getPoint(e1[1]));
	double t;
	if (seg0.intersectionInBounds(seg1, t)) {
		const double tol = Tolerance::sameDistTol();
		Vector2d pt1 = seg0.interpolate(t);
		size_t idx1 = findPtIndex(pt1);
		if (idx1 == e0[0] || idx1 == e0[1])
			return false;

		Vector2d pt0 = seg0[0];
		Vector2d pt2 = seg0[1];

		Vector2d v0 = pt1 - pt0;
		Vector2d v1 = pt2 - pt1;
		if (v0.norm() > tol && v1.norm() > tol) {
			size_t idx0 = addPoint(pt0);
			size_t idx1 = addPoint(pt1);
			size_t idx2 = addPoint(pt2);

			assert(idx0 != idx1);
			assert(idx1 != idx2);
			result.insert(Edge2D(idx0, idx1));
			result.insert(Edge2D(idx1, idx2));
		}
	}

	return !result.empty();
}

bool Splitter2D::splitWithAllPoints(const Edge2D& e0, set<Edge2D>& subSegs)
{
	const double tol = Tolerance::paramTol();
	vector<Vector2d> pts;
	LineSegment2d seg(_pts[e0[0]], _pts[e0[1]]);
	for (auto& pt : _pts) {
		double t;
		if (seg.project(pt, t) && tol < t && t < 1 - tol) {
			if (pts.empty())
				pts.push_back(_pts[e0[0]]);
			pts.push_back(seg.interpolate(t));
		}
	}

	if (pts.empty())
		return false;
	else {
		pts.push_back(_pts[e0[1]]);
		for (size_t i = 0; i < pts.size() - 1; i++) {
			size_t j = i + 1;

			size_t idx0 = addPoint(pts[i]);
			size_t idx1 = addPoint(pts[j]);
			Edge2D e(idx0, idx1);
			subSegs.insert(e);
		}
	}

	return true;
}

bool Splitter2D::project(const Vector3d& pt, Vector2d& result, double tol) const
{
	auto dist = _plane.distanceToPoint(pt);
	if (dist < tol) {
		Vector3d v = pt - _plane.getOrigin();
		double x = v.dot(_xAxis);
		double y = v.dot(_yAxis);
		result = Vector2d(x, y);
		return true;
	}

	return false;
}

inline Vector3d Splitter2D::unproject(const Vector2d& pt2d) const
{
	Vector3d pt3d(_plane.getOrigin() + _xAxis * pt2d[0] + _yAxis * pt2d[1]);
	return pt3d;
}

inline Vector3d Splitter2D::unproject(size_t idx) const
{
	return unproject(_pts[idx]);
}

inline const Vector2d& Splitter2D::getPoint(size_t idx) const
{
	return _pts[idx];
}

Edge2D::Edge2D(size_t i0, size_t i1)

{
	_indices[0] = (i0 < i1) ? i0 : i1;
	_indices[1] = (i0 < i1) ? i1 : i0;
}

bool Edge2D::operator < (const Edge2D& rhs) const
{
	for (int i = 0; i < 2; i++) {
		if (_indices[i] < rhs._indices[i])
			return true;
		else if(_indices[i] > rhs._indices[i])
			return false;
	}
	return false;
}

inline size_t Edge2D::operator[](size_t i) const
{
	return _indices[i];
}

inline size_t Edge2D::otherIdx(size_t i) const
{
	return _indices[1 - i];
}

size_t Splitter2D::Polyline::firstIdx() const
{
	if (empty())
		return -1;
	return *begin();
}

size_t Splitter2D::Polyline::lastIdx() const
{
	if (empty())
		return -1;
	return *rbegin();
}

size_t Splitter2D::Polyline::createVector(vector<size_t>& vec, const vector<Vector2d>& pts) const
{
	vec.clear();
	if (pts.empty()) {
		for (auto iter = begin(); iter != end(); iter++) {
			vec.push_back(*iter);
		}
	} else {
		for (auto iter = begin(); iter != end(); iter++) {
			size_t nextIdx = *iter;
			if (vec.size() > 2) {
				size_t idx0 = vec[vec.size() - 2];
				size_t idx1 = vec[vec.size() - 1];

				const auto& pt0 = pts[idx0];
				const auto& pt1 = pts[idx1];
				const auto& pt2 = pts[nextIdx];
				auto l0 = (pt2 - pt0).norm();
				auto l1 = (pt2 - pt1).norm();
				if (l1 / l0 > 0.001) {
					vec.push_back(nextIdx);
				}
			} else {
				vec.push_back(nextIdx);
			}
		}
	}

	return vec.size();
}

void Splitter2D::addHit() const
{
	std::lock_guard lg(_mutex);
	_numHits++;
}

