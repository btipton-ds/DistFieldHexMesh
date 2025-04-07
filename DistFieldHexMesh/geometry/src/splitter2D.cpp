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
#include <fstream>
#include <splitter2D.h>
#include <tm_lineSegment.hpp>
#include <tolerances.h>
#include <vertex.h>

using namespace std;
using namespace DFHM;

class LineSegment2d {
public:
	LineSegment2d(const Vector2d& pt0 = Vector2d(0, 0), const Vector2d& pt1 = Vector2d(0, 0));
	LineSegment2d(const LineSegment2d& rhs) = default;

	bool project(const Vector2d& pt, double& t) const;
	bool intersect(const LineSegment2d& other, double& t) const;
	const Vector2d& operator[](size_t idx) const;
	Vector2d& operator[](size_t idx);
	Vector2d interpolate(double t);

private:
	vector<Vector2d> _vals;
};

Splitter2D::Splitter2D(const Planed& plane)
	: _plane(plane)
{
	_xAxis = _plane.getXRef();
	_yAxis = _plane.getNormal().cross(_xAxis);
}

Splitter2D::Splitter2D(const MTC::vector<Vector3d>& polyPoints)
{
	auto origin = BI_LERP(polyPoints, 0.5, 0.5);

	_xAxis = BI_LERP(polyPoints, 1.0, 0.5) - BI_LERP(polyPoints, 0.0, 0.5);

	_xAxis.normalize();
	Vector3d v = polyPoints[2] - polyPoints[0];
	Vector3d n = _xAxis.cross(v);
	_plane = Planed(origin, n);
	_plane.setXRef(_xAxis);
	_yAxis = _plane.getNormal().cross(_xAxis);

#ifdef _DEBUG
	Vector2d ctr(0, 0);
#endif
	for (size_t i = 0; i < polyPoints.size(); i++) {
		size_t j = (i + 1) % polyPoints.size();
		Vector2d p0, p1;
		if (project(polyPoints[i], p0) && project(polyPoints[j], p1)) {
			size_t idx0 = addPoint(p0);
			size_t idx1 = addPoint(p1);
			Edge2D e(idx0, idx1);

			_boundaryIndices.push_back(idx0);
			_boundaryEdges.insert(e);
		}

	}
}

void Splitter2D::addEdge(const Edge2D& edge)
{
	if (edge[0] == edge[1])
		return;

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

void Splitter2D::addEdge(const Vector2d& pt0, const Vector2d& pt1)
{
	size_t idx0 = addPoint(pt0);
	size_t idx1 = addPoint(pt1);
	addEdge(Edge2D(idx0, idx1));
}

void Splitter2D::add3DEdge(const Vector3d& pt3D0, const Vector3d& pt3D1)
{
	Vector2d p0, p1;
	if (project(pt3D0, p0) && project(pt3D1, p1)) {
		addEdge(p0, p1);
	}
}

void Splitter2D::add3DTriEdges(const Vector3d* pts[3])
{
	const auto tolSqr = Tolerance::sameDistTolSqr();

	set<Vector2d> iPts;
	int numInBounds = 0;
	for (int i = 0; i < 3; i++) {
		int j = (i + 1) % 3;
		LineSegmentd seg(*pts[i], *pts[j]);
		RayHitd hit;
		if (_plane.intersectLineSegment(seg, hit, Tolerance::sameDistTol())) {
			assert(_plane.distanceToPoint(hit.hitPt) < Tolerance::sameDistTol());
			assert(seg.distanceToPoint(hit.hitPt) < Tolerance::sameDistTol());
			assert (seg.parameterize(hit.hitPt) > -Tolerance::sameDistTol() && seg.parameterize(hit.hitPt) < 1 + Tolerance::sameDistTol());
			Vector2d pt;
			if (project(hit.hitPt, pt)) {
				iPts.insert(pt);
			}
		}
	}

	if (iPts.size() == 2) {
		auto it = iPts.begin();
		const auto& pt0 = *it++;
		const auto& pt1 = *it;
		if ((pt1 - pt0).squaredNorm() > tolSqr && (insideBoundary(pt0) || insideBoundary(pt1))) {
			addEdge(pt0, pt1);
		}
	}
}

void Splitter2D::imprint3DPoint(const Vector3d& pt3D0)
{
	Vector2d pt;
	if (!project(pt3D0, pt))
		return;

	auto iter = _ptToIndexMap.find(pt);
	if (iter != _ptToIndexMap.end())
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
	auto idx = addPoint(pt);
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
			Vector3d pt(pt3D(e[i]));
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
			pl3.push_back(pt3D(pt2));
		}
		polylines.push_back(pl3);
	}

	for (const auto& pl2 : loops2d) {
		vector<Vector3d> pl3;
		for (const auto& pt2 : pl2) {
			pl3.push_back(pt3D(pt2));
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
	pl.createVector(indices);
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

		}
	}

	createEdgeUsageMap(ptMap, edgeUsage);
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

size_t Splitter2D::getCurvatures(vector<double>& curvatures) const
{
	curvatures.clear();
	POINT_MAP_TYPE ptMap;
	createPointPointMap(ptMap);

	double maxCurv = 0;
	for (size_t i = 0; i < _pts.size(); i++) {
		const auto& connectedIndices = ptMap[i];
		if (connectedIndices.size() < 2) {
			continue;
		}
		auto iter2 = connectedIndices.begin();
		const Vector2d& pt1 = _pts[i];
		const Vector2d& pt0 = _pts[*iter2++];
		const Vector2d& pt2 = _pts[*iter2];

		Vector2d v0 = (pt1 - pt0);
		auto v1 = (pt2 - pt1);
		auto l0 = v0.norm();
		auto l1 = v1.norm();
		if (l0 < Tolerance::paramTol() || l1 < Tolerance::paramTol()) {
			// Zero length segment(s) skip
			continue;
		}

		v0 /= l0;
		v1 /= l1;
		double cp = fabs(v0[0] * v1[1] - v0[1] * v1[0]);
		if (cp < Tolerance::paramTol()) {
			curvatures.push_back(0);
			continue; // Colinear, zero curvature
		}

		v0 = Vector2d(-v0[1], v0[0]);
		v1 = Vector2d(-v1[1], v1[0]);

		auto mid0 = (pt0 + pt1) * 0.5;
		auto mid1 = (pt1 + pt2) * 0.5;
		LineSegment2d seg0(mid0, v0);
		LineSegment2d seg1(mid1, v1);
		double t;
		if (!seg0.intersect(seg1, t)) {
			curvatures.push_back(0);
			continue; // Skip sharps and colinear points
		}
		Vector2d pt = seg0.interpolate(t);
		auto radius = (pt0 - pt).norm();
		if (radius < Tolerance::paramTol()) {
			curvatures.push_back(0);
			continue; // Skip sharps and colinear points
		}

		double c = 1 / radius;
		if (c > maxCurv)
			maxCurv = c;
		curvatures.push_back(c);
	}

	if (maxCurv < 1.0e-12)
		return 0;

	return curvatures.size();
}

size_t Splitter2D::getGaps(vector<double>& curvatures) const
{
	return 0;
}

void Splitter2D::writeObj(const string& filenameRoot) const
{
	size_t i = 0;
	for (auto iter = _edges.begin(); iter != _edges.end(); iter++) {
		string str = filenameRoot + to_string(i++) + ".obj";
		ofstream out(str);

		out << "#Vertices " << _pts.size() << "\n";
		for (const auto& pt : _pts) {
			auto pt2 = pt3D(pt);
			out << "v " << pt2[0] << " " << pt2[1] << " " << pt2[2] << "\n";
		}

		out << "#Edges " << _edges.size() << "\n";
		out << "l ";
		auto& edge = *iter;
		for (int i = 0; i < 2; i++) {
			out << (edge[i] + 1) << " ";
		}
		out << "\n";
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

bool Splitter2D::insideBoundary(const Vector2d& testPt) const
{
	vector<Vector2d> boundaryPts;
	for (size_t i : _boundaryIndices)
		boundaryPts.push_back(_pts[i]);
	return insideBoundary(boundaryPts, testPt);
}

bool Splitter2D::insideBoundary(const vector<Vector2d>& boundaryPts, const vector<Vector2d>& testFacePts) const
{
	for (const auto& testPt : testFacePts) {
		if (!insideBoundary(boundaryPts, testPt))
			return false;
	}
	return true;
}
bool Splitter2D::insideBoundary(const vector<Vector2d>& boundaryPts, const Vector2d& testPt) const
{
	// Check if the point lies on the positive side of all edges
	for (size_t i = 0; i < boundaryPts.size(); i++) {
		Vector2d v = testPt - boundaryPts[i];
		if (v.squaredNorm() < Tolerance::sameDistTolSqr())
			return true; // Points coincident

		size_t j = (i + 1) % boundaryPts.size();
		Vector2d xAxis = boundaryPts[j] - boundaryPts[i];
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
	Vector3d pt0 = pt3D(idx0);
	Vector3d pt1 = pt3D(idx1);
	Vector3d pt2 = pt3D(idx2);

	Vector3d v0 = pt0 - pt1;
	Vector3d v1 = pt2 - pt1;
	return v1.cross(v0);
}

bool Splitter2D::isColinear(size_t idx0, size_t idx1, size_t idx2) const
{
	const auto& pt0 = _pts[idx0];
	const auto& pt1 = _pts[idx1];
	const auto& pt2 = _pts[idx2];

	Vector2d v0 = (pt1 - pt0).normalized();
	Vector2d v1 = pt2 - pt0;
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

inline Vector3d Splitter2D::pt3D(const Vector2d& pt2d) const
{
	Vector3d pt3d(_plane.getOrgin() + _xAxis * pt2d[0] + _yAxis * pt2d[1]);
	return pt3d;
}

inline Vector3d Splitter2D::pt3D(size_t idx) const
{
	return pt3D(_pts[idx]);
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

void Splitter2D::PolylineNode::extend(POINT_MAP_TYPE& m, bool terminateAtBranch, vector<vector<size_t>>& results) {

	const auto& pCon = m[_idx];
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
				n2.extend(m, terminateAtBranch, results);
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

size_t Splitter2D::createPolylines(POINT_MAP_TYPE& m, map<Edge2D, size_t>& edgeUsage, vector<Polyline>& polylines) const
{
	polylines.clear();
	PolylineNode n;

	bool isLoop = false;
	n.setIdx(getSpurSeedIndex(m));

	if (n.getIdx() == -1) {
		isLoop = true;
		n.setIdx(getLoopSeedIndex(m));
	}
	if (n.getIdx() == -1)
		return 0;

	vector<vector<size_t>> tmp, allPolylineIndices;
	n.extend(m, !isLoop, tmp);
	sort(tmp.begin(), tmp.end(), [](const vector<size_t>& lhs, const vector<size_t>& rhs) {
		return lhs.size() < rhs.size();
		});

	for (const auto& poly : tmp) {
		vector<size_t> verifiedPoly;
		for (size_t idx : poly) {
			const auto& conP = m[idx];
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
		const auto& conP = m[pl.lastIdx()];
		pl._isClosed = pl.size() > 2 && conP.contains(pl.firstIdx());
		if (isLoop)
			assert(pl._isClosed);
		else
			assert(!pl._isClosed);

		removePolylineFromMaps(pl, m, edgeUsage);

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
	if (seg0.intersect(seg1, t) && tol < t && t < 1 - tol) {
		const double tol = Tolerance::sameDistTol();
		Vector2d pt1 = seg0.interpolate(t);
		auto iter = _ptToIndexMap.find(pt1);
		if (iter != _ptToIndexMap.end()) {
			if (iter->second == e0[0] || iter->second == e0[1])
				return false;
		}

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

bool Splitter2D::project(const Vector3d& pt, Vector2d& result) const
{
	auto dist = _plane.distanceToPoint(pt);
	if (dist < Tolerance::sameDistTol()) {
		Vector3d v = pt - _plane.getOrgin();
		double x = v.dot(_xAxis);
		double y = v.dot(_yAxis);
		result = Vector2d(x, y);
		return true;
	}

	return false;
}

inline const Vector2d& Splitter2D::getPoint(size_t idx) const
{
	return _pts[idx];
}

size_t Splitter2D::addPoint(const Vector2d& pt)
{
	auto iter = _ptToIndexMap.find(pt);
	if (iter == _ptToIndexMap.end()) {
		size_t idx = _pts.size();
		_pts.push_back(pt);
		iter = _ptToIndexMap.insert(make_pair(pt, idx)).first;
	}
	return iter->second;
}

Vector2d::Vector2d(double x, double y)
	: Eigen::Vector2d(x, y)
{
}

Vector2d::Vector2d(const Eigen::Vector2d& rhs)
	: Eigen::Vector2d(rhs)
{
}

Vector2d Vector2d::operator -(const Vector2d& rhs) const
{
	auto x = Eigen::Vector2d::operator-(rhs);
	return Vector2d(x);
}

Vector2d Vector2d::operator +(const Vector2d& rhs) const
{
	auto x = Eigen::Vector2d::operator+(rhs);
	return Vector2d(x);
}

Vector2d Vector2d::operator *(double rhs) const
{
	auto x = Eigen::Vector2d::operator*(rhs);
	return Vector2d(x);
}

Eigen::Matrix<int64_t, 2, 1> Vector2d::asIntVec() const
{
	int64_t iVal0 = Vertex::scaleToSearch((*this)[0]);
	int64_t iVal1 = Vertex::scaleToSearch((*this)[1]);
	Eigen::Matrix<int64_t, 2, 1> result(iVal0, iVal1);
	return result;
}

bool Vector2d::operator < (const Vector2d& rhs) const
{
	int64_t scale = Vertex::scaleToSearch();
	auto pt0 = asIntVec();
	auto pt1 = rhs.asIntVec();
	for (int i = 0; i < 2; i++) {
		if (pt0[i] < pt1[i])
			return true;
		else if (pt1[i] < pt0[i])
			return false;
	}
	return false;
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

inline LineSegment2d::LineSegment2d(const Vector2d& pt0, const Vector2d& pt1)
{
	_vals = { pt0, pt1 };
}

bool LineSegment2d::project(const Vector2d& pt, double& t) const
{
	const double tolSqr = Tolerance::sameDistTolSqr();

	Vector2d dir = (_vals[1] - _vals[0]);
	auto len = dir.norm();
	dir /= len;

	Vector2d v = pt - _vals[0];
	auto dp = v.dot(dir);
	t = dp / len;
	v = v - dir * dp;
	auto errSqr = v.squaredNorm();
	return errSqr < tolSqr;
}

bool LineSegment2d::intersect(const LineSegment2d& other, double& t) const
{
	const double tol = Tolerance::paramTol();
	const double MIN_COS_ANGLE = 1.0e-8;

	for (int i = 0; i < 2; i++) {
		for (int j = 0; j < 2; j++) {
			if ((_vals[i] - other._vals[j]).squaredNorm() < Tolerance::sameDistTolSqr()) {
				// The line segments share a point
				t = (i == 0) ? 0 : 1;
				return true;
			}
		}
	}

	Vector2d xAxis = _vals[1] - _vals[0];
	double len0 = xAxis.norm();
	if (len0 < tol)
		return  false;
	xAxis /= len0;

	Vector2d yAxis(-xAxis[1], xAxis[0]);

	Vector2d dir = other[1] - other[0];
	double len1 = dir.norm();
	if (len1 < tol)
		return false;
	dir /= len1;

	auto dp = dir.dot(yAxis);
	if (fabs(dp) < MIN_COS_ANGLE) {
		for (int i = 0; i < 2; i++) {
			Vector2d v = other[i] - _vals[0];
			if (fabs(v.dot(yAxis)) < tol) {
				double l = v.dot(xAxis);
				t = l / len0;
				if (tol < t && t < 1 - tol)
					return true;
			}
		}
		return false; // dir is perpendicular to yAxis, so parallel to xAxis
	}

	Vector2d v = other[0] - _vals[0];

	auto h = v.dot(yAxis);
	auto dist0 = -h / dp;
	Vector2d hitPt = other[0] +  dir * dist0;

	v = hitPt - _vals[0];
	auto dist1 = v.norm();
	t = dist1 / len0;

	bool inBounds = tol < t && t < 1 - tol;
#if VALIDATION_ON && defined(_DEBUG) // Verification code
	if (inBounds) {
		Vector2d vTest = hitPt - _vals[0];
		double testDist = vTest.dot(yAxis);
		if (fabs(testDist) > tol) {
			assert(!"Point not on seg0");
		}

		Vector2d vx = hitPt - other[0];
		vx = vx - dir * vx.dot(dir);
		if (vx.norm() > tol) {
			assert(!"Point not on seg1");
		}
	}
#endif
	
	return inBounds;
}

inline const Vector2d& LineSegment2d::operator[](size_t idx) const
{
	return _vals[idx];
}

inline Vector2d& LineSegment2d::operator[](size_t idx)
{
	return _vals[idx];
}

Vector2d LineSegment2d::interpolate(double t)
{
	const double tol = Tolerance::paramTol();
	assert(t >= -tol && t <= 1 + tol);
	if (t < 0)
		t = 0;
	else if (t > 1)
		t = 1;
	Vector2d v = _vals[1] - _vals[0];
	Vector2d x = _vals[0] + v * t;
	return x;
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

size_t Splitter2D::Polyline::createVector(vector<size_t>& vec) const
{
	vec.clear();
	for (auto iter = begin(); iter != end(); iter++)
		vec.push_back(*iter);

	return vec.size();
}
