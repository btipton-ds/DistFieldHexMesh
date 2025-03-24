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

#include <list>
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
		auto p0 = project(polyPoints[i]);
		auto p1 = project(polyPoints[j]);
		addEdge(p0, p1);

		size_t idx0 = addPoint(p0);
		size_t idx1 = addPoint(p1);

#ifdef _DEBUG
		ctr += p0;
#endif
		_boundaryPts.push_back(p0);
		_boundaryEdges.insert(Edge2D(idx0, idx1));
	}

#ifdef _DEBUG
	ctr = ctr * (1.0 / _boundaryPts.size());
	assert(insideBoundary(ctr));
	Vector2d v2 = _boundaryPts[0] - ctr;
	Vector2d testPt = ctr + v2 * 1.01;
	assert(!insideBoundary(testPt));
#endif // _DEBUG
}

void Splitter2D::addEdge(const Vector2d& pt0, const Vector2d& pt1)
{
	size_t idx0 = addPoint(pt0);
	size_t idx1 = addPoint(pt1);
	if (idx0 == idx1)
		return;

	Edge2D e(idx0, idx1);
	splitExisting(e);

	set<Edge2D> subs;
	for (const auto& e1 : _edges) {
		split(e, e1, subs);
	}

	if (subs.empty()) {
		if (e[0] != e[1])
			_edges.insert(e);
	} else {
		for (auto& s : subs) {
			if (s[0] != s[1])
				_edges.insert(s);
		}
	}
}

void Splitter2D::add3DEdge(const Vector3d& pt3D0, const Vector3d& pt3D1)
{
	addEdge(project(pt3D0), project(pt3D1));

}

void Splitter2D::add3DTriEdge(const Vector3d pts[3])
{
	set<size_t> iPts;
	int numInBounds = 0;
	for (int i = 0; i < 3; i++) {
		int j = (i + 1) % 3;
		LineSegmentd seg(pts[i], pts[j]);
		RayHitd hit;
		if (_plane.intersectLineSegment(seg, hit, Tolerance::sameDistTol())) {
			assert(_plane.distanceToPoint(hit.hitPt) < Tolerance::sameDistTol());
			assert(seg.distanceToPoint(hit.hitPt) < Tolerance::sameDistTol());
			assert (seg.parameterize(hit.hitPt) > -Tolerance::sameDistTol() && seg.parameterize(hit.hitPt) < 1 + Tolerance::sameDistTol());
			auto pt = project(hit.hitPt);
			size_t idx = addPoint(pt);
			iPts.insert(idx);
		}
	}

	if (iPts.size() == 2) {
		auto it = iPts.begin();
		size_t idx0 = *it++;
		size_t idx1 = *it;
		if (idx0 != idx1 && (insideBoundary(_pts[idx0]) || insideBoundary(_pts[idx1]))) {
			_edges.insert(Edge2D(idx0, idx1));
		}
	}
}

void Splitter2D::imprint3DPoint(const Vector3d& pt3D0)
{
	Vector2d pt = project(pt3D0);
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

bool Splitter2D::contains3DEdge(const Vector3d& pt3D0, const Vector3d& pt3D1)
{
	auto pt0 = project(pt3D0);
	auto pt1 = project(pt3D1);

	vector<vector<Vector2d>> facePoints;
	getAllFacePoints(facePoints);
	for (const auto& bounds : facePoints) {
		if (insideBoundary(bounds, pt0) && insideBoundary(bounds, pt1))
			return true;
	}
	return false;
}

size_t Splitter2D::getFacePoints(vector<vector<Vector3d>>& facePoints)
{
	if (_allFacePoints.empty()) {
		getAllFacePoints(_allFacePoints);
	}

	for (const auto& testFacePoints2D : _allFacePoints) {
		vector<Vector3d> pts;
		for (const auto& pt2d : testFacePoints2D) {
			auto pt = pt3D(pt2d);
			pts.push_back(pt);
		}

		Vector3d v0 = (pts[1] - pts[0]).normalized();
		Vector3d v1 = (pts[2] - pts[1]).normalized();
		Vector3d n = v1.cross(v0);
		if (n.dot(_plane.getNormal()) < 0) {
			reverse(pts.begin(), pts.end());
		}

		facePoints.push_back(pts);
	}

	return facePoints.size();
}

size_t Splitter2D::getFacePoints(const vector<Vector3d>& boundaryFacePts, vector<vector<Vector3d>>& facePoints)
{
	if (_allFacePoints.empty()) {
		getAllFacePoints(_allFacePoints);
	}

	vector<Vector2d> boundaryFacePts2D;
	for (const auto& pt : boundaryFacePts)
		boundaryFacePts2D.push_back(project(pt));

	for (const auto& testFacePoints2D : _allFacePoints) {
		if (insideBoundary(boundaryFacePts2D, testFacePoints2D)) {
			vector<Vector3d> pts;
			for (const auto& pt2d : testFacePoints2D) {
				auto pt = pt3D(pt2d);
				pts.push_back(pt);
			}
			facePoints.push_back(pts);
		}
	}
	return facePoints.size();
}

void Splitter2D::getEdgePts(vector<vector<Vector3d>>& edgePts) const
{
	for (const auto& e : _edges) {
		if (_boundaryEdges.contains(e))
			continue;

		vector<Vector3d> pts;
		for (int i = 0; i < 2; i++) {
			Vector3d pt(pt3D(e[i]));
			pts.push_back(pt);
		}
		edgePts.push_back(pts);
	}
}

struct Splitter2D::Polyline : public list<size_t> {
	inline size_t firstIdx() const
	{
		if (empty())
			return -1;
		return *begin();
	}

	inline size_t lastIdx() const
	{
		if (empty())
			return -1;
		return *rbegin();
	}

	inline bool addEdge(const Edge2D& e) {
		if (empty()) {
			push_back(e[0]);
			push_back(e[1]);
			_usedIndices.insert(e[0]);
			_usedIndices.insert(e[1]);
			return true;
		}

		for (int i = 0; i < 2; i++) {
			int j = 1 - i;
			size_t idx0 = e[i];
			size_t idx1 = e[j];
			if (!_usedIndices.contains(idx1)) {
				if (idx0 == back()) {
					push_back(idx1);
					_usedIndices.insert(idx1);
					return true;
				} else if (idx0 == front()) {
					push_front(idx1);
					_usedIndices.insert(idx1);
					return true;
				}
			}
		}
		return false;
	}

	set<size_t> _usedIndices;
};

size_t Splitter2D::getPolylines(vector<vector<Vector3d>>& polylines) const
{
	vector<vector<Vector2d>> polylines2D;
	getPolylines(polylines2D);
	for (const auto& pl2 : polylines2D) {
		vector<Vector3d> pl3;
		for (const auto& pt2 : pl2) {
			pl3.push_back(pt3D(pt2));
		}
		polylines.push_back(pl3);
	}

	return polylines.size();
}

size_t Splitter2D::findMinConnectedIndex(const map<size_t, set<size_t>>& ptMap)
{
	size_t result = -1;
	size_t min = SIZE_MAX;
	for (const auto& pair : ptMap) {
		if (pair.second.size() < min) {
			assert(!pair.second.empty());
			min = pair.second.size();
			result = pair.first;
		}
	}
	return result;
}

size_t Splitter2D::getPolylines(vector<vector<Vector2d>>& polylines) const
{
	if (_edges.size() > _boundaryEdges.size()) {
		vector<Polyline> plVec;

		set<Edge2D> tmp(_edges);
		for (const auto& e : _boundaryEdges) {
			tmp.erase(e);
		}

		map<size_t, set<size_t>> ptMap;
		for (const auto& e : tmp) {
			auto iter = ptMap.find(e[0]);
			if (iter == ptMap.end())
				iter = ptMap.insert(make_pair(e[0], set<size_t>())).first;
			iter->second.insert(e[1]);

			iter = ptMap.find(e[1]);
			if (iter == ptMap.end())
				iter = ptMap.insert(make_pair(e[1], set<size_t>())).first;
			iter->second.insert(e[0]);
		}

		Edge2D seedEdge;
		while (!tmp.empty()) {
			seedEdge = *tmp.begin();

			Polyline pl;
			buildPolyline(ptMap, seedEdge, pl);
			if (!pl.empty()) {
				plVec.push_back(pl);
				auto iter0 = pl.begin();
				auto iter1 = iter0;
				iter1++;

				while (iter1 != pl.end()) {
					Edge2D e(*iter0++, *iter1++);
					tmp.erase(e);

					auto iter = ptMap.find(e[0]);
					if (iter != ptMap.end())
						iter->second.erase(e[1]);
					if (iter->second.empty())
						ptMap.erase(e[0]);

					iter = ptMap.find(e[1]);
					if (iter != ptMap.end())
						iter->second.erase(e[0]);
					if (iter->second.empty())
						ptMap.erase(e[1]);
				}

				removeColinearVertsFromPolyline(pl);
				vector<Vector2d> pts;
				for (iter0 = pl.begin(); iter0 != pl.end(); iter0++) {
					pts.push_back(_pts[*iter0]);
				}
				polylines.push_back(pts);
			}
		};
	}

	return polylines.size();
}

size_t Splitter2D::getCurvatures(std::vector<std::vector<double>>& curvatures) const
{
	curvatures.clear();
	vector<vector<Vector2d>> polylines;
	getPolylines(polylines);

	for (const auto& pl : polylines) {
		std::vector<double> cvec;
		for (size_t j = 0; j < pl.size(); j++) {
			if (j == 0 || j == pl.size() - 2) {
				cvec.push_back(0);
				continue;
			}
			size_t i = (j + pl.size() - 1) % pl.size();
			size_t k = (j + 1) % pl.size();
			const Vector2d& pt0 = pl[i];
			const Vector2d& pt1 = pl[1];
			const Vector2d& pt2 = pl[2];

			Vector2d v0 = (pt1 - pt0);
			auto v1 = (pt2 - pt1);
			auto l0 = v0.norm();
			auto l1 = v1.norm();
			if (l0 < Tolerance::paramTol() || l1 < Tolerance::paramTol()) {
				cvec.push_back(0);
				continue;
			}

			v0 /= l0;
			v1 /= l1;
			double cp = fabs(v0[0] * v1[1] - v0[1] * v1[0]);
			if (cp > 0.7071 || cp < Tolerance::paramTol()) {
				cvec.push_back(0);
				continue; // Skip sharps and colinear points
			}

			v0 = Vector2d(-v0[1], v0[0]);
			v1 = Vector2d(-v1[1], v1[0]);

			auto mid0 = (pt0 + pt1) * 0.5;
			auto mid1 = (pt1 + pt2) * 0.5;
			LineSegment2d seg0(mid0, v0);
			LineSegment2d seg1(mid1, v1);
			double t;
			if (!seg0.intersect(seg1, t)) {
				cvec.push_back(0);
				continue; // Skip sharps and colinear points
			}
			Vector2d pt = seg0.interpolate(t);
			auto radius = (pt0 - pt).norm();
			if (radius < Tolerance::paramTol()) {
				cvec.push_back(0);
				continue; // Skip sharps and colinear points
			}
			cvec.push_back(1 / radius);
		}
	
		curvatures.push_back(cvec);
	}
	return curvatures.size();
}

size_t Splitter2D::getGaps(std::vector<double>& curvatures) const
{
	return 0;
}

void Splitter2D::buildPolyline(std::map<size_t, std::set<size_t>>& ptMap, const Edge2D& e, Polyline& pl) const
{
	pl.addEdge(e);
	while (extendPolyline(ptMap, pl));

}

bool Splitter2D::extendPolyline(std::map<size_t, std::set<size_t>>& ptMap, Polyline& pl) const
{
	size_t lastIdx = pl.lastIdx();
	auto iter = ptMap.find(lastIdx);
	if (iter != ptMap.end()) {
		const auto& s = iter->second;
		if (s.size() <= 2) {
			set<size_t> options;
			for (size_t idx : s) {
				if (!pl._usedIndices.contains(idx))
					options.insert(idx);
			}
			if (options.size() == 1) {
				if (pl.addEdge(Edge2D(lastIdx, *options.begin())))
					return true;
			}
		}
	}

	lastIdx = pl.firstIdx();
	iter = ptMap.find(lastIdx);
	if (iter != ptMap.end()) {
		const auto& s = iter->second;
		if (s.size() <= 2) {
			set<size_t> options;
			for (size_t idx : s) {
				if (!pl._usedIndices.contains(idx))
					options.insert(idx);
			}
			if (options.size() == 1) {
				if (pl.addEdge(Edge2D(lastIdx, *options.begin())))
					return true;
			}
		}
	}

	return false;
}

void Splitter2D::removeColinearVertsFromPolyline(Polyline& pl) const
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

size_t Splitter2D::getAllFacePoints(vector<vector<Vector2d>>& facePoints)
{
	map<size_t, set<size_t>> pointToPointMap;

#if 0 && defined(_DEBUG)
	for (const auto& pt : _pts) {
		cout << "[" << pt[0] << ", " << pt[1] << "]\n";
	}
#endif // _DEBUG

	for (const auto& edge : _edges) {
		for (int i = 0; i < 2; i++) {
			size_t ptIdx = edge[i];
			size_t otherPtIdx = edge[1 - i];
			auto iter = pointToPointMap.find(ptIdx);
			if (iter == pointToPointMap.end())
				iter = pointToPointMap.insert(make_pair(ptIdx, set<size_t>())).first;
			iter->second.insert(otherPtIdx);
		}
	}

	set<size_t> deadEntries;
	for (const auto& pair : pointToPointMap) {
		if (pair.second.size() == 1) {
			deadEntries.insert(pair.first);
		}
	}

	for (size_t idx0 : deadEntries) {
		auto iter0 = pointToPointMap.find(idx0);
		if (iter0 != pointToPointMap.end()) {
			for (const size_t idx1 : iter0->second) {
				auto iter1 = pointToPointMap.find(idx1);
				if (iter1 != pointToPointMap.end()) {
					iter1->second.erase(idx0);
				}
			}
		}

		pointToPointMap.erase(idx0);
	}

	vector<vector<Vector2d>> faceIndices;

	bool done;
	do {
		done = true;
		vector<size_t> faceIndices;
		if (createPolygon(pointToPointMap, faceIndices) > 2) {
			done = false;
			vector<Vector2d> pts;
			for (size_t idx : faceIndices) {
				pts.push_back(_pts[idx]);
			}
			facePoints.push_back(pts);
		}
	} while (!done);

	return facePoints.size();
}

bool Splitter2D::insideBoundary(const Vector2d& testPt) const
{
	return insideBoundary(_boundaryPts, testPt);
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

namespace
{
	struct PolylineNode {
		PolylineNode* _pPrior = nullptr;
		size_t _idx;
	};

	void extendPl(map<size_t, set<size_t>>& m, PolylineNode& n, vector<vector<size_t>>& results) {
		set<size_t> used;
		auto* pPrior = &n;
		while (pPrior) {
			used.insert(pPrior->_idx);
			pPrior = pPrior->_pPrior;
		}

		auto iter = m.find(n._idx);
		if (iter != m.end()) {
			bool extended = false;
			for (size_t nextIdx : iter->second) {
				if (!used.contains(nextIdx)) {
					PolylineNode n2;
					n2._pPrior = &n;
					n2._idx = nextIdx;
					extendPl(m, n2, results);
					extended = true;
				}
			}
			if (!extended) {
				// NextIdx is already in the list, so we can't use it again.
				vector<size_t> r;
				r.push_back(n._idx);
				auto p = n._pPrior;
				while (p) {
					r.push_back(p->_idx);
					p = p->_pPrior;
				}
	
				size_t maxSize = 0;
				vector<size_t> tooLong;
				for (size_t i = 0; i < results.size(); i++) {
					if (results[i].size() > r.size()) {
						tooLong.push_back(i);
					} else if (results[i].size() > maxSize)
						maxSize = results[i].size();
				}

				for (size_t idx = tooLong.size() - 1; idx != -1; idx--)
					results.erase(results.begin() + idx);

				if (results.empty() || r.size() <= maxSize)
					results.push_back(r);
			}
		}
	}

	void removeIndices(map<size_t, set<size_t>>& m, vector<size_t>& indices)
	{
		for (size_t idx : indices) {
			auto iter = m.find(idx);
			for (size_t idx2 : indices) {
				iter->second.erase(idx2);
			}
		}

		for (size_t idx : indices) {
			auto iter = m.find(idx);
			if (iter->second.empty())
				m.erase(idx);
		}

		// There may be missing edges in the map due to the removal
		// This restores the connections for edges in indices which were removed
		// from the map.
		for (size_t i = 0; i < indices.size(); i++) {
			size_t j = (i + 1) % indices.size();
			auto iterI = m.find(indices[i]);
			auto iterJ = m.find(indices[j]);
			if (iterI != m.end() && iterJ != m.end()) {
				iterI->second.insert(indices[j]);
				iterJ->second.insert(indices[i]);
			}
		}
	}
}

size_t Splitter2D::createPolygon(map<size_t, set<size_t>>& m, vector<size_t>& faceVerts) const
{
#if 0
	if (m.empty())
		return 0;

	PolylineNode n;
	n._idx = m.begin()->first;
	vector<vector<size_t>> tmp, indices;
	extendPl(m, n, tmp);
	sort(tmp.begin(), tmp.end(), [](const vector<size_t>& lhs, const vector<size_t>& rhs) {
		return lhs.size() < rhs.size();
	});

	for (const auto& poly : tmp) {
		vector<size_t> verifiedPoly;
		for (size_t idx : poly) {
			auto iter = m.find(idx);
			if (iter != m.end()) {
				verifiedPoly.push_back(iter->first);
			}
		}
		if (verifiedPoly.size() == poly.size()) {
			indices.push_back(verifiedPoly);
		}
	}

	if (indices.size() >= 1) {
		assert(indices.front().size() <= 8);
		faceVerts = indices.front();
		removeIndices(m, faceVerts);
	}
#else
	// Start the loop
	for (const auto& pair : m) {
		if (pair.second.size() == 2) {
			auto iter = pair.second.begin();
			size_t idx0 = *iter++;
			size_t idx1 = pair.first;
			size_t idx2 = *iter;

			if (!isColinear(idx0, idx1, idx2)) {
				Vector3d n = calNormal(idx0, idx1, idx2);
				const auto& planeNorm = _plane.getNormal();
				if (n.dot(planeNorm) < 0) {
					// Start reversed
					faceVerts.push_back(idx2);
					faceVerts.push_back(idx1);
					faceVerts.push_back(idx0);
				}
				else {
					// Start normal
					faceVerts.push_back(idx0);
					faceVerts.push_back(idx1);
					faceVerts.push_back(idx2);
				}
				break;
			}
		}
	}

	size_t lastIdx, curIdx, nextIdx;
	do {
		if (faceVerts.size() <= 2) {
			break;
		}
		lastIdx = faceVerts[faceVerts.size() - 2];
		curIdx = faceVerts[faceVerts.size() - 1];

		auto iter = m.find(faceVerts.back());
		if (iter != m.end()) {
			double maxPositiveTurn = -DBL_MAX;
			const auto& connected = iter->second;
			for (const size_t idx : connected) {
				if (idx != lastIdx) {
					Vector2d turningVector = calTurningUnitVector(lastIdx, curIdx, idx);
					if (turningVector[0] > -Tolerance::paramTol() && turningVector[1] > maxPositiveTurn) {
						maxPositiveTurn = turningVector[1];
						nextIdx = idx;
					}
				}
			}

			faceVerts.push_back(nextIdx);
		}
	} while (faceVerts.front() != faceVerts.back());

	if (!faceVerts.empty()) {
		faceVerts.pop_back();

		vector<size_t> deadEntries;
		for (size_t idx : faceVerts) {
			auto iter = m.find(idx);
			if (iter != m.end() && iter->second.size() == 2) {
				deadEntries.push_back(idx);
			}
		}

		for (size_t idx : deadEntries) {
			m.erase(idx);
		}

		for (auto& pair : m) {
			auto& connected = pair.second;
			for (size_t i : deadEntries) {
				connected.erase(i);
			}
		}
	}
#endif
	return faceVerts.size();
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
	LineSegment2d seg0(getPoint(e0[0]), getPoint(e0[1]));
	LineSegment2d seg1(getPoint(e1[0]), getPoint(e1[1]));
	double t;
	if (seg0.intersect(seg1, t) ) {
		const double tol = Tolerance::sameDistTol();
		Vector2d pt0 = seg0[0];
		Vector2d pt1 = seg0.interpolate(t);
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

Vector2d Splitter2D::project(const Vector3d& pt) const
{
	Vector3d v = pt - _plane.getOrgin();
	double x = v.dot(_xAxis);
	double y = v.dot(_yAxis);
	Vector2d result(x, y);

	return result;
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

bool Vector2d::operator < (const Vector2d& rhs) const
{
	int64_t scale = Vertex::scaleToSearch();
	Eigen::Matrix<int64_t, 2, 1> pt0((int64_t)((scale * (*this)[0]) + 0.5) , (int64_t)((scale * (*this)[1]) + 0.5));
	Eigen::Matrix<int64_t, 2, 1> pt1((int64_t)((scale * rhs[0]) + 0.5), (int64_t)((scale * rhs[1]) + 0.5));
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
#if 1 // Verification code
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

