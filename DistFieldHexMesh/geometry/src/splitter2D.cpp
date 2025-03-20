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
	Vector2d eval(double t);

private:
	vector<Vector2d> _vals;
};

Splitter2D::Splitter2D(const Planed& plane)
	: _plane(plane)
{
	_xAxis = _plane.getXRef();
	_yAxis = _plane.getNormal().cross(_xAxis);
}

void Splitter2D::add3DEdge(const Vector3d& pt3D0, const Vector3d& pt3D1)
{
	size_t idx0 = addPoint(project(pt3D0));
	size_t idx1 = addPoint(project(pt3D1));
	Edge2D e(idx0, idx1);
	splitExisting(e);

	set<Edge2D> subs;
	for (const auto& e1 : _edges) {
		split(e, e1, subs);
	}

	if (subs.empty()) {
		_edges.insert(e);
	} else {
		_edges.insert(subs.begin(), subs.end());
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

	std::vector<std::vector<Vector2d>> facePoints;
	getAllFacePoints(facePoints);
	for (const auto& bounds : facePoints) {
		if (insideBoundary(bounds, pt0) && insideBoundary(bounds, pt1))
			return true;
	}
	return false;
}

size_t Splitter2D::getFacePoints(std::vector<std::vector<Vector3d>>& facePoints)
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
		vector<Vector3d> pts;
		for (int i = 0; i < 2; i++) {
			Vector3d pt(pt3D(e[i]));
			pts.push_back(pt);
		}
		edgePts.push_back(pts);
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
	// Check if the point is an exact match for a boundary pt
	for (const auto& boundaryPt : boundaryPts) {
		Vector2d v = testPt - boundaryPt;
		if (v.squaredNorm() < Tolerance::sameDistTolSqr())
			return true;
	}

	// Check if the point lies on the positive side of all edges
	for (size_t i = 0; i < boundaryPts.size(); i++) {
		size_t j = (i + 1) % boundaryPts.size();
		Vector2d xAxis = boundaryPts[j] - boundaryPts[i];
		xAxis.normalize();
		Vector2d yAxis(-xAxis[1], xAxis[0]);
		Vector2d v = testPt - boundaryPts[i];
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

size_t Splitter2D::createPolygon(map<size_t, set<size_t>>& map, vector<size_t>& faceVerts) const
{
	// Start the loop
	for (const auto& pair : map) {
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

		auto iter = map.find(faceVerts.back());
		if (iter != map.end()) {
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
			auto iter = map.find(idx);
			if (iter != map.end() && iter->second.size() == 2) {
				deadEntries.push_back(idx);
			}
		}

		for (size_t idx : deadEntries) {
			map.erase(idx);
		}

		for (auto& pair : map) {
			auto& connected = pair.second;
			for (size_t i : deadEntries) {
				connected.erase(i);
			}
		}
	}

	return faceVerts.size();
}

void Splitter2D::splitExisting(const Edge2D& e0)
{
	for (const auto& e1 : _edges) {
		set<Edge2D> subs;
		if (split(e1, e0, subs)) {
			_edges.erase(e1);
			for (const auto& s : subs) {
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
	if (seg0.intersect(seg1, t)) {
		Vector2d pt0 = seg0[0];
		Vector2d pt1 = seg0.eval(t);
		Vector2d pt2 = seg0[1];

		size_t idx0 = addPoint(pt0);
		size_t idx1 = addPoint(pt1);
		size_t idx2 = addPoint(pt2);
		result.insert(Edge2D(idx0, idx1));
		result.insert(Edge2D(idx1, idx2));
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

#if 1 // Verification code
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

#endif
	
	return tol < t && t < 1 - tol;
}

inline const Vector2d& LineSegment2d::operator[](size_t idx) const
{
	return _vals[idx];
}

inline Vector2d& LineSegment2d::operator[](size_t idx)
{
	return _vals[idx];
}

Vector2d LineSegment2d::eval(double t)
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

