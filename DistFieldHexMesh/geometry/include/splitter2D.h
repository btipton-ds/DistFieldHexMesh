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

#include <set>
#include <map>

#include <defines.h>
#include <vector>
#include <list>
#include <map>
#include <set>
#include <tm_vector3.h>
#include <tm_plane.h>
#include <tm_ray.h>
#include <tm_lineSegment.h>
#include <Eigen/src/Core/Matrix.h>
#include <fastBisectionSet.h>
#include <lineSegment2D.h>
#include <MultiCoreUtil.h>

namespace DFHM {

struct SplittingParams;
class Polygon;

class Edge2D {
public:
	Edge2D(size_t i0 = -1, size_t i1 = -1);
	bool operator < (const Edge2D& rhs) const;
	size_t operator[](size_t i) const;
	size_t otherIdx(size_t i) const;
private:
	size_t _indices[2];
};

class Splitter2D {
	using POINT_MAP_TYPE = std::vector<std::set<size_t>>;
public:
	struct Polyline : public std::list<size_t> {
		size_t firstIdx() const;
		size_t lastIdx() const;

		size_t createVector(std::vector<size_t>& vec, const std::vector<Vector2d>& pts) const;

		bool _isClosed = false;
	};

	Splitter2D(const Planed& plane);
	Splitter2D(const MTC::vector<Vector3d>& polyPoints);
	Splitter2D(const MTC::vector<const Vector3d*>& polyPoints);
	Splitter2D(const Polygon& face);

	void add3DEdge(const Vector3d& pt0, const Vector3d& pt1);
	void add3DTriEdges(const Vector3d pts[3], bool split);
	void add3DTriEdges(const Vector3d* pts[3], bool split);
	void addFaceEdges(const Polygon& face, bool split);
	void imprint3DPoint(const Vector3d& pt0);

	size_t getFacePoints(std::vector<std::vector<Vector3d>>& facePoints);

	void getEdgePts(std::vector<std::vector<Vector3d>>& edgePts) const;

	void getLoops(std::vector<std::vector<Vector3d>>& polylines, std::vector<std::vector<Vector3d>>& loops) const;
	void getLoops(std::vector<std::vector<Vector2d>>& polylines, std::vector<std::vector<Vector2d>>& loops) const;

	size_t getCurvatures(const SplittingParams& params, std::vector<double>& curvatures) const;
	size_t getGaps(std::vector<double>& gaps) const;

	bool intersectsTriPoints(const Vector3d* const * triPts) const;
	bool intersectWithRay(const Rayd& ray, std::vector<LineSegmentd>& segs) const;
	bool intersectWithSeg(const LineSegmentd& seg) const;

	void writeObj(const std::string& filenameRoot) const;
	void writeBoundaryEdgesObj(const std::string& filenameRoot) const;
	void writePolylinesObj(const std::string& filenameRoot) const;

	const std::set<Edge2D>& getEdges() const;
	inline const std::vector<Vector2d>& getPoints() const;
	const Planed& getPlane() const;

	Vector3d pt3D(const Vector2d& pt2d) const;

private:
	class PolylineNode {
	public:
		static bool sameLoop(const std::vector<size_t>& A, const std::vector<size_t>& B);

		size_t getIdx() const;
		void setIdx(size_t val);
		size_t getIndices(std::vector<size_t>& indices) const;
		size_t getIndices(std::set<size_t>& indices) const;
		void extend(POINT_MAP_TYPE& m, bool terminateAtBranch, std::vector<std::vector<size_t>>& results);
		bool contains(size_t idx) const;

	private:
		PolylineNode* _pPrior = nullptr;
		size_t _idx = -1;
	};

	static void cleanMap(std::map<size_t, std::set<size_t>>& map, size_t indexToRemove);

	void initFromPoints(const MTC::vector<Vector3d>& polyPoints);
	void addEdge(const Vector2d& pt0, const Vector2d& pt1, bool split);
	void addEdge(const Edge2D& edge, bool split);
	bool insideBoundary(const Vector2d& testPt) const;
	bool insideBoundary(const std::vector<Vector2d>& testFacePts) const;
	bool segIntersectsBoundary(const LineSegment2d& testSeg) const;

	size_t createPolylines(POINT_MAP_TYPE& ptMap, std::map<Edge2D, size_t>& edgeUsage, std::vector<Polyline>& polylines) const;
	Vector3d calNormal(size_t idx0, size_t idx1, size_t idx2) const;
	bool isColinear(size_t i, size_t j, size_t k) const;
	Vector2d calTurningUnitVector(size_t idx0, size_t idx1, size_t idx2) const;
	bool project(const Vector3d& pt, Vector2d& result, double tol) const;
	Vector3d pt3D(size_t idx) const;
	bool calIntersectionTriPts(const Vector3d* const * pts, Vector2d& pt0, Vector2d& pt1) const;

	size_t getPolylines(std::vector<Polyline>& polylines) const;
	void removeColinearVertsFromVertexLoop(Polyline& pl) const;
	void createPointPointMap(POINT_MAP_TYPE& ptMap) const;
	void createEdgeUsageMap(const POINT_MAP_TYPE& ptMap, std::map<Edge2D, size_t>& edgeUsage) const;
	void removePolylineFromMaps(const Polyline& pl, POINT_MAP_TYPE& ptMap, std::map<Edge2D, size_t>& edgeUsage) const;

	size_t getLoopSeedIndex(const POINT_MAP_TYPE& ptMap) const;
	size_t getSpurSeedIndex(const POINT_MAP_TYPE& ptMap) const;

	size_t addPoint(const Vector2d& pt);
	const Vector2d& getPoint(size_t idx) const;
	void splitExisting(const Edge2D& edge);
	bool split(const Edge2D& e0, const Edge2D& e1, std::set<Edge2D>& result);
	bool splitWithAllPoints(const Edge2D& e0, std::set<Edge2D>& result);

	std::vector<Vector2d> _pts;
	std::vector<size_t> _boundaryIndices;
	std::map<Vector2d, size_t> _ptToIndexMap;
	std::set<Edge2D> _edges, _boundaryEdges;

	Vector3d _xAxis, _yAxis;
	Planed _plane;
};

inline const std::set<Edge2D>& Splitter2D::getEdges() const
{
	return _edges;
}

inline const std::vector<Vector2d>& Splitter2D::getPoints() const
{
	return _pts;
}

inline const Planed& Splitter2D::getPlane() const
{
	return _plane;
}

}
