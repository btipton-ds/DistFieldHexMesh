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
#include <memory>
#include <vector>
#include <set>
#include <tm_ray.h>

namespace OGL
{
	struct Indices;
	using IndicesPtr = std::shared_ptr<Indices>;
}

namespace DFHM {

class Polygon;

class DebugMeshData {
public:
	DebugMeshData() = default;
	DebugMeshData(const DebugMeshData& src) = default;
	virtual ~DebugMeshData();

	void clear();

	void add(const Vector3d& pt);
	void add(const Rayd& ray);
	void add(const LineSegmentd& seg);
	void add(const Polygon& face);
	void addTri(const Vector3d pts[3]);
	void addQuad(const Vector3d pts[4]);

	void remove(const Vector3d& pt);

	void getGLEdges(std::vector<float>& pts, std::vector<unsigned int>& indices) const;
	void getGLTris(std::vector<float>& pts, std::vector<float>& normals, std::vector<unsigned int>& indices) const;

	void setEdgeTess(const OGL::IndicesPtr& edgeTess);
	const OGL::IndicesPtr& getEdgeTess() const;

	void setFaceTess(const OGL::IndicesPtr& faceTess);
	const OGL::IndicesPtr& getFaceTess() const;

private:
	std::set<Vector3d> _points;
	std::vector<Rayd> _rays;
	std::vector<LineSegmentd> _segments;
	std::vector<float> _triPts, _triNormals;

	OGL::IndicesPtr _edgeTess;
	OGL::IndicesPtr _faceTess;
};

inline void DebugMeshData::setEdgeTess(const OGL::IndicesPtr& edgeTess)
{
	_edgeTess = edgeTess;
}

inline const OGL::IndicesPtr& DebugMeshData::getEdgeTess() const
{
	return _edgeTess;
}

inline void DebugMeshData::setFaceTess(const OGL::IndicesPtr& faceTess)
{
	_faceTess = faceTess;
}

inline const OGL::IndicesPtr& DebugMeshData::getFaceTess() const
{
	return _faceTess;
}

}