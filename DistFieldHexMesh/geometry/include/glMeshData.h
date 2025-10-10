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
#include <vector>
#include <atomic>
#include <tm_vector3.h>

namespace DFHM {
	class Polygon;
	class Block;
	class GraphicsCanvas;

	class GlPoints : public std::vector<float>
	{
	public:
		GlPoints();
		GlPoints(const GlPoints& src);
		size_t getId() const;
		size_t changeNumber() const;
		void changed();
	private:
		static std::atomic<size_t> _statId;
		size_t _id, _changeNumber = 0;
	};

	struct GlMeshFaces {
		// all faces for a block
		void addFace(const Polygon& face);
		void addTriangle(const Vector3d& pt0, const Vector3d& pt1, const Vector3d& pt2);
		size_t numTriVertices() const;
		size_t numEdgeVertices() const;

		std::vector<float> _glTriPoints, _glTriNormals, _glEdgePoints;
	};

	struct VertexPoint {
		VertexPoint(const Vector3f& pt = Vector3f());
		bool operator < (const VertexPoint& rhs) const;
		Vector3<int> _iPoint;
	};

	struct VertexPointAndNormal {
		VertexPointAndNormal(const Vector3f& pt = Vector3f(), const Vector3f& normal = Vector3f());
		bool operator < (const VertexPointAndNormal& rhs) const;
		Vector3<int> _iPoint, _iNormal;
	};

	struct GLEdge {
		GLEdge(unsigned int idx0 = -1, unsigned int idx1 = -1);
		GLEdge(const GLEdge& src) = default;

		bool operator < (const GLEdge& rhs) const;

		const unsigned int _idx0;
		const unsigned int _idx1;
	};

	using GlMeshFacesPtr = std::shared_ptr<GlMeshFaces>;
	using GlMeshFacesVector = std::vector<GlMeshFacesPtr>;
	using GlMeshFacesGroup = std::vector<GlMeshFacesVector>;

	using glPointsPtr = std::shared_ptr<GlPoints>;
	using glPointsVector = std::vector<glPointsPtr>;
	using glPointsGroup = std::vector<glPointsVector>;

	inline size_t GlPoints::getId() const
	{
		return _id;
	}

	inline size_t GlPoints::changeNumber() const
	{
		return _changeNumber;
	}

	inline void GlPoints::changed()
	{
		_changeNumber++;
	}

}

