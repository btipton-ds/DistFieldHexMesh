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

#include <tm_vector3.h>
#include <tm_plane.h>
#include <index3D.h>
#include <polygon.h>
#include <intersectEdge.h>

namespace DFHM {

	class Block;
	class Polyhedron;

	class PolygonSplitter {
	public:
		PolygonSplitter(Block* pBlock, const Index3DId& polygonId);

		bool splitAtCentroid();
		bool splitAtPoint(const Vector3d& pt);
		bool createTrimmedFace(const MTC::vector<MTC::set<Edge>>& patchFaceEdges, 
			const MTC::set<Index3DId>& skippedVerts, Index3DId& faceId);
		Edge createIntersectionEdge(const Planed& plane);
		void createTrimmedFacesFromFaces(const MTC::set<Index3DId>& modelFaces, MTC::set<Index3DId>& imprintedVertIds, MTC::set<Index3DId>& newFaceIds);

		// Create ordered vertices to form a new polygon who's face normal is in the same direction as triangle normals defined in edges.
		// Edges should be created by intersectModelTris which will record the triangle normals from the model as they are intersected.
		static bool connectIntersectEdges(const Block* pBlock, const MTC::set<IntersectEdge>& edges, MTC::vector<MTC::vector<IntersectVertId>>& faceVertices, bool isIntersection = false);
		static bool connectEdges(const Block* pBlock, const MTC::set<Edge>& edges, MTC::vector<MTC::vector<Index3DId>>& faceVertices);

		const Block* getBlockPtr() const;
		Block* getBlockPtr();

		LAMBDA_CLIENT_DECLS

	private:
		bool splitAtPointInner(Polygon& realFace, Polygon& referanceFace, const Vector3d& pt) const;
		bool calModelNorm(const MTC::map<Index3DId, MTC::set<Edge>>& vertModEdgeMap, const Index3DId& vertId, Vector3d& norm) const;
		bool createTrimmedEdge(const Edge& srcEdge, const Edge& cuttingEdge, Edge& newEdge);
		void createTrimmedFaceEdges(const MTC::set<Edge>& modFaceEdges, MTC::set<Edge>& trimEdges);
		bool createConvexFaceVerts(const MTC::vector<Index3DId>& verts, MTC::set<Index3DId>& imprintedVertIds, MTC::vector<MTC::vector<Index3DId>>& convexFaceVerts);

		Index3DId addVertex(const Vector3d& pt) const;
		Index3DId addFace(const Polygon& face) const;

		Block* _pBlock;
		Index3DId _polygonId;
	};

}
