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
#include <tm_vector3.h>
#include <tm_plane.h>
#include <index3D.h>
#include <pool_set.h>
#include <pool_vector.h>

namespace DFHM {

class Block;
class Polygon;
class Polyhedron;

class PolyhedronSplitter {
public:
	PolyhedronSplitter(Block* pBlock, const Index3DId& polyhedronId);

	bool splitIfNeeded();
	bool splitAtPoint(const Vector3d& pt);

	// Cutting creates final faces "on" the model.
	bool cutWithModelMesh(const BuildCFDParams& params);

	const Block* getBlockPtr() const;
	Block* getBlockPtr();

	LAMBDA_CLIENT_DECLS

private:
	bool splitAtPointInner(Polyhedron& realCell, Polyhedron& referanceCell, const Vector3d& pt) const;

	void findSharpVertPierecPoints(size_t vertIdx, MTC::vector<Vector3d>& piercePoints, const BuildCFDParams& params) const;
	void sortNewFacePoints(const Vector3d& tipPt, const Vector3d& xAxis, const Vector3d& yAxis, MTC::vector<Vector3d>& points) const;
	bool splitWithPlane(const Planed& plane, MTC::set<Index3DId>& newCellIds);

	bool createCellsFromFaces(MTC::set<Index3DId>& faceIds, MTC::set<Index3DId>& newCellIds);
	bool createConvexCells(const MTC::set<Index3DId>& cellFaces, MTC::set<Index3DId>& newCellIds);
	bool cutWithModelMeshInner(const BuildCFDParams& params, MTC::set<Index3DId>& deadCellIds, MTC::set<Index3DId>& newCellIds);
	bool createModelMeshPatches(const BuildCFDParams& params, std::vector<TriMesh::PatchPtr>& patches,
		std::vector<std::vector<std::vector<size_t>>>& allChains) const;
	bool cutWithPatches(const Polyhedron& realCell, const std::vector<TriMesh::PatchPtr>& patches, const BuildCFDParams& params, MTC::set<Index3DId>& newCellIds);
	void createModelFaceInterectionEdges(const Index3DId& faceId, const std::vector<size_t>& modFaceTris, const BuildCFDParams& params,
		MTC::set<IntersectEdge>& faceEdges);
	bool createAllModelMeshFaces(const std::vector<TriMesh::PatchPtr>& patches, const BuildCFDParams& params, MTC::set<Index3DId>& modelFaces);
	void createAllFaceEdges(const Polyhedron& realCell, const std::vector<size_t>& modFaceTris, const BuildCFDParams& params,
		MTC::set<Index3DId>& skippedVerts, MTC::set<Edge>& faceEdges) const;

	void createAllPatchFaceEdges_deprecated(const Polyhedron& realCell, const TriMesh::PatchPtr& pPatch, const BuildCFDParams& params,
		MTC::set<Index3DId>& skippedVerts, MTC::vector<MTC::set<Edge>>& patchEdges) const;
	void createFaceEdgesFromMeshFace(const Polyhedron& realCell, const std::vector<size_t>& modelFaceTris, const BuildCFDParams& params,
		MTC::set<Edge>& patchEdges) const;

	void createPierceEdges(const Polyhedron& realCell, const std::vector<size_t>& sharpEdges, MTC::set<Edge>& pierceEdges) const;
	bool findPiercePoint(const Polygon& face, const std::vector<size_t>& pierceChain, Vector3d& pt) const;
	bool splitWithSharpEdgePlanes(std::vector<std::vector<std::vector<size_t>>>& allChains, MTC::set<Index3DId>& newCellIds);
	bool findPiercePoints(const std::vector<std::vector<size_t>>& faceEdgeChains, std::vector<Vector3d>& pts) const;
	bool findPiercePoints(const std::vector<std::vector<std::vector<size_t>>>& allFaceEdgeChains, std::vector<Vector3d>& pts) const;
	IntersectVertId createPierceVertex(const Polygon& face, const std::vector<size_t>& pierceChain) const;
	bool facesFormClosedCell(const MTC::set<Index3DId>& faceIds) const;
	// Outside is relative to the model/patch, not the cell or face.
	Vector3d calModelFaceNormal(const std::vector<size_t>& modelFaceTris) const;

	Block* _pBlock;
	Index3DId _polyhedronId;
};

inline const Block* PolyhedronSplitter::getBlockPtr() const
{
	return _pBlock;
}

inline Block* PolyhedronSplitter::getBlockPtr()
{
	return _pBlock;
}

}
