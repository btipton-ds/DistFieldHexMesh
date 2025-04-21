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

#include <stdint.h>
#include <vector>
#include <map>
#include <mutex>
#include <MultiCoreUtil.h>
#include <index3D.h>
#include <triMesh.h>
#include <tm_spatialSearch.h>
#include <unalignedBBox.h>
#include <objectPool.h>
#include <polygon.h>
#include <polyhedron.h>
#include <block.h>
#include <polymeshTables.h>
#include <progressReporter.h>

namespace TriMesh {
	class CMesh;
	using CMeshPtr = std::shared_ptr<CMesh>;
}

namespace DFHM {

	struct SplittingParams;

	class AppData;
	using AppDataPtr = std::shared_ptr<AppData>;

	class Model;
	using ModelPtr = std::shared_ptr<Model>;

	class PolyMesh;
	using PolyMeshPtr = std::shared_ptr<PolyMesh>;

	class PolyMesh : public ObjectPoolOwner {
	public:
		friend class Block;
		PolyMesh();
		PolyMesh(const TriMesh::CMeshPtr& srcMesh);
		~PolyMesh();

		const Index3D& getBlockIdx() const override;

		Volume* getVolume() override;
		const Volume* getVolume() const override;

		const Block* getOwner(const Index3D& blockIdx) const override;
		Block* getOwner(const Index3D& blockIdx) override;

		const PolyMesh* getPolyMeshPtr() const override;
		PolyMesh* getPolyMeshPtr() override;

		const Vector3d& getVertexPoint(const Index3DId& id) const;
		void makeQuads();
		void removeFace(const Index3DId& id);

		LAMBDA_BLOCK_DECLS(vertex, Index3DId, Vertex)
		LAMBDA_BLOCK_DECLS(face, Index3DId, Polygon)
		LAMBDA_BLOCK_DECLS(cell, Index3DId, Polyhedron)
		LAMBDA_BLOCK_DECLS(edge, EdgeKey, Edge)

	private:
		bool isLongestEdge(const Polygon& face, const Edge& edge) const;
		void mergeToQuad(const Edge& edge);

		CMesh::BoundingBox _modelBoundingBox;

		ObjectPool<Vertex> _vertices;
		ObjectPool<Polygon> _polygons;
	};

	using VolumePtr = std::shared_ptr<Volume>;


} // end namespace DFHM
