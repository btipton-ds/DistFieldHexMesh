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
		PolyMesh(const AppDataPtr& pAppData);
		PolyMesh(const AppDataPtr& pAppData, const TriMesh::CMeshPtr& srcMesh);
		~PolyMesh();

		const Index3D& getBlockIdx() const override;

		Volume* getVolume() override;
		const Volume* getVolume() const override;

		const Block* getOwner(const Index3D& blockIdx) const override;
		Block* getOwner(const Index3D& blockIdx) override;

		const PolyMesh* getPolyMeshPtr() const override;
		PolyMesh* getPolyMeshPtr() override;

		const Vector3d& getVertexPoint(const Index3DId& id) const;
		void simplify(const SplittingParams& params, bool flattenQuads);
		void calCurvatures();
		double getEdgeCurvature(const EdgeKey& key) const;

		template<class FACE_FUNC>
		void iterateFaces(FACE_FUNC faceFunc) const;

		void getGlTriPoints(std::vector<float>& result) const;
		void getGlTriNormals(std::vector<float>& result) const;
		void getGlTriIndices(std::vector<unsigned int>& result) const;

		template<typename LAMBDA>
		void getGlEdges(LAMBDA& curvatureToColorFunc, bool includeSmooth, std::vector<float>& points, std::vector<float>& colors,
			double sinSharpAngle, std::vector<unsigned int>& sharpIndices, std::vector<unsigned int>& smoothIndices);

		LAMBDA_BLOCK_DECLS(vertex, Index3DId, Vertex)
		LAMBDA_BLOCK_DECLS(face, Index3DId, Polygon)
		LAMBDA_BLOCK_DECLS(cell, Index3DId, Polyhedron)
		LAMBDA_BLOCK_DECLS(edge, EdgeKey, Edge)

	private:
		void flattenSharps(const SplittingParams& params);
		void flattenEdgeLoop(const std::vector<Index3DId>& loop);
		void flattenFaces(const SplittingParams& params);
		void flattenFaces_deprecated(const SplittingParams& params);
		void makeQuads(const SplittingParams& params, bool flatten);
		void reduceSlivers(const SplittingParams& params, double maxSliverAngleRadians);
		Index3DId mergeToQuad(const SplittingParams& params, const Edge& edge);
		void makeCoplanarFaceSets(const Index3DId& radiantId, const FastBisectionSet<Index3DId>& faceIds, MTC::vector<MTC::vector<Index3DId>>& planarFaceSets);
		void processPlanarFaces(const SplittingParams& params, const Index3DId& radiantId, double minAngleRadians, const MTC::vector<Index3DId>& faceIds);
		void createSharpEdgeLoops(const SplittingParams& params, std::vector<std::shared_ptr<std::vector<Index3DId>>>& sharpLoops) const;
		Index3DId removeEdge(const EdgeKey& key, const Index3DId& faceId0, const Index3DId& faceId1, const MTC::vector<Index3DId>& newVertIds, const Planed& plane);
		Index3DId removeEdgeWithChecks(const SplittingParams& params, const Planed& plane, const Index3DId& radiantVertId, const Index3DId& otherVertId);
		bool mergeVertices(const EdgeKey& key, const Polygon& face0, const Polygon& face1, MTC::vector<Index3DId>& newVertIds) const;
		void removeFace(const Index3DId& id);
		double calEdgeAngle(const Index3DId& vertId, const Vector3d& origin, const Vector3d& xAxis, const Vector3d& yAxis) const;
		double calEdgeAngle(const Edge& edge) const;

		bool hasHighLocalConvexity(const SplittingParams& params, const Vector3d& norm, const MTC::vector<Index3DId>& vertIds) const;
		bool adjacentEdgesHaveSimilarLength(const EdgeKey& edgeKey) const;
		bool isShortEdge(const Edge& edge, const Polygon& face0, const Polygon& face1) const;
		bool isCoplanar(const SplittingParams& params, const Edge& edge) const;
		bool isRemovable(const SplittingParams& params, const EdgeKey& key) const;
		bool formsValidPolygon(const SplittingParams& params, const MTC::vector<Index3DId>& vertIds, const Vector3d& norm) const;
		bool formsValidQuad(const SplittingParams& params, const MTC::vector<Index3DId>& vertIds) const;

		Index3DId chooseSmoothestVert(const SplittingParams& params, const std::set<Index3DId>& conVerts, const std::vector<Index3DId>& loop) const;

		void dumpVertsAsPolygon(const std::string& path, const MTC::vector<Index3DId>& vertIds) const;
		void dumpVertsAsLineSegs(const std::string& path, const MTC::vector<Index3DId>& vertIds) const;
		void dumpFaceSetAsObj(const std::string& path, size_t num, const Index3DId& radiantId, const MTC::vector<Index3DId>& faceIds, const std::set<Index3DId>& uniqueVerts) const;

		AppDataPtr _pAppData;

		size_t _maxRemovableVerts;
		double _maxRemovableAreaRatio, _minAspectRatio = 5;

		ObjectPool<Vertex> _vertices;
		ObjectPool<Polygon> _polygons;
		MTC::map<EdgeKey, double> _edgeCurvatures;
	};

	template<class FACE_FUNC>
	inline void PolyMesh::iterateFaces(FACE_FUNC faceFunc) const {
		_polygons.iterateInOrder(faceFunc);
	}

	inline double PolyMesh::getEdgeCurvature(const EdgeKey& key) const
	{
		auto iter = _edgeCurvatures.find(key);
		if (iter != _edgeCurvatures.end())
			return iter->second;

		return 0;
	}

	template<typename LAMBDA>
	void PolyMesh::getGlEdges(LAMBDA& curvatureToColorFunc, bool includeSmooth, std::vector<float>& points, std::vector<float>& colors,
		double sinSharpAngle, std::vector<unsigned int>& sharpIndices, std::vector<unsigned int>& smoothIndices)
	{
		calCurvatures();
		size_t idx = 0;
		for (const auto& pair : _edgeCurvatures) {
			auto& ek = pair.first;
			double c = pair.second;
			edgeFunc(ek, [this, c, &curvatureToColorFunc, &idx, &points, &colors, &smoothIndices](const Edge& edge) {
				for (int i = 0; i < 2; i++) {
					auto& pt = getVertexPoint(edge[i]);
					double c = getEdgeCurvature(edge);
					float rgb[3] = { 0, 0, 0 };
					if (c > 0.001)
						curvatureToColorFunc(c, rgb);
					for (int j = 0; j < 3; j++) {
						points.push_back((float)pt[j]);
						colors.push_back(rgb[j]);
					}
					smoothIndices.push_back(idx++);
				}
			});
		}
	}

	using VolumePtr = std::shared_ptr<Volume>;

} // end namespace DFHM
