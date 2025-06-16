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

#include <tm_vector3.h>
#include <graphicsVBORec.h>
#include <OGLMath.h>
#include <OGLMultiVboHandler.h>
#include <OGLExtensions.h>
#include <drawModelMesh.h>
#include <polyMesh.h>

namespace OGL
{
	class Shader;
}

namespace DFHM {

	class DrawModelMesh;
	struct SplittingParams;

	class AppData;

	class PolyMesh;
	using PolyMeshPtr = std::shared_ptr<PolyMesh>;

	class MeshData;
	using MeshDataPtr = std::shared_ptr<MeshData>;

	class MeshData {
	public:
		MeshData(const AppDataPtr& pAppData);
		MeshData(const AppDataPtr& pAppData, const TriMesh::CMeshPtr& _pMesh, const std::wstring& name);
		virtual ~MeshData();

		void clear();

		size_t numBytes() const;
		size_t getId() const;
		CBoundingBox3Dd getBBox() const;

		const TriMesh::CMeshPtr& getMesh() const;
		const std::wstring& getName() const;

		void splitLongTris(const SplittingParams& params, double maxLength);
		void getEdgeData(std::vector<float>& normPts, std::vector<unsigned int>& normIndices) const;

		void write(std::ostream& out) const;
		void read(std::istream& in);

		bool isActive() const;
		void setActive(bool val);

		const PolyMeshPtr& getPolyMesh() const;

		void setFaceTess(const OGL::IndicesPtr& faceTess);
		void setTessEdges(const OGL::IndicesPtr& allEdgeTess, const OGL::IndicesPtr& sharpEdgeTess, const OGL::IndicesPtr& smoothEdgeTess);
		void setTessNormals(const OGL::IndicesPtr& normalsTess);
		const OGL::IndicesPtr getFaceTess() const;
		const OGL::IndicesPtr getAllEdgeTess() const;
		const OGL::IndicesPtr getSmoothEdgeTess() const;
		const OGL::IndicesPtr getSharpEdgeTess() const;
		const OGL::IndicesPtr getNormalTess() const;

	private:
		friend class AppData;

		void postReadCreate();

		void addPointMarker(TriMesh::CMeshPtr& pMesh, const Vector3d& pt, double radius) const;
		TriMesh::CMeshPtr getSharpVertMesh() const;

		std::wstring getCacheFilename() const;
		bool isMeshCashed() const;
		void cacheMesh();
		void readMeshFromCache();

		AppDataPtr _pAppData;
		bool _active = true;
		size_t _id;

		std::wstring _name;
		TriMesh::CMeshPtr _pMesh;
		PolyMeshPtr _pPolyMesh;

		OGL::IndicesPtr 
			_faceTess, 
			_allEdgeTess,
			_smoothEdgeTess,
			_sharpEdgeTess,
			_normalTess, 
			_sharpPointTess;
	};

	inline size_t MeshData::getId() const
	{
		return _id;
	}

	inline const TriMesh::CMeshPtr& MeshData::getMesh() const
	{
		return _pMesh;
	}

	inline const std::wstring& MeshData::getName() const
	{
		return _name;
	}

	inline bool MeshData::isActive() const
	{
		return _active;
	}

	inline const OGL::IndicesPtr MeshData::getFaceTess() const
	{
		return _faceTess;
	}

	inline const OGL::IndicesPtr MeshData::getAllEdgeTess() const
	{
		return _allEdgeTess;
	}

	inline const OGL::IndicesPtr MeshData::getSmoothEdgeTess() const
	{
		return _smoothEdgeTess;
	}

	inline const OGL::IndicesPtr MeshData::getSharpEdgeTess() const
	{
		return _sharpEdgeTess;
	}

	inline const OGL::IndicesPtr MeshData::getNormalTess() const
	{
		return _normalTess;
	}

	inline const PolyMeshPtr& MeshData::getPolyMesh() const
	{
		return _pPolyMesh;
	}

}