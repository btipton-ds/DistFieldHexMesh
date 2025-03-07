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

#include <defines.h>
#include <memory>

#include <tm_vector3.h>
#include <triMesh.h>
#include <graphicsVBORec.h>
#include <OGLMath.h>
#include <OGLMultiVboHandler.h>
#include <OGLExtensions.h>
#include <drawModelMesh.h>

namespace OGL
{
	class Shader;
}

namespace DFHM {

	class TriMeshIndex {
	public:
		TriMeshIndex(size_t meshIdx = -1, size_t triIdx = -1);
		TriMeshIndex(const TriMeshIndex& src) = default;

		size_t getMeshIdx() const;
		size_t getTriIdx() const;

		bool operator < (const TriMeshIndex& rhs) const;
		bool operator == (const TriMeshIndex& rhs) const;

	private:
		size_t _meshIdx = -1;
		size_t _triIdx = -1;
	};

	class DrawModelMesh;
	struct SplittingParams;

	class AppData;

	class MeshData;
	using MeshDataPtr = std::shared_ptr<MeshData>;

	class MeshData {
	public:
		MeshData(const AppData* pAppData);
		MeshData(const AppData* pAppData, const TriMesh::CMeshPtr& _pMesh, const std::wstring& name);
		virtual ~MeshData();

		void clear();

		size_t numBytes() const;

		const TriMesh::CMeshPtr& getMesh() const;
		const std::wstring& getName() const;

		void splitLongTris(const SplittingParams& params, double maxLength);

		void write(std::ostream& out) const;
		void read(std::istream& in);

		void makeOGLTess(std::shared_ptr<DrawModelMesh>& pDrawModelMesh);
		void changeViewElements(std::shared_ptr<DrawModelMesh>& pDraw);

		bool isActive() const;
		void setActive(bool val);

		const OGL::IndicesPtr getFaceTess() const;
		const OGL::IndicesPtr getAllEdgeTess() const;
		const OGL::IndicesPtr getSmoothEdgeTess() const;
		const OGL::IndicesPtr getSharpEdgeTess() const;
		const OGL::IndicesPtr getNormalTess() const;

	private:

		// vertiIndices is index pairs into points, normals and parameters to form triangles. It's the standard OGL element index structure
		const OGL::IndicesPtr createFaceTessellation(const TriMesh::CMeshPtr& pMesh, std::shared_ptr<DrawModelMesh>& _pDrawModelMesh);

		// vertiIndices is index pairs into points, normals and parameters to form triangles. It's the standard OGL element index structure
		void setEdgeSegTessellation(const TriMesh::CMeshPtr& pMesh, std::shared_ptr<DrawModelMesh>& pDrawModelMesh);

		void getEdgeData(std::vector<float>& normPts, std::vector<unsigned int>& normIndices) const;

		void addPointMarker(TriMesh::CMeshPtr& pMesh, const Vector3d& pt, double radius) const;
		TriMesh::CMeshPtr getSharpVertMesh() const;

		std::wstring getCacheFilename() const;
		bool isMeshCashed() const;
		void cacheMesh();
		void readMeshFromCache();

		bool _active = true;

		const AppData* _pAppData;
		std::wstring _name;
		TriMesh::CMeshPtr _pMesh;

		OGL::IndicesPtr 
			_faceTess, 
			_allEdgeTess,
			_smoothEdgeTess,
			_sharpEdgeTess,
			_normalTess, 
			_sharpPointTess;
	};

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

	inline TriMeshIndex::TriMeshIndex(size_t meshIdx, size_t triIdx)
		: _meshIdx(meshIdx)
		, _triIdx(triIdx)
	{
	}

	inline size_t TriMeshIndex::getMeshIdx() const
	{
		return _meshIdx;
	}

	inline size_t TriMeshIndex::getTriIdx() const
	{
		return _triIdx;
	}

	inline bool TriMeshIndex::operator < (const TriMeshIndex& rhs) const
	{
		if (_meshIdx < rhs._meshIdx)
			return true;
		else if (_meshIdx > rhs._meshIdx)
			return false;

		return _triIdx < rhs._triIdx;
	}

	inline bool TriMeshIndex::operator == (const TriMeshIndex& rhs) const
	{
		return _meshIdx == rhs._meshIdx && _triIdx == rhs._triIdx;
	}

}