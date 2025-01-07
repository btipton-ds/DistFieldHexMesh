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

#include <drawMesh.h>

namespace OGL
{
	class Shader;
}

namespace DFHM {
	class MeshData;
	using MeshDataPtr = std::shared_ptr<MeshData>;

	struct ModelViewOptions {
		bool
			showSharpEdges = false,
			showTriNormals = false,
			showSharpVerts = false,
			showEdges = false,
			showFaces = true,
			showCurvature = false;
	};

	class DrawModelMesh : public DrawMesh {
	public:
		DrawModelMesh(GraphicsCanvas* pCanvas);
		virtual ~DrawModelMesh();

		void changeViewElements(MeshDataPtr& pData);

		bool showSharpEdges() const;
		bool toggleShowSharpEdges();

		bool showSharpVerts() const;
		bool toggleShowSharpVerts();

		bool showTriNormals() const;
		bool toggleShowTriNormals();

		bool showFaces() const;
		bool toggleShowFaces();

		bool showCurvature() const;
		bool toggleShowCurvature();

		bool showEdges() const;
		bool toggleShowEdges();

	protected:
		OGL::MultiVBO::DrawVertexColorMode preDrawEdges(int key) override;
		void postDrawEdges() override;

		OGL::MultiVBO::DrawVertexColorMode preDrawFaces(int key) override;
		void postDrawFaces() override;

	private:
		ModelViewOptions _options;
	};

	inline bool DrawModelMesh::showSharpEdges() const
	{
		return _options.showSharpEdges;
	}

	inline bool DrawModelMesh::toggleShowSharpEdges()
	{
		return toggle(_options.showSharpEdges);
	}

	inline bool DrawModelMesh::showSharpVerts() const
	{
		return _options.showSharpVerts;
	}

	inline bool DrawModelMesh::toggleShowSharpVerts()
	{
		return toggle(_options.showSharpVerts);
	}

	inline bool DrawModelMesh::showTriNormals() const
	{
		return _options.showTriNormals;
	}

	inline bool DrawModelMesh::toggleShowTriNormals()
	{
		return toggle(_options.showTriNormals);
	}

	inline bool DrawModelMesh::showFaces() const
	{
		return _options.showFaces;
	}

	inline bool DrawModelMesh::toggleShowFaces()
	{
		return toggle(_options.showFaces);
	}

	inline bool DrawModelMesh::showCurvature() const
	{
		return _options.showCurvature;
	}

	inline bool DrawModelMesh::toggleShowCurvature()
	{
		return toggle(_options.showCurvature);
	}

	inline bool DrawModelMesh::showEdges() const
	{
		return _options.showEdges;
	}

	inline bool DrawModelMesh::toggleShowEdges()
	{
		return toggle(_options.showEdges);
	}

}