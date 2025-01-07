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

#define DECL_OPTS(NAME) \
	bool show##NAME() const; \
	bool toggleShow##NAME();

#define IMPL_OPTS(NAME) \
	inline bool DrawHexMesh::show##NAME() const \
	{\
		return _options.show##NAME;\
	}\
	inline bool DrawHexMesh::toggleShow##NAME()\
	{\
		return toggle(_options.show##NAME);\
	}

namespace DFHM {
	struct HexMeshViewOptions {
		bool
			showEdges = true,
			showFaces = false,
			showBoundary = false,
			showWalls = false,

			showBack = false,
			showFront = false,
			showBottom = false,
			showTop = false,
			showLeft = false,
			showRight = false,

			showSelectedBlocks = false;
	};


	class DrawHexMesh : public DrawMesh {
	public:
		DrawHexMesh(GraphicsCanvas* pCanvas);
		virtual ~DrawHexMesh();

		void setFaceTessellations(const std::vector<std::vector<OGL::IndicesPtr>>& src);
		void setEdgeTessellations(const std::vector<std::vector<OGL::IndicesPtr>>& src);
		void changeViewElements();

		DECL_OPTS(Edges)
		DECL_OPTS(Faces)
		DECL_OPTS(Walls)
		DECL_OPTS(Boundary)

		DECL_OPTS(Back)
		DECL_OPTS(Front)
		DECL_OPTS(Bottom)
		DECL_OPTS(Top)
		DECL_OPTS(Left)
		DECL_OPTS(Right)
			
		void setShowSelectedBlocks(bool val);

	protected:
		OGL::MultiVBO::DrawVertexColorMode preDrawEdges(int key) override;
		void postDrawEdges() override;

		OGL::MultiVBO::DrawVertexColorMode preDrawFaces(int key) override;
		void postDrawFaces() override;

	private:
		HexMeshViewOptions _options;
		std::vector<std::vector<OGL::IndicesPtr>> _faceTessellations, _edgeTessellations;
	};

	inline void DrawHexMesh::setFaceTessellations(const std::vector<std::vector<OGL::IndicesPtr>>& src)
	{
		_faceTessellations = src;
	}

	inline void DrawHexMesh::setEdgeTessellations(const std::vector<std::vector<OGL::IndicesPtr>>& src)
	{
		_edgeTessellations = src;
	}

	IMPL_OPTS(Edges)
	IMPL_OPTS(Faces)
	IMPL_OPTS(Walls)
	IMPL_OPTS(Boundary)

	IMPL_OPTS(Back)
	IMPL_OPTS(Front)
	IMPL_OPTS(Bottom)
	IMPL_OPTS(Top)
	IMPL_OPTS(Left)
	IMPL_OPTS(Right)

	inline void DrawHexMesh::setShowSelectedBlocks(bool val)
	{
		_options.showSelectedBlocks = val;
	}

}