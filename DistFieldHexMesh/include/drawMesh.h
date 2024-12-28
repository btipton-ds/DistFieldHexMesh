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

#include <memory>

#include <defines.h>
#include <tm_vector3.h>
#include <triMesh.h>
#include <graphicsVBORec.h>
#include <OGLMath.h>
#include <OGLMultiVboHandler.h>
#include <OGLExtensions.h>
#include <OGLShader.h>

namespace OGL
{
class Shader;
struct Indices;
using IndicesPtr = std::shared_ptr<Indices>;
}

namespace DFHM {
	class GraphicsCanvas;

	class DrawMesh : public OGL::Extensions {
	public:
		struct ChangeElementsOptions {
			bool
				showSharpEdges = false,
				showSharpVerts = false,
				showTriNormals = false,
				showEdges = true,
				showFaces = true,
				showModelBoundary = false,
				showOuter = false,
				showCurvature = false,
				showSelectedBlocks = false;
		};

		DrawMesh(GraphicsCanvas* pCanvas);
		virtual ~DrawMesh();

		void setShader(std::shared_ptr<OGL::Shader>& pShader);
		std::shared_ptr<VBORec> getVBOs();
		const std::shared_ptr<VBORec> getVBOs() const;

		void render();
		void drawEdges();
		void drawFaces();

	protected:
		virtual OGL::MultiVBO::DrawVertexColorMode preDrawEdges(int key);
		virtual OGL::MultiVBO::DrawVertexColorMode preDrawFaces(int key);

		virtual void postDrawEdges();
		virtual void postDrawFaces();

		virtual OGL::MultiVBO::DrawVertexColorMode preTexDraw(int key);
		virtual void postTexDraw();

		GraphicsCanvas* _pCanvas;
		const OGL::IndicesPtr _pTriTess, _pEdgeTess, _pSharpVertTess, _pSharpEdgeTess, _pNormalTess;
		std::vector<std::vector<OGL::IndicesPtr>> _faceTessellations, _edgeTessellations;

		std::shared_ptr<VBORec> _VBOs;
	};

	inline std::shared_ptr<VBORec> DrawMesh::getVBOs()
	{
		return _VBOs;
	}

	inline const std::shared_ptr<VBORec> DrawMesh::getVBOs() const
	{
		return _VBOs;
	}

}