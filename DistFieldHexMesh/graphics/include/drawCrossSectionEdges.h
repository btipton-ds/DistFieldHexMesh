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

#include <OGLMultiVbo.h>
#include <drawMesh.h>
#include <Index3D.h>
#include <block.h>

namespace OGL
{
	class Shader;
}

#define DECL_OPTS(NAME) \
	bool show##NAME() const; \
	bool toggleShow##NAME();

#define IMPL_OPTS(NAME) \
	inline bool DrawCrossSectionEdges::show##NAME() const \
	{\
		return _options.show##NAME;\
	}\
	inline bool DrawCrossSectionEdges::toggleShow##NAME()\
	{\
		return toggle(_options.show##NAME);\
	}

namespace DFHM {

class Volume;
using VolumePtr = std::shared_ptr<Volume>;

struct DrawCrossSectionEdgesOptions {
	bool
		showX = true,
		showY = true,
		showZ = true;
};

class DrawCrossSectionEdges : public DrawMesh {
public:
	DrawCrossSectionEdges(GraphicsCanvas* pCanvas);
	virtual ~DrawCrossSectionEdges();

	void setTessellations(const std::vector<OGL::IndicesPtr>& src);
	void changeViewElements();

	DECL_OPTS(X)
	DECL_OPTS(Y)
	DECL_OPTS(Z)
			
	void buildTables(const VolumePtr& pVolume, const Index3D& min, const Index3D& max, bool multiCore);
	void copyTablesToVBOs();

protected:
	OGL::MultiVBO::DrawVertexColorMode preDrawEdges(int key) override;
	void postDrawEdges() override;

	OGL::MultiVBO::DrawVertexColorMode preDrawFaces(int key) override;
	void postDrawFaces() override;

private:
	static DrawStates faceTypeToDrawState(FaceDrawType ft);
	static bool includeElementIndices(bool enabled, OGL::MultiVboHandler& VBO, FaceDrawType ft, std::vector<OGL::IndicesPtr>& tessellations);
	void includeElements(OGL::MultiVboHandler& VBO, std::vector<OGL::IndicesPtr>& tess) const;
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

	void createBlockMeshStorage(const Block::GlHexFacesVector& faces);

	void clearPrior();
	void clearPost();

	DrawCrossSectionEdgesOptions _options;

	std::vector<float> _points, _colors; // Appears to not being cleared
	std::vector<std::vector<unsigned int>> _indices;
	std::vector<OGL::IndicesPtr> _tessellations;
};

inline void DrawCrossSectionEdges::setTessellations(const std::vector<OGL::IndicesPtr>& src)
{
	_tessellations = src;
}

IMPL_OPTS(X)
IMPL_OPTS(Y)
IMPL_OPTS(Z)

}

#undef DECL_OPTS
#undef IMPL_OPTS
