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
#include <string>

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

class Splitter2D;
using Splitter2DPtr = std::shared_ptr<Splitter2D>;

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

	size_t numBytes() const override;

	void changeViewElements();

	DECL_OPTS(X)
	DECL_OPTS(Y)
	DECL_OPTS(Z)

		void buildTables(const SplittingParams& params, const std::vector<Splitter2DPtr>* crossSections, size_t idx0 = -1, size_t idx1 = -1);
	void copyTablesToVBOs();

	void writeGLObj(const std::string& fullPath) const;

protected:
	OGL::MultiVBO::DrawVertexColorMode preDrawEdges(int key) override;
	void postDrawEdges() override;

	OGL::MultiVBO::DrawVertexColorMode preDrawFaces(int key) override;
	void postDrawFaces() override;

private:
	static DrawStates faceTypeToDrawState(IntersectionDrawType idt);
	void includeElements(OGL::MultiVboHandler& VBO, std::vector<OGL::IndicesPtr>& tess) const;

	void clearPrior();
	void clearPost();

	DrawCrossSectionEdgesOptions _options;

	std::vector<float> _points, _colors;
	std::vector<unsigned int> _indices[3];
	OGL::IndicesPtr _allTessellations;
	std::vector<OGL::IndicesPtr> _tessellations;
};

inline bool DrawCrossSectionEdges::showX() const {
	return _options.showX;
} 

inline bool DrawCrossSectionEdges::toggleShowX() {
	return toggle(_options.showX);
}
inline bool DrawCrossSectionEdges::showY() const {
	return _options.showY;
} 

inline bool DrawCrossSectionEdges::toggleShowY() {
	return toggle(_options.showY);
}
inline bool DrawCrossSectionEdges::showZ() const {
	return _options.showZ;
} 

inline bool DrawCrossSectionEdges::toggleShowZ() {
	return toggle(_options.showZ);
}

}

#undef DECL_OPTS
#undef IMPL_OPTS
