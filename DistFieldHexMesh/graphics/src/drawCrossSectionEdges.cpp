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
#include <drawCrossSectionEdges.h>

using namespace std;
using namespace DFHM;

DrawCrossSectionEdges::DrawCrossSectionEdges(GraphicsCanvas* pCanvas)
	: DrawMesh(pCanvas, 50)
{

}

DrawCrossSectionEdges::~DrawCrossSectionEdges()
{

}

void DrawCrossSectionEdges::changeViewElements()
{

}

void DrawCrossSectionEdges::buildTables(const VolumePtr& pVolume, const Index3D& min, const Index3D& max, bool multiCore)
{

}

void DrawCrossSectionEdges::copyTablesToVBOs()
{

}

OGL::MultiVBO::DrawVertexColorMode DrawCrossSectionEdges::preDrawEdges(int key)
{
	return OGL::MultiVBO::DRAW_COLOR;
}

void DrawCrossSectionEdges::postDrawEdges()
{

}

OGL::MultiVBO::DrawVertexColorMode DrawCrossSectionEdges::preDrawFaces(int key)
{
	return OGL::MultiVBO::DRAW_COLOR_NONE;
}

void DrawCrossSectionEdges::postDrawFaces()
{

}

DrawStates DrawCrossSectionEdges::faceTypeToDrawState(FaceDrawType ft)
{
	return DS_SECTION_X;
}

bool DrawCrossSectionEdges::includeElementIndices(bool enabled, OGL::MultiVboHandler& VBO, FaceDrawType ft, std::vector<OGL::IndicesPtr>& tessellations)
{
	return false;
}

void DrawCrossSectionEdges::includeElements(OGL::MultiVboHandler& VBO, std::vector<OGL::IndicesPtr>& tess) const
{

}

void DrawCrossSectionEdges::createBlockMeshStorage(const Block::GlHexFacesVector& faces)
{

}

void DrawCrossSectionEdges::clearPrior()
{
	_tessellations.clear();
}

void DrawCrossSectionEdges::clearPost()
{
	_points.clear();
	_colors.clear();
	_indices.clear();
}
