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
#include <fstream>

#include <drawCrossSectionEdges.h>
#include <splitter2D.h>
#include <graphicsCanvas.h>
#include <rgbaColor.h>

using namespace std;
using namespace DFHM;

DrawCrossSectionEdges::DrawCrossSectionEdges(GraphicsCanvas* pCanvas)
	: DrawMesh(pCanvas, 50)
{

}

DrawCrossSectionEdges::~DrawCrossSectionEdges()
{

}

size_t DrawCrossSectionEdges::numBytes() const
{
	size_t result = DrawMesh::numBytes();
	result += _points.size() * sizeof(float);
	result += _colors.size() * sizeof(float);
	for (int i = 0; i < 3; i++)
		result += _indices[i].size() * sizeof(unsigned int);

	for (const auto& p : _tessellations) {
		if (p)
			result += p->numBytes();
	}

	return result;
}

void DrawCrossSectionEdges::changeViewElements()
{
	auto& edgeVBO = _VBOs->_edgeVBO;

	edgeVBO.beginSettingElementIndices(0xffffffffffffffff);

	includeElements(edgeVBO, _tessellations);

	edgeVBO.endSettingElementIndices();

}

void DrawCrossSectionEdges::buildTables(const SplittingParams& params, const std::vector<Splitter2DPtr>* crossSections, size_t idx0, size_t idx1)
{
	if (!crossSections)
		return;
	_points.clear();
	_colors.clear();
	unsigned int idx = 0;
	for (int axisNum = 0; axisNum < 3; axisNum++) {
		_indices[axisNum].clear();
		if (!crossSections[axisNum].empty()) {
			vector<Vector3d> points;
			vector<double> curvatures;

			auto& sections = crossSections[axisNum];
			size_t start = 0, end = sections.size();
			if (idx0 != -1 && idx0 < sections.size())
				start = idx0;

			if (idx1 != -1 && idx1 <= sections.size() && idx1 > idx0)
				end = idx1;

			for (size_t sectionNum = start; sectionNum < end; sectionNum++) {
				auto& pSection = sections[sectionNum];
				if (pSection)
					pSection->getPointCurvatures(params, points, curvatures);
			}
			for (size_t i = 0; i < points.size(); i++) {
				_indices[axisNum].push_back(idx++);
				for (int j = 0; j < 3; j++) {
					_points.push_back(float(points[i][j]));
				}
				rgbaColor c = curvatureToColor(curvatures[i]);
				for (int j = 0; j < 3; j++)
					_colors.push_back(c._rgba[j] / 255.0f);
			}
		}
	}
}

void DrawCrossSectionEdges::copyTablesToVBOs()
{
	auto& edgeVBO = _VBOs->_edgeVBO;
	edgeVBO.beginEdgeTesselation();
	if (!_indices->empty()) {
		_tessellations.resize(3);

		// Setup the VBOs for all sections in one VBO
		vector<unsigned int> indices(_indices[0]);
		indices.insert(indices.end(), _indices[1].begin(), _indices[1].end());
		indices.insert(indices.end(), _indices[2].begin(), _indices[2].end());
		_allTessellations = edgeVBO.setEdgeSegTessellation(DS_INTERSECTION_ALL, 0, _points, _colors, indices);

		for (int i = 0; i < 3; i++) {
			if (!_indices[i].empty()) {
				DrawStates ds;
				switch (i) {
				case 0:
					ds = DS_INTERSECTION_X;
					break;
				case 1:
					ds = DS_INTERSECTION_Y;
					break;
				case 2:
					ds = DS_INTERSECTION_Z;
					break;
				}

				// Create a sub tessellation just for this axis' indices.
				// No other data is duplicated
				_tessellations[i] = edgeVBO.setEdgeSegTessellation(ds, _allTessellations, _indices[i]);
			}
		}
	}
	edgeVBO.endEdgeTesselation();
}

void DrawCrossSectionEdges::writeGLObj(const std::string& fullPath) const
{
	ofstream out(fullPath);
	size_t nPts = _points.size() / 3;
	out << "#Vertices " << nPts << "\n";
	for (size_t i = 0; i < nPts; i++) {
		out << "v " << _points[3 * i + 0] << " " << _points[3 * i + 1] << " " << _points[3 * i + 2] << "\n";
	}

	size_t nEdges = nPts / 2;
	out << "#Edges " << nEdges << "\n";
	for (size_t i = 0; i < nEdges; i++) {
		out << "l ";
		size_t idx0 = 2 * i + 1;
		size_t idx1 = idx0 + 1;
		out << idx0 << " " << idx1 << "\n";
	}

}

OGL::MultiVBO::DrawVertexColorMode DrawCrossSectionEdges::preDrawEdges(int key)
{
	OGL::MultiVBO::DrawVertexColorMode result = OGL::MultiVBO::DrawVertexColorMode::DRAW_COLOR;

	DrawStates ds = (DrawStates)key;

	auto& UBO = _pCanvas->getUBO();

	_priorNormalShadingOn = UBO.normalShadingOn;
	UBO.normalShadingOn = 0;

	glLineWidth(1.0f);
	UBO.useDefColor = 1;
	UBO.defColor = p4f(1.0f, 0.0f, 1.0f, 1.0f);

	glBufferData(GL_UNIFORM_BUFFER, sizeof(UBO), &UBO, GL_DYNAMIC_DRAW);

	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LESS);

	UBO.normalShadingOn = _priorNormalShadingOn;
	return result;
}

void DrawCrossSectionEdges::postDrawEdges()
{

}

OGL::MultiVBO::DrawVertexColorMode DrawCrossSectionEdges::preDrawFaces(int key)
{
	return OGL::MultiVBO::DRAW_COLOR_SKIP;
}

void DrawCrossSectionEdges::postDrawFaces()
{

}

DrawStates DrawCrossSectionEdges::faceTypeToDrawState(IntersectionDrawType idt)
{
    switch (idt) {
		default:
		case IDT_INTERSECTION_X:
			return DS_INTERSECTION_X;
		case IDT_INTERSECTION_Y:
			return DS_INTERSECTION_Y;
		case IDT_INTERSECTION_Z:
			return DS_INTERSECTION_Z;
	}
}

void DrawCrossSectionEdges::includeElements(OGL::MultiVboHandler& VBO, std::vector<OGL::IndicesPtr>& tess) const
{
	if (tess.empty())
		return;

	if (_options.showX && tess[0])
		VBO.includeElementIndices(DS_INTERSECTION_X, tess[0]);

	if (_options.showY && tess[1])
		VBO.includeElementIndices(DS_INTERSECTION_Y, tess[1]);

	if (_options.showZ && tess[2])
		VBO.includeElementIndices(DS_INTERSECTION_Z, tess[2]);
}

void DrawCrossSectionEdges::clearPrior()
{
	// Clear all graphics memory
	_allTessellations = nullptr;
	_tessellations.clear();
}

void DrawCrossSectionEdges::clearPost()
{
	// Now that it's copied, clear all the local caches.
	_points.clear();
	_colors.clear();
	for (int i = 0; i < 3; i++)
		_indices[i].clear();
}
