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

#include <memory>
#include <splitParams.h>
#include <divideHexMeshDlg.h>
#include <wx/string.h>
#include <wx/wfstream.h>
#include <filesystem>
#include <block.h>
#include <vertex.h>
#include <volume.h>

#ifdef WIN32
#include "windows.h"
#endif // WIN32

using namespace std;
using namespace DFHM;

namespace
{
	enum Ids {
		UNIFORM_RATIO_ID = 1,
		SPLIT_SHARP_VERTS_ID = 1,
		MAX_SPLIT_FACES_ID,
		MAX_GAP_SIZE_ID,
		IGNORE_CURVATURE_RADIUS_ID,
		MIN_EDGE_LENGTH_ID,
		MIN_EDGE_LENGTH_GAP_ID,
		SHARP_ANGLE_DEGREES_ID,
		NUM_SIMPLE_DIVS_ID,
		NUM_INTERSECTION_DIVS_ID,
		NUM_SHARP_EDGE_INTERSECTION_DIVS_ID,
		NUM_CURVATURE_DIVS_ID,
		NUM_DIVS_PER_RADIUS_ID,
		NUM_DIVS_GAP_PER_RADIUS_ID,
	};
}

DivideHexMeshDlg::DivideHexMeshDlg(SplittingParams& params, wxWindow* parent, wxWindowID id, const wxString& title, const wxPoint& pos)
	: wxDialog(parent, id, title, pos, wxSize(400, 460), wxDEFAULT_DIALOG_STYLE, wxString("Divide Mesh"))
{

#ifdef WIN32
	int gap = 3;
	int descent = 3;
	int promptWidth = 140;
	int boxWidth = 80;
	int boxHeight = 21;
	int rowHeight = boxHeight + descent;
	int col0 = 8;
	int col1 = col0 + promptWidth + gap;
	int col2 = col1 + promptWidth + gap;
	int col3 = col2 + promptWidth + gap;
	int baseRowPixels = 5;
#else
	int gap = 3;
	int descent = 3;
	int promptWidth = 50;
	int boxWidth = 20;
	int boxHeight = 21;
	int rowHeight = boxHeight + descent;
	int col0 = 8;
	int col1 = col0 + promptWidth + gap;
	int col2 = col1 + promptWidth + gap;
	int col3 = col2 + promptWidth + gap;
	int baseRowPixels = 5;
#endif

	int rowNum = 0;
	_uniformRatioCheckBox = new wxCheckBox(this, UNIFORM_RATIO_ID, _T("Uniform cells per side"), wxPoint(col0, baseRowPixels + rowNum * rowHeight), wxSize(promptWidth, boxHeight));
	_uniformRatioCheckBox->SetValue(params.uniformRatio);

	rowNum++;
	_splitSharpVertsCheckBox = new wxCheckBox(this, SPLIT_SHARP_VERTS_ID, _T("Split sharp verts"), wxPoint(col0, baseRowPixels + rowNum * rowHeight), wxSize(promptWidth, boxHeight));
	_splitSharpVertsCheckBox->SetValue(params.splitAtSharpVerts);

	rowNum++;
	_maxGapSizePrompt = new wxStaticText(this, 0, _T("Max gap size"), wxPoint(col0, baseRowPixels + rowNum * rowHeight), wxSize(promptWidth, boxHeight));
	_maxGapSizeBox = new wxTextCtrl(this, MAX_GAP_SIZE_ID, std::to_string(params.maxGapSize), wxPoint(col1, baseRowPixels + rowNum * rowHeight - descent), wxSize(boxWidth, boxHeight), wxTE_RIGHT);

	rowNum++;
	_maxCurvatureRadiusPrompt = new wxStaticText(this, 0, _T("Don't split radii >"), wxPoint(col0, baseRowPixels + rowNum * rowHeight), wxSize(promptWidth, boxHeight));
	_ignoreCurvatureRadius_metersBox = new wxTextCtrl(this, IGNORE_CURVATURE_RADIUS_ID, std::to_string(params.ignoreCurvatureRadius_meters), wxPoint(col1, baseRowPixels + rowNum * rowHeight - descent), wxSize(boxWidth, boxHeight), wxTE_RIGHT);

	rowNum++;
	_minSplitEdgeLengthCurvaturePrompt = new wxStaticText(this, 0, _T("Min edge length"), wxPoint(col0, baseRowPixels + rowNum * rowHeight), wxSize(promptWidth, boxHeight));
	_minSplitEdgeLengthCurvatureBox = new wxTextCtrl(this, MIN_EDGE_LENGTH_ID, std::to_string(params.minSplitEdgeLengthCurvature_meters), wxPoint(col1, baseRowPixels + rowNum * rowHeight - descent), wxSize(boxWidth, boxHeight), wxTE_RIGHT);

	rowNum++;
	_minSplitEdgeLengthGapCurvaturePrompt = new wxStaticText(this, 0, _T("Min gap edge length"), wxPoint(col0, baseRowPixels + rowNum * rowHeight), wxSize(promptWidth, boxHeight));
	_minSplitEdgeLengthGapCurvatureBox = new wxTextCtrl(this, MIN_EDGE_LENGTH_GAP_ID, std::to_string(params.minSplitEdgeLengthGapCurvature_meters), wxPoint(col1, baseRowPixels + rowNum * rowHeight - descent), wxSize(boxWidth, boxHeight), wxTE_RIGHT);

	rowNum++;
	_sharpAnglePrompt = new wxStaticText(this, 0, _T("Sharp angle degrees"), wxPoint(col0, baseRowPixels + rowNum * rowHeight), wxSize(promptWidth, boxHeight));
	_sharpAngleBox = new wxTextCtrl(this, SHARP_ANGLE_DEGREES_ID, std::to_string(params.sharpAngle_degrees), wxPoint(col1, baseRowPixels + rowNum * rowHeight - descent), wxSize(boxWidth, boxHeight), wxTE_RIGHT);

	rowNum++;
	_maxCellFacesPrompt = new wxStaticText(this, 0, _T("Max faces per cell"), wxPoint(col0, baseRowPixels + rowNum * rowHeight), wxSize(promptWidth, boxHeight));
	_maxCellFacesBox = new wxTextCtrl(this, MAX_SPLIT_FACES_ID, std::to_string(params.maxCellFaces), wxPoint(col1, baseRowPixels + rowNum * rowHeight - descent), wxSize(boxWidth, boxHeight), wxTE_RIGHT);

	rowNum++;
	_numSimpleDivsPrompt = new wxStaticText(this, 0, _T("# simple divs"), wxPoint(col0, baseRowPixels + rowNum * rowHeight), wxSize(promptWidth, boxHeight));
	_numSimpleDivsBox = new wxTextCtrl(this, NUM_SIMPLE_DIVS_ID, std::to_string(params.numSimpleDivs), wxPoint(col1, baseRowPixels + rowNum * rowHeight - descent), wxSize(boxWidth, boxHeight), wxTE_RIGHT);

	rowNum++;
	_numIntersectionDivsPrompt = new wxStaticText(this, 0, _T("# intersection divs"), wxPoint(col0, baseRowPixels + rowNum * rowHeight), wxSize(promptWidth, boxHeight));
	_numIntersectionDivsBox = new wxTextCtrl(this, NUM_INTERSECTION_DIVS_ID, std::to_string(params.numIntersectionDivs), wxPoint(col1, baseRowPixels + rowNum * rowHeight - descent), wxSize(boxWidth, boxHeight), wxTE_RIGHT);

	rowNum++;
	_numSharpEdgeIntersectionDivsPrompt = new wxStaticText(this, 0, _T("# sharp edge intersection divs"), wxPoint(col0, baseRowPixels + rowNum * rowHeight), wxSize(promptWidth, boxHeight));
	_numSharpEdgeIntersectionDivs = new wxTextCtrl(this, NUM_SHARP_EDGE_INTERSECTION_DIVS_ID, std::to_string(params.numSharpEdgeIntersectionDivs), wxPoint(col1, baseRowPixels + rowNum * rowHeight - descent), wxSize(boxWidth, boxHeight), wxTE_RIGHT);

	rowNum++;
	_numCurvatureDivsPrompt = new wxStaticText(this, 0, _T("# curavture divs"), wxPoint(col0, baseRowPixels + rowNum * rowHeight), wxSize(promptWidth, boxHeight));
	_numCurvatureDivsBox = new wxTextCtrl(this, NUM_CURVATURE_DIVS_ID, std::to_string(params.numCurvatureDivs), wxPoint(col1, baseRowPixels + rowNum * rowHeight - descent), wxSize(boxWidth, boxHeight), wxTE_RIGHT);

	rowNum++;
	_divsPerCurvaturePrompt = new wxStaticText(this, 0, _T("curv divs / circ"), wxPoint(col0, baseRowPixels + rowNum * rowHeight), wxSize(promptWidth, boxHeight));
	_divsPerCurvatureBox = new wxTextCtrl(this, NUM_DIVS_PER_RADIUS_ID, std::to_string(params.curvatureDivsPerCircumference), wxPoint(col1, baseRowPixels + rowNum * rowHeight - descent), wxSize(boxWidth, boxHeight), wxTE_RIGHT);

	rowNum++;
	_divsPerGapCurvaturePrompt = new wxStaticText(this, 0, _T("# gap divs / radius"), wxPoint(col0, baseRowPixels + rowNum * rowHeight), wxSize(promptWidth, boxHeight));
	_divsPerGapCurvatureBox = new wxTextCtrl(this, NUM_DIVS_GAP_PER_RADIUS_ID, std::to_string(params.divsPerGapCurvatureRadius), wxPoint(col1, baseRowPixels + rowNum * rowHeight - descent), wxSize(boxWidth, boxHeight), wxTE_RIGHT);

	rowNum++;
	_okButton = new wxButton(this, wxID_CANCEL, _T("Cancel"), wxPoint(col1 + 100, baseRowPixels + rowNum * rowHeight));
	_cancelButton = new wxButton(this, wxID_OK, _T("OK"), wxPoint(col1, baseRowPixels + rowNum * rowHeight));
}

bool DivideHexMeshDlg::getValue(wxTextCtrl* item, size_t& value) const
{
	try {
		wxString wstr = item->GetValue();
		if (wstr.length() > 0) {
			value = stoi(wstr.c_str().AsChar());
			return true;
		}
	} catch (const invalid_argument& ) {
	}
	return false;
}

bool DivideHexMeshDlg::getValue(wxTextCtrl* item, double& value) const
{
	try {
		wxString wstr = item->GetValue();
		if (wstr.length() > 0) {
			value = stod(wstr.c_str().AsChar());
			return true;
		}
	} catch (const invalid_argument& ) {
	}
	return false;
}

void DivideHexMeshDlg::getParams(SplittingParams& params) const
{
	params.uniformRatio = _uniformRatioCheckBox->GetValue();
	params.splitAtSharpVerts = _splitSharpVertsCheckBox->GetValue();
	getValue(_numSimpleDivsBox, params.numSimpleDivs);
	getValue(_numIntersectionDivsBox, params.numIntersectionDivs);
	getValue(_numSharpEdgeIntersectionDivs, params.numSharpEdgeIntersectionDivs);
	getValue(_numCurvatureDivsBox, params.numCurvatureDivs);
	getValue(_divsPerCurvatureBox, params.curvatureDivsPerCircumference);
	getValue(_divsPerGapCurvatureBox, params.divsPerGapCurvatureRadius);
	getValue(_maxCellFacesBox, params.maxCellFaces);
	getValue(_maxGapSizeBox, params.maxGapSize);
	getValue(_ignoreCurvatureRadius_metersBox, params.ignoreCurvatureRadius_meters);
	getValue(_sharpAngleBox, params.sharpAngle_degrees);
	getValue(_minSplitEdgeLengthCurvatureBox, params.minSplitEdgeLengthCurvature_meters);
	getValue(_minSplitEdgeLengthGapCurvatureBox, params.minSplitEdgeLengthGapCurvature_meters);
}
