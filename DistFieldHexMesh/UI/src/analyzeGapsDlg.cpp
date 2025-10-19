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
#include <analyzeGapsDlg.h>
#include <wx/string.h>
#include <wx/wfstream.h>
#include <filesystem>
#include <appData.h>
#include <model.h>
#include <mainFrame.h>

#ifdef WIN32
#include "windows.h"
#endif // WIN32

using namespace std;
using namespace DFHM;

namespace
{
enum Ids {
	MAX_GAP_DIST = 1,
	GRID_SIZE,
};


#ifdef WIN32
int baseRowPixels = 5;
int boxHeight = 21;
int descent = 3;
int rowHeight = boxHeight + descent;
int frameWidth = 250;
int frameHeight = 5 * rowHeight + baseRowPixels;
int rightEdge = frameWidth - 20;
int gap = 2;
int promptWidth = 110;
int buttonWidth = 80;
int boxWidth = 80;
int boxWidth1 = 30;
int col0 = 8;
int col1 = col0 + promptWidth + gap;
#else
int baseRowPixels = 5;
int boxHeight = 21;
int descent = 3;
int rowHeight = boxHeight + descent;
int frameWidth = 250;
int frameHeight = 5 * rowHeight + baseRowPixels;
int rightEdge = frameWidth - 20;
int gap = 2;
int promptWidth = 110;
int buttonWidth = 80;
int boxWidth = 80;
int boxWidth1 = 30;
int col0 = 8;
int col1 = col0 + promptWidth + gap;
#endif
}

AnalyzeGapsDlg::AnalyzeGapsDlg(AppDataPtr& pAppData, MainFrame* parent, wxWindowID id, const wxString& title, const wxPoint& pos)
	: wxDialog(parent, id, title, pos, wxSize(frameWidth, frameHeight), wxDEFAULT_DIALOG_STYLE, title)
	, _pAppData(pAppData)
	, _pMainFrame(parent)
{
	auto& params = pAppData->getParams();

	int rowNum = 0;
	_maxGapDistPrompt = new wxStaticText(this, 0, _T("Max Gap Dist(m)"), wxPoint(col0, baseRowPixels + rowNum * rowHeight), wxSize(promptWidth, boxHeight));
	_maxGapDistText = new wxTextCtrl(this, MAX_GAP_DIST, std::to_string(params.gapBoundingBoxSemiSpan * 2), wxPoint(col1, baseRowPixels + rowNum * rowHeight - descent), wxSize(boxWidth, boxHeight), wxTE_RIGHT);

	rowNum++;
	_gridSizePrompt = new wxStaticText(this, 0, _T("Grid Size (m)"), wxPoint(col0, baseRowPixels + rowNum * rowHeight), wxSize(promptWidth, boxHeight));
	_gridSizeText = new wxTextCtrl(this, GRID_SIZE, std::to_string(params.gapGridSpacing), wxPoint(col1, baseRowPixels + rowNum * rowHeight - descent), wxSize(boxWidth, boxHeight), wxTE_RIGHT);

	rowNum++;
	_okButton = new wxButton(this, wxID_OK, _T("OK"), wxPoint(rightEdge - 2 * (buttonWidth + gap), baseRowPixels + rowNum * rowHeight));
	_cancelButton = new wxButton(this, wxID_CANCEL, _T("Cancel"), wxPoint(rightEdge - 1 * (buttonWidth + gap), baseRowPixels + rowNum * rowHeight));

}

AnalyzeGapsDlg::~AnalyzeGapsDlg()
{
}

bool AnalyzeGapsDlg::getValue(wxTextCtrl* item, Index3DBaseType& value) const
{
	try {
		wxString wstr = item->GetValue();
		if (wstr.length() > 0) {
			value = (Index3DBaseType)stoi(wstr.c_str().AsChar());
			return true;
		}
	} catch (const invalid_argument&) {
	}

	return false;
}

bool AnalyzeGapsDlg::getValue(wxTextCtrl* item, unsigned short& value) const
{
	try {
		wxString wstr = item->GetValue();
		if (wstr.length() > 0) {
			value = (unsigned short)stoi(wstr.c_str().AsChar());
			return true;
		}
	} catch (const invalid_argument& ) {
	}

	return false;
}

bool AnalyzeGapsDlg::getValue(wxTextCtrl* item, size_t& value) const
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

bool AnalyzeGapsDlg::getValue(wxTextCtrl* item, double& value) const
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

void AnalyzeGapsDlg::OnOk()
{
	auto& params = _pAppData->getParams();
	getValue(_maxGapDistText, params.gapBoundingBoxSemiSpan);
	params.gapBoundingBoxSemiSpan /= 2;
	getValue(_gridSizeText, params.gapGridSpacing);
}

