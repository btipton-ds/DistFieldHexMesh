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
#include <splitLongTrisDlg.h>
#include <wx/string.h>
#include <wx/wfstream.h>
#include <filesystem>
#include <block.h>
#include <vertex.h>
#include <volume.h>
#include <appData.h>

#ifdef WIN32
#include "windows.h"
#endif // WIN32

using namespace std;
using namespace DFHM;

namespace
{
#ifdef WIN32
int frameWidth = 400;
int frameHeight = 150;
int rightEdge = frameWidth - 20;
int gap = 2;
int descent = 3;
int promptWidth = 110;
int buttonWidth = 80;
int boxWidth = 80;
int boxWidth1 = 30;
int boxHeight = 21;
int rowHeight = boxHeight + descent;
int col0 = 8;
int col1 = col0 + promptWidth + gap;
int col2 = col1 + promptWidth + gap;
int col3 = col2 + promptWidth + gap;
int baseRowPixels = 5;
#else
int frameWidth = 400;
int frameHeight = 150;
int rightEdge = frameWidth - 20;
int gap = 3;
int descent = 3;
int promptWidth = 50;
int buttonWidth = 80;
int boxWidth = 20;
int boxWidth1 = 30;
int boxHeight = 21;
int rowHeight = boxHeight + descent;
int col0 = 8;
int col1 = col0 + promptWidth + gap;
int col2 = col1 + promptWidth + gap;
int col3 = col2 + promptWidth + gap;
int baseRowPixels = 5;
#endif
}

SplitLongTrisDlg::SplitLongTrisDlg(wxWindow* parent, wxWindowID id, const wxString& title, const wxPoint& pos)
	: wxDialog(parent, id, title, pos, wxSize(frameWidth, frameHeight), wxDEFAULT_DIALOG_STYLE, wxString("Make Block"))
{
	int rowNum = 0;
	_maxLengthPrompt = new wxStaticText(this, 0, _T("Max edge length (m)"), wxPoint(col0, baseRowPixels + rowNum * rowHeight), wxSize(promptWidth, boxHeight));
	_maxLengthText = new wxTextCtrl(this, 1.0, std::to_string(1.0), wxPoint(col1, baseRowPixels + rowNum * rowHeight - descent), wxSize(boxWidth, boxHeight), wxTE_RIGHT);


	rowNum++;
	_okButton = new wxButton(this, wxID_OK, _T("Ok"), wxPoint(rightEdge - 3 * (buttonWidth + gap), baseRowPixels + rowNum * rowHeight));
	_cancelButton = new wxButton(this, wxID_CANCEL, _T("Cancel"), wxPoint(rightEdge - 1 * (buttonWidth + gap), baseRowPixels + rowNum * rowHeight));

}

double SplitLongTrisDlg::getMaxLength() const
{
	double value = -1;
	wxString wstr = _maxLengthText->GetValue();
	if (wstr.length() > 0) {
		try {
			value = stod(wstr.c_str().AsChar());
		} catch (const invalid_argument& ) {
		}
	}

	return value;
}
