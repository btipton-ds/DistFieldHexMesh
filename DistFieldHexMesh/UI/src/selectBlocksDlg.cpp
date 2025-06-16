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
#include <selectBlocksDlg.h>
#include <wx/string.h>
#include <wx/wfstream.h>
#include <filesystem>
#include <block.h>
#include <vertex.h>
#include <volume.h>
#include <mainFrame.h>

#ifdef WIN32
#include "windows.h"
#endif // WIN32

using namespace std;
using namespace DFHM;

namespace
{
	enum IDS {
		X_MIN_ID,
		X_MAX_ID,
		Y_MIN_ID,
		Y_MAX_ID,
		Z_MIN_ID,
		Z_MAX_ID,
	};
}

SelectBlocksDlg::SelectBlocksDlg(MainFrame* parent, wxWindowID id, const wxString& title,
	const wxPoint& pos)
	: wxDialog(parent, id, title, pos, wxSize(400, 460), wxDEFAULT_DIALOG_STYLE, wxString("Make Block"))
{
#ifdef WIN32
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
	int baseRow = 5;
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
	int baseRow = 5;
#endif

	
	Index3D min, max;
	parent->getAppData()->getDisplayMinMax(min, max);

	int rowNum = 0;
	_xMinPrompt = new wxStaticText(this, 0, _T("Min x:"), wxPoint(col0, baseRow + rowNum * rowHeight), wxSize(promptWidth, boxHeight));
	_xMinBox = new wxTextCtrl(this, X_MAX_ID, std::to_string(min[0]), wxPoint(col1, baseRow + rowNum * rowHeight - descent), wxSize(boxWidth, boxHeight));
	_xMaxPrompt = new wxStaticText(this, 0, _T("Max x:"), wxPoint(col2, baseRow + rowNum * rowHeight), wxSize(promptWidth, boxHeight));
	_xMaxBox = new wxTextCtrl(this, X_MAX_ID, std::to_string(max[0]), wxPoint(col3, baseRow + rowNum * rowHeight - descent), wxSize(boxWidth, boxHeight));

	rowNum++;
	_yMinPrompt = new wxStaticText(this, 0, _T("Min y:"), wxPoint(col0, baseRow + rowNum * rowHeight), wxSize(promptWidth, boxHeight));
	_yMinBox = new wxTextCtrl(this, Y_MAX_ID, std::to_string(min[1]), wxPoint(col1, baseRow + rowNum * rowHeight - descent), wxSize(boxWidth, boxHeight));
	_yMaxPrompt = new wxStaticText(this, 0, _T("Max y:"), wxPoint(col2, baseRow + rowNum * rowHeight), wxSize(promptWidth, boxHeight));
	_yMaxBox = new wxTextCtrl(this, Y_MAX_ID, std::to_string(max[1]), wxPoint(col3, baseRow + rowNum * rowHeight - descent), wxSize(boxWidth, boxHeight));

	rowNum++;
	_zMinPrompt = new wxStaticText(this, 0, _T("Min z:"), wxPoint(col0, baseRow + rowNum * rowHeight), wxSize(promptWidth, boxHeight));
	_zMinBox = new wxTextCtrl(this, Z_MAX_ID, std::to_string(min[2]), wxPoint(col1, baseRow + rowNum * rowHeight - descent), wxSize(boxWidth, boxHeight));
	_zMaxPrompt = new wxStaticText(this, 0, _T("Max z:"), wxPoint(col2, baseRow + rowNum * rowHeight), wxSize(promptWidth, boxHeight));
	_zMaxBox = new wxTextCtrl(this, Z_MAX_ID, std::to_string(max[2]), wxPoint(col3, baseRow + rowNum * rowHeight - descent), wxSize(boxWidth, boxHeight));

	rowNum++;
	_okButton = new wxButton(this, wxID_CANCEL, _T("Cancel"), wxPoint(col1 + 100, baseRow + rowNum * rowHeight));
	_cancelButton = new wxButton(this, wxID_OK, _T("OK"), wxPoint(col1, baseRow + rowNum * rowHeight));

}

Index3D SelectBlocksDlg::getMin() const
{
	Index3D result;

	result[0] = stol(_xMinBox->GetValue().c_str().AsChar());
	result[1] = stol(_yMinBox->GetValue().c_str().AsChar());
	result[2] = stol(_zMinBox->GetValue().c_str().AsChar());

	return result;
}

Index3D SelectBlocksDlg::getMax() const
{
	Index3D result;

	result[0] = stol(_xMaxBox->GetValue().c_str().AsChar());
	result[1] = stol(_yMaxBox->GetValue().c_str().AsChar());
	result[2] = stol(_zMaxBox->GetValue().c_str().AsChar());

	return result;
}
