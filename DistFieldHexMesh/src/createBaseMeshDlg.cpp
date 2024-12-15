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
#include <splitParams.h>
#include <createBaseMeshDlg.h>
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
		X_ROT_ANGLE = 1,
		Y_ROT_ANGLE,
		Z_ROT_ANGLE,

		X_SYM,
		Y_SYM,
		Z_SYM,

		BASE_OFFSET,
	};
}

CreateBaseMeshDlg::CreateBaseMeshDlg(BuildCFDParams& params, wxWindow* parent, wxWindowID id, const wxString& title, const wxPoint& pos)
	: wxDialog(parent, id, title, pos, wxSize(400, 460), wxDEFAULT_DIALOG_STYLE, wxString("Make Block"))
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
	_xRotatationPrompt = new wxStaticText(this, 0, _T("X rotation (deg)"), wxPoint(col0, baseRowPixels + rowNum * rowHeight), wxSize(promptWidth, boxHeight));
	_xRotationText = new wxTextCtrl(this, X_ROT_ANGLE, std::to_string(params.xRotationDeg), wxPoint(col1, baseRowPixels + rowNum * rowHeight - descent), wxSize(boxWidth, boxHeight), wxTE_RIGHT);

	rowNum++;
	_yRotatationPrompt = new wxStaticText(this, 0, _T("Y rotation (deg)"), wxPoint(col0, baseRowPixels + rowNum * rowHeight), wxSize(promptWidth, boxHeight));
	_yRotationText = new wxTextCtrl(this, Y_ROT_ANGLE, std::to_string(params.yRotationDeg), wxPoint(col1, baseRowPixels + rowNum * rowHeight - descent), wxSize(boxWidth, boxHeight), wxTE_RIGHT);

	rowNum++;
	_zRotatationPrompt = new wxStaticText(this, 0, _T("Z rotation (deg)"), wxPoint(col0, baseRowPixels + rowNum * rowHeight), wxSize(promptWidth, boxHeight));
	_zRotationText = new wxTextCtrl(this, Z_ROT_ANGLE, std::to_string(params.zRotationDeg), wxPoint(col1, baseRowPixels + rowNum * rowHeight - descent), wxSize(boxWidth, boxHeight), wxTE_RIGHT);

	rowNum++;
	_baseBoxOffsetPrompt = new wxStaticText(this, 0, _T("Base offset"), wxPoint(col0, baseRowPixels + rowNum * rowHeight), wxSize(promptWidth, boxHeight));
	_baseBoxOffsetText = new wxTextCtrl(this, BASE_OFFSET, std::to_string(params.baseBoxOffset), wxPoint(col1, baseRowPixels + rowNum * rowHeight - descent), wxSize(boxWidth, boxHeight), wxTE_RIGHT);

	rowNum++;
	_symXCheckBox = new wxCheckBox(this, X_SYM, _T("Y-Z Symmetry"), wxPoint(col0, baseRowPixels + rowNum * rowHeight), wxSize(promptWidth, boxHeight));
	_symXCheckBox->SetValue(params.symXAxis);

	rowNum++;
	_symYCheckBox = new wxCheckBox(this, Y_SYM, _T("X-Z Symmetry"), wxPoint(col0, baseRowPixels + rowNum * rowHeight), wxSize(promptWidth, boxHeight));
	_symYCheckBox->SetValue(params.symYAxis);

	rowNum++;
	_symZCheckBox = new wxCheckBox(this, Z_SYM, _T("X-Y Symmetry"), wxPoint(col0, baseRowPixels + rowNum * rowHeight), wxSize(promptWidth, boxHeight));
	_symZCheckBox->SetValue(params.symZAxis);

	rowNum++;
	_okButton = new wxButton(this, wxID_CANCEL, _T("Cancel"), wxPoint(col1 + 100, baseRowPixels + rowNum * rowHeight));
	_cancelButton = new wxButton(this, wxID_OK, _T("OK"), wxPoint(col1, baseRowPixels + rowNum * rowHeight));

}

void CreateBaseMeshDlg::getValue(wxTextCtrl* item, size_t& value) const
{
	wxString wstr = item->GetValue();
	if (wstr.length() > 0) {
		value = stoi(wstr.c_str().AsChar());
	}
}

void CreateBaseMeshDlg::getValue(wxTextCtrl* item, double& value) const
{
	wxString wstr = item->GetValue();
	if (wstr.length() > 0) {
		value = stod(wstr.c_str().AsChar());
	}
}

void CreateBaseMeshDlg::getParams(BuildCFDParams& params) const
{
	params.xRotationDeg = _symXCheckBox->GetValue();
	params.yRotationDeg = _symYCheckBox->GetValue();
	params.zRotationDeg = _symZCheckBox->GetValue();

	getValue(_xRotationText, params.xRotationDeg);
	getValue(_yRotationText, params.yRotationDeg);
	getValue(_zRotationText, params.zRotationDeg);
	getValue(_baseBoxOffsetText, params.baseBoxOffset);

}
