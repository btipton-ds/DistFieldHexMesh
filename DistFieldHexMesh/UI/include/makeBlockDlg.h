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

#include "wx/wxprec.h"
#include <tm_vector3.h>

#ifndef WX_PRECOMP
#include "wx/wx.h"
#include <wx/dialog.h>
#endif

namespace DFHM {

class MakeBlockDlg : public wxDialog {
public:
	enum BlockType {
		BLOCK_TYPE_BLOCK = 0,
		BLOCK_TYPE_CYLINDER = 1,
		BLOCK_TYPE_WEDGE = 2,
	};

	MakeBlockDlg(wxWindow* parent, wxWindowID id, const wxString& title, 
	const wxPoint& pos = wxDefaultPosition);

	void comboBoxTypeComboAction(const wxCommandEvent& event);
	void comboBoxAxisComboAction(const wxCommandEvent& event);

	int getSelection() const;
	Vector3d getBlockOrigin() const;
	Vector3d getBlockSpan() const;

private:
	void comboBoxTypeKeyAction(wxKeyEvent& event);
	void comboBoxAxisKeyAction(wxKeyEvent& event);

	void setBlockType(int blockType);

	int _curType = -1;
	wxChoice* _comboBoxType;
	wxStaticText* _comboBoxTypePrompt;

	wxChoice* _comboBoxAxis;
	wxStaticText
		* _startPrompt = nullptr,
		* _endPrompt = nullptr,
		* _comboBoxAxisPrompt = nullptr,
		* _outerRadiusBoxPrompt = nullptr,
		* _innerRadiusBoxPrompt = nullptr,
		* _widthPrompt = nullptr,
		* _heightPrompt = nullptr,
		* _axialDivsBoxPrompt,
		* _axialGradingBoxPrompt,
		* _angularDivsBoxPrompt,
		* _radialDivsBoxPrompt,
		* _radialGradingBoxPrompt;

	wxTextCtrl
		* _outerRadiusBox = nullptr,
		* _innerRadiusBox = nullptr,
		* _startBox = nullptr,
		* _endBox = nullptr,
		* _widthBox = nullptr,
		* _heightBox = nullptr,
		*_axialDivsBox,
		* _axialGradingBox,
		* _angularDivsBox,
		* _radialDivsBox, 
		* _radialGradingBox;

	wxButton
		* _okButton, 
		* _cancelButton;

};

inline int MakeBlockDlg::getSelection() const
{
	if (_comboBoxType)
		return _comboBoxType->GetSelection();
	return -1;
}

}
