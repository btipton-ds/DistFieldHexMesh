#pragma once

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
