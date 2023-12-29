#include <memory>
#include <makeBlockDlg.h>
#include <wx/string.h>
#include <wx/wfstream.h>
#include <filesystem>
#include <block.h>
#include <volume.h>

#ifdef WIN32
#include "windows.h"
#endif // WIN32

using namespace std;
using namespace DFHM;

#define COMBO_TYPE_ID 0
#define COMBO_AXIS_ID 1
#define OUTER_RAD_ID 2

namespace {
#ifdef WIN32
	int col0 = 8;
	int col1 = 120;
	int rowHeight = 24;
	int comboOffset = 3;
#else
	int col0 = 8;
	int col1 = 150;
	int rowHeight = 34;
	int comboOffset = 6;
#endif
}

MakeBlockDlg::MakeBlockDlg(wxWindow* parent, wxWindowID id, const wxString& title,
	const wxPoint& pos)
	: wxDialog(parent, id, title, pos, wxSize(400, 460), wxDEFAULT_DIALOG_STYLE, wxString("Make Block"))
{

	int rowNum = 1;
	_comboBoxTypePrompt = new wxStaticText(this, 20, _T("Block Type"), wxPoint(col0, rowNum * rowHeight));
	wxArrayString choices;
	choices.Add(_T("Block"));
	choices.Add(_T("Cylinder"));
	choices.Add(_T("Wedge"));
	_comboBoxType = new wxChoice(this, COMBO_TYPE_ID, wxPoint(col1, rowNum * rowHeight - comboOffset), wxDefaultSize, choices);
	_comboBoxType->GetEventHandler()->Connect(wxEVT_KEY_DOWN, wxKeyEventHandler(MakeBlockDlg::comboBoxTypeKeyAction));
	Bind(wxEVT_COMMAND_CHOICE_SELECTED, &MakeBlockDlg::comboBoxTypeComboAction, this, COMBO_TYPE_ID);
	rowNum++;

	_comboBoxAxisPrompt = new wxStaticText(this, 20, _T("Flow Axis"), wxPoint(col0, rowNum * rowHeight));
	choices.clear();
	choices.Add(_T("X Axis"));
	choices.Add(_T("Y Axis"));
	choices.Add(_T("Z Axis"));
	_comboBoxAxis = new wxChoice(this, COMBO_AXIS_ID, wxPoint(col1, rowNum * rowHeight - comboOffset), wxDefaultSize, choices);
	_comboBoxAxis->Select(1);
	_comboBoxAxis->GetEventHandler()->Connect(wxEVT_KEY_DOWN, wxKeyEventHandler(MakeBlockDlg::comboBoxAxisKeyAction));
	Bind(wxEVT_COMMAND_CHOICE_SELECTED, &MakeBlockDlg::comboBoxAxisComboAction, this, COMBO_AXIS_ID);
	rowNum++;

	_startPrompt = new wxStaticText(this, 20, _T("Start dist"), wxPoint(col0, rowNum * rowHeight));
	_startBox = new wxTextCtrl(this, OUTER_RAD_ID, "0.0", wxPoint(col1, rowNum * rowHeight));
	rowNum++;

	_endPrompt = new wxStaticText(this, 20, _T("End dist"), wxPoint(col0, rowNum * rowHeight));
	_endBox = new wxTextCtrl(this, OUTER_RAD_ID, "1.0", wxPoint(col1, rowNum * rowHeight));
	rowNum++;

	int subRowNum = 0;
	_angularDivsBoxPrompt = new wxStaticText(this, 20, _T("Angular Divisions"), wxPoint(col0, (rowNum + subRowNum) * rowHeight));
	_angularDivsBox = new wxTextCtrl(this, OUTER_RAD_ID, "16", wxPoint(col1, (rowNum + subRowNum) * rowHeight));
	subRowNum++;
	_outerRadiusBoxPrompt = new wxStaticText(this, 20, _T("Outer Radius"), wxPoint(col0, (rowNum + subRowNum) * rowHeight));
	_outerRadiusBox = new wxTextCtrl(this, OUTER_RAD_ID, "1.0", wxPoint(col1, (rowNum + subRowNum) * rowHeight));
	subRowNum++;
	_innerRadiusBoxPrompt = new wxStaticText(this, 20, _T("Inner Radius"), wxPoint(col0, (rowNum + subRowNum) * rowHeight));
	_innerRadiusBox = new wxTextCtrl(this, OUTER_RAD_ID, "0.5", wxPoint(col1, (rowNum + subRowNum) * rowHeight));
	subRowNum++;
	_axialDivsBoxPrompt = new wxStaticText(this, 20, _T("Axial Divisions"), wxPoint(col0, (rowNum + subRowNum) * rowHeight));
	_axialDivsBox = new wxTextCtrl(this, OUTER_RAD_ID, "8", wxPoint(col1, (rowNum + subRowNum) * rowHeight));
	subRowNum++;
	_axialGradingBoxPrompt = new wxStaticText(this, 20, _T("Axial Grading"), wxPoint(col0, (rowNum + subRowNum) * rowHeight));
	_axialGradingBox = new wxTextCtrl(this, OUTER_RAD_ID, "4", wxPoint(col1, (rowNum + subRowNum) * rowHeight));
	subRowNum++;
	_radialDivsBoxPrompt = new wxStaticText(this, 20, _T("Radial Divisions"), wxPoint(col0, (rowNum + subRowNum) * rowHeight));
	_radialDivsBox = new wxTextCtrl(this, OUTER_RAD_ID, "8", wxPoint(col1, (rowNum + subRowNum) * rowHeight));
	subRowNum++;
	_radialGradingBoxPrompt = new wxStaticText(this, 20, _T("Radial Grading"), wxPoint(col0, (rowNum + subRowNum) * rowHeight));
	_radialGradingBox = new wxTextCtrl(this, OUTER_RAD_ID, "4", wxPoint(col1, (rowNum + subRowNum) * rowHeight));
	subRowNum++;

	_angularDivsBoxPrompt->Show(false);
	_angularDivsBox->Show(false);
	_outerRadiusBoxPrompt->Show(false);
	_outerRadiusBox->Show(false);
	_innerRadiusBoxPrompt->Show(false);
	_innerRadiusBox->Show(false);
	_axialDivsBoxPrompt->Show(false);
	_axialDivsBox->Show(false);
	_radialDivsBoxPrompt->Show(false);
	_radialDivsBox->Show(false);
	_radialGradingBoxPrompt->Show(false);
	_radialGradingBox->Show(false);

	_widthPrompt = new wxStaticText(this, 20, _T("Width"), wxPoint(col0, (rowNum + 0) * rowHeight));
	_widthBox = new wxTextCtrl(this, OUTER_RAD_ID, "1.0", wxPoint(col1, (rowNum + 0) * rowHeight));
	_heightPrompt = new wxStaticText(this, 20, _T("Height"), wxPoint(col0, (rowNum + 1) * rowHeight));
	_heightBox = new wxTextCtrl(this, OUTER_RAD_ID, "0.5", wxPoint(col1, (rowNum + 1) * rowHeight));
	_widthPrompt->Show(false);
	_widthBox->Show(false);
	_heightPrompt->Show(false);
	_heightBox->Show(false);

	_okButton = new wxButton(this, wxID_CANCEL, _T("Cancel"), wxPoint(col1 + 100, (rowNum + 7) * rowHeight));
	_cancelButton = new wxButton(this, wxID_OK, _T("OK"), wxPoint(col1, (rowNum + 7) * rowHeight));

	setBlockType(BLOCK_TYPE_CYLINDER);
}

void MakeBlockDlg::comboBoxTypeKeyAction(wxKeyEvent& event)
{
	if (event.GetKeyCode() == WXK_TAB)
		Navigate(wxNavigationKeyEvent::IsForward);
	else
		event.Skip();
}

void MakeBlockDlg::comboBoxTypeComboAction(const wxCommandEvent& event)
{
	setBlockType(event.GetInt());
}

void MakeBlockDlg::comboBoxAxisKeyAction(wxKeyEvent& event)
{
	if (event.GetKeyCode() == WXK_TAB)
		Navigate(wxNavigationKeyEvent::IsForward);
	else
		event.Skip();
}

void MakeBlockDlg::comboBoxAxisComboAction(const wxCommandEvent& event)
{
	switch (event.GetSelection()) {
	case 0: {
		break;
	}
	case 1: {
		break;
	}
	case 2: {
		break;
	}
	default:
		break;
	}
}

void MakeBlockDlg::setBlockType(int blockType)
{
	if (blockType != _curType) {
		_angularDivsBoxPrompt->Show(false);
		_angularDivsBox->Show(false);
		_outerRadiusBoxPrompt->Show(false);
		_outerRadiusBox->Show(false);
		_innerRadiusBoxPrompt->Show(false);
		_innerRadiusBox->Show(false);
		_axialDivsBoxPrompt->Show(false);
		_axialDivsBox->Show(false);
		_axialGradingBoxPrompt->Show(false);
		_axialGradingBox->Show(false);
		_radialDivsBoxPrompt->Show(false);
		_radialDivsBox->Show(false);
		_radialGradingBoxPrompt->Show(false);
		_radialGradingBox->Show(false);

		_widthPrompt->Show(false);
		_widthBox->Show(false);
		_heightPrompt->Show(false);
		_heightBox->Show(false);

		_curType = blockType;
	}
	switch (blockType) {
		case BLOCK_TYPE_BLOCK: {
			_widthPrompt->Show(true);
			_widthBox->Show(true);
			_heightPrompt->Show(true);
			_heightBox->Show(true);
			break;
		}
		case BLOCK_TYPE_CYLINDER: {
			_angularDivsBoxPrompt->Show(true);
			_angularDivsBox->Show(true);
			_outerRadiusBoxPrompt->Show(true);
			_outerRadiusBox->Show(true);
			_innerRadiusBoxPrompt->Show(true);
			_innerRadiusBox->Show(true);
			_axialDivsBoxPrompt->Show(true);
			_axialDivsBox->Show(true);
			_axialGradingBoxPrompt->Show(true);
			_axialGradingBox->Show(true);
			_radialDivsBoxPrompt->Show(true);
			_radialDivsBox->Show(true);
			_radialGradingBoxPrompt->Show(true);
			_radialGradingBox->Show(true);
			break;
		}
		case BLOCK_TYPE_WEDGE: {
			_outerRadiusBoxPrompt->Show(true);
			_outerRadiusBox->Show(true);
			_innerRadiusBoxPrompt->Show(true);
			_innerRadiusBox->Show(true);
			_axialDivsBoxPrompt->Show(true);
			_axialDivsBox->Show(true);
			_axialGradingBoxPrompt->Show(true);
			_axialGradingBox->Show(true);
			break;
		}
		default:
			break;
	}
	_comboBoxType->SetSelection(blockType);
}
