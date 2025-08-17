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

#pragma warning( disable : 4005 ) // M_PI is being redefined somewhere in wxWidgets 3.2.8

#include <memory>
#include <editPrefsDlg.h>
#include <wx/string.h>
#include <wx/wfstream.h>
#include <filesystem>
#include <appData.h>
#include <mainFrame.h>

#ifdef WIN32
#include "windows.h"
#endif // WIN32

using namespace std;
using namespace DFHM;

namespace
{
enum Ids {
	PREFS_TEXT_ID = 1,
};


#ifdef WIN32
int frameWidth = 400;
int baseRowPixels = 5;
int rightEdge = frameWidth - 20;
int gap = 2;
int descent = 3;
int margin = 7;
int col0 = margin;
int promptWidth = 110;
int buttonWidth = 80;
int buttonHeight = 20;
int promptHeight = 20;

int boxWidth = rightEdge - margin;
int boxHeight = 15 * 20;
int buttonY = baseRowPixels + promptHeight + boxHeight + margin;

int frameHeight = buttonY + buttonHeight + margin + 40;

#else
int frameWidth = 400;
int baseRowPixels = 5;
int rightEdge = frameWidth - 20;
int gap = 2;
int descent = 3;
int margin = 7;
int col0 = margin;
int promptWidth = 110;
int buttonWidth = 80;
int buttonHeight = 20;
int promptHeight = 20;

int boxWidth = rightEdge - margin;
int boxHeight = 15 * 20;
int buttonY = baseRowPixels + promptHeight + boxHeight + margin;

int frameHeight = buttonY + buttonHeight + margin + 40;
#endif
}

EditPrefsDlg::EditPrefsDlg(MainFrame* parent, const wxString& title, const wxPoint& pos)
	: wxDialog(parent, 1, title, pos, wxSize(frameWidth, frameHeight), wxDEFAULT_DIALOG_STYLE, wxString("Edit Prefs"))
	, _pMainFrame(parent)
{
	auto pAppData = _pMainFrame->getAppData();
	string prefsStr;
	pAppData->readPrefsFile(prefsStr);

	_prefsTextPrompt = new wxStaticText(this, 0, _T("Prefs"), wxPoint(col0, baseRowPixels), wxSize(promptWidth, promptHeight));
	_prefsText = new wxTextCtrl(this, PREFS_TEXT_ID, prefsStr, wxPoint(col0, baseRowPixels + promptHeight), wxSize(boxWidth, boxHeight), wxTE_MULTILINE);

	_okButton = new wxButton(this, wxID_OK, _T("Ok"), wxPoint(rightEdge - margin - 2 * buttonWidth, buttonY), wxSize(buttonWidth, buttonHeight));
	_cancelButton = new wxButton(this, wxID_CANCEL, _T("Cancel"), wxPoint(rightEdge - buttonWidth, buttonY), wxSize(buttonWidth, buttonHeight));
}

void EditPrefsDlg::save()
{
	auto pAppData = _pMainFrame->getAppData();
	string str(_prefsText->GetValue());
	pAppData->updatePrefsFile(str);
	pAppData->updateHexTess();
	pAppData->updateDebugTess();
}
