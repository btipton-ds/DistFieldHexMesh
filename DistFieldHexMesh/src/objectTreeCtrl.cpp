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

#include <objectTreeCtrl.h>
#include <wx/dataview.h>
#include <meshData.h>
#include <appData.h>
#include <graphicsCanvas.h>
#include <mainFrame.h>

using namespace std;
using namespace DFHM;

BEGIN_EVENT_TABLE(ObjectTreeCtrl, wxDataViewTreeCtrl)
EVT_DATAVIEW_SELECTION_CHANGED(ID_OBJ_TREE_CTRL, ObjectTreeCtrl::onSelectionChanged)
EVT_DATAVIEW_ITEM_CONTEXT_MENU(ID_OBJ_TREE_CTRL, ObjectTreeCtrl::onContextMenu)
END_EVENT_TABLE()

ObjectTreeCtrl::ObjectTreeCtrl(MainFrame* pMainFrame, wxWindowID id, const wxPoint& pos,
	const wxSize& size, long style, const wxValidator& validator)
	: _pMainFrame(pMainFrame)
	, wxDataViewTreeCtrl(pMainFrame, id, pos, size, style, validator)
{
}

void ObjectTreeCtrl::onSelectionChanged(wxDataViewEvent& event)
{
	auto name = getCurrentItemName();
	wcout << L"onSelectionChanged " << name << "\n";
}

void ObjectTreeCtrl::onContextMenu(wxDataViewEvent& event)
{
	auto name = getCurrentItemName();
	const auto& pAppData = _pMainFrame->getAppData();
	const auto& meshObjs = pAppData->getMeshData();
	auto iter = meshObjs->find(name);
	if (iter != meshObjs->end()) {
		const auto& pData = iter->second;
		wxMenu* contextMenu = new wxMenu;

		contextMenu->Append(ID_TREE_CTRL_SHOW, "Show", "", true);
		Bind(wxEVT_MENU, &ObjectTreeCtrl::OnToggleShow, this, ID_TREE_CTRL_SHOW);
		auto item = contextMenu->FindItem(ID_TREE_CTRL_SHOW);
		if (item)
			item->Check(pData->isActive());

		contextMenu->AppendSeparator();

		contextMenu->Append(ID_VerifyClosed, "Mesh stats...", "", true);
		Bind(wxEVT_MENU, &ObjectTreeCtrl::OnMeshStats, this, ID_VerifyClosed);

		contextMenu->Append(ID_VerifyNormals, "Verify Normals");
		Bind(wxEVT_MENU, &ObjectTreeCtrl::OnVerifyNormals, this, ID_VerifyNormals);

		contextMenu->Append(ID_AnalyzeGaps, "Analyze Gaps");
		Bind(wxEVT_MENU, &ObjectTreeCtrl::OnAnalyzeGaps, this, ID_AnalyzeGaps);

		contextMenu->Append(ID_FindMinimumGap, "Find Minimum Gap");
		Bind(wxEVT_MENU, &ObjectTreeCtrl::OnFindMinGap, this, ID_FindMinimumGap);


		PopupMenu(contextMenu, event.GetPosition());
		delete contextMenu;
	}
}

void ObjectTreeCtrl::OnToggleShow(wxCommandEvent& event)
{
	auto name = getCurrentItemName();
	const auto& pAppData = _pMainFrame->getAppData();
	const auto& pData = pAppData->getMeshData()->find(name)->second;
	pData->setActive(!pData->isActive());
	_pMainFrame->getCanvas()->changeViewElements();
}

void ObjectTreeCtrl::OnMeshStats(wxCommandEvent& event)
{
	auto name = getCurrentItemName();
	const auto& pAppData = _pMainFrame->getAppData();
	const auto& pData = pAppData->getMeshData()->find(name)->second;
	pAppData->doVerifyClosed(pData->getMesh());
}

void ObjectTreeCtrl::OnVerifyNormals(wxCommandEvent& event)
{
	auto name = getCurrentItemName();
	const auto& pAppData = _pMainFrame->getAppData();
	const auto& pData = pAppData->getMeshData()->find(name)->second;
	pAppData->doVerifyNormals(pData->getMesh());
}

void ObjectTreeCtrl::OnAnalyzeGaps(wxCommandEvent& event)
{
	auto name = getCurrentItemName();
	const auto& pAppData = _pMainFrame->getAppData();
	const auto& pData = pAppData->getMeshData()->find(name)->second;
	pAppData->doAnalyzeGaps(pData->getMesh());
}

void ObjectTreeCtrl::OnFindMinGap(wxCommandEvent& event)
{
	auto name = getCurrentItemName();
	const auto& pAppData = _pMainFrame->getAppData();
	const auto& pData = pAppData->getMeshData()->find(name)->second;
	pAppData->doFindMinGap(pData->getMesh());
}

wstring ObjectTreeCtrl::getCurrentItemName() const
{
	wxString name = GetItemText(GetCurrentItem());
	wstring result = name.ToStdWstring();
	return result;
}
