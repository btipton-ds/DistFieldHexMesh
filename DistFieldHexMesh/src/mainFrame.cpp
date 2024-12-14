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
#include <wx/filedlg.h>
#include <wx/string.h>
#include <wx/wfstream.h>
#include <filesystem>

#ifdef WIN32
#include "windows.h"
#endif // WIN32

#include <tm_math.h>
#include <triMesh.h>
#include <readWriteStl.h>
#include <MultiCoreUtil.h>

#include <splitParams.h>
#include <mainFrame.h>
#include <makeBlockDlg.h>
#include <selectBlocksDlg.h>
#include <buildCFDHexesDlg.h>
#include <graphicsCanvas.h>
#include <meshData.h>
#include <volume.h>
#include <vertex.h>

using namespace std;
using namespace DFHM;

MainFrame::MainFrame(wxWindow* parent,
    wxWindowID id,
    const wxString& title,
    const wxPoint& pos,
    const wxSize& size,
    long style,
    const wxString& name)
    : wxFrame(parent, id, title, pos, size, style, name)
{
#ifdef WIN32
#pragma warning(push)
#pragma warning(disable: 4996)

    AllocConsole();
    freopen("conin$", "r", stdin);
    freopen("conout$", "w", stdout);
    freopen("conout$", "w", stderr);

#pragma warning(pop)
#endif // WIN32

    _pAppData = make_shared<AppData>(this);

    auto sizer = new wxBoxSizer(wxHORIZONTAL);

    _pCanvas = new GraphicsCanvas(this, _pAppData);
    _pCanvas->setBackColor(rgbaColor(0.9f, 0.9f, 1.0f));

    _pObjectTree = new ObjectTreeCtrl(this, ID_OBJ_TREE_CTRL, wxDefaultPosition, wxSize(200, 200));

    sizer->Add(_pObjectTree, 0, wxEXPAND | wxDOWN, FromDIP(0));
    sizer->Add(_pCanvas, 1, wxEXPAND | wxALL, FromDIP(0));

    SetSizer(sizer);

    addMenus();
    addModelPanel();
    addStatusBar();

    wxBitmapBundle bitMap0, bitMap1;
    bitMap0.FromFiles("D:/DarkSky/Projects/DistFieldHexMesh/DistFieldHexMesh/src/SheetIcon.bmp");
    bitMap1.FromFiles("D:/DarkSky/Projects/DistFieldHexMesh/DistFieldHexMesh/src/SolidIcon.bmp");
    _images.push_back(bitMap0);
    _images.push_back(bitMap1);

}

MainFrame::~MainFrame()
{
    // Clear the canvas first
    _pCanvas->preDestroy();
    _pAppData = nullptr;
}

void MainFrame::addMenus()
{
    _menuBar = new wxMenuBar;

    createFileMenu();
    createEditMenu();
    createViewMenu();
    createHelpMenu();

    SetMenuBar(_menuBar);
}

void MainFrame::createFileMenu()
{
    _fileMenu = new wxMenu;

    _fileMenu->Append(wxID_NEW, "New...");
    Bind(wxEVT_MENU, &MainFrame::OnNew, this, wxID_NEW);

    _fileMenu->Append(wxID_OPEN);
    Bind(wxEVT_MENU, &MainFrame::OnOpen, this, wxID_OPEN);

    _fileMenu->Append(wxID_SAVE);
    Bind(wxEVT_MENU, &MainFrame::OnSave, this, wxID_SAVE);

    _fileMenu->Append(wxID_SAVEAS);
    Bind(wxEVT_MENU, &MainFrame::OnSaveAs, this, wxID_SAVEAS);

    _fileMenu->AppendSeparator();

    _fileMenu->Append(ID_IMPORT_MESH, "Import mesh...");
    Bind(wxEVT_MENU, &MainFrame::OnImportMesh, this, ID_IMPORT_MESH);

    _fileMenu->Append(ID_WRITE_POLYMESH, "Write Polymesh...");
    Bind(wxEVT_MENU, &MainFrame::OnWritePolymesh, this, ID_WRITE_POLYMESH);

    _fileMenu->Append(wxID_CLOSE);
    Bind(wxEVT_MENU, &MainFrame::OnClose, this, wxID_CLOSE);

    _fileMenu->AppendSeparator();

    _fileMenu->Append(wxID_EXIT);
    Bind(wxEVT_MENU, &MainFrame::OnExit, this, wxID_EXIT);

    _menuBar->Append(_fileMenu, "&File");
}

void MainFrame::createEditMenu()
{
    _editMenu = new wxMenu;
    
    _editMenu->Append(wxID_CUT);
    Bind(wxEVT_MENU, &MainFrame::OnCut, this, wxID_CUT);

    _editMenu->Append(wxID_COPY);
    Bind(wxEVT_MENU, &MainFrame::OnCopy, this, wxID_COPY);

    _editMenu->Append(wxID_PASTE);
    Bind(wxEVT_MENU, &MainFrame::OnPaste, this, wxID_PASTE);

    _editMenu->AppendSeparator();

    _editMenu->Append(ID_VerifyClosed, "Verify Closed");
    Bind(wxEVT_MENU, &MainFrame::OnVerifyClosed, this, ID_VerifyClosed);

    _editMenu->Append(ID_VerifyNormals, "Verify Normals");
    Bind(wxEVT_MENU, &MainFrame::OnVerifyNormals, this, ID_VerifyNormals);

    _editMenu->Append(ID_AnalyzeGaps, "Analyze Gaps");
    Bind(wxEVT_MENU, &MainFrame::OnAnalyzeGaps, this, ID_AnalyzeGaps);
    
    _editMenu->Append(ID_FindMinimumGap, "Find Minimum Gap");
    Bind(wxEVT_MENU, &MainFrame::OnFindMinGap, this, ID_FindMinimumGap);

    _editMenu->Append(ID_BuildCFDHexes, "Build CFD Hexes");
    Bind(wxEVT_MENU, &MainFrame::OnBuildCFDHexes, this, ID_BuildCFDHexes);

    _menuBar->Append(_editMenu, "&Edit");

}

void MainFrame::createViewMenu()
{
    wxMenu* menu= new wxMenu;

    addViewSubMenu(menu);

    menu->Append(ID_SHOW_SHARP_EDGES, "Show Sharp Edges", "Turns rendering of sharp edges on/off", true);
    Bind(wxEVT_MENU, &MainFrame::OnShowSharpEdges, this, ID_SHOW_SHARP_EDGES);

    menu->Append(ID_SHOW_SHARP_VERTS, "Show Sharp Vertices", "Turns rendering of sharp vertices on/off", true);
    Bind(wxEVT_MENU, &MainFrame::OnShowSharpVerts, this, ID_SHOW_SHARP_VERTS);

    menu->Append(ID_SHOW_TRI_NORMALS, "Show Tri Normals", "Turns rendering of triangle normals on/off", true);
    Bind(wxEVT_MENU, &MainFrame::OnShowTriNormals, this, ID_SHOW_TRI_NORMALS);

    menu->Append(ID_SHOW_CURVATURE, "Show Curvature", "Turns rendering of curvature on/off", true);
    Bind(wxEVT_MENU, &MainFrame::OnShowCurvature, this, ID_SHOW_CURVATURE);

    menu->Append(ID_SHOW_FACES, "Show Faces", "Turns rendering of faces on/off", true);
    Bind(wxEVT_MENU, &MainFrame::OnShowFaces, this, ID_SHOW_FACES);

    menu->Append(ID_SHOW_EDGES, "Show Edges", "Turns rendering of edges on/off", true);
    Bind(wxEVT_MENU, &MainFrame::OnShowEdges, this, ID_SHOW_EDGES);

    menu->Append(ID_SHOW_OUTER, "Show Outer", "Turns rendering of edges on/off", true);
    Bind(wxEVT_MENU, &MainFrame::OnShowOuter, this, ID_SHOW_OUTER);

    menu->Append(ID_SHOW_MODEL_BOUNDARY, "Show Model Boundary", "Turns rendering of model boundary", true);
    Bind(wxEVT_MENU, &MainFrame::OnShowOuter, this, ID_SHOW_MODEL_BOUNDARY);

    menu->Append(ID_SHOW_SELECTED_BLOCKS, "Show Selected Blocks", "Shows only selected blocks", true);
    Bind(wxEVT_MENU, &MainFrame::OnShowSelectedBlocks, this, ID_SHOW_SELECTED_BLOCKS);

    _menuBar->Append(menu, "&View");

    auto item = _menuBar->FindItem(ID_SHOW_SHARP_EDGES);
    if (item)
        item->Check(getCanvas()->showSharpEdges());

    item = _menuBar->FindItem(ID_SHOW_SHARP_VERTS);
    if (item)
        item->Check(getCanvas()->showSharpVerts());

    item = _menuBar->FindItem(ID_SHOW_TRI_NORMALS);
    if (item)
        item->Check(getCanvas()->showTriNormals());

    item = _menuBar->FindItem(ID_SHOW_FACES);
    if (item)
        item->Check(getCanvas()->showFaces());

    item = _menuBar->FindItem(ID_SHOW_EDGES);
    if (item)
        item->Check(getCanvas()->showEdges());

    item = _menuBar->FindItem(ID_SHOW_OUTER);
    if (item)
        item->Check(getCanvas()->showOuter());
}

void MainFrame::addViewSubMenu(wxMenu* pParentMenu)
{
    wxMenu* pMenu = new wxMenu;

    pMenu->Append(ID_VIEW_FRONT, "Front", "Set view to front", false);
    Bind(wxEVT_MENU, &MainFrame::OnSetViewFront, this, ID_VIEW_FRONT);

    pMenu->Append(ID_VIEW_BACK, "Back", "Set view to back", false);
    Bind(wxEVT_MENU, &MainFrame::OnSetViewBack, this, ID_VIEW_BACK);

    pMenu->Append(ID_VIEW_RIGHT, "Right", "Set view to right", false);
    Bind(wxEVT_MENU, &MainFrame::OnSetViewRight, this, ID_VIEW_RIGHT);

    pMenu->Append(ID_VIEW_LEFT, "Left", "Set view to left", false);
    Bind(wxEVT_MENU, &MainFrame::OnSetViewLeft, this, ID_VIEW_LEFT);

    pMenu->Append(ID_VIEW_TOP, "Top", "Set view to top", false);
    Bind(wxEVT_MENU, &MainFrame::OnSetViewTop, this, ID_VIEW_TOP);

    pMenu->Append(ID_VIEW_BOTTOM, "Bottom", "Set view to bottom", false);
    Bind(wxEVT_MENU, &MainFrame::OnSetViewBottom, this, ID_VIEW_BOTTOM);

    pMenu->AppendSeparator();

    pMenu->Append(ID_VIEW_RESET, "Reset", "Reset view to default", false);
    Bind(wxEVT_MENU, &MainFrame::OnResetView, this, ID_VIEW_RESET);

    pParentMenu->AppendSubMenu(pMenu, "Principal Views", "Principle views");
}

void MainFrame::createHelpMenu()
{
    wxMenu* menuHelp = new wxMenu;
    menuHelp->Append(wxID_ABOUT);
    Bind(wxEVT_MENU, &MainFrame::OnAbout, this, wxID_ABOUT);

    _menuBar->Append(menuHelp, "&Help");


}

void MainFrame::addModelPanel()
{

}

void MainFrame::addStatusBar()
{
    CreateStatusBar();
    SetStatusText("Welcome to DistFieldHexMesh!");
}

void MainFrame::OnInternalIdle()
{
    wxFrame::OnInternalIdle();

    _pCanvas->Refresh();

    if (_editMenu) {
        bool hasMesh = !_pAppData->getMeshObjects().empty();
        _editMenu->Enable(ID_VerifyClosed, hasMesh);
        _editMenu->Enable(ID_VerifyNormals, hasMesh);
        _editMenu->Enable(ID_AnalyzeGaps, hasMesh);
        _editMenu->Enable(ID_FindMinimumGap, hasMesh);
        _editMenu->Enable(ID_BuildCFDHexes, hasMesh);
    }
    if (_fileMenu) {

    }
}

void MainFrame::OnOpen(wxCommandEvent& event)
{
    _pAppData->doOpen();
}

void MainFrame::registerMeshData(MeshDataPtr& pMeshData)
{
    _pCanvas->registerMeshData(pMeshData);
}

void MainFrame::refreshObjectTree()
{
    _pObjectTree->DeleteAllItems();
    const auto& meshObjects = _pAppData->getMeshObjects();
    _pObjectTree->SetImages(_images);
    auto solidsItem = _pObjectTree->AppendContainer(wxDataViewItem(), "Solids", -1, 0);
    auto surfacesItem = _pObjectTree->AppendContainer(wxDataViewItem(), "Surfaces", -1, 0);
    for (const auto& pair : meshObjects) {
        wxDataViewItem item;
        const auto pMesh = pair.second->getMesh();
        if (pMesh->isClosed())
            item = _pObjectTree->AppendItem(solidsItem, pair.second->getName());
        else
            item = _pObjectTree->AppendItem(surfacesItem, pair.second->getName());


    }
    _pObjectTree->Expand(solidsItem);
    _pObjectTree->Expand(surfacesItem);
}

void MainFrame::OnImportMesh(wxCommandEvent& event)
{
    if (_pAppData->doImportMesh()) {
        refreshObjectTree();
    }
}

void MainFrame::OnNew(wxCommandEvent& event)
{
    MakeBlockDlg dlg(this, 1, wxString("Make Block"), wxPoint(40,40));
    if (dlg.ShowModal() == wxID_OK) {
    	_pAppData->doNew(dlg);
    }
}

void MainFrame::OnSave(wxCommandEvent& event)
{
    _pAppData->doSave();
}

void MainFrame::OnSaveAs(wxCommandEvent& event)
{
    _pAppData->doSaveAs();

}

void MainFrame::OnWritePolymesh(wxCommandEvent& event)
{
    wxDirDialog dlg(this, "Choose OpenFoam Project Directory");
    if (dlg.ShowModal() == wxID_OK) {
        auto dirPath = dlg.GetPath().ToStdString();
        _pAppData->getVolume()->writePolyMesh(dirPath);
    }
}

void MainFrame::OnClose(wxCommandEvent& event)
{

}

void MainFrame::OnExit(wxCommandEvent& event)
{
    Close(true);
}

void MainFrame::OnAbout(wxCommandEvent& event)
{
    wxMessageBox("This is a wxWidgets Hello World example",
        "About Hello World", wxOK | wxICON_INFORMATION);
}

void MainFrame::OnCut(wxCommandEvent& event)
{

}

void MainFrame::OnCopy(wxCommandEvent& event)
{

}

void MainFrame::OnPaste(wxCommandEvent& event)
{

}

void MainFrame::OnVerifyClosed(wxCommandEvent& event)
{
    _pAppData->doVerifyClosed();
}

void MainFrame::OnVerifyNormals(wxCommandEvent& event)
{
    _pAppData->doVerifyNormals();
}

void MainFrame::OnAnalyzeGaps(wxCommandEvent& event)
{
    _pAppData->doAnalyzeGaps();
}

void MainFrame::OnFindMinGap(wxCommandEvent& event)
{
    _pAppData->doFindMinGap();

}

void MainFrame::OnBuildCFDHexes(wxCommandEvent& event)
{
    BuildCFDParams& params = _pAppData->getParams();
    BuildCFDHexesDlg dlg(params, this, 1, wxString("Make Block"), wxPoint(40, 40));
    if (dlg.ShowModal() == wxID_OK) {
        _pAppData->doBuildCFDHexes(dlg);
    }
}

void MainFrame::OnShowSharpEdges(wxCommandEvent& event)
{
    getCanvas()->toggleShowSharpEdges();

    auto item = _menuBar->FindItem(ID_SHOW_SHARP_EDGES);
    if (item)
        item->Check(getCanvas()->showSharpEdges());
}

void MainFrame::OnShowSharpVerts(wxCommandEvent& event)
{
    getCanvas()->toggleShowSharpVerts();

    auto item = _menuBar->FindItem(ID_SHOW_SHARP_VERTS);
    if (item)
        item->Check(getCanvas()->showSharpVerts());
}

void MainFrame::OnShowTriNormals(wxCommandEvent& event)
{
    getCanvas()->toggleShowTriNormals();

    auto normItem = _menuBar->FindItem(ID_SHOW_TRI_NORMALS);
    if (normItem)
        normItem->Check(getCanvas()->showTriNormals());
}

void MainFrame::OnShowCurvature(wxCommandEvent& event)
{
    getCanvas()->toggleShowCurvature();

    auto item = _menuBar->FindItem(ID_SHOW_CURVATURE);
    if (item)
        item->Check(getCanvas()->showCurvature());
}

void MainFrame::OnShowFaces(wxCommandEvent& event)
{
    getCanvas()->toggleShowFaces();

    auto normItem = _menuBar->FindItem(ID_SHOW_FACES);
    if (normItem)
        normItem->Check(getCanvas()->showFaces());
}

void MainFrame::OnShowEdges(wxCommandEvent& event)
{
    getCanvas()->toggleShowEdges();

    auto normItem = _menuBar->FindItem(ID_SHOW_EDGES);
    if (normItem)
        normItem->Check(getCanvas()->showEdges());
}

void MainFrame::OnShowOuter(wxCommandEvent& event)
{
    getCanvas()->toggleShowOuter();

    auto normItem = _menuBar->FindItem(ID_SHOW_OUTER);
    if (normItem)
        normItem->Check(getCanvas()->showOuter());
}

void MainFrame::OnShowModelBoundary(wxCommandEvent& event)
{
    getCanvas()->toggleShowModelBoundary();

    auto normItem = _menuBar->FindItem(ID_SHOW_MODEL_BOUNDARY);
    if (normItem)
        normItem->Check(getCanvas()->showOuter());
}

void MainFrame::OnShowSelectedBlocks(wxCommandEvent& event)
{
    SelectBlocksDlg dlg(this, 1, wxString("Make Block"), wxPoint(40, 40));

    if (dlg.ShowModal() == wxID_OK) {
        _pAppData->doSelectBlocks(dlg);
    }
}

void MainFrame::OnSetViewFront(wxCommandEvent& event)
{
    _pCanvas->setView(GraphicsCanvas::VIEW_FRONT);
}

void MainFrame::OnSetViewBack(wxCommandEvent& event)
{
    _pCanvas->setView(GraphicsCanvas::VIEW_BACK);
}

void MainFrame::OnSetViewRight(wxCommandEvent& event)
{
    _pCanvas->setView(GraphicsCanvas::VIEW_RIGHT);
}

void MainFrame::OnSetViewLeft(wxCommandEvent& event)
{
    _pCanvas->setView(GraphicsCanvas::VIEW_LEFT);
}

void MainFrame::OnSetViewTop(wxCommandEvent& event)
{
    _pCanvas->setView(GraphicsCanvas::VIEW_TOP);
}

void MainFrame::OnSetViewBottom(wxCommandEvent& event)
{
    _pCanvas->setView(GraphicsCanvas::VIEW_BOTTOM);
}

void MainFrame::OnResetView(wxCommandEvent& event)
{
    _pCanvas->resetView();
}
