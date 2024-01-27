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
#include <readStl.h>
#include <MultiCoreUtil.h>

#include <mainFrame.h>
#include <makeBlockDlg.h>
#include <graphicsCanvas.h>
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

    _pCanvas = new GraphicsCanvas(this);
    _pCanvas->setBackColor(rgbaColor(1.0f, 1.0f, 1.0f));

    _pAppData = make_shared<AppData>(this);
    addMenus();
    addStatusBar();
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

    _fileMenu->Append(wxID_OPEN);
    Bind(wxEVT_MENU, &MainFrame::OnOpen, this, wxID_OPEN);

    _fileMenu->Append(wxID_NEW, "New...");
    Bind(wxEVT_MENU, &MainFrame::OnNew, this, wxID_NEW);

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

    menu->Append(ID_SHOW_SHARP_EDGES, "Show Sharps", "Turns rendering of sharp edges on/off", true);
    Bind(wxEVT_MENU, &MainFrame::OnShowSharpEdges, this, ID_SHOW_SHARP_EDGES);

    menu->Append(ID_SHOW_TRI_NORMALS, "Show Tri Normals", "Turns rendering of triangle normals on/off", true);
    Bind(wxEVT_MENU, &MainFrame::OnShowTriNormals, this, ID_SHOW_TRI_NORMALS);

    menu->Append(ID_SHOW_FACES, "Show Faces", "Turns rendering of faces on/off", true);
    Bind(wxEVT_MENU, &MainFrame::OnShowFaces, this, ID_SHOW_FACES);

    menu->Append(ID_SHOW_EDGES, "Show Edges", "Turns rendering of edges on/off", true);
    Bind(wxEVT_MENU, &MainFrame::OnShowEdges, this, ID_SHOW_EDGES);

    menu->Append(ID_SHOW_OUTER, "Show Outer", "Turns rendering of edges on/off", true);
    Bind(wxEVT_MENU, &MainFrame::OnShowOuter, this, ID_SHOW_OUTER);

    auto sharpItem = _menuBar->FindItem(ID_SHOW_SHARP_EDGES);
    if (sharpItem)
        sharpItem->Check(getCanvas()->showSharpEdges());

    auto normItem = _menuBar->FindItem(ID_SHOW_TRI_NORMALS);
    if (normItem)
        normItem->Check(getCanvas()->showTriNormals());

    auto facesItem = _menuBar->FindItem(ID_SHOW_FACES);
    if (facesItem)
        facesItem->Check(getCanvas()->showFaces());

    auto edgesItem = _menuBar->FindItem(ID_SHOW_EDGES);
    if (edgesItem)
        edgesItem->Check(getCanvas()->showEdges());

    auto outerItem = _menuBar->FindItem(ID_SHOW_OUTER);
    if (outerItem)
        outerItem->Check(getCanvas()->showOuter());

    _menuBar->Append(menu, "&View");
}

void MainFrame::createHelpMenu()
{
    wxMenu* menuHelp = new wxMenu;
    menuHelp->Append(wxID_ABOUT);
    Bind(wxEVT_MENU, &MainFrame::OnAbout, this, wxID_ABOUT);

    _menuBar->Append(menuHelp, "&Help");


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
        _editMenu->Enable(ID_VerifyClosed, _pAppData->getMesh() != nullptr);
        _editMenu->Enable(ID_VerifyNormals, _pAppData->getMesh() != nullptr);
        _editMenu->Enable(ID_AnalyzeGaps, _pAppData->getMesh() != nullptr);
        _editMenu->Enable(ID_FindMinimumGap, _pAppData->getMesh() != nullptr);
        _editMenu->Enable(ID_BuildCFDHexes, _pAppData->getMesh() != nullptr);
    }
    if (_fileMenu) {

    }
}

void MainFrame::OnOpen(wxCommandEvent& event)
{
    _pAppData->doOpen();
}

void MainFrame::OnNew(wxCommandEvent& event)
{
    MakeBlockDlg dlg(this, 1, wxString("Make Block"), wxPoint(40,40));
    if (dlg.ShowModal() == wxID_OK) {
    	_pAppData->doNew(dlg);
    }
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
    _pAppData->doBuildCFDHexes();
}

void MainFrame::OnShowSharpEdges(wxCommandEvent& event)
{
    getCanvas()->toggleShowSharpEdges();

    auto sharpItem = _menuBar->FindItem(ID_SHOW_SHARP_EDGES);
    if (sharpItem)
        sharpItem->Check(getCanvas()->showSharpEdges());

}

void MainFrame::OnShowTriNormals(wxCommandEvent& event)
{
    getCanvas()->toggleShowTriNormals();

    auto normItem = _menuBar->FindItem(ID_SHOW_TRI_NORMALS);
    if (normItem)
        normItem->Check(getCanvas()->showTriNormals());
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
