#include <memory>
#include <wx/filedlg.h>
#include <wx/string.h>
#include <wx/wfstream.h>
#include <readStl.h>
#include <filesystem>

#ifdef WIN32
#include "windows.h"
#endif // WIN32

#include "mainFrame.h"

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
    cout << "Console up\n";

#pragma warning(pop)
#endif // WIN32


    _pAppData = make_shared<AppData>(this);
    addMenus();
    addStatusBar();
}

void MainFrame::addMenus()
{
    _menuBar = new wxMenuBar;

    createFileMenu();
    createEditMenu();
    createHelpMenu();

    SetMenuBar(_menuBar);
}

void MainFrame::createFileMenu()
{
    _fileMenu = new wxMenu;

    _fileMenu->Append(wxID_OPEN);
    Bind(wxEVT_MENU, &MainFrame::OnOpen, this, wxID_OPEN);

    _fileMenu->Append(wxID_NEW);
    Bind(wxEVT_MENU, &MainFrame::OnNew, this, wxID_NEW);

    _fileMenu->Append(wxID_CLOSE);
    Bind(wxEVT_MENU, &MainFrame::OnClose, this, wxID_CLOSE);

    _fileMenu->AppendSeparator();

    _fileMenu->Append(wxID_EXIT, "Quit\tCtrl-Q");
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
    
        

    _menuBar->Append(_editMenu, "&Edit");

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

    if (_editMenu) {
        _editMenu->Enable(ID_VerifyClosed, _pAppData->getMesh() != nullptr);
        _editMenu->Enable(ID_VerifyNormals, _pAppData->getMesh() != nullptr);
        _editMenu->Enable(ID_AnalyzeGaps, _pAppData->getMesh() != nullptr);
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

AppData::AppData(MainFrame* pMainFrame)
    : _pMainFrame(pMainFrame)
{

}

void AppData::doOpen()
{
    wxFileDialog openFileDialog(_pMainFrame, _("Open Triangle Mesh file"), "", "",
        "TriMesh files (*.stl)|*.stl", wxFD_OPEN | wxFD_FILE_MUST_EXIST);
    if (openFileDialog.ShowModal() == wxID_CANCEL)
        return;     // the user changed idea...

    // proceed loading the file chosen by the user;
    // this can be done with e.g. wxWidgets input streams:
    wxString pathStr = openFileDialog.GetPath();
    if (pathStr.find(".stl") != 0) {
        TriMesh::CMeshPtr pMesh = make_shared<TriMesh::CMesh>();
        CReadSTL reader(pMesh);

        wstring filename(openFileDialog.GetFilename().ToStdWstring());
        wstring path(pathStr.ToStdWstring());
        auto pos = path.find(filename);
        path = path.substr(0, pos);
        if (reader.read(path, filename)) {
            _pMesh = pMesh;

            wstring dumpTriFilename(path + L"triDump.txt");
            wstring dumpTreeFilename(path + L"treeDump.txt");
            if (!filesystem::exists(dumpTriFilename)) {
                _pMesh->dumpTris(dumpTriFilename);
                _pMesh->dumpTree(dumpTreeFilename);
            }
            if (!_pMesh->compareDumpedTris(dumpTriFilename)) {
                wxMessageBox(L"Check file did not match latest read file.", "Load Stl Error", wxOK | wxICON_INFORMATION);

            }

            stringstream ss;
            ss << "Tri mesh read. Num tris: " << _pMesh->numTris();
            wxMessageBox(ss.str().c_str(), "Load Stl Results", wxOK | wxICON_INFORMATION);
        }
    }
}

void AppData::doVerifyClosed()
{
    int numOpen = _pMesh->numLaminarEdges();

    stringstream ss;
    ss << "Number of edges: " << _pMesh->numEdges() << "\nNumber of open edges: " << numOpen;
    wxMessageBox(ss.str().c_str(), "Verify Closed", wxOK | wxICON_INFORMATION);
}

void AppData::doVerifyNormals()
{
    size_t numMisMatched = 0;
    size_t numEdges = _pMesh->numEdges();
    for (size_t i = 0; i < numEdges; i++) {
        const auto& edge = _pMesh->getEdge(i);
        if (edge._numFaces == 2) {
            size_t ptIdx0 = edge._vertIndex[0];
            size_t ptIdx1 = edge._vertIndex[1];

            const Vector3i& faceIndices0 = _pMesh->getTri(edge._faceIndex[0]);
            const Vector3i& faceIndices1 = _pMesh->getTri(edge._faceIndex[1]);

            bool face0Pos = false, face1Pos = false;
            for (int i = 0; i < 3; i++) {
                if (faceIndices0[i] == ptIdx0) {
                    face0Pos = (faceIndices0[(i + 1) % 3] == ptIdx1);
                    break;
                }
            }

            for (int i = 0; i < 3; i++) {
                if (faceIndices1[i] == ptIdx0) {
                    face1Pos = (faceIndices1[(i + 1) % 3] == ptIdx1);
                    break;
                }
            }

            if (face0Pos == face1Pos) {
                numMisMatched++;
            }
        }
    }
    stringstream ss;
    ss << "Number of tris: " << _pMesh->numTris() << "\nNumber of opposed faces: " << numMisMatched;
    wxMessageBox(ss.str().c_str(), "Verify Normals", wxOK | wxICON_INFORMATION);
}

void AppData::doAnalyzeGaps()
{
    vector<double> binSizes({ 0.050 / 64, 0.050 / 32, 0.050 / 16, 0.050 / 8, 0.050 / 4, 0.050 / 2, 0.050});
    vector<size_t> bins;
    bins.resize(binSizes.size(), 0);
    _pMesh->getGapHistogram(binSizes, bins, true);

    stringstream ss;
    ss << "Gap histogram\n";
    for (size_t i = 0; i < binSizes.size(); i++) {
        ss << "hits < " << binSizes[i] << ": " << bins[i] << "\n";
    }
    wxMessageBox(ss.str().c_str(), "Gap Analysis", wxOK | wxICON_INFORMATION);
}
