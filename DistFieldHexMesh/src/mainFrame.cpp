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
#include <graphicsCanvas.h>
#include <volume.h>

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
    
    _editMenu->Append(ID_FindMinimumGap, "Find Minimum Gap");
    Bind(wxEVT_MENU, &MainFrame::OnFindMinGap, this, ID_FindMinimumGap);

    _editMenu->Append(ID_BuildCFDHexes, "Build CFD Hexes");
    Bind(wxEVT_MENU, &MainFrame::OnBuildCFDHexes, this, ID_BuildCFDHexes);

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

    _pCanvas->Refresh();

    if (_editMenu) {
        _editMenu->Enable(ID_VerifyClosed, _pAppData->getMesh() != nullptr);
        _editMenu->Enable(ID_VerifyNormals, _pAppData->getMesh() != nullptr);
        _editMenu->Enable(ID_AnalyzeGaps, _pAppData->getMesh() != nullptr);
        _editMenu->Enable(ID_FindMinimumGap, _pAppData->getMesh() != nullptr);
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

void MainFrame::OnFindMinGap(wxCommandEvent& event)
{
    _pAppData->doFindMinGap();

}

void MainFrame::OnBuildCFDHexes(wxCommandEvent& event)
{
    _pAppData->doBuildCFDHexes();
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
            auto pCanvas = _pMainFrame->getCanvas();

            const auto& pts = _pMesh->getGlPoints();
            const auto& norms = _pMesh->getGlNormals(false);
            const auto& params = _pMesh->getGlParams();
            const auto& indices = _pMesh->getGlFaceIndices();

            pCanvas->beginFaceTesselation();
            auto faceTess = pCanvas->setFaceTessellation(_pMesh->getId(), _pMesh->getChangeNumber(), pts, norms, params, indices);
            pCanvas->endFaceTesselation(false);

            pCanvas->beginSettingFaceElementIndices(0xffffffffffffffff);
            pCanvas->includeFaceElementIndices(0, *faceTess);
            pCanvas->endSettingFaceElementIndices();

            vector<size_t> sharps;
            if (pMesh->collectSharpEdges(sharps, sin(30 * M_PI / 180.0)) > 0) {
                vector<float> pts;
                vector<int> indices;
                for (size_t i : sharps) {
                    const auto& edge = pMesh->getEdge(i);
                    const auto pt0 = pMesh->getVert(edge._vertIndex[0]);
                    const auto pt1 = pMesh->getVert(edge._vertIndex[1]);

                    for (int j = 0; j < 3; j++)
                        pts.push_back((float)pt0._pt[j]);

                    for (int j = 0; j < 3; j++)
                        pts.push_back((float)pt1._pt[j]);

                    indices.push_back((int)indices.size());
                    indices.push_back((int)indices.size());
                }

                pCanvas->beginEdgeTesselation();
                auto edgeTess = pCanvas->setEdgeSegTessellation(_pMesh->getId(), _pMesh->getChangeNumber(), pts, indices);
                pCanvas->endEdgeTesselation();

                pCanvas->beginSettingEdgeElementIndices(0xffffffffffffffff);
                pCanvas->includeEdgeElementIndices(0, *edgeTess);
                pCanvas->endSettingEdgeElementIndices();
            }
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

void AppData::doFindMinGap() const
{
    double t = _pMesh->findMinGap() * 10;
    auto bb = _pMesh->getBBox();
    auto bbMin = bb.getMin();
    auto range = bb.range();
    size_t numX = (size_t)(range[0] / t + 0.5);
    size_t numY = (size_t)(range[1] / t + 0.5);
    size_t numZ = (size_t)(range[2] / t + 0.5);

    numX = (numX / 8 + 1) * 8;
    numY = (numY / 8 + 1) * 8;
    numZ = (numZ / 8 + 1) * 8;
    Vector3i dim(numX, numY, numZ);


    MultiCore::runLambda([this, dim, bb](size_t threadNum, size_t numThreads) {
        auto range = bb.range();
        auto bbMin = bb.getMin();
        Vector3d zAxis(0, 0, 1);
        for (size_t ix = threadNum; ix < dim[0]; ix += numThreads) {
            double t = ix / (dim[0] - 1.0);
            double x = bbMin[0] + t * range[0];
            for (size_t iy = 0; iy < dim[1]; iy++) {
                double u = iy / (dim[1] - 1.0);
                double y = bbMin[1] + u * range[1];
                Vector3d ctr(x, y, 0);
                Ray ray(ctr, zAxis);

                vector<RayHit> hits;
                if (_pMesh->rayCast(ray, hits)) {
                    int dbgBreak = 1;
                }
            }
        }
        }, true);


    stringstream ss;
    ss << "Span: [" << numX << ", " << numY << ", " << numZ << "]\n";
    wxMessageBox(ss.str().c_str(), "Box span in steps", wxOK | wxICON_INFORMATION);
}

void AppData::doBuildCFDHexes()
{
    if (!_volume)
        _volume = make_shared<Volume>();

    Block::setBlockDim(16);

    double gap = 0.001;
    if (gap <= 0)
        gap = _pMesh->findMinGap();

    double superCellSize = gap * Block::getBlockDim();

    auto blockMesh = _volume->buildCFDHexes(_pMesh, superCellSize);
    auto pCanvas = _pMainFrame->getCanvas();

    pCanvas->beginFaceTesselation();
    auto triTess = pCanvas->setFaceTessellation(_pMesh->getId(), _pMesh->getChangeNumber(), _pMesh->getGlPoints(), _pMesh->getGlNormals(false), 
        _pMesh->getGlParams(), _pMesh->getGlFaceIndices());

    auto blockTess = pCanvas->setFaceTessellation(blockMesh->getId(), blockMesh->getChangeNumber(), blockMesh->getGlPoints(), blockMesh->getGlNormals(false),
        blockMesh->getGlParams(), blockMesh->getGlFaceIndices());

    pCanvas->endFaceTesselation(false);

    pCanvas->beginSettingFaceElementIndices(0xffffffffffffffff);
    pCanvas->includeFaceElementIndices(0, *triTess);
    pCanvas->includeFaceElementIndices(1, *blockTess);
    pCanvas->endSettingFaceElementIndices();
}
