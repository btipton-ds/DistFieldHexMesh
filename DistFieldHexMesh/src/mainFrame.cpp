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
#include <ctime>
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
#include <createBaseMeshDlg.h>
#include <graphicsCanvas.h>
#include <graphicsDebugCanvas.h>
#include <meshData.h>
#include <volume.h>
#include <vertex.h>

using namespace std;
using namespace DFHM;

#define PROG_MAX 300

BEGIN_EVENT_TABLE(MainFrame, wxFrame)
EVT_PAINT(MainFrame::doPaint)
END_EVENT_TABLE()

const std::shared_ptr<wxGLContext>& MainFrame::getGLContext(wxGLCanvas* pCanvas)
{
    static std::shared_ptr<wxGLContext> pContext;
    if (!pContext)
        pContext = make_shared<wxGLContext>(pCanvas);
    else
        pContext->SetCurrent(*pCanvas);
    return pContext;
}

namespace
{
    int attribs[] = {
        WX_GL_DEPTH_SIZE, 16,
#if GRAPHICS_OVER_SAMPLING > 1
        WX_GL_SAMPLES, GRAPHICS_OVER_SAMPLING * GRAPHICS_OVER_SAMPLING,
#endif
        0
    };

}

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

    _pSizer = new wxBoxSizer(wxHORIZONTAL);
    SetSizer(_pSizer);

    _pCanvas = new GraphicsCanvas(this, _pAppData);
    rgbaColor backColor(0.95f, 0.95f, 1.0f, 1.0f);
    _pCanvas->setBackColor(backColor);

#if INCLUDE_DEBUG_WX_FRAME
    _pDebugCanvas = new GraphicsDebugCanvas(this);
    _pCanvas->setDebugCanvas(_pDebugCanvas);
#endif

    _pObjectTree = new ObjectTreeCtrl(this, ID_OBJ_TREE_CTRL, wxDefaultPosition, wxSize(200, 200));

    _pSizer->Add(_pObjectTree, 0, wxEXPAND | wxDOWN, FromDIP(0));

    _pSizer->GetItemCount();
    _pSizer->Add(_pCanvas, 1, wxEXPAND | wxALL, FromDIP(0));

    _debugFrameIndex = _pSizer->GetItemCount();
#if INCLUDE_DEBUG_WX_FRAME
    _pSizer->Add(_pDebugCanvas, 1, wxEXPAND | wxALL, FromDIP(0));
    _pSizer->Hide(_debugFrameIndex);
#endif

    addMenus();
    addStatusBar();

    wxBitmapBundle bitMap0, bitMap1;
    bitMap0.FromFiles("D:/DarkSky/Projects/DistFieldHexMesh/DistFieldHexMesh/src/SheetIcon.bmp");
    bitMap1.FromFiles("D:/DarkSky/Projects/DistFieldHexMesh/DistFieldHexMesh/src/SolidIcon.bmp");
    _images.push_back(bitMap0);
    _images.push_back(bitMap1);
}

MainFrame::~MainFrame()
{
    _pAppData->preDestroy();
    _pAppData = nullptr;
}

void MainFrame::addMenus()
{
    _menuBar = new wxMenuBar;

    createFileMenu();
    createEditMenu();
    createViewMenu();
    createHelpMenu();
#if INCLUDE_DEBUG_WX_FRAME
    createDebugMenu();
#endif

    SetMenuBar(_menuBar);
}

void MainFrame::doPaint(wxPaintEvent& WXUNUSED(event)) {
//    wxPaintDC dc(this); // Despite the documentation, wxPaintDC BREAKS many things and the destructor DOES NOT roll up the stack with multiple GlCanvases.
    _pCanvas->render();

#if INCLUDE_DEBUG_WX_FRAME
    if (_pDebugCanvas)
        _pDebugCanvas->render();
#endif

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

    _editMenu->Append(ID_CREATE_BASE_MESH, "Create Base Mesh...");
    Bind(wxEVT_MENU, &MainFrame::OnCreateBaseMesh, this, ID_CREATE_BASE_MESH);

    _editMenu->Append(ID_BuildCFDHexes, "Build CFD Hexes...");
    Bind(wxEVT_MENU, &MainFrame::OnBuildCFDHexes, this, ID_BuildCFDHexes);

    _menuBar->Append(_editMenu, "&Edit");

}

void MainFrame::createViewMenu()
{
    _viewMenu = new wxMenu;

    addStandardViewsSubMenu(_viewMenu);

    _viewMenu->Append(ID_SHOW_MODEL_FACES, "Model - Show Faces", "Turns rendering of faces on/off", true);
    Bind(wxEVT_MENU, &MainFrame::OnShowModelFaces, this, ID_SHOW_MODEL_FACES);

    _viewMenu->Append(ID_SHOW_MODEL_EDGES, "Model - Show Edges", "Turns rendering of edges on/off", true);
    Bind(wxEVT_MENU, &MainFrame::OnShowModelEdges, this, ID_SHOW_MODEL_EDGES);

    _viewMenu->Append(ID_SHOW_MODEL_SHARP_EDGES, "Model - Show Sharp Edges", "Turns rendering of sharp edges on/off", true);
    Bind(wxEVT_MENU, &MainFrame::OnShowModelSharpEdges, this, ID_SHOW_MODEL_SHARP_EDGES);

    _viewMenu->Append(ID_SHOW_SHARP_VERTS, "Model - Show Sharp Vertices", "Turns rendering of sharp vertices on/off", true);
    Bind(wxEVT_MENU, &MainFrame::OnShowModelSharpVerts, this, ID_SHOW_SHARP_VERTS);

    _viewMenu->Append(ID_SHOW_TRI_NORMALS, "Model - Show Tri Normals", "Turns rendering of triangle normals on/off", true);
    Bind(wxEVT_MENU, &MainFrame::OnShowModelTriNormals, this, ID_SHOW_TRI_NORMALS);

    _viewMenu->Append(ID_SHOW_CURVATURE, "Model - Show Curvature", "Turns rendering of curvature on/off", true);
    Bind(wxEVT_MENU, &MainFrame::OnShowModelCurvature, this, ID_SHOW_CURVATURE);

    _viewMenu->AppendSeparator();

    addBoundarySubMenu(_viewMenu);

    _viewMenu->Append(ID_SHOW_MESH_FACES, "Mesh - Show Faces", "Turns rendering of faces on/off", true);
    Bind(wxEVT_MENU, &MainFrame::OnShowMeshFaces, this, ID_SHOW_MESH_FACES);

    _viewMenu->Append(ID_SHOW_MESH_EDGES, "Mesh - Show Edges", "Turns rendering of edges on/off", true);
    Bind(wxEVT_MENU, &MainFrame::OnShowMeshEdges, this, ID_SHOW_MESH_EDGES);

    _viewMenu->Append(ID_SHOW_MESH_WALL, "Mesh - Show Walls", "Turns rendering of walls on/off", true);
    Bind(wxEVT_MENU, &MainFrame::OnShowMeshWalls, this, ID_SHOW_MESH_WALL);

    _viewMenu->Append(ID_SHOW_MESH_ALL_BLOCKS, "Mesh - Show all Blocks", "Shows all blocks", true);
    Bind(wxEVT_MENU, &MainFrame::OnShowAllBlocks, this, ID_SHOW_MESH_ALL_BLOCKS);

    _viewMenu->Append(ID_SHOW_MESH_SELECTED_BLOCKS, "Mesh - Show Selected Blocks", "Shows only selected blocks", true);
    Bind(wxEVT_MENU, &MainFrame::OnShowMeshSelectedBlocks, this, ID_SHOW_MESH_SELECTED_BLOCKS);
    addLayersMenu(_viewMenu);

    _menuBar->Append(_viewMenu, "&View");

}

void MainFrame::addStandardViewsSubMenu(wxMenu* pParentMenu)
{
    _viewStandardViewsSubMenu = new wxMenu;

    _viewStandardViewsSubMenu->Append(ID_VIEW_FRONT, "Front", "Set view to front", false);
    Bind(wxEVT_MENU, &MainFrame::OnSetViewFront, this, ID_VIEW_FRONT);

    _viewStandardViewsSubMenu->Append(ID_VIEW_BACK, "Back", "Set view to back", false);
    Bind(wxEVT_MENU, &MainFrame::OnSetViewBack, this, ID_VIEW_BACK);

    _viewStandardViewsSubMenu->Append(ID_VIEW_RIGHT, "Right", "Set view to right", false);
    Bind(wxEVT_MENU, &MainFrame::OnSetViewRight, this, ID_VIEW_RIGHT);

    _viewStandardViewsSubMenu->Append(ID_VIEW_LEFT, "Left", "Set view to left", false);
    Bind(wxEVT_MENU, &MainFrame::OnSetViewLeft, this, ID_VIEW_LEFT);

    _viewStandardViewsSubMenu->Append(ID_VIEW_TOP, "Top", "Set view to top", false);
    Bind(wxEVT_MENU, &MainFrame::OnSetViewTop, this, ID_VIEW_TOP);

    _viewStandardViewsSubMenu->Append(ID_VIEW_BOTTOM, "Bottom", "Set view to bottom", false);
    Bind(wxEVT_MENU, &MainFrame::OnSetViewBottom, this, ID_VIEW_BOTTOM);

    _viewStandardViewsSubMenu->AppendSeparator();

    _viewStandardViewsSubMenu->Append(ID_VIEW_RESET, "Reset", "Reset view to default", false);
    Bind(wxEVT_MENU, &MainFrame::OnResetView, this, ID_VIEW_RESET);

    pParentMenu->AppendSubMenu(_viewStandardViewsSubMenu, "Principal Views", "Principle views");
}

void MainFrame::addBoundarySubMenu(wxMenu* pParentMenu)
{
    _viewBoundarySubMenu = new wxMenu;

    _viewBoundarySubMenu->Append(ID_SHOW_ALL_SIDES, "Show All", "Show all boundary faces", false);
    Bind(wxEVT_MENU, &MainFrame::OnShowAllSides, this, ID_SHOW_ALL_SIDES);

    _viewBoundarySubMenu->Append(ID_HIDE_ALL_SIDES, "Hide All", "Hide all boundary faces", false);
    Bind(wxEVT_MENU, &MainFrame::OnHideAllSides, this, ID_HIDE_ALL_SIDES);

    _viewBoundarySubMenu->AppendSeparator();

    _viewBoundarySubMenu->Append(ID_SHOW_FRONT, "Front", "Show front boundary faces", true);
    Bind(wxEVT_MENU, &MainFrame::OnShowFront, this, ID_SHOW_FRONT);

    _viewBoundarySubMenu->Append(ID_SHOW_BACK, "Back", "Show back boundary faces", true);
    Bind(wxEVT_MENU, &MainFrame::OnShowBack, this, ID_SHOW_BACK);

    _viewBoundarySubMenu->Append(ID_SHOW_RIGHT, "Right", "Show right boundary faces", true);
    Bind(wxEVT_MENU, &MainFrame::OnShowRight, this, ID_SHOW_RIGHT);

    _viewBoundarySubMenu->Append(ID_SHOW_LEFT, "Left", "Show left boundary faces", true);
    Bind(wxEVT_MENU, &MainFrame::OnShowLeft, this, ID_SHOW_LEFT);

    _viewBoundarySubMenu->Append(ID_SHOW_TOP, "Top", "Show top boundary faces", true);
    Bind(wxEVT_MENU, &MainFrame::OnShowTop, this, ID_SHOW_TOP);

    _viewBoundarySubMenu->Append(ID_SHOW_BOTTOM, "Bottom", "Show bottom boundary faces", true);
    Bind(wxEVT_MENU, &MainFrame::OnShowBottom, this, ID_SHOW_BOTTOM);

    pParentMenu->AppendSubMenu(_viewBoundarySubMenu, "Show boundaries", "Boundary face drawing");
}

#define STRINGIFY(X) #X

#define IMPL_LAYER1(NUM) \
_layersSubMenu->Append(ID_SHOW_MESH_LAYER_##NUM, "Show layer " STRINGIFY(NUM), "Show layer " STRINGIFY(NUM), true); \
Bind(wxEVT_MENU, &MainFrame::OnShowLayer##NUM, this, ID_SHOW_MESH_LAYER_##NUM)

#define IMPL_LAYER2(NUM) \
_layersSubMenu->Append(ID_SHOW_MESH_LAYER_##NUM, "Show layers 0-" STRINGIFY(NUM), "Show layers 0-" STRINGIFY(NUM), true); \
Bind(wxEVT_MENU, &MainFrame::OnShowLayer##NUM, this, ID_SHOW_MESH_LAYER_##NUM)

void MainFrame::addLayersMenu(wxMenu* pParentMenu)
{
    _layersSubMenu = new wxMenu;

    _layersSubMenu->Append(ID_SHOW_MESH_LAYERS_OFF, "Show layers off", "Turn display of all layers off", false);
    Bind(wxEVT_MENU, &MainFrame::OnShowLayersOff, this, ID_SHOW_MESH_LAYERS_OFF);

    IMPL_LAYER1(0);
    IMPL_LAYER2(1);
    IMPL_LAYER2(2);
    IMPL_LAYER2(3);
    IMPL_LAYER2(4);
    IMPL_LAYER2(5);
    IMPL_LAYER2(6);
    IMPL_LAYER2(7);
    IMPL_LAYER2(8);
    _layersSubMenu->Append(ID_SHOW_MESH_LAYER_9, "Show layers 0-" "9", "Show layers 0-" "9", true); 
    Bind(wxEVT_MENU, &MainFrame::OnShowLayer9, this, ID_SHOW_MESH_LAYER_9);

    pParentMenu->AppendSubMenu(_layersSubMenu, "Show Layers", "Boundary layer drawing");
}
void MainFrame::createHelpMenu()
{
    wxMenu* pMenu = new wxMenu;

    pMenu->Append(wxID_ABOUT);
    Bind(wxEVT_MENU, &MainFrame::OnAbout, this, wxID_ABOUT);

    _menuBar->Append(pMenu, "&Help");


}
#if INCLUDE_DEBUG_WX_FRAME

void MainFrame::createDebugMenu()
{
    wxMenu* pMenu = new wxMenu;

    pMenu->Append(ID_TOGGLE_DEBUG_FRAME, "Toggle Debug Frame", "Show debug render frame");
    Bind(wxEVT_MENU, &MainFrame::OnToggleDebugFrame, this, ID_TOGGLE_DEBUG_FRAME);

    _menuBar->Append(pMenu, "&Debug");
}

#endif

void MainFrame::addStatusBar()
{
    _statusBar = new wxStatusBar(this, wxID_ANY, wxST_SIZEGRIP | wxNO_BORDER);
    _statusBar->SetFieldsCount(2); // all fields will have text values except the second where I want the wxGauge
    SetStatusBar(_statusBar);

    wxRect rect;
    auto field0 = _statusBar->GetField(0);
    field0.SetWidth(250);
    _statusBar->GetFieldRect(1, rect);
    _progress = new wxGauge(_statusBar, ID_QUERY_PROGRESS, 0, rect.GetPosition(), rect.GetSize(), wxGA_SMOOTH);
    _progress->SetRange(PROG_MAX);
    _progress->SetValue(0);

    Bind(wxEVT_UPDATE_UI, &MainFrame::OnUpdateUI, this);
}

void MainFrame::OnInternalIdle()
{
    wxFrame::OnInternalIdle();

    _pCanvas->Refresh();

    const auto& pMeshData = _pAppData->getMeshData();
    const auto& pVolume = _pAppData->getVolume();

    bool hasModel = (pMeshData && !pMeshData->empty());
    bool hasMesh = (pVolume && pVolume->numPolyhedra() > 0);

    if (_editMenu) {
        _editMenu->Enable(ID_CREATE_BASE_MESH, hasModel);
        _editMenu->Enable(ID_BuildCFDHexes, hasMesh);
    }

    if (_fileMenu) {

    }

    if (_viewMenu) {
        _viewMenu->Check(ID_SHOW_MODEL_FACES, _pCanvas->showModelFaces());
        _viewMenu->Check(ID_SHOW_MODEL_EDGES, _pCanvas->showModelEdges());
        _viewMenu->Check(ID_SHOW_MODEL_SHARP_EDGES, _pCanvas->showModelSharpEdges());
        _viewMenu->Check(ID_SHOW_SHARP_VERTS, _pCanvas->showSharpVerts());
        _viewMenu->Check(ID_SHOW_TRI_NORMALS, _pCanvas->showTriNormals());
        _viewMenu->Check(ID_SHOW_CURVATURE, _pCanvas->showCurvature());

        _viewMenu->Check(ID_SHOW_MESH_FACES, _pCanvas->showMeshFaces());
        _viewMenu->Check(ID_SHOW_MESH_EDGES, _pCanvas->showMeshEdges());
        _viewMenu->Check(ID_SHOW_MESH_WALL, _pCanvas->showMeshWalls());
//        _viewMenu->Check(ID_SHOW_MESH_SELECTED_BLOCKS, _pCanvas->showMeshFaces());


        _viewMenu->Enable(ID_SHOW_MODEL_FACES, hasModel);
        _viewMenu->Enable(ID_SHOW_MODEL_EDGES, hasModel);
        _viewMenu->Enable(ID_SHOW_MODEL_SHARP_EDGES, hasModel);
        _viewMenu->Enable(ID_SHOW_SHARP_VERTS, hasModel);
        _viewMenu->Enable(ID_SHOW_TRI_NORMALS, hasModel);
        _viewMenu->Enable(ID_SHOW_CURVATURE, hasModel);

        _viewMenu->Enable(ID_SHOW_MESH_FACES, hasMesh);
        _viewMenu->Enable(ID_SHOW_MESH_EDGES, hasMesh);
        _viewMenu->Enable(ID_SHOW_MESH_WALL, hasMesh);
        _viewMenu->Enable(ID_SHOW_MESH_SELECTED_BLOCKS, hasMesh);

        if (_viewBoundarySubMenu) {
            _viewBoundarySubMenu->Check(ID_SHOW_FRONT, _pCanvas->showFace(GraphicsCanvas::VIEW_FRONT));
            _viewBoundarySubMenu->Check(ID_SHOW_BACK, _pCanvas->showFace(GraphicsCanvas::VIEW_BACK));
            _viewBoundarySubMenu->Check(ID_SHOW_LEFT, _pCanvas->showFace(GraphicsCanvas::VIEW_LEFT));
            _viewBoundarySubMenu->Check(ID_SHOW_RIGHT, _pCanvas->showFace(GraphicsCanvas::VIEW_RIGHT));
            _viewBoundarySubMenu->Check(ID_SHOW_BOTTOM, _pCanvas->showFace(GraphicsCanvas::VIEW_BOTTOM));
            _viewBoundarySubMenu->Check(ID_SHOW_TOP, _pCanvas->showFace(GraphicsCanvas::VIEW_TOP));

            _viewBoundarySubMenu->Enable(ID_SHOW_ALL_SIDES, hasMesh);
            _viewBoundarySubMenu->Enable(ID_HIDE_ALL_SIDES, hasMesh);

            _viewBoundarySubMenu->Enable(ID_SHOW_FRONT, hasMesh);
            _viewBoundarySubMenu->Enable(ID_SHOW_BACK, hasMesh);
            _viewBoundarySubMenu->Enable(ID_SHOW_LEFT, hasMesh);
            _viewBoundarySubMenu->Enable(ID_SHOW_RIGHT, hasMesh);
            _viewBoundarySubMenu->Enable(ID_SHOW_BOTTOM, hasMesh);
            _viewBoundarySubMenu->Enable(ID_SHOW_TOP, hasMesh);
        }

        if (_layersSubMenu) {
            _layersSubMenu->Check(ID_SHOW_MESH_LAYER_0, _pCanvas->showLayer(0));
            _layersSubMenu->Check(ID_SHOW_MESH_LAYER_1, _pCanvas->showLayer(1));
            _layersSubMenu->Check(ID_SHOW_MESH_LAYER_2, _pCanvas->showLayer(2));
            _layersSubMenu->Check(ID_SHOW_MESH_LAYER_3, _pCanvas->showLayer(3));
            _layersSubMenu->Check(ID_SHOW_MESH_LAYER_4, _pCanvas->showLayer(4));
            _layersSubMenu->Check(ID_SHOW_MESH_LAYER_5, _pCanvas->showLayer(5));
            _layersSubMenu->Check(ID_SHOW_MESH_LAYER_6, _pCanvas->showLayer(6));
            _layersSubMenu->Check(ID_SHOW_MESH_LAYER_7, _pCanvas->showLayer(7));
            _layersSubMenu->Check(ID_SHOW_MESH_LAYER_8, _pCanvas->showLayer(8));
            _layersSubMenu->Check(ID_SHOW_MESH_LAYER_9, _pCanvas->showLayer(9));

            _layersSubMenu->Enable(ID_SHOW_MESH_LAYERS_OFF, hasMesh && _pCanvas->showLayersOn());
            _layersSubMenu->Enable(ID_SHOW_MESH_LAYER_0, hasMesh);
            _layersSubMenu->Enable(ID_SHOW_MESH_LAYER_1, hasMesh);
            _layersSubMenu->Enable(ID_SHOW_MESH_LAYER_2, hasMesh);
            _layersSubMenu->Enable(ID_SHOW_MESH_LAYER_3, hasMesh);
            _layersSubMenu->Enable(ID_SHOW_MESH_LAYER_4, hasMesh);
            _layersSubMenu->Enable(ID_SHOW_MESH_LAYER_5, hasMesh);
            _layersSubMenu->Enable(ID_SHOW_MESH_LAYER_6, hasMesh);
            _layersSubMenu->Enable(ID_SHOW_MESH_LAYER_7, hasMesh);
            _layersSubMenu->Enable(ID_SHOW_MESH_LAYER_8, hasMesh);
            _layersSubMenu->Enable(ID_SHOW_MESH_LAYER_9, hasMesh);

        }
    }

    updateStatusBar();
}

void MainFrame::OnOpen(wxCommandEvent& event)
{
    if (_pAppData->doOpen()) {
        refreshObjectTree();
        _pCanvas->changeViewElements();
        _pCanvas->resetView();
    }
}

void MainFrame::refreshObjectTree()
{
    _pObjectTree->DeleteAllItems();
    const auto& meshObjects = _pAppData->getMeshData();
    _pObjectTree->SetImages(_images);
    auto solidsItem = _pObjectTree->AppendContainer(wxDataViewItem(), "Solids", -1, 0);
    auto surfacesItem = _pObjectTree->AppendContainer(wxDataViewItem(), "Surfaces", -1, 0);
    for (const auto& pair : *meshObjects) {
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
        _pCanvas->changeViewElements();
        _pCanvas->resetView();
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

void MainFrame::reportProgressInner(double fraction)
{
    int val = (int)(PROG_MAX * fraction + 0.5);
    if (val != _progressValue) {
        _progressValue = val;
        wxUpdateUIEvent evt(0);
        QueueEvent(evt.Clone());
    }

}

void MainFrame::OnWritePolymesh(wxCommandEvent& event)
{
    wxDirDialog dlg(this, "Choose OpenFoam Project Directory");
    if (dlg.ShowModal() == wxID_OK) {
        auto dirPath = dlg.GetPath().ToStdString();
        _pBackgroundFuture = make_shared<future<int>> (std::async(std::launch::async, [this, dirPath]()->int {
            _pAppData->getVolume()->writePolyMesh(dirPath, this);

            Sleep(500);

            return 1;
        }));
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

#if INCLUDE_DEBUG_WX_FRAME
void MainFrame::OnToggleDebugFrame(wxCommandEvent& event)
{
    bool isShown = _pSizer->IsShown(_debugFrameIndex);
    _pSizer->Show(_debugFrameIndex, !isShown);
    _pSizer->Layout();
}
#endif

void MainFrame::OnCut(wxCommandEvent& event)
{

}

void MainFrame::OnCopy(wxCommandEvent& event)
{

}

void MainFrame::OnPaste(wxCommandEvent& event)
{

}

void MainFrame::OnCreateBaseMesh(wxCommandEvent& event)
{
    CreateBaseMeshDlg* pDlg = new CreateBaseMeshDlg(_pAppData, this, 1, wxString("Create base volume"), wxPoint(40, 40));
    pDlg->Show();
}

void MainFrame::OnBuildCFDHexes(wxCommandEvent& event)
{
    BuildCFDParams& params = _pAppData->getParams();
    BuildCFDHexesDlg dlg(params, this, 1, wxString("Make Block"), wxPoint(40, 40));
    if (dlg.ShowModal() == wxID_OK) {
        _pAppData->doBuildCFDHexes(dlg);
    }
}

void MainFrame::OnShowModelSharpEdges(wxCommandEvent& event)
{
    getCanvas()->toggleShowModelSharpEdges();

    auto item = _menuBar->FindItem(ID_SHOW_MODEL_SHARP_EDGES);
    if (item)
        item->Check(getCanvas()->showModelSharpEdges());
}

void MainFrame::OnShowModelSharpVerts(wxCommandEvent& event)
{
    getCanvas()->toggleShowSharpVerts();

    auto item = _menuBar->FindItem(ID_SHOW_SHARP_VERTS);
    if (item)
        item->Check(getCanvas()->showSharpVerts());
}

void MainFrame::OnShowModelTriNormals(wxCommandEvent& event)
{
    getCanvas()->toggleShowTriNormals();

    auto normItem = _menuBar->FindItem(ID_SHOW_TRI_NORMALS);
    if (normItem)
        normItem->Check(getCanvas()->showTriNormals());
}

void MainFrame::OnShowModelCurvature(wxCommandEvent& event)
{
    getCanvas()->toggleShowCurvature();

    auto item = _menuBar->FindItem(ID_SHOW_CURVATURE);
    if (item)
        item->Check(getCanvas()->showCurvature());
}

void MainFrame::OnShowModelFaces(wxCommandEvent& event)
{
    getCanvas()->toggleShowModelFaces();

    auto normItem = _menuBar->FindItem(ID_SHOW_MODEL_FACES);
    if (normItem)
        normItem->Check(getCanvas()->showModelFaces());
}

void MainFrame::OnShowModelEdges(wxCommandEvent& event)
{
    getCanvas()->toggleShowModelEdges();

    auto normItem = _menuBar->FindItem(ID_SHOW_MODEL_EDGES);
    if (normItem)
        normItem->Check(getCanvas()->showModelEdges());
}

void MainFrame::OnShowMeshFaces(wxCommandEvent& event)
{
    getCanvas()->toggleShowMeshFaces();

    auto normItem = _menuBar->FindItem(ID_SHOW_MESH_FACES);
    if (normItem)
        normItem->Check(getCanvas()->showMeshFaces());
}

void MainFrame::OnShowMeshEdges(wxCommandEvent& event)
{
    getCanvas()->toggleShowMeshEdges();

    auto normItem = _menuBar->FindItem(ID_SHOW_MESH_EDGES);
    if (normItem)
        normItem->Check(getCanvas()->showMeshEdges());
}

void MainFrame::OnShowMeshWalls(wxCommandEvent& event)
{
    getCanvas()->toggleShowMeshWalls();

    auto normItem = _menuBar->FindItem(ID_SHOW_MESH_WALL);
    if (normItem)
        normItem->Check(getCanvas()->showMeshWalls());
}

void MainFrame::OnShowAllBlocks(wxCommandEvent& event)
{
    getCanvas()->toggleShowMeshAll();

    auto normItem = _menuBar->FindItem(ID_SHOW_MESH_WALL);
    if (normItem)
        normItem->Check(getCanvas()->showMeshAll());
}

void MainFrame::OnShowMeshSelectedBlocks(wxCommandEvent& event)
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

void MainFrame::OnShowAllSides(wxCommandEvent& event)
{
    if (!_pCanvas->showFace(GraphicsCanvas::VIEW_BACK))
        _pCanvas->toggleShowFace(GraphicsCanvas::VIEW_BACK);

    if (!_pCanvas->showFace(GraphicsCanvas::VIEW_FRONT))
        _pCanvas->toggleShowFace(GraphicsCanvas::VIEW_FRONT);

    if (!_pCanvas->showFace(GraphicsCanvas::VIEW_LEFT))
        _pCanvas->toggleShowFace(GraphicsCanvas::VIEW_LEFT);
    if (!_pCanvas->showFace(GraphicsCanvas::VIEW_RIGHT))
        _pCanvas->toggleShowFace(GraphicsCanvas::VIEW_RIGHT);

    if (!_pCanvas->showFace(GraphicsCanvas::VIEW_BOTTOM))
        _pCanvas->toggleShowFace(GraphicsCanvas::VIEW_BOTTOM);
    if (!_pCanvas->showFace(GraphicsCanvas::VIEW_TOP))
        _pCanvas->toggleShowFace(GraphicsCanvas::VIEW_TOP);

}

void MainFrame::OnHideAllSides(wxCommandEvent& event)
{
    if (_pCanvas->showFace(GraphicsCanvas::VIEW_BACK))
        _pCanvas->toggleShowFace(GraphicsCanvas::VIEW_BACK);

    if (_pCanvas->showFace(GraphicsCanvas::VIEW_FRONT))
        _pCanvas->toggleShowFace(GraphicsCanvas::VIEW_FRONT);

    if (_pCanvas->showFace(GraphicsCanvas::VIEW_LEFT))
        _pCanvas->toggleShowFace(GraphicsCanvas::VIEW_LEFT);
    if (_pCanvas->showFace(GraphicsCanvas::VIEW_RIGHT))
        _pCanvas->toggleShowFace(GraphicsCanvas::VIEW_RIGHT);

    if (_pCanvas->showFace(GraphicsCanvas::VIEW_BOTTOM))
        _pCanvas->toggleShowFace(GraphicsCanvas::VIEW_BOTTOM);
    if (_pCanvas->showFace(GraphicsCanvas::VIEW_TOP))
        _pCanvas->toggleShowFace(GraphicsCanvas::VIEW_TOP);

}

void MainFrame::OnShowFront(wxCommandEvent& event)
{
    _pCanvas->toggleShowFace(GraphicsCanvas::VIEW_FRONT);
    auto pItem = _menuBar->FindItem(ID_SHOW_FRONT);
    if (pItem)
        pItem->Check(getCanvas()->showFace(GraphicsCanvas::VIEW_FRONT));
}

void MainFrame::OnShowBack(wxCommandEvent& event)
{
    _pCanvas->toggleShowFace(GraphicsCanvas::VIEW_BACK);
    auto pItem = _menuBar->FindItem(ID_SHOW_BACK);
    if (pItem)
        pItem->Check(getCanvas()->showFace(GraphicsCanvas::VIEW_BACK));
}

void MainFrame::OnShowRight(wxCommandEvent& event)
{
    _pCanvas->toggleShowFace(GraphicsCanvas::VIEW_RIGHT);
    auto pItem = _menuBar->FindItem(ID_SHOW_RIGHT);
    if (pItem)
        pItem->Check(getCanvas()->showFace(GraphicsCanvas::VIEW_RIGHT));
}

void MainFrame::OnShowLeft(wxCommandEvent& event)
{
    _pCanvas->toggleShowFace(GraphicsCanvas::VIEW_LEFT);
    auto pItem = _menuBar->FindItem(ID_SHOW_LEFT);
    if (pItem)
        pItem->Check(getCanvas()->showFace(GraphicsCanvas::VIEW_LEFT));
}

void MainFrame::OnShowTop(wxCommandEvent& event)
{
    _pCanvas->toggleShowFace(GraphicsCanvas::VIEW_TOP);
    auto pItem = _menuBar->FindItem(ID_SHOW_TOP);
    if (pItem)
        pItem->Check(getCanvas()->showFace(GraphicsCanvas::VIEW_TOP));
}

void MainFrame::OnShowBottom(wxCommandEvent& event)
{
    _pCanvas->toggleShowFace(GraphicsCanvas::VIEW_BOTTOM);
    auto pItem = _menuBar->FindItem(ID_SHOW_BOTTOM);
    if (pItem)
        pItem->Check(getCanvas()->showFace(GraphicsCanvas::VIEW_BOTTOM));
}

void MainFrame::OnShowLayersOff(wxCommandEvent& event)
{
    getCanvas()->setShowLayer(-1);
}

void MainFrame::OnShowLayer0(wxCommandEvent& event)
{
    getCanvas()->setShowLayer(0);
}

void MainFrame::OnShowLayer1(wxCommandEvent& event)
{
    getCanvas()->setShowLayer(1);
}

void MainFrame::OnShowLayer2(wxCommandEvent& event)
{
    getCanvas()->setShowLayer(2);
}

void MainFrame::OnShowLayer3(wxCommandEvent& event)
{
    getCanvas()->setShowLayer(3);
}

void MainFrame::OnShowLayer4(wxCommandEvent& event)
{
    getCanvas()->setShowLayer(4);
}

void MainFrame::OnShowLayer5(wxCommandEvent& event)
{
    getCanvas()->setShowLayer(5);
}

void MainFrame::OnShowLayer6(wxCommandEvent& event)
{
    getCanvas()->setShowLayer(6);
}

void MainFrame::OnShowLayer7(wxCommandEvent& event)
{
    getCanvas()->setShowLayer(7);
}

void MainFrame::OnShowLayer8(wxCommandEvent& event)
{
    getCanvas()->setShowLayer(8);
}

void MainFrame::OnShowLayer9(wxCommandEvent& event)
{
    getCanvas()->setShowLayer(9);
}

void MainFrame::OnUpdateUI(wxUpdateUIEvent& event)
{
    if (_progress && _pBackgroundFuture && _pBackgroundFuture->valid()) {
        _progress->SetValue(_progressValue);
        if (_pBackgroundFuture->wait_for(0.1ms) == future_status::ready) {
            int result = _pBackgroundFuture->get(); // clear it
            _pBackgroundFuture = nullptr;
            _progressValue = 0;
            _progress->SetValue(_progressValue);
            if (result == 2) {
                auto pVol = _pAppData->getVolume();
                if (pVol) {
                    const Index3D min(0, 0, 0);
                    const Index3D max(pVol ? pVol->volDim() : Index3D());
                    _pAppData->setDisplayMinMax(min, max);
                    _pAppData->copyHexFaceTablesToVBOs();
                }
            }
        }
    }
}

void MainFrame::updateStatusBar()
{
    wstringstream ss;
    double t = clock() / (double)CLOCKS_PER_SEC;
    static auto lastTime = t - 10;

    if (t - lastTime > 5) {
        lastTime = t;
        t = clock() / (double)CLOCKS_PER_SEC;
        _numCells = _numFaces = _numBytes = 0;
        auto pVol = _pAppData->getVolume();
        if (pVol) {
            _numCells = pVol->numPolyhedra();
            _numFaces = pVol->numFaces(true);
        }
        _numBytes = _pAppData->numBytes();
        _numBytes += _pCanvas->numBytes();
    }

    Index3D dim;
    if (_pAppData) {
        const auto& pVol = _pAppData->getVolume();
        if (pVol)
            dim = pVol->volDim();
    }

    double mem = _numBytes / 1024.0 / 1024.0;
    if (dim.isValid())
        ss << L"Volume Dimension: (" << dim[0] << L"," << dim[1] << L"," << dim[2] << L"), ";

    ss << L"Num Cells: " << _numCells << ", Num Faces: " << _numFaces << ", Memory: " << mem << " mb";
    SetStatusText(ss.str());

}
