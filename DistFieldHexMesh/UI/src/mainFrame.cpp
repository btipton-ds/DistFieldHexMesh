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
#include <editPrefsDlg.h>
#include <selectBlocksDlg.h>
#include <divideHexMeshDlg.h>
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
    createDebugMenu();


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

    _fileMenu->Append(ID_EDIT_PREFS, "Edit Prefs...");
    Bind(wxEVT_MENU, &MainFrame::OnEditPrefs, this, ID_EDIT_PREFS);

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

    _editMenu->Append(ID_BuildCFDHexes, "Divide Mesh...");
    Bind(wxEVT_MENU, &MainFrame::OnDivideCFDHexes, this, ID_BuildCFDHexes);

    _menuBar->Append(_editMenu, "&Edit");

}

void MainFrame::createDebugMenu()
{
    _debugMenu = new wxMenu;

    _debugMenu->Append(ID_MESH_INFO, "Mesh Info", "Selects a face and reports info", true);
    Bind(wxEVT_MENU, &MainFrame::OnToggleMeshInfo, this, ID_MESH_INFO);

    _debugMenu->Append(ID_ADD_TO_MESH_DEBUG, "Select Break Cells", "Selects a face and reports info", true);
    Bind(wxEVT_MENU, &MainFrame::OnToggleMeshDebug, this, ID_ADD_TO_MESH_DEBUG);

    _debugMenu->Append(ID_MESH_DEBUG_SPLIT_CELL, "Test Split", "Selects a face and reports info", true);
    Bind(wxEVT_MENU, &MainFrame::OnTestCellSplit, this, ID_MESH_DEBUG_SPLIT_CELL);

#if INCLUDE_DEBUG_WX_FRAME
    _debugMenu->Append(ID_TOGGLE_DEBUG_FRAME, "Toggle Debug Frame", "Show debug render frame");
    Bind(wxEVT_MENU, &MainFrame::OnToggleDebugFrame, this, ID_TOGGLE_DEBUG_FRAME);
#endif
    _menuBar->Append(_debugMenu, "&Debug");
}

void MainFrame::createViewMenu()
{
    _viewMenu = new wxMenu;

    addStandardViewsSubMenu(_viewMenu);
    addModelViewsSubMenu(_viewMenu);
    addBoundarySubMenu(_viewMenu);
    addLayersMenu(_viewMenu);

    _viewMenu->AppendSeparator();

    auto modelMenu = new wxMenu();

    modelMenu->Append(ID_SHOW_MODEL_FACES, "Model - Show Faces", "Turns rendering of faces on/off", true);
    Bind(wxEVT_MENU, &MainFrame::OnShowModelFaces, this, ID_SHOW_MODEL_FACES);

    modelMenu->Append(ID_SHOW_MODEL_EDGES, "Model - Show Edges", "Turns rendering of edges on/off", true);
    Bind(wxEVT_MENU, &MainFrame::OnShowModelEdges, this, ID_SHOW_MODEL_EDGES);

    modelMenu->Append(ID_SHOW_MODEL_SHARP_EDGES, "Model - Show Sharp Edges", "Turns rendering of sharp edges on/off", true);
    Bind(wxEVT_MENU, &MainFrame::OnShowModelSharpEdges, this, ID_SHOW_MODEL_SHARP_EDGES);

    modelMenu->Append(ID_SHOW_SHARP_VERTS, "Model - Show Sharp Vertices", "Turns rendering of sharp vertices on/off", true);
    Bind(wxEVT_MENU, &MainFrame::OnShowModelSharpVerts, this, ID_SHOW_SHARP_VERTS);

    modelMenu->Append(ID_SHOW_TRI_NORMALS, "Model - Show Tri Normals", "Turns rendering of triangle normals on/off", true);
    Bind(wxEVT_MENU, &MainFrame::OnShowModelTriNormals, this, ID_SHOW_TRI_NORMALS);

    modelMenu->Append(ID_SHOW_CURVATURE, "Model - Show Curvature", "Turns rendering of curvature on/off", true);
    Bind(wxEVT_MENU, &MainFrame::OnShowModelCurvature, this, ID_SHOW_CURVATURE);

    _viewMenu->AppendSubMenu(modelMenu, "Model", "Model view controls");

    auto meshMenu = new wxMenu();

    meshMenu->Append(ID_SHOW_MESH_SELECTED_BLOCKS, "Mesh - Show Selected Blocks", "Shows only selected blocks", true);
    Bind(wxEVT_MENU, &MainFrame::OnShowMeshSelectedBlocks, this, ID_SHOW_MESH_SELECTED_BLOCKS);

    meshMenu->Append(ID_SHOW_MESH_FACES, "Mesh - Show Faces", "Turns rendering of faces on/off", true);
    Bind(wxEVT_MENU, &MainFrame::OnShowMeshFaces, this, ID_SHOW_MESH_FACES);

    meshMenu->Append(ID_SHOW_MESH_EDGES, "Mesh - Show Edges", "Turns rendering of edges on/off", true);
    Bind(wxEVT_MENU, &MainFrame::OnShowMeshEdges, this, ID_SHOW_MESH_EDGES);

    meshMenu->Append(ID_SHOW_MESH_WALL, "Mesh - Show Walls", "Turns rendering of walls on/off", true);
    Bind(wxEVT_MENU, &MainFrame::OnShowMeshWalls, this, ID_SHOW_MESH_WALL);

    meshMenu->Append(ID_SHOW_MESH_ALL_BLOCKS, "Mesh - Show all Blocks", "Shows all blocks", true);
    Bind(wxEVT_MENU, &MainFrame::OnShowAllBlocks, this, ID_SHOW_MESH_ALL_BLOCKS);

    _viewMenu->AppendSubMenu(meshMenu, "Mesh", "Mesh view controls");

    auto sectionMenu = new wxMenu();

    sectionMenu->Append(ID_ENABLE_CROSSSECTION_GRAPHICS, "Enable cross section graphics", "Show sections in X", true);
    Bind(wxEVT_MENU, &MainFrame::OnShowEnableSectionGraphics, this, ID_ENABLE_CROSSSECTION_GRAPHICS);

    sectionMenu->Append(ID_SHOW_SECTIONS_X, "Sections X", "Show sections in X", true);
    Bind(wxEVT_MENU, &MainFrame::OnShowSectionsX, this, ID_SHOW_SECTIONS_X);

    sectionMenu->Append(ID_SHOW_SECTIONS_Y, "Sections Y", "Show sections in Y", true);
    Bind(wxEVT_MENU, &MainFrame::OnShowSectionsY, this, ID_SHOW_SECTIONS_Y);

    sectionMenu->Append(ID_SHOW_SECTIONS_Z, "Sections Z", "Show sections in Z", true);
    Bind(wxEVT_MENU, &MainFrame::OnShowSectionsZ, this, ID_SHOW_SECTIONS_Z);

    _viewMenu->AppendSubMenu(sectionMenu, "Cross sections", "Cross section view controls");

    _viewMenu->AppendSeparator();

    _viewMenu->Append(ID_SHOW_CLIPPING_SINGLE, "Clipping plane", "Single clipping plane", true);
    Bind(wxEVT_MENU, &MainFrame::OnClippingSingle, this, ID_SHOW_CLIPPING_SINGLE);

    _viewMenu->Append(ID_SHOW_CLIPPING_DOUBLE, "Clipping slice", "Two clipping planes view a slice", true);
    Bind(wxEVT_MENU, &MainFrame::OnClippingDouble, this, ID_SHOW_CLIPPING_DOUBLE);

    _viewMenu->AppendSeparator();

    _viewMenu->Append(ID_CLIPPING_MOVE, "Move clipping plane", "Single clipping plane", true);
    Bind(wxEVT_MENU, &MainFrame::OnClippingMove, this, ID_CLIPPING_MOVE);

    _viewMenu->Append(ID_CLIPPING_ROTATE, "Rotate clipping plane", "Single clipping plane", true);
    Bind(wxEVT_MENU, &MainFrame::OnClippingRotate, this, ID_CLIPPING_ROTATE);

    _menuBar->Append(_viewMenu, "&View");

}

void MainFrame::addStandardViewsSubMenu(wxMenu* pParentMenu)
{
    auto menu = new wxMenu;

    menu->Append(ID_VIEW_FRONT, "Front", "Set view to front", false);
    Bind(wxEVT_MENU, &MainFrame::OnSetViewFront, this, ID_VIEW_FRONT);

    menu->Append(ID_VIEW_BACK, "Back", "Set view to back", false);
    Bind(wxEVT_MENU, &MainFrame::OnSetViewBack, this, ID_VIEW_BACK);

    menu->Append(ID_VIEW_RIGHT, "Right", "Set view to right", false);
    Bind(wxEVT_MENU, &MainFrame::OnSetViewRight, this, ID_VIEW_RIGHT);

    menu->Append(ID_VIEW_LEFT, "Left", "Set view to left", false);
    Bind(wxEVT_MENU, &MainFrame::OnSetViewLeft, this, ID_VIEW_LEFT);

    menu->Append(ID_VIEW_TOP, "Top", "Set view to top", false);
    Bind(wxEVT_MENU, &MainFrame::OnSetViewTop, this, ID_VIEW_TOP);

    menu->Append(ID_VIEW_BOTTOM, "Bottom", "Set view to bottom", false);
    Bind(wxEVT_MENU, &MainFrame::OnSetViewBottom, this, ID_VIEW_BOTTOM);

    menu->AppendSeparator();

    menu->Append(ID_VIEW_RESET, "Reset", "Reset view to default", false);
    Bind(wxEVT_MENU, &MainFrame::OnResetView, this, ID_VIEW_RESET);

    pParentMenu->AppendSubMenu(menu, "Principal Views", "Principle views");
}

void MainFrame::addModelViewsSubMenu(wxMenu* pParentMenu)
{
    auto menu = new wxMenu;

    menu->Append(ID_MODEL_VIEW_FRONT, "Front", "Set view to front", false);
    Bind(wxEVT_MENU, &MainFrame::OnSetModelViewFront, this, ID_MODEL_VIEW_FRONT);

    menu->Append(ID_MODEL_VIEW_BACK, "Back", "Set view to back", false);
    Bind(wxEVT_MENU, &MainFrame::OnSetModelViewBack, this, ID_MODEL_VIEW_BACK);

    menu->Append(ID_MODEL_VIEW_RIGHT, "Right", "Set view to right", false);
    Bind(wxEVT_MENU, &MainFrame::OnSetModelViewRight, this, ID_MODEL_VIEW_RIGHT);

    menu->Append(ID_MODEL_VIEW_LEFT, "Left", "Set view to left", false);
    Bind(wxEVT_MENU, &MainFrame::OnSetModelViewLeft, this, ID_MODEL_VIEW_LEFT);

    menu->Append(ID_MODEL_VIEW_TOP, "Top", "Set view to top", false);
    Bind(wxEVT_MENU, &MainFrame::OnSetModelViewTop, this, ID_MODEL_VIEW_TOP);

    menu->Append(ID_MODEL_VIEW_BOTTOM, "Bottom", "Set view to bottom", false);
    Bind(wxEVT_MENU, &MainFrame::OnSetModelViewBottom, this, ID_MODEL_VIEW_BOTTOM);

    menu->AppendSeparator();

    menu->Append(ID_MODEL_VIEW_RESET, "Reset", "Reset view to default", false);
    Bind(wxEVT_MENU, &MainFrame::OnResetModelView, this, ID_MODEL_VIEW_RESET);

    pParentMenu->AppendSubMenu(menu, "Model Views", "Principle views");
}

void MainFrame::addBoundarySubMenu(wxMenu* pParentMenu)
{
    auto menu = new wxMenu;

    menu->Append(ID_SHOW_ALL_SIDES, "Show All", "Show all boundary faces", false);
    Bind(wxEVT_MENU, &MainFrame::OnShowAllSides, this, ID_SHOW_ALL_SIDES);

    menu->Append(ID_HIDE_ALL_SIDES, "Hide All", "Hide all boundary faces", false);
    Bind(wxEVT_MENU, &MainFrame::OnHideAllSides, this, ID_HIDE_ALL_SIDES);

    menu->AppendSeparator();

    menu->Append(ID_SHOW_FRONT, "Front", "Show front boundary faces", true);
    Bind(wxEVT_MENU, &MainFrame::OnShowFront, this, ID_SHOW_FRONT);

    menu->Append(ID_SHOW_BACK, "Back", "Show back boundary faces", true);
    Bind(wxEVT_MENU, &MainFrame::OnShowBack, this, ID_SHOW_BACK);

    menu->Append(ID_SHOW_RIGHT, "Right", "Show right boundary faces", true);
    Bind(wxEVT_MENU, &MainFrame::OnShowRight, this, ID_SHOW_RIGHT);

    menu->Append(ID_SHOW_LEFT, "Left", "Show left boundary faces", true);
    Bind(wxEVT_MENU, &MainFrame::OnShowLeft, this, ID_SHOW_LEFT);

    menu->Append(ID_SHOW_TOP, "Top", "Show top boundary faces", true);
    Bind(wxEVT_MENU, &MainFrame::OnShowTop, this, ID_SHOW_TOP);

    menu->Append(ID_SHOW_BOTTOM, "Bottom", "Show bottom boundary faces", true);
    Bind(wxEVT_MENU, &MainFrame::OnShowBottom, this, ID_SHOW_BOTTOM);

    pParentMenu->AppendSubMenu(menu, "Show boundaries", "Boundary face drawing");
}

#define STRINGIFY(X) #X

#define IMPL_LAYER1(NUM) \
menu->Append(ID_SHOW_MESH_LAYER_##NUM, "Show layer " STRINGIFY(NUM), "Show layer " STRINGIFY(NUM), true); \
Bind(wxEVT_MENU, &MainFrame::OnShowLayer##NUM, this, ID_SHOW_MESH_LAYER_##NUM)

#define IMPL_LAYER2(NUM0, NUM1) \
menu->Append(ID_SHOW_MESH_LAYER_##NUM1, "Show layers " STRINGIFY(NUM0) "-" STRINGIFY(NUM1), "Show layers " STRINGIFY(NUM0) "-" STRINGIFY(NUM1), true); \
Bind(wxEVT_MENU, &MainFrame::OnShowLayer##NUM1, this, ID_SHOW_MESH_LAYER_##NUM1)

void MainFrame::addLayersMenu(wxMenu* pParentMenu)
{
    auto menu = new wxMenu;

    menu->Append(ID_SHOW_MESH_LAYERS_OFF, "Show layers off", "Turn display of all layers off", false);
    Bind(wxEVT_MENU, &MainFrame::OnShowLayersOff, this, ID_SHOW_MESH_LAYERS_OFF);

    IMPL_LAYER1(0);
    IMPL_LAYER2(0, 1);
    IMPL_LAYER2(1, 2);
    IMPL_LAYER2(2, 3);
    IMPL_LAYER2(3, 4);

    pParentMenu->AppendSubMenu(menu, "Show Layers", "Boundary layer drawing");
}
void MainFrame::createHelpMenu()
{
    wxMenu* pMenu = new wxMenu;

    pMenu->Append(wxID_ABOUT);
    Bind(wxEVT_MENU, &MainFrame::OnAbout, this, wxID_ABOUT);

    _menuBar->Append(pMenu, "&Help");


}

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

    const auto& model = _pAppData->getModel();
    const auto& pVolume = _pAppData->getVolume();

    bool hasModel = (!model.empty());
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

        _viewMenu->Check(ID_ENABLE_CROSSSECTION_GRAPHICS, _pCanvas->drawSectionsEnabled());
        _viewMenu->Check(ID_SHOW_SECTIONS_X, _pCanvas->showSections(0));
        _viewMenu->Check(ID_SHOW_SECTIONS_Y, _pCanvas->showSections(1));
        _viewMenu->Check(ID_SHOW_SECTIONS_Z, _pCanvas->showSections(2));

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

        _viewMenu->Enable(ID_SHOW_SECTIONS_X, _pCanvas->drawSectionsEnabled() && _pCanvas->hasSections());
        _viewMenu->Enable(ID_SHOW_SECTIONS_Y, _pCanvas->drawSectionsEnabled() && _pCanvas->hasSections());
        _viewMenu->Enable(ID_SHOW_SECTIONS_Z, _pCanvas->drawSectionsEnabled() && _pCanvas->hasSections());

        checkItem(ID_SHOW_FRONT, _pCanvas->showFace(GraphicsCanvas::VIEW_FRONT));
        checkItem(ID_SHOW_BACK, _pCanvas->showFace(GraphicsCanvas::VIEW_BACK));
        checkItem(ID_SHOW_LEFT, _pCanvas->showFace(GraphicsCanvas::VIEW_LEFT));
        checkItem(ID_SHOW_RIGHT, _pCanvas->showFace(GraphicsCanvas::VIEW_RIGHT));
        checkItem(ID_SHOW_BOTTOM, _pCanvas->showFace(GraphicsCanvas::VIEW_BOTTOM));
        checkItem(ID_SHOW_TOP, _pCanvas->showFace(GraphicsCanvas::VIEW_TOP));

        enableItem(ID_SHOW_ALL_SIDES, hasMesh);
        enableItem(ID_HIDE_ALL_SIDES, hasMesh);

        enableItem(ID_SHOW_FRONT, hasMesh);
        enableItem(ID_SHOW_BACK, hasMesh);
        enableItem(ID_SHOW_LEFT, hasMesh);
        enableItem(ID_SHOW_RIGHT, hasMesh);
        enableItem(ID_SHOW_BOTTOM, hasMesh);
        enableItem(ID_SHOW_TOP, hasMesh);

        checkItem(ID_SHOW_MESH_LAYER_0, _pCanvas->showLayer(0));
        checkItem(ID_SHOW_MESH_LAYER_1, _pCanvas->showLayer(1));
        checkItem(ID_SHOW_MESH_LAYER_2, _pCanvas->showLayer(2));
        checkItem(ID_SHOW_MESH_LAYER_3, _pCanvas->showLayer(3));
        checkItem(ID_SHOW_MESH_LAYER_4, _pCanvas->showLayer(4));

        enableItem(ID_SHOW_MESH_LAYERS_OFF, hasMesh && _pCanvas->showLayersOn());
        enableItem(ID_SHOW_MESH_LAYER_0, hasMesh);
        enableItem(ID_SHOW_MESH_LAYER_1, hasMesh);
        enableItem(ID_SHOW_MESH_LAYER_2, hasMesh);
        enableItem(ID_SHOW_MESH_LAYER_3, hasMesh);
        enableItem(ID_SHOW_MESH_LAYER_4, hasMesh);
    }

    updateStatusBar();
}

void MainFrame::checkItem(int itemId, bool val)
{
    auto pItem = _menuBar->FindItem(itemId);
    if (pItem) {
        pItem->Check(val);
    }
}

void MainFrame::enableItem(int itemId, bool val)
{
    auto pItem = _menuBar->FindItem(itemId);
    if (pItem) {
        pItem->Enable(val);
    }
}

void MainFrame::OnOpen(wxCommandEvent& event)
{
    if (_pAppData->doOpen()) {
        refreshObjectTree();
        _pCanvas->changeViewElements();
        OnResetView(event);
        resetClippingPlanes();
    }
}

void MainFrame::resetClippingPlanes()
{
    auto pVol = _pAppData->getVolume();
    auto& pts = pVol->getModelCornerPts();
    Vector3d origin = TRI_LERP(pts, Vector3d(0.5, 0.5, 0.5));
    Vector3d pt0 = TRI_LERP(pts, Vector3d(0, 0.5, 0.5));
    Vector3d pt1 = TRI_LERP(pts, Vector3d(1, 0.5, 0.5));
    Vector3d dir = (pt1 - pt0).normalized();

    Planed sectionPlane0(origin, dir);
    _pCanvas->setClipplingPlane(0, sectionPlane0);

    Planed sectionPlane1(origin + dir * 0.5, -dir);

    _pCanvas->setClipplingPlane(0, sectionPlane0);
    _pCanvas->setClipplingPlane(1, sectionPlane1);
}

void MainFrame::refreshObjectTree()
{
    _pObjectTree->DeleteAllItems();
    const auto& model = _pAppData->getModel();
    _pObjectTree->SetImages(_images);
    auto solidsItem = _pObjectTree->AppendContainer(wxDataViewItem(), "Solids", -1, 0);
    auto surfacesItem = _pObjectTree->AppendContainer(wxDataViewItem(), "Surfaces", -1, 0);
    for (const auto& pData : model) {
        wxDataViewItem item;
        const auto pMesh = pData->getMesh();
        if (pMesh->isClosed())
            item = _pObjectTree->AppendItem(solidsItem, pData->getName());
        else
            item = _pObjectTree->AppendItem(surfacesItem, pData->getName());
    }
    _pObjectTree->Expand(solidsItem);
    _pObjectTree->Expand(surfacesItem);
}

void MainFrame::OnImportMesh(wxCommandEvent& event)
{
    if (_pAppData->doImportMesh()) {
        refreshObjectTree();
        _pCanvas->changeViewElements();
        OnResetView(event);
        resetClippingPlanes();
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
            _pAppData->getVolume()->polymeshWrite(dirPath, this);

            Sleep(500);

            return 1;
        }));
    }
}

void MainFrame::OnEditPrefs(wxCommandEvent& event)
{
    EditPrefsDlg dlg(this, "Edit Prefs");
    if (dlg.ShowModal() == wxID_OK) {
        dlg.save();
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

void MainFrame::OnDivideCFDHexes(wxCommandEvent& event)
{
    SplittingParams& params = _pAppData->getParams();
    DivideHexMeshDlg dlg(params, this, 1, wxString("Make Block"), wxPoint(40, 40));
    if (dlg.ShowModal() == wxID_OK) {
        _updateUIEnabled = false;
        _pAppData->doDivideHexMesh(dlg);
        _updateUIEnabled = true;
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

void MainFrame::OnShowEnableSectionGraphics(wxCommandEvent& event)
{
    auto pVol = _pAppData->getVolume();
    _pCanvas->toggleDrawSections(pVol);
}

void MainFrame::OnShowSectionsX(wxCommandEvent& event)
{
    getCanvas()->toggleShowSections(0);

    auto item = _menuBar->FindItem(ID_SHOW_SECTIONS_X);
    if (item)
        item->Check(getCanvas()->showSections(0));
}

void MainFrame::OnShowSectionsY(wxCommandEvent& event)
{
    getCanvas()->toggleShowSections(1);

    auto item = _menuBar->FindItem(ID_SHOW_SECTIONS_Y);
    if (item)
        item->Check(getCanvas()->showSections(1));
}

void MainFrame::OnShowSectionsZ(wxCommandEvent& event)
{
    getCanvas()->toggleShowSections(2);

    auto item = _menuBar->FindItem(ID_SHOW_SECTIONS_Z);
    if (item)
        item->Check(getCanvas()->showSections(2));
}

void MainFrame::OnClippingSingle(wxCommandEvent& event)
{
    getCanvas()->setClippingPlaneEnabled(0, event.IsChecked());
    getCanvas()->setClippingPlaneEnabled(1, false);

    if (event.IsChecked()) {
        auto item = _menuBar->FindItem(ID_SHOW_CLIPPING_DOUBLE);
        if (item)
            item->Check(false);
    }
}

void MainFrame::OnClippingDouble(wxCommandEvent& event)
{
    getCanvas()->setClippingPlaneEnabled(0, event.IsChecked());
    getCanvas()->setClippingPlaneEnabled(1, event.IsChecked());
    if (event.IsChecked()) {
        auto item = _menuBar->FindItem(ID_SHOW_CLIPPING_SINGLE);
        if (item)
            item->Check(false);
    }
}

void MainFrame::OnClippingMove(wxCommandEvent& event)
{
    if (event.IsChecked()) {
        auto item = _menuBar->FindItem(ID_CLIPPING_ROTATE);
        if (item)
            item->Check(false);
        getCanvas()->setClippingRotateEnabled(false);
    }
    getCanvas()->setClippingMoveEnabled(event.IsChecked());
}

void MainFrame::OnClippingRotate(wxCommandEvent& event)
{
    if (event.IsChecked()) {
        auto item = _menuBar->FindItem(ID_CLIPPING_MOVE);
        if (item)
            item->Check(false);
        getCanvas()->setClippingMoveEnabled(false);
    }
    getCanvas()->setClippingRotateEnabled(event.IsChecked());
}

void MainFrame::OnSetViewFront(wxCommandEvent& event)
{
    _pCanvas->setView(M_PI, 0);
}

void MainFrame::OnSetViewBack(wxCommandEvent& event)
{
    _pCanvas->setView(0, 0);
}

void MainFrame::OnSetViewRight(wxCommandEvent& event)
{
    _pCanvas->setView(-M_PI / 2, 0);
}

void MainFrame::OnSetViewLeft(wxCommandEvent& event)
{
    _pCanvas->setView(M_PI / 2, 0);
}

void MainFrame::OnSetViewTop(wxCommandEvent& event)
{
    _pCanvas->setView(0, M_PI / 2);
}

void MainFrame::OnSetViewBottom(wxCommandEvent& event)
{
    _pCanvas->setView(0, -M_PI / 2);
}

void MainFrame::OnResetView(wxCommandEvent& event)
{
    _pCanvas->resetView();
    OnSetViewFront(event);
}

void MainFrame::OnSetModelViewFront(wxCommandEvent& event)
{
    auto& params = _pAppData->getParams();
    _pCanvas->setView(M_PI + params.zRotationDeg * M_PI / 180, -params.yRotationDeg * M_PI / 180);
}

void MainFrame::OnSetModelViewBack(wxCommandEvent& event)
{
    auto& params = _pAppData->getParams();
    _pCanvas->setView(params.zRotationDeg * M_PI / 180, -params.yRotationDeg * M_PI / 180);
}

void MainFrame::OnSetModelViewRight(wxCommandEvent& event)
{
    auto& params = _pAppData->getParams();
    _pCanvas->setView(-M_PI / 2 + params.zRotationDeg * M_PI / 180, -params.yRotationDeg * M_PI / 180);
}

void MainFrame::OnSetModelViewLeft(wxCommandEvent& event)
{
    auto& params = _pAppData->getParams();
    _pCanvas->setView(M_PI / 2 + params.zRotationDeg * M_PI / 180, -params.yRotationDeg * M_PI / 180);
}

void MainFrame::OnSetModelViewTop(wxCommandEvent& event)
{
    auto& params = _pAppData->getParams();
    _pCanvas->setView(M_PI + params.zRotationDeg * M_PI / 180, -M_PI / 2 - params.yRotationDeg * M_PI / 180);
}

void MainFrame::OnSetModelViewBottom(wxCommandEvent& event)
{
    auto& params = _pAppData->getParams();
    _pCanvas->setView(M_PI + params.zRotationDeg * M_PI / 180, M_PI / 2 - params.yRotationDeg * M_PI / 180);
}

void MainFrame::OnResetModelView(wxCommandEvent& event)
{
    auto pVol = _pAppData->getVolume();
    if (pVol) {
        auto& pts = pVol->getModelCornerPts();
        auto pt0 = TRI_LERP(pts, 0.5, 0.0, 0.5);
        auto pt1 = TRI_LERP(pts, 0.5, 1.0, 0.5);
        auto v = (pt1 - pt0).normalized();
        _pCanvas->resetView();
    }
}

void MainFrame::OnToggleMeshInfo(wxCommandEvent& event)
{
    bool startPicking = _pCanvas->toggleMeshSelection();
    if (startPicking) {
        _pAppData->beginMeshFaceInfoPick();
    }
}

void MainFrame::OnToggleMeshDebug(wxCommandEvent& event)
{
    bool startPicking = _pCanvas->toggleMeshSelection();
    if (startPicking)
        _pAppData->beginMeshFaceDebugPick();
}

void MainFrame::OnTestCellSplit(wxCommandEvent& event)
{
    bool startPicking = _pCanvas->toggleMeshSelection();
    if (startPicking)
        _pAppData->testCellSplit();
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
        if (_pBackgroundFuture->wait_for(50ms) == future_status::ready) {
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
                    _pAppData->updateHexTess();
                    _pCanvas->changeViewElements();
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

    if (t - lastTime > 5 && _updateUIEnabled) {
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
