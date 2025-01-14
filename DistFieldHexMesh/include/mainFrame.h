#pragma once

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

#include "wx/wxprec.h"


#ifndef WX_PRECOMP
#include "wx/wx.h"
#endif

#include <wx/dataview.h>

#include <memory>
#include <triMesh.h>
#include <volume.h>
#include <appData.h>
#include <objectTreeCtrl.h>

namespace DFHM {

/*
TODO 
Add base cube
Add read/write of a document file. This contains all the configuration specific settings. Use OpenFoam dictionary format.
Add subdivision of a triMesh to curvature works better.

Done Fix the annoying graphics!! Get clipping, pan, rotate zoom working right.
Done Support reading multiple mesh files with settings.
*/
    
class Volume;

enum DFHM_MENU_ID
{
    DFHM_LOWEST = wxID_HIGHEST,
    ID_VerifyClosed,
    ID_VerifyNormals,
    ID_AnalyzeGaps,
    ID_FindMinimumGap,
    ID_CREATE_BASE_MESH,
    ID_BuildCFDHexes,
    ID_IMPORT_MESH,
    ID_WRITE_POLYMESH,

    ID_VIEW_FRONT,
    ID_VIEW_BACK,
    ID_VIEW_TOP,
    ID_VIEW_BOTTOM,
    ID_VIEW_LEFT,
    ID_VIEW_RIGHT,
    ID_VIEW_RESET,

    ID_SHOW_FRONT,
    ID_SHOW_BACK,
    ID_SHOW_TOP,
    ID_SHOW_BOTTOM,
    ID_SHOW_LEFT,
    ID_SHOW_RIGHT,
    ID_SHOW_RESET,

    ID_SHOW_CURVATURE,
    ID_SHOW_SHARP_VERTS,
    ID_SHOW_TRI_NORMALS,
    ID_SHOW_MODEL_SHARP_EDGES,
    ID_SHOW_MODEL_EDGES,
    ID_SHOW_MODEL_FACES,

    ID_SHOW_MESH_EDGES,
    ID_SHOW_MESH_FACES,
    ID_SHOW_MESH_BOUNDARY,
    ID_SHOW_MESH_WALL,
    ID_SHOW_MESH_SELECTED_BLOCKS,

    ID_OBJ_TREE_CTRL,
    ID_TREE_CTRL_SHOW,

};

class GraphicsCanvas;
class MainFrame;
class MeshData;
using MeshDataPtr = std::shared_ptr<MeshData>;

class MainFrame : public wxFrame
{
public:
    MainFrame(wxWindow* parent,
        wxWindowID id,
        const wxString& title,
        const wxPoint& pos = wxDefaultPosition,
        const wxSize& size = wxDefaultSize,
        long style = wxDEFAULT_FRAME_STYLE,
        const wxString& name = wxString::FromAscii(wxFrameNameStr));

    virtual ~MainFrame();

    const GraphicsCanvas* getCanvas() const;
    GraphicsCanvas* getCanvas();

    AppDataPtr getAppData();
    void refreshObjectTree();

private:
    void addMenus();
    void addModelPanel();
    void addStatusBar();
    void OnInternalIdle();

    void createFileMenu();
    void createEditMenu();
    void createViewMenu();
    void addViewSubMenu(wxMenu* pParentMenu);
    void addBoundarySubMenu(wxMenu* pParentMenu);
    void createHelpMenu();

    void OnOpen(wxCommandEvent& event);
    void OnImportMesh(wxCommandEvent& event);
    void OnNew(wxCommandEvent& event);
    void OnSave(wxCommandEvent& event);
    void OnSaveAs(wxCommandEvent& event);
    void OnWritePolymesh(wxCommandEvent& event);
    void OnClose(wxCommandEvent& event);
    void OnExit(wxCommandEvent& event);
    void OnAbout(wxCommandEvent& event);

    void OnCut(wxCommandEvent& event);
    void OnCopy(wxCommandEvent& event);
    void OnPaste(wxCommandEvent& event);
    void OnCreateBaseMesh(wxCommandEvent& event);
    void OnBuildCFDHexes(wxCommandEvent& event);

    void OnShowModelSharpEdges(wxCommandEvent& event);
    void OnShowModelSharpVerts(wxCommandEvent& event);
    void OnShowModelTriNormals(wxCommandEvent& event);
    void OnShowModelCurvature(wxCommandEvent& event);
    void OnShowModelFaces(wxCommandEvent& event);
    void OnShowModelEdges(wxCommandEvent& event);

    void OnShowMeshFaces(wxCommandEvent& event);
    void OnShowMeshEdges(wxCommandEvent& event);
    void OnShowMeshWalls(wxCommandEvent& event);
    void OnShowMeshBoundary(wxCommandEvent& event);
    void OnShowMeshSelectedBlocks(wxCommandEvent& event);

    void OnSetViewFront(wxCommandEvent& event);
    void OnSetViewBack(wxCommandEvent& event);
    void OnSetViewRight(wxCommandEvent& event);
    void OnSetViewLeft(wxCommandEvent& event);
    void OnSetViewTop(wxCommandEvent& event);
    void OnSetViewBottom(wxCommandEvent& event);
    void OnResetView(wxCommandEvent& event);

    void OnShowFront(wxCommandEvent& event);
    void OnShowBack(wxCommandEvent& event);
    void OnShowRight(wxCommandEvent& event);
    void OnShowLeft(wxCommandEvent& event);
    void OnShowTop(wxCommandEvent& event);
    void OnShowBottom(wxCommandEvent& event);

    wxMenuBar* _menuBar = nullptr;
    wxMenu* _editMenu = nullptr,
        * _fileMenu = nullptr;
    ObjectTreeCtrl* _pObjectTree;
    AppDataPtr _pAppData;
    GraphicsCanvas* _pCanvas = nullptr;
    wxWithImages::Images _images;
};

inline const GraphicsCanvas* MainFrame::getCanvas() const
{
    return _pCanvas;
}

inline GraphicsCanvas* MainFrame::getCanvas()
{
    return _pCanvas;
}

inline AppDataPtr MainFrame::getAppData()
{
    return _pAppData;
}

}