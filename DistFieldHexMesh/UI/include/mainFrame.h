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

    Copyright Robert R Tipton, 2022, all rights reserved except those granted in prior license statement.

    Dark Sky Innovative Solutions http://darkskyinnovation.com/
*/
#ifndef _MAIN_FRAME
#define _MAIN_FRAME

#include <defines.h>

#include "wx/wx.h"
#include "wx/dataview.h"
#include "wx/glcanvas.h"

#include <memory>
#include <future>
#include <triMesh.h>
#include <volume.h>
#include <appData.h>
#include <objectTreeCtrl.h>

class wxGLContext;

namespace DFHM {

/*
****************************************************************************************************************************************
TODO
****************************************************************************************************************************************

Don't split edges which are "too short."
Visual cross sectioning. ParaView is abysmally slow because it requires real splitting.
Cross mesh gap analysis using the new search tree.

Memory reporting is far out of line with reported memory allocations.

Add grid relaxation with vertex sliding along geometric surfaces
Add cell cutting
Add cell,face,edge squeezing
Add distance field offsetting for boundary layers
Switch objectPool search to use FastBisectionMap.

Split normals at shared vertices. Same as sharp edge rendering, but this fixes badly interpolated curvatures which are messing up
splitting.

Add read/write of a document file. This contains all the configuration specific settings. Use OpenFoam dictionary format.

*******************************************************************************************
Future - too hard and time consuming for now

Conditional intersection splitting 1 hex into 2 wedges
Splitting of wedges

Restore graphics multisampling for OIT - There's a mutlsampling facility for anti aliasing, but it requires changing all the buffer code.
    I spent a day on it and it was just a big can of worms. Higher priorities right now.

****************************************************************************************************************************************
Already done
****************************************************************************************************************************************

Curvature splitting was messing up because of miscounting partial splits as full splits. This blocked some needed splits.
Looks like some sections are not being found. The rear cross bar is not radially splitting as it should
Curvature calculation needs to by switched to use polygon mesh of models instead of TriMesh. TriMesh produces too many sliver edges which create bad curvatures
Add bi/quad/oct/wedge splitting in addition to oct splitting.
raycasting
Intersect mesh and model
Switch from tri mesh to polymesh
High level of division, with all block ON creates laminar edges at level 5+, but processing only a few blocks goes up to level 10 with no issues. It's not related to numerical precision
within blocks, but might be releated across blocks. THIS WAS CAUSED by not resolving all of the "too many faces" cases BEFORE the bad orthogonality cases. MUST FIX NUM FACES BEFORE ORTHO!!!

Complex cell splitting is broken at high levels. That's was fixed but came back when complexity splitting was separated from conditional splitting.
Need to handle the edge/chord length ratio taking into account the principal axes of the face. Otherwise, one axis is being over divided.
It may be best to split the faces to make them closer to 1:1 aspect ratio, before curvature splitting.
It seems we are dividing high aspect ratio cells that should not be split. This may be done by the quality enforcer, if so that's fine. Turn it off and see if that's the cause.
Complex spliting to support the new states
2D face curvature splitting using model triangles intersecting mesh faces.
Remove individual search trees per mesh and replace with a single search tree for all meshes
    Then redo gap sampling, ignore sharp verts and edges inside solids

Layer counting is still off

Split faces are intermittently not fusing at high division levels and getting false "wall" face reports
Generating polyhedra with more than the maximum number of faces!
    This is improved, but there is a problem splitting partially split cells resulting in bad cells.
Layer counting is INCREDIBLY slow at high levels - rewrite it ASAP.
DDP is getting the alpha of the front layer wrong. Test this by turning on only left, back and front boundaries. When faces over lap, the front transparency is WRONG.
    It was actually a the back layer not blending because blending was disabled - fixed.
Fix crash when splitting long triangles more than once. --- This was just an assert on a particular case where the mesh didn't close correctly.
Fix crash on opening file when file is loaded
Fix failure to split cells with too many split faces.
Added Dual Depth Peeling
Fix cell intersections. They are currently reporting false positives
Double check layer counting
Done Add base cube
Done Fix the annoying graphics!! Get clipping, pan, rotate zoom working right.
Done Support reading multiple mesh files with settings.
Done Add triangle splitting to localize curvatures
*/

class Volume;

enum DFHM_MENU_ID
{
    DFHM_LOWEST = wxID_HIGHEST,
    ID_REMESH,
    ID_ANALYZE_GAPS,
    ID_CREATE_BASE_MESH,
    ID_BuildCFDHexes,
    ID_EDIT_PREFS,
    ID_IMPORT_MESH,
    ID_WRITE_POLYMESH,

    ID_MESH_INFO,
    ID_ADD_TO_MESH_DEBUG,
    ID_TEST_CONDITIONAL_SPLIT_CELL,
    ID_TEST_COMPLEXITY_SPLIT_CELL,

    ID_VIEW_FRONT,
    ID_VIEW_BACK,
    ID_VIEW_TOP,
    ID_VIEW_BOTTOM,
    ID_VIEW_LEFT,
    ID_VIEW_RIGHT,
    ID_VIEW_RESET,

    ID_MODEL_VIEW_FRONT,
    ID_MODEL_VIEW_BACK,
    ID_MODEL_VIEW_TOP,
    ID_MODEL_VIEW_BOTTOM,
    ID_MODEL_VIEW_LEFT,
    ID_MODEL_VIEW_RIGHT,
    ID_MODEL_VIEW_RESET,

    ID_SHOW_ALL_SIDES,
    ID_HIDE_ALL_SIDES,
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
    ID_SHOW_MESH_WALL,
    ID_SHOW_MESH_ALL_BLOCKS,
    ID_SHOW_MESH_SELECTED_BLOCKS,

    ID_SHOW_MESH_LAYERS_OFF,
    ID_SHOW_MESH_LAYER_0,
    ID_SHOW_MESH_LAYER_1,
    ID_SHOW_MESH_LAYER_2,
    ID_SHOW_MESH_LAYER_3,
    ID_SHOW_MESH_LAYER_4,

    ID_OBJ_TREE_CTRL,
    ID_TREE_CTRL_SHOW,

    ID_ENABLE_CROSSSECTION_GRAPHICS,
    ID_SHOW_SECTIONS_X,
    ID_SHOW_SECTIONS_Y,
    ID_SHOW_SECTIONS_Z,

    ID_SHOW_CLIPPING_SINGLE,
    ID_SHOW_CLIPPING_DOUBLE,
    ID_CLIPPING_MOVE,
    ID_CLIPPING_ROTATE,

    ID_QUERY_PROGRESS,
#if INCLUDE_DEBUG_WX_FRAME
    ID_TOGGLE_DEBUG_FRAME,
#endif
};

class GraphicsCanvas;
#if INCLUDE_DEBUG_WX_FRAME
class GraphicsDebugCanvas;
#endif
class MainFrame;
class MeshData;
using MeshDataPtr = std::shared_ptr<MeshData>;

class MainFrame : public wxFrame, public ProgressReporter
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

    void doPaint(wxPaintEvent& event);

    AppDataPtr getAppData();
    void refreshObjectTree();
    void reportProgressInner(double fraction) override;
    void setFuture(std::shared_ptr<std::future<int>>& pFuture);

private:
    void addMenus();
    void addStatusBar();
    void OnInternalIdle() override;

    void createFileMenu();
    void createEditMenu();
    void createDebugMenu();
    void createViewMenu();
    void addStandardViewsSubMenu(wxMenu* pParentMenu);
    void addModelViewsSubMenu(wxMenu* pParentMenu);
    void addBoundarySubMenu(wxMenu* pParentMenu);
    void addLayersMenu(wxMenu* pParentMenu);
    void createHelpMenu();

    void OnOpen(wxCommandEvent& event);
    void OnAnalyzeGaps(wxCommandEvent& event);
    void OnImportMesh(wxCommandEvent& event);
    void OnNew(wxCommandEvent& event);
    void OnSave(wxCommandEvent& event);
    void OnSaveAs(wxCommandEvent& event);
    void OnWritePolymesh(wxCommandEvent& event);

    void OnEditPrefs(wxCommandEvent& event);

    void OnClose(wxCommandEvent& event);
    void OnExit(wxCommandEvent& event);
    void OnAbout(wxCommandEvent& event);
#if INCLUDE_DEBUG_WX_FRAME
    void OnToggleDebugFrame(wxCommandEvent& event);
#endif

    void OnCut(wxCommandEvent& event);
    void OnCopy(wxCommandEvent& event);
    void OnPaste(wxCommandEvent& event);
    void OnCreateBaseMesh(wxCommandEvent& event);
    void OnDivideCFDHexes(wxCommandEvent& event);

    void OnShowModelSharpEdges(wxCommandEvent& event);
    void OnShowModelSharpVerts(wxCommandEvent& event);
    void OnShowModelTriNormals(wxCommandEvent& event);
    void OnShowModelCurvature(wxCommandEvent& event);
    void OnShowModelFaces(wxCommandEvent& event);
    void OnShowModelEdges(wxCommandEvent& event);

    void OnShowMeshFaces(wxCommandEvent& event);
    void OnShowMeshEdges(wxCommandEvent& event);
    void OnShowMeshWalls(wxCommandEvent& event);
    void OnShowAllBlocks(wxCommandEvent& event);
    void OnShowMeshSelectedBlocks(wxCommandEvent& event);

    void OnShowEnableSectionGraphics(wxCommandEvent& event);
    void OnShowSectionsX(wxCommandEvent& event);
    void OnShowSectionsY(wxCommandEvent& event);
    void OnShowSectionsZ(wxCommandEvent& event);

    void OnClippingSingle(wxCommandEvent& event);
    void OnClippingDouble(wxCommandEvent& event);
    void OnClippingMove(wxCommandEvent& event);
    void OnClippingRotate(wxCommandEvent& event);

    void OnSetViewFront(wxCommandEvent& event);
    void OnSetViewBack(wxCommandEvent& event);
    void OnSetViewRight(wxCommandEvent& event);
    void OnSetViewLeft(wxCommandEvent& event);
    void OnSetViewTop(wxCommandEvent& event);
    void OnSetViewBottom(wxCommandEvent& event);
    void OnResetView(wxCommandEvent& event);

    void OnSetModelViewFront(wxCommandEvent& event);
    void OnSetModelViewBack(wxCommandEvent& event);
    void OnSetModelViewRight(wxCommandEvent& event);
    void OnSetModelViewLeft(wxCommandEvent& event);
    void OnSetModelViewTop(wxCommandEvent& event);
    void OnSetModelViewBottom(wxCommandEvent& event);
    void OnResetModelView(wxCommandEvent& event);

    void OnToggleMeshInfo(wxCommandEvent& event);
    void OnToggleMeshDebug(wxCommandEvent& event);
    void OnTestCellConditionalSplit(wxCommandEvent& event);
    void OnTestCellComplexitySplit(wxCommandEvent& event);

    void OnShowAllSides(wxCommandEvent& event);
    void OnHideAllSides(wxCommandEvent& event);

    void OnShowFront(wxCommandEvent& event);
    void OnShowBack(wxCommandEvent& event);
    void OnShowRight(wxCommandEvent& event);
    void OnShowLeft(wxCommandEvent& event);
    void OnShowTop(wxCommandEvent& event);
    void OnShowBottom(wxCommandEvent& event);

    void OnShowLayersOff(wxCommandEvent& event);
    void OnShowLayer0(wxCommandEvent& event);
    void OnShowLayer1(wxCommandEvent& event);
    void OnShowLayer2(wxCommandEvent& event);
    void OnShowLayer3(wxCommandEvent& event);
    void OnShowLayer4(wxCommandEvent& event);
    void OnShowLayer5(wxCommandEvent& event);
    void OnShowLayer6(wxCommandEvent& event);
    void OnShowLayer7(wxCommandEvent& event);
    void OnShowLayer8(wxCommandEvent& event);
    void OnShowLayer9(wxCommandEvent& event);

    void OnUpdateUI(wxUpdateUIEvent& event);

    void onSizeChange(wxSizeEvent& event);
    void checkItem(int itemId, bool val);
    void enableItem(int itemId, bool val);

    void resetClippingPlanes();
    void updateStatusBar();

    bool _updateUIEnabled = true;
    std::shared_ptr<std::future<int>> _pBackgroundFuture;
    wxStatusBar* _statusBar;
    int _progressValue = 0;
    wxGauge* _progress;
    wxMenuBar* _menuBar = nullptr;
    wxSizer* _pSizer = nullptr;
    int _debugFrameIndex = -1;
    wxSizerItem 
        *_pObjTreeSizer = nullptr, 
        *_pCanvasSizer = nullptr, 
        *_pDebugCanvasSizer = nullptr;
    wxMenu
        * _editMenu = nullptr,
        * _fileMenu = nullptr,
        * _viewMenu = nullptr,
        * _debugMenu = nullptr;

    size_t _numCells = 0;
    size_t _numFaces = 0;
    size_t _numBytes = 0;
    Index3D _dim;

    ObjectTreeCtrl* _pObjectTree;
    AppDataPtr _pAppData;
    GraphicsCanvas* _pCanvas = nullptr;
#if INCLUDE_DEBUG_WX_FRAME
    GraphicsDebugCanvas* _pDebugCanvas = nullptr;
#endif
    wxWithImages::Images _images;

protected:
    wxDECLARE_EVENT_TABLE();
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

inline void MainFrame::setFuture(std::shared_ptr<std::future<int>>& pFuture)
{
    _pBackgroundFuture = pFuture;
}

}

#endif
