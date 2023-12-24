#pragma once

#include "wx/wxprec.h"


#ifndef WX_PRECOMP
#include "wx/wx.h"
#endif

#include <memory>
#include <triMesh.h>
#include <volume.h>
#include <appData.h>

namespace DFHM {

class Volume;

enum DFHM_MENU_ID
{
    DFHM_LOWEST = wxID_HIGHEST,
    ID_VerifyClosed,
    ID_VerifyNormals,
    ID_AnalyzeGaps,
    ID_FindMinimumGap,
    ID_BuildCFDHexes,
    ID_WRITE_POLYMESH,

    ID_SHOW_SHARP_EDGES,
    ID_SHOW_TRI_NORMALS,
};

class GraphicsCanvas;
class MainFrame;

class MainFrame : public wxFrame
{
public:
    MainFrame(wxWindow* parent,
        wxWindowID id,
        const wxString& title,
        const wxPoint& pos = wxDefaultPosition,
        const wxSize& size = wxDefaultSize,
        long style = wxDEFAULT_FRAME_STYLE,
        const wxString& name = wxASCII_STR(wxFrameNameStr));

    void addMenus();
    void addStatusBar();
    void OnInternalIdle() wxOVERRIDE;

    const GraphicsCanvas* getCanvas() const;
    GraphicsCanvas* getCanvas();

private:
    wxMenuBar* _menuBar = nullptr;
    wxMenu *_editMenu = nullptr,
        *_fileMenu = nullptr;
    AppDataPtr _pAppData;
    GraphicsCanvas* _pCanvas = nullptr;

    void createFileMenu();
    void createEditMenu();
    void createViewMenu();
    void createHelpMenu();

    void OnOpen(wxCommandEvent& event);
    void OnNew(wxCommandEvent& event);
    void OnWritePolymesh(wxCommandEvent& event);
    void OnClose(wxCommandEvent& event);
    void OnExit(wxCommandEvent& event);
    void OnAbout(wxCommandEvent& event);

    void OnCut(wxCommandEvent& event);
    void OnCopy(wxCommandEvent& event);
    void OnPaste(wxCommandEvent& event);
    void OnVerifyClosed(wxCommandEvent& event);
    void OnVerifyNormals(wxCommandEvent& event);
    void OnAnalyzeGaps(wxCommandEvent& event);
    void OnFindMinGap(wxCommandEvent& event);
    void OnBuildCFDHexes(wxCommandEvent& event);

    void OnShowSharpEdges(wxCommandEvent& event);
    void OnShowTriNormals(wxCommandEvent& event);
};

inline const GraphicsCanvas* MainFrame::getCanvas() const
{
    return _pCanvas;
}

inline GraphicsCanvas* MainFrame::getCanvas()
{
    return _pCanvas;
}

}