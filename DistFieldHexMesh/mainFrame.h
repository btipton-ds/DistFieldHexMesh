#pragma once

#include "wx/wxprec.h"


#ifndef WX_PRECOMP
#include "wx/wx.h"
#endif

#include <memory>
#include <triMesh.h>

namespace DFHM {

enum DFHM_MENU_ID
{
    DFHM_LOWEST = wxID_HIGHEST,
    ID_VerifyClosed,
    ID_VerifyNormals,
    ID_AnalyzeGaps,
};

class MainFrame;

class AppData {
public:
    AppData(MainFrame* pMainFrame);
    void doOpen();
    void doVerifyClosed();
    void doVerifyNormals();
    void doAnalyzeGaps();

private:
    MainFrame* _pMainFrame = nullptr;
    TriMesh::CMeshPtr _pMesh;

    std::vector<double> _binSizes;
    std::vector<std::vector<int>> _bins;
};
using AppDataPtr = std::shared_ptr<AppData>;

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

private:
    wxMenuBar* _menuBar = nullptr;
    AppDataPtr _pAppData;

    void createFileMenu();
    void createEditMenu();
    void createHelpMenu();

    void OnOpen(wxCommandEvent& event);
    void OnNew(wxCommandEvent& event);
    void OnClose(wxCommandEvent& event);
    void OnExit(wxCommandEvent& event);
    void OnAbout(wxCommandEvent& event);

    void OnCut(wxCommandEvent& event);
    void OnCopy(wxCommandEvent& event);
    void OnPaste(wxCommandEvent& event);
    void OnVerifyClosed(wxCommandEvent& event);
    void OnVerifyNormals(wxCommandEvent& event);
    void OnAnalyzeGaps(wxCommandEvent& event);
};

}