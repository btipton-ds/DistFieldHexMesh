#pragma once

#include "wx/wxprec.h"


#ifndef WX_PRECOMP
#include "wx/wx.h"
#endif

namespace DFHM {

enum
{
    ID_Hello = 1
};

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
};

}