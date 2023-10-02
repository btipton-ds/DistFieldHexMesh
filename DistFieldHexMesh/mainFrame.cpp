#include "mainFrame.h"

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
    wxMenu* menuFile = new wxMenu;

    menuFile->Append(wxID_OPEN);
    Bind(wxEVT_MENU, &MainFrame::OnOpen, this, wxID_OPEN);

    menuFile->Append(wxID_NEW);
    Bind(wxEVT_MENU, &MainFrame::OnNew, this, wxID_NEW);

    menuFile->Append(wxID_CLOSE);
    Bind(wxEVT_MENU, &MainFrame::OnClose, this, wxID_CLOSE);

    menuFile->AppendSeparator();

    menuFile->Append(wxID_EXIT, "Quit\tCtrl-Q");
    Bind(wxEVT_MENU, &MainFrame::OnExit, this, wxID_EXIT);

    _menuBar->Append(menuFile, "&File");
}

void MainFrame::createEditMenu()
{
    wxMenu* menu = new wxMenu;
    
    menu->Append(wxID_CUT);
    Bind(wxEVT_MENU, &MainFrame::OnCut, this, wxID_CUT);

    menu->Append(wxID_COPY);
    Bind(wxEVT_MENU, &MainFrame::OnCopy, this, wxID_COPY);

    menu->Append(wxID_PASTE);
    Bind(wxEVT_MENU, &MainFrame::OnPaste, this, wxID_PASTE);

    _menuBar->Append(menu, "&Edit");

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
    SetStatusText("Welcome to VulkanQuickStart!");
}

void MainFrame::OnOpen(wxCommandEvent& event)
{

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
