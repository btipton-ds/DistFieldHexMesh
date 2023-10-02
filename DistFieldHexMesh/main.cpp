// DistFieldHexMesh.cpp : Defines the entry point for the application.
//

#include <iostream>

#include "wx/wxprec.h"


#ifndef WX_PRECOMP
#include "wx/wx.h"
#endif

#include <triMesh.h>
#include "mainFrame.h"
#include "volume.h"

using namespace std;
using namespace DFHM;

class DFHMApp : public wxApp
{
public:
	virtual bool OnInit() wxOVERRIDE;
};


// Create a new application object
wxIMPLEMENT_APP(DFHMApp);

// 'Main program' equivalent: the program execution "starts" here
bool DFHMApp::OnInit()
{
    wxApp::OnInit();

    MainFrame* frame = new MainFrame(nullptr, 0, L"DistFieldHexMesh", wxPoint(50, 50), wxSize(1000, 1000));
    frame->Show(true);

    // just for Motif
#ifdef __WXMOTIF__
    frame->UpdateInfoText();
#endif

    // enter the main message loop and run the app
    return true;
}
#if 0
int main(int numParams, const char** params)
{
	Block::setBlockDim(8);
	CMesh mesh;

	return 0;
}
#endif

