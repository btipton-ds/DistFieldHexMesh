// DistFieldHexMesh.cpp : Defines the entry point for the application.
//

#include <iostream>

#include "wx/wxprec.h"


#ifndef WX_PRECOMP
#include "wx/wx.h"
#endif

#include <triMesh.h>
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
#if 0
    // create the main application window
    wxFrame* frame = new wxFrame;

    // show it
    frame->Show(true);

    // just for Motif
#ifdef __WXMOTIF__
    frame->UpdateInfoText();
#endif
#endif
    // enter the main message loop and run the app
    return true;
}

int main(int numParams, const char** params)
{
	Block::setBlockDim(8);
	CMesh mesh;

	return 0;
}
