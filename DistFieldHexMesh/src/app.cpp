#include <app.h>
#include <mainFrame.h>

using namespace DFHM;

bool App::OnInit()
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
