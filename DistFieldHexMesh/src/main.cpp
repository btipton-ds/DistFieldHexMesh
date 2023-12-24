// DistFieldHexMesh.cpp : Defines the entry point for the application.
//

#include <iostream>

#include "wx/wxprec.h"

#ifndef WX_PRECOMP
#include "wx/wx.h"
#endif

#include <triMesh.h>
#include <volume.h>

#include <app.h>
#include <mainFrame.h>
#include <vertex.h>

using namespace std;
using namespace DFHM;



// Create a new application object
wxIMPLEMENT_APP(App);

// 'Main program' equivalent: the program execution "starts" here

#if 0
int main(int numParams, const char** params)
{
	Block::setBlockDim(8);
	CMesh mesh;

	return 0;
}
#endif

