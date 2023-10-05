#pragma once

#include "wx/wxprec.h"


#ifndef WX_PRECOMP
#include "wx/wx.h"
#endif

namespace DFHM {

class App : public wxApp
{
public:
	bool OnInit() wxOVERRIDE;
};

}
