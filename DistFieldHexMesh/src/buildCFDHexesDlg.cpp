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

	Robert R Tipton - Author

	Dark Sky Innovative Solutions http://darkskyinnovation.com/
*/

#include <memory>
#include <buildCFDHexesDlg.h>
#include <wx/string.h>
#include <wx/wfstream.h>
#include <filesystem>
#include <block.h>
#include <Vertex.h>
#include <volume.h>

#ifdef WIN32
#include "windows.h"
#endif // WIN32

using namespace std;
using namespace DFHM;

BuildCFDHexesDlg::BuildCFDHexesDlg(wxWindow* parent, wxWindowID id, const wxString& title, const wxPoint& pos)
	: wxDialog(parent, id, title, pos, wxSize(400, 460), wxDEFAULT_DIALOG_STYLE, wxString("Make Block"))
{
#ifdef WIN32
	int gap = 3;
	int descent = 3;
	int promptWidth = 50;
	int boxWidth = 20;
	int boxHeight = 21;
	int rowHeight = boxHeight + descent;
	int col0 = 8;
	int col1 = col0 + promptWidth + gap;
	int col2 = col1 + promptWidth + gap;
	int col3 = col2 + promptWidth + gap;
	int baseRow = 5;
#else
	int gap = 3;
	int descent = 3;
	int promptWidth = 50;
	int boxWidth = 20;
	int boxHeight = 21;
	int rowHeight = boxHeight + descent;
	int col0 = 8;
	int col1 = col0 + promptWidth + gap;
	int col2 = col1 + promptWidth + gap;
	int col3 = col2 + promptWidth + gap;
	int baseRow = 5;
#endif
}

void BuildCFDHexesDlg::getParams(BuildCFDParams& params) const
{

}
