#pragma once

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

#include "wx/wxprec.h"
#include <tm_vector3.h>

#ifndef WX_PRECOMP
#include "wx/wx.h"
#include <wx/dialog.h>
#endif

namespace DFHM {
	struct BuildCFDParams;

	class BuildCFDHexesDlg : public wxDialog {
	public:

		BuildCFDHexesDlg(BuildCFDParams& params, wxWindow* parent, wxWindowID id, const wxString& title,
			const wxPoint& pos = wxDefaultPosition);

		void getParams(BuildCFDParams& params) const;

	private:
		void getValue(wxTextCtrl* item, size_t& curValue) const;
		void getValue(wxTextCtrl* item, double& curValue) const;

		wxStaticText
			* _minBlocksPerSidePrompt = nullptr,
			* _numBlockDivsPrompt = nullptr,
			* _numSimpleDivsPrompt = nullptr,
			* _numIntersectionDivsPrompt = nullptr,
			* _numSharpEdgeIntersectionDivsPrompt = nullptr,
			* _numCurvatureDivsPrompt = nullptr,
			* _divsPerCurvaturePrompt = nullptr,
			* _divsPerGapCurvaturePrompt = nullptr,
			* _maxCellFacesPrompt = nullptr,
			* _maxGapSizePrompt = nullptr,
			* _maxCurvatureRadiusPrompt = nullptr,
			* _sharpAnglePrompt = nullptr,
			* _minSplitEdgeLengthCurvaturePrompt = nullptr,
			* _minSplitEdgeLengthGapCurvaturePrompt = nullptr;

		wxCheckBox
			* _uniformRatioCheckBox = nullptr,
			* _splitSharpVertsCheckBox = nullptr;

		wxTextCtrl
			* _minBlocksPerSideBox = nullptr,
			* _numBlockDivsBox = nullptr,
			* _numSimpleDivsBox = nullptr,
			* _numIntersectionDivsBox = nullptr,
			* _numSharpEdgeIntersectionDivs = nullptr,
			* _numCurvatureDivsBox = nullptr,
			* _divsPerCurvatureBox = nullptr,
			* _divsPerGapCurvatureBox = nullptr,
			* _maxCellFacesBox = nullptr,
			* _maxGapSizeBox = nullptr,
			* _maxCurvatureRadiusBox = nullptr,
			* _sharpAngleBox = nullptr,
			* _minSplitEdgeLengthCurvatureBox = nullptr,
			* _minSplitEdgeLengthGapCurvatureBox = nullptr;

		wxButton
			* _okButton,
			* _cancelButton;

	};


}
