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

#ifndef WX_PRECOMP
#include "wx/wx.h"
#endif

#include <memory>
#include <triMesh.h>
#include <OGLMultiVboHandler.h>
#include <volume.h>

namespace DFHM {

class GraphicsCanvas;
class MainFrame;
class MakeBlockDlg;
class SelectBlocksDlg;
class BuildCFDHexesDlg;

class AppData {
public:
    using OGLIndices = COglMultiVboHandler::OGLIndices;

    AppData(MainFrame* pMainFrame = nullptr);
    void doOpen();
    void doSave();
    void doSaveAs();
    void doVerifyClosed();
    void doVerifyNormals();
    void doAnalyzeGaps();
    void doFindMinGap() const;
    void doBuildCFDHexes(const BuildCFDHexesDlg& dlg);
    void doNew(const MakeBlockDlg& dlg);
    void doSelectBlocks(const SelectBlocksDlg& dlg);

	inline CMeshPtr getMesh() const
    {
        return _pMesh;
    }

    inline VolumePtr getVolume() const
    {
        return _volume;
    }

    void getDisplayMinMax(Index3D& min, Index3D& max) const;
    void setDisplayMinMax(const Index3D& min, const Index3D& max);

private:
	void makeBlock(const MakeBlockDlg& dlg);
	void makeCylinderWedge(const MakeBlockDlg& dlg, bool isCylinder);
    void updateTessellation(const Index3D& min, const Index3D& max);
    void addFacesToScene(GraphicsCanvas* pCanvas, const Index3D& min, const Index3D& max, bool multiCore);
    void addEdgesToScene(GraphicsCanvas* pCanvas, const Index3D& min, const Index3D& max, bool multiCore);
    void getEdgeData(std::vector<float>& normPts, std::vector<unsigned int>& normIndices) const;
    CMeshPtr getSharpVertMesh() const;
    void addPointMarker(CMeshPtr& pMesh, const Vector3d& pt, double radius) const;
    void readStl(const std::wstring& path, const std::wstring& filename);
    void readDHFM(const std::wstring& path, const std::wstring& filename);
    void writeDHFM() const;
    void postReadMesh();

	std::string _workDirName;
    MainFrame* _pMainFrame = nullptr;
    CMeshPtr _pMesh;
    VolumePtr _volume;
    const COglMultiVboHandler::OGLIndices* _modelFaceTess = nullptr;
    const COglMultiVboHandler::OGLIndices* _modelEdgeTess = nullptr;
    const COglMultiVboHandler::OGLIndices* _modelNormalTess = nullptr;
    const COglMultiVboHandler::OGLIndices* _sharpPointTess = nullptr;

    Index3D _minDisplayBlock, _maxDisplayBlock;
    std::vector<double> _binSizes;
    std::vector<std::vector<int>> _bins;

    std::wstring _dhfmFilename;
};

using AppDataPtr = std::shared_ptr<AppData>;

}
