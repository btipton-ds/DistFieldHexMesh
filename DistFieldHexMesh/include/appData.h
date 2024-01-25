#pragma once

#include "wx/wxprec.h"


#ifndef WX_PRECOMP
#include "wx/wx.h"
#endif

#include <memory>
#include <triMesh.h>
#include <volume.h>

namespace DFHM {

class GraphicsCanvas;
class MainFrame;
class MakeBlockDlg;

class AppData {
public:
    AppData(MainFrame* pMainFrame = nullptr);
    void doOpen();
    void doVerifyClosed();
    void doVerifyNormals();
    void doAnalyzeGaps();
    void doFindMinGap() const;
    void doBuildCFDHexes();
    void doNew(const MakeBlockDlg& dlg);

	inline TriMesh::CMeshPtr getMesh() const
    {
        return _pMesh;
    }

    inline VolumePtr getVolume() const
    {
        return _volume;
    }

private:
	void makeBlock(const MakeBlockDlg& dlg);
	void makeCylinderWedge(const MakeBlockDlg& dlg, bool isCylinder);
    void addTriangles(GraphicsCanvas* pCanvas, bool outerFacesOnly, size_t minSplitNum);
    void addFaceEdges(GraphicsCanvas* pCanvas, bool outerFacesOnly, size_t minSplitNum);

	std::string _workDirName;
    MainFrame* _pMainFrame = nullptr;
    TriMesh::CMeshPtr _pMesh;
    VolumePtr _volume;

    std::vector<double> _binSizes;
    std::vector<std::vector<int>> _bins;
};

using AppDataPtr = std::shared_ptr<AppData>;

}
