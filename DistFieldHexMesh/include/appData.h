#pragma once

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

class AppData {
public:
    using OGLIndices = COglMultiVboHandler::OGLIndices;

    AppData(MainFrame* pMainFrame = nullptr);
    void doOpen();
    void doVerifyClosed();
    void doVerifyNormals();
    void doAnalyzeGaps();
    void doFindMinGap() const;
    void doBuildCFDHexes();
    void doNew(const MakeBlockDlg& dlg);

	inline CMeshPtr getMesh() const
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
    void addFacesToScene(GraphicsCanvas* pCanvas);
    void addEdgesToScene(GraphicsCanvas* pCanvas);
    void getEdgeData(std::vector<float>& normPts, std::vector<unsigned int>& normIndices) const;
    CMeshPtr getSharpVertMesh() const;
    void addPointMarker(CMeshPtr& pMesh, const Vector3d& pt, double radius) const;

	std::string _workDirName;
    MainFrame* _pMainFrame = nullptr;
    CMeshPtr _pMesh;
    VolumePtr _volume;

    std::vector<double> _binSizes;
    std::vector<std::vector<int>> _bins;
};

using AppDataPtr = std::shared_ptr<AppData>;

}
