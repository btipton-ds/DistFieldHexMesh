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

class AppData {
public:
    AppData(MainFrame* pMainFrame = nullptr);
    void doOpen();
    void doVerifyClosed();
    void doVerifyNormals();
    void doAnalyzeGaps();
    void doFindMinGap() const;
    void doBuildCFDHexes();

    inline TriMesh::CMeshPtr getMesh() const
    {
        return _pMesh;
    }

    inline VolumePtr getVolume() const
    {
        return _volume;
    }

private:
    std::string _workDirName;
    MainFrame* _pMainFrame = nullptr;
    TriMesh::CMeshPtr _pMesh;
    VolumePtr _volume;

    std::vector<double> _binSizes;
    std::vector<std::vector<int>> _bins;
};

using AppDataPtr = std::shared_ptr<AppData>;

}
