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
#include <splitParams.h>
#include <volume.h>

namespace DFHM {

class GraphicsCanvas;
class MainFrame;
class MakeBlockDlg;
class SelectBlocksDlg;
class BuildCFDHexesDlg;
class CreateBaseMeshDlg;

class MeshData;
using MeshDataPtr = std::shared_ptr<MeshData>;

class AppData;
using AppDataPtr = std::shared_ptr<AppData>;

class AppData {
public:
    using OGLIndices = COglMultiVboHandler::OGLIndices;

    enum CubeTopolType {
        CTT_BOTTOM,
        CTT_TOP,
        CTT_LEFT,
        CTT_RIGHT,
        CTT_FRONT,
        CTT_BACK,

    };

    AppData(MainFrame* pMainFrame = nullptr);
    virtual ~AppData();

    void doOpen();
    bool doImportMesh();
    void doSave();
    void doSaveAs();
    void doVerifyClosed(const CMeshPtr& pMesh);
    void doVerifyNormals(const CMeshPtr& pMesh);
    void doAnalyzeGaps(const CMeshPtr& pMesh);
    void doFindMinGap(const CMeshPtr& pMesh) const;
    void doCreateBaseVolumePreview();
    void doRemoveBaseVolumePreview();
    void doCreateBaseVolume();
    void doRemoveBaseVolume();
    void doBuildCFDHexes(const BuildCFDHexesDlg& dlg);
    void doNew(const MakeBlockDlg& dlg);
    void doSelectBlocks(const SelectBlocksDlg& dlg);

    inline VolumePtr getVolume() const
    {
        return _pVolume;
    }

    CBoundingBox3Dd getBoundingBox() const;
    CBoundingBox3Dd getMeshBoundingBox() const;
    void getDisplayMinMax(Index3D& min, Index3D& max) const;
    void setDisplayMinMax(const Index3D& min, const Index3D& max);

    BuildCFDParams& getParams();
    const BuildCFDParams& getParams() const;

    const std::map<std::wstring, MeshDataPtr>& getMeshObjects() const;
    bool doesBaseMeshExist() const;

private:
    struct GradingRec {
        Vector3i divs = Vector3i(0, 0, 0);
        Vector3d grading;
        void createVec(int axis, double& scale, double& growFactor) const;
    };

    void makeBlock(const MakeBlockDlg& dlg);
	void makeCylinderWedge(const MakeBlockDlg& dlg, bool isCylinder);
    void updateTessellation(const Index3D& min, const Index3D& max);
    void addFacesToScene(GraphicsCanvas* pCanvas, const Index3D& min, const Index3D& max, bool multiCore);
    void addEdgesToScene(GraphicsCanvas* pCanvas, const Index3D& min, const Index3D& max, bool multiCore);
    void makeCubePoints(Vector3d pts[8], CBoundingBox3Dd& volBox);
    CMeshPtr readStl(const std::wstring& path, const std::wstring& filename);
    template<class L>
    void makeSuround(Vector3d cPts[8], const L& fLambda) const;
    template<class L>
    void makeGradedHexOnFace(Vector3d cPts[8], CubeTopolType dir, const L& fLambda) const;
    template<class L>
    void makeGradedHexOnEdge(Vector3d cPts[8], CubeTopolType dir0, CubeTopolType dir1, const L& fLambda) const;
    template<class L>
    void makeGradedHexOnCorners(Vector3d cPts[8], const L& fLambda) const;
    void readDHFM(const std::wstring& path, const std::wstring& filename);
    void writeDHFM() const;

    size_t _xDivs, _yDivs, _zDivs;
	std::string _workDirName;
    MainFrame* _pMainFrame = nullptr;
    CMeshPtr _pHexMesh;
    TriMesh::CMeshRepoPtr _pModelMeshRepo;
    std::map<std::wstring, MeshDataPtr> _meshData;
    VolumePtr _pVolume;
    const COglMultiVboHandler::OGLIndices* _modelFaceTess = nullptr;
    const COglMultiVboHandler::OGLIndices* _modelEdgeTess = nullptr;
    const COglMultiVboHandler::OGLIndices* _modelNormalTess = nullptr;
    const COglMultiVboHandler::OGLIndices* _sharpPointTess = nullptr;

    Index3D _minDisplayBlock, _maxDisplayBlock;
    std::vector<double> _binSizes;
    std::vector<std::vector<int>> _bins;

    BuildCFDParams _params;
    std::wstring _dhfmFilename;
};

inline BuildCFDParams& AppData::getParams()
{
    return _params;
}

inline const BuildCFDParams& AppData::getParams() const
{
    return _params;
}

inline const std::map<std::wstring, MeshDataPtr>& AppData::getMeshObjects() const
{
    return _meshData;
}


}
