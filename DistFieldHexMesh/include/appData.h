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

    Copyright Robert R Tipton, 2022, all rights reserved except those granted in prior license statement.

    Dark Sky Innovative Solutions http://darkskyinnovation.com/
*/

#include "wx/wxprec.h"

#ifndef WX_PRECOMP
#include "wx/wx.h"
#endif

#include <defines.h>
#include <string>
#include <memory>

#include <triMesh.h>
#include <OGLMultiVboHandler.h>
#include <MultiCoreUtil.h>
#include <splitParams.h>
#include <model.h>
#include <volume.h>

namespace DFHM {

class GraphicsCanvas;
class MainFrame;
class MakeBlockDlg;
class SelectBlocksDlg;
class DivideHexMeshDlg;
class CreateBaseMeshDlg;
class Index3DId;

class MeshData;
using MeshDataPtr = std::shared_ptr<MeshData>;

class AppData;
using AppDataPtr = std::shared_ptr<AppData>;

using Index3DIdSearchTree = CSpatialSearchBase<double, Index3DId, 25>;

class AppData : public std::enable_shared_from_this<AppData> {
public:
    class MeshPickHandler {
    public:
        virtual bool handle(wxMouseEvent& event, const Rayd& ray, const std::vector<Index3DId>& hits) const = 0;
    };

    AppData(MainFrame* pMainFrame = nullptr);
    virtual ~AppData();
    void preDestroy();

    size_t numBytes() const;

    bool doOpen();
    bool doImportMesh();
    void doSave();
    void doSaveAs();
    void doVerifyClosed(const CMeshPtr& pMesh);
    void doVerifyNormals(const CMeshPtr& pMesh);
    void doAnalyzeGaps(const CMeshPtr& pMesh);
    void doFindMinGap(const CMeshPtr& pMesh) const;
    void doCreateBaseVolume();
    void doRemoveBaseVolume();
    void doDivideHexMesh(const DivideHexMeshDlg& dlg);
    void doNew(const MakeBlockDlg& dlg);
    void doSelectBlocks(const SelectBlocksDlg& dlg);
    void handleMeshRayCast(wxMouseEvent& event, const Rayd& ray) const;

    const std::shared_ptr<MultiCore::ThreadPool>& getThreadPool() const;
        
    VolumePtr getVolume() const;
    MainFrame* getMainFrame();

    CBoundingBox3Dd getBoundingBox() const;
    CBoundingBox3Dd getMeshBoundingBox() const;
    void getDisplayMinMax(Index3D& min, Index3D& max) const;
    void setDisplayMinMax(const Index3D& min, const Index3D& max);

    SplittingParams& getParams();
    const SplittingParams& getParams() const;

    std::wstring getCacheDirName() const;

    const Model& getModel() const;
    MeshDataConstPtr getMeshData(const std::wstring& name) const;
    MeshDataPtr getMeshData(const std::wstring& name);

    void beginMeshFaceInfoPick();
    void beginMeshFaceDebugPick();
    void testCellSplit();

    const std::set<Index3DId>& getSelectedCellIds() const;
    std::set<Index3DId>& getSelectedCellIds();
    std::set<Index3D>& getSelectedBlockIds();
    std::set<Index3D>& getProcessOnlyBlockIds();
    bool getDoQualitySplits() const;

    void loadPrefs();
    bool readPrefsFile(std::string& contents) const;
    void updatePrefsFile() const;
    void updatePrefsFile(const std::string& contents) const;

    void buildHexFaceTables();
    void copyHexFaceTablesToVBOs();
    void updateHexTess();
    void updateModelTess();

private:
    class MeshFaceInfoSelectHandler;
    class MeshFaceDebugSelectHandler;
    class MeshTestSplitSelectHandler;
    friend class MeshFaceInfoSelectHandler;
    friend class MeshFaceDebugSelectHandler;
    friend class MeshTestSplitSelectHandler;

    void clear(bool includeModelData);
    void clearCache();
    void makeBlock(const MakeBlockDlg& dlg);
	void makeCylinderWedge(const MakeBlockDlg& dlg, bool isCylinder);
    void makeModelCubePoints(Vector3d pts[8], CBoundingBox3Dd& volBox);

    void makeOGLTess(const MeshDataPtr& pData, const SplittingParams& params, std::shared_ptr<DrawModelMesh>& pDrawModelMesh);
    void changeViewElements(const MeshDataPtr& pData, std::shared_ptr<DrawModelMesh>& pDraw);

    void initMeshSearchTree();
    bool handleMeshFaceInfoClick(wxMouseEvent& event, const Rayd& ray, const std::vector<Index3DId>& hits) const;
    bool handleMeshFaceDebugClick(wxMouseEvent& event, const Rayd& ray, const std::vector<Index3DId>& hits);
    bool handleMeshTestSplit(wxMouseEvent& event, const Rayd& ray, const std::vector<Index3DId>& hits);
    Index3DId faceCellDisplayed(const Polygon& face) const;

    CMeshPtr readStl(const std::wstring& path, const std::wstring& filename);
    void readDHFM(const std::wstring& path, const std::wstring& filename);
    void writeDHFM() const;

    bool _doQualitySplits = true;
	std::string _workDirName;
    MainFrame* _pMainFrame = nullptr;
    Model _model;

    VolumePtr _pVolume;
    std::shared_ptr<const MeshPickHandler> _pMeshPickHandler;
    std::shared_ptr<Index3DIdSearchTree> _pFaceSearchTree;

    const OGL::IndicesPtr 
        _modelFaceTess, 
        _modelEdgeTess, 
        _modelNormalTess, 
        _sharpPointTess;

    Index3D _minDisplayBlock, _maxDisplayBlock;
    std::set<Index3DId> _selectedCellIds;
    std::set<Index3D> _selectedBlockIds;
    std::set<Index3D> _processOnlyBlocks;

    SplittingParams _params;
    std::wstring _dhfmFilename;

    mutable std::shared_ptr<MultiCore::ThreadPool> _pThreadPool;
};

inline SplittingParams& AppData::getParams()
{
    return _params;
}

inline const SplittingParams& AppData::getParams() const
{
    return _params;
}

inline const Model& AppData::getModel() const
{
    return _model;
}

inline const std::set<Index3DId>& AppData::getSelectedCellIds() const
{
    return _selectedCellIds;
}

inline std::set<Index3DId>& AppData::getSelectedCellIds()
{
    return _selectedCellIds;
}

inline std::set<Index3D>& AppData::getSelectedBlockIds()
{
    return _selectedBlockIds;
}

inline std::set<Index3D>& AppData::getProcessOnlyBlockIds()
{
    return _processOnlyBlocks;
}

inline bool AppData::getDoQualitySplits() const
{
    return _doQualitySplits;
}

inline VolumePtr AppData::getVolume() const
{
    return _pVolume;
}

inline MainFrame* AppData::getMainFrame()
{
    return _pMainFrame;
}

}
