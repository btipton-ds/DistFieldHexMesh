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

#ifndef _APP_DATA
#define _APP_DATA

#include "wx/wx.h"

#include <defines.h>
#include <string>
#include <memory>

#include <OGLMultiVboHandler.h>
#include <MultiCoreUtil.h>
#include <splitParams.h>
#include <model.h>
#include <volume.h>
#include <appDataIntf.h>

namespace DFHM {

class PolyMeshIntf;
class AppData : public AppDataIntf, public std::enable_shared_from_this<AppData> {
public:
    class MeshPickHandler {
    public:
        virtual bool handle(wxMouseEvent& event, const Rayd& ray, const std::vector<Index3DId>& hits) const = 0;
    };

    AppData(MainFrame* pMainFrame = nullptr);
    virtual ~AppData();
    void preDestroy() override;

    size_t numBytes() const override;

    bool doOpen() override;
    bool doImportMesh() override;
    void doSave() override;
    void doSaveAs() override;
    void doCreateBaseVolume() override;
    void doRemoveBaseVolume() override;
    void doDivideHexMesh(const DivideHexMeshDlg& dlg) override;
    void doNew(const MakeBlockDlg& dlg) override;
    void doSelectBlocks(const SelectBlocksDlg& dlg) override;
    void handleMeshRayCast(wxMouseEvent& event, const Rayd& ray) const override;

    const std::shared_ptr<MultiCore::ThreadPool>& getThreadPool() const override;
        
    VolumePtr getVolume() const override;
    MainFrame* getMainFrame() override;

    CBoundingBox3Dd getBoundingBox() const override;
    CBoundingBox3Dd getMeshBoundingBox() const override;
    void getDisplayMinMax(Index3D& min, Index3D& max) const override;
    void setDisplayMinMax(const Index3D& min, const Index3D& max) override;

    SplittingParams& getParams() override;
    const SplittingParams& getParams() const override;

    std::wstring getCacheDirName() const override;

    const Model& getModel() const override;
    Model& getModel() override;
    MeshDataConstPtr getMeshData(const std::wstring& name) const override;
    MeshDataPtr getMeshData(const std::wstring& name) override;
    const DebugMeshDataPtr& getDebugMeshData() override;
    const DebugMeshDataConstPtr getDebugMeshData() const override;

    void beginMeshFaceInfoPick() override;
    void beginMeshFaceDebugPick() override;
    void testConditionalCellSplit() override;
    void testComplexityCellSplit() override;

    const std::set<Index3DId>& getSelectedCellIds() const override;
    std::set<Index3DId>& getSelectedCellIds() override;
    std::set<Index3D>& getSelectedBlockIds() override;
    std::set<Index3D>& getProcessOnlyBlockIds() override;
    bool getDoQualitySplits() const override;

    void loadPrefs() override;
    bool readPrefsFile(std::string& contents) const override;
    void updatePrefsFile() const override;
    void updatePrefsFile(const std::string& contents) const override;

    void buildHexFaceTables() override;
    void copyHexFaceTablesToVBOs() override;
    void updateHexTess() override;
    void updateDebugTess() override;
    void updateModelTess() override;

private:
    class MeshFaceInfoSelectHandler;
    class MeshFaceDebugSelectHandler;
    class MeshTestConditionalSplitSelectHandler;
    class MeshTestComplexitySplitSelectHandler;
    friend class MeshFaceInfoSelectHandler;
    friend class MeshFaceDebugSelectHandler;
    friend class MeshTestConditionalSplitSelectHandler;
    friend class MeshTestComplexitySplitSelectHandler;

    void clear(bool includeModelData);
    void clearCache();
    void makeBlock(const MakeBlockDlg& dlg);
	void makeCylinderWedge(const MakeBlockDlg& dlg, bool isCylinder);
    void makeModelCubePoints(Vector3d pts[8], CBoundingBox3Dd& volBox);

    void makeOGLTess(const MeshDataPtr& pData, const SplittingParams& params, std::shared_ptr<DrawModelMesh>& pDrawModelMesh);

    void initMeshSearchTree();
    bool handleMeshFaceInfoClick(wxMouseEvent& event, const Rayd& ray, const std::vector<Index3DId>& hits) const;
    bool handleMeshFaceDebugClick(wxMouseEvent& event, const Rayd& ray, const std::vector<Index3DId>& hits);
    bool handleMeshConditionalTestSplit(wxMouseEvent& event, const Rayd& ray, const std::vector<Index3DId>& hits);
    bool handleMeshComplexityTestSplit(wxMouseEvent& event, const Rayd& ray, const std::vector<Index3DId>& hits);
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
    DebugMeshDataPtr _pDebugMeshData;

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

inline Model& AppData::getModel()
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

#endif // !_APP_DATA
