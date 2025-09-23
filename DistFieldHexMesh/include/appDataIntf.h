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

#include <defines.h>
#include <string>
#include <triMesh.h>
#include <MultiCoreUtil.h>
#include <index3D.h>

class wxMouseEvent;

namespace DFHM {

    struct SplittingParams;

    class GraphicsCanvas;
    class MainFrame;
    class MakeBlockDlg;
    class SelectBlocksDlg;
    class DivideHexMeshDlg;
    class CreateBaseMeshDlg;
    class Index3DId;

    class MeshData;
    using MeshDataPtr = std::shared_ptr<MeshData>;
    using MeshDataConstPtr = std::shared_ptr<const MeshData>;

    class Volume;
    using VolumePtr = std::shared_ptr<Volume>;

    class Model;

    class AppDataIntf;
    using AppDataPtr = std::shared_ptr<AppDataIntf>;

    using Index3DIdSearchTree = CSpatialSearchBase<double, Index3DId, 25>;

    class AppDataIntf {
    public:
        virtual ~AppDataIntf() {};
        virtual void preDestroy() = 0;

        virtual size_t numBytes() const = 0;

        virtual bool doOpen() = 0;
        virtual bool doImportMesh() = 0;
        virtual void doSave() = 0;
        virtual void doSaveAs() = 0;
        virtual void doVerifyClosed(const TriMesh::CMeshPtr& pMesh) = 0;
        virtual void doVerifyNormals(const TriMesh::CMeshPtr& pMesh) = 0;
        virtual void doAnalyzeGaps(const TriMesh::CMeshPtr& pMesh) = 0;
        virtual void doFindMinGap(const TriMesh::CMeshPtr& pMesh) const = 0;
        virtual void doCreateBaseVolume() = 0;
        virtual void doRemoveBaseVolume() = 0;
        virtual void doDivideHexMesh(const DivideHexMeshDlg& dlg) = 0;
        virtual void doNew(const MakeBlockDlg& dlg) = 0;
        virtual void doSelectBlocks(const SelectBlocksDlg& dlg) = 0;
        virtual void handleMeshRayCast(wxMouseEvent& event, const Rayd& ray) const = 0;

        virtual const std::shared_ptr<MultiCore::ThreadPool>& getThreadPool() const = 0;

        virtual VolumePtr getVolume() const = 0;
        virtual MainFrame* getMainFrame() = 0;

        virtual CBoundingBox3Dd getBoundingBox() const = 0;
        virtual CBoundingBox3Dd getMeshBoundingBox() const = 0;
        virtual void getDisplayMinMax(Index3D& min, Index3D& max) const = 0;
        virtual void setDisplayMinMax(const Index3D& min, const Index3D& max) = 0;

        virtual SplittingParams& getParams() = 0;
        virtual const SplittingParams& getParams() const = 0;

        virtual std::wstring getCacheDirName() const = 0;

        virtual const Model& getModel() const = 0;
        virtual Model& getModel() = 0;
        virtual MeshDataConstPtr getMeshData(const std::wstring& name) const = 0;
        virtual MeshDataPtr getMeshData(const std::wstring& name) = 0;

        virtual void beginMeshFaceInfoPick() = 0;
        virtual void beginMeshFaceDebugPick() = 0;
        virtual void testConditionalCellSplit() = 0;
        virtual void testComplexityCellSplit() = 0;

        virtual const std::set<Index3DId>& getSelectedCellIds() const = 0;
        virtual std::set<Index3DId>& getSelectedCellIds() = 0;
        virtual std::set<Index3D>& getSelectedBlockIds() = 0;
        virtual std::set<Index3D>& getProcessOnlyBlockIds() = 0;
        virtual bool getDoQualitySplits() const = 0;

        virtual void loadPrefs() = 0;
        virtual bool readPrefsFile(std::string& contents) const = 0;
        virtual void updatePrefsFile() const = 0;
        virtual void updatePrefsFile(const std::string& contents) const = 0;

        virtual void buildHexFaceTables() = 0;
        virtual void copyHexFaceTablesToVBOs() = 0;
        virtual void updateHexTess() = 0;
        virtual void updateDebugTess() = 0;
        virtual void updateModelTess() = 0;

    };
}