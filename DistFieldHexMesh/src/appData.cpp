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

#include <filesystem>
#include <wx/filedlg.h>
#include <wx/string.h>
#include <wx/wfstream.h>
#include <wx/dataview.h>
#include <wx/dir.h>

#undef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES 1
#include "defines.h"

#include <Eigen/Geometry>
#include <Eigen/src/Core/Matrix.h>

#include <tm_ray.h>
#include <splitParams.h>
#include <appData.h>
#include <tm_math.h>
#include <triMesh.hpp>
#include <tm_spatialSearch.hpp>

#include <readWriteStl.h>
#include <MultiCoreUtil.h>

#include <graphicsCanvas.h>
#include <selectBlocksDlg.h>
#include <createBaseMeshDlg.h>
#include <divideHexMeshDlg.h>

#include <splitParams.h>
#include <meshData.h>
#include <makeBlockDlg.h>
#include <mainFrame.h>
#include <drawModelMesh.h>
#include <drawHexMesh.h>
#include <volume.h>
#include <vertex.h>
#include <utils.h>
#include <gradingOp.h>
#include <splitter3D.h>
#include <debugMeshData.h>

using namespace std;
using namespace DFHM;

namespace
{
    wstring _gBaseVolumeName(L"Bounds");
}

AppData::AppData(MainFrame* pMainFrame)
    : _pMainFrame(pMainFrame)
    , _pDebugMeshData(make_shared<DebugMeshData>())
{
    clearCache();
    loadPrefs();
}

AppData::~AppData()
{
}

const std::shared_ptr<MultiCore::ThreadPool>& AppData::getThreadPool() const
{
    if (!_pThreadPool) {
        // TODO the first entry is thread per job, the second entry is threads avaialable. 
        // Threads per job needs to be set on each call. If the job never calls subthreads, the number threads per job should be the
        // number of cores. If it has sub threads then, that number should be smaller - like numCores/ [4 to 8] - to allow threads for the sub jobs.
        // If the primary jobs are well balanced, then there should be no sub threads at all, because all threads are loaded.
        // This gets tricky and requirs a lot of tuning.
        // The capability is available, it now needs to be used properly.
        int numCores = MultiCore::getNumCores();
        int numThreads = (int)(numCores * 1);
        int numSubThreads = (int)(numCores * 0.75);
        int numAvailable = (int)(numCores * 1);
        _pThreadPool = make_shared< MultiCore::ThreadPool>(numThreads, numSubThreads, numAvailable);
    }
    return _pThreadPool;
}

void AppData::preDestroy()
{
    // Clear these to avoid infinite loop on shared_ptr destruction. Ugly, but gets around hang on exit.
    _pVolume = nullptr;
    _model.clear();
}

size_t AppData::numBytes() const
{
    size_t result = 0;
    if (_pVolume)
        result += _pVolume->numBytes();

    result += _model.numBytes();

    if (_pVolume)
        result += _pVolume->numBytes();

    return result;
}

class AppData::MeshFaceInfoSelectHandler : public AppData::MeshPickHandler
{
public:
    MeshFaceInfoSelectHandler(AppData* pAppData)
        : _pAppData(pAppData)
    {
    }

    bool handle(wxMouseEvent& event, const Rayd& ray, const std::vector<Index3DId>& hits) const override
    {
        if (_pAppData) {
            if (_pAppData->handleMeshFaceInfoClick(event, ray, hits)) {
                return true;
            }
        }
        return false;
    }

private:
    AppData* _pAppData;
};

class AppData::MeshTestConditionalSplitSelectHandler : public AppData::MeshPickHandler
{
public:
    MeshTestConditionalSplitSelectHandler(AppData* pAppData)
        : _pAppData(pAppData)
    {
    }

    bool handle(wxMouseEvent& event, const Rayd& ray, const std::vector<Index3DId>& hits) const override
    {
        if (_pAppData) {
            if (_pAppData->handleMeshConditionalTestSplit(event, ray, hits)) {
                return true;
            }
        }
        return false;
    }

private:
    AppData* _pAppData;

};

class AppData::MeshTestComplexitySplitSelectHandler : public AppData::MeshPickHandler
{
public:
    MeshTestComplexitySplitSelectHandler(AppData* pAppData)
        : _pAppData(pAppData)
    {
    }

    bool handle(wxMouseEvent& event, const Rayd& ray, const std::vector<Index3DId>& hits) const override
    {
        if (_pAppData) {
            if (_pAppData->handleMeshComplexityTestSplit(event, ray, hits)) {
                return true;
            }
        }
        return false;
    }

private:
    AppData* _pAppData;

};

class AppData::MeshFaceDebugSelectHandler : public AppData::MeshPickHandler
{
public:
    MeshFaceDebugSelectHandler(AppData* pAppData) 
        : _pAppData(pAppData)
    {
    }

    bool handle(wxMouseEvent& event, const Rayd& ray, const std::vector<Index3DId>& hits) const override
    {
        if (_pAppData) {
            if (_pAppData->handleMeshFaceDebugClick(event, ray, hits)) {
                return true;
            }
        }
        return false;
    }

private:
    AppData* _pAppData;
};

void AppData::initMeshSearchTree()
{
    if (!_pFaceSearchTree && _pVolume) {
        auto bBox = _pVolume->getVolumeBBox();
        mutex mut;
        _pFaceSearchTree = make_shared<Index3DIdSearchTree>(bBox);

        MultiCore::runLambda([this, &mut](size_t threadNum, size_t numThreads) {
            for (size_t blkIdx = threadNum; blkIdx < _pVolume->numBlocks(); blkIdx += numThreads) {
                auto pBlk = _pVolume->getBlockPtr(blkIdx);
                if (pBlk) {
                    pBlk->iteratePolygonsInOrder([this, &mut](const Index3DId& id, const Polygon& face)->bool {
                        auto bBox = face.getBBox();
                        lock_guard lg(mut);
                        _pFaceSearchTree->add(bBox, id);
                        return true;
                        });
                }
            }
        }, RUN_MULTI_THREAD);
    }
}

bool AppData::handleMeshFaceInfoClick(wxMouseEvent& event, const Rayd& ray, const std::vector<Index3DId>& hits) const
{
    double minDist = DBL_MAX;
    Index3DId hitId;
    for (const auto& id : hits) {
        const auto& face = _pVolume->getPolygon(id);
        RayHitd hit;
        auto cellId = faceCellDisplayed(face);
        if (cellId.isValid() && face.intersect(ray, hit)) {
            if (hit.dist < minDist) {
                minDist = hit.dist;
                hitId = cellId;
            }
        }
    }

    if (hitId.isValid()) {
        const auto& cell = _pVolume->getPolyhedron(hitId);
        const auto& faceIds = cell.getFaceIds();
        cout << "Info for cell " << hitId << "\n";
        cout << "  Layer                  : " << cell.getLayerNum() << "\n";
        cout << "\n";
        cout << "  Intersects Model       : " << cell.intersectsModel() << "\n";
        cout << "  isTooComplex           : " << cell.isTooComplex(_params) << "\n";
        cout << "  isTooNonOrthogoal      : " << cell.isTooNonOrthogoal(_params) << "\n";
        cout << "  hasTooHighCurvature    : " << cell.hasTooHighCurvature(_params) << "\n";
        cout << "  hasTooManyFaces        : " << cell.hasTooManyFaces(_params) << "\n";
        cout << "  hasTooManySplits       : " << cell.hasTooManySplits() << "\n";
        cout << "  needsCurvatureSplit x  : " << cell.needsCurvatureSplit(_params, 0) << "\n";
        cout << "  needsCurvatureSplit y  : " << cell.needsCurvatureSplit(_params, 1) << "\n";
        cout << "  needsCurvatureSplit z  : " << cell.needsCurvatureSplit(_params, 2) << "\n";
        cout << "\n";
        cout << "  HexYZ curvature        : " << cell.calCurvatureHexYZPlane(_params) << "\n";
        cout << "  HexZX curvature        : " << cell.calCurvatureHexZXPlane(_params) << "\n";
        cout << "  HexXY curvature        : " << cell.calCurvatureHexXYPlane(_params) << "\n";

        cout << "\n";
        cout << "  Norm curvature x       : " << cell.getCurvatureByNormalAxis(_params, 0) << "\n";
        cout << "  Norm curvature y       : " << cell.getCurvatureByNormalAxis(_params, 1) << "\n";
        cout << "  Norm curvature z       : " << cell.getCurvatureByNormalAxis(_params, 2) << "\n";

        cout << "  numFaces               : " << faceIds.size() << "\n";
        for (const auto& faceId : faceIds) {
            const auto& face = _pVolume->getPolygon(faceId);
            cout << "  Info for face " << faceId << "\n";
            const auto& vertIds = face.getVertexIds();
            cout << "    num verts/edges      : " << vertIds.size() << "\n";
        }
        cout << "\n";
        cout << "\n";
        return true;
    }
    return false;
}

bool AppData::handleMeshFaceDebugClick(wxMouseEvent& event, const Rayd& ray, const std::vector<Index3DId>& hits)
{
    double minDist = DBL_MAX;
    Index3DId hitFaceId, hitCellId;
    for (const auto& id : hits) {
        const auto& face = _pVolume->getPolygon(id);
        RayHitd hit;
        if (face.intersect(ray, hit)) {
            auto cellId = faceCellDisplayed(face);
            if (hit.dist < minDist && cellId.isValid()) {
                minDist = hit.dist;
                hitFaceId = id;
                hitCellId = cellId;
            }
        }
    }

    if (hitCellId.isValid()) {
        if (event.ShiftDown())
            _selectedCellIds.erase(hitCellId);
        else
            _selectedCellIds.insert(hitCellId);

        updatePrefsFile();
        updateHexTess();
        updateDebugTess();

        return true;
    }
    return false;
}

bool AppData::handleMeshConditionalTestSplit(wxMouseEvent& event, const Rayd& ray, const std::vector<Index3DId>& hits)
{
    // Not testing curvature divs yet
    _params.numCurvatureDivs = 0;

    double minDist = DBL_MAX;
    Index3DId hitFaceId, hitCellId;
    for (const auto& id : hits) {
        const auto& face = _pVolume->getPolygon(id);
        RayHitd hit;
        if (face.intersect(ray, hit)) {
            auto cellId = faceCellDisplayed(face);
            if (hit.dist < minDist && cellId.isValid()) {
                minDist = hit.dist;
                hitFaceId = id;
                hitCellId = cellId;
            }
        }
    }

    if (hitCellId.isValid()) {
        auto& cell = _pVolume->getPolyhedron(hitCellId);

        bool needsCrossSections = _params.numCurvatureDivs > 0;
        if (needsCrossSections) {
            _pVolume->createCrossSections(_params, _pVolume->_splitNum);
        }

        auto pBlock = cell.getBlockPtr();
        Splitter3D sp(pBlock, hitCellId, _pVolume->_splitNum + 1);
        if (sp.splitConditional()) {

            _pVolume->setLayerNums();

            updateHexTess();
            updateDebugTess();

            _pFaceSearchTree = nullptr;
            initMeshSearchTree();
        }
        return true;
    }

    return false;
}

bool AppData::handleMeshComplexityTestSplit(wxMouseEvent& event, const Rayd& ray, const std::vector<Index3DId>& hits)
{
    // Not testing curvature divs yet
    _params.numCurvatureDivs = 0;

    double minDist = DBL_MAX;
    Index3DId hitFaceId, hitCellId;
    for (const auto& id : hits) {
        const auto& face = _pVolume->getPolygon(id);
        RayHitd hit;
        if (face.intersect(ray, hit)) {
            auto cellId = faceCellDisplayed(face);
            if (hit.dist < minDist&& cellId.isValid()) {
                minDist = hit.dist;
                hitFaceId = id;
                hitCellId = cellId;
            }
        }
    }

    if (hitCellId.isValid()) {
        auto& cell = _pVolume->getPolyhedron(hitCellId);

        auto pBlock = cell.getBlockPtr();
        Splitter3D sp(pBlock, hitCellId, _pVolume->_splitNum + 1);
        if (sp.splitComplex()) {

            _pVolume->setLayerNums();

            updateHexTess();
            updateDebugTess();

            _pFaceSearchTree = nullptr;
            initMeshSearchTree();
        }
        return true;
    }

    return false;
}

Index3DId AppData::faceCellDisplayed(const Polygon& face) const
{
    auto pCanvas = _pMainFrame->getCanvas();
    const auto& cellIds = face.getCellIds();
    for (const auto& id : cellIds) {
        const auto& cell = _pVolume->getPolyhedron(id);
        size_t layerNum = cell.getLayerNum();
        if (pCanvas->showLayer(layerNum))
            return id;
    }
    return Index3DId();
}

void AppData::beginMeshFaceInfoPick()
{
    initMeshSearchTree();
    _pMeshPickHandler = make_shared<MeshFaceInfoSelectHandler>(this);
}

void AppData::beginMeshFaceDebugPick()
{
    initMeshSearchTree();
    _pMeshPickHandler = make_shared<MeshFaceDebugSelectHandler>(this);
}

void AppData::testConditionalCellSplit()
{
    initMeshSearchTree();
    _pMeshPickHandler = make_shared<MeshTestConditionalSplitSelectHandler>(this);
}

void AppData::testComplexityCellSplit()
{
    initMeshSearchTree();
    _pMeshPickHandler = make_shared<MeshTestComplexitySplitSelectHandler>(this);
}

void AppData::analyzeGaps()
{
    _model.calculateGaps(_params);
    updateModelTess();
    updateDebugTess();
}

bool AppData::doOpen()
{
    wxFileDialog openFileDialog(_pMainFrame, _("Open DFHM file"), "", "",
        "All (*.dfhm)|*.dfhm|DFHM files (*.dfhm)|*.dfhm", wxFD_OPEN | wxFD_FILE_MUST_EXIST);
    if (openFileDialog.ShowModal() == wxID_CANCEL)
        return false;     // the user canceled

    clear(true);

    // proceed loading the file chosen by the user;
    // this can be done with e.g. wxWidgets input streams:
    _workDirName = openFileDialog.GetDirectory();
    wxString pathStr = openFileDialog.GetPath();
    wstring path(pathStr.ToStdWstring());
    wstring filename(openFileDialog.GetFilename().ToStdWstring());
    auto pos = path.find(filename);
    path = path.replace(pos, filename.length(), L"");
    pos = path.find(L"\\");
    while (pos != wstring::npos) {
        path = path.replace(pos, 1, L"/");
        pos = path.find(L"\\");
    }
    if (filename.find(L".dfhm") != -1) {
        readDHFM(path, filename);
        updateModelTess();
        return true;
    }
    return false;
}

CMeshPtr AppData::readStl(const wstring& pathIn, const wstring& filename)
{
    CMeshPtr pMesh = make_shared<CMesh>();
    CReadWriteSTL reader(pMesh);

    wstring path(pathIn);
    auto pos = path.find(filename);
    path = path.substr(0, pos);
    if (reader.read(path, filename)) {
        return pMesh;
    }

    return nullptr;
}

bool AppData::doImportMesh()
{
    wxFileDialog openFileDialog(_pMainFrame, _("Open Triangle Mesh file"), "", "",
        "All (*.stl)|*.stl", wxFD_OPEN | wxFD_FILE_MUST_EXIST);
    if (openFileDialog.ShowModal() == wxID_CANCEL)
        return false;     // the user changed idea...

    // this can be done with e.g. wxWidgets input streams:
    _workDirName = openFileDialog.GetDirectory();
    wxString pathStr = openFileDialog.GetPath();
    wstring path(pathStr.ToStdWstring());
    wstring filename(openFileDialog.GetFilename().ToStdWstring());
    auto pos = path.find(filename);
    path = path.replace(pos, filename.length(), L"");
    pos = path.find(L"\\");
    while (pos != wstring::npos) {
        path = path.replace(pos, 1, L"/");
        pos = path.find(L"\\");
    }

    auto pCanvas = _pMainFrame->getCanvas();
    if (filename.find(L".stl") != -1) {
        auto pMesh = readStl(path, filename);
        auto pos = filename.find(L".");
        wstring name = filename.replace(pos, filename.size(), L"");
        size_t index = _model.size();
        MeshDataPtr pMeshData = make_shared<MeshData>(shared_from_this(), pMesh, index, name);
        _model.add(pMeshData);

        _model.rebuildSearchTree();

        updateModelTess();
        updateDebugTess();
        return true;
    }

    return false;
}

std::wstring AppData::getCacheDirName() const
{
#ifdef _WIN32
    return L"c:/tmp/";
#else
    return L"c:/tmp/";
#endif // _WIN32

}

void AppData::updateHexTess()
{
    if (_pVolume) {
        buildHexFaceTables();
        copyHexFaceTablesToVBOs();

        if (_pMainFrame->getCanvas()->drawSectionsEnabled() && _pVolume->hasCrossSections()) {
            auto pCanvas = _pMainFrame->getCanvas();
            auto pDraw = pCanvas->getDrawCrossSectionEdges();
            pDraw->buildTables(_params, _pVolume->getCrossSections());
//            pDraw->writeGLObj("D:/DarkSky/Projects/output/objs/allSections.obj");
            pDraw->copyTablesToVBOs();
        }
    }
}

void AppData::updateDebugTess()
{
#if ENABLE_VERTEX_DEBUG_GRAPHICS || ENABLE_GAP_DEBUG_GRAPHICS
    auto pCanvas = _pMainFrame->getCanvas();
    auto pDbgData = getDebugMeshData();

    pDbgData->clear();

#if ENABLE_VERTEX_DEBUG_GRAPHICS
    if (_pVolume) {
        for (const auto& pBlk : _pVolume->_blocks) {
            pBlk->iterateVerticesInOrder([&pDbgData](const Index3DId& vertId, Vertex& vert)->bool {
                if (vert.getTopolgyState() == TOPST_SOLID) {
                    pDbgData->add(vert.getPoint());
                }
                return true;
            });
        }
    }

#endif

#if ENABLE_GAP_DEBUG_GRAPHICS
    if (!_model.empty()) {
        _model.addGapDebugGraphicsData(pDbgData);
    }
#endif

    auto pDrawDbgMesh = pCanvas->getDrawDebugMesh();
    pDrawDbgMesh->createTessellation(*pDbgData);
#endif
}

void AppData::updateModelTess()
{
    auto pCanvas = _pMainFrame->getCanvas();
    auto pDrawModelMesh = pCanvas->getDrawModelMesh();
    for (size_t meshIdx = 0; meshIdx < _model.size(); meshIdx++) {
        auto& pData = _model.getMeshData(meshIdx);
        assert(pData->getId() == meshIdx);
        makeOGLTess(pData, _params, pDrawModelMesh);
    }
}

void AppData::doSave()
{
    if (_dhfmFilename.empty()) {
        doSaveAs();
    } else
        writeDHFM();
}

void AppData::doSaveAs()
{
    wxFileDialog saveFileDialog(_pMainFrame, _("Save DFHM file"), "", "",
        "DFHM files (*.dfhm)|*.dfhm", wxFD_SAVE);
    if (saveFileDialog.ShowModal() == wxID_CANCEL)
        return;     // the user changed idea...

    _dhfmFilename = saveFileDialog.GetPath().ToStdWstring();
    writeDHFM();
}

void AppData::writeDHFM() const
{
    ofstream out(filesystem::path(_dhfmFilename), ios::out | ios::trunc | ios::binary);

    int checksEnabled = 0;
#if ENABLE_FILE_PACKET_SIZE_CHECKS
    checksEnabled = 1;
#endif
    // DO NOT use IoUtil because it will cause infinite recursion
    IoUtil::writeChecksEnabled() = checksEnabled;
    out.write((const char*)&checksEnabled, sizeof(checksEnabled));

    uint8_t version = 0;
    IoUtil::write(out, version);

    _params.write(out);

    size_t numMeshes = _model.size();
    IoUtil::write(out, numMeshes);

    for (const auto& pData : _model) {
        pData->write(out);
    }

    bool hasVolume = _pVolume != nullptr;
    IoUtil::write(out, hasVolume);

    if (hasVolume)
        _pVolume->write(out);    
}

const DebugMeshDataPtr& AppData::getDebugMeshData()
{
    return _pDebugMeshData;
}

const DebugMeshDataConstPtr AppData::getDebugMeshData() const
{
    return _pDebugMeshData;
}


void AppData::readDHFM(const wstring& path, const wstring& filename)
{
    _dhfmFilename = path + filename;

    ifstream in(filesystem::path(_dhfmFilename), ifstream::binary);

    int checksEnabled = 0;
    // DO NOT use IoUtil because it will cause infinite recursion
    in.read((char*)&checksEnabled, sizeof(checksEnabled));
    // read checks are determined by the file state when the file was written.
    IoUtil::readChecksEnabled() = checksEnabled == 1;

    uint8_t version;
    IoUtil::read(in, version);
    _params.read(in);

    size_t numMeshes = _model.size();
    IoUtil::read(in, numMeshes);

    vector<MeshDataPtr> meshes;
    CBoundingBox3Dd bbox;
    auto sharedThis = shared_from_this();
    for (size_t i = 0; i < numMeshes; i++) {
        auto pData = make_shared<MeshData>(sharedThis, i);
        pData->read(in);
        meshes.push_back(pData);
        bbox.merge(pData->getPolyMesh()->getBBox());
    }
    MultiCore::runLambda([&meshes](size_t threadNum, size_t numThreads) {
        for (size_t i = threadNum; i < meshes.size(); i += numThreads) {
            meshes[i]->postReadCreate();
        }
    }, true);

    _model.setBounds(bbox);
    for (auto& pData : meshes) {
        _model.add(pData);
    }

    _model.rebuildSearchTree();

    updateModelTess();

    bool hasVolume;
    IoUtil::read(in, hasVolume);
    if (hasVolume) {
        _pVolume = make_shared<Volume>();
        _pVolume->setAppData(shared_from_this(), getThreadPool());

        _pVolume->read(in);
        _pVolume->createAdHocBlockSearchTree();

        updateHexTess();
    }

    updateDebugTess();
    _pMainFrame->refreshObjectTree();
}

void AppData::doNew(const MakeBlockDlg& dlg)
{
	switch (dlg.getSelection()) {
		default:
			break;
		case MakeBlockDlg::BLOCK_TYPE_BLOCK:
			makeBlock(dlg);
			break;
		case MakeBlockDlg::BLOCK_TYPE_CYLINDER:
			makeCylinderWedge(dlg, true);
			break;
		case MakeBlockDlg::BLOCK_TYPE_WEDGE:
			makeCylinderWedge(dlg, false);
			break;
	}
}

void AppData::doSelectBlocks(const SelectBlocksDlg& dlg)
{

    auto pCanvas = _pMainFrame->getCanvas();
    pCanvas->clearMesh3D();

    _selectedBlockIds.clear();
    auto min = dlg.getMin();
    auto max = dlg.getMax();
    Index3D idx;
    for (idx[0] = min[0]; idx[0] < max[0]; idx[0]++) {
        for (idx[1] = min[1]; idx[1] < max[1]; idx[1]++) {
            for (idx[2] = min[2]; idx[2] < max[2]; idx[2]++) {
                _selectedBlockIds.insert(idx);
            }
        }
    }

    updatePrefsFile();
    updateHexTess();
}

void AppData::handleMeshRayCast(wxMouseEvent& event, const Rayd& ray) const
{
    vector<Index3DId> hits;
    if (_pMeshPickHandler && _pFaceSearchTree && _pFaceSearchTree->biDirRayCast(ray, hits)) {
        _pMeshPickHandler->handle(event, ray, hits);
    }
}

CBoundingBox3Dd AppData::getBoundingBox() const
{
    CBoundingBox3Dd result;
    for (const auto& pData : _model) {
        result.merge(pData->getPolyMesh()->getBBox());
    }

    if (_pVolume)
        result.merge(_pVolume->getModelBBox());

    if (result.empty()) {
        result.merge(Vector3d(-1, -1, -1));
        result.merge(Vector3d(1, 1, 1));
    }

    return result;
}

CBoundingBox3Dd AppData::getMeshBoundingBox() const
{
    CBoundingBox3Dd result;
    for (const auto& pData : _model) {
        if (pData->isActive())
            result.merge(pData->getPolyMesh()->getBBox());
    }

    return result;
}

void AppData::getDisplayMinMax(Index3D& min, Index3D& max) const
{
    min = _minDisplayBlock;
    max = _maxDisplayBlock;
}

void AppData::setDisplayMinMax(const Index3D& min, const Index3D& max)
{
    _minDisplayBlock = min;
    _maxDisplayBlock = max;
}

void AppData::clearCache()
{
    auto dir = getCacheDirName();
    wxArrayString list;
    wxDir::GetAllFiles(dir, &list);
    for (size_t i = 0; i < list.size(); i++) {
        wstring fn(list[i]);
        if (fn.find(L".dfhm_tmp_mesh") != wstring::npos) {
            wxRemoveFile(fn);
        }
    }
}

namespace
{
    void cleanUpStr(string& str) {
        while (!str.empty() && str[0] == ' ') {
            str = str.substr(1, str.length());
        }

        for (size_t i = 0; i < str.size(); i++) {
            str[i] = std::tolower(str[i]);
            if (str[i] == ',')
                str[i] = ' ';
        }
    }
}

void AppData::loadPrefs()
{
    const string qsStr("qualitysplits");
    const string selStartStr("selected start");
    const string selEndStr("selected end");
    const string cellIdStr("cellid");
    const string blockIdStr("blockid");
    const string processOnlyStartStr("onlyprocessblocks start");
    const string processOnlyEndStr("onlyprocessblocks end");

    string filename = "assets/prefs.txt";
    ifstream in(filename);
    if (!in.good())
        return;

    _selectedCellIds.clear();
    _selectedBlockIds.clear();
    _processOnlyBlocks.clear();

    char buf[1024];
    while (!in.eof()) {
        in.getline(buf, 1024);
        string str(buf);
        cleanUpStr(str);

        if (str.find(qsStr) != string::npos) {
            str = str.substr(qsStr.length(), str.length());
            int val = std::stoi(str);
            _doQualitySplits = val != 0;
        } else if (str.find(selStartStr) != string::npos) {
            bool done = false;
            do {
                in.getline(buf, 1024);
                string str2(buf);
                cleanUpStr(str2);
                if (str2.find(selEndStr) != string::npos) {
                    done = true;
                } else if (str2.find(cellIdStr) != string::npos) {
                    str2 = str2.substr(cellIdStr.length(), str2.length());
                    stringstream ss(str2);
                    size_t i, j, k, el;
                    ss >> i >> j >> k >> el;
                    Index3DId cellId(i, j, k, el);
                    _selectedCellIds.insert(cellId);
                } else if (str2.find(blockIdStr) != string::npos) {
                    str2 = str2.substr(blockIdStr.length(), str2.length());
                    stringstream ss(str2);
                    size_t i, j, k;
                    ss >> i >> j >> k;
                    Index3D blockId(i, j, k);
                    _selectedBlockIds.insert(blockId);
                }
            } while (!done);
        } else if (str.find(processOnlyStartStr) != string::npos) {
            bool done = false;
            do {
                in.getline(buf, 1024);
                string str2(buf);
                cleanUpStr(str2);
                if (str2.find(processOnlyEndStr) != string::npos) {
                    done = true;
                } else if (str2.find(blockIdStr) != string::npos) {
                    str2 = str2.substr(blockIdStr.length(), str2.length());
                    stringstream ss(str2);
                    size_t i, j, k;
                    ss >> i >> j >> k;
                    Index3D blockId(i, j, k);
                    _processOnlyBlocks.insert(blockId);
                }
            } while (!done);
        }
    }
}

bool AppData::readPrefsFile(std::string& contents) const
{
    contents.clear();
    std::string filepath = { "assets" };
    bool filepathExists = filesystem::exists(filepath);
    if (!filepathExists) {
        // Create an empty assets directory and prefs.txt file
        updatePrefsFile();
    }


    string filename = "assets/prefs.txt";
    ifstream in(filename);
    if (in.good()) {
        char buf[1024];
        while (!in.eof()) {
            in.getline(buf, 1024);
            string str(buf);
            contents += str + "\n";
        }
        return true;
    }

    return false;
}

void AppData::updatePrefsFile() const
{
    stringstream contents;

    const string qsStr("qualitysplits");
    const string selStartStr("selected start");
    const string selEndStr("selected end");
    const string cellIdStr("cellid");
    const string blockIdStr("blockid");
    const string processOnlyStartStr("onlyprocessblocks start");
    const string processOnlyEndStr("onlyprocessblocks end");

    contents << qsStr << " " << (_doQualitySplits ? 1 : 0) << "\n";

    contents << selStartStr << "\n";
    for (const auto& id : _selectedCellIds) {
        contents << "  " << cellIdStr << " " << (int)id[0] << " " << (int)id[1] << " " << (int)id[2] << " " << id.elementId() << "\n";
    }
    for (const auto& id : _selectedBlockIds) {
        contents << "  " << blockIdStr << " " << (int)id[0] << " " << (int)id[1] << " " << (int)id[2] << "\n";
    }
    contents << selEndStr << "\n";

    contents << processOnlyStartStr << "\n";
    for (const auto& id : _processOnlyBlocks) {
        contents << "  " << (int)id[0] << " " << (int)id[1] << " " << (int)id[2] << "\n";
    }
    contents << processOnlyEndStr << "\n";

    updatePrefsFile(contents.str());
}

void AppData::updatePrefsFile(const std::string& contents) const
{
    std::string filepath = { "assets" };
    bool filepathExists = filesystem::exists(filepath);
    if (!filepathExists) {
        std::filesystem::create_directory("assets");
    }
    string filename = "assets/prefs.txt";
    ofstream out(filename);
    out << contents;
}

void AppData::makeBlock(const MakeBlockDlg& dlg)
{
#if 0
    Volume::setVolDim(Index3D(2, 2, 2));
	Volume vol;
//    vol.setOrigin(dlg.getBlockOrigin());
//    vol.setSpan(dlg.getBlockSpan());
    
    auto pCanvas = _pMainFrame->getCanvas();
    pCanvas->beginFaceTesselation();

    Block::TriMeshGroup blockMeshes;
    Block::glPointsGroup faceEdges;
    vol.addAllBlocks(blockMeshes, faceEdges);
    vector<vector<OGL::IndicesPtr>> faceTesselations;
    for (size_t mode = 0; mode < blockMeshes.size(); mode++) {
        faceTesselations.push_back(vector<OGL::IndicesPtr>());
        for (size_t i = 0; i < blockMeshes[mode].size(); i++) {
            auto pBlockMesh = blockMeshes[mode][i];
            auto pBlockTess = pCanvas->setFaceTessellation(pBlockMesh);
            if (pBlockTess)
                faceTesselations[mode].push_back(pBlockTess);
        }
    }

    pCanvas->endFaceTesselation(faceTesselations);
#endif
}

void AppData::makeCylinderWedge(const MakeBlockDlg& dlg, bool isCylinder)
{
}

void AppData::makeModelCubePoints(Vector3d pts[8], CBoundingBox3Dd& volBox)
{
    auto bbox = getMeshBoundingBox();
    Vector3d ctr = bbox.getMin() + bbox.range() * 0.5;

    Vector3d xAxis(1, 0, 0), yAxis(0, 1, 0), zAxis(0, 0, 1);
    Eigen::Matrix4d trans(GraphicsCanvas::createTranslation(-ctr)), untrans(GraphicsCanvas::createTranslation(ctr));
    Eigen::Matrix4d xRot = GraphicsCanvas::createRotation(xAxis, _params.xRotationDeg * M_PI / 180);
    Eigen::Matrix4d yRot = GraphicsCanvas::createRotation(yAxis, _params.yRotationDeg * M_PI / 180);
    Eigen::Matrix4d zRot = GraphicsCanvas::createRotation(zAxis, _params.zRotationDeg * M_PI / 180);
    Eigen::Matrix4d xform;
    xform.setIdentity();
    xform = trans * xform;
    xform = xRot * xform;
    xform = yRot * xform;
    xform = zRot * xform;

    CBoundingBox3Dd bboxOriented;

    for (const auto& pData : _model) {
        const auto pMesh = pData->getPolyMesh();
        pMesh->iterateVertices([this, &pMesh, &xform, &bboxOriented](const Index3DId& vertId, const Vertex& vert)->bool {
            const Vector3d& pt = vert.getPoint();
            Eigen::Vector4d pt4(pt[0], pt[1], pt[2], 1);

            if (_params.symXAxis && pt4[0] < 0)
                pt4[0] = 0;

            if (_params.symYAxis && pt4[1] < 0)
                pt4[1] = 0;

            if (_params.symZAxis && pt4[2] < 0)
                pt4[2] = 0;

            pt4 = xform * pt4;
            Vector3d ptX(pt4[0], pt4[1], pt4[2]);
            bboxOriented.merge(ptX);

            return true;
        });
    }

    Eigen::Vector4d cubePts4[8];
    auto min = bboxOriented.getMin();
    auto max = bboxOriented.getMax();

    if (!_params.symXAxis)
        min[0] -= _params.baseBoxOffset;
    if (!_params.symYAxis)
        min[1] -= _params.baseBoxOffset;
    if (!_params.symZAxis)
        min[2] -= _params.baseBoxOffset;

    max[0] += _params.baseBoxOffset;
    max[1] += _params.baseBoxOffset;
    max[2] += _params.baseBoxOffset;

    size_t idx = 0;
    cubePts4[idx++] = Eigen::Vector4d(min[0], min[1], min[2], 1);
    cubePts4[idx++] = Eigen::Vector4d(max[0], min[1], min[2], 1);
    cubePts4[idx++] = Eigen::Vector4d(max[0], max[1], min[2], 1);
    cubePts4[idx++] = Eigen::Vector4d(min[0], max[1], min[2], 1);

    cubePts4[idx++] = Eigen::Vector4d(min[0], min[1], max[2], 1); // 1
    cubePts4[idx++] = Eigen::Vector4d(max[0], min[1], max[2], 1); // 2
    cubePts4[idx++] = Eigen::Vector4d(max[0], max[1], max[2], 1); // 3
    cubePts4[idx++] = Eigen::Vector4d(min[0], max[1], max[2], 1); // 4

    xform = xform.inverse();

    for (size_t i = 0; i < 8; i++) {
        Eigen::Vector4d pt4 = xform * cubePts4[i];
        pts[i] = Vector3d(pt4[0], pt4[1], pt4[2]);
    }

    double xLen = 0;
    xLen += pts[1][0] - pts[0][0];
    xLen += pts[2][0] - pts[3][0];
    xLen += pts[5][0] - pts[4][0];
    xLen += pts[6][0] - pts[7][0];
    xLen /= 4.0;

    _params.volDivs[0] = (size_t)(xLen / _params.dims[0] + 0.5);
    if (_params.volDivs[0] < 2)
        _params.volDivs[0] = 2;

    double yLen = 0;
    yLen += pts[2][1] - pts[1][1];
    yLen += pts[3][1] - pts[0][1];
    yLen += pts[6][1] - pts[5][1];
    yLen += pts[7][1] - pts[4][1];
    yLen /= 4.0;

    _params.volDivs[1] = (size_t)(yLen / _params.dims[1] + 0.5);
    if (_params.volDivs[1] < 2)
        _params.volDivs[1] = 2;

    double zLen = 0;
    zLen += pts[4][2] - pts[0][2];
    zLen += pts[5][2] - pts[1][2];
    zLen += pts[6][2] - pts[2][2];
    zLen += pts[7][2] - pts[3][2];
    zLen /= 4.0;

    _params.volDivs[2] = (size_t)(zLen / _params.dims[2] + 0.5);
    if (_params.volDivs[2] < 2)
        _params.volDivs[2] = 2;

    Vector3d cubePts1[8] = {
        Vector3d(_params.xMin, _params.yMin, _params.zMin),
        Vector3d(_params.xMax, _params.yMin, _params.zMin),
        Vector3d(_params.xMax, _params.yMax, _params.zMin),
        Vector3d(_params.xMin, _params.yMax, _params.zMin),
        Vector3d(_params.xMin, _params.yMin, _params.zMax),
        Vector3d(_params.xMax, _params.yMin, _params.zMax),
        Vector3d(_params.xMax, _params.yMax, _params.zMax),
        Vector3d(_params.xMin, _params.yMax, _params.zMax),
    };

    volBox.clear();
    for (int i = 0; i < 8; i++)
        volBox.merge(cubePts1[i]);

    _pVolume->setVolDim(_params.volDivs, true);
}

void AppData::makeOGLTess(const MeshDataPtr& pData, const SplittingParams& params, std::shared_ptr<DrawModelMesh>& pDrawModelMesh)
{
    pDrawModelMesh->createFaceTessellation(_params, pData);
    pDrawModelMesh->createEdgeTessellation(_params, pData);
}

MeshDataConstPtr AppData::getMeshData(const std::wstring& name) const
{
    for (const auto& pData : _model) {
        if (pData->getName() == name)
            return pData;
    }
    return nullptr;
}

MeshDataPtr AppData::getMeshData(const std::wstring& name)
{
    for (const auto& pData : _model) {
        if (pData->getName() == name)
            return pData;
    }
    return nullptr;
}

void AppData::clear(bool includeModelData)
{
    auto pCanvas = _pMainFrame->getCanvas();
    pCanvas->clearMesh3D();

    _pVolume = nullptr;

    clearCache();

    if (includeModelData) {
        pCanvas->clearModel();
        _model.clear();
    }
}

void AppData::doCreateBaseVolume()
{
    clear(false);
    _pVolume = make_shared<Volume>(_params.volDivs);
    _pVolume->setAppData(shared_from_this(), getThreadPool());

    _pVolume->setVolCornerPts(_params.getVolBounds());

    Index3D::setBlockDim(1);

    _pMainFrame->startProgress(14);
    auto pFuture = make_shared<future<int>>(async(std::launch::async, [this]()->int {
        Vector3d cubePts[8];
        CBoundingBox3Dd volBox;
        makeModelCubePoints(cubePts, volBox);

        _pVolume->createBaseVolume(_params, cubePts, volBox, _pMainFrame, RUN_MULTI_THREAD);

        assert(_pVolume->verifyTopology(true));
        //    _pVolume->verifyUniquePoints(RUN_MULTI_THREAD);

        _pVolume->setLayerNums();
        _pMainFrame->reportProgress(1);

        return 2;
    }));
    _pMainFrame->setFuture(pFuture);
}

void AppData::doRemoveBaseVolume()
{
    _pVolume = nullptr;
}

void AppData::doDivideHexMesh(const DivideHexMeshDlg& dlg)
{
    try {
        auto pCanvas = _pMainFrame->getCanvas();
        pCanvas->clearMesh3D();

        dlg.getParams(_params);

#if ENABLE_BACKGROUND_PROCESSING
        size_t numProgSteps = 1 + _model.size() + _params.numSimpleDivs + 3 * _params.numConditionalPasses();
        _pMainFrame->startProgress(numProgSteps);
        auto pFuture = make_shared<future<int>>(async(std::launch::async, [this]()->int {
            _pVolume->divideHexMesh(_model, _params, _pMainFrame, RUN_MULTI_THREAD);

            _pMainFrame->reportProgress(1);

            _pFaceSearchTree = nullptr;
            initMeshSearchTree();

            return 2;
        }));
        _pMainFrame->setFuture(pFuture);
#else
        _pVolume->divideHexMesh(_model, _params, _pMainFrame, RUN_MULTI_THREAD);
        const Index3D min(0, 0, 0);
        const Index3D max(_pVolume->volDim());
        setDisplayMinMax(min, max);
        updateHexTess();
        updateDebugTess();

        _pFaceSearchTree = nullptr;
        initMeshSearchTree();
#endif

    } catch (const std::runtime_error& err) {
        static mutex mut;
        lock_guard lg(mut);
        cout << "Exception: " << err.what() << "\n";
    }
}

void AppData::buildHexFaceTables()
{
    if (_pVolume) {
        const Index3D min(0, 0, 0);
        const Index3D max(_pVolume ? _pVolume->volDim() : Index3D());
        Utils::Timer tmr0(Utils::Timer::TT_analyzeModelMesh);

        auto pCanvas = _pMainFrame->getCanvas();
        pCanvas->buildHexFaceTables(_pVolume, min, max, RUN_MULTI_THREAD);

    }
}

void AppData::copyHexFaceTablesToVBOs()
{
    const Index3D min(0, 0, 0);
    const Index3D max(_pVolume ? _pVolume->volDim() : Index3D());

    auto pCanvas = _pMainFrame->getCanvas();
    pCanvas->copyHexFaceTablesToVBOs();

    setDisplayMinMax(min, max);

    pCanvas->changeViewElements();

}
