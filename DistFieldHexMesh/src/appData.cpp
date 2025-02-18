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

#include <filesystem>
#include <wx/filedlg.h>
#include <wx/string.h>
#include <wx/wfstream.h>
#include <wx/dataview.h>
#include <wx/dir.h>

#include "defines.h"

#include <Eigen/Geometry>
#include <Eigen/src/Core/Matrix.h>

#include <tm_ray.h>
#include <splitParams.h>
#include <appData.h>
#include <tm_math.h>
#include <triMesh.h>
#include <readWriteStl.h>
#include <MultiCoreUtil.h>
#include <selectBlocksDlg.h>
#include <createBaseMeshDlg.h>
#include <buildCFDHexesDlg.h>

#include <splitParams.h>
#include <meshData.h>
#include <makeBlockDlg.h>
#include <mainFrame.h>
#include <drawModelMesh.h>
#include <drawHexMesh.h>
#include <graphicsCanvas.h>
#include <volume.h>
#include <vertex.h>
#include <utils.h>
#include <gradingOp.h>

using namespace std;
using namespace DFHM;

namespace
{
    wstring _gBaseVolumeName(L"Bounds");
}

AppData::AppData(MainFrame* pMainFrame)
    : _pMainFrame(pMainFrame)
{
    _pModelMeshData = make_shared<map<wstring, MeshDataPtr>>();
    _pModelMeshRepo = make_shared<TriMesh::CMeshRepo>();

    clearCache();
}

AppData::~AppData()
{
}

void AppData::preDestroy()
{
    // Clear these to avoid infinite loop on shared_ptr destruction. Ugly, but gets around hang on exit.
    _pVolume = nullptr;
    _pModelMeshData = nullptr;
}

size_t AppData::numBytes() const
{
    size_t result = 0;
    if (_pVolume)
        result += _pVolume->numBytes();

    result += _pModelMeshRepo->numBytes();
    result += _pModelMeshData->size() * sizeof (pair<wstring, MeshDataPtr>);
    for (const auto& pair : *_pModelMeshData) {
        result += sizeof(pair.first);
        result += pair.first.length() * sizeof(wchar_t);
        result += pair.second->numBytes();
    }

    if (_pVolume)
        result += _pVolume->numBytes();

    return result;
}

bool AppData::doOpen()
{
    wxFileDialog openFileDialog(_pMainFrame, _("Open Triangle Mesh file"), "", "",
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
        MeshDataPtr pMeshData = make_shared<MeshData>(this, pMesh, name);
        _pModelMeshData->insert(make_pair(pMeshData->getName(), pMeshData));

        updateModelTess();
        return true;
    }

    return false;
}

std::wstring AppData::getCacheDirName() const
{
#ifdef _WIN32
    return L"c:/tmp/";
#else
#endif // _WIN32

}

void AppData::updateModelTess()
{
    auto pCanvas = _pMainFrame->getCanvas();
    auto pDrawModelMesh = pCanvas->getDrawModelMesh();
    auto VBOs = pDrawModelMesh->getVBOs();

    auto& edgeVBO = VBOs->_edgeVBO;
    auto& faceVBO = VBOs->_faceVBO;

    edgeVBO.beginEdgeTesselation();
    faceVBO.beginFaceTesselation();
    for (auto& pair : *_pModelMeshData) {
        auto pData = pair.second;
        pData->makeOGLTess(pDrawModelMesh);
    }
    faceVBO.endFaceTesselation(false);
    edgeVBO.endEdgeTesselation();
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

    uint8_t version = 4;
    out.write((char*)&version, sizeof(version));

    _params.write(out);

    size_t numMeshes = _pModelMeshData->size();
    for (const auto& pair : *_pModelMeshData) {
        const auto& pData = pair.second;
    }

    out.write((char*)&numMeshes, sizeof(numMeshes));

    for (const auto& pair : *_pModelMeshData) {
        const auto& pData = pair.second;
        pData->write(out);
    }

    bool hasVolume = _pVolume != nullptr;
    out.write((char*)&hasVolume, sizeof(hasVolume));
    if (hasVolume)
        _pVolume->write(out);    
}

void AppData::readDHFM(const wstring& path, const wstring& filename)
{
    _dhfmFilename = path + filename;

    ifstream in(filesystem::path(_dhfmFilename), ifstream::binary);

    uint8_t version;
    in.read((char*)&version, sizeof(version));
    if (version == 0) {
    } else if (version == 1) {
    } else if (version >= 2) {
        if (version >= 3) {
            _params.read(in);
            if (version < 4) {
                bool hasBaseMesh;
                in.read((char*)&hasBaseMesh, sizeof(hasBaseMesh));
            }
        }

        size_t numMeshes = _pModelMeshData->size();
        in.read((char*)&numMeshes, sizeof(numMeshes));

        for (size_t i = 0; i < numMeshes; i++) {
            auto pData = make_shared<MeshData>(this, _pModelMeshRepo);
            pData->read(in);
            _pModelMeshData->insert(make_pair(pData->getName(), pData));
        }

        updateModelTess();
    }

    bool hasVolume;
    in.read((char*)&hasVolume, sizeof(hasVolume));
    if (hasVolume) {
        _pVolume = make_shared<Volume>();
        _pVolume->setAppData(shared_from_this());

        _pVolume->read(in);
        _pVolume->createAdHocBlockSearchTree();

        buildHexFaceTables();
        copyHexFaceTablesToVBOs();
    }

    _pMainFrame->refreshObjectTree();

    _pMainFrame->getCanvas()->resetView();
}

void AppData::doVerifyClosed(const CMeshPtr& pMesh)
{
    int numOpen = pMesh->numLaminarEdges();

    stringstream ss;
    ss << "Number of edges: " << pMesh->numEdges() << "\nNumber of open edges: " << numOpen;
    wxMessageBox(ss.str().c_str(), "Verify Closed", wxOK | wxICON_INFORMATION);
}

void AppData::doVerifyNormals(const CMeshPtr& pMesh)
{
    size_t numMisMatched = 0;
    size_t meshId = pMesh->getId();
    size_t numEdges = pMesh->numEdges();
    for (size_t i = 0; i < numEdges; i++) {
        const auto& edge = pMesh->getEdge(i);
        if (edge.numFaces(meshId) == 2) {
            size_t ptIdx0 = edge._vertIndex[0];
            size_t ptIdx1 = edge._vertIndex[1];

            const Index3D& faceIndices0 = pMesh->getTri(edge.getTriIdx(meshId, 0));
            const Index3D& faceIndices1 = pMesh->getTri(edge.getTriIdx(meshId, 1));

            bool face0Pos = false, face1Pos = false;
            for (int i = 0; i < 3; i++) {
                if (faceIndices0[i] == ptIdx0) {
                    face0Pos = (faceIndices0[(i + 1) % 3] == ptIdx1);
                    break;
                }
            }

            for (int i = 0; i < 3; i++) {
                if (faceIndices1[i] == ptIdx0) {
                    face1Pos = (faceIndices1[(i + 1) % 3] == ptIdx1);
                    break;
                }
            }

            if (face0Pos == face1Pos) {
                numMisMatched++;
            }
        }
    }

    stringstream ss;
    ss << "Number of tris: " << pMesh->numTris() << "\nNumber of opposed faces: " << numMisMatched;
    wxMessageBox(ss.str().c_str(), "Verify Normals", wxOK | wxICON_INFORMATION);
}

void AppData::doAnalyzeGaps(const CMeshPtr& pMesh)
{
    vector<double> binSizes({ 0.050 / 64, 0.050 / 32, 0.050 / 16, 0.050 / 8, 0.050 / 4, 0.050 / 2, 0.050 });
    vector<size_t> bins;
    bins.resize(binSizes.size(), 0);
    pMesh->getGapHistogram(binSizes, bins, true);

    stringstream ss;
    ss << "Gap histogram\n";
    for (size_t i = 0; i < binSizes.size(); i++) {
        ss << "hits < " << binSizes[i] << ": " << bins[i] << "\n";
    }
    wxMessageBox(ss.str().c_str(), "Gap Analysis", wxOK | wxICON_INFORMATION);
    
}

void AppData::doFindMinGap(const CMeshPtr& pMesh) const
{
    double t = pMesh->findMinGap() * 10;
    auto bb = pMesh->getBBox();
    auto bbMin = bb.getMin();
    auto range = bb.range();
    size_t numX = (size_t)(range[0] / t + 0.5);
    size_t numY = (size_t)(range[1] / t + 0.5);
    size_t numZ = (size_t)(range[2] / t + 0.5);

    numX = (numX / 8 + 1) * 8;
    numY = (numY / 8 + 1) * 8;
    numZ = (numZ / 8 + 1) * 8;
    Index3D dim(numX, numY, numZ);


    MultiCore::runLambda([this, dim, bb](size_t threadNum, size_t numThreads)->bool {
        auto range = bb.range();
        auto bbMin = bb.getMin();
        Vector3d zAxis(0, 0, 1);
        for (size_t ix = threadNum; ix < dim[0]; ix += numThreads) {
            double t = ix / (dim[0] - 1.0);
            double x = bbMin[0] + t * range[0];
            for (size_t iy = 0; iy < dim[1]; iy++) {
                double u = iy / (dim[1] - 1.0);
                double y = bbMin[1] + u * range[1];
                Vector3d ctr(x, y, 0);
                Rayd ray(ctr, zAxis);

#if DEBUG_BREAKS && defined(_DEBUG)
                vector<RayHit> hits;
                if (_pMesh->rayCast(ray, hits)) {
                    int dbgBreak = 1;
                }
#endif
            }
        }
        return true;
    }, true);


    stringstream ss;
    ss << "Span: [" << numX << ", " << numY << ", " << numZ << "]\n";
    wxMessageBox(ss.str().c_str(), "Box span in steps", wxOK | wxICON_INFORMATION);
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
    pCanvas->setShowMeshSelectedBlocks(true);

    buildHexFaceTables();
    copyHexFaceTablesToVBOs();
}

CBoundingBox3Dd AppData::getBoundingBox() const
{
    CBoundingBox3Dd result;
    for (const auto& pair : *_pModelMeshData) {
        const auto& pData = pair.second;
        result.merge(pData->getMesh()->getBBox());
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
    for (const auto& pair : *_pModelMeshData) {
        const auto pData = pair.second;
        if (pData->isActive())
            result.merge(pData->getMesh()->getBBox());
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

    for (const auto& pair : *_pModelMeshData) {
        const auto pMesh = pair.second->getMesh();
        for (size_t i = 0; i < pMesh->numVertices(); i++) {
            const Vector3d& pt = pMesh->getVert(i)._pt;
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
        }
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

void AppData::clear(bool includeModelData)
{
    auto pCanvas = _pMainFrame->getCanvas();
    pCanvas->clearMesh3D();

    _pVolume = nullptr;

    clearCache();

    if (includeModelData) {
        pCanvas->clearModel();
        _pModelMeshData = make_shared<map<wstring, MeshDataPtr>>();
        _pModelMeshRepo = make_shared<TriMesh::CMeshRepo>();
    }
}

void AppData::doCreateBaseVolume()
{
    clear(false);
    _pVolume = make_shared<Volume>(_params.volDivs);
    _pVolume->setAppData(shared_from_this());

    _pVolume->setVolCornerPts(_params.getVolBounds());

    Index3D::setBlockDim(1);

    _pMainFrame->startProgress(14);
    auto pFuture = make_shared<future<int>>(async(std::launch::async, [this]()->int {
        Vector3d cubePts[8];
        CBoundingBox3Dd volBox;
        makeModelCubePoints(cubePts, volBox);

        _pVolume->buildModelBlocks(_params, cubePts, volBox, _pMainFrame, RUN_MULTI_THREAD);

        //    _pVolume->verifyUniquePoints(RUN_MULTI_THREAD);

        _pVolume->setLayerNums();
        _pMainFrame->reportProgress();

        buildHexFaceTables();

        _pMainFrame->reportProgress();

        return 2;
    }));
    _pMainFrame->setFuture(pFuture);
}

void AppData::doRemoveBaseVolume()
{
    _pVolume = nullptr;
}

void AppData::doBuildCFDHexes(const BuildCFDHexesDlg& dlg)
{
    try {
        auto pCanvas = _pMainFrame->getCanvas();
        pCanvas->clearMesh3D();

        dlg.getParams(_params);

        size_t numProgSteps = 1 + _pModelMeshData->size() + _params.numSimpleDivs + 3 * _params.numConditionalPasses();
        _pMainFrame->startProgress(numProgSteps);
        auto pFuture = make_shared<future<int>>(async(std::launch::async, [this]()->int {
            _pVolume->buildCFDHexes(*_pModelMeshData, _params, _pMainFrame, RUN_MULTI_THREAD);

            buildHexFaceTables();
            _pMainFrame->reportProgress();

            return 2;
        }));
        _pMainFrame->setFuture(pFuture);

    } catch (const char* errStr) {
        cout << errStr << "\n";
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
