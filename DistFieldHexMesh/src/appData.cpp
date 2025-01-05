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

bool AppData::doOpen()
{
    wxFileDialog openFileDialog(_pMainFrame, _("Open Triangle Mesh file"), "", "",
        "All (*.dfhm)|*.dfhm|DFHM files (*.dfhm)|*.dfhm", wxFD_OPEN | wxFD_FILE_MUST_EXIST);
    if (openFileDialog.ShowModal() == wxID_CANCEL)
        return false;     // the user canceled


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
        makeModelTess();
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
    try {
        if (reader.read(path, filename)) {
            return pMesh;
        }
    }
    catch (const char* errStr) {
        cout << errStr << "\n";
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
    auto pDrawModelMesh = pCanvas->getDrawModelMesh();
    auto pVBOs = pDrawModelMesh->getVBOs();
    if (filename.find(L".stl") != -1) {
        auto pMesh = readStl(path, filename);
        auto pos = filename.find(L".");
        wstring name = filename.replace(pos, filename.size(), L"");
        MeshDataPtr pMeshData = make_shared<MeshData>(this, pMesh, name, _pMainFrame->getCanvas()->getViewOptions());
        _pModelMeshData->insert(make_pair(pMeshData->getName(), pMeshData));

        makeModelTess();
        return true;
    }

    return false;
}

void AppData::makeModelTess()
{
    auto pCanvas = _pMainFrame->getCanvas();
    auto pDrawModelMesh = pCanvas->getDrawModelMesh();
    auto pVBOs = pDrawModelMesh->getVBOs();

    pVBOs->_edgeVBO.beginEdgeTesselation();
    pVBOs->_faceVBO.beginFaceTesselation();
    for (auto& pair : *_pModelMeshData) {
        auto pData = pair.second;
        pData->makeOGLTess(pDrawModelMesh);
    }
    pVBOs->_faceVBO.endFaceTesselation(false);
    pVBOs->_edgeVBO.endEdgeTesselation();
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

    uint8_t version = 3;
    out.write((char*)&version, sizeof(version));

    _params.write(out);
    bool hasBaseMesh = _pModelMeshData->find(_gBaseVolumeName) != _pModelMeshData->end();
    out.write((char*)&hasBaseMesh, sizeof(hasBaseMesh));

    size_t numMeshes = 0;
    for (const auto& pair : *_pModelMeshData) {
        const auto& pData = pair.second;
        if (!pData->isReference())
            numMeshes++;
    }

    out.write((char*)&numMeshes, sizeof(numMeshes));

    for (const auto& pair : *_pModelMeshData) {
        const auto& pData = pair.second;
        if (!pData->isReference())
            pData->write(out);
    }

    bool hasVolume = _pVolume != nullptr;
    out.write((char*)&hasVolume, sizeof(hasVolume));
    if (hasVolume)
        _pVolume->write(out);    
}

void AppData::readDHFM(const wstring& path, const wstring& filename)
{
    bool hasBaseMesh = false;
    _dhfmFilename = path + filename;

    ifstream in(filesystem::path(_dhfmFilename), ifstream::binary);

    uint8_t version;
    in.read((char*)&version, sizeof(version));
    if (version == 0) {
    } else if (version == 1) {
    } else if (version >= 2) {
        if (version >= 3) {
            _params.read(in);
            in.read((char*)&hasBaseMesh, sizeof(hasBaseMesh));
        }

        size_t numMeshes = _pModelMeshData->size();
        in.read((char*)&numMeshes, sizeof(numMeshes));

        for (size_t i = 0; i < numMeshes; i++) {
            auto pData = make_shared<MeshData>(this, _pMainFrame->getCanvas()->getViewOptions(), _pModelMeshRepo);
            pData->read(in);
            _pModelMeshData->insert(make_pair(pData->getName(), pData));
        }

        if (hasBaseMesh)
            doCreateBaseVolumePreview();

        makeModelTess();
    }

    bool hasVolume;
    in.read((char*)&hasVolume, sizeof(hasVolume));
    if (hasVolume) {
        _pVolume = make_shared<Volume>();
        _pVolume->setAppData(shared_from_this());

        _pVolume->read(in);
        updateTessellation();
    }

    if (hasBaseMesh)
        doCreateBaseVolumePreview();
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
    size_t numEdges = pMesh->numEdges();
    for (size_t i = 0; i < numEdges; i++) {
        const auto& edge = pMesh->getEdge(i);
        auto pTopol = edge.getTopol(pMesh->getId());
        if (pTopol->_numFaces == 2) {
            size_t ptIdx0 = edge._vertIndex[0];
            size_t ptIdx1 = edge._vertIndex[1];

            const Index3D& faceIndices0 = pMesh->getTri(pTopol->_faceIndices[0]);
            const Index3D& faceIndices1 = pMesh->getTri(pTopol->_faceIndices[1]);

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

    updateTessellation();
}

CBoundingBox3Dd AppData::getBoundingBox() const
{
    CBoundingBox3Dd result;
    for (const auto& pair : *_pModelMeshData) {
        const auto& pData = pair.second;
        if (!pData->isReference())
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
        if (!pData->isReference() && pData->isActive())
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

void AppData::doCreateBaseVolumePreview()
{
    CMeshPtr pMesh;
    auto makeGradedBlock = [&pMesh](const Vector3d cPts[8], CubeFaceType dir, CubeFaceType dir1, CubeFaceType dir2, const GradingOp& gr) {
        const auto& divs = gr.getDivs();
        if (divs[0] == 0) {
            pMesh->addQuad(cPts[0], cPts[3], cPts[2], cPts[1]);
            pMesh->addQuad(cPts[4], cPts[5], cPts[6], cPts[7]);
            pMesh->addQuad(cPts[1], cPts[2], cPts[6], cPts[5]);
            pMesh->addQuad(cPts[0], cPts[4], cPts[7], cPts[3]);
            pMesh->addQuad(cPts[0], cPts[1], cPts[5], cPts[4]);
            pMesh->addQuad(cPts[3], cPts[7], cPts[6], cPts[2]);
        } else {
            double xScale, yScale, zScale;
            double xGrading, yGrading, zGrading;
            gr.calGradingFactors(0, xScale, xGrading);
            gr.calGradingFactors(1, yScale, yGrading);
            gr.calGradingFactors(2, zScale, zGrading);

            double kx = 1;
            double t0 = 0;
            for (size_t i = 0; i < divs[0]; i++) {
                double t1 = t0 + 1.0 / (double)divs[0] * kx * xScale;
                kx *= xGrading;

                double ky = 1;
                double u0 = 0;
                for (size_t j = 0; j < divs[1]; j++) {
                    double u1 = u0 + 1.0 / (double)divs[1] * ky * yScale;
                    ky *= yGrading;

                    double kz = 1;
                    double v0 = 0;
                    for (size_t k = 0; k < divs[2]; k++) {
                        double v1 = v0 + 1.0 / (double)divs[2] * kz * zScale;
                        kz *= zGrading;

                        Vector3d gPts[8];
                        gPts[0] = TRI_LERP(cPts, t0, u0, v0);
                        gPts[1] = TRI_LERP(cPts, t1, u0, v0);
                        gPts[2] = TRI_LERP(cPts, t1, u1, v0);
                        gPts[3] = TRI_LERP(cPts, t0, u1, v0);
                        gPts[4] = TRI_LERP(cPts, t0, u0, v1);
                        gPts[5] = TRI_LERP(cPts, t1, u0, v1);
                        gPts[6] = TRI_LERP(cPts, t1, u1, v1);
                        gPts[7] = TRI_LERP(cPts, t0, u1, v1);

                        pMesh->addQuad(gPts[0], gPts[3], gPts[2], gPts[1]);
                        pMesh->addQuad(gPts[4], gPts[5], gPts[6], gPts[7]);
                        pMesh->addQuad(gPts[1], gPts[2], gPts[6], gPts[5]);
                        pMesh->addQuad(gPts[0], gPts[4], gPts[7], gPts[3]);
                        pMesh->addQuad(gPts[0], gPts[1], gPts[5], gPts[4]);
                        pMesh->addQuad(gPts[3], gPts[7], gPts[6], gPts[2]);

                        v0 = v1;
                    }
                    u0 = u1;
                }
                t0 = t1;
            }
        }
        };

    //    Volume::setVolDim(Index3D(5, 5, 5));
    auto pCanvas = _pMainFrame->getCanvas();
    pCanvas->clearMesh3D();
    _pVolume = make_shared<Volume>();

    doRemoveBaseVolumePreview();

    Vector3d cubePts[8];
    CBoundingBox3Dd bbox, volBox;
    makeModelCubePoints(cubePts, volBox);

    for (size_t i = 0; i < 8; i++) {
        bbox.merge(cubePts[i]);
        bbox.merge(cubePts[i]);
    }
    bbox.growPercent(0.05);
    volBox.growPercent(0.05);

    pMesh = make_shared<CMesh>(volBox/*, _pModelMeshRepo*/);
#if 0
    {
        GradingOp r;
        r.setDivs(_params.volDivs);
        makeGradedBlock(cubePts, CFT_UNDEFINED, CFT_UNDEFINED, CFT_UNDEFINED, r);
    }
#endif

    makeSurroundingBlocks(cubePts, makeGradedBlock);

    MeshDataPtr pMeshData = make_shared<MeshData>(this, pMesh, _gBaseVolumeName, _pMainFrame->getCanvas()->getViewOptions());
    pMeshData->setReference(true);

    _pModelMeshData->insert(make_pair(pMeshData->getName(), pMeshData));

    _pMainFrame->refreshObjectTree();
    makeModelTess();
    pCanvas->changeViewElements();
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
}

template<class L>
void AppData::makeSurroundingBlocks(Vector3d cPts[8], const L& f) const
{
    if (!_params.symXAxis)
        _pVolume->insertBlocks(_params, CFT_BACK);
#if 1
    _pVolume->insertBlocks(_params, CFT_FRONT);

    if (!_params.symYAxis)
        _pVolume->insertBlocks(_params, CFT_LEFT);

    _pVolume->insertBlocks(_params, CFT_RIGHT);

    if (!_params.symZAxis)
        _pVolume->insertBlocks(_params, CFT_BOTTOM);

    _pVolume->insertBlocks(_params, CFT_TOP);
#endif

#if 0
    makeGradedHexOnFace(cPts, CFT_BOTTOM, f);
    makeGradedHexOnFace(cPts, CFT_TOP, f);
    makeGradedHexOnFace(cPts, CFT_FRONT, f);
    makeGradedHexOnFace(cPts, CFT_BACK, f);
    if (!_params.symYAxis)
        makeGradedHexOnFace(cPts, CFT_LEFT, f);
    makeGradedHexOnFace(cPts, CFT_RIGHT, f);

    makeGradedHexOnEdge(cPts, CFT_BACK, CFT_BOTTOM, f);
    makeGradedHexOnEdge(cPts, CFT_BACK, CFT_TOP, f);
    if (!_params.symYAxis)
        makeGradedHexOnEdge(cPts, CFT_BACK, CFT_LEFT, f);
    makeGradedHexOnEdge(cPts, CFT_BACK, CFT_RIGHT, f);

    makeGradedHexOnEdge(cPts, CFT_FRONT, CFT_BOTTOM, f);
    makeGradedHexOnEdge(cPts, CFT_FRONT, CFT_TOP, f);
    if (!_params.symYAxis)
        makeGradedHexOnEdge(cPts, CFT_FRONT, CFT_LEFT, f);
    makeGradedHexOnEdge(cPts, CFT_FRONT, CFT_RIGHT, f);

    makeGradedHexOnEdge(cPts, CFT_RIGHT, CFT_BOTTOM, f);
    makeGradedHexOnEdge(cPts, CFT_RIGHT, CFT_TOP, f);

    makeGradedHexOnCorners(cPts, f);
#endif

}

void AppData::gradeSurroundingBlocks() const
{
    Index3D idx;
    const auto& dims = _pVolume->volDim();
    CubeFaceType dir0, dir1, dir2;

#if 1
    for (idx[0] = 0; idx[0] < dims[0]; idx[0]++) {
        for (idx[1] = 0; idx[1] < dims[1]; idx[1]++) {
            for (int j = 0; j < 2; j++) {
                dir0 = dir1 = dir2 = CFT_UNDEFINED;
                Index3D divs(1, 1, 1);
                Vector3d grading(1, 1, 1);

                if (j == 0) {
                    idx[2] = 0;
                    dir0 = CFT_BOTTOM;
                    if (!_params.symZAxis) {
                        divs[2] = _params.zMinDivs;
                        grading[2] = 1 / _params.zMinGrading;
                    }
                } else {
                    idx[2] = dims[2] - 1;
                    dir0 = CFT_TOP;
                    divs[2] = _params.zMaxDivs;
                    grading[2] = _params.zMaxGrading;
                }

                if (idx[0] == 0 && !_params.symXAxis) {
                    dir1 = CFT_BACK;
                    divs[0] = _params.xMinDivs;
                    grading[0] = 1 / _params.xMinGrading;
                } else if (idx[0] == dims[0] - 1) {
                    dir1 = CFT_FRONT;
                    divs[0] = _params.xMaxDivs;
                    grading[0] = _params.xMaxGrading;
                }

                if (idx[1] == 0 && !_params.symYAxis) {
                    divs[1] = _params.yMinDivs;
                    grading[1] = 1 / _params.yMinGrading;

                    if (dir1 == CFT_UNDEFINED)
                        dir1 = CFT_LEFT;
                    else
                        dir2 = CFT_LEFT;
                } else if (idx[1] == dims[1] - 1) {
                    divs[1] = _params.yMaxDivs;
                    grading[1] = _params.yMaxGrading;

                    if (dir1 == CFT_UNDEFINED)
                        dir1 = CFT_RIGHT;
                    else
                        dir2 = CFT_RIGHT;
                }

                auto pBlk = _pVolume->getBlockPtr(idx);
                GradingOp gr(pBlk, _params, divs, grading);
                gr.createGradedCells();
            }
        }
    }
#endif
}

template<class L>
void AppData::makeGradedHexOnFace(Vector3d cPts[8], CubeFaceType dir, const L& fLambda) const
{
    Vector3d pts[8];
    GradingOp gr;

    switch (dir) {
        case CFT_BACK:
            gr.setDivs(Vector3i(_params.xMinDivs, _params.volDivs[1], _params.volDivs[2]));
            gr.setGrading(Vector3d(1 / _params.xMinGrading, 1, 1));

            pts[0] = pts[1] = cPts[0];
            pts[3] = pts[2] = cPts[3];
            pts[7] = pts[6] = cPts[7];
            pts[4] = pts[5] = cPts[4];
            pts[0][0] = _params.xMin;
            pts[3][0] = _params.xMin;
            pts[7][0] = _params.xMin;
            pts[4][0] = _params.xMin;
            break;
        case CFT_FRONT:
            gr.setDivs(Vector3i(_params.xMaxDivs, _params.volDivs[1], _params.volDivs[2]));
            gr.setGrading(Vector3d(_params.xMaxGrading, 1, 1));

            pts[0] = pts[1] = cPts[1];
            pts[3] = pts[2] = cPts[2];
            pts[7] = pts[6] = cPts[6];
            pts[4] = pts[5] = cPts[5];
            pts[1][0] = _params.xMax;
            pts[2][0] = _params.xMax;
            pts[6][0] = _params.xMax;
            pts[5][0] = _params.xMax;
            break;
        case CFT_BOTTOM:
            gr.setDivs(Vector3i(_params.volDivs[0], _params.volDivs[1], _params.zMinDivs));
            gr.setGrading(Vector3d(1, 1, 1 / _params.zMinGrading));
            pts[4] = pts[0] = cPts[0];
            pts[5] = pts[1] = cPts[1];
            pts[6] = pts[2] = cPts[2];
            pts[7] = pts[3] = cPts[3];
            pts[0][2] = _params.zMin;
            pts[1][2] = _params.zMin;
            pts[2][2] = _params.zMin;
            pts[3][2] = _params.zMin;
            break;
        case CFT_TOP:
            gr.setDivs(Vector3i(_params.volDivs[0], _params.volDivs[1], _params.zMaxDivs));
            gr.setGrading(Vector3d(1, 1, _params.zMaxGrading));

            pts[4] = pts[0] = cPts[4];
            pts[5] = pts[1] = cPts[5];
            pts[6] = pts[2] = cPts[6];
            pts[7] = pts[3] = cPts[7];
            pts[4][2] = _params.zMax;
            pts[5][2] = _params.zMax;
            pts[6][2] = _params.zMax;
            pts[7][2] = _params.zMax;
            break;
        case CFT_LEFT:
            gr.setDivs(Vector3i(_params.volDivs[0], _params.yMinDivs, _params.volDivs[2]));
            gr.setGrading(Vector3d(1, 1 / _params.yMinGrading, 1));

            pts[2] = pts[1] = cPts[2];
            pts[3] = pts[0] = cPts[3];
            pts[7] = pts[4] = cPts[7];
            pts[6] = pts[5] = cPts[6];
            pts[1][1] = _params.yMin;
            pts[0][1] = _params.yMin;
            pts[4][1] = _params.yMin;
            pts[5][1] = _params.yMin;
            break;
        case CFT_RIGHT:
            gr.setDivs(Vector3i(_params.volDivs[0], _params.yMaxDivs, _params.volDivs[2]));
            gr.setGrading(Vector3d(1, _params.yMaxGrading, 1));

            pts[3] = pts[0] = cPts[3];
            pts[2] = pts[1] = cPts[2];
            pts[7] = pts[4] = cPts[7];
            pts[6] = pts[5] = cPts[6];
            pts[2][1] = _params.yMax;
            pts[3][1] = _params.yMax;
            pts[7][1] = _params.yMax;
            pts[6][1] = _params.yMax;
            break;
    }

    fLambda(pts, dir, CFT_UNDEFINED, CFT_UNDEFINED, gr);
}

template<class L>
void AppData::makeGradedHexOnEdge(Vector3d cPts[8], CubeFaceType dir0, CubeFaceType dir1, const L& fLambda) const
{
    Vector3d pts[8];
    GradingOp gr;

    switch (dir0) {
    case CFT_BACK:
        switch (dir1) {
        case CFT_BOTTOM:
            gr.setDivs(Vector3i(_params.xMinDivs, _params.volDivs[1], _params.zMinDivs));
            gr.setGrading(Vector3d(1 / _params.xMinGrading, 1, 1 / _params.zMinGrading));

            pts[1] = pts[4] = pts[5] = cPts[0];
            pts[2] = pts[7] = pts[6] = cPts[3];

            pts[1][2] = _params.zMin;
            pts[2][2] = _params.zMin;

            pts[7][0] = _params.xMin;
            pts[4][0] = _params.xMin;

            pts[0] = Vector3d(_params.xMin, cPts[0][1], _params.zMin);
            pts[3] = Vector3d(_params.xMin, cPts[3][1], _params.zMin);
            break;
        case CFT_TOP:
            gr.setDivs(Vector3i(_params.xMinDivs, _params.volDivs[1], _params.zMaxDivs));
            gr.setGrading(Vector3d(1 / _params.xMinGrading, 1, _params.zMaxGrading));

            pts[1] = cPts[4];
            pts[0] = Vector3d(_params.xMin, cPts[4][1], cPts[4][2]);
            pts[5] = Vector3d(cPts[4][0], cPts[4][1], _params.zMax);
            pts[4] = Vector3d(_params.xMin, cPts[4][1], _params.zMax);

            pts[2] = cPts[7];
            pts[3] = Vector3d(_params.xMin, cPts[7][1], cPts[7][2]);
            pts[6] = Vector3d(cPts[7][0], cPts[7][1], _params.zMax);
            pts[7] = Vector3d(_params.xMin, cPts[7][1], _params.zMax);
            break;
        case CFT_LEFT:
            gr.setDivs(Vector3i(_params.xMinDivs, _params.yMinDivs, _params.volDivs[2]));
            gr.setGrading(Vector3d(1 / _params.xMinGrading, 1 / _params.yMinGrading, 1));
            pts[0] = pts[1] = pts[2] = pts[3] = cPts[2];
            pts[0][0] = _params.xMin;
            pts[2][1] = _params.yMin;
            pts[3][0] = _params.xMin;
            pts[3][1] = _params.yMin;

            pts[4] = pts[5] = pts[6] = pts[7] = cPts[6];
            pts[4][0] = _params.xMin;
            pts[6][1] = _params.yMin;
            pts[7][0] = _params.xMin;
            pts[7][1] = _params.yMin;
            break;
        case CFT_RIGHT:
            gr.setDivs(Vector3i(_params.xMinDivs, _params.yMaxDivs, _params.volDivs[2]));
            gr.setGrading(Vector3d(1 / _params.xMinGrading, _params.yMaxGrading, 1));

            pts[0] = pts[2] = pts[3] = pts[1] = cPts[3];
            // pts[1]
            pts[0][0] = _params.xMin;
            pts[2][1] = _params.yMax;
            pts[3][1] = _params.yMax;
            pts[3][0] = _params.xMin;

            pts[4] =pts[5] = pts[6] = pts[7] = cPts[7];
            // pts[5]
            pts[4][0] = _params.xMin;
            pts[6][1] = _params.yMax;
            pts[7][0] = _params.xMin;
            pts[7][1] = _params.yMax;
            break;
        }
        break;
    case CFT_FRONT:
        switch (dir1) {
        case CFT_BOTTOM:
            gr.setDivs(Vector3i(_params.xMaxDivs, _params.volDivs[1], _params.zMinDivs));
            gr.setGrading(Vector3d(_params.xMaxGrading, 1, 1 / _params.zMinGrading));

            pts[0] = pts[1] = pts[4] = pts[5] = cPts[1];
            pts[0][2] = _params.zMin;
            pts[5][0] = _params.xMax;
            pts[1][0] = _params.xMax;
            pts[1][2] = _params.zMin;

            pts[2] = pts[3] = pts[6] = pts[7] = cPts[2];
            pts[6][0] = _params.xMax;
            pts[3][2] = _params.zMin;
            pts[2][0] = _params.xMax;
            pts[2][2] = _params.zMin;
            break;
        case CFT_TOP:
            gr.setDivs(Vector3i(_params.xMaxDivs, _params.volDivs[1], _params.zMaxDivs));
            gr.setGrading(Vector3d(_params.xMaxGrading, 1, _params.zMaxGrading));
            pts[0] = pts[1] = pts[4] = pts[5] = cPts[5];
            pts[1][0] = _params.xMax;
            pts[4][2] = _params.zMax;
            pts[5][0] = _params.xMax;
            pts[5][2] = _params.zMax;

            pts[2] = pts[3] = pts[6] = pts[7] = cPts[6];
            pts[2][0] = _params.xMax;
            pts[7][2] = _params.zMax;
            pts[6][0] = _params.xMax;
            pts[6][2] = _params.zMax;
            break;
        case CFT_LEFT:
            return;
            break;
        case CFT_RIGHT:
            gr.setDivs(Vector3i(_params.xMaxDivs, _params.yMaxDivs, _params.volDivs[2]));
            gr.setGrading(Vector3d(_params.xMaxGrading, _params.yMaxGrading, 1));

            pts[0] = pts[1] = pts[2] = pts[3] = cPts[2];
            // pts[0]
            pts[1][0] = _params.xMax;
            pts[3][1] = _params.yMax;
            pts[2][0] = _params.xMax;
            pts[2][1] = _params.yMax;

            pts[4] = pts[5] = pts[6] = pts[7] = cPts[6];
            // pts[4]
            pts[5][0] = _params.xMax;
            pts[6][0] = _params.xMax;
            pts[6][1] = _params.yMax;
            pts[7][1] = _params.yMax;
            break;
        }
        break;
    case CFT_BOTTOM:
        return;
        break;
    case CFT_TOP:
        return;
        break;
    case CFT_LEFT:
        return;
        break;
    case CFT_RIGHT:
        switch (dir1) {
        case CFT_BOTTOM:
            gr.setDivs(Vector3i(_params.volDivs[0], _params.yMaxDivs, _params.zMinDivs));
            gr.setGrading(Vector3d(1, _params.yMaxGrading, 1 / _params.zMinGrading));

            pts[0] = pts[3] = pts[4] = pts[7] = cPts[3];
            pts[0][2] = _params.zMin;
            pts[3][1] = _params.yMax;
            pts[3][2] = _params.zMin;
            // pts[4]
            pts[7][1] = _params.yMax;

            pts[1] = pts[2] = pts[5] = pts[6] = cPts[2];
            pts[1][2] = _params.zMin;
            // pts[5]
            pts[6][1] = _params.yMax;
            pts[2][1] = _params.yMax;
            pts[2][2] = _params.zMin;
            break;
        case CFT_TOP:
            gr.setDivs(Vector3i(_params.volDivs[0], _params.yMaxDivs, _params.zMaxDivs));
            gr.setGrading(Vector3d(1, _params.yMaxGrading, _params.zMaxGrading));

            pts[0] = pts[3] = pts[4] = pts[7] = cPts[7];
            // pts[0]
            pts[3][1] = _params.yMax;
            pts[4][2] = _params.zMax;
            pts[7][1] = _params.yMax;
            pts[7][2] = _params.zMax;

            pts[1] = pts[2] = pts[5] = pts[6] = cPts[6];
            // pts[1]
            pts[2][1] = _params.yMax;
            pts[5][2] = _params.zMax;
            pts[6][1] = _params.yMax;
            pts[6][2] = _params.zMax;
            break;
        case CFT_LEFT:
            return;
            break;
        case CFT_RIGHT:
            return;
            break;
        }
        break;
    }

    fLambda(pts, dir0, dir1, CFT_UNDEFINED, gr);
}

template<class L>
void AppData::makeGradedHexOnCorners(Vector3d cPts[8], const L& fLambda) const
{
    Vector3d pts[8];
    GradingOp gr;
    CubeFaceType dir0, dir1, dir2;

    for (int i = 0; i < 8; i++) {
        for (int j = 0; j < 8; j++)
            pts[j] = cPts[i];
        switch (i) {
        case 0: {
            if (_params.symXAxis || _params.symYAxis || _params.symZAxis)
                continue;
            continue;
            break;
        }
        case 1: {
            if (_params.symYAxis || _params.symZAxis)
                continue;
            continue;
            break;
        }
        case 2: {
            if (_params.symZAxis)
                continue;

            dir0 = CFT_FRONT;
            dir1 = CFT_RIGHT;
            dir2 = CFT_BOTTOM;

            gr.setDivs(Vector3i(_params.xMaxDivs, _params.yMaxDivs, _params.zMinDivs));
            gr.setGrading(Vector3d(_params.xMaxGrading, _params.yMaxGrading, 1 / _params.zMinGrading));

            pts[0][2] = _params.zMin;

            pts[1][0] = _params.xMax;
            pts[1][2] = _params.zMin;

            pts[2] = Vector3d(_params.xMax, _params.yMax, _params.zMin);

            pts[3][1] = _params.yMax;
            pts[3][2] = _params.zMin;

            // pts[4]

            pts[5][0] = _params.xMax;

            pts[6][0] = _params.xMax;
            pts[6][1] = _params.yMax;

            pts[7][1] = _params.yMax;
            break;
        }
        case 3: {
            if (_params.symXAxis || _params.symZAxis)
                continue;

            dir0 = CFT_BACK;
            dir1 = CFT_RIGHT;
            dir2 = CFT_BOTTOM;

            gr.setDivs(Vector3i(_params.xMinDivs, _params.yMaxDivs, _params.zMinDivs));
            gr.setGrading(Vector3d(1 / _params.xMinGrading, _params.yMaxGrading, 1 / _params.zMinGrading));

            pts[0][0] = _params.xMin;
            pts[0][2] = _params.zMin;

            pts[1][2] = _params.zMin;

            pts[2][1] = _params.yMax;
            pts[2][2] = _params.zMin;

            pts[3] = Vector3d(_params.xMin, _params.yMax, _params.zMin);

            pts[4][0] = _params.xMin;

            // pts[5]

            pts[6][1] = _params.yMax;

            pts[7][0] = _params.xMin;
            pts[7][1] = _params.yMax;
            break;
        }
        case 4: {
            if (_params.symXAxis || _params.symYAxis)
                continue;
            continue;
            break;
        }
        case 5: {
            if (_params.symYAxis)
                continue;
            continue;
            break;
        }
        case 6: {
            // pts[0]

            dir0 = CFT_FRONT;
            dir1 = CFT_RIGHT;
            dir2 = CFT_TOP;

            gr.setDivs(Vector3i(_params.xMaxDivs, _params.yMaxDivs, _params.zMaxDivs));
            gr.setGrading(Vector3d(_params.xMaxGrading, _params.yMaxGrading, _params.zMaxGrading));

            pts[1][0] = _params.xMax;

            pts[2][0] = _params.xMax;
            pts[2][1] = _params.yMax;

            pts[3][1] = _params.yMax;

            pts[4][2] = _params.zMax;

            pts[5][0] = _params.xMax;
            pts[5][2] = _params.zMax;

            pts[6] = Vector3d(_params.xMax, _params.yMax, _params.zMax);

            pts[7][1] = _params.yMax;
            pts[7][2] = _params.zMax;
            break;
        }
        case 7: {
            if (_params.symXAxis)
                continue;

            dir0 = CFT_BACK;
            dir1 = CFT_RIGHT;
            dir2 = CFT_TOP;

            gr.setDivs(Vector3i(_params.xMinDivs, _params.yMaxDivs, _params.zMaxDivs));
            gr.setGrading(Vector3d(1 / _params.xMinGrading, _params.yMaxGrading, _params.zMaxGrading));

            pts[0][0] = _params.xMin;

            // pts[1]

            pts[2][1] = _params.yMax;

            pts[3][0] = _params.xMin;
            pts[3][1] = _params.yMax;

            pts[4][0] = _params.xMin;
            pts[4][2] = _params.zMax;

            pts[5][2] = _params.zMax;

            pts[6][1] = _params.yMax;
            pts[6][2] = _params.zMax;

            pts[7] = Vector3d(_params.xMin, _params.yMax, _params.zMax);
            break;
        }
        }

        fLambda(pts, dir0, dir1, dir2, gr);
    }

}

void AppData::doRemoveBaseVolumePreview()
{
    auto iter = _pModelMeshData->find(_gBaseVolumeName);
    if (iter != _pModelMeshData->end()) {
        auto pData = iter->second;
        _pModelMeshData->erase(iter);
    }

    _pMainFrame->refreshObjectTree();
}

void AppData::doCreateBaseVolume()
{
    _pVolume = nullptr;
    auto makeGradedBlock = [this](const Vector3d cPts[8], CubeFaceType dir0, CubeFaceType dir1, CubeFaceType dir2, const GradingOp& gr) {
#if 1
        const auto& divs = gr.getDivs();
        if (divs[0] != 0) {
            double xScale, yScale, zScale;
            double xGrading, yGrading, zGrading;
            gr.calGradingFactors(0, xScale, xGrading);
            gr.calGradingFactors(1, yScale, yGrading);
            gr.calGradingFactors(2, zScale, zGrading);

            double kx = 1;
            double t0 = 0;
            for (size_t i = 0; i < divs[0]; i++) {
                double t1 = t0 + 1.0 / (double)divs[0] * kx * xScale;
                kx *= xGrading;

                double ky = 1;
                double u0 = 0;
                for (size_t j = 0; j < divs[1]; j++) {
                    double u1 = u0 + 1.0 / (double)divs[1] * ky * yScale;
                    ky *= yGrading;

                    double kz = 1;
                    double v0 = 0;
                    for (size_t k = 0; k < divs[2]; k++) {
                        double v1 = v0 + 1.0 / (double)divs[2] * kz * zScale;
                        kz *= zGrading;

                        Index3D blkIdx(i, j, k);
                        if (dir1 == CFT_UNDEFINED && dir2 == CFT_UNDEFINED) {
                            switch (dir0) {
                            case CFT_BACK:
                                blkIdx[0] = 0;
                                break;
                            case CFT_FRONT:
                                blkIdx[0] = divs[0] - 1;
                                break;
                            case CFT_LEFT:
                                blkIdx[1] = 0;
                                break;
                            case CFT_RIGHT:
                                blkIdx[1] = divs[1] - 1;
                                break;
                            case CFT_BOTTOM:
                                blkIdx[2] = 0;
                                break;
                            case CFT_TOP:
                                blkIdx[2] = divs[2] - 1;
                                break;
                            default:
                                break;
                            }
                        }
                        auto ownerBlock = _pVolume->getBoundingBlock(blkIdx, cPts);

                        vector<Vector3d> gPts;
                        gPts.resize(8);
                        gPts[0] = TRI_LERP(cPts, t0, u0, v0);
                        gPts[1] = TRI_LERP(cPts, t1, u0, v0);
                        gPts[2] = TRI_LERP(cPts, t1, u1, v0);
                        gPts[3] = TRI_LERP(cPts, t0, u1, v0);
                        gPts[4] = TRI_LERP(cPts, t0, u0, v1);
                        gPts[5] = TRI_LERP(cPts, t1, u0, v1);
                        gPts[6] = TRI_LERP(cPts, t1, u1, v1);
                        gPts[7] = TRI_LERP(cPts, t0, u1, v1);
                        ownerBlock->addHexCell(gPts);

                        v0 = v1;
                    }
                    u0 = u1;
                }
                t0 = t1;
            }
        }
#endif
    };

    _pVolume = make_shared<Volume>(_params.volDivs);
    _pVolume->setAppData(shared_from_this());

    _pVolume->setVolCornerPts(_params.getVolBounds());
    auto pCanvas = _pMainFrame->getCanvas();
    pCanvas->clearMesh3D();

    doRemoveBaseVolumePreview();
#if 1
    Vector3d cubePts[8];
    CBoundingBox3Dd volBox;
    makeModelCubePoints(cubePts, volBox);
    Index3D::setBlockDim(1);

    _pVolume->buildBlocks(_params, cubePts, volBox, false);
    makeSurroundingBlocks(cubePts, makeGradedBlock);
    gradeSurroundingBlocks();

    updateTessellation();
#endif
}

void AppData::doRemoveBaseVolume()
{}

bool AppData::doesBaseMeshExist() const
{
    return _pModelMeshData->find(_gBaseVolumeName) != _pModelMeshData->end();
}

void AppData::doBuildCFDHexes(const BuildCFDHexesDlg& dlg)
{
    try {
        auto pCanvas = _pMainFrame->getCanvas();
        pCanvas->clearMesh3D();
        _pVolume = make_shared<Volume>();

        dlg.getParams(_params);

//        _pVolume->buildCFDHexes(_pMesh, _params, RUN_MULTI_THREAD);

        updateTessellation();
    } catch (const char* errStr) {
        cout << errStr << "\n";
    }
}

void AppData::updateTessellation()
{
    const Index3D min(0, 0, 0);
    const Index3D max(_pVolume->volDim());
    Utils::Timer tmr0(Utils::Timer::TT_analyzeModelMesh);
    cout << "Tessellating graphics.\n";

    addHexFacesToScene(min, max, RUN_MULTI_THREAD);
    addHexEdgesToScene(min, max, RUN_MULTI_THREAD);

    setDisplayMinMax(min, max);

    auto pCanvas = _pMainFrame->getCanvas();
    pCanvas->changeViewElements();
}

void AppData::addHexFacesToScene(const Index3D& min, const Index3D& max, bool multiCore)
{
    Block::TriMeshGroup blockMeshes;
    auto pCanvas = _pMainFrame->getCanvas();
    _pVolume->makeFaceTris(blockMeshes, min, max, multiCore);

    auto pDraw = pCanvas->getDrawHexMesh();
    auto& faceVBO = pDraw->getVBOs()->_faceVBO;
    faceVBO.beginFaceTesselation();

    vector<vector<OGL::IndicesPtr>> faceTesselations;
    for (size_t mode = 0; mode < blockMeshes.size(); mode++) {
        auto& thisGroup = blockMeshes[mode];
        faceTesselations.push_back(vector<OGL::IndicesPtr>());
        faceTesselations.back().reserve(thisGroup.size());

        for (const auto& pBlockMesh : thisGroup) {
            if (pBlockMesh && pBlockMesh->numTris() > 0) {
                auto meshId = pBlockMesh->getId();
                auto changeNumber = pBlockMesh->getChangeNumber();
                const auto& points = pBlockMesh->getGlTriPoints();
                const auto& normals = pBlockMesh->getGlTriNormals(false);
                const auto& parameters = pBlockMesh->getGlTriParams();
                const auto& vertIndices = pBlockMesh->getGlTriIndices();
                auto pBlockTess = faceVBO.setFaceTessellation(meshId, changeNumber, points, normals, parameters, vertIndices);
                if (pBlockTess)
                    faceTesselations.back().push_back(pBlockTess);
            }
        }
    }

    pDraw->setFaceTessellations(faceTesselations);
    faceVBO.endFaceTesselation(false);
}

void AppData::addHexEdgesToScene(const Index3D& min, const Index3D& max, bool multiCore)
{
    Block::glPointsGroup edgeSets;
    auto pCanvas = _pMainFrame->getCanvas();
    _pVolume->makeEdgeSets(edgeSets, min, max, multiCore);

    auto pDraw = pCanvas->getDrawHexMesh();
    auto& edgeVBO = pDraw->getVBOs()->_edgeVBO;
    edgeVBO.beginEdgeTesselation();

    vector<vector<OGL::IndicesPtr>> edgeTesselations;
    edgeTesselations.reserve(edgeSets.size());
    for (size_t mode = 0; mode < edgeSets.size(); mode++) {
        edgeTesselations.push_back(vector<OGL::IndicesPtr>());
        for (const auto& faceEdgesPtr : edgeSets[mode]) {
            if (faceEdgesPtr) {
                const auto& faceEdges = *faceEdgesPtr;
                vector<unsigned int> indices;
                indices.reserve(faceEdges.size());
                for (size_t j = 0; j < faceEdges.size(); j++)
                    indices.push_back(j);
                if (!faceEdges.empty()) {
                    auto pEdgeTess = edgeVBO.setEdgeSegTessellation(faceEdgesPtr->getId(), faceEdgesPtr->changeNumber(), faceEdges, indices);
                    if (pEdgeTess)
                        edgeTesselations[mode].push_back(pEdgeTess);
                }
            }
        }
    }

    pDraw->setEdgeTessellations(edgeTesselations);
    edgeVBO.endEdgeTesselation();
}

