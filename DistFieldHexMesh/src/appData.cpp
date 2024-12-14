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

#include <tm_ray.h>
#include <splitParams.h>
#include <appData.h>
#include <tm_math.h>
#include <triMesh.h>
#include <readWriteStl.h>
#include <MultiCoreUtil.h>
#include <selectBlocksDlg.h>
#include <buildCFDHexesDlg.h>

#include <splitParams.h>
#include <meshData.h>
#include <makeBlockDlg.h>
#include <mainFrame.h>
#include <graphicsCanvas.h>
#include <volume.h>
#include <vertex.h>
#include <utils.h>

using namespace std;
using namespace DFHM;

AppData::AppData(MainFrame* pMainFrame)
    : _pMainFrame(pMainFrame)
{
}

void AppData::doOpen()
{
    wxFileDialog openFileDialog(_pMainFrame, _("Open Triangle Mesh file"), "", "",
        "All (*.dfhm)|*.dfhm|DFHM files (*.dfhm)|*.dfhm", wxFD_OPEN | wxFD_FILE_MUST_EXIST);
    if (openFileDialog.ShowModal() == wxID_CANCEL)
        return;     // the user changed idea...


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
    }
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
    if (filename.find(L".stl") != -1) {
        auto pMesh = readStl(path, filename);
        auto pos = filename.find(L".");
        wstring name = filename.replace(pos, filename.size(), L"");
        MeshDataPtr pMeshData = make_shared<MeshData>(pMesh, name, _pMainFrame->getCanvas()->getViewOptions());

        _pMainFrame->registerMeshData(pMeshData);
        pMeshData->makeOGLTess();
        _meshData.insert(make_pair(pMeshData->getName(), pMeshData));

        _pMainFrame->refreshObjectTree();

        _pMainFrame->getCanvas()->resetView();
        return true;
    }
    return false;
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

    uint8_t version = 1;
    out.write((char*)&version, sizeof(version));

    size_t numMeshes = _meshData.size();
    out.write((char*)&numMeshes, sizeof(numMeshes));

    for (const auto& pair : _meshData) {
        const auto& name = pair.first;
        const auto& pData = pair.second;
        size_t numChars = name.size();
        out.write((char*)&numChars, sizeof(numChars));
        out.write((char*)name.c_str(), numChars * sizeof(wchar_t));
        pData->getMesh()->write(out);
    }

    bool hasVolume = _pVolume != nullptr;
    out.write((char*)&hasVolume, sizeof(hasVolume));
    if (hasVolume)
        _pVolume->write(out);    
}

void AppData::readDHFM(const std::wstring& path, const std::wstring& filename)
{
    _dhfmFilename = path + filename;

    ifstream in(filesystem::path(_dhfmFilename), ifstream::binary);

    uint8_t version;
    in.read((char*)&version, sizeof(version));
    if (version == 0) {

        bool hasMesh;
        in.read((char*)&hasMesh, sizeof(hasMesh));
        if (hasMesh) {
            CMeshPtr pMesh = make_shared<CMesh>();
            pMesh->read(in);
            wstring name(L"default");
            MeshDataPtr pData = make_shared<MeshData>(pMesh, name, _pMainFrame->getCanvas()->getViewOptions());
            _pMainFrame->registerMeshData(pData);
            pData->makeOGLTess();
            _meshData.insert(make_pair(name, pData));
        }

    } else if (version == 1) {
        size_t numMeshes;
        in.read((char*)&numMeshes, sizeof(numMeshes));

        for (size_t i = 0; i < numMeshes; i++) {
            wchar_t buf[1024];
            for (size_t j = 0; j < 1024; j++)
                buf[j] = 0;
            size_t numChars;

            in.read((char*)&numChars, sizeof(numChars));
            in.read((char*)buf, numChars * sizeof(wchar_t));
            CMeshPtr pMesh = make_shared<CMesh>();
            pMesh->read(in);

            wstring name(buf);
            MeshDataPtr pData = make_shared<MeshData>(pMesh, name, _pMainFrame->getCanvas()->getViewOptions());
            _pMainFrame->registerMeshData(pData);
            pData->makeOGLTess();
            _meshData.insert(make_pair(name, pData));
        }

    }

    bool hasVolume;
    in.read((char*)&hasVolume, sizeof(hasVolume));
    if (hasVolume) {
        _pVolume = make_shared<Volume>();
        //        _pVolume->setModelMesh(_pMesh);

        _pVolume->read(in);
        updateTessellation(Index3D(0, 0, 0), Volume::volDim());
    }

    _pMainFrame->refreshObjectTree();

    _pMainFrame->getCanvas()->resetView();
}

void AppData::doVerifyClosed()
{
    /*
    int numOpen = _pMesh->numLaminarEdges();

    stringstream ss;
    ss << "Number of edges: " << _pMesh->numEdges() << "\nNumber of open edges: " << numOpen;
    wxMessageBox(ss.str().c_str(), "Verify Closed", wxOK | wxICON_INFORMATION);
    */
}

void AppData::doVerifyNormals()
{
    /*
    size_t numMisMatched = 0;
    size_t numEdges = _pMesh->numEdges();
    for (size_t i = 0; i < numEdges; i++) {
        const auto& edge = _pMesh->getEdge(i);
        if (edge._numFaces == 2) {
            size_t ptIdx0 = edge._vertIndex[0];
            size_t ptIdx1 = edge._vertIndex[1];

            const Index3D& faceIndices0 = _pMesh->getTri(edge._faceIndices[0]);
            const Index3D& faceIndices1 = _pMesh->getTri(edge._faceIndices[1]);

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
    ss << "Number of tris: " << _pMesh->numTris() << "\nNumber of opposed faces: " << numMisMatched;
    wxMessageBox(ss.str().c_str(), "Verify Normals", wxOK | wxICON_INFORMATION);
    */
}

void AppData::doAnalyzeGaps()
{
    /*
    vector<double> binSizes({ 0.050 / 64, 0.050 / 32, 0.050 / 16, 0.050 / 8, 0.050 / 4, 0.050 / 2, 0.050 });
    vector<size_t> bins;
    bins.resize(binSizes.size(), 0);
    _pMesh->getGapHistogram(binSizes, bins, true);

    stringstream ss;
    ss << "Gap histogram\n";
    for (size_t i = 0; i < binSizes.size(); i++) {
        ss << "hits < " << binSizes[i] << ": " << bins[i] << "\n";
    }
    wxMessageBox(ss.str().c_str(), "Gap Analysis", wxOK | wxICON_INFORMATION);
    */
}

void AppData::doFindMinGap() const
{
    /*
    double t = _pMesh->findMinGap() * 10;
    auto bb = _pMesh->getBBox();
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
    */
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
    Index3D min = dlg.getMin();
    Index3D max = dlg.getMax();

    auto pCanvas = _pMainFrame->getCanvas();
    pCanvas->clearMesh3D();
    pCanvas->setShowSelectedBlocks(true);

    updateTessellation(min, max);
}

CBoundingBox3Dd AppData::getBoundingBox() const
{
    CBoundingBox3Dd result;
    for (const auto& pair : _meshData) {
        result.merge(pair.second->getMesh()->getBBox());
    }
    if (_pVolume)
        result.merge(_pVolume->getBBox());

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
    Volume::setVolDim(Index3D(2, 2, 2));
	Volume vol;
    vol.setOrigin(dlg.getBlockOrigin());
    vol.setSpan(dlg.getBlockSpan());
    
    auto pCanvas = _pMainFrame->getCanvas();
    pCanvas->beginFaceTesselation(true);

    Block::TriMeshGroup blockMeshes;
    Block::glPointsGroup faceEdges;
    vol.addAllBlocks(blockMeshes, faceEdges);
    vector<vector<const OGLIndices*>> faceTesselations;
    for (size_t mode = 0; mode < blockMeshes.size(); mode++) {
        faceTesselations.push_back(vector<const OGLIndices*>());
        for (size_t i = 0; i < blockMeshes[mode].size(); i++) {
            auto pBlockMesh = blockMeshes[mode][i];
            auto pBlockTess = pCanvas->setFaceTessellation(pBlockMesh);
            if (pBlockTess)
                faceTesselations[mode].push_back(pBlockTess);
        }
    }

    pCanvas->endFaceTesselation(faceTesselations);
}

void AppData::makeCylinderWedge(const MakeBlockDlg& dlg, bool isCylinder)
{
}

void AppData::doBuildCFDHexes(const BuildCFDHexesDlg& dlg)
{
    try {
        auto pCanvas = _pMainFrame->getCanvas();
        pCanvas->clearMesh3D();
        _pVolume = make_shared<Volume>();

        dlg.getParams(_params);

//        _pVolume->buildCFDHexes(_pMesh, _params, RUN_MULTI_THREAD);

        updateTessellation(Index3D(0, 0, 0), Volume::volDim());
    } catch (const char* errStr) {
        cout << errStr << "\n";
    }
}

void AppData::updateTessellation(const Index3D& min, const Index3D& max)
{
    Utils::Timer tmr0(Utils::Timer::TT_analyzeModelMesh);
    cout << "Tessellating graphics.\n";

    auto pCanvas = _pMainFrame->getCanvas();

    addFacesToScene(pCanvas, min, max, RUN_MULTI_THREAD);
    addEdgesToScene(pCanvas, min, max, RUN_MULTI_THREAD);

    setDisplayMinMax(min, max);
}

void AppData::addFacesToScene(GraphicsCanvas* pCanvas, const Index3D& min, const Index3D& max, bool multiCore)
{
    Block::TriMeshGroup blockMeshes;
    _pVolume->makeFaceTris(blockMeshes, min, max, multiCore);

    pCanvas->beginFaceTesselation(false);

    vector<vector<const OGLIndices*>> faceTesselations;
    for (size_t mode = 0; mode < blockMeshes.size(); mode++) {
        auto& thisGroup = blockMeshes[mode];
        faceTesselations.push_back(vector<const OGLIndices*>());
        faceTesselations.back().reserve(thisGroup.size());

        for (const auto& pBlockMesh : thisGroup) {
            if (pBlockMesh && pBlockMesh->numTris() > 0) {
                auto pBlockTess = pCanvas->setFaceTessellation(pBlockMesh);
                if (pBlockTess)
                    faceTesselations.back().push_back(pBlockTess);
            }
        }
    }
    pCanvas->endFaceTesselation(faceTesselations);
}

void AppData::addEdgesToScene(GraphicsCanvas* pCanvas, const Index3D& min, const Index3D& max, bool multiCore)
{
    Block::glPointsGroup edgeSets;
    _pVolume->makeEdgeSets(edgeSets, min, max, multiCore);

    pCanvas->beginEdgeTesselation(false);

    vector<vector<const OGLIndices*>> edgeTesselations;
    edgeTesselations.reserve(edgeSets.size());
    for (size_t mode = 0; mode < edgeSets.size(); mode++) {
        edgeTesselations.push_back(vector<const OGLIndices*>());
        for (const auto& faceEdgesPtr : edgeSets[mode]) {
            if (faceEdgesPtr) {
                const auto& faceEdges = *faceEdgesPtr;
                vector<unsigned int> indices;
                indices.reserve(faceEdges.size());
                for (size_t j = 0; j < faceEdges.size(); j++)
                    indices.push_back(j);
                if (!faceEdges.empty()) {
                    auto pEdgeTess = pCanvas->setEdgeSegTessellation(faceEdgesPtr->getId(), faceEdgesPtr->changeNumber(), faceEdges, indices);
                    if (pEdgeTess)
                        edgeTesselations[mode].push_back(pEdgeTess);
                }
            }
        }
    }

    pCanvas->endEdgeTesselation(edgeTesselations);
}

