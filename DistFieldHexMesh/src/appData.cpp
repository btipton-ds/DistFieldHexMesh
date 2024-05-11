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

#include "defines.h"

#include <tm_ray.h>
#include <splitParams.h>
#include <appData.h>
#include <tm_math.h>
#include <triMesh.h>
#include <readStl.h>
#include <MultiCoreUtil.h>
#include <selectBlocksDlg.h>

#include <makeBlockDlg.h>
#include <mainFrame.h>
#include <graphicsCanvas.h>
#include <volume.h>
#include <vertex.h>

using namespace std;
using namespace DFHM;

AppData::AppData(MainFrame* pMainFrame)
    : _pMainFrame(pMainFrame)
{
}

void AppData::getEdgeData(std::vector<float>& normPts, std::vector<unsigned int>& normIndices) const
{
    normPts.clear();
    normIndices.clear();

    for (size_t triIdx = 0; triIdx < _pMesh->numTris(); triIdx++) {
        const Index3D& triIndices = _pMesh->getTri(triIdx);
        const auto pt0 = _pMesh->getVert(triIndices[0])._pt;
        const auto pt1 = _pMesh->getVert(triIndices[1])._pt;
        const auto pt2 = _pMesh->getVert(triIndices[2])._pt;

        Vector3d ctr = (pt0 + pt1 + pt2) / 3.0;
        Vector3d v0 = pt0 - pt1;
        Vector3d v1 = pt2 - pt1;
        Vector3d n = v1.cross(v0);
        double area = n.norm() / 2;
        double charLen = sqrt(area);
        Vector3d ptEnd = ctr + n.normalized() * 0.01;// *charLen;

        for (int j = 0; j < 3; j++)
            normPts.push_back((float)ctr[j]);

        for (int j = 0; j < 3; j++)
            normPts.push_back((float)ptEnd[j]);

        normIndices.push_back((int)normIndices.size());
        normIndices.push_back((int)normIndices.size());
    }

}

void AppData::doOpen()
{
    wxFileDialog openFileDialog(_pMainFrame, _("Open Triangle Mesh file"), "", "",
        "All (*.stl;*.dfhm)|*.stl;*.dfhm|TriMesh files (*.stl)|*.stl|DFHM files (*.dfhm)|*.dfhm", wxFD_OPEN | wxFD_FILE_MUST_EXIST);
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
    if (filename.find(L".stl") != -1) {
        readStl(path, filename);
    } else if (filename.find(L".dfhm") != -1) {
        readDHFM(path, filename);
    }
}

void AppData::readStl(const wstring& pathIn, const wstring& filename)
{
    CMeshPtr pMesh = make_shared<CMesh>();
    CReadSTL reader(pMesh);

    wstring path(pathIn);
    auto pos = path.find(filename);
    path = path.substr(0, pos);
    try {
        if (reader.read(path, filename)) {
            _pMesh = pMesh;
            postReadMesh();
        }
    }
    catch (const char* errStr) {
        cout << errStr << "\n";
    }

}

void AppData::postReadMesh()
{
    _pMesh->squeezeSkinnyTriangles(0.025);
    _pMesh->buildCentroids();
    _pMesh->calCurvatures(SHARP_EDGE_ANGLE_RADIANS, false);
    _pMesh->calGaps();

    auto pCanvas = _pMainFrame->getCanvas();

    pCanvas->beginFaceTesselation(true);
    auto pSharpVertMesh = getSharpVertMesh();
    _modelFaceTess = pCanvas->setFaceTessellation(_pMesh);
    if (pSharpVertMesh)
        _sharpPointTess = pCanvas->setFaceTessellation(pSharpVertMesh);
    pCanvas->endFaceTesselation(_modelFaceTess, _sharpPointTess, false);

    vector<float> normPts;
    vector<unsigned int> normIndices;
    getEdgeData(normPts, normIndices);

    pCanvas->beginEdgeTesselation(true);

    _modelEdgeTess = pCanvas->setEdgeSegTessellation(_pMesh);

    if (!normPts.empty())
        _modelNormalTess = pCanvas->setEdgeSegTessellation(_pMesh->getId() + 10000, _pMesh->getChangeNumber(), normPts, normIndices);

    pCanvas->endEdgeTesselation(_modelEdgeTess, _modelNormalTess);
}

void AppData::readDHFM(const std::wstring& path, const std::wstring& filename)
{
    _dhfmFilename = path + filename;

    ifstream in(filesystem::path(_dhfmFilename), ifstream::binary);

    uint8_t version;
    in.read((char*)&version, sizeof(version));

    bool hasMesh;
    in.read((char*)&hasMesh, sizeof(hasMesh));
    if (hasMesh) {
        _pMesh = make_shared<TriMesh::CMesh>();
        _pMesh->read(in);
        postReadMesh();
    }

    bool hasVolume;
    in.read((char*)&hasVolume, sizeof(hasVolume));
    if (hasVolume) {
        _volume = make_shared<Volume>();
        _volume->setModelMesh(_pMesh);

        _volume->read(in);
        updateTessellation(Index3D(0, 0, 0), Volume::volDim());
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

    uint8_t version = 0;
    out.write((char*)&version, sizeof(version));

    bool hasMesh = _pMesh != nullptr;
    out.write((char*)&hasMesh, sizeof(hasMesh));
    if (_pMesh)
        _pMesh->write(out);

    bool hasVolume = _volume != nullptr;
    out.write((char*)&hasVolume, sizeof(hasVolume));
    if (hasVolume)
        _volume->write(out);    
}

void AppData::doVerifyClosed()
{
    int numOpen = _pMesh->numLaminarEdges();

    stringstream ss;
    ss << "Number of edges: " << _pMesh->numEdges() << "\nNumber of open edges: " << numOpen;
    wxMessageBox(ss.str().c_str(), "Verify Closed", wxOK | wxICON_INFORMATION);
}

void AppData::doVerifyNormals()
{
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
}

void AppData::doAnalyzeGaps()
{
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
}

void AppData::doFindMinGap() const
{
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
                Ray<double> ray(ctr, zAxis);

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
    Index3D min = dlg.getMin();
    Index3D max = dlg.getMax();

    auto pCanvas = _pMainFrame->getCanvas();
    pCanvas->clearMesh3D();
    pCanvas->setShowSelectedBlocks(true);

    updateTessellation(min, max);
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
    auto triTess = pCanvas->setFaceTessellation(_pMesh);

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

void AppData::doBuildCFDHexes()
{
    try {
        if (!_volume)
            _volume = make_shared<Volume>();

        BuildCFDParams params;

        params.uniformRatio = false;
        params.minBlocksPerSide = 6; // def = 6
        params.numBlockDivs = 0;
        params.numSimpleDivs = 0;
        params.numCurvatureDivs = 5;
        params.divsPerCurvatureRadius = 3;
        params.divsPerGapCurvatureRadius = 6;
        params.maxGapSize = 0.02;
        params.minSplitEdgeLengthCurvature_meters = 0.0025;
        params.minSplitEdgeLengthGapCurvature_meters = params.minSplitEdgeLengthGapCurvature_meters / 4;
        params.minSplitEdgeLengthSharpVertex_meters = 0.001;
        params.sharpAngle_degrees = SHARP_EDGE_ANGLE_RADIANS;
        params.maxCellFaces = 12;


        _volume->buildCFDHexes(_pMesh, params, RUN_MULTI_THREAD);
        updateTessellation(Index3D(0, 0, 0), Volume::volDim());
    } catch (const char* errStr) {
        cout << errStr << "\n";
    }
}

void AppData::updateTessellation(const Index3D& min, const Index3D& max)
{
#ifdef _WIN32
    LARGE_INTEGER startCount, freq;
    QueryPerformanceFrequency(&freq);
    QueryPerformanceCounter(&startCount);
#endif // _WIN32

    cout << "Tessellating graphics.\n";

    auto pCanvas = _pMainFrame->getCanvas();

    addFacesToScene(pCanvas, min, max, RUN_MULTI_THREAD);
    addEdgesToScene(pCanvas, min, max, RUN_MULTI_THREAD);

    setDisplayMinMax(min, max);

#ifdef _WIN32
    LARGE_INTEGER endCount;
    QueryPerformanceCounter(&endCount);
    double deltaT = (endCount.QuadPart - startCount.QuadPart) / (double)(freq.QuadPart);
    cout << "Time for updateTessellation: " << deltaT << " secs\n";
#endif // _WIN32

}

void AppData::addFacesToScene(GraphicsCanvas* pCanvas, const Index3D& min, const Index3D& max, bool multiCore)
{
    Block::TriMeshGroup blockMeshes;
    _volume->makeFaceTris(blockMeshes, min, max, multiCore);

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
    _volume->makeEdgeSets(edgeSets, min, max, multiCore);

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

CMeshPtr AppData::getSharpVertMesh() const
{
    auto bBox = _pMesh->getBBox();
    double span = bBox.range().norm();
    double radius = span / 500;
    bBox.grow(2 * radius);

    vector<size_t> sVerts;
    Volume::findSharpVertices(_pMesh, SHARP_EDGE_ANGLE_RADIANS, sVerts);
    if (!sVerts.empty()) {
        CMeshPtr pMesh = make_shared<CMesh>(bBox);
        for (size_t vertIdx : sVerts) {
            auto pt = _pMesh->getVert(vertIdx)._pt;
            addPointMarker(pMesh, pt, radius);
        }

        return pMesh;
    }

    return nullptr;
}

void AppData::addPointMarker(CMeshPtr& pMesh, const Vector3d& origin, double radius) const
{
    Vector3d xAxis(radius, 0, 0), yAxis(0, radius, 0), zAxis(0, 0, radius);
    size_t stepsI = 72;
    size_t stepsJ = stepsI / 2;
    for (size_t i = 0; i < stepsI; i++) {
        double alpha0 = 2 * M_PI * i / (double)stepsI;
        double alpha1 = 2 * M_PI * (i + 1) / (double)stepsI;

        for (size_t j = 0; j < stepsJ; j++) {
            double phi0 = M_PI * (-0.5 + j / (double)stepsJ);
            double phi1 = M_PI * (-0.5 + (j + 1) / (double)stepsJ);

            Vector3d pt00 = origin + cos(phi0) * (cos(alpha0) * xAxis + sin(alpha0) * yAxis) + sin(phi0) * zAxis;
            Vector3d pt01 = origin + cos(phi0) * (cos(alpha1) * xAxis + sin(alpha1) * yAxis) + sin(phi0) * zAxis;
            Vector3d pt10 = origin + cos(phi1) * (cos(alpha0) * xAxis + sin(alpha0) * yAxis) + sin(phi1) * zAxis;
            Vector3d pt11 = origin + cos(phi1) * (cos(alpha1) * xAxis + sin(alpha1) * yAxis) + sin(phi1) * zAxis;
            if (j == 0) {
                pMesh->addTriangle(pt00, pt11, pt10);
            } else if (j == stepsJ - 1) {
                pMesh->addTriangle(pt00, pt01, pt11);
            } else {
                pMesh->addTriangle(pt00, pt01, pt11);
                pMesh->addTriangle(pt00, pt11, pt10);
            }
        }
    }
}
