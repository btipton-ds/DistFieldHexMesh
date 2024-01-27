#include <wx/filedlg.h>
#include <wx/string.h>
#include <wx/wfstream.h>

#include <appData.h>
#include <tm_math.h>
#include <triMesh.h>
#include <readStl.h>
#include <MultiCoreUtil.h>

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

void AppData::getEdgeData(std::vector<float>& sharpPts, std::vector<int>& sharpIndices, std::vector<float>& normPts, std::vector<int>& normIndices) const
{
    sharpPts.clear();
    sharpIndices.clear();
    normPts.clear();
    normIndices.clear();

    const vector<size_t>& sharps = _pMesh->getSharpEdgeIndices(15 * M_PI / 180.0);
    if (!sharps.empty()) {
        for (size_t i : sharps) {
            const auto& edge = _pMesh->getEdge(i);
            const auto pt0 = _pMesh->getVert(edge._vertIndex[0]);
            const auto pt1 = _pMesh->getVert(edge._vertIndex[1]);

            for (int j = 0; j < 3; j++)
                sharpPts.push_back((float)pt0._pt[j]);

            for (int j = 0; j < 3; j++)
                sharpPts.push_back((float)pt1._pt[j]);

            sharpIndices.push_back((int)sharpIndices.size());
            sharpIndices.push_back((int)sharpIndices.size());
        }

    }

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
        Vector3d ptEnd = ctr + n.normalized() * charLen;

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
        "TriMesh files (*.stl)|*.stl", wxFD_OPEN | wxFD_FILE_MUST_EXIST);
    if (openFileDialog.ShowModal() == wxID_CANCEL)
        return;     // the user changed idea...

    // proceed loading the file chosen by the user;
    // this can be done with e.g. wxWidgets input streams:
    _workDirName = openFileDialog.GetDirectory();
    wxString pathStr = openFileDialog.GetPath();
    if (pathStr.find(".stl") != 0) {
        TriMesh::CMeshPtr pMesh = make_shared<TriMesh::CMesh>();
        CReadSTL reader(pMesh);

        wstring filename(openFileDialog.GetFilename().ToStdWstring());
        wstring path(pathStr.ToStdWstring());
        auto pos = path.find(filename);
        path = path.substr(0, pos);
        if (reader.read(path, filename)) {
            _pMesh = pMesh;
            auto pCanvas = _pMainFrame->getCanvas();

            pCanvas->beginFaceTesselation();
            auto faceTess = pCanvas->setFaceTessellation(_pMesh);
            pCanvas->endFaceTesselation(faceTess, false);

            vector<float> sharpPts, normPts;
            vector<int> sharpIndices, normIndices;
            getEdgeData(sharpPts, sharpIndices, normPts, normIndices);

            pCanvas->beginEdgeTesselation();
            const COglMultiVboHandler::OGLIndices* sharpEdgeTess = nullptr;
            const COglMultiVboHandler::OGLIndices* normEdgeTess = nullptr;

            if (!sharpPts.empty())
                sharpEdgeTess = pCanvas->setEdgeSegTessellation(_pMesh->getId(), _pMesh->getChangeNumber(), sharpPts, sharpIndices);

            if (!normPts.empty())
                normEdgeTess = pCanvas->setEdgeSegTessellation(_pMesh->getId() + 10000, _pMesh->getChangeNumber(), normPts, normIndices);

            pCanvas->endEdgeTesselation(sharpEdgeTess, normEdgeTess);
        }
    }
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

            const Index3D& faceIndices0 = _pMesh->getTri(edge._faceIndex[0]);
            const Index3D& faceIndices1 = _pMesh->getTri(edge._faceIndex[1]);

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


    MultiCore::runLambda([this, dim, bb](size_t threadNum, size_t numThreads) {
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
                Ray ray(ctr, zAxis);

                vector<RayHit> hits;
                if (_pMesh->rayCast(ray, hits)) {
                    int dbgBreak = 1;
                }
            }
        }
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

void AppData::makeBlock(const MakeBlockDlg& dlg)
{
    Volume::setVolDim(Index3D(2, 2, 2));
	Volume vol;
    vol.setOrigin(dlg.getBlockOrigin());
    vol.setSpan(dlg.getBlockSpan());
    
    auto pCanvas = _pMainFrame->getCanvas();
    pCanvas->beginFaceTesselation();
    auto triTess = pCanvas->setFaceTessellation(_pMesh);

    Block::TriMeshGroup blockMeshes;
    Block::glPointsGroup faceEdges;
    vol.addAllBlocks(blockMeshes, faceEdges);
    vector<vector<const COglMultiVboHandler::OGLIndices*>> faceTesselations;
    for (size_t mode = 0; mode < blockMeshes.size(); mode++) {
        faceTesselations.push_back(vector<const COglMultiVboHandler::OGLIndices*>());
        for (size_t i = 0; i < blockMeshes[mode].size(); i++) {
            auto pBlockMesh = blockMeshes[mode][i];
            auto pBlockTess = pCanvas->setFaceTessellation(pBlockMesh);
            if (pBlockTess)
                faceTesselations[mode].push_back(pBlockTess);
        }
    }

    pCanvas->endFaceTesselation(triTess, faceTesselations, false);

    pCanvas->beginSettingFaceElementIndices(0xffffffffffffffff);
    pCanvas->includeFaceElementIndices(0, *triTess);
    for (size_t mode = 0; mode <= faceTesselations.size(); mode++) {
        for (auto pBlockTess : faceTesselations[mode]) {
            if (pBlockTess)
                pCanvas->includeFaceElementIndices(mode + 1, *pBlockTess);
        }
    }
    pCanvas->endSettingFaceElementIndices();
}

void AppData::makeCylinderWedge(const MakeBlockDlg& dlg, bool isCylinder)
{
}

void AppData::doBuildCFDHexes()
{
    if (!_volume)
        _volume = make_shared<Volume>();

    Index3D::setBlockDim(8);

    double blockSize = 0.25;

    _volume->buildCFDHexes(_pMesh, blockSize);

    auto pCanvas = _pMainFrame->getCanvas();

    size_t minSplits = 0;
    addTriangles(pCanvas, minSplits);
    addFaceEdges(pCanvas, minSplits);
}

void AppData::addTriangles(GraphicsCanvas* pCanvas, size_t minSplitNum)
{
    Block::TriMeshGroup blockMeshes;
    _volume->makeTris(blockMeshes, minSplitNum, true);

    pCanvas->beginFaceTesselation();

    const COglMultiVboHandler::OGLIndices* triTess = pCanvas->setFaceTessellation(_pMesh);

    vector<vector<const COglMultiVboHandler::OGLIndices*>> faceTesselations;
    for (size_t mode = 0; mode < blockMeshes.size(); mode++) {
        auto& thisGroup = blockMeshes[mode];
        faceTesselations.push_back(vector<const COglMultiVboHandler::OGLIndices*>());
        faceTesselations.back().reserve(thisGroup.size());

        for (const auto& pBlockMesh : thisGroup) {
            if (pBlockMesh && pBlockMesh->numTris() > 0) {
                auto pBlockTess = pCanvas->setFaceTessellation(pBlockMesh);
                if (pBlockTess)
                    faceTesselations.back().push_back(pBlockTess);
            }
        }
    }
    pCanvas->endFaceTesselation(triTess, faceTesselations, false);
}

void AppData::addFaceEdges(GraphicsCanvas* pCanvas, size_t minSplitNum)
{
    Block::glPointsGroup faceEdgeSets;
    _volume->makeFaceEdges(faceEdgeSets, minSplitNum, true);

    pCanvas->beginEdgeTesselation();

    const COglMultiVboHandler::OGLIndices* sharpEdgeTess = nullptr;
    const COglMultiVboHandler::OGLIndices* normEdgeTess = nullptr;
    vector<float> sharpPts, normPts;
    vector<int> sharpIndices, normIndices;
    getEdgeData(sharpPts, sharpIndices, normPts, normIndices);
    if (!sharpPts.empty())
        sharpEdgeTess = pCanvas->setEdgeSegTessellation(_pMesh->getId(), _pMesh->getChangeNumber(), sharpPts, sharpIndices);

    if (!normPts.empty())
        normEdgeTess = pCanvas->setEdgeSegTessellation(_pMesh->getId() + 10000, _pMesh->getChangeNumber(), normPts, normIndices);

    vector<vector<const COglMultiVboHandler::OGLIndices*>> edgeTesselations;
    edgeTesselations.reserve(faceEdgeSets.size());
    for (size_t mode = 0; mode < faceEdgeSets.size(); mode++) {
        edgeTesselations.push_back(vector<const COglMultiVboHandler::OGLIndices*>());
        for (const auto& faceEdgesPtr : faceEdgeSets[mode]) {
            if (faceEdgesPtr) {
                const auto& faceEdges = *faceEdgesPtr;
                vector<int> indices;
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

    pCanvas->endEdgeTesselation(sharpEdgeTess, normEdgeTess, edgeTesselations);
}