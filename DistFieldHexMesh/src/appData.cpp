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

#define SHARP_EDGE_ANGLE (15 * M_PI / 180.0)

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
        "TriMesh files (*.stl)|*.stl", wxFD_OPEN | wxFD_FILE_MUST_EXIST);
    if (openFileDialog.ShowModal() == wxID_CANCEL)
        return;     // the user changed idea...


    // proceed loading the file chosen by the user;
    // this can be done with e.g. wxWidgets input streams:
    _workDirName = openFileDialog.GetDirectory();
    wxString pathStr = openFileDialog.GetPath();
    if (pathStr.find(".stl") != 0) {
        CMeshPtr pMesh = make_shared<CMesh>();
        CReadSTL reader(pMesh);

        wstring filename(openFileDialog.GetFilename().ToStdWstring());
        wstring path(pathStr.ToStdWstring());
        auto pos = path.find(filename);
        path = path.substr(0, pos);
        try {
            if (reader.read(path, filename)) {
                _pMesh = pMesh;
                _pMesh->squeezeSkinnyTriangles(0.1);
                _pMesh->buildCentroids();
                _pMesh->calCurvatures(SHARP_EDGE_ANGLE, false);
                vector<double> radii;
                radii.reserve(_pMesh->numEdges());
                for (size_t i = 0; i < _pMesh->numEdges(); i++) {
                    auto curv = _pMesh->edgeCurvature(i);
                    if (curv > 0)
                        radii.push_back(1 / curv);
                }
                sort(radii.begin(), radii.end());
                auto pCanvas = _pMainFrame->getCanvas();

                pCanvas->beginFaceTesselation(true);
                auto pSharpVertMesh = getSharpVertMesh();
                _modelFaceTess = pCanvas->setFaceTessellation(_pMesh);
                if (pSharpVertMesh)
                    _sharpPointTess = pCanvas->setFaceTessellation(pSharpVertMesh);
                pCanvas->endFaceTesselation(_modelFaceTess, false);

                vector<float> normPts;
                vector<unsigned int> normIndices;
                getEdgeData(normPts, normIndices);

                pCanvas->beginEdgeTesselation(true);

                _modelEdgeTess = pCanvas->setEdgeSegTessellation(_pMesh);

                if (!normPts.empty())
                    _modelNormalTess = pCanvas->setEdgeSegTessellation(_pMesh->getId() + 10000, _pMesh->getChangeNumber(), normPts, normIndices);

                pCanvas->endEdgeTesselation(_modelEdgeTess, _modelNormalTess);
            }
        } catch (const char* errStr) {
            cout << errStr << "\n";
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
                Ray ray(ctr, zAxis);

                vector<RayHit> hits;
                if (_pMesh->rayCast(ray, hits)) {
                    int dbgBreak = 1;
                }
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

        Volume::BuildCFDParams params;

        params.numSimpleDivs = 2;
        params.numCurvatureDivs = 8;
        params.divsPerRadius = 4;
        params.maxCurvatureRadius = 0.1; // 50 cm
        params.sharpAngleDegrees = SHARP_EDGE_ANGLE;

        _volume->buildCFDHexes(_pMesh, params);

        auto pCanvas = _pMainFrame->getCanvas();

        addFacesToScene(pCanvas);
        addEdgesToScene(pCanvas);
    } catch (const char* errStr) {
        cout << errStr << "\n";
    }
}

void AppData::addFacesToScene(GraphicsCanvas* pCanvas)
{
    Block::TriMeshGroup blockMeshes;
    _volume->makeFaceTris(blockMeshes, true);

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

void AppData::addEdgesToScene(GraphicsCanvas* pCanvas)
{
    Block::glPointsGroup edgeSets;
    _volume->makeEdgeSets(edgeSets, true);

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
    double radius = span / 100;
    bBox.grow(2 * radius);
    CMeshPtr pMesh = make_shared<CMesh>(bBox);

    if (_volume) {
        auto sVerts = _volume->getSharpVertIndices();
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
    size_t stepsI = 36;
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
