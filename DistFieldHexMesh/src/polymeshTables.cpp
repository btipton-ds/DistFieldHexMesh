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

#include <polyMeshTables.h>
#include <volume.h>
#include <appData.h>

using namespace std;
using namespace DFHM;

#define WRITE_DEBUG_FILES 0

PolymeshTables::PolymeshTables(const Volume* pVol, ProgressReporter* pReporter)
	: _pVol(pVol)
	, _pProgReporter(pReporter)
{
	for (int i = 0; i < 6; i++) {
		boundaryIndices[i] = -1;
	}
}

void PolymeshTables::reportProgress() const
{
	if (_pProgReporter)
		_pProgReporter->reportProgress(1);
}

void PolymeshTables::create()
{
	if (_pProgReporter)
		_pProgReporter->startProgress(9);

	reportProgress();

	createMaps();
	reportProgress();


	createSortPolygons();
	reportProgress();

	createPolygonTables();
	reportProgress();
}

void PolymeshTables::writeFile(const std::string& dirName) const
{

	// These can all be done in parallel
	auto f1 = std::async(std::launch::async, [this, dirName]()->bool {
		writePoints(dirName);
		return true;
		});

	auto f2 = std::async(std::launch::async, [this, dirName]()->bool {
		writeFaces(dirName);
		return true;
		});

	auto f3 = std::async(std::launch::async, [this, dirName]()->bool {
		writeOwnerCells(dirName);
		return true;
		});

	auto f4 = std::async(std::launch::async, [this, dirName]()->bool {
		writeNeighborCells(dirName);
		writeBoundaries(dirName);
		return true;
		});

	// Wait for them fastest to slowest
	f4.wait();
	reportProgress();
	f3.wait();
	reportProgress();
	f2.wait();
	reportProgress();
	f1.wait();
	reportProgress();
}

void PolymeshTables::createMaps()
{
	for (size_t i = 0; i < _pVol->_blocks.size(); i++) {
		const shared_ptr<const Block>& pBlk = _pVol->_blocks[i];
		if (pBlk) {
			pBlk->_polyhedra.iterateInOrder([this, &pBlk](const Index3DId& id, const Polyhedron& cell) {
				int32_t cIdx = (int32_t)cellIdxIdMap.size();
				cellIdxIdMap.push_back(id);
				cellIdIdxMap.insert(make_pair(id, cIdx));

				const auto& faceIds = cell.getFaceIds();
				for (const auto& faceId : faceIds) {
					if (pBlk->polygonExists(faceId)) {
						auto faceIter = faceIdIdxMap.find(faceId);
						if (faceIter == faceIdIdxMap.end()) {
							int32_t newFaceIdx = faceIdxIdMap.size();
							faceIdIdxMap.insert(make_pair(faceId, newFaceIdx));
							faceIdxIdMap.push_back(faceId);

							const auto& face = pBlk->getPolygon(faceId);
							const auto& vertIds = face.getNonColinearVertexIds();
							for (const auto& vertId : vertIds) {
								auto vertIter = vertIdIdxMap.find(vertId);
								if (vertIter == vertIdIdxMap.end()) {
									int32_t newVertIdx = vertIdIdxMap.size();
									vertIdIdxMap.insert(make_pair(vertId, newVertIdx));
									vertIdxIdMap.push_back(vertId);
								}

							}
						}
					}
				}
			});
		}
	}
}

void PolymeshTables::createSortPolygons()
{
	std::vector<Planed> planes;
	_pVol->getModelBoundaryPlanes(planes);

	for (auto pBlk : _pVol->_blocks) {
		if (pBlk) {
			pBlk->_polygons.iterateInOrder([this, &planes](const Index3DId& id, const Polygon& face) {
				const auto& cellIds = face.getCellIds();
				SearchRec r;
				r.faceIdx = faceIdIdxMap[id];
				r.nCells = cellIds.size();
				r.ownerIdx = getFaceOwnerIdx(cellIds);
				if (r.nCells > 1) {
					r.neighborIdx = getFaceNeighbourIdx(cellIds);
				}
				else {
					for (int i = 0; i < 6; i++) {
						if (face.isCoplanar(planes[i])) {
							r.coplanarIdx = i;
						}
					}
				}

				faceIdToSearchRecMap.insert(id, r);
				});
		}
	}

	sort(faceIdxIdMap.begin(), faceIdxIdMap.end(), [this, &planes](const Index3DId& lhsFaceId, const Index3DId& rhsFaceId) {
		const auto& lhsSR = faceIdToSearchRecMap[lhsFaceId];
		const auto& rhsSR = faceIdToSearchRecMap[rhsFaceId];

		if (lhsSR.nCells == 1 && rhsSR.nCells == 1) {

			if (lhsSR.coplanarIdx < rhsSR.coplanarIdx)
				return true;
			else if (lhsSR.coplanarIdx > rhsSR.coplanarIdx)
				return false;

			return lhsSR.faceIdx < rhsSR.faceIdx;
		}
		else if (lhsSR.nCells > rhsSR.nCells)
			return true; // Reverse order by size
		else if (lhsSR.nCells < rhsSR.nCells)
			return false; // Reverse order by size

		if (lhsSR.ownerIdx < rhsSR.ownerIdx)
			return true;
		else if (lhsSR.ownerIdx > rhsSR.ownerIdx)
			return false;

		return lhsSR.neighborIdx < rhsSR.neighborIdx;
	});

}

void PolymeshTables::createPolygonTables()
{
	const auto& blocks = _pVol->_blocks;

	// Cell order determines the order of shared faces. Create it first.

	numInner = 0;
	for (int i = 0; i < 6; i++)
		boundaryIndices[i] = INT32_MAX;

	for (size_t i = 0; i < faceIdxIdMap.size(); i++) {
		const auto& faceId = faceIdxIdMap[i];
		const auto& r = faceIdToSearchRecMap[faceId];
		if (r.nCells > 1) {
			numInner++;
		} else if (r.coplanarIdx != -1) {
			if (i < boundaryIndices[r.coplanarIdx])
				boundaryIndices[r.coplanarIdx] = (int32_t)i;			
		}
	}

	int32_t idx = 0;
	faceIndices.reserve(faceIdxIdMap.size() + 1);
	vertIndices.reserve(4 * faceIdxIdMap.size());

	for (size_t i = 0; i < faceIdxIdMap.size(); i++) {
		const auto& face = _pVol->getPolygon(faceIdxIdMap[i]);
		const auto verts = face.getNonColinearVertexIds();
		vector<int32_t> faceVertIndices;
		for (const auto& vId : verts) {
			int32_t vIdx = vertIdIdxMap[vId];
			assert(vertIdxIdMap[vIdx] == vId);
			faceVertIndices.push_back(vIdx);
		}

		reverseFaceIfNeeded(face.getCellIds(), faceVertIndices);
		vertIndices.insert(vertIndices.end(), faceVertIndices.begin(), faceVertIndices.end());

		faceIndices.push_back(idx);


		idx += (int32_t)faceVertIndices.size();
	}
	faceIndices.push_back(idx); // Append the index of the start of the next face, even though that face doesn't exist.

}

int PolymeshTables::getFaceNeighbourIdx(const FastBisectionSet<Index3DId>& cellIds) const
{
	if (cellIds.size() == 2) {
		int32_t maxCellIdx = -1;
		for (const auto& cellId : cellIds) {
			auto iter = cellIdIdxMap.find(cellId);
			if (iter != cellIdIdxMap.end()) {
				auto cellIdx = iter->second;
				if (cellIdx > maxCellIdx)
					maxCellIdx = cellIdx;
			}
		}
		return maxCellIdx;
	}

	return -1;
}

int PolymeshTables::getFaceOwnerIdx(const FastBisectionSet<Index3DId>& cellIds) const
{
	int32_t minCellIdx = INT_MAX;
	for (const auto& cellId : cellIds) {
		auto iter = cellIdIdxMap.find(cellId);
		if (iter != cellIdIdxMap.end()) {
			auto cellIdx = iter->second;
			if (cellIdx < minCellIdx)
				minCellIdx = cellIdx;
		}
	}

	return minCellIdx;
}

#ifdef _WIN32
#define FMT_SIZE "%llu\n"
#else
#define FMT_SIZE "%lu\n"
#endif

void PolymeshTables::writePoints(const string& dirName) const
{
	string filename = dirName + "/points";
	FILE* fOut = fopen(filename.c_str(), "wb");
	try {
		vector<double> vals;
		vals.resize(3 * vertIdxIdMap.size());
		size_t idx = 0;
		for (size_t i = 0; i < vertIdxIdMap.size(); i++) {
			const auto& vertId = vertIdxIdMap[i];
			const auto& pt = _pVol->getVertex(vertId).getPoint();

			vals[idx++] = pt[0];
			vals[idx++] = pt[1];
			vals[idx++] = pt[2];
		}

		writeFOAMHeader(fOut, "binary", "vectorField", "points");

		fprintf(fOut, FMT_SIZE, vertIdxIdMap.size());
		fprintf(fOut, "(");
		fwrite(vals.data(), sizeof(double), vals.size(), fOut);
		fprintf(fOut, ")\n");
		fprintf(fOut, "//**********************************************************************************//\n");
#if WRITE_DEBUG_FILES
		{
			ofstream out(filename + ".csv");
			size_t nPts = vals.size() / 3;
			for (size_t i = 0; i < nPts; i++) {
				out << vals[3 * i + 0] << "," << vals[3 * i + 1] << "," << vals[3 * i + 2] << "\n";
			}
		}
#endif
	} catch (const std::runtime_error& err) {
		cout << "Exception thrown: " << err.what() << "\n";
	}
	fclose(fOut);
}

void PolymeshTables::reverseFaceIfNeeded(const FastBisectionSet<Index3DId>& cellIds, vector<int32_t>& faceVertIds)
{
	bool needToReverse = false;
	Vector3d faceCtr;
	vector<Vector3d> pts;
	for (size_t i = 0; i < faceVertIds.size(); i++) {
		const auto& vertId = vertIdxIdMap[faceVertIds[i]];
		const auto& pt = _pVol->getVertex(vertId).getPoint();
		faceCtr += pt;
		pts.push_back(pt);
	}

	faceCtr /= pts.size();
	Vector3d v0 = pts[0] - pts[1];
	Vector3d v1 = pts[2] - pts[1];
	Vector3d n = v1.cross(v0).normalized();
#ifdef _DEBUG
	Vector3d v = pts[3] - pts[0];
	double dp = v.dot(n);
	assert(fabs(dp) < 1.0e-6);
#endif // _DEBUG

	Vector3d cellCtr;
	if (cellIds.size() == 1) {
		const Index3DId cellId = cellIds[0];
		const auto& cell = _pVol->getPolyhedron(cellId);
		cellCtr = cell.calCentroidApprox();
		Vector3d v = (faceCtr - cellCtr).normalized();
		if (v.dot(n) < 0)
			needToReverse = true;
	} else {
		assert(cellIds.size() == 2);
		int maxCellIdx = -1;
		for (const auto& cellId : cellIds) {
			int cellIdx = cellIdIdxMap[cellId];
			if (maxCellIdx < cellIdx) {
				maxCellIdx = cellIdx;
			}
		}
		const auto& cell = _pVol->getPolyhedron(cellIdxIdMap[maxCellIdx]);
		cellCtr = cell.calCentroidApprox();

		Vector3d v = (faceCtr - cellCtr).normalized();
		if (v.dot(n) > 0)
			needToReverse = true;
	}

	if (needToReverse) {
		auto iter0 = faceVertIds.begin();
		auto iter1 = faceVertIds.end();
		std::reverse(iter0, iter1);
	}
}

void PolymeshTables::writeFaces(const string& dirName) const
{
	string filename = dirName + "/faces";
	FILE* fOut = fopen(filename.c_str(), "wb");
	try {

		// Write poly index table
		// An extra entry is required to compute the number of vertices in the last face
		// So, this is actually #faces + 1, not #faces.

		writeFOAMHeader(fOut, "binary", "faceCompactList", "faces");
		fprintf(fOut, FMT_SIZE, faceIndices.size());
		fprintf(fOut, "(");
		fwrite(faceIndices.data(), sizeof(int32_t), faceIndices.size(), fOut);
		fprintf(fOut, ")\n");

		fprintf(fOut, "\n");

		fprintf(fOut, FMT_SIZE, vertIndices.size());
		fprintf(fOut, "(");
		fwrite(vertIndices.data(), sizeof(int32_t), vertIndices.size(), fOut);
		fprintf(fOut, ")\n");

		fprintf(fOut, "//**********************************************************************************//\n");

#if WRITE_DEBUG_FILES
		{
			ofstream out(filename + ".csv");
			out << "Face indices\n";
			for (size_t i = 0; i < faceIndices.size(); i++) {
				out << faceIndices[i] << "\n";
			}
			out << "Vert indices\n";
			for (size_t i = 0; i < faceIndices.size() - 1; i++) {
				int32_t idx0 = faceIndices[i];
				int32_t idx1 = faceIndices[i + 1];
				for (int32_t j = idx0; j < idx1; j++) {
					out << vertIndices[j] << " ";
				}
				out << "\n";
			}
		}
		{
			ofstream out(dirName + "/fat_polys.csv");
			for (size_t i = 0; i < faceIndices.size() - 1; i++) {
				int32_t idx0 = faceIndices[i];
				int32_t idx1 = faceIndices[i + 1];
				out << "face " << i << " {\n";
				for (int32_t j = idx0; j < idx1; j++) {
					const auto& vertId = vertIdxIdMap[vertIndices[j]];
					const auto& pt = _pVol->getVertex(vertId).getPoint();
					out << "  (" << pt[0] << " " << pt[1] << " " << pt[2] << "),\n";
				}
				out << "}\n";
			}
		}
		{
			ofstream out(dirName + "/faces.obj");
			size_t numPts = vertIdxIdMap.size();
			for (size_t i = 0; i < numPts; i++) {
				const auto& vertId = vertIdxIdMap[i];
				const auto& pt = _pVol->getVertex(vertId).getPoint();
				out << "v " << pt[0] << " " << pt[1] << " " << pt[2] << "\n";
			}
			size_t numFaces = faceIndices.size() - 1;
			for (size_t i = 0; i < numFaces; i++) {
				int32_t idx0 = faceIndices[i];
				int32_t idx1 = faceIndices[i + 1];
				out << "f";
				for (size_t j = idx0; j < idx1; j++) {
					int32_t vertIdx = vertIndices[j];
					out << " " << (vertIdx + 1);
				}
				out << "\n";
			}
		}
#endif
	}	catch (const std::runtime_error& err) {
		cout << "Exception thrown: " << err.what() << "\n";
	}

	fclose(fOut);
}

void PolymeshTables::writeOwnerCells(const std::string& dirName) const
{

	string filename = dirName + "/owner";
	FILE* fOut = fopen(filename.c_str(), "wb");
	try {
		vector<int> indices;
		indices.reserve(faceIdxIdMap.size());
		for (const auto& faceId : faceIdxIdMap) {
			const auto& face = _pVol->getPolygon(faceId);
			const auto& cellIds = face.getCellIds();
			int oIdx = getFaceOwnerIdx(cellIds);
			indices.push_back(oIdx);
		}

		writeFOAMHeader(fOut, "binary", "labelList", "owner");
		fprintf(fOut, FMT_SIZE, indices.size());
		fprintf(fOut, "(");
		fwrite(indices.data(), sizeof(int), indices.size(), fOut);
		fprintf(fOut, ")\n");
		fprintf(fOut, "//**********************************************************************************//\n");

#if WRITE_DEBUG_FILES
		{
			ofstream out(filename + ".csv");
			for (size_t i = 0; i < indices.size(); i++) {
				out << indices[i] << "\n";
			}
		}
#endif
	} catch (const std::runtime_error& err) {
		cout << "Exception thrown: " << err.what() << "\n";
	}

	fclose(fOut);
}

void PolymeshTables::writeNeighborCells(const std::string& dirName) const
{
	string filename = dirName + "/neighbour";
	FILE* fOut = fopen(filename.c_str(), "wb");
	try {
		vector<int32_t> indices;
		indices.reserve(faceIdxIdMap.size());
		for (const auto& faceId : faceIdxIdMap) {
			const auto& face = _pVol->getPolygon(faceId);
			int nIdx = getFaceNeighbourIdx(face.getCellIds());
			if (nIdx != -1) {
				indices.push_back(nIdx);
			}
		}

		writeFOAMHeader(fOut, "binary", "labelList", "neighbour");
		fprintf(fOut, FMT_SIZE, indices.size());
		fprintf(fOut, "(");
		fwrite(indices.data(), sizeof(int32_t), indices.size(), fOut);
		fprintf(fOut, ")\n");
		fprintf(fOut, "//**********************************************************************************//\n");
#if WRITE_DEBUG_FILES
		{
			ofstream out(filename + ".csv");
			for (size_t i = 0; i < indices.size(); i++) {
				out << indices[i] << "\n";
			}
		}
#endif
	} catch (const std::runtime_error& err) {
		cout << "Exception thrown: " << err.what() << "\n";
	}

	fclose(fOut);

}

void PolymeshTables::writeBoundaries(const std::string& dirName) const
{
	const string names[6] = {
		"bottom",
		"top",
		"left",
		"right",
		"back",
		"front",
	};

	string filename = dirName + "/boundary";
	FILE* fOut = fopen(filename.c_str(), "wb");
	try {
		int32_t nWallFaces = boundaryIdx > 0 ? (int32_t)faceIdxIdMap.size() - boundaryIdx : 0;
		int32_t numBoundaries = 6;
		if (nWallFaces > 0)
			numBoundaries++;

		writeFOAMHeader(fOut, "binary", "polyBoundaryMesh", "boundary");

		fprintf(fOut, "%d\n", numBoundaries);
		fprintf(fOut, "(\n");

		for (int32_t i = 0; i < 6; i++) {
			fprintf(fOut, "  %s\n", names[i].c_str());
			fprintf(fOut, "  {\n");
			fprintf(fOut, "    type patch;\n");
			int32_t nFaces;
			if (i + 1 < 6)
				nFaces = boundaryIndices[i + 1] - boundaryIndices[i];
			else if (boundaryIdx > boundaryIndices[i])
				nFaces = boundaryIdx - boundaryIndices[i];
			else
				nFaces = (int32_t)faceIdxIdMap.size() - boundaryIndices[i];
			fprintf(fOut, "    nFaces %d;\n", nFaces);
			fprintf(fOut, "    startFace %d;\n", boundaryIndices[i]);
			fprintf(fOut, "  }\n");
		}

		if (nWallFaces > 0) {
			fprintf(fOut, "  walls\n");
			fprintf(fOut, "  {\n");
			fprintf(fOut, "    type wall;\n");
			fprintf(fOut, "    nFaces %d;\n", nWallFaces);
			fprintf(fOut, "    startFace %d;\n", boundaryIdx);
			fprintf(fOut, "  }\n");
		}

		fprintf(fOut, ")\n");
		fprintf(fOut, "//**********************************************************************************//\n");
	} catch (const std::runtime_error& err) {
		cout << "Exception thrown: " << err.what() << "\n";
	}

	fclose(fOut);
}

void PolymeshTables::writeFOAMHeader(FILE* fOut, const string& fileType, const string& foamClass, const string& object) const
{
	fprintf(fOut, "/*--------------------------------*- C++ -*----------------------------------*\\\n");
	fprintf(fOut, "  =========                 |\n");
	fprintf(fOut, "  \\\\      /  F ield         | OpenFOAM: The Open Source CFD Toolbox\n");
	fprintf(fOut, "   \\\\    /   O peration     | Version:  v1812\n");
	fprintf(fOut, "    \\\\  /    A nd           | Web:      www.OpenFOAM.com\n");
	fprintf(fOut, "     \\\\/     M anipulation  |\n");
	fprintf(fOut, "\\* --------------------------------------------------------------------------*/\n");
	fprintf(fOut, "FoamFile\n");
	fprintf(fOut, "	{\n");
	fprintf(fOut, "		format      %s;\n", fileType.c_str());
	fprintf(fOut, "		class       %s;\n", foamClass.c_str());
	fprintf(fOut, "		location    \"constant/polyMesh\";\n");
	fprintf(fOut, "		object      %s;\n", object.c_str());
	fprintf(fOut, "	}\n");
	fprintf(fOut, "//* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *//\n");
	fprintf(fOut, "\n");
	fprintf(fOut, "\n");

}
