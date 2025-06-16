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
#include <stdint.h>

#if USE_MULTI_THREAD_CONTAINERS
#include <pool_vector.h>
#include <pool_set.h>
#include <pool_map.h>
#else
#include <vector>
#include <map>
#include <set>
#endif

#include <string>
#include <future>
#include <index3D.h>
#include <fastBisectionMap.h>
#include <fastBisectionSet.h>

namespace DFHM {

class Volume;
class ProgressReporter;

using Index3DToIdxMap = FastBisectionMap<Index3DId, int32_t>;

class PolymeshTables {
public:
	PolymeshTables(const Volume* pVol, ProgressReporter* pReporter);
	void create();
	void writeFile(const std::string& dirName) const;

private:
	void createVertMaps();
	void createPolyhedraMaps();
	void createPolygonMaps();
	void createSortPolygons();
	void createPolygonTables();

	void writePoints(const std::string& dirName) const;
	void writeFaces(const std::string& dirName) const;
	void writeOwnerCells(const std::string& dirName) const;
	void writeNeighborCells(const std::string& dirName) const;
	void writeBoundaries(const std::string& dirName) const;
	void writeFOAMHeader(FILE* fOut, const std::string& fileType, const std::string& foamClass, const std::string& object) const;

	int getFaceOwnerIdx(const FastBisectionSet<Index3DId>& cellIds) const;
	int getFaceNeighbourIdx(const FastBisectionSet<Index3DId>& cellIds) const;
	void reverseFaceIfNeeded(const FastBisectionSet<Index3DId>& cellIds, std::vector<int32_t>& faceVertIds);

	void reportProgress() const;

	const Volume* _pVol;
	ProgressReporter* _pProgReporter;
	int32_t numInner = -1;
	int32_t boundaryIdx = -1;
	int32_t boundaryIndices[6];
	std::vector<Index3DId> vertIdxIdMap, faceIdxIdMap, cellIdxIdMap;
//	std::map<Index3DId, int32_t> ;
	Index3DToIdxMap vertIdIdxMap, faceIdIdxMap, cellIdIdxMap;
	std::vector<int32_t> faceIndices, vertIndices;

	struct SearchRec {
		size_t faceIdx = -1, nCells = -1;
		int32_t ownerIdx = -1, neighborIdx = -1;
		int coplanarIdx = -1;
	};

	FastBisectionMap<Index3DId, SearchRec> faceIdToSearchRecMap;
};

}
