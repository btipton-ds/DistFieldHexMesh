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

	Robert R Tipton - Author

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
#include <index3D.h>
#include <fastBisectionMap.h>

namespace DFHM {

class Volume;

using Index3DToIdxMap = FastBisectionMap<Index3DId, int32_t>;

class PolymeshTables {
public:
	PolymeshTables(const Volume* pVol);
	template<class PROG_LAMBDA>
	void create(PROG_LAMBDA progFunc);
	template<class PROG_LAMBDA>
	void writeFile(const std::string& dirName, PROG_LAMBDA progFunc) const;

private:
	void createInner();
	void writePoints(const std::string& dirName) const;
	void writeFaces(const std::string& dirName) const;
	void writeOwnerCells(const std::string& dirName) const;
	void writeNeighborCells(const std::string& dirName) const;
	void writeBoundaries(const std::string& dirName) const;
	void writeFOAMHeader(FILE* fOut, const std::string& fileType, const std::string& foamClass, const std::string& object) const;

	int getFaceOwnerIdx(const MTC::set<Index3DId>& cellIds) const;
	int getFaceNeighbourIdx(const MTC::set<Index3DId>& cellIds) const;
	void reverseFaceIfNeeded(const MTC::set<Index3DId>& cellIds, std::vector<int32_t>& faceVertIds);

	const Volume* _pVol;
	int32_t numInner = -1;
	int32_t boundaryIdx = -1;
	int32_t boundaryIndices[6];
	std::vector<Index3DId> vertIdxIdMap, faceIdxIdMap, cellIdxIdMap;
//	std::map<Index3DId, int32_t> ;
	Index3DToIdxMap vertIdIdxMap, faceIdIdxMap, cellIdIdxMap;
	std::vector<int32_t> faceIndices, vertIndices;
};

template<class PROG_LAMBDA>
void PolymeshTables::create(PROG_LAMBDA progFunc)
{
	progFunc(0);
	createInner();
	progFunc(1 / 6.0);
}

template<class PROG_LAMBDA>
void PolymeshTables::writeFile(const std::string& dirName, PROG_LAMBDA progFunc) const
{
	int i = 1;
	double steps = 6;
	writePoints(dirName);
	progFunc(++i / steps);

	writeFaces(dirName);
	progFunc(++i / steps);

	writeOwnerCells(dirName);
	progFunc(++i / steps);

	writeNeighborCells(dirName);
	progFunc(++i / steps);

	writeBoundaries(dirName);
	progFunc(++i / steps);

}

}
