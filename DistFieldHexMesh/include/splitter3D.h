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

#include <set>
#include <map>

#include <defines.h>
#include <tm_vector3.h>
#include <tm_plane.h>
#include <index3D.h>
#include <pool_set.h>
#include <pool_vector.h>
#include <tolerances.h>
#include <volume.h>

namespace DFHM {

class Block;
class Volume;
using VolumePtr = std::shared_ptr<Volume>;
class Polygon;
class Polyhedron;

class Splitter3D {
public:
	static void dumpSplitStats();
	static void clearThreadLocal();
	Splitter3D(Block* pBlock, const Index3DId& polyhedronId, MTC::vector<Index3DId>& localTouched);
	~Splitter3D();

	bool splitAtCenter();
private:
	enum CellType {
		CT_HEX,
		CT_UNKNOWN,
	};

	struct ScopedDisconnect {
		ScopedDisconnect(Splitter3D& splitter);
		~ScopedDisconnect();
		Splitter3D& _splitter;
	};

	Block* getBlockPtr();
	const Block* getBlockPtr() const;

	void reset();

	void disconnectVertEdgeTopology();
	void connectVertEdgeTopology();
	void imprintEverything();

	Index3DId vertId(const Vector3d& pt);
	const Vector3d& vertexPoint(const  Index3DId& id) const;
	void createHexCellData(const Polyhedron& parentCell);
	bool splitHexCell(const Vector3d& tuv);
	bool splitHexCell8(const Index3DId& parentId, const Vector3d& tuv);
	bool splitHexCell2(const Index3DId& parentId, const Vector3d& tuv, int axis);
	bool splitHexCell4(const Index3DId& parentId, const Vector3d& tuv, int axis);
	Index3DId createScratchCell();
	Index3DId createScratchFace(const Index3DId& srcFaceId);
	void replaceExistingFaces(const Index3DId& existingFaceId, const std::vector<std::vector<Vector3d>>& newFacePoints);
	Index3DId addHexCell(const Index3DId& parentId, const MTC::vector<Index3DId>& cubeVerts, double tol);
	void createFaces(const Index3DId& parentId, const MTC::vector<Index3DId>& newFaceVertIds, MTC::set<Index3DId>& newFaceIds, double tol);

	LAMBDA_CLIENT_DECLS;

	bool _testRun = false;
	Block* _pBlock;
	Block* _pSrcBlock;
	Block* _pScratchBlock;
	const SplittingParams& _params;
	Index3DId _polyhedronId;

	const double _distTol = Tolerance::sameDistTol();
	const double _distTolSr = _distTol * _distTol;
	const double _paramTol = Tolerance::paramTol();
	const double _paramTolSqr = _paramTol * _paramTol;

	MTC::vector<Index3DId>& _localTouched;
	std::vector<Vector3d> _cornerPts;
	MTC::vector<MTC::vector<Vector3d>> _cellFacePoints;
	MTC::vector<MTC::vector<Index3DId>> _cellFaceVertIds;
	FastBisectionSet<Index3DId> _adjacentCellIds, _newCellIds;

	thread_local static VolumePtr _pScratchVol;
};

inline Block* Splitter3D::getBlockPtr()
{
	return _pBlock;
}

inline const Block* Splitter3D::getBlockPtr() const
{
	return _pBlock;
}

}
