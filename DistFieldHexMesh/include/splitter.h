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
#include <volume.h>

namespace DFHM {

class Block;
class Volume;
using VolumePtr = std::shared_ptr<Volume>;
class Polygon;
class Polyhedron;

class Splitter {
public:
	static void clearThreadLocal();
	Splitter(Block* pBlock, const Index3DId& polyhedronId, std::vector<Index3DId>& localTouched);
	~Splitter();

	bool splitAtCenter();

private:
	enum CellType {
		CT_HEX,
		CT_UNKNOWN,
	};
	static void calHexCellFaceTU(int i, const Vector3d& tuv, double& t, double& u);

	Block* getBlockPtr();
	const Block* getBlockPtr() const;

	void conditionalSplitQuadFaceAtParam(const Index3DId& faceId, const std::vector<Vector3d>& facePts, double t, double u);
	Index3DId vertId(const Vector3d& pt);
	void createHexCellData(const Polyhedron& parentCell);
	void splitHexCell(const Vector3d& tuv);
	void splitHexCell8(const Index3DId& parentId, const Vector3d& tuv);
	void addHexCell(const Index3DId& parentId, const std::vector<Vector3d>& cubePts, double tol);
	Index3DId findSourceFaceId(const Index3DId& parentId, const std::vector<Vector3d>& facePts, double tol) const;
	void findSourceFaceId_inner(const Index3DId& faceId, const std::vector<Vector3d>& facePts, double& minErr, Index3DId& result, double tol) const;

	LAMBDA_CLIENT_DECLS;

	Block* _pBlock;
	Block* _pScratchBlock;
	const SplittingParams& _params;
	Index3DId _polyhedronId, _scratchCellId;
	size_t _numSplitFaces = 0;
	std::vector<Index3DId>& _localTouched;
	FastBisectionSet<Index3DId> _newCellIds;
	MTC::vector<Vector3d> _cornerPts;
	MTC::vector<MTC::vector<Vector3d>> _cellFacePoints;
	MTC::vector<MTC::vector<Index3DId>> _cellFaceVertIds;


	thread_local static VolumePtr _pScratchVol;
};

inline Block* Splitter::getBlockPtr()
{
	return _pBlock;
}

inline const Block* Splitter::getBlockPtr() const
{
	return _pBlock;
}

}
