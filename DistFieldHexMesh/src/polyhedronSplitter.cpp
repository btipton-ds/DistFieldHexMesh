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
#include <assert.h>
#include <tm_bestFit.h>
#include <tm_lineSegment.h>
#include <tm_ray.h>
#include <triMesh.h>
#include <triMeshPatch.h>
#include <pool_vector.h>
#include <pool_set.h>
#include <splitParams.h>
#include <polygon.h>
#include <polyhedron.h>
#include <polyhedronSplitter.h>
#include <block.h>
#include <volume.h>
#include <tolerances.h>
#include <utils.h>

#ifdef _WIN32
#include <windows.h>
#include <profileapi.h>
#endif // _WIN32

using namespace std;
using namespace DFHM;

PolyhedronSplitter::PolyhedronSplitter(Block* pBlock, const Index3DId& polyhedronId)
	: _pBlock(pBlock)
	, _polyhedronId(polyhedronId)
{
}

bool PolyhedronSplitter::splitIfNeeded()
{
	bool result = false;
	Vector3d ctr;
	_pBlock->cellFunc(_polyhedronId, [this, &ctr, &result](const Polyhedron& cell) {
		if (cell.needsDivideAtCentroid()) {
			ctr = cell.calCentroid();
		}
		else {
			// TODO This needs to be an adaptive split to match the neighbor
			// We get here when we have to split a neighbor cell prior to splitting a cell.
			ctr = cell.calCentroid();
		}
		});
	result = splitAtPoint(ctr);

	return result;
}

bool PolyhedronSplitter::splitAtPoint(const Vector3d& pt)
{
	if (!_pBlock->polyhedronExists(_polyhedronId))
		return false;

	const Polyhedron& referenceCell = _pBlock->getPolyhedron(_polyhedronId);
	Polyhedron& realCell = _pBlock->getPolyhedron(_polyhedronId);

	bool result = splitAtPointInner(realCell, referenceCell, pt);

	return result;
}

bool PolyhedronSplitter::splitAtPointInner(Polyhedron& realCell, const Polyhedron& referenceCell, const Vector3d& pt)
{

#if 0 && defined(_DEBUG)
	// Now split the cell
	if (Index3DId(0, 12, 0, 19) == _polyhedronId) {
		_pBlock->dumpPolyhedraObj({ _polyhedronId }, false, false, false, { pt });
	}
#endif

	Utils::Timer tmr(Utils::Timer::TT_splitAtPointInner);

	MTC::vector<Vector3d> corners;
	if (referenceCell.classify(corners) == 8) {
		Vector3d tuv;
		if (TRI_LERP_INV(pt, corners, tuv)) {
			MTC::vector<Vector3d> pts[] = {
				{corners[0], corners[3], corners[2], corners[1]}, // bottom
				{corners[4], corners[5], corners[6], corners[7]}, // top

				{corners[0], corners[4], corners[7], corners[3]}, // back
				{corners[1], corners[2], corners[6], corners[5]}, // front

				{corners[0], corners[1], corners[5], corners[4]}, // right
				{corners[3], corners[7], corners[6], corners[2]}, // left
			};
			MTC::vector<Index3DId> cornersToFaceIdMap;
			for (int i = 0; i < 6; i++) {
				const auto& facePts = pts[i];
				MTC::vector<Index3DId> faceIds;
				for (int j = 0; j < 4; j++) {
					faceIds.push_back(_pBlock->getVertexIdOfPoint(facePts[j]));
				}

				Polygon face(faceIds);
				Index3DId faceId = _pBlock->findFace(face);

				double t, u;
				switch (i) {
				case 0: // bottom
				case 1: // top
					t = tuv[0];
					u = tuv[1];
					break;

				case 2: // back
				case 3: // front
					t = tuv[1];
					u = tuv[2];
					break;

				case 4: // left
				case 5: // right
					t = tuv[0];
					u = tuv[2];
					break;
				}

				splitFaceAtParam(faceId, facePts, t, u);
				cornersToFaceIdMap.push_back(faceId);
			}

			for (int i = 0; i < 2; i++) {
				double t0 = (i == 0) ? 0 : tuv[0];
				double t1 = (i == 0) ? tuv[0] : 1;
				for (int j = 0; j < 2; j++) {
					double u0 = (j == 0) ? 0 : tuv[1];
					double u1 = (j == 0) ? tuv[1] : 1;
					for (int k = 0; k < 2; k++) {
						double v0 = (k == 0) ? 0 : tuv[2];
						double v1 = (k == 0) ? tuv[2] : 1;

						MTC::vector<Vector3d> subCorners = {
							TRI_LERP(corners, t0, u0, v0),
							TRI_LERP(corners, t1, u0, v0),
							TRI_LERP(corners, t1, u1, v0),
							TRI_LERP(corners, t0, u1, v0),

							TRI_LERP(corners, t0, u0, v1),
							TRI_LERP(corners, t1, u0, v1),
							TRI_LERP(corners, t1, u1, v1),
							TRI_LERP(corners, t0, u1, v1),
						};

						Index3DId newCellId = _pBlock->addHexCell(subCorners);
#if 0
						realCell.addSplitCellId(newCellId);
						_pBlock->cellFunc(newCellId, [&referenceCell](Polyhedron& newCell) {
							newCell.setParentId(referenceCell.getId());
							});
#endif
						int dbgBreak = 1;
					}
				}
			}
		}
	}

	return true;
}

void PolyhedronSplitter::splitFaceAtParam(const Index3DId& faceId, const std::vector<Vector3d>& facePts, double t, double u)
{
	_pBlock->faceFunc(faceId, [this, &facePts, t, u](const Polygon& refFace) {
		_pBlock->faceFunc(refFace.getId(), [this, &facePts, &refFace, t, u](Polygon& dstFace) {
			if (true /*dstFace.isActive()*/) {
				const auto& id = refFace.getId();
				auto cellIds = refFace.getCellIds();
				for (const auto& cellId : cellIds) {
					dstFace.removeCellId(cellId); // detach this cell
				}

				for (int i = 0; i < 2; i++) {
					double t0 = (i == 0) ? 0 : t;
					double t1 = (i == 0) ? t : 1;
					for (int j = 0; j < 2; j++) {
						double u0 = (i == 0) ? 0 : u;
						double u1 = (i == 0) ? u : 1;

						vector<Vector3d> subFacePts = {
							BI_LERP<double>(facePts[0], facePts[1], facePts[2], facePts[3], t0, u0),
							BI_LERP<double>(facePts[0], facePts[1], facePts[2], facePts[3], t1, u0),
							BI_LERP<double>(facePts[0], facePts[1], facePts[2], facePts[3], t1, u1),
							BI_LERP<double>(facePts[0], facePts[1], facePts[2], facePts[3], t0, u1),
						};

						auto newFaceId = _pBlock->addFace(subFacePts);
						MTC::vector<Index3DId> vertIds;
						_pBlock->faceFunc(newFaceId, [&refFace, &cellIds, &vertIds](Polygon& newFace) {
							vertIds = newFace.getVertexIds();
							for (const auto& cellId : cellIds) {
								//								newFace.addCellId(cellId);
							}
							});
#if 0
						dstFace.addSplitFaceId(newFaceId);
						dstFace.setParentId(id);

						for (const auto& cellId : cellIds) {
							_pBlock->cellFunc(cellId, [&vertIds](Polyhedron& cell) {
								cell.imprintFaceVertices(vertIds);
								});
						}
#endif
					}
				}
			}
			});
		});
}
