#include <iostream>

#define _USE_MATH_DEFINES
#include <cmath>

#include <subBlock.h>
#include <block.h>
#include <polygon.h>
#include <polyhedron.h>
#include <vertex.h>

using namespace std;
using namespace DFHM;

void SubBlock::init(Block* pBlock, const Index3D& cellIdx)
{
	_pBlock = pBlock;
	_ourIdx = cellIdx;
}

void SubBlock::addRayHits(const Vector3d* cellCornerPts, size_t blockDim, const Index3D& cellIdx, const FaceRayHits rayTriHits[3])
{
	const double tol = 1.0e-5;
	CBoundingBox3Dd bbox;
	for (size_t i = 0; i < 8; i++) {
		bbox.merge(cellCornerPts[i]);
	}
	bbox.grow(tol);

	for (int axis = 0; axis < 3; axis++) {
		const auto& faceHits = rayTriHits[axis];
		size_t i, j;
		switch (axis) {
			case 0: {
				i = cellIdx[1];
				j = cellIdx[2];
				break;
			}
			case 1: {
				i = cellIdx[0];
				j = cellIdx[2];
				break;
			}
			case 2: {
				i = cellIdx[0];
				j = cellIdx[1];
				break;
			}
		}

		for (int ii = 0; ii < 2; ii++) {
			for (int jj = 0; jj < 2; jj++) {
				const auto& rayHits = rayTriHits->_data[i + ii][j + jj];
				for (const auto& hit : rayHits) {
					if (!bbox.contains(hit._hitPt))
						continue;
					double t = hit._dist / hit._segLen;
					size_t rayIdx = (size_t)(t * blockDim);
					switch (axis) {
					case 0:
						if (rayIdx >= cellIdx[0] && rayIdx < cellIdx[0] + 1) {
							addRayHit(axis, ii, jj, hit);
						}
						break;

					case 1:
						if (rayIdx >= cellIdx[1] && rayIdx < cellIdx[1] + 1) {
							addRayHit(axis, ii, jj, hit);
						}
						break;

					case 2:
						if (rayIdx >= cellIdx[2] && rayIdx < cellIdx[2] + 1) {
							addRayHit(axis, ii, jj, hit);
						}
						break;
					}

				}
			}
		}

		int dbgBreak = 1;
	}
}

void SubBlock::addRayHit(int axis, size_t i, size_t j, const RayTriHit& hit)
{
	FaceRayHits& faceHits = _faceRayHits[axis];
	if (faceHits.isEmpty()) {
		faceHits.resize(2);
	}
	assert(i < 2);
	assert(j < 2);
	faceHits._data[i][j].push_back(hit);
}

size_t SubBlock::maxIntersectionsPerLeg() const
{
	size_t result = 0;
	for (size_t axis = 0; axis < 3; axis++) {
		const auto& faceHits = _faceRayHits[axis]._data;
		for (size_t i = 0; i < faceHits.size(); i++) {
			for (size_t j = 0; j < faceHits.size(); j++) {
				if (faceHits[i][j].size() > result) {
					result = faceHits[i][j].size();
				}
			}
		}
	}

	return result;
}

void SubBlock::divide()
{
	auto blockCornerPts = _pBlock->getCornerPts();	
	auto polyId = _pBlock->addHexCell(blockCornerPts.data(), _pBlock->_blockDim, _ourIdx);
	if (polyId != -1)
		addPolyhdra(polyId);
}

bool SubBlock::createDividedHexCell(const std::vector<Vector3d>& srcCorners, size_t divs)
{
	bool hadHits = false;
	Index3D idx;
	for (idx[0] = 0; idx[0] < divs; idx[0]++) {
		for (idx[1] = 0; idx[1] < divs; idx[1]++) {
			for (idx[2] = 0; idx[2] < divs; idx[2]++) {
				auto polyId = _pBlock->addHexCell(srcCorners.data(), divs, idx);
				if (polyId != -1) {
					hadHits = true;
					addPolyhdra(polyId);
				}
			}
		}
	}

	return hadHits;
}

bool SubBlock::unload(ostream& out)
{
	out.write((char*)&_volType, sizeof(_volType));

	{
		// This writes out persistent ids
		size_t numPolyhedra = _polyhedra.size();

		// Write out the polygon ids
		out.write((char*)&numPolyhedra, sizeof(numPolyhedra));
		for (size_t id : _polyhedra) {
		}
		_polyhedra.clear();
	}

	return true;
}

bool SubBlock::load(istream& in)
{
	in.read((char*)&_volType, sizeof(_volType));


	return true;
}
