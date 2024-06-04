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

#include <memory>
#include <iostream>

#include <triMesh.h>

#include <block.h>
#include <testBlock.h>
#include <vertex.h>

using namespace std;
using namespace TriMesh;
using namespace DFHM;

bool TestBlock::testBlock00()
{
	return true;
}

bool TestBlock::testBlock01()
{
#if 0
	auto& cp = Block::_subBlocks;
	cp.testReset();

	const size_t bd = Block::getBlockDim();
	Block block;

	for (size_t i = 0; i < bd; i++) {
		for (size_t j = 0; j < bd; j++) {
			for (size_t k = 0; k < bd; k++) {
				if (block.subBlockExists(i, j, k)) {
					cout << "testBlock fail: Bad empty subBlock test " << i << ", " << j << ", " << k << "\n";
					return false;
				}
			}
		}
	}

#endif
	return true;
}

bool TestBlock::testBlock02()
{
#if 0
	auto& cp = Block::_subBlocks;
	cp.testReset();

	const size_t bd = Block::getBlockDim();
	Block block;

	for (size_t i = 0; i < bd; i++) {
		for (size_t j = 0; j < bd; j++) {
			for (size_t k = 0; k < bd; k++) {
				block.addCell(i, j, k);
				if (!block.subBlockExists(i, j, k)) {
					cout << "testBlock fail: Bad subBlock test after create " << i << ", " << j << ", " << k << "\n";
					return false;
				}
			}
		}
	}

	if (block.numCells() != bd * bd * bd) {
		cout << "testBlock fail: Bad subBlock count\n";
		return false;
	}

	if (cp.getNumAllocated() != bd * bd * bd) {
		cout << "testBlock fail: Allocated subBlock count mismatch\n";
		return false;
	}

	if (cp.getNumAvailable() != 0) {
		cout << "testBlock fail: Available pool wrong size\n";
		return false;
	}

	if (cp.getNumAvailableIds() != 0) {
		cout << "testBlock fail: Available id pool wrong size\n";
		return false;
	}

	for (size_t i = 0; i < bd; i++) {
		for (size_t j = 0; j < bd; j++) {
			for (size_t k = 0; k < bd; k++) {
				block.freeCell(i, j, k);
				if (block.subBlockExists(i, j, k)) {
					cout << "testBlock fail: Bad empty subBlock test after create and free " << i << ", " << j << ", " << k << "\n";
					return false;
				}
			}
		}
	}

	if (block.numCells() != bd * bd * bd) {
		cout << "testBlock fail: Bad subBlock count\n";
		return false;
	}

	block.pack();

	if (block.numCells() != 0) {
		cout << "testBlock fail: Bad subBlock count after pack\n";
		return false;
	}

	if (cp.getNumAllocated() != 0) {
		cout << "testBlock fail: Allocated subBlock count mismatch\n";
		return false;
	}

	if (cp.getNumAvailable() != bd * bd * bd) {
		cout << "testBlock fail: Available pool wrong size\n";
		return false;
	}

	if (cp.getNumAvailableIds() != bd * bd * bd) {
		cout << "testBlock fail: Available id pool wrong size\n";
		return false;
	}

#endif
	return true;
}

bool TestBlock::testBlock03()
{
#if 0
	auto& cp = Block::_subBlocks;
	cp.testReset();

	const size_t bd = Block::getBlockDim();
	Block block;

	vector<bool> subBlocksToCreate;
	subBlocksToCreate.resize(bd * bd * bd);

	for (size_t i = 0; i < bd; i++) {
		for (size_t j = 0; j < bd; j++) {
			for (size_t k = 0; k < bd; k++) {
				auto v = rand() / (double)RAND_MAX;
				if (v < 0.25) {
					size_t idx = Block::calcCellIndex(i, j, k);
					subBlocksToCreate[idx] = true;
				}
			}
		}
	}

	block.createCells(subBlocksToCreate, 0);

	for (size_t i = 0; i < bd; i++) {
		for (size_t j = 0; j < bd; j++) {
			for (size_t k = 0; k < bd; k++) {
				size_t idx = Block::calcCellIndex(i, j, k);
				if (subBlocksToCreate[idx]) {
					if (!block.subBlockExists(i, j, k)) {
						cout << "testBlock fail: Bad empty block.createCells test. No block at " << i << ", " << j << ", " << k << "\n";
						return false;
					}
				}
				else {
					if (block.subBlockExists(i, j, k)) {
						cout << "testBlock fail: Bad empty block.createCells test. Block at " << i << ", " << j << ", " << k << "\n";
						return false;
					}
				}
			}
		}
	}
#endif

	return true;
}

bool TestBlock::testBlock04()
{
#if 0
	auto& cp = Block::_subBlocks;
	cp.testReset();

	const size_t bd = Block::getBlockDim();
	Block block;

	vector<bool> subBlocksToCreate;
	subBlocksToCreate.resize(bd * bd * bd);

	for (size_t i = 0; i < bd; i++) {
		for (size_t j = 0; j < bd; j++) {
			for (size_t k = 0; k < bd; k++) {
				auto v = rand() / (double)RAND_MAX;
				if (v < 0.25) {
					size_t idx = Block::calcCellIndex(i, j, k);
					subBlocksToCreate[idx] = true;
				}
			}
		}
	}

	block.createCells(subBlocksToCreate, 0);

	Vector3d blockOrigin(0, 0, 0), span(1, 1, 1), subBlockOrgin, subBlockSpan;
	for (int i = 0; i < 3; i++)
		subBlockSpan[i] = span[i] / bd;

	CMesh::BoundingBox meshBB(blockOrigin, blockOrigin + span);
	meshBB.grow(0.1);

	CMeshPtr pMesh = make_shared<CMesh>();
	pMesh->reset(meshBB);

	block.addBlockFaces(ObjectPoolId(0, 0), blockOrigin, span, true);

	for (size_t i = 0; i < bd; i++) {
		subBlockOrgin[0] = blockOrigin[0] + i * subBlockSpan[0];
		for (size_t j = 0; j < bd; j++) {
			subBlockOrgin[1] = blockOrigin[1] + j * subBlockSpan[1];
			for (size_t k = 0; k < bd; k++) {
				subBlockOrgin[2] = blockOrigin[2] + k * subBlockSpan[2];

				size_t idx = Block::calcCellIndex(i, j, k);
				if (subBlocksToCreate[idx]) {
					CBoundingBox3Dd bb;
					bb.merge(subBlockOrgin);
					bb.merge(subBlockOrgin + subBlockSpan);
					Vector3d subBlockCentroid = 0.5 * (bb.getMin() + bb.getMax());

					vector<size_t> triIndices;
					pMesh->findTris(bb, triIndices, CMesh::BoxTestType::Intersects);
					size_t numPos = 0, numNeg = 0;
					for (size_t triIdx : triIndices) {
						auto vertIndices = pMesh->getTri(triIdx);
						Vector3d pts[] = {
							pMesh->getVert(vertIndices[0])._pt,
							pMesh->getVert(vertIndices[1])._pt,
							pMesh->getVert(vertIndices[2])._pt,
						};
						if (bb.contains(pts[0]) && bb.contains(pts[1]) && bb.contains(pts[2])) {
							Vector3d triCentroid = pMesh->triCentroid(triIdx);
							Vector3d v = (triCentroid - subBlockCentroid).normalized();
							Vector3d n = pMesh->triUnitNormal(triIdx);
							auto dp = n.dot(v);
							if (dp > 0)
								numPos++;
							else
								numNeg++;
						}
					}

					if (numPos != 12) {
						cout << "testBlock fail: Bad TriMesh triangle at " << i << ", " << j << ", " << k << "\n";
						return false;
					}
				}
			}
		}
	}
#endif
	return true;
}

