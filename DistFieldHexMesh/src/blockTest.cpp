#include <memory>
#include <iostream>

#include <triMesh.h>

#include <cell.h>
#include <block.h>
#include <blockTest.h>

using namespace std;
using namespace TriMesh;
using namespace DFHM;

bool TestBlock::testBlock00()
{
	const size_t bd = Block::getBlockDim();

	if (Block::getBlockDim() != bd) {
		cout << "testBlock fail: Bad blockdim\n";
		return false;
	}

	{
		Block block;
		block.fillEmpty();

		if (block.numCells() != bd * bd * bd) {
			cout << "testBlock fail: Bad cell count\n";
			return false;
		}
	}

	return true;
}

bool TestBlock::testBlock01()
{

	const size_t bd = Block::getBlockDim();
	Block block;

	for (size_t i = 0; i < bd; i++) {
		for (size_t j = 0; j < bd; j++) {
			for (size_t k = 0; k < bd; k++) {
				if (block.getCell(i, j, k)) {
					cout << "testBlock fail: Bad empty cell test " << i << ", " << j << ", " << k << "\n";
					return false;
				}
			}
		}
	}
	return true;
}

bool TestBlock::testBlock02()
{
	const size_t bd = Block::getBlockDim();
	Block block;

	for (size_t i = 0; i < bd; i++) {
		for (size_t j = 0; j < bd; j++) {
			for (size_t k = 0; k < bd; k++) {
				auto* pCell = block.getCell(i, j, k, true);
				if (!block.getCell(i, j, k)) {
					cout << "testBlock fail: Bad cell test after create " << i << ", " << j << ", " << k << "\n";
					return false;
				}
			}
		}
	}

	if (block.numCells() != bd * bd * bd) {
		cout << "testBlock fail: Bad cell count\n";
		return false;
	}

	for (size_t i = 0; i < bd; i++) {
		for (size_t j = 0; j < bd; j++) {
			for (size_t k = 0; k < bd; k++) {
				block.freeCell(i, j, k);
				if (block.getCell(i, j, k)) {
					cout << "testBlock fail: Bad empty cell test after create and free " << i << ", " << j << ", " << k << "\n";
					return false;
				}
			}
		}
	}

	if (block.numCells() != bd * bd * bd) {
		cout << "testBlock fail: Bad cell count\n";
		return false;
	}

	block.pack();

	if (block.numCells() != 0) {
		cout << "testBlock fail: Bad cell count after pack\n";
		return false;
	}

	return true;
}

bool TestBlock::testBlock03()
{
	const size_t bd = Block::getBlockDim();
	Block block;

	vector<bool> cellsToCreate;
	cellsToCreate.resize(bd * bd * bd);

	for (size_t i = 0; i < bd; i++) {
		for (size_t j = 0; j < bd; j++) {
			for (size_t k = 0; k < bd; k++) {
				auto v = rand() / (double)RAND_MAX;
				if (v < 0.25) {
					size_t idx = Block::calcLinearCellIndex(i, j, k);
					cellsToCreate[idx] = true;
				}
			}
		}
	}

	RayHitRec rayHits;
	Vector3d blockOrigin(0, 0, 0);
	Vector3d blockSpan(1, 1, 1);
	block.createCells(blockOrigin, blockSpan, Vector3i(0, 0, 0), Vector3i(0, 0, 0), cellsToCreate, rayHits);

	for (size_t i = 0; i < bd; i++) {
		for (size_t j = 0; j < bd; j++) {
			for (size_t k = 0; k < bd; k++) {
				size_t idx = Block::calcLinearCellIndex(i, j, k);
				if (cellsToCreate[idx]) {
					if (!block.getCell(i, j, k)) {
						cout << "testBlock fail: Bad empty block.createCells test. No block at " << i << ", " << j << ", " << k << "\n";
						return false;
					}
				}
				else {
					if (block.getCell(i, j, k)) {
						cout << "testBlock fail: Bad empty block.createCells test. Block at " << i << ", " << j << ", " << k << "\n";
						return false;
					}
				}
			}
		}
	}

	return true;
}

bool TestBlock::testBlock04()
{
	const size_t bd = Block::getBlockDim();
	Block block;

	vector<bool> cellsToCreate;
	cellsToCreate.resize(bd * bd * bd);

	for (size_t i = 0; i < bd; i++) {
		for (size_t j = 0; j < bd; j++) {
			for (size_t k = 0; k < bd; k++) {
				auto v = rand() / (double)RAND_MAX;
				if (v < 0.25) {
					size_t idx = Block::calcLinearCellIndex(i, j, k);
					cellsToCreate[idx] = true;
				}
			}
		}
	}

	RayHitRec rayHits;
	Vector3d blockOrigin(0, 0, 0), span(1, 1, 1), cellOrgin, cellSpan;
	block.createCells(blockOrigin, span, Vector3i(0, 0, 0), Vector3i(0, 0, 0), cellsToCreate, rayHits);

	for (int i = 0; i < 3; i++)
		cellSpan[i] = span[i] / bd;

	TriMesh::CMesh::BoundingBox meshBB(blockOrigin, blockOrigin + span);
	meshBB.grow(0.1);

	TriMesh::CMeshPtr pMesh = make_shared<TriMesh::CMesh>();
	pMesh->reset(meshBB);

	block.addBlockTris(blockOrigin, span, pMesh, true);

	for (size_t i = 0; i < bd; i++) {
		cellOrgin[0] = blockOrigin[0] + i * cellSpan[0];
		for (size_t j = 0; j < bd; j++) {
			cellOrgin[1] = blockOrigin[1] + j * cellSpan[1];
			for (size_t k = 0; k < bd; k++) {
				cellOrgin[2] = blockOrigin[2] + k * cellSpan[2];

				size_t idx = Block::calcLinearCellIndex(i, j, k);
				if (cellsToCreate[idx]) {
					CBoundingBox3Dd bb;
					bb.merge(cellOrgin);
					bb.merge(cellOrgin + cellSpan);
					Vector3d cellCentroid = 0.5 * (bb.getMin() + bb.getMax());

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
							Vector3d v = (triCentroid - cellCentroid).normalized();
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

	return true;
}

