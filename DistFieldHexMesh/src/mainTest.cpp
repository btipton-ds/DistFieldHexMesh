// DistFieldHexMesh.cpp : Defines the entry point for the application.
//

#include <memory>
#include <iostream>

#include <triMesh.h>
#include <volume.h>
#include <blockTest.h>
#include <dataPool.h>
#include <MultiCoreUtil.h>
#include <vertex.h>

using namespace std;

namespace DFHM {

}

using namespace DFHM;

void testBlock(size_t bd = 8)
{
	Block::setBlockDim(bd);
	TestBlock tb;

	if (!tb.testBlock00()) return;
	if (!tb.testBlock01()) return;
	if (!tb.testBlock02()) return;
	if (!tb.testBlock03()) return;
	if (!tb.testBlock04()) return;


	cout << "testBlock pass\n";
}

int main(int numParams, const char** params)
{
	DataPool::setNumThreads(MultiCore::getNumCores());
	testBlock();

	return 0;
}

