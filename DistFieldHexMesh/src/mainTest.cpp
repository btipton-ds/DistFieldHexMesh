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

#include <defines.h>
#include <tests.h>
#include <triMesh.h>
#include <volume.h>
#include <blockTest.h>
#include <MultiCoreUtil.h>
#include <vertex.h>
#include <pool_vector.h>

using namespace std;

namespace DFHM {

}

using namespace DFHM;

void testBlock(size_t bd = 8)
{
	Index3D::setBlockDim(bd);
	TestBlock tb;

	if (!tb.testBlock00()) return;
	if (!tb.testBlock01()) return;
	if (!tb.testBlock02()) return;
	if (!tb.testBlock03()) return;
	if (!tb.testBlock04()) return;


	cout << "testBlock pass\n";
}

class TestPoolMemory {
public:
	bool testAll();
private:
	bool testAllocator();
	bool testAllocator0();

	bool testVector();
	bool testVector0();
};

bool TestPoolMemory::testAll()
{
	if (!testAllocator())
		return false;

	if (!testVector())
		return false;

	cout << "TestPoolMemory pass\n";
	return true;
}

bool TestPoolMemory::testAllocator()
{
	if (!testAllocator0()) return false;

	cout << "TestAllocator pass";

	return true;
}

bool TestPoolMemory::testAllocator0()
{
	PoolUtils::localHeap alloc(128);
	double *pD = (double *) alloc.alloc(sizeof(double));
	TEST_TRUE(pD != nullptr, "Failed to allocate pointer");
	*pD = 1.0;
	alloc.free(pD);

	double* pD2 = (double*)alloc.alloc(sizeof(double));
	
	TEST_TRUE(pD2 != nullptr, "Failed to allocate pointer");
	TEST_EQUAL(pD, pD2, "Delete and allocate pointer did not result in the same address");

	*pD2 = 1.0;
	alloc.free(pD2);

	pD = (double*)alloc.alloc(3 * sizeof(double));
	TEST_TRUE(pD != nullptr, "Failed to allocate pointer");
	pD[0] = 1.0;
	pD[1] = 2.0;
	pD[2] = 3.0;
	alloc.free(pD);

	pD2 = (double*)alloc.alloc(3 * sizeof(double));

	TEST_TRUE(pD2 != nullptr, "Failed to allocate pointer");
	TEST_EQUAL(pD, pD2, "Delete and allocate pointer did not result in the same address");

	pD2[0] = 1.0;
	pD2[1] = 2.0;
	pD2[2] = 3.0;
	alloc.free(pD2);

	return true;
}

bool TestPoolMemory::testVector()
{
	if (!testVector0()) return false;

	cout << "TestAllocator pass";
	return true;
}

bool TestPoolMemory::testVector0()
{
	PoolUtils::localHeap(1024);
	PoolUtils::vector<size_t> vec;

	TEST_TRUE(vec.empty(), "vec empty failed");

	for (size_t i = 0; i < 128; i++)
		vec.push_back(i);

	TEST_FALSE(vec.empty(), "vec not empty failed");
	TEST_EQUAL(vec.size(), 128, "vec size failed");

	for (size_t i = 0; i < 128; i++)
		vec.pop_back();

	TEST_TRUE(vec.empty(), "vec empty failed");
	TEST_EQUAL(vec.size(), 0, "vec size failed");

	cout << "TestAllocator0 pass";
	return true;
}

int main(int numParams, const char** params)
{
//	testBlock();
	TestPoolMemory tm;
	if (!tm.testAll()) 
		return 0;

	return 0;
}

