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
	bool testVectorSort();
	bool testVectorInsertErase(bool useInitializer);
	bool testVectorForLoops();
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

	cout << "TestAllocator pass\n";

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
	if (!testVectorSort()) return false;
	if (!testVectorInsertErase(true)) return false;
	if (!testVectorInsertErase(false)) return false;
	if (!testVectorForLoops()) return false;

	cout << "testVector pass\n";
	return true;
}

bool TestPoolMemory::testVector0()
{
	PoolUtils::localHeap lh(1024);
	PoolUtils::vector<size_t> vec(lh);

	TEST_TRUE(vec.empty(), "vec empty failed");

	for (size_t i = 0; i < 128; i++)
		vec.push_back(i);

	TEST_FALSE(vec.empty(), "vec not empty failed");
	TEST_EQUAL(vec.size(), 128, "vec size failed");

	for (size_t i = 0; i < 128; i++)
		vec.pop_back();

	TEST_TRUE(vec.empty(), "vec empty failed");
	TEST_EQUAL(vec.size(), 0, "vec size failed");

	cout << "testVector0 pass\n";
	return true;
}

bool TestPoolMemory::testVectorSort()
{
	PoolUtils::localHeap lh(1024);
	PoolUtils::vector<size_t> vec({ 7, 4, 5, 3, 1, 6, 0, 2 }, lh);
	std::vector<size_t> stdVec({ 7, 4, 5, 3, 1, 6, 0, 2 });

	TEST_EQUAL(stdVec.size(), stdVec.size(), "vec size failed");
	TEST_EQUAL(stdVec.size(), stdVec.size(), "vec size failed");
	for (size_t i = 0; i < vec.size(); i++) {
		TEST_EQUAL(stdVec[i], stdVec[i], "vec size failed");
	}

	std::sort(stdVec.begin(), stdVec.end());
	std::sort(vec.begin(), vec.end());

	for (size_t i = 0; i < vec.size(); i++) {
		TEST_EQUAL(stdVec[i], stdVec[i], "vec size failed");
	}

	std::sort(stdVec.begin(), stdVec.end(), std::greater<>());
	std::sort(vec.begin(), vec.end(), std::greater<>());

	for (size_t i = 0; i < vec.size(); i++) {
		TEST_EQUAL(stdVec[i], stdVec[i], "vec size failed");
	}

	cout << "testVectorSort pass\n";
	return true;
}

bool TestPoolMemory::testVectorInsertErase(bool useInitializer)
{
	PoolUtils::localHeap lh(1024);
	PoolUtils::vector<size_t> vec(lh);
	if (useInitializer)
		vec = PoolUtils::vector<size_t>({ 0,1,2,3,4,5,6 }, lh);
	else {
		for (size_t i = 0; i < 7; i++)
			vec.push_back(i);
	}

	std::vector<size_t> stdVec;
	if (useInitializer)
		stdVec = std::vector<size_t>({ 0,1,2,3,4,5,6 });
	else {
		for (size_t i = 0; i < 7; i++)
			stdVec.push_back(i);
	}

	TEST_EQUAL(vec.size(), 7, "Vec size");
	TEST_EQUAL(stdVec.size(), 7, "stdVec size");
	TEST_EQUAL(vec[3], 3, "Vec pos 3");
	TEST_EQUAL(stdVec[3], 3, "stdVec pos 3");

	auto iter = find(vec.begin(), vec.end(), 3);
	TEST_TRUE(vec.begin() + 3 == iter, "Vec find");

	auto stdIter = find(stdVec.begin(), stdVec.end(), 3);
	TEST_TRUE(stdVec.begin() + 3 == stdIter, "Vec std find");

	vec.insert(iter, 10);
	TEST_TRUE(vec.begin() + 3 == find(vec.begin(), vec.end(), 10), "Find after insert");
	stdVec.insert(stdIter, 10);
	TEST_TRUE(stdVec.begin() + 3 == find(stdVec.begin(), stdVec.end(), 10), "Find after std insert");

	iter = find(vec.begin(), vec.end(), 10);
	vec.insert(iter, { 11, 12 });
	TEST_TRUE(vec.begin() + 3 == find(vec.begin(), vec.end(), 11), "Find after insert");
	TEST_TRUE(vec.begin() + 4 == find(vec.begin(), vec.end(), 12), "Find after insert");
	TEST_TRUE(vec.begin() + 5 == find(vec.begin(), vec.end(), 10), "Find after insert");

	stdIter = find(stdVec.begin(), stdVec.end(), 10);
	stdVec.insert(stdIter, {11, 12});
	TEST_TRUE(stdVec.begin() + 3 == find(stdVec.begin(), stdVec.end(), 11), "Find after insert");
	TEST_TRUE(stdVec.begin() + 4 == find(stdVec.begin(), stdVec.end(), 12), "Find after insert");
	TEST_TRUE(stdVec.begin() + 5 == find(stdVec.begin(), stdVec.end(), 10), "Find after insert");

	size_t s = vec.size();
	iter = find(vec.begin(), vec.end(), 10);
	vec.erase(iter);
	TEST_TRUE(s - 1 == vec.size(), "Vec erase");

	s = stdVec.size();
	stdIter = find(stdVec.begin(), stdVec.end(), 10);
	stdVec.erase(stdIter);
	TEST_TRUE(s - 1 == stdVec.size(), "stdVec erase");

	cout << "testVectorSort pass\n";
	return true;
}

bool TestPoolMemory::testVectorForLoops() {

	PoolUtils::localHeap lh(1024);
	PoolUtils::vector<size_t> vec(lh);
	for (size_t i = 0; i < 20; i++)
		vec.push_back(i);

	size_t i = 0;
	for (size_t val : vec) {
		TEST_EQUAL(val, i++, "Vec for loop");
	}

	for (i = 0; i < vec.size(); i++) {
		TEST_EQUAL(vec[i], i, "Vec for loop");
	}

	i = 0;
	for (auto iter = vec.begin(); iter != vec.end(); iter++) {
		TEST_EQUAL(*iter, i++, "Vec for loop");
	}
	cout << "testVectorForLoops pass\n";
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

