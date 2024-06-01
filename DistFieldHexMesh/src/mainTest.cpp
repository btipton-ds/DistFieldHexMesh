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
#include <cstdlib>

#include <defines.h>
#include <tests.h>
#include <triMesh.h>
#include <volume.h>
#include <blockTest.h>
#include <MultiCoreUtil.h>
#include <vertex.h>
#include <pool_vector.h>
#include <pool_set.h>
#include <pool_map.h>

using namespace std;
using namespace DFHM;

#define MULTI_CORE_TESTS_ENABLED 1

namespace
{
struct Dummy {
	Dummy()
	{
	}

	Dummy(size_t val)
		: _val(val)
	{
	}

	Dummy(const Dummy& src)
		: _val(src._val)
	{
	}

	~Dummy()
	{
		_val = -1;
	}

	inline bool operator < (const Dummy& rhs) const
	{
		return _val < rhs._val;
	}

	inline bool operator == (const Dummy& rhs) const
	{
		return _val == rhs._val;
	}

	size_t _val = -1;
};

class DummyVec
{
public:
	DummyVec() = default;
	DummyVec(size_t size)
		: _id(size)
	{
		for (int i = 0; i < size; i++)
			_data.push_back(i);
	}

	void clear() const
	{
		_data.clear();
	}

	bool operator < (const DummyVec& rhs) const
	{
		return _id < rhs._id;
	}

	size_t size() const
	{
		return _id;
	}
private:
	size_t _id = 0;
	mutable MultiCore::vector<Dummy> _data;
};

}

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
	bool testAllocator1();

	bool testVector();
	bool testVectorSizeT();
	bool testVectorVectorSizeT();
	bool testVectorInsert();
	bool testVectorSort();
	bool testVectorInsertErase(bool useInitializer);
	bool testVectorForLoops();
	bool testVectorMisc();
	bool memoryStressTest();

	bool testSet();
	bool testSetInsertErase();
	bool testHeavySetInsertErase();

	bool testMap();
	bool testMapBasic();
	bool testMapHeavy();
};

bool TestPoolMemory::testAll()
{
	if (!testAllocator()) return false;

	if (!testVector()) return false;

	if (!testSet()) return false;

	if (!testMap()) return false;

	cout << "testAll pass\n";
	return true;
}

bool TestPoolMemory::testAllocator()
{
	if (!testAllocator0()) return false;
	if (!testAllocator1()) return false;

	cout << "TestAllocator pass\n";

	return true;
}

bool TestPoolMemory::testAllocator0()
{
	MultiCore::local_heap alloc(128);
	double* pD = alloc.alloc<double>(1);
	TEST_TRUE(pD != nullptr, "Failed to allocate pointer");
	*pD = 1.0;
	alloc.free(pD);

	double* pD2 = alloc.alloc<double>(1);

	TEST_TRUE(pD2 != nullptr, "Failed to allocate pointer");
//	TEST_EQUAL(pD, pD2, "Delete and allocate pointer at same address");

	*pD2 = 1.0;
	alloc.free(pD2);

	pD = alloc.alloc<double>(3);
	TEST_TRUE(pD != nullptr, "Failed to allocate pointer");
	pD[0] = 1.0;
	pD[1] = 2.0;
	pD[2] = 3.0;
	alloc.free(pD);

	pD2 = alloc.alloc<double>(3);

	TEST_TRUE(pD2 != nullptr, "Failed to allocate pointer");
//	TEST_EQUAL(pD, pD2, "Delete and allocate pointer at same address");

	pD2[0] = 1.0;
	pD2[1] = 2.0;
	pD2[2] = 3.0;
	alloc.free(pD2);

	TEST_TRUE(MultiCore::local_heap::getThreadHeapPtr()->verify(), "Verify heap");
	return true;
}

bool TestPoolMemory::testAllocator1()
{
	MultiCore::local_heap heap(2048, 64);
	MultiCore::local_heap::scoped_set_thread_heap st(&heap);

	for (size_t i = 0; i < 1000; i ++) {
		size_t num = 5 + (size_t)((95 * std::rand() / (double)RAND_MAX) + 0.5);
		auto p = heap.alloc<Dummy>(num);
		heap.free(p);
	}

	for (size_t i = 0; i < 1000; i++) {
		size_t num = 5 + (size_t)((95 * std::rand() / (double)RAND_MAX) + 0.5);
		auto p = heap.alloc<Dummy>(num);
		heap.free(p);
	}

	TEST_TRUE(MultiCore::local_heap::getThreadHeapPtr()->verify(), "Verify heap");
	return true;
}

bool TestPoolMemory::testVector()
{
	if (!testVectorSizeT()) return false;
	if (!testVectorInsert()) return false;
	if (!testVectorSort()) return false;
	if (!testVectorInsertErase(true)) return false;
	if (!testVectorInsertErase(false)) return false;
	if (!testVectorForLoops()) return false;
	if (!testVectorMisc()) return false;
	if (!testVectorVectorSizeT()) return false;

#if MULTI_CORE_TESTS_ENABLED
	MultiCore::runLambda([this](size_t threadNum, size_t numThreads) {
		MultiCore::local_heap alloc(1024);
		MultiCore::local_heap::scoped_set_thread_heap st(&alloc);
		if (!testVectorSizeT())
			return false;
		return true;
	}, true);

	MultiCore::runLambda([this](size_t threadNum, size_t numThreads) {
		MultiCore::local_heap alloc(1024);
		MultiCore::local_heap::scoped_set_thread_heap st(&alloc);
		if (!testVectorVectorSizeT())
			return false;
		return true;
		}, true);

	MultiCore::runLambda([this](size_t threadNum, size_t numThreads) {
		MultiCore::local_heap alloc(1024);
		MultiCore::local_heap::scoped_set_thread_heap st(&alloc);
		if (!testVectorInsert())
			return false;
		return true;
	}, true);

	MultiCore::runLambda([this](size_t threadNum, size_t numThreads) {
		MultiCore::local_heap alloc(1024);
		MultiCore::local_heap::scoped_set_thread_heap st(&alloc);
		if (!testVectorSort())
			return false;
		return true;
	}, true);

	MultiCore::runLambda([this](size_t threadNum, size_t numThreads) {
		MultiCore::local_heap alloc(1024);
		MultiCore::local_heap::scoped_set_thread_heap st(&alloc);
		if (!testVectorInsertErase(true))
			return false;
		return true;
	}, true);

	MultiCore::runLambda([this](size_t threadNum, size_t numThreads) {
		MultiCore::local_heap alloc(1024);
		MultiCore::local_heap::scoped_set_thread_heap st(&alloc);
		if (!testVectorInsertErase(false))
			return false;
		return true;
	}, true);

	MultiCore::runLambda([this](size_t threadNum, size_t numThreads) {
		MultiCore::local_heap alloc(1024);
		MultiCore::local_heap::scoped_set_thread_heap st(&alloc);
		if (!testVectorForLoops())
			return false;
		return true;
	}, true);

	MultiCore::runLambda([this](size_t threadNum, size_t numThreads) {
		MultiCore::local_heap alloc(1024);
		MultiCore::local_heap::scoped_set_thread_heap st(&alloc);
		if (!testVectorMisc())
			return false;
		return true;
	}, true);

#endif

	if (!memoryStressTest()) return false;

	cout << "Test vector pass\n";
	return true;
}

bool TestPoolMemory::testVectorSizeT()
{
	MultiCore::vector<Dummy> vec;

	size_t size = 2048;
	TEST_TRUE(vec.empty(), "vec empty");

	for (size_t i = 0; i < size; i++) {
		vec.push_back(i);
	}

	TEST_FALSE(vec.empty(), "vec not empty failed");
	TEST_EQUAL(vec.size(), size, "vec size");

	for (size_t i = 0; i < size; i++)
		vec.pop_back();

	TEST_TRUE(vec.empty(), "vec empty");
	TEST_EQUAL(vec.size(), 0, "vec size");

	TEST_TRUE(MultiCore::local_heap::getThreadHeapPtr()->verify(), "Verify heap");
	return true;
}

bool TestPoolMemory::testVectorVectorSizeT()
{
	MultiCore::local_heap alloc(1024);
	MultiCore::local_heap::scoped_set_thread_heap st(&alloc);
	MultiCore::vector<MultiCore::vector<Dummy>> vec;

	size_t size0 = 1000;
	size_t size1 = 500;
	TEST_TRUE(vec.empty(), "vec empty");

	for (size_t i = 0; i < size0; i++) {
		vec.push_back(MultiCore::vector<Dummy>());
		auto& subVec = vec.back();
		for (size_t j = 0; j < size1; j++) {
			if (i == 0 && j == 0) {
				int dbgBreak = 1;
			}
			subVec.push_back(j);
		}
	}

	TEST_FALSE(vec.empty(), "vec not empty failed");
	TEST_EQUAL(vec.size(), size0, "vec size");

	for (size_t i = 0; i < size0; i++)
		vec.pop_back();

	TEST_TRUE(vec.empty(), "vec empty");
	TEST_EQUAL(vec.size(), 0, "vec size");

	TEST_TRUE(MultiCore::local_heap::getThreadHeapPtr()->verify(), "Verify heap");
	return true;
}

bool TestPoolMemory::testVectorInsert()
{
	MultiCore::vector<size_t> vec({ 0, 1, 2, 3 });
	std::vector<size_t> stdVec({ 0, 1, 2, 3 });

	TEST_EQUAL(vec.size(), 4, "vec size");
	TEST_EQUAL(vec.size(), stdVec.size(), "vec size");

	for (size_t i = 0; i < vec.size(); i++) {
		TEST_EQUAL(vec[i], i, "vec value");
		TEST_EQUAL(vec[i], stdVec[i], "vec value");
	}

	vec.insert(vec.end(), {4, 5, 6, 7});
	stdVec.insert(stdVec.end(), { 4, 5, 6, 7 });

	TEST_EQUAL(vec.size(), 8, "vec size");
	TEST_EQUAL(vec.size(), stdVec.size(), "vec size");

	for (size_t i = 0; i < vec.size(); i++) {
		TEST_EQUAL(vec[i], i, "vec value");
		TEST_EQUAL(vec[i], stdVec[i], "vec value");
	}

	vec.insert(vec.begin() + 3, { 8, 9, 10, 11 });
	stdVec.insert(stdVec.begin() + 3, { 8, 9, 10, 11 });

	TEST_EQUAL(vec.size(), 12, "vec size");
	TEST_EQUAL(vec.size(), stdVec.size(), "vec size");

	for (size_t i = 0; i < vec.size(); i++) {
		TEST_EQUAL(vec[i], stdVec[i], "vec value");
	}

	{
		MultiCore::vector<size_t> vec;
		std::set<size_t> set;
		for (size_t i = 0; i < 20; i++)
			set.insert(i);
		vec.insert(vec.end(), set.begin(), set.end());

		TEST_EQUAL(vec.size(), set.size(), "vec set same size");

		auto sIter = set.begin();
		auto vIter = vec.begin();
		while (sIter != set.end() && vIter != vec.end()) {
			const auto sVal = *sIter++;
			const auto vVal = *vIter++;
			TEST_EQUAL(sVal, vVal, "vec set same value");
		}
	}

	TEST_TRUE(MultiCore::local_heap::getThreadHeapPtr()->verify(), "Verify heap");
	return true;
}

bool TestPoolMemory::testVectorSort()
{
	MultiCore::vector<size_t> vec({ 7, 4, 5, 3, 1, 6, 0, 2 });
	std::vector<size_t> stdVec({ 7, 4, 5, 3, 1, 6, 0, 2 });

	TEST_EQUAL(vec.size(), 8, "vec size");
	TEST_EQUAL(vec.size(), stdVec.size(), "vec size");
	for (size_t i = 0; i < vec.size(); i++) {
		TEST_EQUAL(vec[i], stdVec[i], "vec size");
	}

	auto sb = stdVec.begin();
	auto sbP = sb + stdVec.size() - 1;
	auto sbP2 = sb + stdVec.size();
	auto se = stdVec.end();
	std::sort(stdVec.begin(), stdVec.end());
	std::sort(vec.begin(), vec.end());

	for (size_t i = 0; i < vec.size(); i++) {
		TEST_EQUAL(vec[i], stdVec[i], "vec size");
	}

	std::sort(stdVec.begin(), stdVec.end(), std::greater<>());
	std::sort(vec.begin(), vec.end(), std::greater<>());

	for (size_t i = 0; i < vec.size(); i++) {
		TEST_EQUAL(vec[i], stdVec[i], "vec size");
	}

	TEST_TRUE(MultiCore::local_heap::getThreadHeapPtr()->verify(), "Verify heap");
	return true;
}

bool TestPoolMemory::testVectorInsertErase(bool useInitializer)
{
	{
		MultiCore::vector<size_t> vec0;
		std::vector<size_t> stdVec0;

		for (size_t i = 0; i < 10; i++) {
			vec0.insert(vec0.begin(), i);
			stdVec0.insert(stdVec0.begin(), i);
		}

		TEST_EQUAL(vec0.size(), 10, "Vec0 size");
		for (size_t i = 0; i < vec0.size(); i++) {
			TEST_EQUAL(vec0[i], stdVec0[i], "vec0 size");
		}
	}

	MultiCore::vector<size_t> vec;
	if (useInitializer)
		vec = MultiCore::vector<size_t>({ 0,1,2,3,4,5,6 });
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
	stdVec.insert(stdIter, { 11, 12 });
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

	for (size_t i = 0; i < vec.size(); i++) {
		TEST_EQUAL(vec[i], stdVec[i], "vec size");
	}

	size_t preSize = vec.size();
	iter = find(vec.begin(), vec.end(), 4);
	TEST_TRUE(iter != vec.end(), "Find succeed");
	vec.erase(iter);
	TEST_TRUE(vec.size() == preSize - 1, "Erase in list succeed");

	preSize = vec.size();
	iter = find(vec.begin(), vec.end(), 100);
	TEST_TRUE(iter == vec.end(), "Find fail");
	vec.erase(iter);
	TEST_TRUE(vec.size() == preSize, "Erase not in list succeed");

	{
		MultiCore::vector<size_t> vec ({ 0, 1, 2, 2, 2, 3, 4, 5, 6 });

		preSize = vec.size();
		vec.erase(vec.begin() + 2, vec.begin() + 4);
		TEST_TRUE(vec.size() == preSize - 2, "Erase multiple");

		for (size_t i = 0; i < vec.size(); i++) {
			TEST_EQUAL(vec[i], i, "vec size");
		}
	}

	TEST_TRUE(MultiCore::local_heap::getThreadHeapPtr()->verify(), "Verify heap");
	return true;
}

bool TestPoolMemory::testVectorForLoops() {
	MultiCore::vector<size_t> vec;
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

	i = vec.size() - 1;
	for (auto iter = vec.rbegin(); iter != vec.rend(); iter++) {
		TEST_EQUAL(*iter, i--, "Vec for loop");
	}

	TEST_TRUE(MultiCore::local_heap::getThreadHeapPtr()->verify(), "Verify heap");
	return true;
}

bool TestPoolMemory::testVectorMisc() {
	MultiCore::vector<size_t> vec;

	TEST_EQUAL(vec.size(), 0, "Test size() == 0");

	vec.push_back(1);
	TEST_EQUAL(vec.size(), 1, "Test size() == 1");
	TEST_EQUAL(vec.front(), 1, "Test front()");
	TEST_EQUAL(vec.back(), 1, "Test front()");

	vec.push_back(2);
	TEST_EQUAL(vec.size(), 2, "Test size() == 2");
	TEST_EQUAL(vec.front(), 1, "Test front()");
	TEST_EQUAL(vec.back(), 2, "Test front()");

	vec.pop_back();
	TEST_EQUAL(vec.front(), 1, "Test front()");
	TEST_EQUAL(vec.size(), 1, "Test size() == 1");

	vec.pop_back();
	TEST_EQUAL(vec.size(), 0, "Test size() == 1");
	TEST_TRUE(vec.empty(), "Test empty()");

	{
		MultiCore::vector<size_t> vec({ 0, 1, 2, 3 });
		MultiCore::vector<size_t> vec2({ 4, 5, 6, 7 });
		for (size_t i = 0; i < vec.size(); i++) {
			TEST_EQUAL(vec[i], i, "vec copy entry");
			TEST_EQUAL(vec2[i], i + 4, "vec copy entry");
		}

		vec = vec2;
		for (size_t i = 0; i < vec.size(); i++) {
			TEST_EQUAL(vec[i], i + 4, "vec copy entry");
			TEST_EQUAL(vec2[i], i + 4, "vec copy entry");
		}
	}
	TEST_TRUE(MultiCore::local_heap::getThreadHeapPtr()->verify(), "Verify heap");
	return true;
}

bool TestPoolMemory::memoryStressTest()
{
	MultiCore::runLambda([](size_t threadNum, size_t numThreads) {
		const size_t steps = 1000;
		MultiCore::local_heap heap(2048);
		MultiCore::vector<MultiCore::vector<size_t>> vec;
		vec.reserve(100);
		for (size_t i = 0; i < steps; i++) {
			vec.push_back(MultiCore::vector<size_t>());
			auto& v = vec.back();
			for (size_t j = 0; j < 100; j++) {
				v.push_back(j);
			}
		}

		std::sort(vec.back().begin(), vec.back().end(), std::greater<>());
		std::sort(vec.back().begin(), vec.back().end());
	}, false);

	TEST_TRUE(MultiCore::local_heap::getThreadHeapPtr()->verify(), "Verify heap");
	return true;
}

bool TestPoolMemory::testSet()
{
	if (!testSetInsertErase()) return false;

	if (!testHeavySetInsertErase()) return false;

#if MULTI_CORE_TESTS_ENABLED
	MultiCore::runLambda([this](size_t threadNum, size_t numThreads) {
		MultiCore::local_heap alloc(1024);
		MultiCore::local_heap::scoped_set_thread_heap st(&alloc);
		if (!testSetInsertErase())
			return false;
		return true;
	}, true);

	MultiCore::runLambda([this](size_t threadNum, size_t numThreads) {
		MultiCore::local_heap alloc(1024);
		MultiCore::local_heap::scoped_set_thread_heap st(&alloc);
		if (!testHeavySetInsertErase())
			return false;
		return true;
	}, true);
#endif

	cout << "Test set pass\n";
	return true;
}

bool TestPoolMemory::testSetInsertErase()
{
	std::vector<size_t> ref({ 7, 5, 44, 9, 23, 2, 17, 3, 1, 0, 100 });
	MultiCore::set<size_t> s;
	for (size_t v : ref)
		s.insert(v);

	sort(ref.begin(), ref.end());
	TEST_EQUAL(ref.size(), s.size(), "Verify set size");

	size_t i = 0;
	for (auto iter = s.begin(); iter != s.end(); iter++)
		TEST_EQUAL(*iter, ref[i++], "Verify set entry");

	auto iter = s.find(17);
	TEST_TRUE(iter != s.end(), "Find in set");

	iter = s.find(4);
	TEST_TRUE(iter == s.end(), "Find not in set");

	iter = s.find(23);
	TEST_TRUE(iter != s.end(), "Find in set");
	s.erase(iter);
	iter = s.find(23);
	TEST_TRUE(iter == s.end(), "Find not in set");

	TEST_TRUE(s.size() == ref.size() - 1, "Found item removed from set");

	iter = s.find(77);
	TEST_TRUE(iter == s.end(), "Find not in set");
	s.erase(iter);

	TEST_TRUE(s.size() == ref.size() - 1, "Erase item not in set");

	TEST_TRUE(MultiCore::local_heap::getThreadHeapPtr()->verify(), "Verify heap");
	return true;
}

bool TestPoolMemory::testHeavySetInsertErase()
{
	MultiCore::set<DummyVec> s;
	for (size_t i = 10; i != 0; i--) {
		s.insert(DummyVec(i));
	}

	for (auto& v : s) {
		v.clear();
	}

	TEST_TRUE(MultiCore::local_heap::getThreadHeapPtr()->verify(), "Verify heap");
	return true;
}

bool TestPoolMemory::testMap()
{
	if (!testMapBasic()) return false;
	if (!testMapHeavy()) return false;

#if MULTI_CORE_TESTS_ENABLED
	MultiCore::runLambda([this](size_t threadNum, size_t numThreads) {
		MultiCore::local_heap alloc(1024);
		MultiCore::local_heap::scoped_set_thread_heap st(&alloc);
		if (!testMapBasic())
			return false;
		return true;
	}, true);

	MultiCore::runLambda([this](size_t threadNum, size_t numThreads) {
		MultiCore::local_heap alloc(1024);
		MultiCore::local_heap::scoped_set_thread_heap st(&alloc);
		if (!testMapHeavy())
			return false;
		return true;
	}, true);
#endif
	cout << "Test map pass\n";
	return true;
}

bool TestPoolMemory::testMapBasic()
{
	MultiCore::map<size_t, Dummy> m;
	TEST_TRUE(m.empty(), "New map empty");
	TEST_EQUAL(m.size(), 0, "New size 0");
	for (size_t i = 0; i < 20; i++) {
		auto iter = m.insert(MultiCore::make_pair(i, Dummy(i + 1))).first;
		TEST_TRUE(iter != m.end(), "insert iter test");
		TEST_EQUAL(m.size(), i + 1, "Size");
		TEST_EQUAL(iter->second._val, i + 1, "Size");
	}
	for (size_t i = m.size() - 1; i != -1; i--) {
		auto iter = m.find(i);
		TEST_EQUAL(iter->first, i, "first match");
		TEST_EQUAL(iter->second._val, i + 1, "second match");
	}

	size_t i = 0;
	for (auto iter = m.begin(); iter != m.end(); iter++) {
		auto& pair = *iter;
		TEST_EQUAL(iter->first, i, "first match");
		TEST_EQUAL(iter->second._val, i + 1, "second match");
		i++;
	}

	i = 0;
	for (auto pair : m) {
		TEST_EQUAL(pair.first, i, "first match");
		TEST_EQUAL(pair.second._val, i + 1, "second match");
		i++;
	}

	while (!m.empty()) {
		size_t s = m.size();
		m.erase(m.begin());
		TEST_EQUAL(m.size(), s - 1, "Size");
	}
	TEST_EQUAL(m.size(), 0, "Size");

	return true;
}

bool TestPoolMemory::testMapHeavy()
{
	MultiCore::map<size_t, DummyVec> m;
	TEST_TRUE(m.empty(), "New map empty");
	TEST_EQUAL(m.size(), 0, "New size 0");
	for (size_t i = 0; i < 200; i++) {
		size_t vecSize = 100 + 2 * i;
		auto iter = m.insert(MultiCore::make_pair(i, DummyVec(vecSize))).first;
		TEST_TRUE(iter != m.end(), "insert iter test");
		TEST_EQUAL(m.size(), i + 1, "Size");
		TEST_EQUAL(iter->second.size(), vecSize, "Size");
	}

	for (size_t i = m.size() - 1; i != -1; i--) {
		size_t vecSize = 100 + 2 * i;
		auto iter = m.find(i);
		TEST_EQUAL(iter->first, i, "first match");
		TEST_EQUAL(iter->second.size(), vecSize, "second match");
	}

	size_t i = 0;
	for (auto iter = m.begin(); iter != m.end(); iter++) {
		size_t vecSize = 100 + 2 * i;
		auto& pair = *iter;
		TEST_EQUAL(iter->first, i, "first match");
		TEST_EQUAL(iter->second.size(), vecSize, "second match");
		i++;
	}

	i = 0;
	for (auto pair : m) {
		size_t vecSize = 100 + 2 * i;
		TEST_EQUAL(pair.first, i, "first match");
		TEST_EQUAL(pair.second.size(), vecSize, "second match");
		i++;
	}

	while (!m.empty()) {
		size_t s = m.size();
		m.erase(m.begin());
		TEST_EQUAL(m.size(), s - 1, "Size");
	}
	TEST_EQUAL(m.size(), 0, "Size");

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

