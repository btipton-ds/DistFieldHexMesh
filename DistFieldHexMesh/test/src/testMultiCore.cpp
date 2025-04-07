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

#include <iostream>
#include <defines.h>
#include <tests.h>
#include <testMultiCore.h>
#include <MultiCoreUtil.h>

#ifdef _WIN32
#include <windows.h>
#include <profileapi.h>
#endif // _WIN32

using namespace std;
using namespace DFHM;

bool TestMultiCore::testAll()
{
	if (!testSpeed(-1)) return false;

	if (!test0(4)) return false;
	if (!test0(8)) return false;
	if (!test0(16)) return false;
	if (!test0(-1)) return false;

	cout << "TestMultiCore pass\n";
	return true;
}

bool TestMultiCore::test0(size_t numCores)
{
	for (size_t i = 0; i < 10; i++) {
		MultiCore::ThreadPool tp(numCores, numCores);

		std::vector<std::vector<size_t>> vv;
		vv.resize(tp.getNumAllocatedThreads());
		for (size_t i = 0; i < tp.getNumAllocatedThreads(); i++) {
			tp.run(512, [i, &vv](size_t threadNum, size_t idx)->bool {
				auto& v = vv[threadNum];
				for (size_t i = 0; i < idx + 5; i++) {
					v.push_back(i);
				}
				return true;
			}, true);
		}
	}
	return true;
}

bool TestMultiCore::testSpeed(size_t numCores)
{
	MultiCore::ThreadPool tp(numCores, numCores);

#ifdef _WIN32
	LARGE_INTEGER startCount, endCount, freq;
	QueryPerformanceFrequency(&freq);
	QueryPerformanceCounter(&startCount);

	size_t size = 1025 * 1024 * 1024;
	vector<size_t> v;
	v.resize(tp.getNumAllocatedThreads());
	tp.run(size, [&v](size_t threadNum, size_t idx)->bool {
		v[threadNum]++;
		return true;
	}, true);

	QueryPerformanceCounter(&endCount);
	double deltaT = (endCount.QuadPart - startCount.QuadPart) / (double)(freq.QuadPart);
	double rateUS = (1.0e6 * deltaT / size);
//	cout << "Time for testSpeed: " << rateUS << " us/run\n";

#if _DEBUG
	TEST_TRUE(rateUS < 0.008, "debug speed too slow");
#else
	TEST_TRUE(rateUS < 0.003, "release speed too slow");
#endif

	startCount = endCount;

	size_t check = 0;
	for (size_t i = 0; i < v.size(); i++)
		check += v[i];

	TEST_EQUAL(check, size, "testSpeed size match");

#endif // _WIN32
	return true;
}
