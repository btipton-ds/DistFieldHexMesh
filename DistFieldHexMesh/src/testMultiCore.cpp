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
#include <testMultiCore.h>
#include <MultiCoreUtil.h>

using namespace std;
using namespace DFHM;

bool TestMultiCore::testAll()
{
	if (!test0(4)) return false;
	if (!test0(8)) return false;
	if (!test0(16)) return false;
	if (!test0(-1)) return false;

	cout << "TestMultiCore pass\n";
	return true;
}

bool TestMultiCore::test0(size_t numCores)
{
	MultiCore::ThreadPool tp(numCores);

	std::vector<std::vector<size_t>> vv;
	vv.resize(tp.getNumThreads());
	for (size_t i = 0; i < tp.getNumThreads(); i++) {
		tp.run(100, [i, &vv](size_t threadNum, size_t idx) {
			auto& v = vv[threadNum];
			for (size_t i = 0; i < idx; i++) {
				v.push_back(i);
			}
		});
	}

	return true;
}
