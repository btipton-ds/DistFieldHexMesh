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
#include <triMesh.h>
#include <volume.h>
#include <blockTest.h>
#include <MultiCoreUtil.h>
#include <vertex.h>

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

int main(int numParams, const char** params)
{
	testBlock();

	return 0;
}

