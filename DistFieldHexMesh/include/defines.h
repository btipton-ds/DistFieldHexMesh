#pragma once

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

#include <tm_defines.h>

#define RUN_MULTI_THREAD false
#define USE_MULTI_THREAD_CONTAINERS 1
#define SHARP_EDGE_ANGLE_DEGREES 15
#define SHARP_EDGE_ANGLE_RADIANS (SHARP_EDGE_ANGLE_DEGREES * M_PI / 180.0)
#define GRAPHICS_OVER_SAMPLING 2
#define _USE_MATH_DEFINES

#define VERIFY_REDUCED_FINDER 0
#define LOGGING_ENABLED 1
#define DEBUG_BREAKS 0
#define CAN_FREE_TESTS_ENABLED 0
#define LOGGING_VERBOSE_ENABLED (1 && LOGGING_ENABLED)
#define DUMP_BAD_CELL_OBJS 1
#define DUMP_OPEN_CELL_OBJS 1

#if USE_MULTI_THREAD_CONTAINERS
#define MTC MultiCore
#else
#define MTC std
#endif