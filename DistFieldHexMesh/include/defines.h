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

#define _USE_MATH_DEFINES

#ifdef WIN32
#define WIN32_LEAN_AND_MEAN // This suppresses some ambiguous names
#endif

#include <tm_defines.h>
#include <math.h>

#define RUN_MULTI_THREAD 1
#define RUN_MULTI_SUB_THREAD 1
#define ENABLE_BACKGROUND_PROCESSING 1
#define USE_REFINER 0 // Refiner is working, but slows things down by about 2.5 times.
#define COALESCE_POINTS_BEFORE_LOOPING 0
#define ENABLE_DEBUGGING_MUTEXES (0 && _DEBUG && (RUN_MULTI_THREAD || RUN_MULTI_SUB_THREAD))
#define ENABLE_VERIFY_THREAD_AND_BLOCK_IDX_MATCH 0
#define ENABLE_MODEL_SEARCH_TREE_VERIFICATION 0
#define FAST_BISECTION_VALIDATION_ENABLED 0
#define INCLUDE_DEBUG_WX_FRAME 0
#define SHARP_EDGE_ANGLE_DEGREES 15
#define SHARP_EDGE_ANGLE_RADIANS (SHARP_EDGE_ANGLE_DEGREES * M_PI / 180.0)
#define GRAPHICS_OVER_SAMPLING 1

#define OBJECT_POOL_USE_STD_MAP 1

#define MAX_SUB_TREE_COUNT (1 * 1024)
#define SUB_TREE_SPLIT_RATIO 4
#define OBJECT_TREE_WIDTH 200
#define VERIFY_REDUCED_FINDER 0
#define LOGGING_ENABLED 1
#define DEBUG_BREAKS 0
#define CAN_FREE_TESTS_ENABLED 0
#define LOGGING_VERBOSE_ENABLED (1 && LOGGING_ENABLED)
#define DUMP_BAD_CELL_OBJS 1
#define DUMP_OPEN_CELL_OBJS 1
#define VALIDATION_ON 0

#define USE_MULTI_THREAD_CONTAINERS 0 
#if USE_MULTI_THREAD_CONTAINERS
// Combined, local_heap and ThreadPool drop a base case from 2.7 sec to 2.0 sec. In heavier cases, it drops from minutes to seconds.
// This was a complicated and unproductive experiment to make unique
// It did speed up processing, but garbage collection isn't working and memory usages expladed.
// Not actually a leak because because it's deleted on exit
// heaps for each threat to avoid mutex locks.
// Keeping in case it's worth resurrecting
#define MTC MultiCore
#else
#define MTC std
#endif