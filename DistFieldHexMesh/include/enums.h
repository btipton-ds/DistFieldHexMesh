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

#define LOGGING_ENABLED 1
namespace DFHM {

enum Dir {
	Positive = 1,
	Negetive = -1
};


enum Trinary
{
	IS_UNKNOWN,
	IS_TRUE,
	IS_FALSE
};

/*

enum TopolgyState
{
	TS_UNKNOWN         = 0,
	TS_ALL_CLEAN       = 1, // Has no split edges or faces
	TS_HAS_SPLIT_EDGES = 2, // TODO Probably never needed and can be deleted
	TS_HAS_SPLIT_FACES = 4,
	TS_PARTIALLY_SPLIT = 8, // Has split edges or faces
	TS_REFERENCE       = 16, // Has been split and no longer in use. It's kept as reference for working on it's sub topology
};
*/

enum FaceType {
	FT_OUTER,
	FT_INNER,
	FT_LAYER_BOUNDARY,
	FT_BLOCK_BOUNDARY,
	FT_ALL,
};

enum DrawStates {
    DS_MODEL = 0,
	DS_MODEL_CURVATURE = 1,
	DS_MODEL_SHARP_EDGES = 2,
    DS_MODEL_SHARP_VERTS = 3,
    DS_MODEL_NORMALS = 4,
    DS_BLOCK_MESH = 10,
};

enum DrawSubStates {
    DSS_OUTER = 0,
    DSS_INNER = 1,
	DSS_LAYER_BOUNDARY = 2,
	DSS_BLOCK_BOUNDARY = 3,
};


}
