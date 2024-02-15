#pragma once

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
