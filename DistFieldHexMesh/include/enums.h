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

enum MeshType {
	MT_OUTER,
	MT_INNER,
	MT_BLOCK_BOUNDARY,
	MT_BOUNDARY,
	MT_ALL,
};

enum DrawStates {
    DS_MODEL = 0,
    DS_MODEL_SHARP_EDGES = 1,
    DS_MODEL_SHARP_VERTS = 2,
    DS_MODEL_NORMALS = 3,
    DS_BLOCK_MESH = 10,
};
enum DrawSubStates {
    DSS_OUTER = 0,
    DSS_INNER = 1,
    DSS_BLOCK_BOUNDARY = 2,
};

}
