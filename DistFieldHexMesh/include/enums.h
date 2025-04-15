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
#include "defines.h"

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

enum TopolgyState
{
				// As originally created without any splits
			// Was intact, now there is a split version and this is for reference
};

enum CubeFaceType {
	CFT_BOTTOM,
	CFT_TOP,
	CFT_LEFT,
	CFT_RIGHT,
	CFT_BACK,
	CFT_FRONT,
	CFT_UNDEFINED,

};

enum FaceDrawType {
	FT_ERROR_WALL,
	FT_INNER,

	FT_BOTTOM,
	FT_TOP,
	FT_LEFT,
	FT_RIGHT,
	FT_BACK,
	FT_FRONT,

	FT_MESH_LAYER_0,
	FT_MESH_LAYER_1,
	FT_MESH_LAYER_2,
	FT_MESH_LAYER_3,
	FT_MESH_LAYER_4,
	FT_MESH_LAYER_5,
	FT_MESH_LAYER_6,
	FT_MESH_LAYER_7,
	FT_MESH_LAYER_8,
	FT_MESH_LAYER_9,

	FT_MESH_SELECTED,

	FT_ALL,
};

enum DrawStates {
    DS_MODEL_FACES,
	DS_MODEL_EDGES,
	DS_MODEL_SHARP_EDGES,
	DS_MODEL_SMOOTH_EDGES,
	DS_MODEL_SHARP_VERTS,
    DS_MODEL_NORMALS,

	DS_MESH_FACES,
	DS_MESH_EDGES,
	DS_MESH_WALL,		// These are created by cutting mesh and produce real, non-slip wall boundaries.
	DS_MESH_ERROR_WALL, // These are wall faces which were NOT produced by cutting cells with mesh to produce real walls.
	DS_MESH_INNER,

	DS_MESH_BOTTOM,
	DS_MESH_TOP,
	DS_MESH_LEFT,
	DS_MESH_RIGHT,
	DS_MESH_BACK,
	DS_MESH_FRONT,

	DS_MESH_LAYER_0, DS_MESH_LAYER_0_OPAQUE,
	DS_MESH_LAYER_1, DS_MESH_LAYER_1_OPAQUE,
	DS_MESH_LAYER_2, DS_MESH_LAYER_2_OPAQUE,
	DS_MESH_LAYER_3, DS_MESH_LAYER_3_OPAQUE,
	DS_MESH_LAYER_4, DS_MESH_LAYER_4_OPAQUE,
	DS_MESH_LAYER_5, DS_MESH_LAYER_5_OPAQUE,
	DS_MESH_LAYER_6, DS_MESH_LAYER_6_OPAQUE,
	DS_MESH_LAYER_7, DS_MESH_LAYER_7_OPAQUE,
	DS_MESH_LAYER_8, DS_MESH_LAYER_8_OPAQUE,
	DS_MESH_LAYER_9,

	DS_MESH_SELECTED,

	DS_MESH_ALL,
};

enum VertexLockType {
	VLT_NONE,
	VLT_ALL_AXES,
	VLT_MODEL_MESH,
};

enum Index3IdUserFlags {
	UF_FACE_REVERSED = 1,
	UF_UNUSED1 = 2,
	UF_UNUSED2 = 4,
	UF_UNUSED3 = 8,
};

}
