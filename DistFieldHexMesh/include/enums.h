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

	Copyright Robert R Tipton, 2022, all rights reserved except those granted in prior license statement.

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

enum VolumeType {
	VOLTYPE_UNKNOWN,
	VOLTYPE_VOID,
	VOLTYPE_VOID1,
	VOLTYPE_VOID2,
	VOLTYPE_SOLID,
	VOLTYPE_SOLID1,
	VOLTYPE_SOLID2,
	VOLTYPE_INTERSECTING,
};

enum TopolgyState
{
				// As originally created without any splits
			// Was intact, now there is a split version and this is for reference
};

enum HexFaceType {
	CFT_BOTTOM,
	CFT_TOP,
	CFT_LEFT,
	CFT_RIGHT,
	CFT_BACK,
	CFT_FRONT,
	CFT_UNDEFINED,
};

enum MeshDrawType {
	MDT_ERROR_WALL,
	MDT_INNER,

	MDT_BOTTOM,
	MDT_TOP,
	MDT_LEFT,
	MDT_RIGHT,
	MDT_BACK,
	MDT_FRONT,

	MDT_MESH_LAYER_0,
	MDT_MESH_LAYER_1,
	MDT_MESH_LAYER_2,
	MDT_MESH_LAYER_3,
	MDT_MESH_LAYER_4,

	MDT_MESH_SELECTED,

	MDT_ALL,
};

enum IntersectionDrawType {
	IDT_INTERSECTION_X,
	IDT_INTERSECTION_Y,
	IDT_INTERSECTION_Z,

	IDT_ALL,
};

// DrawStates must be universally unique across all entities because they are keys
// into the renderer's VBO map.
enum DrawStates {
    DS_MODEL_FACES_SOLID,
	DS_MODEL_FACES_SURFACE,
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
	DS_MESH_LAYER_4,

	DS_MESH_SELECTED,

	DS_INTERSECTION_X,
	DS_INTERSECTION_Y,
	DS_INTERSECTION_Z,

	DS_INTERSECTION_RADIUS_X,
	DS_INTERSECTION_RADIUS_Y,
	DS_INTERSECTION_RADIUS_Z,

	DS_MODEL_ALL,
	DS_MESH_ALL,
	DS_INTERSECTION_ALL,
	DS_INTERSECTION_RADII_ALL,
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

enum CellType {
	CT_HEX,
	CT_WEDGE,
	CT_PYRAMID,
	CT_TETRAHEDRON,
	CT_UNKNOWN,
};

enum SplitParity {
	SP_0 = 0,
	SP_1 = 1,
};

}
