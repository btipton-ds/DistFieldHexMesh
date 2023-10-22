#pragma once

#include <vector>
#include <memory>
#include <tm_vector3.h>

namespace DFHM {

enum class AxisIndex {
	X, Y, Z
};

struct RayTriIntersect {
	Vector3d _hitPt;
	size_t _triIdx = -1, _blockIdx = -1;
	Vector3i _cellIdx = Vector3i(-1, -1, -1);
};

using RayTriIntersectVec = std::vector<RayTriIntersect>;
using RayBlockIntersectVec = std::vector<std::shared_ptr<RayTriIntersectVec>>;
struct RayHitRec {
	RayBlockIntersectVec _xIntersects, _yIntersects, _zIntersects;
};

}
