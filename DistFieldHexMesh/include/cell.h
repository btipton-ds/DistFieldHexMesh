#pragma once

#include <tm_vector3.h>
#include <types.h>
#include <objectPool.h>

namespace DFHM {
class Volume;

class Cell : public DataPool {
public:
	enum VolumeType {
		VT_UNKNOWN,
		VT_VOID,
		VT_SOLID,
		VT_FLUID,
	};

	struct TriMeshHitRec {
		inline TriMeshHitRec(double t= -1, size_t triIdx = -1)
			: _t(t)
			, _triIdx(triIdx)
		{
		}
		TriMeshHitRec(const TriMeshHitRec&) = default;
		double _t = -1;
		size_t _vertIdx = -1, _triIdx = -1;
	};

	Cell();
	Cell(const Cell& src) = default;
	void addHit(const Vector3d& cellOrigin, const Vector3d& cellSpan, AxisIndex axisIdx, const RayTriIntersect& hit);
	void makeIntersectionFaces(Volume& vol, size_t cellIdx, const Vector3d& cellOrigin, const Vector3d& cellSpan);

	bool unload(std::ostream& out);
	bool load(std::istream& in);

	// Determines how many minimum sized cells this cell spans. (1,1,1) is a minium sized cell
	// (2,2,2) is a cell that is double span in all axes
	// (1,1,2) is a cell that is minimal span in x and y but double in z
	Vector3<char> _span = Vector3<char>(1, 1, 1);

	std::vector<TriMeshHitRec> _hits[3][2][2]; // x,y,z x (0,1) x (0,1)
	VolumeType _volType = VT_UNKNOWN;
	std::vector<size_t> _polygons; // indices of polygons in this cell
	std::vector<size_t> _polyhedra;// indices of polyedra in this cell
};

}
