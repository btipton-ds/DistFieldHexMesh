#pragma once

#include <dataPool.h>

namespace DFHM {

class Cell : public DataPool {
public:
	enum VolumeType {
		VT_UNKNOWN,
		VT_VOID,
		VT_SOLID,
		VT_FLUID,
	};

	bool unload(std::ostream& out);
	bool load(std::istream& in);

	VolumeType volType = VT_UNKNOWN;
	std::vector<ObjectPoolId> _polygons; // indices of polygons in this cell
	std::vector<ObjectPoolId> _polyhedra;// indices of polyedra in this cell
};

}
