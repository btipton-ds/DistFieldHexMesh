#pragma once

#include <objectPool.h>

namespace DFHM {

class Cell : public DataPool {
public:
	enum VolumeType {
		VT_UNKNOWN,
		VT_VOID,
		VT_SOLID,
		VT_FLUID,
	};
	VolumeType volType = VT_UNKNOWN;
	std::vector<size_t> _pPolygons; // indices of polygons in this cell
	std::vector<size_t> _pPolyhedra;// indices of polyedra in this cell
};

}
