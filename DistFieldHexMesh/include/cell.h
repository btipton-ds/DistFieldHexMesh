#pragma once

#include <dataPool.h>

namespace DFHM {

class Cell {
public:
	enum VolumeType {
		VT_UNKNOWN,
		VT_VOID,
		VT_SOLID,
		VT_FLUID,
	};

	bool unload(std::ostream& out);
	bool load(std::istream& in);

	size_t getHash() const;
	bool operator < (const Cell& rhs) const;

private:
	VolumeType volType = VT_UNKNOWN;
	std::vector<ObjectPoolId> _polygons; // indices of polygons in this cell
	std::vector<ObjectPoolId> _polyhedra;// indices of polyedra in this cell
};

inline size_t Cell::getHash() const
{
	return -1;
}

inline bool Cell::operator < (const Cell& rhs) const
{
	assert(!"Cannot do reverse search on cells.");
	return false;
}

}
