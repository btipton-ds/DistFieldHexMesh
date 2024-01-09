#pragma once

#include <set>
#include <exception>

namespace DFHM {

class Cell {
public:
	enum VolumeType {
		VT_UNKNOWN,
		VT_VOID,
		VT_SOLID,
		VT_FLUID,
	};

	void addPolyhdra(size_t id);
	void removePolyhdra(size_t id);

	bool unload(std::ostream& out);
	bool load(std::istream& in);

	bool operator < (const Cell& rhs) const;
private:
	VolumeType _volType = VT_UNKNOWN;
	std::set<size_t> _polyhedra;// indices of polyedra in this cell
};

inline void Cell::addPolyhdra(size_t id)
{
	_polyhedra.insert(id);
}

inline void Cell::removePolyhdra(size_t id)
{
	_polyhedra.erase(id);
}
inline bool Cell::operator < (const Cell& rhs) const
{
	throw std::runtime_error("Cell::operator < is not implmented");
	return false;
}

}
