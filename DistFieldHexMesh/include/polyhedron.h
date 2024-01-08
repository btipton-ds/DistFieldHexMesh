#pragma once

#include <dataPool.h>

namespace DFHM {

class Polyhedron {
public:
	bool unload(std::ostream& out);
	bool load(std::istream& out);

	size_t getHash() const;
	bool operator < (const Polyhedron& rhs) const;

private:
	std::vector<ObjectPoolId> _polygonIds;
};

}

