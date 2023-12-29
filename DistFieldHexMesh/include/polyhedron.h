#pragma once

#include <dataPool.h>

namespace DFHM {

class Polyhedron : public DataPool {
public:
	bool unload(std::ostream& out);
	bool load(std::istream& out);

	std::vector<size_t> polygonId;
};

}

