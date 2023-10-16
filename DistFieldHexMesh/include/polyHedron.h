#pragma once

#include <objectPool.h>

namespace DFHM {

class Polyhedron : public DataPool {
public:
	std::vector<size_t> faceIndices;
};

}

