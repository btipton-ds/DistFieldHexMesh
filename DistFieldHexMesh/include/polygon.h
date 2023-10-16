#pragma once

#include <objectPool.h>

namespace DFHM {

class Polygon : public DataPool {
public:
	std::vector<size_t> vertexIndices;
};

}
