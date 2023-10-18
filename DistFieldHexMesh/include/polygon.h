#pragma once

#include <objectPool.h>

namespace DFHM {

class Polygon : public DataPool {
public:
	bool unload(std::ostream& out, size_t idSelf);
	bool load(std::istream& out, size_t idSelf);

	std::vector<size_t> vertexIds;
};

}
