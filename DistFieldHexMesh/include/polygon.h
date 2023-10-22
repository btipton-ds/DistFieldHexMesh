#pragma once

#include <objectPool.h>

namespace DFHM {

class Polygon : public DataPool {
public:
	bool unload(std::ostream& out, size_t idSelf);
	bool load(std::istream& out, size_t idSelf);

	size_t _cellIdx[2] = {(size_t) -1, (size_t) -1};
	std::vector<size_t> vertexIds;
};

}
