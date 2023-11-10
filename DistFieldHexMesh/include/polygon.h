#pragma once

#include <dataPool.h>

namespace DFHM {

class Polygon : public DataPool {
public:
	bool unload(std::ostream& out, const ObjectPoolId& idSelf);
	bool load(std::istream& out, const ObjectPoolId& idSelf);

	std::vector<ObjectPoolId> vertexIds;
};

}
