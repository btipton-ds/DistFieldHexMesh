#pragma once

#include <vector>
#include <set>
#include <dataPool.h>

namespace DFHM {

class Polygon : public DataPool {
public:
	bool unload(std::ostream& out, const ObjectPoolId& idSelf);
	bool load(std::istream& out, const ObjectPoolId& idSelf);

	void addVertex(const ObjectPoolId& vertId);
	void setOwnerBlockId(const ObjectPoolId& blockId);
	void setNeighborBlockId(const ObjectPoolId& blockId);

private:
	std::vector<ObjectPoolId> _vertexIds;
	ObjectPoolId _ownerBlockId, _neighborBlockId;
	Vector3i _generatorCellIdx;
};

}
