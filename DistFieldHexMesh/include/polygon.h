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
	const ObjectPoolId& getOwnerBlockId() const;
	void setNeighborBlockId(const ObjectPoolId& blockId);
	const ObjectPoolId& getNeighborBlockId() const;

	bool isOuter() const;

	size_t getHash() const;
	bool operator < (const Polygon& rhs) const;

	const std::vector<ObjectPoolId>& getVertexIds() const;
private:
	std::vector<ObjectPoolId> _vertexIds;
	ObjectPoolId _ownerBlockId, _neighborBlockId;
	Vector3i _generatorCellIdx;
};

inline const ObjectPoolId& Polygon::getOwnerBlockId() const
{
	return _ownerBlockId;
}

inline const ObjectPoolId& Polygon::getNeighborBlockId() const
{
	return _neighborBlockId;
}

inline bool Polygon::isOuter() const
{
	return _neighborBlockId == -1;
}

inline const std::vector<ObjectPoolId>& Polygon::getVertexIds() const
{
	return _vertexIds;
}

}
