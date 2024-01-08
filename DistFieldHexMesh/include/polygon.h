#pragma once

#include <vector>
#include <set>
#include <dataPool.h>

namespace DFHM {

class Polygon {
public:
	bool unload(std::ostream& out, size_t idSelf);
	bool load(std::istream& out, size_t idSelf);

	void addVertex(size_t vertId);
	void setOwnerBlockId(size_t blockId);
	size_t getOwnerBlockId() const;
	void setNeighborBlockId(size_t blockId);
	size_t getNeighborBlockId() const;

	bool isOuter() const;

	size_t getHash() const;
	bool operator < (const Polygon& rhs) const;

	const std::vector<size_t>& getVertexIds() const;
private:
	std::vector<size_t> _vertexIds;
	size_t _ownerBlockId, _neighborBlockId;
	Vector3i _generatorCellIdx;
};

inline size_t Polygon::getOwnerBlockId() const
{
	return _ownerBlockId;
}

inline size_t Polygon::getNeighborBlockId() const
{
	return _neighborBlockId;
}

inline bool Polygon::isOuter() const
{
	return _neighborBlockId == -1;
}

inline const std::vector<size_t>& Polygon::getVertexIds() const
{
	return _vertexIds;
}

}
