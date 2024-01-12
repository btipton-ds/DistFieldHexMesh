#pragma once

#include <vector>
#include <set>
#include <vertex.h>
#include <UniversalIndex3D.h>

namespace DFHM {

class Polygon {
public:
	bool unload(std::ostream& out, size_t idSelf);
	bool load(std::istream& out, size_t idSelf);

	void addVertex(const UniversalIndex3D& vertId);
	void setOwnerCellId(size_t cellId);
	size_t getOwnerCellId() const;
	void setNeighborCellId(size_t cellId);
	size_t getNeighborCellId() const;

	void doneCreating();
	void pack();

	bool isOuter() const;

	bool operator < (const Polygon& rhs) const;

	const std::vector<UniversalIndex3D>& getVertexIds() const;
private:
	std::vector<UniversalIndex3D> _vertexIds, _sortedIds;
	size_t _ownerCellId = -1, _neighborCellId = -1;
};

inline void Polygon::addVertex(const UniversalIndex3D& vertId)
{
	_vertexIds.push_back(vertId);
}

inline void Polygon::setOwnerCellId(size_t cellId)
{
	_ownerCellId = cellId;
}

inline void Polygon::setNeighborCellId(size_t cellId)
{
	assert(cellId != _ownerCellId);
	_neighborCellId = cellId;
}


inline size_t Polygon::getOwnerCellId() const
{
	return _ownerCellId;
}

inline size_t Polygon::getNeighborCellId() const
{
	return _neighborCellId;
}

inline bool Polygon::isOuter() const
{
	return _neighborCellId == -1;
}

inline const std::vector<UniversalIndex3D>& Polygon::getVertexIds() const
{
	return _vertexIds;
}

}
