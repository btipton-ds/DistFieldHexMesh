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
	void setOwnerCellId(const UniversalIndex3D& cellId);
	const UniversalIndex3D& getOwnerCellId() const;
	void setNeighborCellId(const UniversalIndex3D& cellId);
	const UniversalIndex3D& getNeighborCellId() const;

	void doneCreating();
	void pack();

	bool isOuter() const;

	bool operator < (const Polygon& rhs) const;

	const std::vector<UniversalIndex3D>& getVertexIds() const;
private:
	std::vector<UniversalIndex3D> _vertexIds, _sortedIds;
	UniversalIndex3D _ownerCellId, _neighborCellId;
};

inline void Polygon::addVertex(const UniversalIndex3D& vertId)
{
	_vertexIds.push_back(vertId);
}

inline void Polygon::setOwnerCellId(const UniversalIndex3D& cellId)
{
	_ownerCellId = cellId;
}

inline void Polygon::setNeighborCellId(const UniversalIndex3D& cellId)
{
	assert(cellId != _ownerCellId);
	_neighborCellId = cellId;
}


inline const UniversalIndex3D& Polygon::getOwnerCellId() const
{
	return _ownerCellId;
}

inline const UniversalIndex3D& Polygon::getNeighborCellId() const
{
	return _neighborCellId;
}

inline bool Polygon::isOuter() const
{
	return !_neighborCellId.isValid();
}

inline const std::vector<UniversalIndex3D>& Polygon::getVertexIds() const
{
	return _vertexIds;
}

}
