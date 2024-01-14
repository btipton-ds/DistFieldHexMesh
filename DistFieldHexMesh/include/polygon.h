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
	void setOwnerSubBlockId(const UniversalIndex3D& subBlockId);
	const UniversalIndex3D& getOwnerSubBlockId() const;
	void setNeighborSubBlockId(const UniversalIndex3D& subBlockId);
	const UniversalIndex3D& getNeighborSubBlockId() const;

	void doneCreating();
	void pack();

	bool isOuter() const;

	bool operator < (const Polygon& rhs) const;

	const std::vector<UniversalIndex3D>& getVertexIds() const;
private:
	std::vector<UniversalIndex3D> _vertexIds, _sortedIds;
	UniversalIndex3D _ownerSubBlockId, _neighborSubBlockId;
};

inline void Polygon::addVertex(const UniversalIndex3D& vertId)
{
	_vertexIds.push_back(vertId);
}

inline void Polygon::setOwnerSubBlockId(const UniversalIndex3D& subBlockId)
{
	_ownerSubBlockId = subBlockId;
}

inline void Polygon::setNeighborSubBlockId(const UniversalIndex3D& subBlockId)
{
	assert(subBlockId != _ownerSubBlockId);
	_neighborSubBlockId = subBlockId;
}


inline const UniversalIndex3D& Polygon::getOwnerSubBlockId() const
{
	return _ownerSubBlockId;
}

inline const UniversalIndex3D& Polygon::getNeighborSubBlockId() const
{
	return _neighborSubBlockId;
}

inline bool Polygon::isOuter() const
{
	return !_neighborSubBlockId.isValid();
}

inline const std::vector<UniversalIndex3D>& Polygon::getVertexIds() const
{
	return _vertexIds;
}

}
