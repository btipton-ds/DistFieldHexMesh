#pragma once

#include <vector>
#include <set>
#include <vertex.h>
#include <Index3DFull.h>

namespace DFHM {

class Polygon {
public:
	bool unload(std::ostream& out, size_t idSelf);
	bool load(std::istream& out, size_t idSelf);

	void addVertex(const Index3DId& vertId);
	void setOwnerSubBlockId(const Index3DIdFull& subBlockId);
	const Index3DIdFull& getOwnerSubBlockId() const;
	void setNeighborSubBlockId(const Index3DIdFull& subBlockId);
	const Index3DIdFull& getNeighborSubBlockId() const;

	void doneCreating();
	void pack();

	bool isOuter() const;

	bool operator < (const Polygon& rhs) const;

	const std::vector<Index3DId>& getVertexIds() const;
private:
	std::vector<Index3DId> _vertexIds, _sortedIds;
	Index3DIdFull _ownerSubBlockId, _neighborSubBlockId;
};

inline void Polygon::setOwnerSubBlockId(const Index3DIdFull& subBlockId)
{
	_ownerSubBlockId = subBlockId;
}

inline void Polygon::setNeighborSubBlockId(const Index3DIdFull& subBlockId)
{
	assert(subBlockId != _ownerSubBlockId);
	_neighborSubBlockId = subBlockId;
}


inline const Index3DIdFull& Polygon::getOwnerSubBlockId() const
{
	return _ownerSubBlockId;
}

inline const Index3DIdFull& Polygon::getNeighborSubBlockId() const
{
	return _neighborSubBlockId;
}

inline bool Polygon::isOuter() const
{
	return !_neighborSubBlockId.isValid();
}

inline const std::vector<Index3DId>& Polygon::getVertexIds() const
{
	return _vertexIds;
}

}
