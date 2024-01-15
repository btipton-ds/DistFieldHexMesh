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

	void addVertex(const Index3DFull& vertId);
	void setOwnerSubBlockId(const Index3DFull& subBlockId);
	const Index3DFull& getOwnerSubBlockId() const;
	void setNeighborSubBlockId(const Index3DFull& subBlockId);
	const Index3DFull& getNeighborSubBlockId() const;

	void doneCreating();
	void pack();

	bool isOuter() const;

	bool operator < (const Polygon& rhs) const;

	const std::vector<Index3DFull>& getVertexIds() const;
private:
	std::vector<Index3DFull> _vertexIds, _sortedIds;
	Index3DFull _ownerSubBlockId, _neighborSubBlockId;
};

inline void Polygon::addVertex(const Index3DFull& vertId)
{
	_vertexIds.push_back(vertId);
}

inline void Polygon::setOwnerSubBlockId(const Index3DFull& subBlockId)
{
	_ownerSubBlockId = subBlockId;
}

inline void Polygon::setNeighborSubBlockId(const Index3DFull& subBlockId)
{
	assert(subBlockId != _ownerSubBlockId);
	_neighborSubBlockId = subBlockId;
}


inline const Index3DFull& Polygon::getOwnerSubBlockId() const
{
	return _ownerSubBlockId;
}

inline const Index3DFull& Polygon::getNeighborSubBlockId() const
{
	return _neighborSubBlockId;
}

inline bool Polygon::isOuter() const
{
	return !_neighborSubBlockId.isValid();
}

inline const std::vector<Index3DFull>& Polygon::getVertexIds() const
{
	return _vertexIds;
}

}
