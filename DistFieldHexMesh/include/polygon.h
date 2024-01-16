#pragma once

#include <vector>
#include <set>
#include <vertex.h>
#include <index3D.h>

namespace DFHM {

class Polygon {
public:
	bool unload(std::ostream& out, size_t idSelf);
	bool load(std::istream& out, size_t idSelf);

	void addVertex(const Index3DId& vertId);
	void setCreatorCellId(const Index3DId& cellId);
	const Index3DId& getCreatorCellId() const;
	void setNeighborCellId(const Index3DId& subBlockId);
	const Index3DId& getNeighborCellId() const;

	void doneCreating();
	void pack();

	bool isOuter() const;

	bool operator < (const Polygon& rhs) const;

	const std::vector<Index3DId>& getVertexIds() const;
private:
	std::vector<Index3DId> _vertexIds, _sortedIds;
	Index3DId _creatorCellId, _neighborCellId;
};

inline void Polygon::setCreatorCellId(const Index3DId& subBlockId)
{
	_creatorCellId = subBlockId;
}

inline void Polygon::setNeighborCellId(const Index3DId& subBlockId)
{
	assert(subBlockId != _creatorCellId);
	_neighborCellId = subBlockId;
}


inline const Index3DId& Polygon::getCreatorCellId() const
{
	return _creatorCellId;
}

inline const Index3DId& Polygon::getNeighborCellId() const
{
	return _neighborCellId;
}

inline bool Polygon::isOuter() const
{
	return !_neighborCellId.isValid();
}

inline const std::vector<Index3DId>& Polygon::getVertexIds() const
{
	return _vertexIds;
}

}
