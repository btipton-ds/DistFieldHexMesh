#include <iostream>
#include <vertex.h>
#include <polygon.h>

using namespace std;
using namespace DFHM;

bool Polygon::unload(std::ostream& out, size_t idSelf)
{

	return true;
}

bool Polygon::load(std::istream& in, size_t idSelf)
{

	return true;
}

void Polygon::addVertex(size_t vertId)
{
	_vertexIds.push_back(vertId);
}

void Polygon::setOwnerBlockId(size_t blockId)
{
	_ownerBlockId = blockId;
}

void Polygon::setNeighborBlockId(size_t blockId)
{
	_neighborBlockId = blockId;
}

size_t Polygon::getHash() const
{
	size_t hash = 0;
	for (auto vertId : _vertexIds) {
//		const auto& vert = _vertexPool.get(vertId);
//		hash += vert->getHash();
	}
	return hash;
}

bool Polygon::operator < (const Polygon& rhs) const
{
	if (_vertexIds.size() < rhs._vertexIds.size())
		return true;
	else if (_vertexIds.size() > rhs._vertexIds.size())
		return false;

	vector<size_t> lhsIndices(_vertexIds), rhsIndices(rhs._vertexIds);
	sort(lhsIndices.begin(), lhsIndices.end());
	sort(rhsIndices.begin(), rhsIndices.end());
	for (size_t i = 0; i < lhsIndices.size(); i++) {
		if (lhsIndices[i] < rhsIndices[i])
			return true;
		else if (rhsIndices[i] < lhsIndices[i])
			return false;
	}

	return false;
}
