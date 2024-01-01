#include <iostream>
#include <vertex.h>
#include <polygon.h>

using namespace std;
using namespace DFHM;

bool Polygon::unload(std::ostream& out, const ObjectPoolId& idSelf)
{
	size_t numVertices = _vertexIds.size();
	for (const auto& vertId : _vertexIds) {
		auto& vert = _vertexPool[vertId];
		vert.removePolygonReference(idSelf);
	}
	out.write((char*)&numVertices, sizeof(numVertices));
	// TODO collect all vertices which will be orphaned and write them out and purge them from storage
	out.write((char*)_vertexIds.data(), _vertexIds.size());

	return true;
}

bool Polygon::load(std::istream& in, const ObjectPoolId& idSelf)
{
	size_t numVertices;

	in.read((char*)&numVertices, sizeof(numVertices));
	_vertexIds.resize(numVertices);

	// TODO collect all cached orphan vertices and read them in

	in.read((char*)_vertexIds.data(), _vertexIds.size());
	for (size_t i = 0; i < _vertexIds.size(); i++) {
		Vertex& vert = _vertexPool[_vertexIds[i]];
	// TODO collect all vertices which will be orphaned and write them out
		vert.addPolygonReference(idSelf);
	}

	return true;
}

void Polygon::addVertex(const ObjectPoolId& vertId)
{
	_vertexIds.push_back(vertId);
}

void Polygon::setOwnerBlockId(const ObjectPoolId& blockId)
{
	_ownerBlockId = blockId;
}

void Polygon::setNeighborBlockId(const ObjectPoolId& blockId)
{
	_neighborBlockId = blockId;
}

size_t Polygon::getHash() const
{
	size_t hash = 0;
	for (auto vertId : _vertexIds)
		hash += vertId.getThreadIndex() + vertId.getIndex();
	return hash;
}

bool Polygon::operator < (const Polygon& rhs) const
{
	if (_vertexIds.size() < rhs._vertexIds.size())
		return true;
	else if (_vertexIds.size() > rhs._vertexIds.size())
		return false;

	vector<ObjectPoolId> lhsIndices(_vertexIds), rhsIndices(rhs._vertexIds);
	sort(lhsIndices.begin(), lhsIndices.begin());
	sort(rhsIndices.begin(), rhsIndices.begin());
	for (size_t i = 0; i < lhsIndices.size(); i++) {
		if (lhsIndices[i] < rhsIndices[i])
			return true;
		else if (rhsIndices[i] < lhsIndices[i])
			return false;
	}

	return false;
}
