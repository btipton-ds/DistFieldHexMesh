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

void Polygon::finished(const ObjectPool<Vertex>& vertices)
{
	Vector3<int64_t> temp(0, 0, 0); // Use long int for overflow
	for (size_t vertId : _vertexIds) {
		const auto& vert = vertices[vertId];
		const auto& iPt = vert.getFixedPt();
		temp[0] += iPt[0];
		temp[1] += iPt[1];
		temp[2] += iPt[2];
	}

	const size_t len = vertices.size();
	_centroid[0] = (int)(temp[0] / len);
	_centroid[1] = (int)(temp[1] / len);
	_centroid[2] = (int)(temp[2] / len);
}

bool Polygon::operator < (const Polygon& rhs) const
{
	if (_vertexIds.size() < rhs._vertexIds.size())
		return true;
	else if (_vertexIds.size() > rhs._vertexIds.size())
		return false;

	for (int i = 0; i < 3; i++) {
		if (_centroid[i] < rhs._centroid[i])
			return true;
		else if (rhs._centroid[i] < _centroid[i])
			return false;
	}
	return false;
}
