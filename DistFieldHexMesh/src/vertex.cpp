#include <vertex.h>
#include <algorithm>

using namespace DFHM;

void Vertex::addPolygonReference(size_t polygonId)
{
	auto pos = std::find(_polygonIds.begin(), _polygonIds.end(), polygonId);
	if (pos == _polygonIds.end())
		_polygonIds.push_back(polygonId);
}

void Vertex::removePolygonReference(size_t polygonId)
{
	auto pos = std::find(_polygonIds.begin(), _polygonIds.end(), polygonId);
	if (pos != _polygonIds.end())
		_polygonIds.erase(pos);
}

const bool Vertex::operator < (const Vertex& rhs) const
{
	for (size_t idx = 0; idx < 3; idx++) {
		if (_pt[idx] < rhs._pt[idx])
			return true;
		else if (rhs._pt[idx] < _pt[idx])
			return false;
	}

	return false;
}
