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

