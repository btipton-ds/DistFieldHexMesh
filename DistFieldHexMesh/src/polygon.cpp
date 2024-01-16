#include <iostream>
#include <vertex.h>
#include <polygon.h>

using namespace std;
using namespace DFHM;

void Polygon::addVertex(const Index3DId& vertId)
{
	assert(find(_vertexIds.begin(), _vertexIds.end(), vertId) == _vertexIds.end());
	_vertexIds.push_back(vertId);
}


bool Polygon::unload(std::ostream& out, size_t idSelf)
{

	return true;
}

bool Polygon::load(std::istream& in, size_t idSelf)
{

	return true;
}

void Polygon::doneCreating()
{
	_sortedIds = _vertexIds;
	sort(_sortedIds.begin(), _sortedIds.end());
}

void Polygon::pack()
{
	_sortedIds.clear();
}

bool Polygon::operator < (const Polygon& rhs) const
{
	if (_sortedIds.size() < rhs._sortedIds.size())
		return true;
	else if (_sortedIds.size() > rhs._sortedIds.size())
		return false;

	for (size_t i = 0; i < _sortedIds.size(); i++) {
		if (_sortedIds[i] < rhs._sortedIds[i])
			return true;
		else if (rhs._sortedIds[i] < _sortedIds[i])
			return false;
	}
	return false;
}
