#include <vertex.h>
#include <edge.h>
#include <polygon.h>
#include <block.h>

using namespace std;
using namespace DFHM;

void Vertex::addFaceId(const Index3DId& faceId)
{
	_faceIds.insert(faceId);
}

void Vertex::removeFaceId(const Index3DId& faceId)
{
	_faceIds.erase(faceId);
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

set<Index3DId> Vertex::getFaceIds(const vector<Index3DId> availFaces) const
{
	set<Index3DId> result, availSet;
	availSet.insert(availFaces.begin(), availFaces.end());

	for (const auto& faceId : _faceIds) {
		if (availSet.count(faceId) != 0)
			result.insert(faceId);
	}
	return result;
}
