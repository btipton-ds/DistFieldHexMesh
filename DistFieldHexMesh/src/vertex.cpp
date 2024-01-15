#include <vertex.h>
#include <algorithm>

using namespace DFHM;

void Vertex::addFaceId(const Index3DFull& faceId)
{
	_faceIds.insert(faceId);
}

void Vertex::removeFaceId(const Index3DFull& faceId)
{
	_faceIds.erase(faceId);
}

void Vertex::addEdgeId(const Index3DFull& edgeId)
{
	_edgeIds.insert(edgeId);
}

void Vertex::removeEdgeId(const Index3DFull& edgeId)
{
	_edgeIds.erase(edgeId);
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
