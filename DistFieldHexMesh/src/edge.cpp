#include <edge.h>
#include <block.h>

using namespace std;
using namespace DFHM;

void Edge::addFaceId(const UniversalIndex3D& faceId)
{
	auto pos = std::find(_faceIds.begin(), _faceIds.end(), faceId);
	if (pos == _faceIds.end())
		_faceIds.push_back(faceId);
}

void Edge::removeFaceId(const UniversalIndex3D& faceId)
{
	auto pos = std::find(_faceIds.begin(), _faceIds.end(), faceId);
	if (pos != _faceIds.end())
		_faceIds.erase(pos);
}

bool Edge::unload(std::ostream& out, size_t idSelf)
{
	return false;
}

bool Edge::load(std::istream& out, size_t idSelf)
{
	return false;
}

bool Edge::operator < (const Edge& rhs) const
{
	for (int i = 0; i < 2; i++) {
		if (_vertexIds[i] < rhs._vertexIds[i])
			return true;
		else if (rhs._vertexIds[i] < _vertexIds[i])
			return false;
	}
	return false;
}

// Should only be called from Polygon::split which will handle topology
UniversalIndex3D Edge::split(Block* pOwnerBlock, double t)
{
	Volume* pVol = pOwnerBlock->getVolume();

	return UniversalIndex3D();
}
