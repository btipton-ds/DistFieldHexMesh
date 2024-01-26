#include <vertex.h>
#include <edge.h>
#include <polygon.h>
#include <block.h>

using namespace std;
using namespace DFHM;

void Vertex::setId(ObjectPoolOwner* pBlock, size_t id)
{
	_pBlock = dynamic_cast<Block*> (pBlock);
	_thisId = Index3DId(pBlock->getBlockIdx(), id);
}


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

set<Index3DId> Vertex::getFaceIds(const set<Index3DId> availFaces) const
{
	set<Index3DId> result;

	for (const auto& faceId : _faceIds) {
		if (availFaces.count(faceId) != 0)
			result.insert(faceId);
	}
	return result;
}

bool Vertex::connectedToFace(const Index3DId& faceId) const
{
	return _faceIds.count(faceId) != 0;
}

bool Vertex::verifyTopology() const
{
	bool valid = true;
	for (const auto& faceId : _faceIds) {
		if (_pBlock->polygonExists(faceId)) {
			_pBlock->faceFunc(faceId, [this, &faceId, &valid](const Block* pBlock, const Polygon& face) {
				bool pass = face.containsVert(_thisId);
				if (!pass)
					valid = false;
				});
		} else
			valid = false;
	}

	return valid;
}
