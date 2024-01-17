#include <edge.h>
#include <block.h>
#include <volume.h>

using namespace std;
using namespace DFHM;

bool Edge::operator == (const Edge& rhs) const
{
	return (_vertexIds[0] == rhs._vertexIds[0]) && (_vertexIds[1] == rhs._vertexIds[1]);

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

Vector3d Edge::getCenter(const Block* pBlock) const
{
	return getPointAt(pBlock, 0.5);
}

Vector3d Edge::getPointAt(const Block* pBlock, double t) const
{
	Vector3d pt0 = pBlock->getVertexPoint(_vertexIds[0]);
	Vector3d pt1 = pBlock->getVertexPoint(_vertexIds[1]);
	Vector3d v = pt1 - pt0;
	v.normalize();
	v = t * v;
	
	Vector3d result = pt0 + v;

	return result;
}

double Edge::paramOfPt(const Block* pBlock, const Vector3d& pt) const
{
	Vector3d pt0 = pBlock->getVertexPoint(_vertexIds[0]);
	Vector3d pt1 = pBlock->getVertexPoint(_vertexIds[1]);
	Vector3d v = pt1 - pt0;
	v.normalize();

	Vector3d v1 = pt - pt0;
	double t = v.dot(v1);

	return t;
}

Vector3d Edge::projectPt(const Block* pBlock, const Vector3d& pt) const
{
	Vector3d pt0 = pBlock->getVertexPoint(_vertexIds[0]);
	Vector3d pt1 = pBlock->getVertexPoint(_vertexIds[1]);
	Vector3d v = pt1 - pt0;
	v.normalize();

	Vector3d v1 = pt - pt0;
	double t = v.dot(v1);

	Vector3d result = pt0 + t * v;
	return result;
}

bool Edge::containsVertex(const Index3DId& vertexId) const
{
	return _vertexIds[0] == vertexId || _vertexIds[1] == vertexId;
}

set<Index3DId> Edge::getFaceIds(const Block* pBlock) const
{
	set<Index3DId> result;

	set<Index3DId> faceIds;
	pBlock->vertexFunc(_vertexIds[0], [&faceIds](const Block* pBlock, const Vertex& vert) {
		faceIds = vert.getFaceIds();
	});
	for (const auto& faceId : faceIds) {
		pBlock->faceFunc(faceId, [this, &result, &faceId](const Block* pBlock, const Polygon& face) {
			if (face.containsEdge(*this))
				result.insert(faceId);
		});
	}
	return result;
}

set<Index3DId> Edge::getFaceIds(const Block* pBlock, set<Index3DId>& availFaces) const
{
	set<Index3DId> result;
	set<Index3DId> edgeFaceIds = getFaceIds(pBlock);

	for (const auto& faceId : edgeFaceIds) {
		if (availFaces.count(faceId) != 0) {
			result.insert(faceId);
		}
	}

	return result;
}
