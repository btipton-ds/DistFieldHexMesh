#include <tm_math.h>
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

double Edge::sameParamTol(const Block* pBlock) const
{
	return Vertex::sameDistTol() / getLength(pBlock);
}

double Edge::getLength(const Block* pBlock) const
{
	LineSegment seg(pBlock->getVertexPoint(_vertexIds[0]), pBlock->getVertexPoint(_vertexIds[1]));
	return seg.calLength();
}

Vector3d Edge::calCenter(const Block* pBlock) const
{
	return calPointAt(pBlock, 0.5);
}

Vector3d Edge::calPointAt(const Block* pBlock, double t) const
{
	Vector3d pt0 = pBlock->getVertexPoint(_vertexIds[0]);
	Vector3d pt1 = pBlock->getVertexPoint(_vertexIds[1]);
	
	Vector3d result = pt0 + t * (pt1 - pt0);

	return result;
}

double Edge::paramOfPt(const Block* pBlock, const Vector3d& pt, bool& inBounds) const
{
	Vector3d pt0 = pBlock->getVertexPoint(_vertexIds[0]);
	Vector3d pt1 = pBlock->getVertexPoint(_vertexIds[1]);
	Vector3d v = pt1 - pt0;
	v.normalize();

	Vector3d v1 = pt - pt0;
	double t = v.dot(v1);
	inBounds = (t >= 0) && (t <= 1);

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

Index3DId Edge::getCommonFace(const Block* pBlock) const
{
	set<Index3DId> faceSet0;
	pBlock->vertexFunc(_vertexIds[0], [&faceSet0](const Block* pBlock, const Vertex& vert) {
		faceSet0 = vert.getFaceIds();
	});

	for (auto iter = faceSet0.begin(); iter != faceSet0.end(); iter++) {
		const auto& faceId = *iter;
		size_t count = 0;
		pBlock->faceFunc(faceId, [this, &count](const Block* pBlock, const Polygon& face) {
			// Do not use containsEdge. It requires adjacency and this function doesn't
			const auto vertIds = face.getVertexIds();
			for (const auto& vertId : vertIds) {
				if ((vertId == _vertexIds[0]) || (vertId == _vertexIds[1]))
					count++;
			}
		});

		if (count == 2)
			return faceId;
	}

	return Index3DId();
}

Index3DId Edge::splitAtParam(Block* pBlock, double t) const
{
	const double tol = 0;
	if (t > tol && t < 1.0 - tol) {
		Vector3d pt = calPointAt(pBlock, t);
		auto midVertId = pBlock->addVertex(pt);

		set<Index3DId> faceIds = getFaceIds(pBlock);
		for (const auto& faceId : faceIds) {
			pBlock->faceFunc(faceId, [this, &midVertId](Block* pBlock, Polygon& face) {
				if (face.containsVert(midVertId)) // This face was already split with this vertex
					return;
				if (!face.insertVertexNTS(*this, midVertId)) {
					assert(!"Should never reach this code block.");
					throw std::runtime_error("Failed insert a vertex in a polygon which must accept the vertex. Not supposed to be possible.");
				}
				});
		}

		return midVertId;
	}

	return Index3DId();
}

double Edge::intesectPlaneParam(Block* pBlock, const Plane& splittingPlane) const
{
	Vector3d pt0 = pBlock->getVertexPoint(_vertexIds[0]);
	Vector3d pt1 = pBlock->getVertexPoint(_vertexIds[1]);
	LineSegment seg(pt0, pt1);
	RayHit hit;
	if (intersectLineSegPlane(seg, splittingPlane, hit)) {
		double t = hit.dist / seg.calLength();
		return t;
	}
	return -1;
}

Index3DId Edge::splitWithPlane(Block* pBlock, const Plane& splittingPlane) const
{
	const double tol = 0;
	double t = intesectPlaneParam(pBlock, splittingPlane);
	if (t > tol && t < 1.0 - tol) {
		return splitAtParam(pBlock, t);
	}
	return Index3DId();
}
