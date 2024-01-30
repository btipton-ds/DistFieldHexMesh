#include <tm_math.h>
#include <edge.h>
#include <block.h>
#include <volume.h>

using namespace std;
using namespace DFHM;

Edge::Edge(ObjectPoolOwner* pBlock, const Index3DId& vert0, const Index3DId& vert1)
{
	_pBlock = dynamic_cast<Block*>(pBlock);
	if (vert0 < vert1) {
		_vertexIds[0] = vert0;
		_vertexIds[1] = vert1;
	}
	else {
		_vertexIds[0] = vert1;
		_vertexIds[1] = vert0;
	}
}


bool Edge::operator == (const Edge& rhs) const
{
	return (_vertexIds[0] == rhs._vertexIds[0]) && (_vertexIds[1] == rhs._vertexIds[1]);

}

bool Edge::isValid() const
{
	return _pBlock != nullptr && _vertexIds[0].isValid() && _vertexIds[1].isValid();
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

double Edge::sameParamTol() const
{
	return Vertex::sameDistTol() / getLength();
}

double Edge::getLength() const
{
	LineSegment seg(_pBlock->getVertexPoint(_vertexIds[0]), _pBlock->getVertexPoint(_vertexIds[1]));
	return seg.calLength();
}

Vector3d Edge::calCenter() const
{
	return calPointAt(0.5);
}

Vector3d Edge::calPointAt(double t) const
{
	Vector3d pt0 = _pBlock->getVertexPoint(_vertexIds[0]);
	Vector3d pt1 = _pBlock->getVertexPoint(_vertexIds[1]);
	
	Vector3d result = pt0 + t * (pt1 - pt0);

	return result;
}

double Edge::paramOfPt(const Vector3d& pt, bool& inBounds) const
{
	Vector3d pt0 = _pBlock->getVertexPoint(_vertexIds[0]);
	Vector3d pt1 = _pBlock->getVertexPoint(_vertexIds[1]);
	Vector3d v = pt1 - pt0;
	v.normalize();

	Vector3d v1 = pt - pt0;
	double t = v.dot(v1);
	inBounds = (t >= 0) && (t <= 1);

	return t;
}

Vector3d Edge::projectPt(const Vector3d& pt) const
{
	Vector3d pt0 = _pBlock->getVertexPoint(_vertexIds[0]);
	Vector3d pt1 = _pBlock->getVertexPoint(_vertexIds[1]);
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

set<Index3DId> Edge::getFaceIds() const
{
	set<Index3DId> result;

	set<Index3DId> faceIds;
	_pBlock->vertexFunc(_vertexIds[0], [&faceIds](const Block* pBlock, const Vertex& vert) {
		faceIds = vert.getFaceIds();
	});

	_pBlock->vertexFunc(_vertexIds[1], [&faceIds](const Block* pBlock, const Vertex& vert) {
		auto temp = vert.getFaceIds();
		if (!temp.empty()) 
			faceIds.insert(temp.begin(), temp.end());
	});

	for (const auto& faceId : faceIds) {
		_pBlock->faceFunc(faceId, [this, &result, &faceId](const Block* pBlock, const Polygon& face) {
			if (face.containsEdge(*this))
				result.insert(faceId);
		});
	}
	return result;
}

Index3DId Edge::getOtherVert(const Index3DId& vert) const
{
	if (vert == _vertexIds[0])
		return _vertexIds[1];
	else if (vert == _vertexIds[1])
		return _vertexIds[0];
	return Index3DId();
}

Index3DId Edge::splitAtParam(double t) const
{
	const double tol = 0;
	if (t > tol && t < 1.0 - tol) {
		Vector3d pt = calPointAt(t);
		auto midVertId = _pBlock->addVertex(pt);

		set<Index3DId> faceIds = getFaceIds();
		for (const auto& faceId : faceIds) {
			_pBlock->faceFunc(faceId, [this, &midVertId](Block* pBlock, Polygon& face) {
				if (midVertId == Index3DId(Index3D(3, 0, 1), 3)) {
					int dbgBreak = 1;
				}
				if (face.containsVert(midVertId)) // This face was already split with this vertex
					return;
				if (!face.insertVertexInEdgeNTS(*this, midVertId)) {
					assert(!"Should never reach this code block.");
					throw std::runtime_error("Failed insert a vertex in a polygon which must accept the vertex. Not supposed to be possible.");
				}
				});
		}

		return midVertId;
	}

	return Index3DId();
}

double Edge::intesectPlaneParam(const Plane& splittingPlane) const
{
	Vector3d pt0 = _pBlock->getVertexPoint(_vertexIds[0]);
	Vector3d pt1 = _pBlock->getVertexPoint(_vertexIds[1]);
	LineSegment seg(pt0, pt1);
	RayHit hit;
	if (intersectLineSegPlane(seg, splittingPlane, hit)) {
		double t = hit.dist / seg.calLength();
		return t;
	}
	return -1;
}

Index3DId Edge::splitWithPlane(const Plane& splittingPlane) const
{
	const double tol = 0;
	double t = intesectPlaneParam(splittingPlane);
	if (t > tol && t < 1.0 - tol) {
		return splitAtParam(t);
	}
	return Index3DId();
}
