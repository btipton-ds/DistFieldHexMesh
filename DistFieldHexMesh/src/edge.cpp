/*
This file is part of the DistFieldHexMesh application/library.

	The DistFieldHexMesh application/library is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	The DistFieldHexMesh application/library is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	This link provides the exact terms of the GPL license <https://www.gnu.org/licenses/>.

	The author's interpretation of GPL 3 is that if you receive money for the use or distribution of the DistFieldHexMesh application/library or a derivative product, GPL 3 no longer applies.

	Under those circumstances, the author expects and may legally pursue a reasoble share of the income. To avoid the complexity of agreements and negotiation, the author makes
	no specific demands in this regard. Compensation of roughly 1% of net or $5 per user license seems appropriate, but is not legally binding.

	In lay terms, if you make a profit by using the DistFieldHexMesh application/library (violating the spirit of Open Source Software), I expect a reasonable share for my efforts.

	Robert R Tipton - Author

	Dark Sky Innovative Solutions http://darkskyinnovation.com/
*/

#include <tm_math.h>
#include <edge.h>
#include <block.h>
#include <volume.h>

using namespace std;
using namespace DFHM;

Edge::Edge(const Index3DId& vert0, const Index3DId& vert1, const std::set<Index3DId>& faceIds)
	: _faceIds(faceIds)
{
	if (vert0 < vert1) {
		_vertexIds[0] = vert0;
		_vertexIds[1] = vert1;
	} else {
		_vertexIds[0] = vert1;
		_vertexIds[1] = vert0;
	}
}

Edge::Edge(const Edge& src, const std::set<Index3DId>& faceIds)
	: _faceIds(faceIds)
{
	_vertexIds[0] = src._vertexIds[0];
	_vertexIds[1] = src._vertexIds[1];
}

bool Edge::operator == (const Edge& rhs) const
{
	return (_vertexIds[0] == rhs._vertexIds[0]) && (_vertexIds[1] == rhs._vertexIds[1]);

}

bool Edge::isValid() const
{
	return _vertexIds[0].isValid() && _vertexIds[1].isValid();
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
	return Tolerance::sameDistTol() / getLength(pBlock);
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

Vector3d Edge::calUnitDir(const Block* pBlock) const
{
	Vector3d pt0 = pBlock->getVertexPoint(_vertexIds[0]);
	Vector3d pt1 = pBlock->getVertexPoint(_vertexIds[1]);
	Vector3d v = pt1 - pt0;
	v.normalize();
	return v;
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

bool Edge::onPrincipalAxis(const Block* pBlock) const
{
	const double tol = Tolerance::paramTol();
	Vector3d dir = calUnitDir(pBlock);
	bool result = false;
	for (int i = 0; i < 3; i++) {
		Vector3d axis(0, 0, 0);
		axis[i] = 1;
		if (axis.cross(dir).norm() < tol)
			return true;
	}
	return false;
}

bool Edge::isColinearWith(const Block* pBlock, const Edge& other) const
{
	LineSegment seg(getSegment(pBlock));
	Vector3d pt0 = pBlock->getVertexPoint(other._vertexIds[0]);

	if (seg.distanceToPoint(pt0) > Tolerance::sameDistTol())
		return false;

	Vector3d pt1 = pBlock->getVertexPoint(other._vertexIds[1]);
	if (seg.distanceToPoint(pt1) > Tolerance::sameDistTol())
		return false;

	return true;
}

bool Edge::isColinearWith(const Block* pBlock, const Index3DId& vert, bool& inBounds) const
{
	Vector3d pt = pBlock->getVertexPoint(vert);
	Vector3d pt0 = pBlock->getVertexPoint(_vertexIds[0]);
	Vector3d pt1 = pBlock->getVertexPoint(_vertexIds[1]);

	Vector3d v = pt1 - pt0;
	v.normalize();

	Vector3d v1 = pt - pt0;
	double t = v.dot(v1);
	inBounds = -Tolerance::paramTol() < t && t < (1 + Tolerance::paramTol());
	v1 = v1 - t * v;
	double dist = v1.norm();

	return fabs(dist) < Tolerance::sameDistTol();
}

bool Edge::isConnectedTo(const Edge& other) const
{
	for (int i = 0; i < 2; i++) {
		if (other.containsVertex(_vertexIds[i]))
			return true;
	}
	return false;
}

LineSegment Edge::getSegment(const Block* pBlock) const
{
	Vector3d pt0 = pBlock->getVertexPoint(_vertexIds[0]);
	Vector3d pt1 = pBlock->getVertexPoint(_vertexIds[1]);
	LineSegment result(pt0, pt1);
	return result;
}

bool Edge::containsVertex(const Index3DId& vertexId) const
{
	return _vertexIds[0] == vertexId || _vertexIds[1] == vertexId;
}

Index3DId Edge::getOtherVert(const Index3DId& vert) const
{
	if (vert == _vertexIds[0])
		return _vertexIds[1];
	else if (vert == _vertexIds[1])
		return _vertexIds[0];
	return Index3DId();
}

void Edge::getFaceIds(std::set<Index3DId>& faceIds) const
{
	faceIds.insert(_faceIds.begin(), _faceIds.end());
}

Index3DId Edge::splitAtParam(Block* pBlock, double t, set<Index3DId>& faceIds) const
{
	const double tol = Tolerance::paramTol();
	if (t > tol && t < 1.0 - tol) {
		Vector3d pt = calPointAt(pBlock, t);
		auto midVertId = pBlock->addVertex(pt);

		getFaceIds(faceIds);
		return midVertId;
	}

	return Index3DId();
}

double Edge::intesectPlaneParam(const Block* pBlock, const Plane& splittingPlane) const
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

Index3DId Edge::splitWithPlane(Block* pBlock, const Plane& splittingPlane, std::set<Index3DId>& faceIds) const
{
	const double tol = Tolerance::paramTol();
	double t = intesectPlaneParam(pBlock, splittingPlane);
	if (t > tol && t < 1.0 - tol) {
		return splitAtParam(pBlock, t, faceIds);
	}
	return Index3DId();
}
