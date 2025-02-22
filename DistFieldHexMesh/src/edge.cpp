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
#include <tm_lineSegment.h>
#include <edge.h>
#include <block.h>
#include <volume.h>
#include <tolerances.h>
#include <logger.h>

using namespace std;
using namespace DFHM;

Edge::Edge(const Index3DId& vert0, const Index3DId& vert1, const MTC::set<Index3DId>& faceIds)
	: _reversed(vert1 < vert0)
{
	_faceIds.insert(faceIds.begin(), faceIds.end());
		
	_vertexIds[0] = vert0;
	_vertexIds[1] = vert1;
}

Edge::Edge(const Edge& src, const MTC::set<Index3DId>& faceIds)
	: _reversed(src._reversed)
{
	_faceIds.insert(faceIds.begin(), faceIds.end());
	_vertexIds[0] = src._vertexIds[0];
	_vertexIds[1] = src._vertexIds[1];
}

bool Edge::isValid() const
{
	return _vertexIds[0].isValid() && _vertexIds[1].isValid();
}

bool Edge::operator == (const Edge& rhs) const
{
	return !operator < (rhs) && !rhs.operator<(*this);
}

bool Edge::operator != (const Edge& rhs) const
{
	return !operator == (rhs);
}

bool Edge::operator < (const Edge& rhs) const
{
	for (int i = 0; i < 2; i++) {
		const auto& v = _reversed ? _vertexIds[1 - i] : _vertexIds[i];
		const auto& rhsV = rhs._reversed ? rhs._vertexIds[1 - i] : rhs._vertexIds[i];

		if (v < rhsV)
			return true;
		else if (rhsV < v)
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
	LineSegmentd seg(pBlock->getVertexPoint(_vertexIds[0]), pBlock->getVertexPoint(_vertexIds[1]));
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

Vector3d Edge::calCoedgeUnitDir(const Block* pBlock, const Index3DId& faceId, const Index3DId& cellId) const
{
	Vector3d result;
	bool found = false;
	pBlock->faceFunc(faceId, [this, pBlock, &cellId, &result, &found](const Polygon& face) {
		face.iterateOrientedEdges([this, pBlock, &cellId, &result, &found](const Edge& edge)->bool {
			if (edge == *this) {
				result = edge.calUnitDir(pBlock);
				found = true;
			}
			return !found; // continue the loop
		}, cellId);
	});

	assert(found);
	return result;
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
	double len = v.norm();
	v.normalize();

	Vector3d v1 = pt - pt0;
	double t = v.dot(v1) / len;
	inBounds = (t > -Tolerance::paramTol()) && (t < 1 + Tolerance::paramTol());

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

bool Edge::isColinearWith(const Block* pBlock, const Index3DId& vert, double& param) const
{
	Vector3d pt = pBlock->getVertexPoint(vert);
	Vector3d pt0 = pBlock->getVertexPoint(_vertexIds[0]);
	Vector3d pt1 = pBlock->getVertexPoint(_vertexIds[1]);

	Vector3d v = pt1 - pt0;
	double len = v.norm();
	v.normalize();

	Vector3d v1 = pt - pt0;
	param = v.dot(v1);
	v1 = v1 - param * v;
	double dist = v1.norm();
	param /= len;

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

double Edge::calDihedralAngleRadians(const Block* pBlock, const Index3DId& refCellId) const
{
	if (_faceIds.size() != 2)
		return 0;
	const auto& faceId0 = _faceIds[0];
	const auto& faceId1 = _faceIds[1];
	Vector3d normal0, normal1;
	pBlock->faceFunc(faceId0, [&normal0, &refCellId](const Polygon& face) {
		normal0 = face.calOrientedUnitNormal(refCellId);
	});
	pBlock->faceFunc(faceId1, [&normal1, &refCellId](const Polygon& face) {
		normal1 = face.calOrientedUnitNormal(refCellId);
	});

	const auto& zAxis = normal0;
	Vector3d yAxis = calCoedgeUnitDir(pBlock, faceId0, refCellId);
	Vector3d xAxis = yAxis.cross(zAxis); // xAxis points from plane0 center to the edge

	double cosTheta = -normal1.dot(zAxis);
	double sinTheta = normal1.dot(xAxis);
	double angle = atan2(sinTheta, cosTheta);

	return angle;
}

bool Edge::isConvex(const Block* pBlock, const Index3DId& refCellId) const
{
	return calDihedralAngleRadians(pBlock, refCellId) >= -Tolerance::angleTol();
}

bool Edge::isOriented(const Block* pBlock, const Index3DId& refCellId) const
{
	bool result = true;
	if (_faceIds.size() != 2)
		return false;

	const auto& id0 = _faceIds[0];
	const auto& id1 = _faceIds[1];

	pBlock->faceFunc(id0, [pBlock, &id1, &refCellId, &result](const Polygon& face0) {
		face0.iterateOrientedEdges([pBlock, &id1, &refCellId, &result](const auto& edge0)->bool {
			pBlock->faceFunc(id1, [&refCellId, &edge0, &result](const Polygon& face1) {
				face1.iterateOrientedEdges([&edge0, &result](const auto& edge1)->bool {
					if (edge0._vertexIds[0] == edge1._vertexIds[0] && edge0._vertexIds[1] == edge1._vertexIds[1]) {
						result = false;
					}
					return result;
				}, refCellId);
			});
			return result;
		}, refCellId);
	});

	return result;
}

LineSegmentd Edge::getSegment(const Block* pBlock) const
{
	Vector3d pt0 = pBlock->getVertexPoint(_vertexIds[0]);
	Vector3d pt1 = pBlock->getVertexPoint(_vertexIds[1]);
	LineSegmentd result(pt0, pt1);
	return result;
}

bool Edge::containsVertex(const Index3DId& vertexId) const
{
	return _vertexIds[0] == vertexId || _vertexIds[1] == vertexId;
}

bool Edge::vertexLiesOnEdge(const Block* pBlock, const Index3DId& vertexId) const
{
	const auto& pt = pBlock->getVertexPoint(vertexId);
	return pointLiesOnEdge(pBlock, pt);
}

bool Edge::pointLiesOnEdge(const Block* pBlock, const Vector3d& pt) const
{
	const double tol = Tolerance::sameDistTol();
	auto seg = getSegment(pBlock);
	double t;
	return seg.contains(pt, t, tol);

}

Index3DId Edge::getOtherVert(const Index3DId& vert) const
{
	if (vert == _vertexIds[0])
		return _vertexIds[1];
	else if (vert == _vertexIds[1])
		return _vertexIds[0];
	return Index3DId();
}

void Edge::getFaceIds(FastBisectionSet<Index3DId>& faceIds) const
{
	faceIds = _faceIds;
}

void Edge::getCellIds(const Block* pBlock, MTC::set<Index3DId>& cellIds) const
{
	cellIds.clear();
	for (const auto& faceId : _faceIds.asVector()) {
		if (pBlock->polygonExists(faceId)) {
			pBlock->faceFunc(faceId, [&pBlock, &cellIds](const Polygon& face) {
				const auto& adjCellIds = face.getCellIds();
				for (const auto& cellId : adjCellIds.asVector()) {
					if (pBlock->polyhedronExists(cellId)) {
						cellIds.insert(cellId);
					}
				}
			});
		}
	}
}

void Edge::write(std::ostream& out) const
{
	uint8_t version = 0;
	out.write((char*)&version, sizeof(uint8_t));

	_vertexIds[0].write(out);
	_vertexIds[1].write(out);

	size_t num = _faceIds.size();
	out.write((char*)&num, sizeof(size_t));
	for (const auto& id : _faceIds.asVector())
		id.write(out);
}

void Edge::read(std::istream& in)
{
	uint8_t version;
	in.read((char*)&version, sizeof(uint8_t));

	_vertexIds[0].read(in);
	_vertexIds[1].read(in);

	size_t num = _faceIds.size();
	in.read((char*)&num, sizeof(size_t));
	for (size_t i = 0; i < num; i++) {
		Index3DId id;
		id.read(in);
		_faceIds.insert(id);
	}
}

ostream& DFHM::operator << (ostream& out, const Edge& edge)
{
	out << "Edge: e(v" << edge.getVertexIds()[0] << " v" << edge.getVertexIds()[1] << ")\n";
	{
		Logger::Indent indent;

		out << Logger::Pad() << "faceIds(" << edge.getFaceIds().size() << "): {";
		for (const auto& faceId : edge.getFaceIds().asVector()) {
			out << "f" << faceId << " ";
		}
		out << "}\n";
	}

	return out;
}
