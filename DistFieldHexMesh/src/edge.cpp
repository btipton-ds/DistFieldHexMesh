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
	: _faceIds(faceIds)
	, _reversed(vert1 < vert0)
{
	_vertexIds[0] = vert0;
	_vertexIds[1] = vert1;
}

Edge::Edge(const Edge& src, const MTC::set<Index3DId>& faceIds)
	: _faceIds(faceIds)
{
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

double Edge::calDihedralAngleRadians(const Block* pBlock) const
{
	assert(_faceIds.size() == 2);
	auto fIter = _faceIds.begin();
	const auto& faceId0 = *fIter++;
	const auto& faceId1 = *fIter++;
	const auto& cellId = getCommonCellId(pBlock, faceId0, faceId1);
	Planed plane0, plane1;
	pBlock->faceFunc(TS_REAL,faceId0, [&plane0, &cellId](const Polygon& face) { 
		plane0 = face.calOrientedPlane(cellId); 
	});
	pBlock->faceFunc(TS_REAL,faceId1, [&plane1, &cellId](const Polygon& face) { 
		plane1 = face.calOrientedPlane(cellId);
	});

	Vector3d edgeCtr = calCenter(pBlock);
	Vector3d edgeDir = calUnitDir(pBlock);
	Vector3d xAxis = edgeCtr - plane0.getOrgin(); // xAxis points from plane0 center to the edge
	xAxis = xAxis - edgeDir.dot(xAxis) * edgeDir;
	xAxis.normalize();

	const auto& zAxis = plane0.getNormal();

	Vector3d v = plane1.getOrgin() - edgeCtr; // v points from the edge to plane1 center. If they are identical, the angle is zero
	double cosTheta = v.dot(xAxis);
	double sinTheta = -v.dot(zAxis);
	double angle = atan2(sinTheta, cosTheta);

	return angle;
}

bool Edge::isConvex(const Block* pBlock) const
{
	return calDihedralAngleRadians(pBlock) >= -Tolerance::angleTol();
}

bool Edge::isOriented(const Block* pBlock) const
{
	bool result = true;
	if (_faceIds.size() != 2)
		return false;

	auto iter = _faceIds.begin();
	const auto& id0 = *iter++;
	const auto& id1 = *iter;

	auto cellId = getCommonCellId(pBlock, id0, id1);
	if (!cellId.isValid())
		return false;


	pBlock->faceFunc(TS_REAL, id0, [pBlock, &id1, &cellId, &result](const Polygon& face0) {
		face0.iterateOrientedEdges([pBlock, &id1, &cellId, &result](const auto& edge0)->bool {
			pBlock->faceFunc(TS_REAL, id1, [&cellId, &edge0, &result](const Polygon& face1) {
				face1.iterateOrientedEdges([&edge0, &result](const auto& edge1)->bool {
					if (edge0._vertexIds[0] == edge1._vertexIds[0] && edge0._vertexIds[1] == edge1._vertexIds[1]) {
						result = false;
					}
					return result;
				}, cellId);
			});
			return result;
		}, cellId);
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

Index3DId Edge::getCommonCellId(const Block* pBlock, const auto& id0, const auto& id1) const
{
	Index3DId result;

	MTC::set<Index3DId> faceCells0, faceCells1, commonCellIds;
	pBlock->faceFunc(TS_REAL, id0, [&faceCells0](const Polygon& face0) {
		faceCells0 = face0.getCellIds();
	});
	pBlock->faceFunc(TS_REAL, id1, [&faceCells1](const Polygon& face1) {
		faceCells1 = face1.getCellIds();
	});

	for (const auto& ci : faceCells0) {
		if (faceCells1.contains(ci))
			commonCellIds.insert(ci);
	}
	for (const auto& ci : faceCells1) {
		if (faceCells0.contains(ci))
			commonCellIds.insert(ci);
	}

	if (commonCellIds.size() == 1)
		result = *commonCellIds.begin();
	
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

void Edge::getFaceIds(MTC::set<Index3DId>& faceIds) const
{
	faceIds = _faceIds;
}

void Edge::write(std::ostream& out) const
{
	uint8_t version = 0;
	out.write((char*)&version, sizeof(uint8_t));

	_vertexIds[0].write(out);
	_vertexIds[1].write(out);

	size_t num = _faceIds.size();
	out.write((char*)&num, sizeof(size_t));
	for (const auto& id : _faceIds)
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
		for (const auto& faceId : edge.getFaceIds()) {
			out << "f" << faceId << " ";
		}
		out << "}\n";
	}

	return out;
}
