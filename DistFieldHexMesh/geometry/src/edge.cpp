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
#include <tolerances.h>
#include <logger.h>
#include <io_utils.h>
#include <edge.h>
#include <block.h>
#include <volume.h>

using namespace std;
using namespace DFHM;

Edge::Edge(const EdgeKey& src, const Block* pBlock)
	: EdgeKey(src)
	, _pBlock(const_cast<Block*>(pBlock))
{
	initFaceIds();
}

void Edge::initFaceIds()
{
	_faceIds.clear();

	vertexFunc(_vertexIds[0], [this](const Vertex& vert0) {
		auto& face0Ids = vert0.getFaceIds();

		vertexFunc(_vertexIds[1], [this, &face0Ids](const Vertex& vert1) {
			auto& face1Ids = vert1.getFaceIds();

			for (const auto& id0 : face0Ids) {
				if (face1Ids.contains(id0))
					_faceIds.insert(id0);
			}
		});
	});


}

double Edge::sameParamTol() const
{
	return Tolerance::sameDistTol() / getLength();
}

double Edge::getLength() const
{
	LineSegmentd seg(getBlockPtr()->getVertexPoint(_vertexIds[0]), getBlockPtr()->getVertexPoint(_vertexIds[1]));
	return seg.calLength();
}

Vector3d Edge::calCenter() const
{
	return calPointAt(0.5);
}

Vector3d Edge::calUnitDir() const
{
	Vector3d pt0 = getBlockPtr()->getVertexPoint(_vertexIds[0]);
	Vector3d pt1 = getBlockPtr()->getVertexPoint(_vertexIds[1]);
	Vector3d v = pt1 - pt0;
	v.normalize();
	return v;
}

Vector3d Edge::calCoedgeUnitDir(const Index3DId& faceId, const Index3DId& cellId) const
{
	Vector3d result;
	bool found = false;
	getBlockPtr()->faceFunc(faceId, [this, &cellId, &result, &found](const Polygon& face) {
		face.iterateOrientedEdges([this, &cellId, &result, &found](const Edge& edge)->bool {
			if (edge == *this) {
				result = edge.calUnitDir();
				found = true;
			}
			return !found; // continue the loop
		}, cellId);
	});

	assert(found);
	return result;
}

Vector3d Edge::calPointAt(double t) const
{
	Vector3d pt0 = getBlockPtr()->getVertexPoint(_vertexIds[0]);
	Vector3d pt1 = getBlockPtr()->getVertexPoint(_vertexIds[1]);
	
	Vector3d result = pt0 + t * (pt1 - pt0);

	return result;
}

double Edge::paramOfPt(const Vector3d& pt, bool& inBounds) const
{
	Vector3d pt0 = getBlockPtr()->getVertexPoint(_vertexIds[0]);
	Vector3d pt1 = getBlockPtr()->getVertexPoint(_vertexIds[1]);
	Vector3d v = pt1 - pt0;
	double len = v.norm();
	v.normalize();

	Vector3d v1 = pt - pt0;
	double t = v.dot(v1) / len;
	inBounds = (t > -Tolerance::paramTol()) && (t < 1 + Tolerance::paramTol());

	return t;
}

Vector3d Edge::projectPt(const Vector3d& pt) const
{
	Vector3d pt0 = getBlockPtr()->getVertexPoint(_vertexIds[0]);
	Vector3d pt1 = getBlockPtr()->getVertexPoint(_vertexIds[1]);
	Vector3d v = pt1 - pt0;
	v.normalize();

	Vector3d v1 = pt - pt0;
	double t = v.dot(v1);

	Vector3d result = pt0 + t * v;
	return result;
}

bool Edge::onPrincipalAxis() const
{
	const double tol = Tolerance::paramTol();
	Vector3d dir = calUnitDir();
	bool result = false;
	for (int i = 0; i < 3; i++) {
		Vector3d axis(0, 0, 0);
		axis[i] = 1;
		if (axis.cross(dir).norm() < tol)
			return true;
	}
	return false;
}

bool Edge::isColinearWith(const Edge& other) const
{
	LineSegment seg(getSegment());
	Vector3d pt0 = getBlockPtr()->getVertexPoint(other._vertexIds[0]);

	if (seg.distanceToPoint(pt0) > Tolerance::sameDistTol())
		return false;

	Vector3d pt1 = getBlockPtr()->getVertexPoint(other._vertexIds[1]);
	if (seg.distanceToPoint(pt1) > Tolerance::sameDistTol())
		return false;

	return true;
}

bool Edge::isColinearWith(const Index3DId& vert, double& param) const
{
	Vector3d pt = getBlockPtr()->getVertexPoint(vert);
	Vector3d pt0 = getBlockPtr()->getVertexPoint(_vertexIds[0]);
	Vector3d pt1 = getBlockPtr()->getVertexPoint(_vertexIds[1]);

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

double Edge::calDihedralAngleRadians(const Index3DId& refCellId) const
{
	if (_faceIds.size() != 2)
		return 0;
	const auto& faceId0 = _faceIds[0];
	const auto& faceId1 = _faceIds[1];
	Vector3d normal0, normal1;
	getBlockPtr()->faceFunc(faceId0, [&normal0, &refCellId](const Polygon& face) {
		normal0 = face.calOrientedUnitNormal(refCellId);
	});
	getBlockPtr()->faceFunc(faceId1, [&normal1, &refCellId](const Polygon& face) {
		normal1 = face.calOrientedUnitNormal(refCellId);
	});

	const auto& zAxis = normal0;
	Vector3d yAxis = calCoedgeUnitDir(faceId0, refCellId);
	Vector3d xAxis = yAxis.cross(zAxis); // xAxis points from plane0 center to the edge

	double cosTheta = -normal1.dot(zAxis);
	double sinTheta = normal1.dot(xAxis);
	double angle = atan2(sinTheta, cosTheta);

	return angle;
}

bool Edge::isConvex(const Index3DId& refCellId) const
{
	const auto angle = calDihedralAngleRadians(refCellId);
	const auto tol = Tolerance::angleTol();
	return angle >= -tol;
}

bool Edge::isOriented(const Index3DId& refCellId) const
{
	bool result = true;
	if (_faceIds.size() != 2)
		return false;

	const auto& id0 = _faceIds[0];
	const auto& id1 = _faceIds[1];

	getBlockPtr()->faceFunc(id0, [this, &id1, &refCellId, &result](const Polygon& face0) {
		face0.iterateOrientedEdges([this, &id1, &refCellId, &result](const auto& edge0)->bool {
			getBlockPtr()->faceFunc(id1, [&refCellId, &edge0, &result](const Polygon& face1) {
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

LineSegmentd Edge::getSegment() const
{
	Vector3d pt0 = getBlockPtr()->getVertexPoint(_vertexIds[0]);
	Vector3d pt1 = getBlockPtr()->getVertexPoint(_vertexIds[1]);
	LineSegmentd result(pt0, pt1);
	return result;
}

bool Edge::containsFace(const Index3DId& faceId) const
{
	return _faceIds.contains(faceId);
}

bool Edge::imprintVertices(const set<Index3DId>& allVertIds)
{
	const double tol = Tolerance::paramTol();

	auto seg = getSegment();
	set<Index3DId> vertsInBoundsSet;
	for (const auto& id : allVertIds) {
#if VALIDATION_ON
		if (!_vertexIds[0].withinRange(id) || !_vertexIds[1].withinRange(id))
			continue;
#endif

		const auto& pt = getBlockPtr()->getVertexPoint(id);
		double t;
		if (seg.contains(pt, t, tol) && tol < t && t < 1 - tol) {
			vertsInBoundsSet.insert(id);
		}
	}

	if (vertsInBoundsSet.empty())
		return false;

	MTC::vector<Vector3d> vertsInBounds;
	for (const auto& id : vertsInBoundsSet)
		vertsInBounds.push_back(getBlockPtr()->getVertexPoint(id));

	bool result = false;
	for (const auto& faceId : _faceIds) {
		faceFunc(faceId, [this, &vertsInBounds, &result](Polygon& face) {
			result = face.imprintPoints(vertsInBounds);
		});
	}

	return result;
}

bool Edge::vertexLiesOnEdge(const Index3DId& vertexId) const
{
	const auto& pt = getBlockPtr()->getVertexPoint(vertexId);
	return pointLiesOnEdge(pt);
}

bool Edge::pointLiesOnEdge(const Vector3d& pt) const
{
	const double tol = Tolerance::sameDistTol();
	auto seg = getSegment();
	double t;
	return seg.contains(pt, t, tol);

}

MTC::set<Index3DId> Edge::getCellIds() const
{
	MTC::set<Index3DId> result;
	for (const auto& faceId : _faceIds) {
		if (getBlockPtr()->polygonExists(faceId)) {
			getBlockPtr()->faceFunc(faceId, [this, &result](const Polygon& face) {
				const auto& adjCellIds = face.getCellIds();
				for (const auto& cellId : adjCellIds) {
					if (getBlockPtr()->polyhedronExists(cellId)) {
						result.insert(cellId);
					}
				}
			});
		}
	}

	return result;
}

void Edge::write(std::ostream& out) const
{
	uint8_t version = 0;
	out.write((char*)&version, sizeof(uint8_t));

	_vertexIds[0].write(out);
	_vertexIds[1].write(out);

	IoUtil::writeObj(out, _faceIds);
}

void Edge::read(std::istream& in)
{
	uint8_t version;
	in.read((char*)&version, sizeof(uint8_t));

	_vertexIds[0].read(in);
	_vertexIds[1].read(in);

	IoUtil::readObj(in, _faceIds);
}

bool Edge::verifyTopology() const
{
	bool result = true;
	for (const auto& faceId : _faceIds) {
		faceFunc(faceId, [this, &result](const Polygon& face) {
			if (!face.containsEdge(*this))
				result = false;
		});
	}

	return result;
}

ostream& DFHM::operator << (ostream& out, const EdgeKey& edge)
{
	out << "Edge: e(v" << edge.getVertexIds()[0] << " v" << edge.getVertexIds()[1] << ")\n";

	return out;
}

//LAMBDA_CLIENT_IMPLS(Edge)
void Edge::vertexFunc(const Index3DId& id, const std::function<void(const Vertex& obj)>& func) const {
	const auto p = getBlockPtr(); 
	p->vertexFunc(id, func);
} 
void Edge::vertexFunc(const Index3DId& id, const std::function<void(Vertex& obj)>& func) {
	auto p = getBlockPtr(); 
	p->vertexFunc(id, func);
} 

void Edge::faceFunc(const Index3DId& id, const std::function<void(const Polygon& obj)>& func) const {
	const auto p = getBlockPtr(); 
	p->faceFunc(id, func);
} 

void Edge::faceFunc(const Index3DId& id, const std::function<void(Polygon& obj)>& func) {
	auto p = getBlockPtr(); 
	p->faceFunc(id, func);
} 

void Edge::cellFunc(const Index3DId& id, const std::function<void(const Polyhedron& obj)>& func) const {
	const auto p = getBlockPtr(); 
	p->cellFunc(id, func);
} 

void Edge::cellFunc(const Index3DId& id, const std::function<void(Polyhedron& obj)>& func) {
	auto p = getBlockPtr(); 
	p->cellFunc(id, func);
} 

void Edge::edgeFunc(const EdgeKey& key, const std::function<void(const Edge& obj)>& func) const {
	const auto p = getBlockPtr(); 
	p->edgeFunc(key, func);
} 

void Edge::edgeFunc(const EdgeKey& key, const std::function<void(Edge& obj)>& func) {
	auto p = getBlockPtr(); 
	p->edgeFunc(key, func);
}
