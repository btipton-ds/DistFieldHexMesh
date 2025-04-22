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
#include <polyMesh.h>
#include <volume.h>
#include <splitParams.h>

using namespace std;
using namespace DFHM;

Edge::Edge(const EdgeKey& src, const Block* pBlock)
	: EdgeKey(src)
	, _pBlock(const_cast<Block*>(pBlock))
{
}

Edge::Edge(const EdgeKey& src, const PolyMesh* pPolyMesh)
	: EdgeKey(src)
	, _pPolyMesh(const_cast<PolyMesh*>(pPolyMesh))
{
}

void Edge::initFaceIds() const
{
	_faceIds.clear();

	const auto& vert0 = getVertex(_vertexIds[0]);
	auto& face0Ids = vert0.getFaceIds();

	const auto& vert1 = getVertex(_vertexIds[1]);
	auto& face1Ids = vert1.getFaceIds();

	auto p0 = face0Ids.data();
	auto p1 = face1Ids.data();

	if (face0Ids.size() < face1Ids.size()) {
		for (size_t i = 0; i < face0Ids.size(); i++) {
			if (face1Ids.contains(*p0)) {
				_faceIds.insert(*p0);
			}
			p0++;
		}
	} else {
		for (size_t i = 0; i < face1Ids.size(); i++) {
			if (face0Ids.contains(*p1)) {
				_faceIds.insert(*p1);
			}
			p1++;
		}
	}
}

const Vector3d& Edge::getVertexPoint(const Index3DId& id) const
{
	if (_pBlock)
		return _pBlock->getVertexPoint(id);
	else if (_pPolyMesh)
		return _pPolyMesh->getVertexPoint(id);

	throw runtime_error("Bad vertex id");
}

double Edge::sameParamTol() const
{
	return Tolerance::sameDistTol() / getLength();
}

double Edge::getLength() const
{
	LineSegmentd seg(getVertexPoint(_vertexIds[0]), getVertexPoint(_vertexIds[1]));
	return seg.calLength();
}

Vector3d Edge::calCenter() const
{
	return calPointAt(0.5);
}

Vector3d Edge::calUnitDir() const
{
	Vector3d pt0 = getVertexPoint(_vertexIds[0]);
	Vector3d pt1 = getVertexPoint(_vertexIds[1]);
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
	Vector3d pt0 = getVertexPoint(_vertexIds[0]);
	Vector3d pt1 = getVertexPoint(_vertexIds[1]);
	
	Vector3d result = pt0 + t * (pt1 - pt0);

	return result;
}

double Edge::paramOfPt(const Vector3d& pt, bool& inBounds) const
{
	Vector3d pt0 = getVertexPoint(_vertexIds[0]);
	Vector3d pt1 = getVertexPoint(_vertexIds[1]);
	Vector3d v = pt1 - pt0;
	double len = v.norm();
	v.normalize();

	Vector3d v1 = pt - pt0;
	double t = v.dot(v1) / len;
	inBounds = (t > -Tolerance::paramTol()) && (t < 1 + Tolerance::paramTol());

	return t;
}

double Edge::calLength() const
{
	Vector3d pt0 = getVertexPoint(_vertexIds[0]);
	Vector3d pt1 = getVertexPoint(_vertexIds[1]);
	Vector3d v = pt1 - pt0;
	return v.norm();
}

double Edge::calCurvature(const SplittingParams& params) const
{
	initFaceIds();
	if (_faceIds.size() != 2)
		return 0;

	auto iter = _faceIds.begin();
	auto faceId0 = *iter++;
	auto faceId1 = *iter;

	auto& face0 = getPolygon(faceId0);
	auto& face1 = getPolygon(faceId1);

	MTC::vector<Index3DId> nclVerts0, nclVerts1;
	if (_pBlock) {
		nclVerts0 = PolygonSearchKey::makeNonColinearVertexIds(_pBlock, face0.getVertexIds());
		nclVerts1 = PolygonSearchKey::makeNonColinearVertexIds(_pBlock, face1.getVertexIds());
	} else if (_pPolyMesh) {
		nclVerts0 = PolygonSearchKey::makeNonColinearVertexIds(_pPolyMesh, face0.getVertexIds());
		nclVerts1 = PolygonSearchKey::makeNonColinearVertexIds(_pPolyMesh, face1.getVertexIds());
	}
	for (int i = 0; i < 2; i++) {
		auto iter = find(nclVerts0.begin(), nclVerts0.end(), _vertexIds[i]);
		if (iter != nclVerts0.end())
			nclVerts0.erase(iter);

		iter = find(nclVerts1.begin(), nclVerts1.end(), _vertexIds[i]);
		if (iter != nclVerts1.end())
			nclVerts1.erase(iter);
	}

	double curvature = 0;
	int count = 0;
	const auto& pt0 = getVertexPoint(_vertexIds[0]);
	const auto& pt1 = getVertexPoint(_vertexIds[1]);
	for (const auto& id0 : nclVerts0) {
		const auto& pt2 = getVertexPoint(id0);
		for (const auto& id1 : nclVerts1) {
			const auto& pt3 = getVertexPoint(id1);
			double c = calCurvature(pt0, pt1, pt2, pt3, params);
			if (c >= 0) {
				curvature += c;
				count++;
			}
		}
	}

	if (count > 0)
		curvature /= count;

	return curvature;
}

double Edge::calCurvature(const Vector3d& origin, const Vector3d& ptAxis, const Vector3d& pt0, const Vector3d& pt1, const SplittingParams& params) const
{
	Vector3d axis = (ptAxis - origin).normalized();

	Vector3d v0 = pt0 - origin;
	v0 = v0 - v0.dot(axis) * axis;
	v0 = v0 - v0.dot(axis) * axis;
	double l0 = v0.norm();
	if (l0 < Tolerance::sameDistTol())
		return -1;
	v0 /= l0;
	assert(fabs(v0.dot(axis)) < Tolerance::looseParamTol());

	if (axis.cross(v0).norm() < 0.01)
		return -1;

	Vector3d v1 = (pt1 - origin);
	v1 = v1 - v1.dot(axis) * axis;
	v1 = v1 - v1.dot(axis) * axis;
	double l1 = v1.norm();
	if (l1 < Tolerance::sameDistTol())
		return -1; // pt3 is colinear with the axis
	v1 /= l1;
	assert(fabs(v1.dot(axis)) < Tolerance::looseParamTol());

	if (axis.cross(v1).norm() < 0.01)
		return -1;

	Vector3d ptB = origin + l0 * v0;
	Vector3d ptC = origin + l1 * v1;

	Vector3d orthAxis = (origin - ptB).normalized();

	Vector3d vCA = ptC - origin;
	Vector3d vAB = origin - ptB;
	Vector3d vBC = ptB - ptC;

	double area = fabs(0.5 * vBC.cross(vAB).dot(axis));
	if (area < Tolerance::sameDistTolSqr())
		return -1;

	double dotProdCA = vCA.dot(orthAxis);
	Vector3d perp = orthAxis.cross(axis).normalized();
	assert(fabs(axis.dot(perp)) < Tolerance::paramTol());
	double dotPerp = vCA.dot(perp);
	double angle = atan2(dotPerp, dotProdCA);
	if (fabs(angle) > params.getSharpAngleRadians())
		return DBL_MAX; // Sharp edge, infinite curvature

	double ca = vCA.norm();
	double ab = vAB.norm();
	double bc = vBC.norm();
	double r = ca * ab * bc / area * 0.25;

	double c = 0;
	if (r > 0.001)
		c = 1 / r;
	return c;
}

Vector3d Edge::projectPt(const Vector3d& pt) const
{
	Vector3d pt0 = getVertexPoint(_vertexIds[0]);
	Vector3d pt1 = getVertexPoint(_vertexIds[1]);
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
	Vector3d pt0 = getVertexPoint(other._vertexIds[0]);

	if (seg.distanceToPoint(pt0) > Tolerance::sameDistTol())
		return false;

	Vector3d pt1 = getVertexPoint(other._vertexIds[1]);
	if (seg.distanceToPoint(pt1) > Tolerance::sameDistTol())
		return false;

	return true;
}

bool Edge::isColinearWith(const Index3DId& vert, double& param) const
{
	Vector3d pt = getVertexPoint(vert);
	Vector3d pt0 = getVertexPoint(_vertexIds[0]);
	Vector3d pt1 = getVertexPoint(_vertexIds[1]);

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
	if (_faceIds.empty())
		initFaceIds();

	if (_faceIds.size() != 2)
		return 0;

	auto iter = _faceIds.begin();
	const auto& faceId0 = *iter++;
	const auto& faceId1 = *iter;
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

	if (_faceIds.empty())
		initFaceIds();

	if (_faceIds.size() != 2)
		return false;

	auto iter = _faceIds.begin();
	const auto& id0 = *iter++;
	const auto& id1 = *iter;

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
	Vector3d pt0 = getVertexPoint(_vertexIds[0]);
	Vector3d pt1 = getVertexPoint(_vertexIds[1]);
	LineSegmentd result(pt0, pt1);
	return result;
}

bool Edge::containsFace(const Index3DId& faceId) const
{
	if (_faceIds.empty())
		initFaceIds();
	return _faceIds.contains(faceId);
}

bool Edge::vertexLiesOnEdge(const Index3DId& vertexId) const
{
	const auto& pt = getVertexPoint(vertexId);
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
	auto& faceIds = getFaceIds();

	for (const auto& faceId : faceIds) {
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
}

void Edge::read(std::istream& in)
{
	uint8_t version;
	in.read((char*)&version, sizeof(uint8_t));

	_vertexIds[0].read(in);
	_vertexIds[1].read(in);
}

bool Edge::verifyTopology() const
{
	bool result = true;

	if (_faceIds.empty())
		initFaceIds();

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
	if (p) 
		p->vertexFunc(id, func); 
	else {
		const auto p2 = getPolyMeshPtr(); 
		if (p2) 
			p2->vertexFunc(id, func);
	}
} 

void Edge::vertexFunc(const Index3DId& id, const std::function<void(Vertex& obj)>& func) {
	auto p = getBlockPtr(); 
	if (p) 
		p->vertexFunc(id, func); 
	else {
		auto p2 = getPolyMeshPtr(); 
		if (p2) 
			p2->vertexFunc(id, func);
	}
} 

void Edge::faceFunc(const Index3DId& id, const std::function<void(const Polygon& obj)>& func) const {
	const auto p = getBlockPtr(); 
	if (p) 
		p->faceFunc(id, func); 
	else {
		const auto p2 = getPolyMeshPtr(); 
		if (p2) 
			p2->faceFunc(id, func);
	}
} 

void Edge::faceFunc(const Index3DId& id, const std::function<void(Polygon& obj)>& func) {
	auto p = getBlockPtr(); 
	if (p) 
		p->faceFunc(id, func); 
	else {
		auto p2 = getPolyMeshPtr(); 
		if (p2) 
			p2->faceFunc(id, func);
	}
} 

void Edge::cellFunc(const Index3DId& id, const std::function<void(const Polyhedron& obj)>& func) const {
	const auto p = getBlockPtr(); 
	if (p) 
		p->cellFunc(id, func); 
	else {
		const auto p2 = getPolyMeshPtr(); 
		if (p2) 
			p2->cellFunc(id, func);
	}
} 

void Edge::cellFunc(const Index3DId& id, const std::function<void(Polyhedron& obj)>& func) {
	auto p = getBlockPtr(); 
	if (p) 
		p->cellFunc(id, func); 
	else {
		auto p2 = getPolyMeshPtr(); 
		if (p2) 
			p2->cellFunc(id, func);
	}
} 

const Vertex& Edge::getVertex(const Index3DId& id) const {
	const auto p = getBlockPtr(); 
	if (p) 
		return p->getVertex(id); 
	else {
		const auto p2 = getPolyMeshPtr(); 
		if (p2) 
			return p2->getVertex(id);
	} 
	throw std::runtime_error("Entity does not exist");
}  

Vertex& Edge::getVertex(const Index3DId& id) {
	auto p = getBlockPtr(); 
	if (p) 
		return p->getVertex(id); 
	else {
		auto p2 = getPolyMeshPtr(); 
		if (p2) 
			return p2->getVertex(id);
	} 
	throw std::runtime_error("Entity does not exist");
} 

const DFHM::Polygon& Edge::getPolygon(const Index3DId& id) const {
	const auto p = getBlockPtr(); 
	if (p) 
		return p->getPolygon(id); 
	else {
		const auto p2 = getPolyMeshPtr(); 
		if (p2) 
			return p2->getPolygon(id);
	} 
	throw std::runtime_error("Entity does not exist");
}  

DFHM::Polygon& Edge::getPolygon(const Index3DId& id) {
	auto p = getBlockPtr(); if (p) 
		return p->getPolygon(id); 
	else {
		auto p2 = getPolyMeshPtr(); 
		if (p2) 
			return p2->getPolygon(id);
	} 
	throw std::runtime_error("Entity does not exist");
} 

const Polyhedron& Edge::getPolyhedron(const Index3DId& id) const {
	const auto p = getBlockPtr(); if (p) 
		return p->getPolyhedron(id); 
	else {
		const auto p2 = getPolyMeshPtr(); 
		if (p2) 
			return p2->getPolyhedron(id);
	} 
	throw std::runtime_error("Entity does not exist");
}  

Polyhedron& Edge::getPolyhedron(const Index3DId& id) {
	auto p = getBlockPtr(); if (p) 
		return p->getPolyhedron(id); 
	else {
		auto p2 = getPolyMeshPtr(); 
		if (p2) 
			return p2->getPolyhedron(id);
	} 
	throw std::runtime_error("Entity does not exist");
} 

void Edge::edgeFunc(const EdgeKey& key, const std::function<void(const Edge& obj)>& func) const {
	const auto p = getBlockPtr(); if (p) 
		p->edgeFunc(key, func); 
	else {
		const auto p2 = getPolyMeshPtr(); 
		if (p2) 
			p2->edgeFunc(key, func);
	}
} 

void Edge::edgeFunc(const EdgeKey& key, const std::function<void(Edge& obj)>& func) {
	auto p = getBlockPtr(); if (p) 
		p->edgeFunc(key, func); 
	else {
		auto p2 = getPolyMeshPtr(); 
		if (p2) 
			p2->edgeFunc(key, func);
	}
}
