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

#include <tolerances.h>
#include <fixedPoint.h>
#include <io_utils.h>
#include <vertex.h>
#include <edge.h>
#include <polygon.h>
#include <block.h>

using namespace std;
using namespace DFHM;

Vertex::Vertex(const Vertex& src)
	: ObjectPoolOwnerUser(src)
	, _pt(src._pt)
	, _lockType(src._lockType)
{
}

void Vertex::setId(const Index3DId& id)
{
	_thisId = id;
}

void Vertex::remapId(const std::vector<size_t>& idRemap, const Index3D& srcDims)
{
	remap(idRemap, srcDims, _thisId);
	remap(idRemap, srcDims, _connectedVertices);
}

Vertex& Vertex::operator = (const Vertex& rhs)
{
	ObjectPoolOwnerUser::operator= (rhs);
	_pt = rhs._pt;
	_lockType = rhs._lockType;

	return *this;
}

MTC::set<EdgeKey> Vertex::getEdges() const
{
	MTC::set<EdgeKey> result;

	for (const auto& otherId : _connectedVertices) {
		EdgeKey edgeKey(getId(), otherId);
		result.insert(edgeKey);
	}

	return result;
}

MTC::set<Index3DId> Vertex::getFaceIds(bool activeOnly) const
{
	MTC::set<Index3DId> result;

	for (const auto& otherId : _connectedVertices) {
		EdgeKey edgeKey(getId(), otherId);
		edgeFunc(edgeKey, [this, activeOnly, &result](const Edge& edge) {
			auto faceIds = edge.getFaceIds();
			if (activeOnly) {
				for (const auto& faceId : faceIds) {
					faceFunc(faceId, [&result](const Polygon& face) {
						if (!face.getCellIds().empty()) {
							result.insert(face.getId());
						}
					});
				}
			} else {
				result.insert(faceIds.begin(), faceIds.end());
			}
		});
	}

	return result;
}

MTC::set<Index3DId> Vertex::getCellIds(bool activeOnly) const
{
	MTC::set<Index3DId> result;
	MTC::set<Index3DId> faceIds = getFaceIds(activeOnly);

	for (const auto& faceId : faceIds) {
		faceFunc(faceId, [&result](const Polygon& face) {
			const auto& cellIds = face.getCellIds();
			result.insert(cellIds.begin(), cellIds.end());
		});
	}

	return result;
}

void Vertex::write(std::ostream& out) const
{
	uint8_t version = 0;
	out.write((char*)&version, sizeof(version));

	writeVector3(out, _pt);

	out.write((char*)&_lockType, sizeof(_lockType));
	IoUtil::writeObj(out, _connectedVertices);
}

void Vertex::read(std::istream& in)
{
	uint8_t version;
	in.read((char*)&version, sizeof(version));

	readVector3(in, _pt);

	in.read((char*)&_lockType, sizeof(_lockType));
	IoUtil::read(in, _connectedVertices);
}

CBoundingBox3Dd Vertex::calBBox(const Vector3d& pt)
{
	CBoundingBox3Dd result(pt, pt);
	result.grow(Tolerance::sameDistTol() / 2.0);

	return result;
}

inline int64_t Vertex::scaleToSearch()
{
	const int64_t micronInv = 1000000;
	return 100 * micronInv; // 1/100 micron
}

inline int64_t Vertex::scaleToSearch(double v)
{
	return (int64_t)(v * scaleToSearch());
}

inline Vector3<int64_t> Vertex::scaleToSearch(const Vector3d& pt)
{
	return Vector3<int64_t>(scaleToSearch(pt[0]), scaleToSearch(pt[1]), scaleToSearch(pt[2]));
}

const bool Vertex::operator < (const Vertex& rhs) const
{
	/*
		this is an innefficient implementation of Edelsbrunner's and Mucke's simulation of simplicity - https://arxiv.org/abs/math/9410209.

		Their approach was to simulate floating point numbers with fixed point integers. This avoids tolerancing errors when doing <, ==, > testing.
		It effectively "rounds" each floating point value to a scaled integer value.

		The original implementation stores the fixed point value and all math is done in that form. That makes floating point math extremely tricky to overflows and underflows.

		This approach stores a double precision value of the point, and only converts it for comparisons. It's a bit time consuming, but it solves vertex sharing across
		blocks which is a key to high performance multithreading.

	*/

	Vector3<int64_t> iPt(scaleToSearch(_pt)), iRhsPt(scaleToSearch(rhs._pt));
	return iPt < iRhsPt;
}

const bool Vertex::operator > (const Vertex& rhs) const
{
	return rhs < *this;
}

const bool Vertex::operator == (const Vertex& rhs) const
{
	Vector3<int64_t> iPt(scaleToSearch(_pt)), iRhsPt(scaleToSearch(rhs._pt));
	return !iPt.operator<(iRhsPt) && !iRhsPt.operator<(iPt);
}

const bool Vertex::operator != (const Vertex& rhs) const
{
	return !operator==(rhs);
}

ostream& DFHM::operator << (ostream& out, const Vertex& vert)
{
	out << "Vertex " << vert.getId();
	return out;
}

//LAMBDA_CLIENT_IMPLS(Vertex)
void Vertex::vertexFunc(const Index3DId& id, const std::function<void(const Vertex& obj)>& func) const {
	const auto p = getBlockPtr(); 
	p->vertexFunc(id, func);
} 

void Vertex::vertexFunc(const Index3DId& id, const std::function<void(Vertex& obj)>& func) {
	auto p = getBlockPtr(); 
	p->vertexFunc(id, func);
} 

void Vertex::faceFunc(const Index3DId& id, const std::function<void(const Polygon& obj)>& func) const {
	const auto p = getBlockPtr(); 
	p->faceFunc(id, func);
} 

void Vertex::faceFunc(const Index3DId& id, const std::function<void(Polygon& obj)>& func) {
	auto p = getBlockPtr(); 
	p->faceFunc(id, func);
} 

void Vertex::cellFunc(const Index3DId& id, const std::function<void(const Polyhedron& obj)>& func) const {
	const auto p = getBlockPtr(); 
	p->cellFunc(id, func);
} 

void Vertex::cellFunc(const Index3DId& id, const std::function<void(Polyhedron& obj)>& func) {
	auto p = getBlockPtr(); 
	p->cellFunc(id, func);
} 

void Vertex::edgeFunc(const EdgeKey& key, const std::function<void(const Edge& obj)>& func) const {
	const auto p = getBlockPtr(); 
	p->edgeFunc(key, func);
} 

void Vertex::edgeFunc(const EdgeKey& key, const std::function<void(Edge& obj)>& func) {
	auto p = getBlockPtr(); 
	p->edgeFunc(key, func);
}
