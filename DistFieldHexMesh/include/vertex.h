#pragma once

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

#include <tm_vector3.h>
#include <tm_boundingBox.h>
#include <index3D.h>
#include <objectPool.h>
#include <fastBisectionSet.h>

#define USE_FIXED_PT 0

namespace DFHM {

class Block;
class Edge;

class Vertex : public ObjectPoolOwnerUser {
public:
	static CBoundingBox3Dd calBBox(const Vector3d& pt);
	static Vector3<int64_t> scaleToSearch(const Vector3d& pt);

	Vertex() = default;
	Vertex(const Vertex& src);
	Vertex(const Vector3d& pt);
	Vertex& operator = (const Vertex& rhs);

#if 0
	void setLockType(LockType val, size_t idx);
	LockType getLockType(size_t& idx) const;
#endif

	const Vector3d& getPoint() const;
	operator const Vector3d& () const;
	CBoundingBox3Dd getBBox() const;
	void setLockType(VertexLockType val);
	VertexLockType getLockType() const;

	const bool operator < (const Vertex& rhs) const;
	const bool operator > (const Vertex& rhs) const;
	const bool operator == (const Vertex& rhs) const;
	const bool operator != (const Vertex& rhs) const;

	Index3DId getId() const override;
	void remapId(const std::vector<size_t>& idRemap, const Index3D& srcDims) override;

	void addConnectedVertexId(const Index3DId& vertId);
	void removeConnectedVertexId(const Index3DId& vertId);
	const FastBisectionSet<Index3DId>& getConnectedVertexIds() const;
	MTC::set<Edge> getEdges() const;
	MTC::set<Index3DId> getFaceIds() const;
	MTC::set<Index3DId> getCellIds() const;

	void write(std::ostream& out) const;
	void read(std::istream& in);

protected:
	void setId(const Index3DId& id) override;

private:
	static int64_t scaleToSearch();
	static int64_t scaleToSearch(double v);
	VertexLockType _lockType = VertexLockType::VLT_NONE;

	Vector3d _pt;
	Index3DId _thisId;
	FastBisectionSet<Index3DId> _connectedVertices;
};

inline Index3DId Vertex::getId() const
{
	return _thisId;
}

inline CBoundingBox3Dd Vertex::getBBox() const
{
	return calBBox(_pt);
}

inline Vertex::Vertex(const Vector3d& pt)
	: _pt(pt)
{
}

inline const Vector3d& Vertex::getPoint() const
{
	return _pt;
}

inline Vertex::operator const Vector3d& () const
{
	return _pt;
}

inline void Vertex::setLockType(VertexLockType val)
{
	_lockType = val;
}

inline VertexLockType Vertex::getLockType() const
{
	return _lockType;
}

inline void Vertex::addConnectedVertexId(const Index3DId& vertId)
{
	_connectedVertices.insert(vertId);
}

inline void Vertex::removeConnectedVertexId(const Index3DId& vertId)
{
	_connectedVertices.erase(vertId);
}

inline const FastBisectionSet<Index3DId>& Vertex::getConnectedVertexIds() const
{
	return _connectedVertices;
}

std::ostream& operator << (std::ostream& out, const Vertex& vert);

}

