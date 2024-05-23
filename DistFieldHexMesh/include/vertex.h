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

#include <set>
#include <tm_vector3.h>
#include <index3D.h>
#include <patient_lock_guard.h>
#include <objectPool.h>
#include <fixedPoint.h>

#define USE_FIXED_PT 0

namespace DFHM {

class Block;
class Edge;

namespace Tolerance
{
	inline double sameDistTol()
	{
#if USE_FIXED_PT
		return 1.0e-5;
#else
#if 0
		return 1.0e-8;
#else
		return 2 * FixedPt::getFixedScale() / FIXED_PT_INT_MAX;
#endif
#endif
	}
	inline double paramTol() {
#if USE_FIXED_PT
		return 1.0e-6;
#else
		return 1.0e-12;
#endif
	}
	inline double looseParamTol() {
#if USE_FIXED_PT
		return 1.0e-4;
#else
		return 1.0e-6;
#endif
	}

	inline double angleTol() {
#if USE_FIXED_PT
		return 1.0e-6;
#else
		return 1.0e-10;
#endif
	}

}


class Vertex : public ObjectPoolOwnerUser {
public:
#if 0
	enum class LockType {
		None,
		Triangle,
		Edge,
		Vertex
	};
#endif
	// Required for use with object pool


	Vertex() = default;
	Vertex(const Vertex& src);
	Vertex(const Vector3d& pt);
	Vertex& operator = (const Vertex& rhs);

#if 0
	void setLockType(LockType val, size_t idx);
	LockType getLockType(size_t& idx) const;
#endif

	void setPoint(const Vector3d& pt);
	Vector3d getPoint() const;
	operator Vector3d () const;
	const FixedPt& getFixedPt() const;
	void setLockType(VertexLockType val);
	VertexLockType getLockType() const;

	const bool operator < (const Vertex& rhs) const;

	void write(std::ostream& out) const;
	void read(std::istream& in);

private:

	VertexLockType _lockType = VertexLockType::VLT_NONE;

	/*
	NOTE - In a single threaded architecture, it saves time to keep edges and faceIds stored with the vertex.
	In this multi-threaded architecture, it leads to deadlocks, more mutexes and slows things down.
	Do NOT add any reference links on the vertex unless the architecture changes.
	*/
#if USE_FIXED_PT
	FixedPt _pt; // Fixed point representation of a double precisions point
#else
	Vector3d _pt;
	FixedPt _searchPt;
#endif
};

inline Vertex::Vertex(const Vector3d& pt)
{
	setPoint(pt);
}

#if 0
inline void Vertex::setLockType(LockType val, size_t idx)
{
	_lockType = val;
	_lockIdx = idx;
}

inline Vertex::LockType Vertex::getLockType(size_t& idx) const
{
	idx = _lockIdx;
	return _lockType;
}
#endif

inline void Vertex::setPoint(const Vector3d& pt)
{
#if USE_FIXED_PT
	_pt = FixedPt::fromDbl(pt);
#else
	_pt = pt;
	_searchPt = FixedPt::fromDbl(_pt);
#endif
}

inline Vector3d Vertex::getPoint() const
{
#if USE_FIXED_PT
	return FixedPt::toDbl(_pt);
#else
	return _pt;
#endif
}

inline const FixedPt& Vertex::getFixedPt() const
{
	return _searchPt;
}

inline Vertex::operator Vector3d () const
{
	return getPoint();
}

inline void Vertex::setLockType(VertexLockType val)
{
	_lockType = val;
}

inline VertexLockType Vertex::getLockType() const
{
	return _lockType;
}

std::ostream& operator << (std::ostream& out, const Vertex& vert);

}

