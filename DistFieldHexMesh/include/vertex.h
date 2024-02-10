#pragma once

#include <set>
#include <tm_vector3.h>
#include <index3D.h>
#include <patient_lock_guard.h>
#include <objectPool.h>

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
		return 1.0e-8;
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

class FixedPt : public Vector3<int>
{
public:
	static int fromDbl(double val);
	static FixedPt fromDbl(const Vector3d& src);
	static double toDbl(int iVal);
	static Vector3d toDbl(const FixedPt& src);
	static double getFixedScale();

	FixedPt() = default;
	FixedPt(const FixedPt& src) = default;
	FixedPt(const Vector3d& pt);

	const bool operator < (const FixedPt& rhs) const;

};

class Vertex : public ObjectPoolOwnerUser {
public:
	enum class LockType {
		None,
		Triangle,
		Edge,
		Vertex
	};

	// Required for use with object pool


	Vertex() = default;
	Vertex(const Vertex& src);
	Vertex(const Vector3d& pt);
	Vertex& operator = (const Vertex& rhs);

	void setLockType(LockType val, size_t idx);
	LockType getLockType(size_t& idx) const;

	void setPoint(const Vector3d& pt);
	Vector3d getPoint() const;
	operator Vector3d () const;
#if USE_FIXED_PT
	const FixedPt& getFixedPt() const;
#endif

	const bool operator < (const Vertex& rhs) const;

private:

	LockType _lockType = LockType::None;
	size_t _lockIdx = -1;

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

inline FixedPt::FixedPt(const Vector3d& pt)
	: Vector3<int>(fromDbl(pt[0]), fromDbl(pt[1]), fromDbl(pt[2]))
{
}

inline double FixedPt::getFixedScale()
{
	return 1000.0; // +/- 25 m volume
}

inline int FixedPt::fromDbl(double val)
{
	double r = val / getFixedScale();
	assert(fabs(r) < 1.0);
	return (int)(r * INT_MAX);
}

inline FixedPt FixedPt::fromDbl(const Vector3d& src)
{
	return FixedPt(src);
}

inline double FixedPt::toDbl(int iVal)
{
	return (iVal / (double)INT_MAX) * getFixedScale();
}

inline Vector3d FixedPt::toDbl(const FixedPt& src)
{
	return Vector3d(toDbl(src[0]), toDbl(src[1]), toDbl(src[2]));
}

inline Vertex::Vertex(const Vector3d& pt)
{
	setPoint(pt);
}

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

#if USE_FIXED_PT
inline const FixedPt& Vertex::getFixedPt() const
{
	return _pt;
}
#endif

inline Vertex::operator Vector3d () const
{
	return getPoint();
}

}

