#pragma once

#include <tm_vector3.h>
#include <UniversalIndex3D.h>

namespace DFHM {

class Vertex {
public:
	using FixedPt = Vector3<int>;
	enum class LockType {
		None,
		Triangle,
		Edge,
		Vertex
	};

	static int fromDbl(double val);
	static FixedPt fromDbl(const Vector3d& src);
	static double toDbl(int iVal);
	static Vector3d toDbl(const FixedPt& src);
	static double getFixedScale();

	Vertex() = default;
	Vertex(const Vertex& src) = default;
	Vertex(const Vector3d& pt);

	void setLockType(LockType val, size_t idx);
	LockType getLockType(size_t& idx) const;

	void setPoint(const Vector3d& pt);
	Vector3d getPoint() const;
	operator Vector3d () const;
	const FixedPt& getFixedPt() const;

	void addPolygonReference(const UniversalIndex3D& polygonId);
	void removePolygonReference(const UniversalIndex3D& polygonId);

	const size_t getHash() const;
	const bool operator < (const Vertex& rhs) const;

private:

	LockType _lockType = LockType::None;
	size_t _lockIdx = -1;

	FixedPt _pt; // Fixed point representation of a double precisions point
	std::vector<UniversalIndex3D> _polygonIds;
};

inline double Vertex::getFixedScale()
{
	return 1000.0;
}

inline int Vertex::fromDbl(double val)
{
	double r = val / getFixedScale();
	assert(fabs(r) < 1.0);
	return (int)(r * INT_MAX);
}

inline Vertex::FixedPt Vertex::fromDbl(const Vector3d& src)
{
	return FixedPt(fromDbl(src[0]), fromDbl(src[1]), fromDbl(src[2]));
}

inline double Vertex::toDbl(int iVal)
{
	return (iVal / (double)INT_MAX) * getFixedScale();
}

inline Vector3d Vertex::toDbl(const FixedPt& src)
{
	return Vector3d(toDbl(src[0]), toDbl(src[1]), toDbl(src[2]));
}

inline Vertex::Vertex(const Vector3d& pt)
{
	setPoint(pt);
}

inline const size_t Vertex::getHash() const
{
	return _pt[0] + _pt[1] + _pt[2];
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
	FixedPt fPt;

	fPt[0] = fromDbl(pt[0]);
	fPt[1] = fromDbl(pt[1]);
	fPt[2] = fromDbl(pt[2]);

	_pt = fPt;
}

inline Vector3d Vertex::getPoint() const
{
	Vector3d pt;

	pt[0] = toDbl(_pt[0]);
	pt[1] = toDbl(_pt[1]);
	pt[2] = toDbl(_pt[2]);

	return pt;
}

inline const Vertex::FixedPt& Vertex::getFixedPt() const
{
	return _pt;
}

inline Vertex::operator Vector3d () const
{
	return getPoint();
}

}

