#pragma once


#include <tm_vector3.h>
#include <dataPool.h>

namespace DFHM {

class Vertex : public DataPool {
public:
	enum class LockType {
		None,
		Triangle,
		Edge,
		Vertex
	};

	Vertex() = default;
	Vertex(const Vertex& src) = default;
	Vertex(const Vector3d& pt);

	void setLockType(LockType val, size_t idx);
	LockType getLockType(size_t& idx) const;

	void setPoint(const Vector3d& pt);
	Vector3d getPoint() const;
	operator Vector3d () const;

	void addPolygonReference(const ObjectPoolId& polygonId);
	void removePolygonReference(const ObjectPoolId& polygonId);

	const size_t getHash() const;
	const bool operator < (const Vertex& rhs) const;

private:
	using FixedPt = Vector3<int>;

	static int fromDbl(double val);
	static double toDbl(int iVal);
	static double getFixedScale();

	LockType _lockType = LockType::None;
	size_t _lockIdx = -1;

	FixedPt _pt; // Fixed point representation of a double precisions point
	std::vector<ObjectPoolId> _polygonIds;
};

inline double Vertex::getFixedScale()
{
	return 1000.0;
}

inline int Vertex::fromDbl(double val)
{
	return (int)(val / getFixedScale() * INT_MAX);
}

inline double Vertex::toDbl(int iVal)
{
	return (iVal / (double)INT_MAX) * getFixedScale();

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

inline Vertex::operator Vector3d () const
{
	return getPoint();
}

}

