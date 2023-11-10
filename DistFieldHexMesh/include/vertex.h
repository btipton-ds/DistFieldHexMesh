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
	const Vector3d& getPoint() const;
	operator const Vector3d& () const;

	void addPolygonReference(const ObjectPoolId& polygonId);
	void removePolygonReference(const ObjectPoolId& polygonId);

private:
	LockType _lockType = LockType::None;
	size_t _lockIdx = -1;

	Vector3d _pt;
	std::vector<ObjectPoolId> _polygonIds;
};

inline Vertex::Vertex(const Vector3d& pt)
	: _pt(pt)
{
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
	_pt = pt;
}

inline const Vector3d& Vertex::getPoint() const
{
	return _pt;
}

inline Vertex::operator const Vector3d& () const
{
	return _pt;
}

}
