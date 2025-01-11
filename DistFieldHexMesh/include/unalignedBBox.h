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

#include <defines.h>
#include <vector>
#include <tm_vector3.h>
#include <tm_boundingBox.h>

namespace DFHM 
{
template<class T>
class UnalignedBBox
{
public:
	UnalignedBBox();
	UnalignedBBox(const std::vector<Vector3<T>>& corners);
	UnalignedBBox(const Vector3<T> corners[8]);
	UnalignedBBox(const UnalignedBBox& src);
	UnalignedBBox(const CBoundingBox3D<T>& src);

	bool contains(const Vector3<T>& pt, int64_t numSubDivisions, bool storeInAdjCell[3]) const;
	CBoundingBox3D<T> getBBox() const;

	const Vector3<T>& operator[](size_t i) const;
	Vector3<T>& operator[](size_t i);
	UnalignedBBox& operator = (const UnalignedBBox& rhs);
	UnalignedBBox& operator = (const std::vector<Vector3<T>>& rhs);

	operator const std::vector<Vector3<T>>& () const;
	operator std::vector<Vector3<T>>& ();

	void getFacePoints(CubeFaceType ft, Vector3<T> pts[4]) const;

private:
	double distFromFace(const Vector3<T>& pt, CubeFaceType ft) const;

	std::vector<Vector3<T>> _corners;
};

using UnalignedBBoxd = UnalignedBBox<double>;
using UnalignedBBoxf = UnalignedBBox<float>;

template<class T>
inline UnalignedBBox<T>::UnalignedBBox()
{
	_corners.resize(8);
	for (size_t i = 0; i < 8; i++) {
		_corners[i] = Vector3<T>(0, 0, 0);
	}
}

template<class T>
inline UnalignedBBox<T>::UnalignedBBox(const std::vector<Vector3<T>>& corners)
{
	assert(corners.size() == 8);
	_corners.resize(8);
	for (size_t i = 0; i < 8; i++)
		_corners[i] = corners[i];
}

template<class T>
void UnalignedBBox<T>::getFacePoints(CubeFaceType ft, Vector3<T> pts[4]) const
{
	switch (ft) {
	case CFT_BOTTOM:
		pts[0] = _corners[0];
		pts[1] = _corners[3];
		pts[2] = _corners[2];
		pts[3] = _corners[1];
		break;
	case CFT_TOP:
		pts[0] = _corners[4];
		pts[1] = _corners[5];
		pts[2] = _corners[6];
		pts[3] = _corners[7];
		break;
	case CFT_BACK:
		pts[0] = _corners[0];
		pts[1] = _corners[4];
		pts[2] = _corners[7];
		pts[3] = _corners[3];
		break;
	case CFT_FRONT:
		pts[0] = _corners[1];
		pts[1] = _corners[2];
		pts[2] = _corners[6];
		pts[3] = _corners[5];
		break;
	case CFT_LEFT:
		pts[0] = _corners[0];
		pts[1] = _corners[1];
		pts[2] = _corners[5];
		pts[3] = _corners[4];
		break;
	case CFT_RIGHT:
		pts[0] = _corners[3];
		pts[1] = _corners[7];
		pts[2] = _corners[6];
		pts[3] = _corners[2];
		break;
	}
}

template<class T>
inline UnalignedBBox<T>::UnalignedBBox(const Vector3<T> corners[8])
{
	assert(corners.size() == 8);
	_corners.resize(8);
	for (int i = 0; i < 8; i++)
		_corners[i] = corners[i];
}

template<class T>
inline UnalignedBBox<T>::UnalignedBBox(const UnalignedBBox& src)
{
	_corners.resize(8);
	for (int i = 0; i < 8; i++)
		_corners[i] = src._corners[i];
}

template<class T>
inline UnalignedBBox<T>::UnalignedBBox(const CBoundingBox3D<T>& src)
{
	_corners.resize(8);
	for (size_t i = 0; i < _corners.size(); i++)
		_corners[i] = src._corners[i];
}

template<class T>
double UnalignedBBox<T>::distFromFace(const Vector3<T>& pt, CubeFaceType ft) const
{
	Vector3<T> v, v0, v1, norm, norm1;
	Vector3<T> pts[4];
	getFacePoints(ft, pts);
	Vector3<T> ctr(0, 0, 0);
	for (int i = 0; i < 4; i++)
		ctr += pts[i];
	ctr /= 4;

	v = pt - ctr;

	v0 = pts[0] - pts[1];
	v1 = pts[2] - pts[1];
	norm = v1.cross(v0);

	v0 = pts[2] - pts[3];
	v1 = pts[0] - pts[3];
	norm1 = v1.cross(v0);

	norm = norm + norm1;
	norm.normalize();
	return v.dot(norm);
}

template<class T>
bool UnalignedBBox<T>::contains(const Vector3<T>& pt, int64_t numSubDivisions, bool storeInAdjBlock[3]) const
{
	Vector3<T> uvw;
	if (!TRI_LERP_INV(pt, _corners, uvw))
		return false;

	for (int i = 0; i < 3; i++) {
		int64_t n = (int64_t)(uvw[i] * numSubDivisions);
		if (n < 0 || n > numSubDivisions)
			return false;
		storeInAdjBlock[i] = n == numSubDivisions;
	}

	return true;
}

template<class T>
inline CBoundingBox3D<T> UnalignedBBox<T>::getBBox() const
{
	CBoundingBox3D<T> bbox;
	for (int i = 0; i < 8; i++)
		bbox.merge(_corners[i]);

	return bbox;
}

template<class T>
inline const Vector3<T>& UnalignedBBox<T>::operator[](size_t i) const
{
	return _corners[i];
}

template<class T>
inline Vector3<T>& UnalignedBBox<T>::operator[](size_t i)
{
	return _corners[i];
}

template<class T>
inline UnalignedBBox<T>& UnalignedBBox<T>::operator = (const UnalignedBBox& rhs)
{
	assert(rhs.size() == 8);
	_corners = rhs._corners;
	return *this;
}

template<class T>
inline UnalignedBBox<T>& UnalignedBBox<T>::operator = (const std::vector<Vector3<T>>& rhs)
{
	assert(rhs.size() == 8);
	_corners = rhs;
	return *this;
}

template<class T>
inline UnalignedBBox<T>::operator const std::vector<Vector3<T>>& () const
{
	return _corners;
}

template<class T>
inline UnalignedBBox<T>::operator std::vector<Vector3<T>>& ()
{
	return _corners;
}

}