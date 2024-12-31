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

	bool contains(const Vector3<T>& pt) const;
	CBoundingBox3D<T> getBBox() const;

	const Vector3<T>& operator[](size_t i) const;
	Vector3<T>& operator[](size_t i);
	UnalignedBBox& operator = (const UnalignedBBox& rhs);
	UnalignedBBox& operator = (const std::vector<Vector3<T>>& rhs);

	operator const std::vector<Vector3<T>>& () const;
	operator std::vector<Vector3<T>>& ();

private:
	bool isOnPosSide(const Vector3<T>& pt, size_t idx0, size_t idx1, size_t idx2, size_t idx3) const;

	std::vector<Vector3<T>> _corners;
};

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
	_corners->resize(8);
	for (size_t i = 0; i < 8; i++)
		_corners[i] = corners[i];
}

template<class T>
inline UnalignedBBox<T>::UnalignedBBox(const Vector3<T> corners[8])
{
	assert(corners.size() == 8);
	_corners->resize(8);
	for (int i = 0; i < 8; i++)
		_corners[i] = corners[i];
}

template<class T>
inline UnalignedBBox<T>::UnalignedBBox(const UnalignedBBox& src)
{
	_corners->resize(8);
	for (int i = 0; i < 8; i++)
		_corners[i] = src._corners[i];
}

template<class T>
inline UnalignedBBox<T>::UnalignedBBox(const CBoundingBox3D<T>& src)
{
	_corners->resize(8);
	const auto& min = src.getMin();
	const auto& max = src.getMax();
	_corners[0] = min;
	_corners[1] = Vector3<T>;
}

template<class T>
bool UnalignedBBox<T>::isOnPosSide(const Vector3<T>& pt, size_t idx0, size_t idx1, size_t idx2, size_t idx3) const
{
	const auto& pt0 = _corners[idx0];
	const auto& pt1 = _corners[idx1];
	const auto& pt2 = _corners[idx2];
	const auto& pt3 = _corners[idx3];

	Vector3<T> v, v0, v1, norm, norm1;

	v = pt - pt0;

	v0 = pt0 - pt1;
	v1 = pt2 - pt1;
	norm = v1.cross(v0).normalized();

	v0 = pt2 - pt3;
	v1 = pt0 - pt3;
	norm1 = v1.cross(v0).normalized();

	norm = norm + norm1;
	norm.normalize();
	if (v.dot(norm) > 0)
		return true;

	return false;
}

template<class T>
bool UnalignedBBox<T>::contains(const Vector3<T>& pt) const
{
	if (isOnPosSide(pt, 4, 5, 6, 7)) // top
		return false;

	if (isOnPosSide(pt, 0, 3, 2, 1)) // bottom
		return false;

	if (isOnPosSide(pt, 0, 4, 7, 3)) // back
		return false;

	if (isOnPosSide(pt, 1, 2, 6, 5)) // front
		return false;

	if (isOnPosSide(pt, 0, 1, 5, 4)) // right
		return false;

	if (isOnPosSide(pt, 2, 3, 7, 6)) // left
		return false;

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