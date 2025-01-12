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
#include <enums.h>

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
	bool contains(const Vector3<T>& pt, size_t numDivisions, bool inNextBlock[3]) const;
	CBoundingBox3D<T> getBBox() const;

	const Vector3<T>& operator[](size_t i) const;
	Vector3<T>& operator[](size_t i);
	UnalignedBBox& operator = (const UnalignedBBox& rhs);
	UnalignedBBox& operator = (const std::vector<Vector3<T>>& rhs);

	operator const std::vector<Vector3<T>>& () const;
	operator std::vector<Vector3<T>>& ();

	void getFacePoints(CubeFaceType ft, Vector3<T> pts[4]) const;

private:
	T distFromFace(const Vector3<T>& pt, CubeFaceType ft) const;

	// need to use planes for contains
	std::vector<Vector3<T>> _corners;
};

using UnalignedBBoxd = UnalignedBBox<double>;
using UnalignedBBoxf = UnalignedBBox<float>;


}