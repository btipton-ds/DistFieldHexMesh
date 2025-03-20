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
#include <Index3D.h>

namespace DFHM {

class Block;

class PolygonSearchKey {
public:
	static MTC::vector<Index3DId> makeNonColinearVertexIds(const Block* pBlock, const MTC::vector<Index3DId>& vertexIds);

	PolygonSearchKey() = default;
	PolygonSearchKey(const PolygonSearchKey& src) = default;
	PolygonSearchKey(const Block* pBlock, const MTC::vector<Index3DId>& ids);

	void set(const Block* pBlock, const MTC::vector<Index3DId>& ids) const;
	bool empty() const;
	bool operator < (const PolygonSearchKey& rhs) const;
	bool operator == (const PolygonSearchKey& rhs) const;
	void clear() const;

private:
	mutable MTC::vector<Index3DId> _ids;
};

inline PolygonSearchKey::PolygonSearchKey(const Block* pBlock, const MTC::vector<Index3DId>& ids)
{
	set(pBlock, ids);
}

inline bool PolygonSearchKey::empty() const
{
	return _ids.empty();
}

inline void PolygonSearchKey::clear() const {
	_ids.clear();
}

inline bool PolygonSearchKey::operator == (const PolygonSearchKey& rhs) const
{
	return !operator < (rhs) && !rhs.operator<(*this);
}

}
