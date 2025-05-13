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
#include <memory>

#include <tm_vector3.h>
#include <tm_spatialSearch.h>
#include <index3D.h>

namespace DFHM {

	class PolyMeshIndex {
	public:
		PolyMeshIndex(size_t meshIdx = -1, const Index3DId& polyId = {});
		PolyMeshIndex(const PolyMeshIndex& src) = default;

		size_t getMeshIdx() const;
		const Index3DId& getPolyId() const;

		bool operator < (const PolyMeshIndex& rhs) const;
		bool operator == (const PolyMeshIndex& rhs) const;

	private:
		size_t _meshIdx = -1;
		Index3DId _polyId;
	};

	inline PolyMeshIndex::PolyMeshIndex(size_t meshIdx, const Index3DId& polyId)
		: _meshIdx(meshIdx)
		, _polyId(polyId)
	{
	}

	inline size_t PolyMeshIndex::getMeshIdx() const
	{
		return _meshIdx;
	}

	inline const Index3DId& PolyMeshIndex::getPolyId() const
	{
		return _polyId;
	}

	inline bool PolyMeshIndex::operator < (const PolyMeshIndex& rhs) const
	{
		if (_meshIdx < rhs._meshIdx)
			return true;
		else if (_meshIdx > rhs._meshIdx)
			return false;

		return _polyId < rhs._polyId;
	}

	inline bool PolyMeshIndex::operator == (const PolyMeshIndex& rhs) const
	{
		return _meshIdx == rhs._meshIdx && _polyId == rhs._polyId;
	}

	struct MultiPolyMeshRayHit : public PolyMeshIndex {
	public:
		MultiPolyMeshRayHit() = default;
		MultiPolyMeshRayHit(size_t meshIdx, const Index3DId& hitID, const Vector3d& pt, double dist);
		MultiPolyMeshRayHit(const MultiPolyMeshRayHit& src) = default;
		MultiPolyMeshRayHit(const PolyMeshIndex& idx = {});

		void setPoint(const Vector3d& pt);
		const Vector3d& getPoint() const;

		bool operator<(const MultiPolyMeshRayHit& rhs) const;

		double getDist() const;
		const Vector3d& getPt() const;

	private:
		double _dist;
		Vector3d _pt;
	};

	inline MultiPolyMeshRayHit::MultiPolyMeshRayHit(const PolyMeshIndex& idx)
		: PolyMeshIndex(idx)
	{
	}

	inline MultiPolyMeshRayHit::MultiPolyMeshRayHit(size_t meshIdx, const Index3DId& hitID, const Vector3d& pt, double dist)
		: PolyMeshIndex(meshIdx, hitID)
		, _pt(pt)
		, _dist(dist)
	{
	}

	inline void MultiPolyMeshRayHit::setPoint(const Vector3d& pt)
	{
		_pt = pt;
	}

	inline const Vector3d& MultiPolyMeshRayHit::getPoint() const
	{
		return _pt;
	}

	inline bool MultiPolyMeshRayHit::operator<(const MultiPolyMeshRayHit& rhs) const
	{
		return _dist < rhs._dist;
	}

	inline double MultiPolyMeshRayHit::getDist() const
	{
		return _dist;
	}

	inline const Vector3d& MultiPolyMeshRayHit::getPt() const
	{
		return _pt;
	}


}
