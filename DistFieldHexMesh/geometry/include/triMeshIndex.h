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

	Copyright Robert R Tipton, 2022, all rights reserved except those granted in prior license statement.

	Dark Sky Innovative Solutions http://darkskyinnovation.com/
*/

#include <defines.h>
#include <memory>

#include <tm_vector3.h>
#include <tm_spatialSearch.h>

namespace DFHM {

	class TriMeshIndex {
	public:
		TriMeshIndex(size_t meshIdx = -1, size_t triIdx = -1);
		TriMeshIndex(const TriMeshIndex& src) = default;

		size_t getMeshIdx() const;
		size_t getTriIdx() const;

		bool operator < (const TriMeshIndex& rhs) const;
		bool operator == (const TriMeshIndex& rhs) const;

	private:
		size_t _meshIdx = -1;
		size_t _triIdx = -1;
	};

	inline TriMeshIndex::TriMeshIndex(size_t meshIdx, size_t triIdx)
		: _meshIdx(meshIdx)
		, _triIdx(triIdx)
	{
	}

	inline size_t TriMeshIndex::getMeshIdx() const
	{
		return _meshIdx;
	}

	inline size_t TriMeshIndex::getTriIdx() const
	{
		return _triIdx;
	}

	inline bool TriMeshIndex::operator < (const TriMeshIndex& rhs) const
	{
		if (_meshIdx < rhs._meshIdx)
			return true;
		else if (_meshIdx > rhs._meshIdx)
			return false;

		return _triIdx < rhs._triIdx;
	}

	inline bool TriMeshIndex::operator == (const TriMeshIndex& rhs) const
	{
		return _meshIdx == rhs._meshIdx && _triIdx == rhs._triIdx;
	}

	struct MultiTriMeshRayHit : public TriMeshIndex {
	public:
		MultiTriMeshRayHit() = default;
		MultiTriMeshRayHit(size_t meshIdx, const RayHitd& hit);
		MultiTriMeshRayHit(const MultiTriMeshRayHit& src) = default;
		MultiTriMeshRayHit(const TriMeshIndex& idx = {});

		void setPoint(const Vector3d& pt);
		const Vector3d& getPoint() const;

		bool operator<(const MultiTriMeshRayHit& rhs) const;

		double getDist() const;
		const Vector3d& getPt() const;

	private:
		double _dist;
		Vector3d _pt;
	};

	inline MultiTriMeshRayHit::MultiTriMeshRayHit(const TriMeshIndex& idx)
		: TriMeshIndex(idx)
	{
	}

	inline MultiTriMeshRayHit::MultiTriMeshRayHit(size_t meshIdx, const RayHitd& hit)
		: TriMeshIndex(meshIdx, hit.triIdx)
		, _pt(hit.hitPt)
		, _dist(hit.dist)
	{
	}

	inline void MultiTriMeshRayHit::setPoint(const Vector3d& pt)
	{
		_pt = pt;
	}

	inline const Vector3d& MultiTriMeshRayHit::getPoint() const
	{
		return _pt;
	}

	inline bool MultiTriMeshRayHit::operator<(const MultiTriMeshRayHit& rhs) const
	{
		return _dist < rhs._dist;
	}

	inline double MultiTriMeshRayHit::getDist() const
	{
		return _dist;
	}

	inline const Vector3d& MultiTriMeshRayHit::getPt() const
	{
		return _pt;
	}


}
