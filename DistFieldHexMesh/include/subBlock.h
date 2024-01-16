#pragma once

#include <set>
#include <exception>
#include <tm_boundingBox.h>
#include <Index3D.h>

namespace DFHM {

class Block;

struct RayTriHit {
	size_t _subBlockIdx;
	size_t _triIdx;
	Vector3d _hitPt;
	double _segLen = 0;
	double _dist = 0;

	inline bool operator < (const RayTriHit& rhs) {
		return _dist < rhs._dist;
	}
};
using RayTriHitVector = std::vector<RayTriHit>;

struct FaceRayHits {
	inline bool isEmpty() const
	{
		return _data.empty();
	}
	void resize(size_t samples)
	{
		_data.resize(samples);
		for (auto& sub : _data)
			sub.resize(samples);
	}
	size_t numHits() const
	{
		size_t result = 0;
		for (const auto& sd0 : _data) {
			for (const auto& sd2 : sd0) {
				result += sd2.size();
			}
		}
		return result;
	}

	std::vector<std::vector<RayTriHitVector>> _data;
};

class SubBlock {
public:
	enum VolumeType {
		VT_UNKNOWN,
		VT_VOID,
		VT_SOLID,
		VT_FLUID,
	};

	void init(Block* pBlock, const Index3D& subBlockIdx);
	void addPolyhdra(size_t id);
	void removePolyhdra(size_t id);
	size_t numPolyhedra() const;
	const std::set<size_t>& getPolyhedraIds() const;

	void addRayHits(const Vector3d* subBlockCornerPts, size_t blockDim, const Index3D& subBlockIdx, const FaceRayHits rayTriHits[3]);
	size_t maxIntersectionsPerLeg() const;
	void divide();

	bool unload(std::ostream& out);
	bool load(std::istream& in);

	bool operator < (const SubBlock& rhs) const;
private:
	void addRayHit(int axis, size_t i, size_t j, const RayTriHit& hit);

	Block* _pBlock = nullptr;
	Index3D _ourIdx;
	FaceRayHits _faceRayHits[3];
	VolumeType _volType = VT_UNKNOWN;
	std::set<size_t> _polyhedra;// indices of polyedra in this subBlock
};

inline void SubBlock::addPolyhdra(size_t id)
{
	_polyhedra.insert(id);
}

inline void SubBlock::removePolyhdra(size_t id)
{
	_polyhedra.erase(id);
}

inline size_t SubBlock::numPolyhedra() const
{
	return _polyhedra.size();
}

inline const std::set<size_t>& SubBlock::getPolyhedraIds() const
{
	return _polyhedra;
}

inline bool SubBlock::operator < (const SubBlock& rhs) const
{
	throw std::runtime_error("SubBlock::operator < is not implmented");
	return false;
}

}