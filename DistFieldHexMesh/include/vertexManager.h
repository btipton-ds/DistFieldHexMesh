#pragma once

#include <objectPool.h>

namespace DFHM {

class Vertex;

class VertexManager : public ObjectPool<Vertex> {
public:
	using Base = ObjectPool<Vertex>;

	void setNumThreads(size_t val) override;
	ObjectPoolId getId(const Vector3d& pt, size_t threadNum = -1) const;
	ObjectPoolId add(const Vertex& obj, const ObjectPoolId& id = ObjectPoolId()) override;
private:
	static int fromDbl(double val);
	static double toDbl(int val);

	using intMap1 = std::map<int, ObjectPoolId>;
	using intMap2 = std::map<int, intMap1>;
	using intMap3 = std::map<int, intMap2>;

	std::vector<intMap3> _threadLocalData;
};

}