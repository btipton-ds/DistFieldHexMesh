#include <vertex.h>
#include <vertexManager.h>

using namespace std;
using namespace DFHM;

int VertexManager::fromDbl(double val)
{
	return (int)((val / 1000) * INT_MAX + 0.5);
}

double VertexManager::toDbl(int val)
{
	return (val / (double)INT_MAX) * 1000;
}

void VertexManager::setNumThreads(size_t val)
{
	Base::setNumThreads(val);
	_threadLocalData.resize(val);
}

ObjectPoolId VertexManager::getId(const Vector3d& pt, size_t threadNum) const
{
	Vector3<int> iPt(fromDbl(pt[0]), fromDbl(pt[1]), fromDbl(pt[2]));

	auto& localMap = _threadLocalData[threadNum];
	auto iter0 = localMap.find(iPt[0]);
	if (iter0 != localMap.end()) {
		auto map1 = iter0->second;

		auto iter1 = map1.find(iPt[1]);
		if (iter1 != map1.end()) {
			auto map2 = iter1->second;

			auto iter2 = map2.find(iPt[2]);
			if (iter2 != map2.end()) {
				return iter2->second;
			}
		}
	}

	return ObjectPoolId();
}

ObjectPoolId VertexManager::add(const Vertex& obj, const ObjectPoolId& id)
{

	ObjectPoolId result = Base::add(obj, id);

	auto pt = obj.getPoint();

	Vector3<int> iPt(fromDbl(pt[0]), fromDbl(pt[1]), fromDbl(pt[2]));

	auto& localMap = _threadLocalData[id.getThreadIndex()];

	auto iter0 = localMap.find(iPt[0]);
	if (iter0 == localMap.end()) {
		iter0 = localMap.insert(make_pair(iPt[0], intMap2())).first;
	}
	auto map1 = iter0->second;

	auto iter1 = map1.find(iPt[1]);
	if (iter1 == map1.end()) {
		iter1 = map1.insert(make_pair(iPt[1], intMap1())).first;
	}
	auto map2 = iter1->second;

	auto iter2 = map2.find(iPt[2]);
	if (iter2 == map2.end()) {
		map2.insert(make_pair(iPt[2], result)).first;
	}

	return result;
}
