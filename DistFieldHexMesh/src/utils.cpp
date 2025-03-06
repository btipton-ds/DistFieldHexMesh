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

#include <utils.h>
#include <block.h>
#ifdef _WIN32
#include <windows.h>
#include <profileapi.h>
#else
#include <cstdint>
#include <time.h>
#endif // _WIN32

using namespace DFHM;

void removeVertFromAvail(MTC::map<Index3DId, MTC::set<EdgeKey>>& vertToEdgeMap, const Index3DId& vertId)
{
	auto iter = vertToEdgeMap.find(vertId);
	vertToEdgeMap.erase(iter);
}

void removeEdgeFromAvail(MTC::map<Index3DId, MTC::set<EdgeKey>>& vertToEdgeMap, const Edge& edge)
{
	removeVertFromAvail(vertToEdgeMap, edge.getVertexIds()[0]);
	removeVertFromAvail(vertToEdgeMap, edge.getVertexIds()[1]);
}

namespace
{
struct VertLoop {
	VertLoop() = default;
	VertLoop(const VertLoop& src) = default;
	VertLoop(const Block* pBlock) 
		: _pBlock(pBlock)
	{
	}

	double add(const Index3DId& vId)
	{
		_verts.push_back(vId);
		if (_verts.size() > 1) {
			Vector3d pt0 = _pBlock->getVertexPoint(_verts[_verts.size() - 1]);
			Vector3d pt1 = _pBlock->getVertexPoint(_verts[_verts.size() - 2]);
			double l = (pt1 - pt0).norm();
			_length += l;
		}
		return _length;
	}

	bool isClosed() const {
		return (_verts.size() > 2) && _verts.front() == _verts.back();
	}

	const Block* _pBlock;
	double _length = 0;
	MTC::vector<Index3DId> _verts;
};

using VertLoopPtr = std::shared_ptr<VertLoop>;

bool VertLoopCompare(const VertLoopPtr& pLhs, const VertLoopPtr& pRhs) {
	return pLhs->_length < pRhs->_length;
}

}

void Utils::formEdgeLoops(const Block* pBlock, const MTC::set<EdgeKey> sharedSegments, MTC::set<EdgeKey> availEdges, MTC::vector<MTC::vector<EdgeKey>>& loops)
{
	for (const auto& edgeKey : sharedSegments) {
		availEdges.insert(edgeKey);
	}

	MTC::map<Index3DId, MTC::set<EdgeKey>> vertToEdgeMap;
	for (const auto& edgeKey : availEdges) {
		const auto vertIds = edgeKey.getVertexIds();

		auto iter = vertToEdgeMap.find(vertIds[0]);
		if (iter == vertToEdgeMap.end()) {
			iter = vertToEdgeMap.insert(make_pair(vertIds[0], MTC::set<EdgeKey>())).first;
		}
		iter->second.insert(edgeKey);

		iter = vertToEdgeMap.find(vertIds[1]);
		if (iter == vertToEdgeMap.end()) {
			iter = vertToEdgeMap.insert(make_pair(vertIds[1], MTC::set<EdgeKey>())).first;
		}
		iter->second.insert(edgeKey);
	}

	MTC::vector<VertLoopPtr> vertLoops;
	VertLoopPtr pVertLoop = std::make_shared<VertLoop>(pBlock);

	auto iter = sharedSegments.begin();
	const auto& startEdge = *iter;
	removeEdgeFromAvail(vertToEdgeMap, startEdge);
	pVertLoop->add(startEdge.getVertexIds()[0]);
	pVertLoop->add(startEdge.getVertexIds()[1]);

	vertLoops.push_back(pVertLoop);
	while (!vertLoops.front()->isClosed()) {
		for (const auto& pCurLoop : vertLoops) {
			const auto& curLoop = pCurLoop->_verts;
			const auto& lastVertId = curLoop[curLoop.size() - 1];
			const auto& prevVertId = curLoop[curLoop.size() - 2];

			Vector3d lastPt = pBlock->getVertexPoint(lastVertId);
			auto vertIter = vertToEdgeMap.find(lastVertId);
			if (vertIter != vertToEdgeMap.end()) {
				auto& edgeSet = vertIter->second;
				for (const auto& edgeKey : edgeSet) {
					const auto otherVertId = edgeKey.getOtherVert(lastVertId);
					if (otherVertId != prevVertId) {
						VertLoopPtr pNewLoop = std::make_shared<VertLoop>(*pVertLoop);
						pNewLoop->add(otherVertId);
					}
				}
				vertToEdgeMap.erase(vertIter);
				break;
			}
		}
		sort(vertLoops.begin(), vertLoops.end(), VertLoopCompare);
	}
}

namespace
{

static std::mutex s_timerMutex;

struct TimerRec
{
	size_t _count = 0;
	double _time = 0;
};

static std::vector<TimerRec> s_times;

}

namespace
{

#ifndef _WIN32
typedef union _LARGE_INTEGER {
  struct {
    uint32_t LowPart;
    uint32_t HighPart;
  } DUMMYSTRUCTNAME;
  struct {
    uint32_t LowPart;
    uint32_t HighPart;
  } u;
  uint64_t QuadPart;
} LARGE_INTEGER;

inline bool QueryPerformanceFrequency(LARGE_INTEGER *value)
{
  value->QuadPart = 1000000000L;
  return true;
}
 
inline bool QueryPerformanceCounter(LARGE_INTEGER *value)
{
  timespec ts;
  // CLOCK_MONOTONIC is available on more systems than CLOCK_THREAD_CPUTIME_ID
  bool result = clock_gettime(CLOCK_MONOTONIC, &ts);
  // assume call succeeds 
  value->QuadPart = ts.tv_nsec;
  if (result = -1) {
    // mimic windows error behavior, 0=error
    result = 0;
    value->QuadPart = 0;
  }
  
  return result;
}
#endif

	static LARGE_INTEGER initFreq()
	{
		LARGE_INTEGER r;
		QueryPerformanceFrequency(&r);
		return r;
	}
	LARGE_INTEGER s_freq = initFreq();
}

void Utils::Timer::dumpAll()
{
	for (size_t i = TT_readVolume; i < TT_lastTag; i++) {
		std::string str;
		switch (i) {
			case TT_analyzeModelMesh:
				str = "analyzeModelMesh"; break;
			case TT_splitAtPointInner:
				str = "splitAtPointInner"; break;
			case TT_needToSplitConditional:
				str = "needToSplitConditional"; break;
			case TT_needToSplitIntersection:
				str = "needToSplitIntersection"; break;
			case TT_divideHexMesh:
				str = "divideHexMesh"; break;
			case TT_UpdateTessellation:
				str = "updateTessellation"; break;
			case TT_readVolume:
				str = "readVolume"; break;
		}

		if (s_times[i]._count > 0)
			std::cout << str << ": " << s_times[i]._time << "s, f: " << (s_times[i]._count / s_times[i]._time) << " call/s\n";
		else if (s_times[i]._time > 0)
			std::cout << str << ": " << s_times[i]._time << "s\n";
	}
}

Utils::Timer::Timer(size_t key)
	:_key(key)
{
	LARGE_INTEGER sc;
	QueryPerformanceCounter(&sc);
	_startCount = sc.QuadPart;
}

Utils::Timer::~Timer()
{
	recordEntry();
}

void Utils::Timer::recordEntry()
{
	if (_recorded)
		return;

	_recorded = true;
	LARGE_INTEGER endCount;
	QueryPerformanceCounter(&endCount);
	double deltaT = (endCount.QuadPart - _startCount) / (double)(s_freq.QuadPart);

	{
		std::scoped_lock sl(s_timerMutex);
		if (s_times.empty())
			s_times.resize(TT_lastTag);
		s_times[_key]._count++;
		s_times[_key]._time += deltaT;
	}
}
