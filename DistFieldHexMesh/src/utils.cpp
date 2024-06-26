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

using namespace std;
using namespace DFHM;

void removeVertFromAvail(map<Index3DId, set<Edge>>& vertToEdgeMap, const Index3DId& vertId)
{
	auto iter = vertToEdgeMap.find(vertId);
	vertToEdgeMap.erase(iter);
}

void removeEdgeFromAvail(map<Index3DId, set<Edge>>& vertToEdgeMap, const Edge& edge)
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
	vector<Index3DId> _verts;
};

using VertLoopPtr = shared_ptr<VertLoop>;

bool VertLoopCompare(const VertLoopPtr& pLhs, const VertLoopPtr& pRhs) {
	return pLhs->_length < pRhs->_length;
}

}

void Utils::formEdgeLoops(const Block* pBlock, const set<Edge> sharedSegments, set<Edge> availEdges, vector<vector<Edge>>& loops)
{
	for (const auto& edge : sharedSegments) {
		availEdges.insert(edge);
	}

	map<Index3DId, set<Edge>> vertToEdgeMap;
	for (const auto& edge : availEdges) {
		const auto vertIds = edge.getVertexIds();

		auto iter = vertToEdgeMap.find(vertIds[0]);
		if (iter == vertToEdgeMap.end()) {
			iter = vertToEdgeMap.insert(make_pair(vertIds[0], set<Edge>())).first;
		}
		iter->second.insert(edge);

		iter = vertToEdgeMap.find(vertIds[1]);
		if (iter == vertToEdgeMap.end()) {
			iter = vertToEdgeMap.insert(make_pair(vertIds[1], set<Edge>())).first;
		}
		iter->second.insert(edge);
	}

	vector<VertLoopPtr> vertLoops;
	VertLoopPtr pVertLoop = make_shared<VertLoop>(pBlock);

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
				for (const auto& edge : edgeSet) {
					const auto otherVertId = edge.getOtherVert(lastVertId);
					if (otherVertId != prevVertId) {
						VertLoopPtr pNewLoop = make_shared<VertLoop>(*pVertLoop);
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
