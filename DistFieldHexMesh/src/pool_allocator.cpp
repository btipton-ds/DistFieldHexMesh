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
#include <assert.h>
#include <algorithm>
#include <pool_allocator.h>

::PoolUtils::localHeap::localHeap(size_t blockSizeChunks, size_t chunkSizeBytes)
	: _blockSizeChunks(blockSizeChunks)
	, _chunkSizeBytes(chunkSizeBytes + sizeof(size_t))
{
	_availChunks.reserve(100);
}

void* ::PoolUtils::localHeap::alloc(size_t bytes)
{
	const auto& headerSize = sizeof(BlockHeader);
	size_t bytesNeeded = bytes + headerSize;
	size_t numChunks = bytesNeeded / _chunkSizeBytes;
	if (bytesNeeded % _chunkSizeBytes != 0)
		numChunks++;

	BlockHeader* pHeader = getAvailChunk(numChunks);
	if (pHeader != nullptr) {
		return (char*)pHeader + headerSize;
	}
	assert(numChunks < _blockSizeChunks);

	AvailChunk info;
	if (_topBlockIdx >= _data.size() || (_topChunkIdx + numChunks >= _blockSizeChunks)) {
		// Not enough room in the block, so make an empty one.

		if (_topChunkIdx + numChunks >= _blockSizeChunks) {
			// Store the empty space for the next allocation
			BlockHeader headerForRemainder;
			headerForRemainder._blockIdx = _topBlockIdx;
			headerForRemainder._chunkIdx = (uint32_t) (_topChunkIdx + numChunks);
			headerForRemainder._numChunks = (uint32_t) (_blockSizeChunks - _topChunkIdx - numChunks);
			addAvailChunk(headerForRemainder);
		}

		_topBlockIdx = (uint32_t)_data.size();
		_topChunkIdx = 0;

		info._header._numChunks = (uint32_t)numChunks;
		info._header._blockIdx = _topBlockIdx;
		info._header._chunkIdx = _topChunkIdx;

		_data.push_back(_STD make_shared<_STD vector<char>>(_blockSizeChunks * _chunkSizeBytes + headerSize, 0));
	}

	auto& blkVec = *_data[info._header._blockIdx];
	size_t startIdx = info._header._chunkIdx * _chunkSizeBytes;
	
	pHeader = (BlockHeader*) &blkVec[startIdx];
	(*pHeader) = info._header;

	_topChunkIdx += (uint32_t) numChunks;
	if (_topChunkIdx >= _blockSizeChunks) {
		_topBlockIdx++;
		_topChunkIdx = 0;
	}

	return (char*)pHeader + headerSize;
}

void ::PoolUtils::localHeap::free(void* ptr)
{
	const auto& headerSize = sizeof(BlockHeader);
	char* pc = (char*)ptr - headerSize;
	BlockHeader* pHeader = (BlockHeader*)pc;
	addAvailChunk(*pHeader);
}

::PoolUtils::localHeap::BlockHeader* ::PoolUtils::localHeap::getAvailChunk(size_t numChunksNeeded)
{
	size_t idx = _availChunkInfoIdx;
	while (idx != -1) {
		auto& curChunk = _availChunks[idx];
		if (curChunk._header._numChunks <= numChunksNeeded) {
			size_t startIdxBytes = curChunk._header._chunkIdx * _chunkSizeBytes;
			auto& blkVec = *_data[curChunk._header._blockIdx];
			auto& startAddr = blkVec[startIdxBytes];
			BlockHeader* pHeader = (BlockHeader*)&startAddr;
			(*pHeader) = curChunk._header;

			// Remove this chunk from the sorted linked list of available chunks
			if (curChunk._prevIdx != -1) {
				_availChunks[curChunk._prevIdx]._nextIdx = curChunk._nextIdx;
			} else {
				_availChunkInfoIdx = curChunk._nextIdx;
			}

			if (curChunk._nextIdx != -1) {
				_availChunks[curChunk._nextIdx]._prevIdx = curChunk._prevIdx;
			}
			return pHeader;
		}
		idx = _availChunks[idx]._nextIdx;
	}

	return nullptr;
}

void ::PoolUtils::localHeap::addAvailChunk(const BlockHeader& header)
{
	size_t newIdx;
	if (!_freeAvailChunks.empty()) {
		newIdx = _freeAvailChunks.back();
		_freeAvailChunks.pop_back();
	} else {
		newIdx = _availChunks.size();
		_availChunks.push_back(AvailChunk());
	}

	auto& newEntry = _availChunks[newIdx];
	newEntry._header = header;

	bool found = false;
	size_t idx = _availChunkInfoIdx, lastIdx = -1;
	while (idx != -1) {
		lastIdx = idx;
		auto& curEntry = _availChunks[idx];
		if (newEntry._header._numChunks <= _availChunks[idx]._header._numChunks) {
			found = true;
			// insert here
			if (idx == _availChunkInfoIdx) {
				newEntry._nextIdx = _availChunkInfoIdx;
				_availChunks[_availChunkInfoIdx]._prevIdx = newIdx;
				_availChunkInfoIdx = newIdx;
			} else {
				newEntry._nextIdx = idx;
				newEntry._prevIdx = curEntry._prevIdx;
				if (newEntry._nextIdx != -1) {
					_availChunks[newEntry._nextIdx]._prevIdx = newIdx;
				}
				if (newEntry._prevIdx != -1) {
					_availChunks[newEntry._prevIdx]._nextIdx = newIdx;
				}
			}
		}

		idx = curEntry._nextIdx;
	}
	if (!found) {
		if (lastIdx == -1 || _availChunkInfoIdx == -1) {
			_availChunkInfoIdx = newIdx;
		} else if (lastIdx != -1) {
			_availChunks[lastIdx]._nextIdx = newIdx;
			newEntry._prevIdx = lastIdx;
		}
	}
}
