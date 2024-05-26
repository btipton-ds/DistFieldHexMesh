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

#include <memory>
#include <vector>
#include <list>

namespace PoolUtils
{

/*
	This class's purpose is to create a heap manager which is local to an object and/or thread.
	Since DFHM's threading is 1-1 on blocks, if the block owns the localHeap all memory allocations within that block
	are within the thread local heap - no mutex conflicts on new/delete.

	It is only possible to REDUCE access to the application heap, not eliminate it totally.

	Justification - during early testing, single threaded was faster than multi threaded and the process was only getting 15% of available CPU.
	In other testing, the app got access to 100% of available CPU.
*/

class localHeap {
public:
	localHeap(size_t blockSizeChunks, size_t chunkSizeBytes = 32);
	
	void* alloc(size_t bytes);
	void free(void* ptr);

private:	
	struct BlockHeader {
		uint32_t _blockIdx;
		uint32_t _chunkIdx;
		uint32_t _numChunks;
	};

	struct AvailChunk {
		size_t _nextIdx = -1;
		size_t _prevIdx = -1;

		BlockHeader _header;
	};

	BlockHeader* getAvailChunk(size_t numChunksNeeded);
	void addAvailChunk(const BlockHeader& header);

	const size_t _blockSizeChunks;
	const size_t _chunkSizeBytes;

	uint32_t _topBlockIdx = 0;
	uint32_t _topChunkIdx = 0;

	using BlockPtr = std::shared_ptr<_STD vector<char>>;
	_STD vector<BlockPtr> _data;

	_STD vector<AvailChunk> _availChunks;
	_STD vector<size_t> _freeAvailChunks;

	size_t _availChunkInfoIdx = -1; // Sorted indices into _availChunks
};

}
