#pragma once

#include <map>
#include <objectPool.h>
#include <vertexManager.h>

namespace DFHM {

class Cell;
class Block;
class Polygon;
class Polyhedron;
class Volume;

class DataPool
{
public:
	static void setNumThreads(size_t numThreads);

protected:
	static VertexManager _vertexPool;
	static ObjectPool<Polygon> _polygonPool;
	static ObjectPool<Polyhedron> _polyhedronPool;
	static ObjectPool<Cell> _cellPool;
	static ObjectPool<Block> _blockPool;
};

}