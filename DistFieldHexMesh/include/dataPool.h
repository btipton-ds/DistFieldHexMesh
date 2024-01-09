#pragma once

#include <map>
#include <objectPool.h>

namespace DFHM {

class Cell;
class Block;
class Polygon;
class Polyhedron;
class Vertex;
class Volume;

class DataPool
{
public:
	static void setNumThreads(size_t numThreads);

protected:
	static ObjectPool<Vertex> _vertexPool;
	static ObjectPool<Polygon> _polygonPool;
	static ObjectPool<Polyhedron> _polyhedronPool;
	static ObjectPool<Cell> _cells;
	static ObjectPool<Block> _blockPool;
};

}