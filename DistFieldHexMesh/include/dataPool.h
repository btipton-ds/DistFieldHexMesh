#pragma once

#include <objectPool.h>

namespace DFHM {

class Cell;
class Block;
class Polygon;
class Polyhedron;
class Volume;
class Vertex;

class DataPool
{
public:
	static void setNumThreads(size_t numThreads);

protected:
	static ObjectPool<Vertex> _vertexPool;
	static ObjectPool<Polygon> _polygonPool;
	static ObjectPool<Polyhedron> _polyhedronPool;
	static ObjectPool<Cell> _cellPool;
	static ObjectPool<Block> _blockPool;
};

}