#include <volume.h> // Includes all the geometry types
#include <cell.h>
#include <vertex.h>

using namespace std;
using namespace DFHM;

VertexManager DataPool::_vertexPool;
ObjectPool<Polygon> DataPool::_polygonPool;
ObjectPool<Polyhedron> DataPool::_polyhedronPool;
ObjectPool<Cell> DataPool::_cellPool;
ObjectPool<Block> DataPool::_blockPool;

void DataPool::setNumThreads(size_t numThreads)
{
	_vertexPool.setNumThreads(numThreads);
	_polygonPool.setNumThreads(numThreads);
	_polyhedronPool.setNumThreads(numThreads);
	_cellPool.setNumThreads(numThreads);
	_blockPool.setNumThreads(numThreads);
}

