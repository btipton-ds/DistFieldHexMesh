#include <volume.h> // Includes all the geometry types
#include <cell.h>
#include <vertex.h>

using namespace std;
using namespace DFHM;

ObjectPool<Vertex> DataPool::_vertexPool(true);
ObjectPool<Polygon> DataPool::_polygonPool(true);
ObjectPool<Polyhedron> DataPool::_polyhedra(true);
ObjectPool<Cell> DataPool::_cells(false);
ObjectPool<Block> DataPool::_blockPool(false);

void DataPool::setNumThreads(size_t numThreads)
{
#if 0
	_vertexPool.setNumThreads(numThreads);
	_polygonPool.setNumThreads(numThreads);
	_polyhedra.setNumThreads(numThreads);
	_cells.setNumThreads(numThreads);
	_blockPool.setNumThreads(numThreads);
#endif
}

