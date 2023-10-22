#include <vertex.h>
#include <volume.h>

using namespace DFHM;

#define RUN_MULTI_THREAD true

ObjectPool<Vertex> DataPool::_vertexPool;
ObjectPool<Polygon> DataPool::_polygonPool;
ObjectPool<Polyhedron> DataPool::_polyhedronPool;
ObjectPool<Block> DataPool::_blockPool;
