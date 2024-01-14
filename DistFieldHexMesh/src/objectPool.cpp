
#include <objectPool.h>
#include <vertex.h>
#include <polygon.h>
#include <polyhedron.h>
#include <subBlock.h>
#include <block.h>

using namespace DFHM;

thread_local const Vertex* ObjectPool<Vertex>::_tl_pCompareObj = nullptr;
thread_local const Polygon* ObjectPool<Polygon>::_tl_pCompareObj = nullptr;
thread_local const Polyhedron* ObjectPool<Polyhedron>::_tl_pCompareObj = nullptr;
thread_local const SubBlock* ObjectPool<SubBlock>::_tl_pCompareObj = nullptr;
thread_local const Block* ObjectPool<Block>::_tl_pCompareObj = nullptr;
