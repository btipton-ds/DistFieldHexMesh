
#include <objectPool.h>
#include <vertex.h>
#include <edge.h>
#include <polygon.h>
#include <polyhedron.h>
#include <subBlock.h>

using namespace DFHM;

#define DECL_THREAD_LOCAL(CLASS_NAME) thread_local const CLASS_NAME* ObjectPool<CLASS_NAME>::_tl_pCompareObj = nullptr

DECL_THREAD_LOCAL(Vertex);
DECL_THREAD_LOCAL(Edge);
DECL_THREAD_LOCAL(Polygon);
DECL_THREAD_LOCAL(Polyhedron);
DECL_THREAD_LOCAL(SubBlock);
