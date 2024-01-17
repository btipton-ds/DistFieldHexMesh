
#include <objectPool.h>
#include <vertex.h>
#include <polygon.h>
#include <polyhedron.h>

using namespace DFHM;

#define DECL_THREAD_LOCAL(CLASS_NAME) thread_local const CLASS_NAME* ObjectPool<CLASS_NAME>::_tl_pCompareObj = nullptr

DECL_THREAD_LOCAL(Vertex);
DECL_THREAD_LOCAL(Polygon);
DECL_THREAD_LOCAL(Polyhedron);
