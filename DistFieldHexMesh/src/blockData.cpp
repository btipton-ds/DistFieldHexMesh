#include <blockData.h>

using namespace std;
using namespace DFHM;

BlockData::BlockData(const Index3D& blockIdx)
	: _blockIdx(blockIdx)
	, _vertices(true)
	, _polygons(true)
	, _polyhedra(false)
{
}

size_t BlockData::numFaces(bool includeInner) const
{
	if (includeInner)
		return _polygons.size();

	size_t result = 0;
	_polygons.iterateInOrder([&result](size_t id, const Polygon& poly) {
		if (poly.isOuter())
			result++;
		});

	return result;
}

size_t BlockData::numPolyhedra() const
{
	return _polyhedra.size();
}

void BlockData::addFaceToLookup(size_t faceId)
{
	patient_lock_guard g(_mutex);
	_polygons.addToLookup(faceId);
}

bool BlockData::removeFaceFromLookUp(size_t faceId)
{
	patient_lock_guard g(_mutex);
	return _polygons.removeFromLookup(faceId);
}

Vector3d BlockData::getVertexPoint(size_t vertId) const
{
	patient_lock_guard g(_mutex);
	return _vertices[vertId].getPoint();
}

bool BlockData::vertexExists(size_t id) const
{
	patient_lock_guard g(_mutex);
	return _vertices.exists(id);
}

bool BlockData::polygonExists(size_t id) const
{
	patient_lock_guard g(_mutex);
	return _polygons.exists(id);
}

bool BlockData::polyhedronExists(size_t id) const
{
	patient_lock_guard g(_mutex);
	return _polyhedra.exists(id);
}

