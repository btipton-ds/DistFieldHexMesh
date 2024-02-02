#pragma once

#include <stdexcept>
#include <vertex.h>
#include <chrono>
#include <mutex>

#include <index3D.h>
#include <patient_lock_guard.h>
#include <objectPool.h>
#include <vertex.h>
#include <polygon.h>
#include <polyhedron.h>

namespace DFHM {

class Block;

/*
	The multicore decomposition works on a single block at a time.
	The race conditions occur when two adjacent blocks are working on faces or vertices which lie in the common side of the block.
	Each block has a recursive_mutex.
	Lock the mutex whenever a block is being accessed or manipulated. Most hits will be within the block and the lock/unlock will have
	near zero overhead. When an operation on a block requires another block both blocks need to be locked.

	On this scheme, there is a single mutex for each paired face. There are extra, unused mutexes on the other positive faces.
*/

class BlockData : public ObjectPoolOwner {
public:
	BlockData(const Index3D& blockIdx);
	BlockData(const BlockData& src) = delete;
	MutexType& getMutex() const;

	size_t numFaces(bool includeInner) const;
	size_t numPolyhedra() const;

	bool vertexExists(size_t id) const;
	Vector3d getVertexPoint(size_t vertId) const;

	size_t findVertexId(const Vertex& obj) const {
		patient_lock_guard g(_mutex);
		return _vertices.findId(obj);
	}
	Index3DId findOrAddVertex(const Vertex& obj, size_t id = -1)
	{
		patient_lock_guard g(_mutex);
		return _vertices.findOrAdd(this, obj, id);
	}

	bool polygonExists(size_t id) const;
	void addFaceToLookup(size_t faceId);
	bool removeFaceFromLookUp(size_t faceId);

	Index3DId findOrAddFace(const Polygon& obj, size_t id = -1)
	{
		patient_lock_guard g(_mutex);
		return _polygons.findOrAdd(this, obj, id);
	}

	bool polyhedronExists(size_t id) const;
	Index3DId findOrAddCell(const Polyhedron& obj, size_t id = -1)
	{
		patient_lock_guard g(_mutex);
		return _polyhedra.findOrAdd(this, obj, id);
	}

	template<class LAMBDA>
	void vertexFunc(size_t vertId, LAMBDA func) const;

	template<class LAMBDA>
	void vertexFunc(size_t vertId, LAMBDA func);

	template<class LAMBDA>
	void allVertexFunc(LAMBDA func) const;

	template<class LAMBDA>
	void allVertexFunc(LAMBDA func);

	template<class LAMBDA>
	void faceFunc(size_t faceId, LAMBDA func) const;

	template<class LAMBDA>
	void faceFunc(size_t faceId, LAMBDA func);

	template<class LAMBDA>
	void allFaceFunc(LAMBDA func) const;

	template<class LAMBDA>
	void allFaceFunc(LAMBDA func);

	template<class LAMBDA>
	void cellFunc(size_t cellId, LAMBDA func) const;

	template<class LAMBDA>
	void cellFunc(size_t cellId, LAMBDA func);

	template<class LAMBDA>
	void allCellFunc(LAMBDA func) const;

	template<class LAMBDA>
	void allCellFunc(LAMBDA func);

protected:
	Index3D _blockIdx;
private:
	mutable MutexType _mutex;
	ObjectPool<Vertex> _vertices;
	ObjectPool<Polygon> _polygons;
	ObjectPool<Polyhedron> _polyhedra;
};

inline MutexType& BlockData::getMutex() const
{
	return _mutex;
}

template<class LAMBDA>
void BlockData::vertexFunc(size_t vertId, LAMBDA func) const
{
	patient_lock_guard g(_mutex);
	// TODO change all these lambdas to take an Index3DId instead of the Block*.
	func((const Block*)this, _vertices[vertId]);
}

template<class LAMBDA>
void BlockData::vertexFunc(size_t vertId, LAMBDA func)
{
	patient_lock_guard g(_mutex);
	func((Block*)this, _vertices[vertId]);
}

template<class LAMBDA>
void BlockData::allVertexFunc(LAMBDA func)
{
	patient_lock_guard g(_mutex);
	_vertices.iterateInOrder(func);
}

template<class LAMBDA>
void BlockData::allVertexFunc(LAMBDA func) const
{
	patient_lock_guard g(_mutex);
	_vertices.iterateInOrder(func);
}

template<class LAMBDA>
void BlockData::faceFunc(size_t faceId, LAMBDA func) const
{
	patient_lock_guard g(_mutex);
	func((const Block*)this, _polygons[faceId]);
}

template<class LAMBDA>
void BlockData::faceFunc(size_t faceId, LAMBDA func)
{
	patient_lock_guard g(_mutex);
	func((Block*)this, _polygons[faceId]);
}

template<class LAMBDA>
void BlockData::allFaceFunc(LAMBDA func) const
{
	patient_lock_guard g(_mutex);
	_polygons.iterateInOrder(func);
}

template<class LAMBDA>
void BlockData::allFaceFunc(LAMBDA func)
{
	patient_lock_guard g(_mutex);
	_polygons.iterateInOrder(func);
}

template<class LAMBDA>
void BlockData::cellFunc(size_t cellId, LAMBDA func) const
{
	patient_lock_guard g(_mutex);
	func((const Block*)this, _polyhedra[cellId]);
}

template<class LAMBDA>
void BlockData::cellFunc(size_t cellId, LAMBDA func)
{
	patient_lock_guard g(_mutex);
	func((Block*)this, _polyhedra[cellId]);
}

template<class LAMBDA>
void BlockData::allCellFunc(LAMBDA func) const
{
	patient_lock_guard g(_mutex);
	_polyhedra.iterateInOrder(func);
}

template<class LAMBDA>
void BlockData::allCellFunc(LAMBDA func)
{
	patient_lock_guard g(_mutex);
	_polyhedra.iterateInOrder(func);
}

}