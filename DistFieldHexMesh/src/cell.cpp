#include <iostream>
#include <cell.h>
#include <polygon.h>
#include <polyhedron.h>
#include <vertex.h>

using namespace std;
using namespace DFHM;

bool Cell::unload(ostream& out)
{
	out.write((char*)&volType, sizeof(volType));

	{
		// This writes out persistent ids
		size_t numPolygons = _polygons.size();

		// Write out the polygon ids
		out.write((char*)&numPolygons, sizeof(numPolygons));
		for (size_t i = 0; i < numPolygons; i++) {
			ObjectPoolId id = _polygons[i];

			out.write((char*)&id, sizeof(id));
			auto& pPolygon = _polygonPool[id];
			if (!pPolygon.unload(out, id))
				return false;
		}
		_polygons.clear();
	}

	{
		// This writes out persistent ids
		size_t numPolyhedra = _polyhedra.size();

		// Write out the polygon ids
		out.write((char*)&numPolyhedra, sizeof(numPolyhedra));
		for (size_t i = 0; i < numPolyhedra; i++) {
			ObjectPoolId id = _polyhedra[i];

			out.write((char*)&id, sizeof(id));
			auto& pPolyhedron = _polyhedronPool[id];
			if (!pPolyhedron.unload(out))
				return false;
		}
		_polyhedra.clear();
	}

	return true;
}

bool Cell::load(istream& in)
{
	in.read((char*)&volType, sizeof(volType));

	{
		// This writes out persistent ids
		size_t numPolygons = _polygons.size();

		// Write out the polygon ids
		in.read((char*)&numPolygons, sizeof(numPolygons));
		_polygons.resize(numPolygons);
		for (size_t i = 0; i < numPolygons; i++) {
			size_t id;

			in.read((char*)&id, sizeof(id));
			_polygons[i] = id;
			auto& polygon = _polygonPool[id];
			if (!polygon.load(in, id))
				return false;
		}
	}

	{
		// This writes out persistent ids
		size_t numPolyhedra = _polyhedra.size();

		// Write out the polygon ids
		in.read((char*)&numPolyhedra, sizeof(numPolyhedra));
		_polyhedra.resize(numPolyhedra);
		for (size_t i = 0; i < numPolyhedra; i++) {
			size_t id;

			in.read((char*)&id, sizeof(id));
			_polyhedra[i] = id;
			auto& polyhedron = _polyhedronPool[id];
			if (!polyhedron.load(in))
				return false;
		}
	}

	return true;
}
