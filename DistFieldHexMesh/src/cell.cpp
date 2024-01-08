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

		}
		_polyhedra.clear();
	}

	return true;
}

bool Cell::load(istream& in)
{
	in.read((char*)&volType, sizeof(volType));


	return true;
}
