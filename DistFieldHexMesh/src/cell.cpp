#include <iostream>
#include <cell.h>
#include <polygon.h>
#include <polyhedron.h>
#include <vertex.h>

using namespace std;
using namespace DFHM;

bool Cell::unload(ostream& out)
{
	out.write((char*)&_volType, sizeof(_volType));

	{
		// This writes out persistent ids
		size_t numPolyhedra = _polyhedra.size();

		// Write out the polygon ids
		out.write((char*)&numPolyhedra, sizeof(numPolyhedra));
		for (size_t id : _polyhedra) {
		}
		_polyhedra.clear();
	}

	return true;
}

bool Cell::load(istream& in)
{
	in.read((char*)&_volType, sizeof(_volType));


	return true;
}
