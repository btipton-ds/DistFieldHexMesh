#include <iostream>
#include <vertex.h>
#include <polygon.h>

using namespace std;
using namespace DFHM;

bool Polygon::unload(std::ostream& out, size_t idSelf)
{
	size_t numVertices = vertexIds.size();
	for (size_t vertId : vertexIds) {
		auto* pVert = _vertexPool.getObj(vertId);
		pVert->removePolygonReference(idSelf);
	}
	out.write((char*)&numVertices, sizeof(numVertices));
	// TODO collect all vertices which will be orphaned and write them out and purge them from storage
	out.write((char*)vertexIds.data(), vertexIds.size());

	return true;
}

bool Polygon::load(std::istream& in, size_t idSelf)
{
	size_t numVertices;

	in.read((char*)&numVertices, sizeof(numVertices));
	vertexIds.resize(numVertices);

	// TODO collect all cached orphan vertices and read them in

	in.read((char*)vertexIds.data(), vertexIds.size());
	for (size_t i = 0; i < vertexIds.size(); i++) {
		Vertex* pVert;
		_vertexPool.getObj(vertexIds[i], pVert, true);
	// TODO collect all vertices which will be orphaned and write them out
		pVert->addPolygonReference(idSelf);
	}

	return true;
}
