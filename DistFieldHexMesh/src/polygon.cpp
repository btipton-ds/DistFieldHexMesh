#include <iostream>
#include <vertex.h>
#include <polygon.h>

using namespace std;
using namespace DFHM;

bool Polygon::unload(std::ostream& out, const ObjectPoolId& idSelf)
{
	size_t numVertices = vertexIds.size();
	for (const auto& vertId : vertexIds) {
		auto& vert = _vertexPool[vertId];
		vert.removePolygonReference(idSelf);
	}
	out.write((char*)&numVertices, sizeof(numVertices));
	// TODO collect all vertices which will be orphaned and write them out and purge them from storage
	out.write((char*)vertexIds.data(), vertexIds.size());

	return true;
}

bool Polygon::load(std::istream& in, const ObjectPoolId& idSelf)
{
	size_t numVertices;

	in.read((char*)&numVertices, sizeof(numVertices));
	vertexIds.resize(numVertices);

	// TODO collect all cached orphan vertices and read them in

	in.read((char*)vertexIds.data(), vertexIds.size());
	for (size_t i = 0; i < vertexIds.size(); i++) {
		Vertex& vert = _vertexPool[vertexIds[i]];
	// TODO collect all vertices which will be orphaned and write them out
		vert.addPolygonReference(idSelf);
	}

	return true;
}
