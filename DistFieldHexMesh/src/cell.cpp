#include <iostream>
#include <cell.h>
#include <polygon.h>
#include <polyhedron.h>
#include <vertex.h>
#include <volume.h>

using namespace std;
using namespace DFHM;

Cell::Cell()
{
}

void Cell::addHit(const Vector3d& cellOrigin, const Vector3d& cellSpan, AxisIndex axisIdx, const RayTriIntersect& hit)
{
	const double TOL = 1.0e-6;
	Vector3d pt = hit._hitPt;
	Vector3d v = pt - cellOrigin;
	Vector3d uvw;
	for (int i = 0; i < 3; i++) {
		uvw[i] = v[i] / cellSpan[i];
		assert((uvw[i] >= -TOL) && (uvw[i] <= (1 + TOL)));
	}
	switch (axisIdx) {
		case AxisIndex::X: {
			double x = Vector3d(1, 0, 0).dot(uvw);
			int iy = (int)(Vector3d(0, 1, 0).dot(uvw) + 0.5);
			int iz = (int)(Vector3d(0, 0, 1).dot(uvw) + 0.5);
			_hits[0][iy][iz].push_back(TriMeshHitRec(x, hit._triIdx));
			break;
		}
		case AxisIndex::Y: {
			int ix = (int)(Vector3d(1, 0, 0).dot(uvw) + 0.5);
			double y = Vector3d(0, 1, 0).dot(uvw);
			int iz = (int)(Vector3d(0, 0, 1).dot(uvw) + 0.5);

			_hits[1][ix][iz].push_back(TriMeshHitRec(y, hit._triIdx));

			break;
		}
		case AxisIndex::Z: {
			int ix = (int)(Vector3d(1, 0, 0).dot(uvw) + 0.5);
			int iy = (int)(Vector3d(0, 1, 0).dot(uvw) + 0.5);
			double z = Vector3d(0, 0, 1).dot(uvw);

			_hits[2][ix][iy].push_back(TriMeshHitRec(z, hit._triIdx));
			break;
		}
	}
}

namespace
{
	struct  {
		bool operator()(const Cell::TriMeshHitRec& a, const Cell::TriMeshHitRec& b) const {
			return a._t < b._t; 
		}
	} HitComp;
}

void Cell::makeIntersectionFaces(Volume& vol, size_t cellIdx, const Vector3d& cellOrigin, const Vector3d& cellSpan)
{
	vector<Vertex> verts;

	for (int axis = 0; axis < 3; axis++) {
		vector<TriMeshHitRec>(&axisHits)[2][2] = _hits[axis];
		for (int j = 0; j < 2; j++) {
			for (int k = 0; k < 2; k++) {
				auto& hits = axisHits[j][k];
				std::sort(hits.begin(), hits.end(), HitComp);
			}
		}
	}

	for (int axis = 0; axis < 3; axis++) {
		vector<TriMeshHitRec>(&axisHits)[2][2] = _hits[axis];
		for (int j = 0; j < 2; j++) {
			for (int k = 0; k < 2; k++) {
				auto& hits = axisHits[j][k];
				if (!hits.empty()) {
					auto hit = hits.front();
					hits.erase(hits.begin());
					switch (axis) {
					case 0:
						if (hit._t >= 0) {
							Vector3d offset(hit._t * cellSpan[0], j * cellSpan[1], k * cellSpan[2]);
							Vertex vert(cellOrigin + offset, Vertex::LockType::Triangle, hit._triIdx);
							verts.push_back(vert);
						}
						break;
					case 1:
						if (hit._t >= 0) {
							Vector3d offset(j * cellSpan[0], hit._t * cellSpan[1], k * cellSpan[2]);
							Vertex vert(cellOrigin + offset, Vertex::LockType::Triangle, hit._triIdx);
							verts.push_back(vert);
						}
						break;
					case 2:
						if (hit._t >= 0) {
							Vector3d offset(j * cellSpan[0], k * cellSpan[1], hit._t * cellSpan[2]);
							Vertex vert(cellOrigin + offset, Vertex::LockType::Triangle, hit._triIdx);
							verts.push_back(vert);
						}
						break;
					}
				}
			}
		}
	}

	if (verts.size() == 3) {
		cout << "created tri\n";
		Polygon* pPoly = nullptr;
		size_t polygonIdx = _polygonPool.create(pPoly, -1);
		if (pPoly) {
			_polygons.push_back(polygonIdx);
			for (Vertex& vert : verts) {
				vert.addPolygonReference(polygonIdx);
				size_t vertIdx = vol.getVertIdx(vert);
				if (vertIdx == -1) {
					vertIdx = vol.addVert(vert);
				}
				pPoly->_cellIdx[0] = cellIdx;
				pPoly->_cellIdx[1] = cellIdx;
				pPoly->vertexIds.push_back(vertIdx);
			}
		}
	} else if (verts.size() == 4) {
		cout << "created quad\n";
		Polygon* pPoly = nullptr;
		size_t polygonIdx = _polygonPool.create(pPoly, -1);
		if (pPoly) {
		}
	} else if (!verts.empty()) {
		cout << "created a mess\n";
		for (const auto& v : verts) {
			size_t idx;
			auto lt = v.getLockType(idx);
			cout << "polyV: [" << v.getPoint().x() << ", " << v.getPoint().y() << ", " << v.getPoint().z() << "], triIdx: " << idx << "\n";
		}
	}

}

bool Cell::unload(ostream& out)
{
	out.write((char*)&_volType, sizeof(_volType));
	out.write(&_span[0], 3);

	{
		// This writes out persistent ids
		size_t numPolygons = _polygons.size();

		// Write out the polygon ids
		out.write((char*)&numPolygons, sizeof(numPolygons));
		for (size_t i = 0; i < numPolygons; i++) {
			size_t id = _polygons[i];

			out.write((char*)&id, sizeof(id));
			auto* pPolygon = _polygonPool.getObj(id);
			if (!pPolygon->unload(out, id))
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
			size_t id = _polyhedra[i];

			out.write((char*)&id, sizeof(id));
			auto* pPolyhedron = _polyhedronPool.getObj(id);
			if (!pPolyhedron->unload(out))
				return false;
		}
		_polyhedra.clear();
	}

	return true;
}

bool Cell::load(istream& in)
{
	in.read((char*)&_volType, sizeof(_volType));
	in.read(&_span[0], 3);

	{
		// This writes out persistent ids
		size_t numPolygons = _polygons.size();

		// Write out the polygon ids
		in.read((char*)&numPolygons, sizeof(numPolygons));
		_polygons.resize(numPolygons);
		for (size_t i = 0; i < numPolygons; i++) {
			size_t id;

			in.read((char*)&id, sizeof(id));
			Polygon* pPolygon = nullptr;
			_polygons[i] = _polygonPool.getObj(id, pPolygon, true);
			if (!pPolygon->load(in, id))
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
			Polyhedron* pPolyhedron = nullptr;
			_polyhedra[i] = _polyhedronPool.getObj(id, pPolyhedron, true);
			if (!pPolyhedron->load(in))
				return false;
		}
	}

	return true;
}
