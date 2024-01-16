#pragma once

#include <iostream>
#include <index3D.h>

namespace DFHM {

class Polyhedron {
public:
	Polyhedron() = default;
	Polyhedron(const Polyhedron& src) = default;
	Polyhedron(const std::vector<Index3DId>& faceIds);

	const std::vector<Index3DId>& getFaceIds() const;
	bool unload(std::ostream& out);
	bool load(std::istream& out);

	bool operator < (const Polyhedron& rhs) const;

private:
	std::vector<Index3DId> _faceIds;
};

inline const std::vector<Index3DId>& Polyhedron::getFaceIds() const
{
	return _faceIds;
}

}

