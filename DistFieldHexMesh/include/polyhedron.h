#pragma once

#include <iostream>
#include <Index3DFull.h>

namespace DFHM {

class Polyhedron {
public:
	Polyhedron() = default;
	Polyhedron(const Polyhedron& src) = default;
	Polyhedron(const std::vector<Index3DFull>& faceIds);

	const std::vector<Index3DFull>& getFaceIds() const;
	bool unload(std::ostream& out);
	bool load(std::istream& out);

	bool operator < (const Polyhedron& rhs) const;

private:
	std::vector<Index3DFull> _faceIds;
};

inline const std::vector<Index3DFull>& Polyhedron::getFaceIds() const
{
	return _faceIds;
}

}

