#pragma once

#include <iostream>
#include <Index3DFull.h>

namespace DFHM {

class Polyhedron {
public:
	Polyhedron() = default;
	Polyhedron(const Polyhedron& src) = default;
	Polyhedron(const std::vector<Index3DIdFull>& faceIds);

	const std::vector<Index3DIdFull>& getFaceIds() const;
	bool unload(std::ostream& out);
	bool load(std::istream& out);

	bool operator < (const Polyhedron& rhs) const;

private:
	std::vector<Index3DIdFull> _faceIds;
};

inline const std::vector<Index3DIdFull>& Polyhedron::getFaceIds() const
{
	return _faceIds;
}

}

