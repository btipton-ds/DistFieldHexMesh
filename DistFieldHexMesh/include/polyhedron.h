#pragma once

#include <iostream>
#include <UniversalIndex3D.h>

namespace DFHM {

class Polyhedron {
public:
	Polyhedron() = default;
	Polyhedron(const Polyhedron& src) = default;
	Polyhedron(const std::vector<UniversalIndex3D>& faceIds);

	const std::vector<UniversalIndex3D>& getFaceIds() const;
	bool unload(std::ostream& out);
	bool load(std::istream& out);

	bool operator < (const Polyhedron& rhs) const;

private:
	std::vector<UniversalIndex3D> _faceIds;
};

inline const std::vector<UniversalIndex3D>& Polyhedron::getFaceIds() const
{
	return _faceIds;
}

}

