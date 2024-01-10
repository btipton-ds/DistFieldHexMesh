#pragma once

#include <iostream>

namespace DFHM {

class Polyhedron {
public:
	Polyhedron() = default;
	Polyhedron(const Polyhedron& src) = default;
	Polyhedron(const std::vector<size_t>& faceIds);

	const std::vector<size_t>& getFaceIds() const;
	bool unload(std::ostream& out);
	bool load(std::istream& out);

	bool operator < (const Polyhedron& rhs) const;

private:
	std::vector<size_t> _faceIds;
};

inline const std::vector<size_t>& Polyhedron::getFaceIds() const
{
	return _faceIds;
}

}

