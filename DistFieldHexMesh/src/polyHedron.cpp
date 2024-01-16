#include <vector>
#include <polyhedron.h>
#include <vertex.h>

using namespace std;
using namespace DFHM;

Polyhedron::Polyhedron(const vector<Index3DIdFull>& faceIds)
	: _faceIds(faceIds)
{
}

bool Polyhedron::unload(std::ostream& out)
{
	return true;
}

bool Polyhedron::load(std::istream& out)
{
	return true;
}

bool Polyhedron::operator < (const Polyhedron& rhs) const
{
	if (_faceIds.size() < rhs._faceIds.size())
		return true;
	else if (_faceIds.size() > rhs._faceIds.size())
		return false;

	vector<Index3DIdFull> lhsIndices(_faceIds), rhsIndices(rhs._faceIds);
	sort(lhsIndices.begin(), lhsIndices.begin());
	sort(rhsIndices.begin(), rhsIndices.begin());
	for (size_t i = 0; i < lhsIndices.size(); i++) {
		if (lhsIndices[i] < rhsIndices[i])
			return true;
		else if (rhsIndices[i] < lhsIndices[i])
			return false;
	}

	return false;

}
