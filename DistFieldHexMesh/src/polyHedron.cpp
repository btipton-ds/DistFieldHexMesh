#include <vector>
#include <polyhedron.h>
#include <vertex.h>

using namespace std;
using namespace DFHM;

bool Polyhedron::unload(std::ostream& out)
{
	return true;
}

bool Polyhedron::load(std::istream& out)
{
	return true;
}

size_t Polyhedron::getHash() const
{
	size_t result = 0;
	for (const auto& val : _polygonIds)
		result += val.getThreadIndex() + val.getIndex();

	return result;
}

bool Polyhedron::operator < (const Polyhedron& rhs) const
{
	if (_polygonIds.size() < rhs._polygonIds.size())
		return true;
	else if (_polygonIds.size() > rhs._polygonIds.size())
		return false;

	vector<ObjectPoolId> lhsIndices(_polygonIds), rhsIndices(rhs._polygonIds);
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
