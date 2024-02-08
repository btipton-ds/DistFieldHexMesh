#include <iostream>
#include <index3D.h>

using namespace std;
using namespace DFHM;

Index3DBaseType Index3DBase::s_blockDim = 8;

bool Index3DBase::operator < (const Index3DBase& rhs) const
{
	for (size_t i = 0; i < 3; i++) {
		if ((*this)[i] < rhs[i])
			return true;
		else if ((*this)[i] > rhs[i])
			return false;
	}
	assert((*this) == rhs);
	return false;
}

void Index3DBase::clampInBounds(size_t bound)
{
	for (size_t i = 0; i < 3; i++) {
		if ((*this)[i] >= bound)
			(*this)[i] = (Index3DBaseType) (bound - 1);
	}
}

ostream& DFHM::operator << (ostream& out, const Index3D& rhs)
{
	out << "(" << rhs[0] << ", " << rhs[1] << ", " << rhs[2] << ")\n";
	return out;
}

ostream& DFHM::operator << (ostream& out, const Index3DId& rhs)
{
	const auto& idx = rhs.blockIdx();
	out << "(" << idx[0] << ", " << idx[1] << ", " << idx[2] << "):" << rhs.elementId() << "\n";
	return out;
}
