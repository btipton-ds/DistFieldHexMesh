#include <index3D.h>

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
