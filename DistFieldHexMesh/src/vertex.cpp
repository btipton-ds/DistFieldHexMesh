#include <vertex.h>
#include <edge.h>
#include <polygon.h>
#include <block.h>

using namespace std;
using namespace DFHM;

const bool FixedPt::operator < (const FixedPt& rhs) const
{
	for (size_t idx = 0; idx < 3; idx++) {
		if ((*this)[idx] < rhs[idx])
			return true;
		else if (rhs[idx] < (*this)[idx])
			return false;
	}

	return false;
}

Vertex::Vertex(const Vertex& src)
	: _lockType(src._lockType)
	, _lockIdx(src._lockIdx)
	, _pt(src._pt)
	, _searchPt(src._searchPt)
{
}

Vertex& Vertex::operator = (const Vertex& rhs)
{
	_lockType = rhs._lockType;
	_pt = rhs._pt;
	_searchPt = rhs._searchPt;

	return *this;
}

const bool Vertex::operator < (const Vertex& rhs) const
{
#if USE_FIXED_PT
	return _pt < rhs._pt;
#else
	return _searchPt < rhs._searchPt;
#endif
}
