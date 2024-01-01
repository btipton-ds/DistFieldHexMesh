#pragma once

namespace DFHM {

class ObjectPoolId
{
public:
	ObjectPoolId(size_t index = -1, size_t threadIndex = 0);
	ObjectPoolId(const ObjectPoolId& src) = default;

	size_t getThreadIndex() const;
	size_t getIndex() const;
	bool operator < (const ObjectPoolId& rhs) const;
	bool operator == (size_t rhs) const;
	bool operator == (const ObjectPoolId& rhs) const;
	bool operator != (size_t rhs) const;
	bool operator != (const ObjectPoolId& rhs) const;

private:
	size_t _threadIndex = 0;
	size_t _index = -1;
};

inline ObjectPoolId::ObjectPoolId(size_t index, size_t threadIndex)
	: _threadIndex(threadIndex)
	, _index(index)
{
}

inline size_t ObjectPoolId::getThreadIndex() const
{
	return _threadIndex;
}

inline size_t ObjectPoolId::getIndex() const
{
	return _index;
}

inline bool ObjectPoolId::operator < (const ObjectPoolId& rhs) const
{
	if (_threadIndex == rhs._threadIndex)
		return _index < rhs._index;
	return _threadIndex < rhs._threadIndex;
}

inline bool ObjectPoolId::operator == (size_t rhs) const
{
	if (rhs == -1) {
		return _threadIndex == rhs || _index == rhs;
	}
	return operator == (ObjectPoolId(rhs, 0));
}

inline bool ObjectPoolId::operator == (const ObjectPoolId& rhs) const
{
	return (_threadIndex == rhs._threadIndex) && (_index == rhs._index);
}

inline bool ObjectPoolId::operator != (size_t rhs) const
{
	if (rhs == -1) {
		return (_threadIndex != rhs) && (_index != rhs);
	}
	return operator != (ObjectPoolId(rhs, 0));
}

inline bool ObjectPoolId::operator != (const ObjectPoolId& rhs) const
{
	return (_threadIndex != rhs._threadIndex) || (_index != rhs._index);
}

}
