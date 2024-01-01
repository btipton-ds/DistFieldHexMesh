#pragma once

#include <map>

namespace DFHM {
template<class OBJ>
class HashBTreeMap {
public:
	size_t find(const OBJ& value) const;

	// Add entry. Returs true if the entity was added, false if it was already in the map
	bool add(const size_t& index, const OBJ& value);
	bool remove(const size_t& value);
	bool remove(const OBJ& value);

	size_t size() const;
	void clear();

private:
	using BMap = std::map<OBJ, size_t>;
	std::map<size_t, BMap> _data;
};

template<class OBJ>
size_t HashBTreeMap<OBJ>::find(const OBJ& value) const
{
	size_t hash = value.getHash();
	auto iter0 = _data.find(hash);
	if (iter0 != _data.end()) {
		auto& map0 = iter0->second;
		auto iter1 = map0.find(value);
		if (iter1 != map0.end()) {
			return  iter1->second;
		}
	}
	size_t result(-1);
	return result;
}

template<class OBJ>
bool HashBTreeMap<OBJ>::add(const size_t& index, const OBJ& value)
{
	size_t hash = value.getHash();
	auto iter0 = _data.find(hash);
	if (iter0 == _data.end()) {
		iter0 = _data.insert(std::make_pair(hash, BMap())).first;
	}
	BMap& map1 = iter0->second;

	auto iter1 = map1.find(value);
	if (iter1 == map1.end()) {
		map1.insert(std::make_pair(value, index));
		return true;
	}
	return false;
}

}
