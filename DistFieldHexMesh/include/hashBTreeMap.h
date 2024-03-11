#pragma once

/*
This file is part of the DistFieldHexMesh application/library.

	The DistFieldHexMesh application/library is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	The DistFieldHexMesh application/library is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	This link provides the exact terms of the GPL license <https://www.gnu.org/licenses/>.

	The author's interpretation of GPL 3 is that if you receive money for the use or distribution of the DistFieldHexMesh application/library or a derivative product, GPL 3 no longer applies.

	Under those circumstances, the author expects and may legally pursue a reasoble share of the income. To avoid the complexity of agreements and negotiation, the author makes
	no specific demands in this regard. Compensation of roughly 1% of net or $5 per user license seems appropriate, but is not legally binding.

	In lay terms, if you make a profit by using the DistFieldHexMesh application/library (violating the spirit of Open Source Software), I expect a reasonable share for my efforts.

	Robert R Tipton - Author

	Dark Sky Innovative Solutions http://darkskyinnovation.com/
*/

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
