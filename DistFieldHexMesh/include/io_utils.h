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

#include <defines.h>
#include <iostream>
#include <tm_ioUtil.h>
#include <pool_vector.h>
#include <pool_set.h>
#include <pool_map.h>
#include <fastBisectionSet.h>

namespace IoUtil
{
	template<class T>
	void write(std::ostream& out, const MultiCore::vector<T>& vals)
	{
		size_t num = vals.size();
		out.write((char*)&num, sizeof(num));
		if (num > 0) {
			out.write((char*)vals.data(), num * sizeof(T));
		}
	}

	template<class T>
	void read(std::istream& in, MultiCore::vector<T>& vals)
	{
		size_t num;
		in.read((char*)&num, sizeof(num));
		if (num > 0) {
			vals.resize(num);
			in.read((char*)vals.data(), num * sizeof(T));
		}
	}

	template<class T>
	void write(std::ostream& out, const MultiCore::set<T>& vals)
	{
		size_t num = vals.size();
		out.write((char*)&num, sizeof(num));
		for (auto& val : vals)
			out.write((char*)&val, sizeof(val));
	}

	template<class T>
	void read(std::istream& in, MultiCore::set<T>& vals)
	{
		vals.clear();
		size_t num;
		in.read((char*)&num, sizeof(num));
		for (size_t i = 0; i < num; i++) {
			T val;
			in.read((char*)&val, sizeof(val));
			vals.insert(val);
		}
	}

	template<class T>
	void write(std::ostream& out, const DFHM::FastBisectionSet<T>& vals)
	{
		size_t num = vals.size();
		out.write((char*)&num, sizeof(num));
		for (auto& val : vals)
			out.write((char*)&val, sizeof(val));
	}

	template<class T>
	void read(std::istream& in, DFHM::FastBisectionSet<T>& vals)
	{
		vals.clear();
		size_t num;
		in.read((char*)&num, sizeof(num));
		for (size_t i = 0; i < num; i++) {
			T val;
			in.read((char*)&val, sizeof(val));
			vals.insert(val);
		}
	}

	template<class T>
	void writeObj(std::ostream& out, const DFHM::FastBisectionSet<T>& vals)
	{
		size_t num = vals.size();
		out.write((char*)&num, sizeof(num));
		for (auto& val : vals)
			val.write(out);
	}

	template<class T>
	void readObj(std::istream& in, DFHM::FastBisectionSet<T>& vals)
	{
		vals.clear();
		size_t num;
		in.read((char*)&num, sizeof(num));
		for (size_t i = 0; i < num; i++) {
			T val;
			val.read(in);
			vals.insert(val);
		}
	}

	template<class T, class U>
	void write(std::ostream& out, const MultiCore::map<T, U>& val)
	{
		size_t num = val.size();
		out.write((char*)&num, sizeof(num));
		for (const auto& pair : val) {
			pair.first.write(out);
			pair.second.write(out);

		}
	}

	template<class T, class U>
	void read(std::istream& in, MultiCore::map<T, U>& val)
	{
		size_t num = val.size();
		in.read((char*)&num, sizeof(num));
		for (size_t i = 0; i < num; i++) {
			T t;
			U u;
			t.read(in);
			u.read(in);
			val.insert(std::make_pair(t, u));
		}
	}

}

