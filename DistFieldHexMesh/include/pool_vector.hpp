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

#include <pool_allocator.h>
#include <pool_vector.h>

#define TEMPL_DECL template<class T> 
#define ITER_TEMPL_DECL template <int IterType>
#define VECTOR_DECL PoolUtils::vector<T> 
#define ITER_DECL PoolUtils::vector<T>::_iterator<IterType>

TEMPL_DECL
VECTOR_DECL::vector(::PoolUtils::localHeap& heap)
	: _pHeap(&heap)
{
}

TEMPL_DECL
VECTOR_DECL::vector(const std::vector<T>& src, ::PoolUtils::localHeap& heap)
	: _pHeap(&heap)
{
	insert(end(), src.begin(), src.end());
}

TEMPL_DECL
VECTOR_DECL::vector(const std::initializer_list<T>& src, ::PoolUtils::localHeap& heap)
	: _pHeap(&heap)
{
	insert(end(), src.begin(), src.end());
}

TEMPL_DECL
VECTOR_DECL::operator std::vector<T>() const
{
	std::vector<T> result;
	result.insert(result.end(), begin(), end());
	return result;
}

TEMPL_DECL
void VECTOR_DECL::clear()
{
	_data.clear();
}

TEMPL_DECL
bool VECTOR_DECL::empty()
{
	return _data.empty();
}


TEMPL_DECL
size_t VECTOR_DECL::size() const
{
	return _data.size();
}

TEMPL_DECL
void VECTOR_DECL::resize(size_t val)
{
	_data.resize(val);
}

TEMPL_DECL
void VECTOR_DECL::reserve(size_t val)
{
	_data.reserve(val);
}

TEMPL_DECL
template<class ITER_TYPE>
void VECTOR_DECL::insert(const iterator& at, const ITER_TYPE& begin, const ITER_TYPE& end)
{
	_data.insert(_data.begin() + at._index, begin, end);
}

TEMPL_DECL
void VECTOR_DECL::insert(const iterator& at, const T& val)
{
	_data.insert(_data.begin() + at._index, val);
}

TEMPL_DECL
void VECTOR_DECL::insert(const iterator& at, const std::initializer_list<T>& vals)
{
	_data.insert(_data.begin() + at._index, vals.begin(), vals.end());
}

TEMPL_DECL
VECTOR_DECL::iterator VECTOR_DECL::erase(const iterator& at)
{
	size_t idx = at._index;
	_data.erase(_data.begin() + idx);
	return iterator(this, std::max(size(), idx));
}

TEMPL_DECL
VECTOR_DECL::iterator VECTOR_DECL::erase(const iterator& begin, const iterator& end)
{
	size_t idx = begin._index;
	_data.erase(begin, end);
	return iterator(this, std::max(size(), idx));
}

TEMPL_DECL
PoolUtils::vector<T>& VECTOR_DECL::operator = (const vector& rhs)
{
	insert(end(), rhs.begin(), rhs.end());
	return *this;
}

TEMPL_DECL
_NODISCARD _CONSTEXPR20 typename VECTOR_DECL::const_iterator VECTOR_DECL::begin() const noexcept
{
	return const_iterator(this, 0);
}

TEMPL_DECL
_NODISCARD _CONSTEXPR20 typename VECTOR_DECL::iterator VECTOR_DECL::begin() noexcept
{
	return iterator(this, 0);
}

TEMPL_DECL
_NODISCARD _CONSTEXPR20 typename VECTOR_DECL::const_iterator VECTOR_DECL::end() const noexcept
{
	return const_iterator(this, size());
}

TEMPL_DECL
_NODISCARD _CONSTEXPR20 typename VECTOR_DECL::iterator VECTOR_DECL::end() noexcept
{
	return iterator(this, size());
}

TEMPL_DECL
_NODISCARD _CONSTEXPR20 typename VECTOR_DECL::const_reverse_iterator VECTOR_DECL::rbegin() const noexcept
{
	return const_reverse_iterator(this, size() - 1);
}

TEMPL_DECL
_NODISCARD _CONSTEXPR20 typename VECTOR_DECL::reverse_iterator VECTOR_DECL::rbegin() noexcept
{
	return reverse_iterator(this, size() - 1);
}

TEMPL_DECL
_NODISCARD _CONSTEXPR20 typename VECTOR_DECL::const_reverse_iterator VECTOR_DECL::rend() const noexcept
{
	return const_reverse_iterator(this, -1);
}

TEMPL_DECL
_NODISCARD _CONSTEXPR20 typename VECTOR_DECL::reverse_iterator VECTOR_DECL::rend() noexcept
{
	return reverse_iterator(this, -1);
}

TEMPL_DECL
const T& VECTOR_DECL::front() const
{
	return _data.front();
}

TEMPL_DECL
T& VECTOR_DECL::front()
{
	return _data.front();
}

TEMPL_DECL
const T& VECTOR_DECL::back() const
{
	return _data.back();
}

TEMPL_DECL
T& VECTOR_DECL::back()
{
	return _data.back();
}

TEMPL_DECL
const T& VECTOR_DECL::operator[](size_t idx) const
{
	return _data[idx];
}

TEMPL_DECL
T& VECTOR_DECL::operator[](size_t idx)
{
	return _data[idx];
}

TEMPL_DECL
size_t VECTOR_DECL::push_back(const T& val)
{
	size_t result = size();
	_data.push_back(val);
	return result;
}

TEMPL_DECL
void VECTOR_DECL::pop_back()
{
	_data.pop_back();
}

TEMPL_DECL
ITER_TEMPL_DECL
ITER_DECL::_iterator(const PoolUtils::vector<T>* pSource, size_t index)
	: _pSource(const_cast<PoolUtils::vector<T>*>(pSource))
	, _index(index)
{

}

TEMPL_DECL
ITER_TEMPL_DECL
bool ITER_DECL::operator == (const _iterator& rhs) const
{
	assert(_pSource == _pSource);
	return _index == rhs._index;
}

TEMPL_DECL
ITER_TEMPL_DECL
bool ITER_DECL::operator != (const _iterator& rhs) const
{
	assert(_pSource == _pSource);
	return _index != rhs._index;
}

TEMPL_DECL
ITER_TEMPL_DECL
bool ITER_DECL::operator < (const _iterator& rhs) const
{
	assert(_pSource == _pSource);
	return _index < rhs._index;
}

TEMPL_DECL
ITER_TEMPL_DECL
bool ITER_DECL::operator > (const _iterator& rhs) const
{
	assert(_pSource == _pSource);
	return _index > rhs._index;
}

TEMPL_DECL
ITER_TEMPL_DECL
ITER_DECL& ITER_DECL::operator ++ ()
{
	if (IterType == FORW_CONST || IterType == FORW)
		_index++;
	else
		_index--;
	return *this;
}

TEMPL_DECL
ITER_TEMPL_DECL
ITER_DECL& ITER_DECL::operator ++ (int)
{
	if (IterType == FORW_CONST || IterType == FORW)
		_index++;
	else
		_index--;
	return *this;
}

TEMPL_DECL
ITER_TEMPL_DECL
ITER_DECL& ITER_DECL::operator --()
{
	if (IterType == FORW_CONST || IterType == FORW)
		_index--;
	else
		_index++;
	return *this;
}

TEMPL_DECL
ITER_TEMPL_DECL
ITER_DECL& ITER_DECL::operator --(int)
{
	if (IterType == FORW_CONST || IterType == FORW)
		_index--;
	else
		_index++;
	return *this;
}

TEMPL_DECL
ITER_TEMPL_DECL
ITER_DECL ITER_DECL::operator + (size_t rhs) const
{
	_iterator result(*this);
	if (IterType == FORW_CONST || IterType == FORW)
		result._index += rhs;
	else
		result._index -= rhs;
	return result;
}

TEMPL_DECL
ITER_TEMPL_DECL
size_t ITER_DECL::operator + (const _iterator& rhs) const
{
	assert(_pSource == rhs._pSource);

	if (IterType == FORW_CONST || IterType == FORW)
		return _index + rhs._index;
	else
		return _index - rhs._index;
}

TEMPL_DECL
ITER_TEMPL_DECL
ITER_DECL ITER_DECL::operator - (size_t rhs) const
{
	_iterator result(*this);
	if (IterType == FORW_CONST || IterType == FORW)
		result._index -= rhs;
	else
		result._index += rhs;
	return result;
}

TEMPL_DECL
ITER_TEMPL_DECL
size_t ITER_DECL::operator - (const _iterator& rhs) const
{
	assert(_pSource == rhs._pSource);
	if (IterType == FORW_CONST || IterType == FORW)
		return _index - rhs._index;
	else
		return _index + rhs._index;
}

TEMPL_DECL
ITER_TEMPL_DECL
T& ITER_DECL::operator *() const
{
	return _pSource->operator[](_index);
}

TEMPL_DECL
ITER_TEMPL_DECL
T* ITER_DECL::operator->() const
{
	return &(_pSource->operator[](_index));
}
