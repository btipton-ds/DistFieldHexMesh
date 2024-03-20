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

#include <logger.h>
#include <ios>

using namespace std;
using namespace DFHM;

std::shared_ptr<Logger> Logger::get(const std::string& streamName)
{
	static map<std::string, shared_ptr<Logger>> s_map;
	auto iter = s_map.find(streamName);
	if (iter == s_map.end()) {
		shared_ptr<Logger> p = make_shared<Logger>("D:/DarkSky/Projects/output/logs/" + streamName);
		iter = s_map.insert(make_pair(streamName, p)).first;
	}
	return iter->second;
}

Logger::Logger(const string& logPath)
	: _stream(logPath, ios::out)
{
}

ostream& Logger::getStream()
{
	return _stream;
}

Padding& Logger::getPadding()
{
	return _padding;
}

void Padding::padIn()
{
	_padDepth++;
}

void Padding::padOut()
{
	_padDepth--;
}

int Padding::getPadDepth() const
{
	return _padDepth;
}

ostream& DFHM::operator << (ostream& out, const Padding& sp)
{
	for (int i = 0; i < sp.getPadDepth(); i++)
		out << "  ";
	return out;
}
