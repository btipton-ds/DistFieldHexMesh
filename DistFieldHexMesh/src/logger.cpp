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

namespace {
	static Logger s_logger("D:/DarkSky/Projects/output/logs/");
}

Logger& Logger::get()
{
	return s_logger;
}

Logger::Logger(const string& logPath)
	: _logPath(logPath)
{
}

ostream& Logger::stream(const string& streamName)
{
	auto iter = _streams.find(streamName);
	if (iter == _streams.end()) {
		shared_ptr<ofstream> p = make_shared<ofstream>(_logPath + streamName, ios_base::out);
		iter = _streams.insert(make_pair(streamName, p)).first;
	}

	return *iter->second.get();
}

Padding& Padding::get()
{
	static Padding s_padding;
	return s_padding;
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
	for (int i = 0; i < Padding::get().getPadDepth(); i++)
		out << "  ";
	return out;
}
