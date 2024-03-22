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

#include <memory>
#include <string>
#include <map>
#include <mutex>
#include <iostream>
#include <fstream>

#if LOGGING_ENABLED
#define LOG(EXPR) { EXPR; }
#else
#define LOG(EXPR)
#endif

namespace DFHM {

class Logger;

class Padding {
public:
	class ScopedPad {
	public:
		inline ScopedPad(Padding& padding)
			: _padding(padding)
		{
			_padding.padIn();
		}

		inline ~ScopedPad() {
			_padding.padOut();
		}

	private:
		Padding& _padding;
	};

	void padIn();
	void padOut();
	int getPadDepth() const;

private:
	int _padDepth = 0;
};

class Logger {
public:
	static std::shared_ptr<Logger> get(const std::string& streamName);


	Logger(const std::string& logPath);
	std::ostream& getStream();
	Padding& getPadding();
	std::mutex& mutex();
	void setEnabled(bool val);
	bool enabled() const;

private:
	bool _enabled = true;
	std::ofstream _stream;
	Padding _padding;
	std::mutex _mutex;
};

inline std::mutex& Logger::mutex()
{
	return _mutex;
}

inline void Logger::setEnabled(bool val)
{
	_enabled = val;
}

inline bool Logger::enabled() const
{
	return _enabled;
}

std::ostream& operator << (std::ostream& out, const Padding& sp);

}