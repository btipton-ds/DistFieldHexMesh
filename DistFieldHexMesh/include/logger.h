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

#define THREAD_SAFE_LOG(EXPR) { lock_guard g(Logger::get().mutex()); EXPR; }
namespace DFHM {

class Logger;

class Padding {
public:
	class ScopedPad {
	public:
		inline ScopedPad()
		{
			Padding::get().padIn();
		}

		inline ~ScopedPad() {
			Padding::get().padOut();
		}
	};

	static Padding& get();

	void padIn();
	void padOut();
	int getPadDepth() const;

private:
	int _padDepth = 0;
};

class Logger {
public:
	static Logger& get();

	Logger(const std::string& logPath);
	std::ostream& stream(const std::string& streamName);
	std::mutex& mutex();

private:
	std::string _logPath;
	std::map<std::string, std::shared_ptr<std::ofstream>> _streams;
	std::mutex _mutex;
};

inline std::mutex& Logger::mutex()
{
	return _mutex;
}

std::ostream& operator << (std::ostream& out, const Padding& sp);

}