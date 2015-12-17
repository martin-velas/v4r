/*
 * Stopwatch.h
 *
 *  Created on: 24.10.2014
 *      Author: Martin Velas
 */

#ifndef STOPWATCH_H_
#define STOPWATCH_H_

#include <ctime>
#include <boost/chrono.hpp>

namespace v4r {

class Stopwatch {
public:

	Stopwatch() {
		restart();
	}

	void restart() {
		start_time = boost::posix_time::microsec_clock::local_time();
		already_elapsed = boost::posix_time::time_duration();
		paused = false;
	}

	void pause() {
		if(!paused) {
			paused = true;
			already_elapsed += boost::posix_time::microsec_clock::local_time() - start_time;
		}
	}

	void goOn() {
		if(paused) {
			paused = false;
			start_time = boost::posix_time::microsec_clock::local_time();
		}
	}

	// [sec]
	double elapsed() {
		return (boost::posix_time::microsec_clock::local_time() - start_time + already_elapsed).total_microseconds() / 1e6;
	}

private:
	boost::posix_time::ptime start_time;
	boost::posix_time::time_duration already_elapsed;
	bool paused;
};

}

#endif /* STOPWATCH_H_ */
