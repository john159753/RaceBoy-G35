#include "StopWatch.h"

StopWatch::StopWatch(enum Resolution res)
{
	_res = res;
	switch (_res) {
	case MICROS:
		_gettime = micros;
		break;
	case MILLIS:
		_gettime = millis;
		break;
	case SECONDS:
		_gettime = seconds;
		break;
	default:
		_gettime = millis;
		break;
	}
	reset();
}
//Reset Timer
void StopWatch::reset()
{
	_state = StopWatch::RESET;
	_starttime = _stoptime = 0;
}
//Start / Continue timer
void StopWatch::start()
{
	if (_state == StopWatch::RESET || _state == StopWatch::STOPPED)
	{
		_state = StopWatch::RUNNING;
		unsigned long t = _gettime();
		_starttime += t - _stoptime;
		_stoptime = t;
	}
}
//Timer value
unsigned long StopWatch::value()
{
	if (_state == StopWatch::RUNNING) _stoptime = _gettime();
	return _stoptime - _starttime;
}
//Stop the timer
void StopWatch::stop()
{
	if (_state == StopWatch::RUNNING)
	{
		_state = StopWatch::STOPPED;
		_stoptime = _gettime();
	}
}
//Returns true if timer is running
bool StopWatch::isRunning()
{
	return (_state == StopWatch::RUNNING);
}
//Returns true if timer is running
enum StopWatch::State StopWatch::state()
{
	return _state;
}
