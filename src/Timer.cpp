#include "Timer.h"

Timer::Timer() {}

Timer::Timer(std::string timerName)
{
    _timerName = timerName;
}

/** starts the timer */
void Timer::start()
{
    _startTime = std::chrono::system_clock::now();
    _isRunning = true;
    _hasRun = true;
}

void Timer::restart()
{
    _startTime = std::chrono::system_clock::now();
    _isRunning = true;
    _hasRun = true;
}

/** stops the timer from running */
void Timer::stop()
{
    _endTime = std::chrono::system_clock::now();
    _isRunning = false;
}

/** returns the number of elapsed milliseconds since starting the timer. */
double Timer::elapsedMilliseconds()
{
    std::chrono::time_point<std::chrono::system_clock> endTime;

    // if timer is still running, simply set endTime.
    if (_isRunning)
    {
        endTime = std::chrono::system_clock::now();
    }
    // if timer isn't running, it has already run so return _endTime.
    else if (_hasRun)
    {
        _endTime = _endTime;
    }
    // error timer has not yet been run at least once.
    else
    {
        throw std::out_of_range("Error, the timer has not yet been run!");
    }

    return std::chrono::duration_cast<std::chrono::milliseconds>(endTime - _startTime).count();
}

/** returns the number of elapsed seconds since the timer has been started. */
double Timer::elapsedSeconds()
{
    return elapsedMilliseconds() / 1000.0;
}

/** prints out a message with the number of seconds elapsed since start */
std::string Timer::getSecondsElapsedMessage()
{

    double elapsedSeconds = this->elapsedSeconds();

    std::string msg = "Timer " + _timerName + ":" + std::to_string(elapsedSeconds) + " seconds elapsed. ";
    return msg;
};

/** prints out a message with the number of milliseconds elapsed since start */
std::string Timer::getMillisecondsElapsedMessage()
{
    double elapsedMilliseconds = this->elapsedMilliseconds();
    std::string msg = "Timer " + _timerName + ":" + std::to_string(elapsedMilliseconds) + " milliseconds elapsed. ";
    return msg;
};

/** returns whether the timer is currently running or not */
bool Timer::isRunning()
{
    return _isRunning;
}

/** returns if the timer has been run at least once */
bool Timer::hasRun()
{
    return _hasRun;
}