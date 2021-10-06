#pragma once

#include <iostream>
#include <chrono>
#include <ctime>
#include <cmath>
#include <stdexcept>
#include <string>

/** simple timer class to time how long parts of algorithm take */
class Timer
{
private:
    std::chrono::time_point<std::chrono::system_clock> _startTime, _endTime;
    bool _isRunning = false;
    bool _hasRun = false;
    std::string _timerName = "";

public:
    Timer();
    Timer(std::string name);
    void restart();
    void start();
    void stop();
    double elapsedMilliseconds();
    double elapsedSeconds();

    std::string getSecondsElapsedMessage();
    std::string getMillisecondsElapsedMessage();

    bool isRunning();
    bool hasRun();
};
