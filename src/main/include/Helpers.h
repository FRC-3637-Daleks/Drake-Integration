#pragma once

#include <frc/Timer.h>

class BlockingTimer
{
public:

    void wait_BLOCKING(double delay)
    {
        double timebegin = MyTimer.Get();
        MyTimer.Reset();
        MyTimer.Start();
        while( MyTimer.HasPeriodPassed(delay) == false);
        MyTimer.Stop();
    }

    frc::Timer MyTimer;
};