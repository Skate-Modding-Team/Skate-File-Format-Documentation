// (c) Electronic Arts. All Rights Reserved.
/*************************************************************************************************************

File: benchmark_timer.hpp

Purpose: Helper class for timings in benchmarks

*/

#ifndef BENCHMARK_TIMER_H
#define BENCHMARK_TIMER_H

#include "benchmarkenvironment/timer.h"
#include <EABase/eabase.h>

// ***********************************************************************************************************

namespace rw
{
    namespace collision
    {
        namespace Tests
        {
            // Helper to time multiple iterations of code.
            class BenchmarkTimer
            {
            public:
                BenchmarkTimer() : mTimer(), mTotalTime(0.0f), mMinTime(0.0f), mMaxTime(0.0f), mCount(0)
                {
                }
                void Start()
                {
                    mTimer.Start();
                }
                void Stop()
                {
                    mTimer.Stop();
                    float duration = mTimer.AsSeconds();
                    mTotalTime += duration;
                    if (mCount == 0)
                    {
                        mMaxTime = duration;
                        mMinTime = duration;
                    }
                    else
                    {
                        if (duration > mMaxTime) mMaxTime = duration;
                        if (duration < mMinTime) mMinTime = duration;
                    }
                    mCount++;
                }
                void Reset()
                {
                    mTotalTime = mMinTime = mMaxTime = 0.0f;
                    mCount = 0;
                }
                /// Convert time to run count iterations into average milliseconds
                double GetAverageDurationMilliseconds()
                {
                    return ToMilliseconds(mTotalTime) / ((double) mCount);
                }
                double GetMinDurationMilliseconds()
                {
                    return ToMilliseconds(mMinTime);
                }
                double GetMaxDurationMilliseconds()
                {
                    return ToMilliseconds(mMaxTime);
                }
            private:
                double ToMilliseconds(float duration)
                {
                    return 1000.0 * (double) duration;
                }
                benchmarkenvironment::Timer mTimer;
                float  mTotalTime;
                float  mMinTime;
                float  mMaxTime;
                uint32_t mCount;
            };

        }
    }
}

// ***********************************************************************************************************

#endif // !BENCHMARK_TIMER_H
