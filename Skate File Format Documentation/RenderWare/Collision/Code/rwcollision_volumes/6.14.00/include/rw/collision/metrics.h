// (c) Electronic Arts. All Rights Reserved.
#ifndef PUBLIC_RW_COLLISION_METRICS_H
#define PUBLIC_RW_COLLISION_METRICS_H

/*************************************************************************************************************

 File: rwcmetrics.hpp

 Purpose: Collision metrics services.
 */

#include "rw/collision/common.h"

namespace rw
{
namespace collision
{

#ifdef RWMETRICS

/**
\internal
*/
#define rwcMETRICS(_code) _code

#else /* RWMETRICS */

/**
\internal
*/
#define rwcMETRICS(_code)

#endif /* RWMETRICS */


#ifdef RWMETRICS

/**
\internal

\importlib rwccore
 */
class Timer
{
public:
    typedef int64_t (*QueryFn)(void);

    Timer();

    void        Start();
    void        Stop();
    void        Reset();

    static void SetQueryFunction(QueryFn fn);

    uint64_t        m_value;

private:
    static QueryFn  m_queryFn;
};

/**
\internal
*/
inline
rw::collision::Timer::Timer() : m_value(0)
{
}

/**
\internal
*/
inline void
rw::collision::Timer::Start()
{
    if (m_queryFn)
    {
        m_value -= (*m_queryFn)();
    }
}

/**
\internal
*/
inline void
rw::collision::Timer::Stop()
{
    if (m_queryFn)
    {
        m_value += (*m_queryFn)();
    }
}

/**
\internal
*/
inline void
rw::collision::Timer::Reset()
{
    m_value = 0;
}

/**
\internal
*/
inline void
rw::collision::Timer::SetQueryFunction(QueryFn fn)
{
    m_queryFn = fn;
}

#endif /* RWMETRICS */

} // namespace collision
} // namespace rw

#endif // PUBLIC_RW_COLLISION_METRICS_H
