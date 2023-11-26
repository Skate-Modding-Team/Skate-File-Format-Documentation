// (c) Electronic Arts. All Rights Reserved.
#ifndef SHARED_TESTSUITEBASE_H
#define SHARED_TESTSUITEBASE_H

#include <EABase/eabase.h>
#include <unit/unit.h>

namespace rw
{
namespace collision
{
namespace tests
{


// The TestSuiteBase class is used to implement common functionality and to cleanly work around some issues with
// EATest::TestSuite. Namely having to overload both overloads of Setup and Teardown to fix compiler
// warnings on some platforms.
//
// NOTE: Test suites should overload SetupSuite and TeardownSuite NOT Setup and Teardown.
class TestSuiteBase : public EATest::TestSuite
{
private:

    // Both versions of Setup get called by unit so it is very important only one does something.

    virtual void Setup()
    {
        SetupSuite();
    }

    virtual void Setup(void *)
    {
    }

    // Both versions of Teardown get called by unit so it is very important only one does something.

    virtual void Teardown()
    {
        TeardownSuite();
    }

    virtual void Teardown(void *)
    {
    }

protected:

    virtual void SetupSuite()
    {
    }

    virtual void TeardownSuite()
    {
    }

public:

    TestSuiteBase()
    {
    }

};


} // namespace tests
} // namespace collision
} // namespace rw


#endif // SHARED_TESTSUITEBASE_H
