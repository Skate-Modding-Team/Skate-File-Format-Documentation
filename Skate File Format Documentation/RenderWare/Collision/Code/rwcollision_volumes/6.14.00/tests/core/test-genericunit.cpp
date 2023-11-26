// (c) Electronic Arts. All Rights Reserved.
/*************************************************************************************************************

File: test-genericunit.cpp

Purpose: instantiate unit tests for GenericClusterUnit class.

*/

// Intentionally include code under test first to check standalone header.
#include <rw/collision/genericclusterunit.h>   // for the GenericClusterUnit<> we're testing

#include "test-clusterunit.h"                   // for TestClusterUnit<> base class


// ***********************************************************************************************************

namespace EA 
{
    namespace Collision
    {
        namespace Tests
        {
            // We'll test the GenricUnit with dynamic compression.
            // Specific compression cases effectively get tested through this.
            typedef rw::collision::GenericClusterUnit<rw::collision::ClusteredMeshCluster::COMPRESSION_DYNAMIC> TestUnit;

            // We could add additional tests to this test suite if we wanted.
            class TestGenericUnit : public TestClusterUnit<TestUnit> 
            {
            public:
                TestGenericUnit() : TestClusterUnit<TestUnit>("TestGenericUnit", "test-genericunit.elf")
                {
                }

            } gGenericUnitTest;
        }
    }
}

// ***********************************************************************************************************

