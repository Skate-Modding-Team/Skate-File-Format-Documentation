// (c) Electronic Arts. All Rights Reserved.

#include <unit/unit.h>

#include <rw/collision/libcore.h>
#include <rw/collision/clusteredmeshcluster.h>

#include <rw/collision/meshbuilder/detail/trianglenormal.h>
#include <rw/collision/meshbuilder/detail/trianglevalidator.h>
#include <rw/collision/meshbuilder/detail/trianglelineintersector.h>

#include "testsuitebase.h" // For TestSuiteBase

using namespace rw::collision::meshbuilder;


// Unit tests for clustered mesh builder utilities
class TestTriangleGeometry : public rw::collision::tests::TestSuiteBase
{

public:

    virtual void Initialize()
    {
        SuiteName("TestTriangleGeometry");

        EATEST_REGISTER("TestComputeTriangleNormalFast", "Check triangle normal", TestTriangleGeometry, TestComputeTriangleNormalFast);
        EATEST_REGISTER("TestIsTriangleValid", "Check validity tests", TestTriangleGeometry, TestIsTriangleValid);
        EATEST_REGISTER("TestTriangleLine2D", "Check separation", TestTriangleGeometry, TestTriangleLine2D);
    }

    virtual void SetupSuite()
    {
        rw::collision::tests::TestSuiteBase::SetupSuite();
        rw::collision::InitializeVTables();
    }

private:

    void TestComputeTriangleNormalFast();
    void TestIsTriangleValid();
    void TestTriangleLine2D();

} TestTriangleGeometrySingleton;

/**
Tests the triangle normal value.
*/
void
TestTriangleGeometry::TestComputeTriangleNormalFast()
{
    const rwpmath::Vector3 p0(0.0f, 0.0f, 0.0f);
    const rwpmath::Vector3 p1(0.0f, 0.0f, 1.0f);
    const rwpmath::Vector3 p2(1.0f, 0.0f, 0.0f);

    rwpmath::Vector3 normal = detail::TriangleNormal::ComputeTriangleNormalFast(
        p0,
        p1,
        p2);

    // Expect an edge cosine of 2
    EATESTAssert(rwpmath::IsSimilar( rwpmath::Vector3(0.0f, 1.0f, 0.0f), normal, 0.001f), "bad normal");
}


/**
Tests the validity flag.
*/
void
TestTriangleGeometry::TestIsTriangleValid()
{
    const rwpmath::Vector3 p0(0.0f, 0.0f, 0.0f);
    const rwpmath::Vector3 p1(0.0f, 0.0f, 1.0f);
    const rwpmath::Vector3 p2(0.0f, 0.0f, 1.0f);

    bool isValid = detail::TriangleValidator::IsTriangleValid(
        p0,
        p1,
        p2);

    // Expect an edge cosine of 2
    EATESTAssert(false == isValid, "triangle should not be valid");
}


/**
Tests the separation.
*/
void
TestTriangleGeometry::TestTriangleLine2D()
{
    rwpmath::Vector2 trianglePoint0(0.0f, 0.0f);
    rwpmath::Vector2 trianglePoint1(1.0f, 0.0f);
    rwpmath::Vector2 trianglePoint2(0.0f, 1.0f);
    rwpmath::Vector2 linePoint0(0.5f, -2.0f);
    rwpmath::Vector2 linePoint1(0.5f, -1.0f);

    const bool intersection =  detail::TriangleLineIntersector::IntersectLineWithTriangle2D(
        trianglePoint0,
        trianglePoint1,
        trianglePoint2,
        linePoint0,
        linePoint1);

    // Expect no intersection
    EATESTAssert(false == intersection, "intersection should be false");
}


