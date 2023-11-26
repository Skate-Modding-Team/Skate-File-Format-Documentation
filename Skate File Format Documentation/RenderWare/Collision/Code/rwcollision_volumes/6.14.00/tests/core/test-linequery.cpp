// (c) Electronic Arts. All Rights Reserved.


#include <EABase/eabase.h>
#include <eaphysics/base.h>
#include <rw/collision/libcore.h>
#include <rw/collision/triangle.h>

#include <eaphysics/unitframework/allocator.h> // For ResetAllocator
#include <eaphysics/unitframework/creator.h> // For Creator

#include "testsuitebase.h" // For TestSuiteBase

#include <unit/unit.h>

using namespace rwpmath;

namespace rw {
namespace collision {
RwpBool
FatTriangleLineSegIntersect(VolumeLineSegIntersectResult &result,
                            Vector3::InParam lineStart, Vector3::InParam lineDelta,
                            Vector3::InParam v0, Vector3::InParam v1, 
                            Vector3::InParam v2, float radius );
}
}

using namespace rw::collision;


// ***********************************************************************************************************
// Test suite

class TestLineQuery : public tests::TestSuiteBase
{
public:
    virtual void Initialize()
    {
        SuiteName("TestLineQuery");
        EATEST_REGISTER("TestFatLineVsTriangle", "TestFatLineVsTriangle", TestLineQuery, TestFatLineVsTriangle);
        EATEST_REGISTER("TestLineVsTwoSidedTriangle", "TestLineVsTwoSidedTriangle", TestLineQuery, TestLineVsTwoSidedTriangle);
        EATEST_REGISTER("TestFatLineVsBox", "TestFatLineVsBox", TestLineQuery, TestFatLineVsBox);
    }
    void SetupSuite()
    {
        tests::TestSuiteBase::SetupSuite();

        // Initialise the collision system
        Volume::InitializeVTable();
    }
    void TeardownSuite()
    {
        EA::Physics::UnitFramework::ResetAllocator();
        tests::TestSuiteBase::TeardownSuite();
    }

private:
    void TestFatLineVsTriangle();
    void TestLineVsTwoSidedTriangle();
    void TestFatLineVsBox();
} TestLineQuerySingleton;


void TestLineQuery::TestFatLineVsTriangle()
{
    VolumeLineSegIntersectResult result;
    Vector3 start(-2, 0, 0);
    Vector3 delta(2,0,0);
    Vector3 v0(1, 0, -1);
    Vector3 v1(0, -1, 0);
    Vector3 v2(0, 1, 0);
    result.normal = Normalize(Cross(v1-v0,v2-v0));
    float rad = .5f;

    RwpBool ok = FatTriangleLineSegIntersect(result, start, delta, v0, v1, v2, rad);
    EATESTAssert(ok == 1, "return ok");
    EATESTAssert(IsSimilar(result.position, Vector3(-.5f,0,0)), "result position");
    EATESTAssert(IsSimilar(result.lineParam, (2-rad)/2), "result lineParam");
    EATESTAssert(IsSimilar(result.volParam, Vector3(.5f,.5f,0)), "result volParam");
    EATESTAssert(IsSimilar(result.normal, Vector3(-1,0,0)), "result normal");
}

void TestLineQuery::TestLineVsTwoSidedTriangle()
{
    VolumeLineSegIntersectResult singleSidedResult;
    VolumeLineSegIntersectResult twoSidedResult;
    Vector3 start(0, 0, -2);
    Vector3 delta(0, 0, 4);
    Vector3 v0(-1, -1, 0);
    Vector3 v1(1, -1, 0);
    Vector3 v2(0, 1, 1);
    singleSidedResult.normal = Normalize(Cross(v1-v0,v2-v0));
    twoSidedResult.normal = Normalize(Cross(v1-v0,v2-v0));

    RwpBool frontFaceTest = TriangleLineSegIntersect(singleSidedResult, start, delta, v0, v1, v2);
    EATESTAssert(frontFaceTest == 0, "single sided front face missed");
    RwpBool backFaceTest = TriangleLineSegIntersect(singleSidedResult, start + delta, delta * -1.0f, v0, v1, v2);
    EATESTAssert(backFaceTest == 1, "single sided back face hit");

    RwpBool frontFaceTwoSidedTest = TriangleLineSegIntersectTwoSided(twoSidedResult, start, delta, v0, v1, v2);
    EATESTAssert(frontFaceTwoSidedTest == 1, "two sided original line hit");

    EATESTAssert(IsSimilar(singleSidedResult.position, Vector3(0,0,0.5)), "position result");
    EATESTAssert(IsSimilar(singleSidedResult.normal, Normalize(Cross(v1-v0,v2-v0))), "normal result");

    EATESTAssert(IsSimilar(singleSidedResult.position, twoSidedResult.position), "single sided and two sided position match");
    EATESTAssert(IsSimilar(singleSidedResult.normal, twoSidedResult.normal * -1.0f), "single sided and two sided normal are inverted");

    Vector3 edge1 = v1 - v0;
    Vector3 edge2 = v2 - v0;
    Vector3 singleSidedParametricPos = v0 + (singleSidedResult.volParam.GetX() * edge1) + (singleSidedResult.volParam.GetY() * edge2);
    EATESTAssert(IsSimilar(singleSidedResult.position, singleSidedParametricPos), "single sided parametric result");
    Vector3 twoSidedParametricPos = v0 + (twoSidedResult.volParam.GetX() * edge1) + (twoSidedResult.volParam.GetY() * edge2);
    EATESTAssert(IsSimilar(twoSidedResult.position, twoSidedParametricPos), "two sided parametric result");

    RwpBool backFaceTwoSidedTest = TriangleLineSegIntersectTwoSided(twoSidedResult, start + delta, delta * -1.0f, v0, v1, v2);
    EATESTAssert(backFaceTwoSidedTest == 1, "two sided reversed line hit");
}

// TestFatLineVsBox: 
// This test was added to check bug hansoft://oh-hansoft;EATech/Task/74464 
// in which the incorrect normal was returned for fatline box intersections with an edge.
void TestLineQuery::TestFatLineVsBox()
{
    //SetUpBoxVolume
    Vector3 boxHalfDimensions(0.3f, 0.3f, 0.6f);
    Matrix44Affine boxTransform = GetMatrix44Affine_Identity();
    boxTransform.SetW(Vector3(14.7f, 0.3f, 66.6f)); 
    BoxVolume * boxVolume = EA::Physics::UnitFramework::Creator<BoxVolume>().New(boxHalfDimensions, 0.0f);

    //Line details
    Vector3 lineStart(14.4541f, 1.29871f, 65.9679f);// 
    Vector3 lineEnd(14.4541f, -0.501287f, 65.9679f);
    VecFloat lineRadius(0.4f);
    //Run LineSegIntersect
    VolumeLineSegIntersectResult result;
    RwpBool hit = boxVolume->LineSegIntersect(lineStart, lineEnd, &boxTransform, result, lineRadius);
    
    //Check Result
    EATESTAssert(hit == 1, "FatLine should hit box");
    EATESTAssert(IsSimilar(result.position, Vector3(14.4541f, 0.6f, 66.0f)), "Incorrect Position");
    EATESTAssert(IsSimilar(result.normal, Vector3(0.0, 0.996776f, -0.0802383f)), "Incorrect Normal");
}
