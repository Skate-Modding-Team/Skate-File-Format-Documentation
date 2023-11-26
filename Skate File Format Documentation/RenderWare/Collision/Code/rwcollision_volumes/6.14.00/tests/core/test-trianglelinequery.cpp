// (c) Electronic Arts. All Rights Reserved.
#ifdef _MSC_VER
#pragma warning(disable: 4700)
#endif

#include <EABase/eabase.h>
#include <eaphysics/base.h>
#include <rw/collision/libcore.h>
#include <rw/collision/trianglequery.h>

#include <unit/unit.h>

#include "testsuitebase.h" // For TestSuiteBase

using namespace rw::collision;
using namespace rwpmath;

// ***********************************************************************************************************
// Test suite


class TestTriangleQuery : public tests::TestSuiteBase
{

public:
    TestTriangleQuery()
    {
    }

    virtual ~TestTriangleQuery()
    {
    }

    virtual void Initialize()
    {
        SuiteName("TestTriangleQuery");
        EATEST_REGISTER("TestBranchedLineTriangleTest", 
                        "Test the branched line test", 
                        TestTriangleQuery, TestBranchedLineTriangleTest);

        EATEST_REGISTER("Test1WayNonNormalizedLineTriangleTest", 
                        "Test the 1 way, vectorized, non-normalized line test", 
                        TestTriangleQuery, Test1WayNonNormalizedLineTriangleTest);

        EATEST_REGISTER("Test1WayNormalizedLineTriangleTest", 
                        "Test the 1 way, vectorized & normalized line test", 
                        TestTriangleQuery, Test1WayNormalizedLineTriangleTest);

        EATEST_REGISTER("Test4WayNonNormalizedLineTriangleTest", 
                        "Test the 4 way, vectorized, non-normalized line test", 
                        TestTriangleQuery, Test4WayNonNormalizedLineTriangleTest);

        EATEST_REGISTER("Test4WayNormalizedLineTriangleTest", 
                        "Test the 4 way, vectorized & normalized line test", 
                        TestTriangleQuery, Test4WayNormalizedLineTriangleTest);
        
        EATEST_REGISTER("Test16WayNonNormalizedLineTriangleTest", 
                        "Test the 16 way, vectorized, non-normalized line test", 
                        TestTriangleQuery, Test16WayNonNormalizedLineTriangleTest);

        EATEST_REGISTER("Test16WayNormalizedLineTriangleTest", 
                        "Test the 16 way, vectorized & normalized line test", 
                        TestTriangleQuery, Test16WayNormalizedLineTriangleTest);

        EATEST_REGISTER("TestLineTriangleFat",
                        "Test line vs fat triangle",
                        TestTriangleQuery, TestLineTriangleFat);
    }

    void Test1WayNonNormalizedLineTriangleTest();
    void Test1WayNormalizedLineTriangleTest();
    void Test4WayNonNormalizedLineTriangleTest();
    void Test4WayNormalizedLineTriangleTest();
    void Test16WayNormalizedLineTriangleTest();
    void Test16WayNonNormalizedLineTriangleTest();
    void TestBranchedLineTriangleTest();
    void TestLineTriangleFat();
private:
    void Do4WayNormalizedTriangleLineTest(Vector3 * v0s, Vector3 * v1s, Vector3 * v2s, 
                                          Vector3::InParam lineStart, Vector3::InParam lineDelta,
                                          RwpBool * expectedIntersections, Vector4::InParam expectedLineParams,
                                          Vector4::InParam expectedTri1Params, Vector4::InParam expectedTri2Params,
                                          Vector4::InParam expectedDets);

    void Do4WayNormalizedTriangleLineTest(Vector3 * v0s, Vector3 * v1s, Vector3 * v2s, 
                                          Vector3::InParam lineStart, Vector3::InParam lineDelta,
                                          RwpBool * expectedIntersections, Vector3 * expectedPositions,
                                          Vector4::InParam expectedLineParams, Vector3 * expectedTriParams);

    void Do16WayNormalizedTriangleLineTest(Vector3 * v0s, Vector3 * v1s, Vector3 * v2s, 
                                           Vector3::InParam lineStart, Vector3::InParam lineDelta,
                                           RwpBool * expectedIntersections, Vector3 * expectedPositions,
                                           VecFloat* expectedLineParams, Vector3 * expectedTriParams);

    void Do16WayNonNormalizedTriangleLineTest(Vector3 * v0s, Vector3 * v1s, Vector3 * v2s, 
                                              Vector3::InParam lineStart, Vector3::InParam lineDelta,
                                              RwpBool * expectedIntersections, VecFloat * expectedLineParams,
                                              VecFloat* expectedTri1Params, VecFloat* expectedTri2Params,
                                              VecFloat* expectedDets);
} testTriangleQuery;


void 
TestTriangleQuery::TestLineTriangleFat()
{
    float TEST_EPSILON = 1e-6f;

    // BUG#32249 - Triangle fat line intersect returns wrong normal
    // http://eahq-dt4.rws.ad.ea.com/scripts/texcel/DevTrack/devtrack.dll?IssueInfo?&IssueAccessID=MNWIDUPUQCWJBWOTW
    {
        Vector3 v0(1.f, 0.f, 0.f);
        Vector3 v1(0.f, 1.f, 0.f);
        Vector3 v2(0.f, 0.f, 0.f);
        Vector3 normal(0.f, 0.f, 1.f);

        Vector3 lineStart(1.f, 1.f, 5.f);
        Vector3 lineDelta(0.f, 0.f, -10.f);

        VolumeLineSegIntersectResult result;
        result.normal = normal; // Must be initialized before calling FatTriangleLineSegIntersect
        RwpBool intersects;

        float radius = 1.f;
        intersects = FatTriangleLineSegIntersect(result, lineStart, lineDelta, v0, v1, v2, radius);
        EATESTAssert(intersects, "Line query should intersect");
        EATESTAssert(IsSimilar(result.lineParam, VecFloat((5.f - SQRT_HALF)/10.f), TEST_EPSILON), "Incorrect lineParam");
        EATESTAssert(IsSimilar(result.normal, Vector3(.5f, .5f, SQRT_HALF), TEST_EPSILON), "Incorrect normal");
        EATESTAssert(IsSimilar(result.position, Vector3(1.f, 1.f, SQRT_HALF), TEST_EPSILON), "Incorrect position");
        EATESTAssert(IsSimilar(result.volParam, Vector3(.5f, 0.f, 0.f), TEST_EPSILON), "Incorrect volParam");
    }

    // http://docs.ea.com/RWPhysicsDev:BUG31507_Fat_line_triangle_queries_return_false_positives
    // A line starting in face region, outside fatness by up to ~1 radius, and 
    // with direction parallel to triangle face can return a false positive.
    {
        Vector3 v0(0.0f, 0.0f, 1.0f);
        Vector3 v1(0.5f, 1.0f, 0.5f);
        Vector3 v2(1.0f, 0.0f, 0.0f);
        Vector3 normal = Normalize(Vector3(-1.0f, 0.0f, -1.0f));

        Vector3 lineStart(0.0f, 0.5f, 0.0f);  // distance sqrt(0.5)=0.71 from triangle face
        Vector3 lineDelta(0.0f, 1.0f, 0.0f);
        
        VolumeLineSegIntersectResult result;
        result.normal = normal; // Must be initialized before calling FatTriangleLineSegIntersect
        RwpBool intersects;
        
        // Test negative result
        float radius = 0.6f;
        intersects = FatTriangleLineSegIntersect(result, lineStart, lineDelta, v0, v1, v2, radius);
        EATESTAssert(!intersects, "Line query should not intersect");

        // Test positive result
        radius = 0.8f;
        intersects = FatTriangleLineSegIntersect(result, lineStart, lineDelta, v0, v1, v2, radius);
        EATESTAssert(intersects, "Line query should intersect");
        EATESTAssert(result.lineParam == 0.0f, "Incorrect lineParam");
        EATESTAssert(IsSimilar(result.normal, normal, TEST_EPSILON), "Incorrect normal");
        EATESTAssert(IsSimilar(result.position, lineStart, TEST_EPSILON), "Incorrect position");
    }

    // BUG#31812 - FatTriangleLineSegIntersect returns invalid normal for line starting on face (of non-fat tri)
    // http://eahq-dt4.rws.ad.ea.com/scripts/texcel/DevTrack/devtrack.dll?IssueInfo?&IssueAccessID=MNWIDPUSQCWJBMIQW
    {
        Vector3 v0(0.0f, 0.0f, 0.0f);
        Vector3 v1(1.0f, 0.0f, 0.0f);
        Vector3 v2(0.0f, 1.0f, 0.0f);

        // Test on-face case
        Vector3 normal(0.0f, 0.0f, 1.0f);
        Vector3 lineStart(0.25f, 0.25f, 0.0f);
        Vector3 lineDelta(0.0f, 0.0f, 1.0f);
        VolumeLineSegIntersectResult result;
        result.normal = normal;
        float radius = 1.0f;
        RwpBool intersects = FatTriangleLineSegIntersect(result, lineStart, lineDelta, v0, v1, v2, radius);
        EATESTAssert(intersects, "Line should intersect");
        EATESTAssert(result.lineParam == 0.0f, "Incorrect lineParam");
        EATESTAssert(IsSimilar(result.normal, normal, TEST_EPSILON), "Incorrect normal");
        EATESTAssert(IsSimilar(result.position, lineStart, TEST_EPSILON), "Incorrectposition");

        // Test not on-face but starting inside
        lineStart.Set(-0.5f, -0.5f, -0.5f);
        normal = Normalize(lineStart);
        intersects = FatTriangleLineSegIntersect(result, lineStart, lineDelta, v0, v1, v2, radius);
        EATESTAssert(intersects, "Line should intersect");
        EATESTAssert(result.lineParam == 0.0f, "Incorrect lineParam");
        EATESTAssert(IsSimilar(result.normal, normal, TEST_EPSILON), "Incorrect normal");
        EATESTAssert(IsSimilar(result.position, lineStart, TEST_EPSILON), "Incorrectposition");
    }
}


void 
TestTriangleQuery::TestBranchedLineTriangleTest()
{
    const float TEST_EPSILON = 1e-6f;
    Vector3 v0(0.0f,  0.0f,  5.0f);
    Vector3 v1(0.0f,  10.0f, 5.0f);
    Vector3 v2(10.0f, 0.0f,  5.0f);

    VecFloat lineParam = rwpmath::GetVecFloat_Zero();
    Vector3 triParams = rwpmath::GetVector3_Zero(), position = rwpmath::GetVector3_Zero();

    // ---- Test Intersecting ----
    Vector3 lineStart(2.0f, 3.0f, 0.0f);
    Vector3 lineDelta(0.0f, 0.0f, 10.0f);

    RwpBool intersects = TriangleLineSegIntersect_Branching(v0, v1, v2, lineStart, lineDelta, position, lineParam, triParams);

    EATESTAssert(intersects, "Line query should intersect");
    EATESTAssert(IsSimilar(lineParam, VecFloat(0.5f), TEST_EPSILON), "Line query line param wrong");
    EATESTAssert(IsSimilar(triParams, Vector3(0.3f, 0.2f, 0.0f), TEST_EPSILON), "Line query triParam wrong");
    EATESTAssert(IsSimilar(position, Vector3(2.0f, 3.0f, 5.0f), TEST_EPSILON), "Line query position wrong");

    // ---- Test not intersecting (line too short)----
    lineStart = Vector3(2.0f, 3.0f, 0.0f);
    lineDelta = Vector3(0.0f, 0.0f, 2.0f);

    intersects = TriangleLineSegIntersect_Branching(v0, v1, v2, lineStart, lineDelta, position, lineParam, triParams);

    EATESTAssert(!intersects, "Line query should not intersect (too short)");

    // ---- Test not intersecting (outside of edge 1)----
    lineStart = Vector3(2.0f, 11.0f, 0.0f);
    lineDelta = Vector3(0.0f, 0.0f, 10.0f);

    intersects = TriangleLineSegIntersect_Branching(v0, v1, v2, lineStart, lineDelta, position, lineParam, triParams);

    EATESTAssert(!intersects, "Line query should not intersect (outside of edge 1)");

    // ---- Test not intersecting (outside of edge 2)----
    lineStart = Vector3(11.0f, 2.0f, 0.0f);
    lineDelta = Vector3(0.0f, 0.0f, 10.0f);

    intersects = TriangleLineSegIntersect_Branching(v0, v1, v2, lineStart, lineDelta, position, lineParam, triParams);

    // ---- Test not intersecting (outside of edge 3)----
    lineStart = Vector3(8.0f, 8.0f, 0.0f);
    lineDelta = Vector3(0.0f, 0.0f, 10.0f);

    intersects = TriangleLineSegIntersect_Branching(v0, v1, v2, lineStart, lineDelta, position, lineParam, triParams);

    EATESTAssert(!intersects, "Line query should not intersect (outside of edge 3)");
}


void 
TestTriangleQuery::Test1WayNormalizedLineTriangleTest()
{
    const float TEST_EPSILON = 1e-6f;
    Vector3 v0(0.0f,  0.0f,  5.0f);
    Vector3 v1(0.0f,  10.0f, 5.0f);
    Vector3 v2(10.0f, 0.0f,  5.0f);

    VecFloat lineParam;
    Vector3 triParams, position;

    // ---- Test Intersecting ----
    Vector3 lineStart(2.0f, 3.0f, 0.0f);
    Vector3 lineDelta(0.0f, 0.0f, 10.0f);

    RwpBool intersects = TriangleLineSegIntersect(v0, v1, v2, lineStart, lineDelta, position, lineParam, triParams);

    EATESTAssert(intersects, "Line query should intersect");
    EATESTAssert(IsSimilar(lineParam, VecFloat(0.5f), TEST_EPSILON), "Line query line param wrong");
    EATESTAssert(IsSimilar(triParams, Vector3(0.3f, 0.2f, 0.0f), TEST_EPSILON), "Line query triParam wrong");
    EATESTAssert(IsSimilar(position, Vector3(2.0f, 3.0f, 5.0f), TEST_EPSILON), "Line query position wrong");

    // ---- Test not intersecting (line too short)----
    lineStart = Vector3(2.0f, 3.0f, 0.0f);
    lineDelta = Vector3(0.0f, 0.0f, 2.0f);

    intersects = TriangleLineSegIntersect(v0, v1, v2, lineStart, lineDelta, position, lineParam, triParams);

    EATESTAssert(!intersects, "Line query should not intersect (too short)");

    // ---- Test not intersecting (outside of edge 1)----
    lineStart = Vector3(2.0f, 11.0f, 0.0f);
    lineDelta = Vector3(0.0f, 0.0f, 10.0f);

    intersects = TriangleLineSegIntersect(v0, v1, v2, lineStart, lineDelta, position, lineParam, triParams);

    EATESTAssert(!intersects, "Line query should not intersect (outside of edge 1)");

    // ---- Test not intersecting (outside of edge 2)----
    lineStart = Vector3(11.0f, 2.0f, 0.0f);
    lineDelta = Vector3(0.0f, 0.0f, 10.0f);

    intersects = TriangleLineSegIntersect(v0, v1, v2, lineStart, lineDelta, position, lineParam, triParams);

    // ---- Test not intersecting (outside of edge 3)----
    lineStart = Vector3(8.0f, 8.0f, 0.0f);
    lineDelta = Vector3(0.0f, 0.0f, 10.0f);

    intersects = TriangleLineSegIntersect(v0, v1, v2, lineStart, lineDelta, position, lineParam, triParams);

    EATESTAssert(!intersects, "Line query should not intersect (outside of edge 3)");
}


void 
TestTriangleQuery::Test1WayNonNormalizedLineTriangleTest()
{
    const float TEST_EPSILON = 1e-4f;
    Vector3 v0(0.0f,  0.0f,  5.0f);
    Vector3 v1(0.0f,  10.0f, 5.0f);
    Vector3 v2(10.0f, 0.0f,  5.0f);

    Vector3 edge1 = v1 - v0;
    Vector3 edge2 = v2 - v0;

    VecFloat lineParam, triParam1, triParam2, det;

    // ---- Test Intersecting ----
    Vector3 lineStart(2.0f, 3.0f, 0.0f);
    Vector3 lineDelta(0.0f, 0.0f, 10.0f);

    MaskScalar intersects = TriangleLineSegIntersect(v0, edge1, edge2, lineStart, lineDelta, det, triParam1, triParam2, lineParam);

    EATESTAssert(intersects.GetBool(), "Line query should intersect");
    EATESTAssert(IsSimilar(lineParam, VecFloat(500.0f), TEST_EPSILON), "Line query line param wrong");
    EATESTAssert(IsSimilar(triParam1, VecFloat(300.0f), TEST_EPSILON), "Line tri param 1 wrong");
    EATESTAssert(IsSimilar(triParam2, VecFloat(200.0f), TEST_EPSILON), "Line tri param 2 wrong");
    EATESTAssert(IsSimilar(det, VecFloat(1000.0f), TEST_EPSILON), "Line query line param wrong");

    // ---- Test not intersecting (line too short)----
    lineStart = Vector3(2.0f, 3.0f, 0.0f);
    lineDelta = Vector3(0.0f, 0.0f, 2.0f);

    intersects = TriangleLineSegIntersect(v0, edge1, edge2, lineStart, lineDelta, det, triParam1, triParam2, lineParam);

    EATESTAssert(!intersects.GetBool(), "Line query should not intersect (too short)");

    // ---- Test not intersecting (outside of edge 1)----
    lineStart = Vector3(2.0f, 11.0f, 0.0f);
    lineDelta = Vector3(0.0f, 0.0f, 10.0f);

    intersects = TriangleLineSegIntersect(v0, edge1, edge2, lineStart, lineDelta, det, triParam1, triParam2, lineParam);

    EATESTAssert(!intersects.GetBool(), "Line query should not intersect (outside of edge 1)");

    // ---- Test not intersecting (outside of edge 2)----
    lineStart = Vector3(11.0f, 2.0f, 0.0f);
    lineDelta = Vector3(0.0f, 0.0f, 10.0f);

    intersects = TriangleLineSegIntersect(v0, edge1, edge2, lineStart, lineDelta, det, triParam1, triParam2, lineParam);

    // ---- Test not intersecting (outside of edge 3)----
    lineStart = Vector3(8.0f, 8.0f, 0.0f);
    lineDelta = Vector3(0.0f, 0.0f, 10.0f);

    intersects = TriangleLineSegIntersect(v0, edge1, edge2, lineStart, lineDelta, det, triParam1, triParam2, lineParam);

    EATESTAssert(!intersects.GetBool(), "Line query should not intersect (outside of edge 3)");
}


void 
TestTriangleQuery::Do4WayNormalizedTriangleLineTest(Vector3 * v0s, Vector3 * v1s, Vector3 * v2s, 
                                                    Vector3::InParam lineStart, Vector3::InParam lineDelta,
                                                    RwpBool * expectedIntersections, Vector4::InParam expectedLineParams,
                                                    Vector4::InParam expectedTri1Params, Vector4::InParam expectedTri2Params,
                                                    Vector4::InParam expectedDets)
{

    Vector4 v0x, v0y, v0z, edge1x, edge1y, edge1z, edge2x, edge2y, edge2z;
    v0x = rwpmath::Vector4(v0s[0].GetX(), v0s[1].GetX(), v0s[2].GetX(), v0s[3].GetX());
    v0y = rwpmath::Vector4(v0s[0].GetY(), v0s[1].GetY(), v0s[2].GetY(), v0s[3].GetY());
    v0z = rwpmath::Vector4(v0s[0].GetZ(), v0s[1].GetZ(), v0s[2].GetZ(), v0s[3].GetZ());

    // Find vectors for the two edges sharing v0
    rwpmath::Vector3 edge1A = v1s[0] - v0s[0];
    rwpmath::Vector3 edge1B = v1s[1] - v0s[1];
    rwpmath::Vector3 edge1C = v1s[2] - v0s[2];
    rwpmath::Vector3 edge1D = v1s[3] - v0s[3];

    edge1x = rwpmath::Vector4(edge1A.GetX(), edge1B.GetX(), edge1C.GetX(), edge1D.GetX());
    edge1y = rwpmath::Vector4(edge1A.GetY(), edge1B.GetY(), edge1C.GetY(), edge1D.GetY());
    edge1z = rwpmath::Vector4(edge1A.GetZ(), edge1B.GetZ(), edge1C.GetZ(), edge1D.GetZ());

    rwpmath::Vector3 edge2A = v2s[0] - v0s[0];
    rwpmath::Vector3 edge2B = v2s[1] - v0s[1];
    rwpmath::Vector3 edge2C = v2s[2] - v0s[2];
    rwpmath::Vector3 edge2D = v2s[3] - v0s[3];

    edge2x = rwpmath::Vector4(edge2A.GetX(), edge2B.GetX(), edge2C.GetX(), edge2D.GetX());
    edge2y = rwpmath::Vector4(edge2A.GetY(), edge2B.GetY(), edge2C.GetY(), edge2D.GetY());
    edge2z = rwpmath::Vector4(edge2A.GetZ(), edge2B.GetZ(), edge2C.GetZ(), edge2D.GetZ());



    Vector4 lineParams, triParam1s, triParam2s, dets;
    Mask4 intersects = TriangleLineSegIntersect(v0x, v0y, v0z,
                                                edge1x, edge1y, edge1z,
                                                edge2x, edge2y, edge2z,
                                                lineStart, lineDelta, dets, 
                                                triParam1s, triParam2s, lineParams);

    RwpBool intersections[4] = {(RwpBool)intersects.GetX().GetBool(), 
                               (RwpBool)intersects.GetY().GetBool(), 
                               (RwpBool)intersects.GetZ().GetBool(), 
                               (RwpBool)intersects.GetW().GetBool()};
    for(int32_t i=0; i<4; i++)
    {
        EATESTAssert(intersections[i] == expectedIntersections[i], "Expected intersection incorrect");
        if(intersections[i])
        {
            EATESTAssert(IsSimilar(lineParams.GetComponent(i), expectedLineParams.GetComponent(i)), "Expected lineparam incorrect");
            EATESTAssert(IsSimilar(triParam1s.GetComponent(i), expectedTri1Params.GetComponent(i)), "Expected triParam2 incorrect");
            EATESTAssert(IsSimilar(triParam2s.GetComponent(i), expectedTri2Params.GetComponent(i)), "Expected triParam1 incorrect");
            EATESTAssert(IsSimilar(dets.GetComponent(i), expectedDets.GetComponent(i)), "Expected lineparam incorrect");
        }
    }
}


void 
TestTriangleQuery::Test4WayNonNormalizedLineTriangleTest()
{

    // ---- Test all intersecting----
    Vector3 v0s[4], v1s[4], v2s[4];
    Vector3 lineStart(4.0f, 1.0f, 0.0f);
    Vector3 lineDelta(0.0f, 0.0f, 10.0f);

    v0s[0] = Vector3(0.0f, 0.0f, 5.0f);
    v1s[0] = Vector3(0.0f, 8.0f, 5.0f);
    v2s[0] = Vector3(8.0f, 0.0f, 5.0f);

    v0s[1] = Vector3(1.0f, 0.0f, 6.0f);
    v1s[1] = Vector3(1.0f, 8.0f, 6.0f);
    v2s[1] = Vector3(8.0f, 0.0f, 6.0f);

    v0s[2] = Vector3(2.0f, 0.0f, 7.0f);
    v1s[2] = Vector3(2.0f, 8.0f, 7.0f);
    v2s[2] = Vector3(8.0f, 0.0f, 7.0f);

    v0s[3] = Vector3(3.0f, 0.0f, 8.0f);
    v1s[3] = Vector3(3.0f, 8.0f, 8.0f);
    v2s[3] = Vector3(8.0f, 0.0f, 8.0f);

    RwpBool expectedIntersections[4] = {true, true, true, true};
    Vector4 expectedLineParams(320.0f, 336.0f, 336.0f, 320.0f);
    Vector4 expectedTri1Params(80.0f, 70.0f, 60.0f, 50.0f);
    Vector4 expectedTri2Params(320.0f, 240.0f, 160.0f, 80.0f);
    Vector4 expectedDets(640.0f, 560.0f, 480.0f, 400.0f);

    Do4WayNormalizedTriangleLineTest(v0s, v1s, v2s, lineStart, lineDelta, expectedIntersections, 
                                     expectedLineParams, expectedTri1Params, expectedTri2Params,
                                     expectedDets);

    //Go through each triangle making it fail the test in a variety of ways
    for(uint32_t i=0 ; i<4; i++)
    {
        Vector3 v0Original = v0s[i];
        Vector3 v1Original = v1s[i];
        Vector3 v2Original = v2s[i];

        // ---- not intersecting (line too short)----
        expectedIntersections[i] = false;
        v0s[i] = Vector3(0.0f, 0.0f, 11.0f);
        v1s[i] = Vector3(0.0f, 8.0f, 11.0f);
        v2s[i] = Vector3(8.0f, 0.0f, 11.0f);

        Do4WayNormalizedTriangleLineTest(v0s, v1s, v2s, lineStart, lineDelta, expectedIntersections, 
            expectedLineParams, expectedTri1Params, expectedTri2Params,
            expectedDets);



        // ---- not intersecting (beyond edge 1)----
        expectedIntersections[i] = false;
        v0s[i] = Vector3(0.0f, 0.0f, 5.0f);
        v1s[i] = Vector3(0.0f, 0.5f, 5.0f);
        v2s[i] = Vector3(8.0f, 0.0f, 5.0f);

        Do4WayNormalizedTriangleLineTest(v0s, v1s, v2s, lineStart, lineDelta, expectedIntersections, 
            expectedLineParams, expectedTri1Params, expectedTri2Params,
            expectedDets);

        // ---- not intersecting (beyond edge 2)----
        expectedIntersections[i] = false;
        v0s[i] = Vector3(0.0f, 0.0f, 5.0f);
        v1s[i] = Vector3(0.0f, 1.0f, 5.0f);
        v2s[i] = Vector3(0.5f, 0.0f, 5.0f);

        Do4WayNormalizedTriangleLineTest(v0s, v1s, v2s, lineStart, lineDelta, expectedIntersections, 
            expectedLineParams, expectedTri1Params, expectedTri2Params,
            expectedDets);

        // ---- not intersecting (beyond edge 3)----
        expectedIntersections[i] = false;
        v0s[i] = Vector3(0.0f, 0.0f, 5.0f);
        v1s[i] = Vector3(0.0f, -1.0f, 5.0f);
        v2s[i] = Vector3(5.0f, 0.0f, 5.0f);

        Do4WayNormalizedTriangleLineTest(v0s, v1s, v2s, lineStart, lineDelta, expectedIntersections, 
            expectedLineParams, expectedTri1Params, expectedTri2Params,
            expectedDets);

        //reset this triangle
        expectedIntersections[i] = true;
        v0s[i] = v0Original;
        v1s[i] = v1Original;
        v2s[i] = v2Original;
    }

}


void 
TestTriangleQuery::Do4WayNormalizedTriangleLineTest(Vector3 * v0s, Vector3 * v1s, Vector3 * v2s, 
                                                    Vector3::InParam lineStart, Vector3::InParam lineDelta,
                                                    RwpBool * expectedIntersections, Vector3 * expectedPositions,
                                                    Vector4::InParam expectedLineParams, Vector3 * expectedTriParams)
{
    Vector3 positions[4];
    Vector3 triParams[4];
    Vector4 lineParams;

    Mask4 intersects = TriangleLineSegIntersect(v0s[0], v1s[0], v2s[0],
                                                v0s[1], v1s[1], v2s[1],
                                                v0s[2], v1s[2], v2s[2],
                                                v0s[3], v1s[3], v2s[3],
                                                lineStart, lineDelta, 
                                                positions[0], positions[1], positions[2], positions[3], lineParams,
                                                triParams[0], triParams[1], triParams[2], triParams[3]);

    RwpBool intersections[4] = {(RwpBool)intersects.GetX().GetBool(), 
                               (RwpBool)intersects.GetY().GetBool(), 
                               (RwpBool)intersects.GetZ().GetBool(), 
                               (RwpBool)intersects.GetW().GetBool()};

    for(int32_t i=0; i<4; i++)
    {
        EATESTAssert(intersections[i] == expectedIntersections[i], "Expected intersection incorrect");
        if(intersections[i])
        {
            EATESTAssert(IsSimilar(positions[i], expectedPositions[i]), "Expected position incorrect");
            EATESTAssert(IsSimilar(lineParams[i], expectedLineParams[i]), "Expected line param incorrect");
            EATESTAssert(IsSimilar(triParams[i], expectedTriParams[i]), "Expected tri param incorrect");
        }
    }
}

void 
TestTriangleQuery::Test4WayNormalizedLineTriangleTest()
{

    // ---- Test all intersecting----
    Vector3 v0s[4], v1s[4], v2s[4];

    v0s[0] = Vector3(0.0f, 0.0f, 5.0f);
    v1s[0] = Vector3(0.0f, 8.0f, 5.0f);
    v2s[0] = Vector3(8.0f, 0.0f, 5.0f);

    v0s[1] = Vector3(1.0f, 0.0f, 6.0f);
    v1s[1] = Vector3(1.0f, 8.0f, 6.0f);
    v2s[1] = Vector3(8.0f, 0.0f, 6.0f);

    v0s[2] = Vector3(2.0f, 0.0f, 7.0f);
    v1s[2] = Vector3(2.0f, 8.0f, 7.0f);
    v2s[2] = Vector3(8.0f, 0.0f, 7.0f);

    v0s[3] = Vector3(3.0f, 0.0f, 8.0f);
    v1s[3] = Vector3(3.0f, 8.0f, 8.0f);
    v2s[3] = Vector3(8.0f, 0.0f, 8.0f);

    Vector3 lineStart(4.0f, 1.0f, 0.0f);
    Vector3 lineDelta(0.0f, 0.0f, 10.0f);

    RwpBool expectedIntersections[4] = {true, true, true, true};

    Vector3 expectedPositions[4] = {Vector3(4.0f, 1.0f, 5.0f),
                                    Vector3(4.0f, 1.0f, 6.0f),
                                    Vector3(4.0f, 1.0f, 7.0f),
                                    Vector3(4.0f, 1.0f, 8.0f)};

    Vector4 expectedLineParams(0.5f, 0.6f, 0.7f, 0.8f);


    Vector3 expectedTriParams[4] = {Vector3(0.125f, 0.5f, 0.0f),
                                    Vector3(0.125f, 0.428571f, 0.0f),
                                    Vector3(0.125f, 0.333333f, 0.0f),
                                    Vector3(0.125f, 0.2f, 0.0f)};

    Do4WayNormalizedTriangleLineTest(v0s, v1s, v2s, lineStart, lineDelta, expectedIntersections, 
                                     expectedPositions, expectedLineParams, expectedTriParams);

    //Go through each triangle making it fail the test in a variety of ways
    for(uint32_t i=0 ; i<4; i++)
    {
        Vector3 v0Original = v0s[i];
        Vector3 v1Original = v1s[i];
        Vector3 v2Original = v2s[i];

        // ---- not intersecting (line too short)----
        expectedIntersections[i] = false;
        v0s[i] = Vector3(0.0f, 0.0f, 11.0f);
        v1s[i] = Vector3(0.0f, 8.0f, 11.0f);
        v2s[i] = Vector3(8.0f, 0.0f, 11.0f);

        Do4WayNormalizedTriangleLineTest(v0s, v1s, v2s, lineStart, lineDelta, expectedIntersections, 
            expectedPositions, expectedLineParams, expectedTriParams);



        // ---- not intersecting (beyond edge 1)----
        expectedIntersections[i] = false;
        v0s[i] = Vector3(0.0f, 0.0f, 5.0f);
        v1s[i] = Vector3(0.0f, 0.5f, 5.0f);
        v2s[i] = Vector3(8.0f, 0.0f, 5.0f);

        Do4WayNormalizedTriangleLineTest(v0s, v1s, v2s, lineStart, lineDelta, expectedIntersections, 
            expectedPositions, expectedLineParams, expectedTriParams);

        // ---- not intersecting (beyond edge 2)----
        expectedIntersections[i] = false;
        v0s[i] = Vector3(0.0f, 0.0f, 5.0f);
        v1s[i] = Vector3(0.0f, 1.0f, 5.0f);
        v2s[i] = Vector3(0.5f, 0.0f, 5.0f);

        Do4WayNormalizedTriangleLineTest(v0s, v1s, v2s, lineStart, lineDelta, expectedIntersections, 
            expectedPositions, expectedLineParams, expectedTriParams);

        // ---- not intersecting (beyond edge 3)----
        expectedIntersections[i] = false;
        v0s[i] = Vector3(0.0f, 0.0f, 5.0f);
        v1s[i] = Vector3(0.0f, -1.0f, 5.0f);
        v2s[i] = Vector3(5.0f, 0.0f, 5.0f);

        Do4WayNormalizedTriangleLineTest(v0s, v1s, v2s, lineStart, lineDelta, expectedIntersections, 
            expectedPositions, expectedLineParams, expectedTriParams);

        //reset this triangle
        expectedIntersections[i] = true;
        v0s[i] = v0Original;
        v1s[i] = v1Original;
        v2s[i] = v2Original;
    }

}


void 
TestTriangleQuery::Do16WayNormalizedTriangleLineTest(Vector3 * v0s, Vector3 * v1s, Vector3 * v2s, 
                                                     Vector3::InParam lineStart, Vector3::InParam lineDelta,
                                                     RwpBool * expectedIntersections, Vector3 * expectedPositions,
                                                     VecFloat* expectedLineParams, Vector3 * expectedTriParams)
{
    TriangleQuery triangleQueries[16];
    for(uint32_t i=0; i<16; i++)
    {
        triangleQueries[i].v0 = v0s[i];
        triangleQueries[i].v1 = v1s[i];
        triangleQueries[i].v2 = v2s[i];
    }

    TriangleLineSegIntersect(triangleQueries[0],
                             triangleQueries[1],
                             triangleQueries[2],
                             triangleQueries[3],
                             triangleQueries[4],
                             triangleQueries[5],
                             triangleQueries[6],
                             triangleQueries[7],
                             triangleQueries[8],
                             triangleQueries[9],
                             triangleQueries[10],
                             triangleQueries[11],
                             triangleQueries[12],
                             triangleQueries[13],
                             triangleQueries[14],
                             triangleQueries[15],
                             lineStart, lineDelta);



    for(uint32_t i=0; i<16; i++)
    {
        EATESTAssert((RwpBool)triangleQueries[i].intersects.GetBool() == expectedIntersections[i], "Expected intersection incorrect");
        if(triangleQueries[i].intersects.GetBool())
        {
            EATESTAssert(IsSimilar(triangleQueries[i].position, expectedPositions[i]), "Expected position incorrect");
            EATESTAssert(IsSimilar(triangleQueries[i].lineParam, expectedLineParams[i]), "Expected line param incorrect");
            EATESTAssert(IsSimilar(triangleQueries[i].triParam, expectedTriParams[i]), "Expected tri param incorrect");
        }
    }
}


void 
TestTriangleQuery::Test16WayNormalizedLineTriangleTest()
{

    // ---- Test all intersecting----
    Vector3 v0s[16], v1s[16], v2s[16];
    Vector3 lineStart(4.0f, 1.0f, 0.0f);
    Vector3 lineDelta(0.0f, 0.0f, 16.0f);


    RwpBool expectedIntersections[16] = {true, true, true, true,
                                        true, true, true, true,
                                        true, true, true, true,
                                        true, true, true, true};
    Vector3 expectedPositions[16];
    Vector3 expectedTriParams[16];
    VecFloat expectedLineParams[16];

    for(uint32_t i=0; i<16; i++)
    {
        v0s[i] = Vector3(0.0f, 0.0f, (float)i);
        v1s[i] = Vector3(0.0f, 20.0f, (float)i);
        v2s[i] = Vector3(20.0f, 0.0f, (float)i);   

        expectedPositions[i] = Vector3(4.0f, 1.0f, (float)i);  
        expectedLineParams[i] = VecFloat(((float) i)/16.0f);
        expectedTriParams[i]  = Vector3(0.05f, 0.2f, 0.0f);
    }


    Do16WayNormalizedTriangleLineTest(v0s, v1s, v2s, lineStart, lineDelta, expectedIntersections, 
                                      expectedPositions, expectedLineParams, expectedTriParams);

    //Go through each triangle making it fail the test in a variety of ways
    for(uint32_t i=0 ; i<16; i++)
    {
        Vector3 v0Original = v0s[i];
        Vector3 v1Original = v1s[i];
        Vector3 v2Original = v2s[i];

        // ---- not intersecting (line too short)----
        expectedIntersections[i] = false;
        v0s[i] = Vector3(0.0f, 0.0f, 17.0f);
        v1s[i] = Vector3(0.0f, 8.0f, 17.0f);
        v2s[i] = Vector3(8.0f, 0.0f, 17.0f);

        Do16WayNormalizedTriangleLineTest(v0s, v1s, v2s, lineStart, lineDelta, expectedIntersections, 
            expectedPositions, expectedLineParams, expectedTriParams);

        // ---- not intersecting (beyond edge 1)----
        expectedIntersections[i] = false;
        v0s[i] = Vector3(0.0f, 0.0f, 5.0f);
        v1s[i] = Vector3(0.0f, 0.5f, 5.0f);
        v2s[i] = Vector3(8.0f, 0.0f, 5.0f);

        Do16WayNormalizedTriangleLineTest(v0s, v1s, v2s, lineStart, lineDelta, expectedIntersections, 
            expectedPositions, expectedLineParams, expectedTriParams);

        // ---- not intersecting (beyond edge 2)----
        expectedIntersections[i] = false;
        v0s[i] = Vector3(0.0f, 0.0f, 5.0f);
        v1s[i] = Vector3(0.0f, 1.0f, 5.0f);
        v2s[i] = Vector3(0.5f, 0.0f, 5.0f);

        Do16WayNormalizedTriangleLineTest(v0s, v1s, v2s, lineStart, lineDelta, expectedIntersections, 
            expectedPositions, expectedLineParams, expectedTriParams);

        // ---- not intersecting (beyond edge 3)----
        expectedIntersections[i] = false;
        v0s[i] = Vector3(0.0f, 0.0f, 5.0f);
        v1s[i] = Vector3(0.0f, -1.0f, 5.0f);
        v2s[i] = Vector3(5.0f, 0.0f, 5.0f);

        Do16WayNormalizedTriangleLineTest(v0s, v1s, v2s, lineStart, lineDelta, expectedIntersections, 
            expectedPositions, expectedLineParams, expectedTriParams);

        //reset this triangle
        expectedIntersections[i] = true;
        v0s[i] = v0Original;
        v1s[i] = v1Original;
        v2s[i] = v2Original;
    }
}


#if 0 // TODO: Wii
void 
TestTriangleQuery::Do16WayNonNormalizedTriangleLineTest(Vector3 * v0s, Vector3 * v1s, Vector3 * v2s, 
                                                        Vector3::InParam lineStart, Vector3::InParam lineDelta,
                                                        RwpBool * expectedIntersections, VecFloat * expectedLineParams,
                                                        VecFloat* expectedTri1Params, VecFloat* expectedTri2Params,
                                                        VecFloat* expectedDets)
{
    //Transpose4to3(v0A, v0B, v0C, v0D, v0x, v0y, v0z);
    rwpmath::Vector4 v0x0 = rwpmath::Vector4(v0s[0].GetX(),  v0s[1].GetX(),  v0s[2].GetX(),  v0s[3].GetX());
    rwpmath::Vector4 v0x1 = rwpmath::Vector4(v0s[4].GetX(),  v0s[5].GetX(),  v0s[6].GetX(),  v0s[7].GetX());
    rwpmath::Vector4 v0x2 = rwpmath::Vector4(v0s[8].GetX(),  v0s[9].GetX(),  v0s[10].GetX(), v0s[11].GetX());
    rwpmath::Vector4 v0x3 = rwpmath::Vector4(v0s[12].GetX(), v0s[13].GetX(), v0s[14].GetX(), v0s[15].GetX());

    rwpmath::Vector4 v0y0 = rwpmath::Vector4(v0s[0].GetY(),  v0s[1].GetY(),  v0s[2].GetY(),  v0s[3].GetY());
    rwpmath::Vector4 v0y1 = rwpmath::Vector4(v0s[4].GetY(),  v0s[5].GetY(),  v0s[6].GetY(),  v0s[7].GetY());
    rwpmath::Vector4 v0y2 = rwpmath::Vector4(v0s[8].GetY(),  v0s[9].GetY(),  v0s[10].GetY(), v0s[11].GetY());
    rwpmath::Vector4 v0y3 = rwpmath::Vector4(v0s[12].GetY(), v0s[13].GetY(), v0s[14].GetY(), v0s[15].GetY());

    rwpmath::Vector4 v0z0 = rwpmath::Vector4(v0s[0].GetZ(),  v0s[1].GetZ(),  v0s[2].GetZ(),  v0s[3].GetZ());
    rwpmath::Vector4 v0z1 = rwpmath::Vector4(v0s[4].GetZ(),  v0s[5].GetZ(),  v0s[6].GetZ(),  v0s[7].GetZ());
    rwpmath::Vector4 v0z2 = rwpmath::Vector4(v0s[8].GetZ(),  v0s[9].GetZ(),  v0s[10].GetZ(), v0s[11].GetZ());
    rwpmath::Vector4 v0z3 = rwpmath::Vector4(v0s[12].GetZ(), v0s[13].GetZ(), v0s[14].GetZ(), v0s[15].GetZ());

    // Find vectors for the two edges sharing v0
    rwpmath::Vector3 edge1A = v1s[0] - v0s[0];
    rwpmath::Vector3 edge1B = v1s[1] - v0s[1];
    rwpmath::Vector3 edge1C = v1s[2] - v0s[2];
    rwpmath::Vector3 edge1D = v1s[3] - v0s[3];
    rwpmath::Vector3 edge1E = v1s[4] - v0s[4];
    rwpmath::Vector3 edge1F = v1s[5] - v0s[5];
    rwpmath::Vector3 edge1G = v1s[6] - v0s[6];
    rwpmath::Vector3 edge1H = v1s[7] - v0s[7];
    rwpmath::Vector3 edge1I = v1s[8] - v0s[8];
    rwpmath::Vector3 edge1J = v1s[9] - v0s[9];
    rwpmath::Vector3 edge1K = v1s[10] - v0s[10];
    rwpmath::Vector3 edge1L = v1s[11] - v0s[11];
    rwpmath::Vector3 edge1M = v1s[12] - v0s[12];
    rwpmath::Vector3 edge1N = v1s[13] - v0s[13];
    rwpmath::Vector3 edge1O = v1s[14] - v0s[14];
    rwpmath::Vector3 edge1P = v1s[15] - v0s[15];

    rwpmath::Matrix44 mtx0 = rwpmath::Matrix44(edge1A, edge1B, edge1C, edge1D);
    rwpmath::Matrix44 mtx1 = rwpmath::Matrix44(edge1E, edge1F, edge1G, edge1H);
    rwpmath::Matrix44 mtx2 = rwpmath::Matrix44(edge1I, edge1J, edge1K, edge1L);
    rwpmath::Matrix44 mtx3 = rwpmath::Matrix44(edge1M, edge1N, edge1O, edge1P);
    mtx0 = rwpmath::Transpose(mtx0);
    mtx1 = rwpmath::Transpose(mtx1);
    mtx2 = rwpmath::Transpose(mtx2);
    mtx3 = rwpmath::Transpose(mtx3);

    rwpmath::Vector4 edge1x0(mtx0.GetX());
    rwpmath::Vector4 edge1x1(mtx1.GetX());
    rwpmath::Vector4 edge1x2(mtx2.GetX());
    rwpmath::Vector4 edge1x3(mtx3.GetX());

    rwpmath::Vector4 edge1y0(mtx0.GetY());
    rwpmath::Vector4 edge1y1(mtx1.GetY());
    rwpmath::Vector4 edge1y2(mtx2.GetY());
    rwpmath::Vector4 edge1y3(mtx3.GetY());

    rwpmath::Vector4 edge1z0(mtx0.GetZ());
    rwpmath::Vector4 edge1z1(mtx1.GetZ());
    rwpmath::Vector4 edge1z2(mtx2.GetZ());
    rwpmath::Vector4 edge1z3(mtx3.GetZ());

    rwpmath::Vector3 edge2A = v2s[0] - v0s[0];
    rwpmath::Vector3 edge2B = v2s[1] - v0s[1];
    rwpmath::Vector3 edge2C = v2s[2] - v0s[2];
    rwpmath::Vector3 edge2D = v2s[3] - v0s[3];
    rwpmath::Vector3 edge2E = v2s[4] - v0s[4];
    rwpmath::Vector3 edge2F = v2s[5] - v0s[5];
    rwpmath::Vector3 edge2G = v2s[6] - v0s[6];
    rwpmath::Vector3 edge2H = v2s[7] - v0s[7];
    rwpmath::Vector3 edge2I = v2s[8] - v0s[8];
    rwpmath::Vector3 edge2J = v2s[9] - v0s[9];
    rwpmath::Vector3 edge2K = v2s[10] - v0s[10];
    rwpmath::Vector3 edge2L = v2s[11] - v0s[11];
    rwpmath::Vector3 edge2M = v2s[12] - v0s[12];
    rwpmath::Vector3 edge2N = v2s[13] - v0s[13];
    rwpmath::Vector3 edge2O = v2s[14] - v0s[14];
    rwpmath::Vector3 edge2P = v2s[15] - v0s[15];

    rwpmath::Vector4 edge2x0 = rwpmath::Vector4(edge2A.GetX(), edge2B.GetX(), edge2C.GetX(), edge2D.GetX());
    rwpmath::Vector4 edge2x1 = rwpmath::Vector4(edge2E.GetX(), edge2F.GetX(), edge2G.GetX(), edge2H.GetX());
    rwpmath::Vector4 edge2x2 = rwpmath::Vector4(edge2I.GetX(), edge2J.GetX(), edge2K.GetX(), edge2L.GetX());
    rwpmath::Vector4 edge2x3 = rwpmath::Vector4(edge2M.GetX(), edge2N.GetX(), edge2O.GetX(), edge2P.GetX());

    rwpmath::Vector4 edge2y0 = rwpmath::Vector4(edge2A.GetY(), edge2B.GetY(), edge2C.GetY(), edge2D.GetY());
    rwpmath::Vector4 edge2y1 = rwpmath::Vector4(edge2E.GetY(), edge2F.GetY(), edge2G.GetY(), edge2H.GetY());
    rwpmath::Vector4 edge2y2 = rwpmath::Vector4(edge2I.GetY(), edge2J.GetY(), edge2K.GetY(), edge2L.GetY());
    rwpmath::Vector4 edge2y3 = rwpmath::Vector4(edge2M.GetY(), edge2N.GetY(), edge2O.GetY(), edge2P.GetY());

    rwpmath::Vector4 edge2z0 = rwpmath::Vector4(edge2A.GetZ(), edge2B.GetZ(), edge2C.GetZ(), edge2D.GetZ());
    rwpmath::Vector4 edge2z1 = rwpmath::Vector4(edge2E.GetZ(), edge2F.GetZ(), edge2G.GetZ(), edge2H.GetZ());
    rwpmath::Vector4 edge2z2 = rwpmath::Vector4(edge2I.GetZ(), edge2J.GetZ(), edge2K.GetZ(), edge2L.GetZ());
    rwpmath::Vector4 edge2z3 = rwpmath::Vector4(edge2M.GetZ(), edge2N.GetZ(), edge2O.GetZ(), edge2P.GetZ());

    rwpmath::Vector4 dets[4];
    rwpmath::Vector4 tri1Params[4];
    rwpmath::Vector4 tri2Params[4];
    rwpmath::Vector4 lineParams[4];
    rwpmath::Mask4 resultsValid[4];

    // test for intersection - returns unnormalized parameters
    TriangleLineSegIntersect(v0x0, v0y0, v0z0,
        v0x1, v0y1, v0z1,
        v0x2, v0y2, v0z2,
        v0x3, v0y3, v0z3, 

        edge1x0, edge1y0, edge1z0, 
        edge1x1, edge1y1, edge1z1, 
        edge1x2, edge1y2, edge1z2, 
        edge1x3, edge1y3, edge1z3,  

        edge2x0, edge2y0, edge2z0, 
        edge2x1, edge2y1, edge2z1, 
        edge2x2, edge2y2, edge2z2, 
        edge2x3, edge2y3, edge2z3, 

        lineStart, lineDelta,
        dets[0], dets[1], dets[2], dets[3],
        tri1Params[0], tri1Params[1], tri1Params[2], tri1Params[3],
        tri2Params[0], tri2Params[1], tri2Params[2], tri2Params[3],
        lineParams[0], lineParams[1], lineParams[2], lineParams[3],
        resultsValid[0], resultsValid[1], resultsValid[2], resultsValid[3]);

    RwpBool intersecting[16];

    for(uint32_t i=0; i<4; i++)
    {
        intersecting[(i*4)] = resultsValid[i].GetX().GetBool();
        intersecting[(i*4)+1] = resultsValid[i].GetY().GetBool();
        intersecting[(i*4)+2] = resultsValid[i].GetZ().GetBool();
        intersecting[(i*4)+3] = resultsValid[i].GetW().GetBool();
    }

    for(uint32_t i=0; i<16; i++)
    {
        uint32_t group = i/4;
        uint32_t component = i%4;

        EATESTAssert(intersecting[i] == expectedIntersections[i], "Expected intersection incorrect");
        if(intersecting[i])
        {
            EATESTAssert(IsSimilar((VecFloat)lineParams[group].GetComponent(component), (VecFloat)expectedLineParams[i]), "Expected line param incorrect");
            EATESTAssert(IsSimilar((VecFloat)tri1Params[group].GetComponent(component), (VecFloat)expectedTri1Params[i]), "Expected tri1 incorrect");
            EATESTAssert(IsSimilar((VecFloat)tri2Params[group].GetComponent(component), (VecFloat)expectedTri2Params[i]), "Expected tri2 incorrect");
            EATESTAssert(IsSimilar((VecFloat)dets[group].GetComponent(component), (VecFloat)expectedDets[i]), "Expected det incorrect");
        }
    }
}
#endif


void 
TestTriangleQuery::Test16WayNonNormalizedLineTriangleTest()
{
#if 0 // TODO: Wii
    // ---- Test all intersecting----
    Vector3 v0s[16], v1s[16], v2s[16];
    Vector3 lineStart(4.0f, 1.0f, 0.0f);
    Vector3 lineDelta(0.0f, 0.0f, 16.0f);


    RwpBool expectedIntersections[16] = {true, true, true, true,
        true, true, true, true,
        true, true, true, true,
        true, true, true, true};
    VecFloat expectedTri1Params[16];
    VecFloat expectedTri2Params[16];
    VecFloat expectedLineParams[16];
    VecFloat expectedDets[16];

    for(uint32_t i=0; i<16; i++)
    {
        v0s[i] = Vector3(0.0f, 0.0f, (float)i);
        v1s[i] = Vector3(0.0f, 20.0f, (float)i);
        v2s[i] = Vector3(20.0f, 0.0f, (float)i);   

        expectedLineParams[i] = (float) i * 400.0f;
        expectedTri1Params[i] = 320.0f;
        expectedTri2Params[i] = 1280.0f;
        expectedDets[i] = 6400;
    }

    Do16WayNonNormalizedTriangleLineTest(v0s, v1s, v2s, lineStart, lineDelta, expectedIntersections, expectedLineParams, 
                                         expectedTri1Params, expectedTri2Params, expectedDets);

    //Go through each triangle making it fail the test in a variety of ways
    for(uint32_t i=0 ; i<16; i++)
    {
        Vector3 v0Original = v0s[i];
        Vector3 v1Original = v1s[i];
        Vector3 v2Original = v2s[i];

        // ---- not intersecting (line too short)----
        expectedIntersections[i] = false;
        v0s[i] = Vector3(0.0f, 0.0f, 17.0f);
        v1s[i] = Vector3(0.0f, 8.0f, 17.0f);
        v2s[i] = Vector3(8.0f, 0.0f, 17.0f);

        Do16WayNonNormalizedTriangleLineTest(v0s, v1s, v2s, lineStart, lineDelta, expectedIntersections, expectedLineParams, 
                                             expectedTri1Params, expectedTri2Params, expectedDets);

        // ---- not intersecting (beyond edge 1)----
        expectedIntersections[i] = false;
        v0s[i] = Vector3(0.0f, 0.0f, 5.0f);
        v1s[i] = Vector3(0.0f, 0.5f, 5.0f);
        v2s[i] = Vector3(8.0f, 0.0f, 5.0f);

        Do16WayNonNormalizedTriangleLineTest(v0s, v1s, v2s, lineStart, lineDelta, expectedIntersections, expectedLineParams, 
                                             expectedTri1Params, expectedTri2Params, expectedDets);

        // ---- not intersecting (beyond edge 2)----
        expectedIntersections[i] = false;
        v0s[i] = Vector3(0.0f, 0.0f, 5.0f);
        v1s[i] = Vector3(0.0f, 1.0f, 5.0f);
        v2s[i] = Vector3(0.5f, 0.0f, 5.0f);

        Do16WayNonNormalizedTriangleLineTest(v0s, v1s, v2s, lineStart, lineDelta, expectedIntersections, expectedLineParams, 
                                             expectedTri1Params, expectedTri2Params, expectedDets);

        // ---- not intersecting (beyond edge 3)----
        expectedIntersections[i] = false;
        v0s[i] = Vector3(0.0f, 0.0f, 5.0f);
        v1s[i] = Vector3(0.0f, -1.0f, 5.0f);
        v2s[i] = Vector3(5.0f, 0.0f, 5.0f);

        Do16WayNonNormalizedTriangleLineTest(v0s, v1s, v2s, lineStart, lineDelta, expectedIntersections, expectedLineParams, 
                                             expectedTri1Params, expectedTri2Params, expectedDets);

        //reset this triangle
        expectedIntersections[i] = true;
        v0s[i] = v0Original;
        v1s[i] = v1Original;
        v2s[i] = v2Original;
    }
#endif
}
