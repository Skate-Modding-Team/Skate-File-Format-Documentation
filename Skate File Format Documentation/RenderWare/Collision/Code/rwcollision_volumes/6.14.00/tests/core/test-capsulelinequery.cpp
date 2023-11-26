// (c) Electronic Arts. All Rights Reserved.

#include <unit/unit.h>

#include <EABase/eabase.h>
#include <eaphysics/base.h>
#include <rw/collision/libcore.h>

#include <eaphysics/unitframework/allocator.h> // For ResetAllocator
#include <eaphysics/unitframework/creator.h> // For Creator

#include "testsuitebase.h" // For TestSuiteBase

#define DISABLE_KNOWN_FAILING_UNITTESTS

using namespace rwpmath;
using namespace rw::collision;

static const float POSITION_TOLERANCE = 1e-4f;
static const float NORMAL_TOLERANCE = 1e-5f;
static const float LINEPARAM_TOLERANCE = 1e-5f;

class TestCapsuleLineQuery: public tests::TestSuiteBase
{
public:
    virtual void Initialize()
    {
        SuiteName("TestCapsuleLineQuery");

        EATEST_REGISTER( "TestLineSegIntersections",  "Testing the LineSegIntersect method",  TestCapsuleLineQuery, TestLineSegIntersections   );
#if !defined(DISABLE_KNOWN_FAILING_UNITTESTS)
        EATEST_REGISTER( "TestrwcCylinderLineSegIntersect",  "Testing the rwcCylinderLineSegIntersect method",  TestCapsuleLineQuery, TestrwcCylinderLineSegIntersect );
#endif
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


    static CapsuleVolume * CreateCapsuleVolume()
    {
        // Capsule with 5.0 halfheight and 1.0 radius
        const float halfHeight = 5.0f;
        const float radius = 1.0f;
        return EA::Physics::UnitFramework::Creator<CapsuleVolume>().New(radius, halfHeight);
    }

    static bool CompareCapsuleVolumes(const CapsuleVolume& original, const CapsuleVolume& copied)
    {
        return (original.GetHalfHeight() == copied.GetHalfHeight()) &&
               (original.GetRadius() == copied.GetRadius());
    }


private:
    void TestLineSegIntersections();
    void TestLineSegIntersectionsWithMatricies( CapsuleVolume* );
    void LineSegIntersectionScenarios( CapsuleVolume*, Matrix44Affine* );
    void TestLineSegIntersect( CapsuleVolume*, VolumeLineSegIntersectResult&, Vector3&, Vector3&, RwpBool , Matrix44Affine*, float );
    void TestrwcCylinderLineSegIntersect();

} TestCapsuleLineQuerySingleton;

// Testing the LineSegIntersect method - called by LineSegIntersectionScenarios
void TestCapsuleLineQuery::TestLineSegIntersect( CapsuleVolume* p_Capsule, VolumeLineSegIntersectResult &expectedResult, Vector3 &lineStart, 
                                       Vector3 &lineEnd, RwpBool shouldHit, Matrix44Affine* mtx, float fatness)
{
    VolumeLineSegIntersectResult result;
    Vector3 LineStart = lineStart;
    Vector3 LineEnd = lineEnd;
    VolumeLineSegIntersectResult expRes = expectedResult;

    if ( mtx )
    {
        LineStart = TransformPoint(LineStart, *mtx);
        LineEnd = TransformPoint(LineEnd, *mtx);
        if ( shouldHit )
        {
            expRes.position = TransformPoint(expRes.position, *mtx);
            expRes.normal = TransformVector( expRes.normal, *mtx);
        }
    }

    RwpBool hit = p_Capsule->LineSegIntersect( LineStart, LineEnd, mtx, result, fatness );

    EATESTAssert( shouldHit == hit, "Test LineSegIntersect - shouldHit" );

    if ( shouldHit )
    {
        EATESTAssert(rwpmath::IsSimilar(expRes.position, result.position, POSITION_TOLERANCE), "Test LineSegIntersect - position" );
        EATESTAssert(rwpmath::IsSimilar(expRes.normal, result.normal, NORMAL_TOLERANCE), "Test LineSegIntersect - normal" );
        EATESTAssert(rwpmath::IsSimilar(expRes.lineParam, result.lineParam, LINEPARAM_TOLERANCE), "Test LineSegIntersect - lineParam" );
    }
}

// Testing the LineSegIntersect method - called by TestLineSegIntersectionsWithMatrices, this method holds all of the
// line volume intersection scenarios
void TestCapsuleLineQuery::LineSegIntersectionScenarios( CapsuleVolume* p_Capsule, Matrix44Affine* mtx ){

    // Capsules HalfHeight and Radius
    float HH = p_Capsule->GetHalfHeight();
    float RADIUS = p_Capsule->GetRadius();

    // Fat value
    const float FAT = 1.0f;
    const float THIN = 0.0f;
    const float EPS = 0.01f;

    VolumeLineSegIntersectResult expectedResult;
    RwpBool shouldHit = true;
    Vector3 lineStart;
    Vector3 lineEnd;

    // Test 1 - line which intersects capsule body from outside capsule
    shouldHit = true;
    // Thin line vs Capsule
    {
        lineStart = Vector3( RADIUS + RADIUS/2.0f, 0.0f, 0.0f );
        lineEnd = Vector3( RADIUS - RADIUS/2.0f, 0.0f, 0.0f );
        expectedResult.position = Vector3( RADIUS, 0.0f, 0.0f );
        expectedResult.normal = Vector3( 1.0f, 0.0f, 0.0f );
        expectedResult.lineParam = 0.5f;
        TestLineSegIntersect( p_Capsule, expectedResult, lineStart, lineEnd, shouldHit, mtx, THIN );
    }
    // Fat-line vs Capsule
    {
        lineStart = Vector3( RADIUS + FAT + RADIUS/2.0f, 0.0f, 0.0f );
        lineEnd = Vector3( RADIUS + FAT - RADIUS/2.0f, 0.0f, 0.0f );
        expectedResult.position = Vector3( RADIUS, 0.0f, 0.0f );
        expectedResult.normal = Vector3( 1.0f, 0.0f, 0.0f );
        expectedResult.lineParam = 0.5f;
        TestLineSegIntersect( p_Capsule, expectedResult, lineStart, lineEnd, shouldHit, mtx, FAT );
    }

    // Test 2 - line which intersects capsule body from inside capsule
    shouldHit = true;
    // Thin Capsule
    {
        lineStart = Vector3( RADIUS - RADIUS/2.0f, 0.0f, 0.0f );
        lineEnd = Vector3( RADIUS + RADIUS/2.0f, 0.0f, 0.0f );
        expectedResult.position = lineStart;
        expectedResult.normal = Vector3( 1.0f, 0.0f, 0.0f );
        expectedResult.lineParam = 0.0f;
        TestLineSegIntersect( p_Capsule, expectedResult, lineStart, lineEnd, shouldHit, mtx, THIN );
    }
    // Fat Capsule
    {
        lineStart = Vector3( FAT + RADIUS - RADIUS/2.0f, 0.0f, 0.0f );
        lineEnd = Vector3( FAT + RADIUS + RADIUS/2.0f, 0.0f, 0.0f );
        expectedResult.position = Vector3( RADIUS - RADIUS/2.0f, 0.0f, 0.0f );
        expectedResult.normal = Vector3( 1.0f, 0.0f, 0.0f );
        expectedResult.lineParam = 0.0f;
        TestLineSegIntersect( p_Capsule, expectedResult, lineStart, lineEnd, shouldHit, mtx, FAT );
    }

#if !defined(DISABLE_KNOWN_FAILING_UNITTESTS)
    // Test 3 - line which lies inside capsule body
    shouldHit = true;
    // Fat and Thin Capsule
    {
        lineStart = Vector3( 0.0f, 0.0f, -HH );
        lineEnd = Vector3( 0.0f, 0.0f, HH );
        expectedResult.position = Vector3( 0.0f, 0.0f, 0.0f);
        expectedResult.normal = Vector3( 0.0f, 0.0f, -1.0f );
        expectedResult.lineParam = 0.0f;
        // Fails because line start is exactly coincident with end of capsule axis - giving undefined normal/position
        TestLineSegIntersect( p_Capsule, expectedResult, lineStart, lineEnd, shouldHit, mtx, THIN );
        expectedResult.position = Vector3( 0.0f, 0.0f, - HH - RADIUS);
        TestLineSegIntersect( p_Capsule, expectedResult, lineStart, lineEnd, shouldHit, mtx, FAT );
    }
#endif /* !defined(DISABLE_KNOWN_FAILING_UNITTESTS) */

    // Test 3b - line which lies inside capsule body (a variation that doesn't give undefined results)
    shouldHit = true;
    // Fat and Thin Capsule
    {
        lineStart = Vector3( 0.0f, 0.0f, -HH - RADIUS/2.0f );
        lineEnd = Vector3( 0.0f, 0.0f, HH + RADIUS/2.0f );
        expectedResult.position = lineStart;
        expectedResult.normal = Vector3( 0.0f, 0.0f, -1.0f );
        expectedResult.lineParam = 0.0f;
        // Fails because line start is exactly coincident with end of capsule axis - giving undefined normal/position
        TestLineSegIntersect( p_Capsule, expectedResult, lineStart, lineEnd, shouldHit, mtx, THIN );
        expectedResult.position = Vector3( 0.0f, 0.0f, - HH - RADIUS/2.0f + FAT);
        TestLineSegIntersect( p_Capsule, expectedResult, lineStart, lineEnd, shouldHit, mtx, FAT );
    }

    // Test 4 - line which lies along length of capsule body (just inside)
    shouldHit = true;
    // Thin Capsule
    {
        lineStart = Vector3( RADIUS - EPS, 0.0f, -HH );
        lineEnd = Vector3( RADIUS - EPS , 0.0f, HH );
        expectedResult.position = Vector3( RADIUS - EPS, 0.0f, -HH);
        expectedResult.normal = Vector3( 1.0f, 0.0f, 0.0f );
        expectedResult.lineParam = 0.0f;
        TestLineSegIntersect( p_Capsule, expectedResult, lineStart, lineEnd, shouldHit, mtx, THIN );
    }
    // Fat Capsule
    {
        lineStart = Vector3( RADIUS - EPS + FAT , 0.0f, -HH );
        lineEnd = Vector3( RADIUS - EPS + FAT , 0.0f, HH );
        expectedResult.position = Vector3( RADIUS - EPS, 0.0f, -HH );
        expectedResult.normal = Vector3( 1.0f, 0.0f, 0.0f );
        expectedResult.lineParam = 0.0f;
        TestLineSegIntersect( p_Capsule, expectedResult, lineStart, lineEnd, shouldHit, mtx, FAT );
    }

    // Test 5 - line which is tangential to capsule body
    shouldHit = true;
    // Thin Capsule
    {
        float cosEps = 1.0f - EPS;
        float sinEps = rw::math::fpu::Sqrt(1.0f - cosEps*cosEps);
        lineStart = Vector3( RADIUS*cosEps, RADIUS, 0.0f );
        lineEnd = Vector3( RADIUS*cosEps, -RADIUS, 0.0f );
        expectedResult.position = Vector3( RADIUS*cosEps, RADIUS*sinEps, 0.0f );
        expectedResult.normal = Vector3( cosEps, sinEps, 0.0f);
        expectedResult.lineParam = 0.5f - 0.5f*sinEps;
        TestLineSegIntersect( p_Capsule, expectedResult, lineStart, lineEnd, shouldHit, mtx, THIN );
    
        // Fat Capsule
        lineStart = Vector3( (RADIUS + FAT)*cosEps , RADIUS + FAT, 0.0f );
        lineEnd = Vector3( (RADIUS + FAT)*cosEps , -( RADIUS + FAT ), 0.0f );
        TestLineSegIntersect( p_Capsule, expectedResult, lineStart, lineEnd, shouldHit, mtx, FAT );
    }

    // Test 6 - line which lies parallel to, and outside of capsule body
    shouldHit = false;
    // Thin Capsule
    {
        lineStart = Vector3( RADIUS + 1.0f , 0.0f , HH );
        lineEnd = Vector3( RADIUS + 1.0f , 0.0f , -HH );
        TestLineSegIntersect( p_Capsule, expectedResult, lineStart, lineEnd, shouldHit, mtx, THIN );
    }
    // Fat Capsule
    {
        lineStart = Vector3( RADIUS + FAT + 1.0f, 0.0f , HH );
        lineEnd = Vector3( RADIUS + FAT + 1.0f , 0.0f , -HH );
        TestLineSegIntersect( p_Capsule, expectedResult, lineStart, lineEnd, shouldHit, mtx, FAT );
    }

    // Test 7 - line which lies outside of capsule body and points towards capsule
    shouldHit = false;
    // Thin Capsule
    {
        lineStart = Vector3( RADIUS + 2.0f, 0.0f, 0.0f );
        lineEnd = Vector3( RADIUS + 1.0f, 0.0f, 0.0f );
        TestLineSegIntersect( p_Capsule, expectedResult, lineStart, lineEnd, shouldHit, mtx, THIN );
    }
    // Fat Capsule
    {
        lineStart = Vector3( RADIUS + FAT + 2.0f, 0.0f, 0.0f );
        lineEnd = Vector3( RADIUS + FAT + 1.0f, 0.0f, 0.0f );
        TestLineSegIntersect( p_Capsule, expectedResult, lineStart, lineEnd, shouldHit, mtx, FAT );
    }

    // Test 8 - line which lies outside of capsule body and points away from capsule
    shouldHit = false;
    // Thin Capsule
    {
        lineStart = Vector3( RADIUS + 1.0f, 0.0f, 0.0f );
        lineEnd = Vector3( RADIUS + 2.0f, 0.0f, 0.0f );
        TestLineSegIntersect( p_Capsule, expectedResult, lineStart, lineEnd, shouldHit, mtx, THIN );
    }
    // Fat Capsule
    {
        lineStart = Vector3( RADIUS + FAT + 1.0f, 0.0f, 0.0f );
        lineEnd = Vector3( RADIUS + FAT + 2.0f, 0.0f, 0.0f );
        TestLineSegIntersect( p_Capsule, expectedResult, lineStart, lineEnd, shouldHit, mtx, FAT );
    }

    // Test 9 - line with end point on capsule body
    shouldHit = true;
    // Thin Capsule
    {
        lineStart = Vector3( RADIUS - EPS + 1.0f, 0.0f , 0.0f );
        lineEnd = Vector3( RADIUS - EPS, 0.0f, 0.0f );
        expectedResult.position = Vector3( RADIUS, 0.0f, 0.0f );
        expectedResult.normal = Vector3( 1.0f, 0.0f, 0.0f );
        expectedResult.lineParam = 1.0f - EPS;
        TestLineSegIntersect( p_Capsule, expectedResult, lineStart, lineEnd, shouldHit, mtx, THIN );
    }
    // Fat Capsule
    {
        lineStart = Vector3( RADIUS + FAT - EPS + 1.0f, 0.0f, 0.0f );
        lineEnd = Vector3( RADIUS + FAT - EPS, 0.0f , 0.0f );
        expectedResult.position = Vector3( RADIUS, 0.0f, 0.0f );
        expectedResult.normal = Vector3( 1.0f, 0.0f, 0.0f );
        expectedResult.lineParam = 1.0f - EPS;
        TestLineSegIntersect( p_Capsule, expectedResult, lineStart, lineEnd, shouldHit, mtx, FAT );
    }

    // Test 10 - line with start point on capsule body
    shouldHit = true;
    // Thin Capsule
    {
        lineStart = Vector3( RADIUS - EPS, 0.0f , 0.0f );
        lineEnd = Vector3( RADIUS - EPS + 1.0f, 0.0f, 0.0f );
        expectedResult.position = lineStart;
        expectedResult.normal = Vector3( 1.0f, 0.0f, 0.0f );
        expectedResult.lineParam = 0.0f;
        TestLineSegIntersect( p_Capsule, expectedResult, lineStart, lineEnd, shouldHit, mtx, THIN );
    }
    // Fat Capsule
    {
        lineStart = Vector3( RADIUS + FAT - EPS, 0.0f, 0.0f );
        lineEnd = Vector3( RADIUS + FAT - EPS + 1.0f, 0.0f , 0.0f );
        expectedResult.position = Vector3( RADIUS - EPS, 0.0f, 0.0f );
        expectedResult.normal = Vector3( 1.0f, 0.0f, 0.0f );
        expectedResult.lineParam = 0.0f;
        TestLineSegIntersect( p_Capsule, expectedResult, lineStart, lineEnd, shouldHit, mtx, FAT );
    }

    // Test 11 - line which intersects +endcap from outside +endcap
    shouldHit = true;
    // Thin Capsule
    {
        lineStart = Vector3( 0.0f , 0.0f , HH + RADIUS + 1.0f );
        lineEnd = Vector3( 0.0f , 0.0f, HH + RADIUS - 1.0f );
        expectedResult.position = Vector3( 0.0f, 0.0f, HH + RADIUS );
        expectedResult.normal = Vector3( 0.0f, 0.0f, 1.0f );
        expectedResult.lineParam = 0.5f;
        TestLineSegIntersect( p_Capsule, expectedResult, lineStart, lineEnd, shouldHit, mtx, THIN );
    }
    // Fat Capsule
    {
        lineStart = Vector3( 0.0f , 0.0f , HH + RADIUS + FAT + 1.0f );
        lineEnd = Vector3( 0.0f , 0.0f,  HH + RADIUS + FAT - 1.0f );
        expectedResult.position = Vector3( 0.0f, 0.0f, HH + RADIUS );
        expectedResult.normal = Vector3( 0.0f, 0.0f, 1.0f );
        expectedResult.lineParam = 0.5f;
        TestLineSegIntersect( p_Capsule, expectedResult, lineStart, lineEnd, shouldHit, mtx, FAT );
    }

    // Test 12 - line which intersects -endcap from outside -endcap
    shouldHit = true;
    // Thin Capsule
    {
        lineStart = Vector3( 0.0f , 0.0f , -( HH + RADIUS + 1.0f ) );
        lineEnd = Vector3( 0.0f , 0.0f, -( HH + RADIUS - 1.0f ) );
        expectedResult.position = Vector3( 0.0f, 0.0f, -( HH + RADIUS ) );
        expectedResult.normal = Vector3( 0.0f, 0.0f, -1.0f );
        expectedResult.lineParam = 0.5f;
        TestLineSegIntersect( p_Capsule, expectedResult, lineStart, lineEnd, shouldHit, mtx, THIN );
    }
    // Fat Capsule
    {
        lineStart = Vector3( 0.0f , 0.0f , -( HH + RADIUS + FAT + 1.0f ) );
        lineEnd = Vector3( 0.0f , 0.0f,  -( HH + RADIUS + FAT - 1.0f ) );
        expectedResult.position = Vector3( 0.0f, 0.0f, -( HH + RADIUS ) );
        expectedResult.normal = Vector3( 0.0f, 0.0f, -1.0f );
        expectedResult.lineParam = 0.5f;
        TestLineSegIntersect( p_Capsule, expectedResult, lineStart, lineEnd, shouldHit, mtx, FAT );
    }

    // Test 13 - line which intersects +endcap from inside +endcap
    shouldHit = true;
    // Thin Capsule
    {
        lineStart = Vector3( 0.0f , 0.0f , HH + 0.5f*RADIUS );
        lineEnd = Vector3( 0.0f , 0.0f, HH + 1.5f*RADIUS );
        expectedResult.position = lineStart;
        expectedResult.normal = Vector3( 0.0f, 0.0f, 1.0f );
        expectedResult.lineParam = 0.0f;
        TestLineSegIntersect( p_Capsule, expectedResult, lineStart, lineEnd, shouldHit, mtx, THIN );
    }
    // Fat Capsule
    {
        lineStart = Vector3( 0.0f , 0.0f , HH + 0.5f*RADIUS );
        lineEnd = Vector3( 0.0f , 0.0f,  HH + 1.5f*RADIUS );
        expectedResult.position = Vector3( 0.0f , 0.0f , HH + 0.5f*RADIUS - FAT );
        expectedResult.normal = Vector3( 0.0f, 0.0f, 1.0f );
        expectedResult.lineParam = 0.0f;
        TestLineSegIntersect( p_Capsule, expectedResult, lineStart, lineEnd, shouldHit, mtx, FAT );
    }

    // Test 14 - line which intersects -endcap from inside -endcap
    shouldHit = true;
    // Thin Capsule
    {
        lineStart = Vector3( 0.0f , 0.0f , -( HH + 0.5f*RADIUS ) );
        lineEnd = Vector3( 0.0f , 0.0f, -( HH + 1.5f*RADIUS ) );
        expectedResult.position = lineStart;
        expectedResult.normal = Vector3( 0.0f, 0.0f, -1.0f );
        expectedResult.lineParam = 0.0f;
        TestLineSegIntersect( p_Capsule, expectedResult, lineStart, lineEnd, shouldHit, mtx, THIN );
    }
    // Fat Capsule
    {
        lineStart = Vector3( 0.0f , 0.0f , -( HH + RADIUS + FAT - 1.0f ) );
        lineEnd = Vector3( 0.0f , 0.0f,  -( HH + RADIUS + FAT + 1.0f ) );
        expectedResult.position = Vector3( 0.0f, 0.0f, -( HH + RADIUS - 1.0f ));
        expectedResult.normal = Vector3( 0.0f, 0.0f, -1.0f );
        expectedResult.lineParam = 0.0f;
        TestLineSegIntersect( p_Capsule, expectedResult, lineStart, lineEnd, shouldHit, mtx, FAT );
    }

    // Test 15 - line which lies inside +endcap
    shouldHit = true;
    // Thin Capsule
    {
        lineStart = Vector3( 0.0f , 0.0f , HH + RADIUS/4.0f );
        lineEnd = Vector3( 0.0f , 0.0f, HH + RADIUS/2.0f );
        expectedResult.position = lineStart;
        expectedResult.normal = Vector3( 0.0f, 0.0f, 1.0f );
        expectedResult.lineParam = 0.0f;
        TestLineSegIntersect( p_Capsule, expectedResult, lineStart, lineEnd, shouldHit, mtx, THIN );
    }
    // Fat Capsule
    {
        lineStart = Vector3( 0.0f , 0.0f , HH + (RADIUS + FAT)/4.0f );
        lineEnd = Vector3( 0.0f , 0.0f,  HH + (RADIUS + FAT)/2.0f );
        expectedResult.position = Vector3( 0.0f , 0.0f , HH + (RADIUS + FAT)/4.0f - FAT);
        expectedResult.normal = Vector3( 0.0f, 0.0f, 1.0f );
        expectedResult.lineParam = 0.0f;
        TestLineSegIntersect( p_Capsule, expectedResult, lineStart, lineEnd, shouldHit, mtx, FAT );
    }

    // Test 16 - line which lies inside -endcap
    shouldHit = true;
    // Thin Capsule
    {
        lineStart = Vector3( 0.0f , 0.0f , -( HH + RADIUS/4.0f ) );
        lineEnd = Vector3( 0.0f , 0.0f, -( HH + RADIUS/2.0f ) );
        expectedResult.position = lineStart;
        expectedResult.normal = Vector3( 0.0f, 0.0f, -1.0f );
        expectedResult.lineParam = 0.0f;
        TestLineSegIntersect( p_Capsule, expectedResult, lineStart, lineEnd, shouldHit, mtx, THIN );
    }
    // Fat Capsule
    {
        lineStart = Vector3( 0.0f , 0.0f , -( HH + (RADIUS + FAT)/4.0f ) );
        lineEnd = Vector3( 0.0f , 0.0f,  -( HH + (RADIUS + FAT)/2.0f ) );
        expectedResult.position = Vector3( 0.0f , 0.0f , -(HH + (RADIUS + FAT)/4.0f) + FAT);
        expectedResult.normal = Vector3( 0.0f, 0.0f, -1.0f );
        expectedResult.lineParam = 0.0f;
        TestLineSegIntersect( p_Capsule, expectedResult, lineStart, lineEnd, shouldHit, mtx, FAT );
    }

    // Test 17 - line which lies outside of +endcap and points towards +endcap
    shouldHit = false;
    // Thin Capsule
    {
        lineStart = Vector3( 0.0f , 0.0f , HH + RADIUS + 2.0f );
        lineEnd = Vector3( 0.0f , 0.0f, HH + RADIUS + 1.0f );
        TestLineSegIntersect( p_Capsule, expectedResult, lineStart, lineEnd, shouldHit, mtx, THIN );
    }
    // Fat Capsule
    {
        lineStart = Vector3( 0.0f , 0.0f , HH + RADIUS + FAT + 2.0f );
        lineEnd = Vector3( 0.0f , 0.0f,  HH + RADIUS + FAT + 1.0f );
        TestLineSegIntersect( p_Capsule, expectedResult, lineStart, lineEnd, shouldHit, mtx, FAT );
    }

    // Test 18 - line which lies outside of -endcap and points towards -endcap
    shouldHit = false;
    // Thin Capsule
    {
        lineStart = Vector3( 0.0f , 0.0f , -( HH + RADIUS + 2.0f ) );
        lineEnd = Vector3( 0.0f , 0.0f, -( HH + RADIUS + 1.0f ) );
        TestLineSegIntersect( p_Capsule, expectedResult, lineStart, lineEnd, shouldHit, mtx, THIN );
    }
    // Fat Capsule
    {
        lineStart = Vector3( 0.0f , 0.0f , -( HH + RADIUS + FAT + 2.0f ) );
        lineEnd = Vector3( 0.0f , 0.0f,  -( HH + RADIUS + FAT + 1.0f ) );
        TestLineSegIntersect( p_Capsule, expectedResult, lineStart, lineEnd, shouldHit, mtx, FAT );
    }

    // Test 19 - line which lies outisde of +endcap and points away from +endcap
    shouldHit = false;
    // Thin Capsule
    {
        lineStart = Vector3( 0.0f , 0.0f , HH + RADIUS + 1.0f );
        lineEnd = Vector3( 0.0f , 0.0f, HH + RADIUS + 2.0f );
        TestLineSegIntersect( p_Capsule, expectedResult, lineStart, lineEnd, shouldHit, mtx, THIN );
    }
    // Fat Capsule
    {
        lineStart = Vector3( 0.0f , 0.0f , HH + RADIUS + FAT + 1.0f );
        lineEnd = Vector3( 0.0f , 0.0f,  HH + RADIUS + FAT + 2.0f );
        TestLineSegIntersect( p_Capsule, expectedResult, lineStart, lineEnd, shouldHit, mtx, FAT );
    }

    // Test 20 - line which lies outside of -endcap and points away from -endcap
    shouldHit = false;
    // Thin Capsule
    {
        lineStart = Vector3( 0.0f , 0.0f , -( HH + RADIUS + 1.0f ) );
        lineEnd = Vector3( 0.0f , 0.0f, -( HH + RADIUS + 2.0f ) );
        TestLineSegIntersect( p_Capsule, expectedResult, lineStart, lineEnd, shouldHit, mtx, THIN );
    }
    // Fat Capsule
    {
        lineStart = Vector3( 0.0f , 0.0f , -( HH + RADIUS + FAT + 1.0f ) );
        lineEnd = Vector3( 0.0f , 0.0f,  -( HH + RADIUS + FAT + 2.0f ) );
        TestLineSegIntersect( p_Capsule, expectedResult, lineStart, lineEnd, shouldHit, mtx, FAT );
    }

    // Test 21 - line with end point on capsule +endcap
    shouldHit = true;
    // Thin Capsule
    {
        lineStart = Vector3( 0.0f , 0.0f , HH + RADIUS - EPS + 1.0f );
        lineEnd = Vector3( 0.0f , 0.0f, HH + RADIUS - EPS );
        expectedResult.position = Vector3( 0.0f, 0.0f, HH + RADIUS );
        expectedResult.normal = Vector3( 0.0f, 0.0f, 1.0f );
        expectedResult.lineParam = 1.0f - EPS;
        TestLineSegIntersect( p_Capsule, expectedResult, lineStart, lineEnd, shouldHit, mtx, THIN );
    }
    // Fat Capsule
    {
        lineStart = Vector3( 0.0f , 0.0f , HH + RADIUS + FAT - EPS + 1.0f);
        lineEnd = Vector3( 0.0f , 0.0f, HH + RADIUS + FAT - EPS );
        expectedResult.position = Vector3( 0.0f, 0.0f, HH + RADIUS );
        expectedResult.normal = Vector3( 0.0f, 0.0f, 1.0f );
        expectedResult.lineParam = 1.0f - EPS;
        TestLineSegIntersect( p_Capsule, expectedResult, lineStart, lineEnd, shouldHit, mtx, FAT );
    }

    // Test 22 - line with end point on capsule -endcap
    shouldHit = true;
    // Thin Capsule
    {
        lineStart = Vector3( 0.0f , 0.0f , -( HH + RADIUS - EPS + 1.0f ) );
        lineEnd = Vector3( 0.0f , 0.0f, -( HH + RADIUS - EPS) );
        expectedResult.position = Vector3( 0.0f, 0.0f, -( HH + RADIUS ) );
        expectedResult.normal = Vector3( 0.0f, 0.0f, -1.0f );
        expectedResult.lineParam = 1.0f - EPS;
        TestLineSegIntersect( p_Capsule, expectedResult, lineStart, lineEnd, shouldHit, mtx, THIN );
    }
    // Fat Capsule
    {
        lineStart = Vector3( 0.0f , 0.0f , -( HH + RADIUS + FAT - EPS + 1.0f ) );
        lineEnd = Vector3( 0.0f , 0.0f,  -( HH + RADIUS + FAT - EPS) );
        expectedResult.position = Vector3( 0.0f, 0.0f, -( HH + RADIUS) );
        expectedResult.normal = Vector3( 0.0f, 0.0f, -1.0f );
        expectedResult.lineParam = 1.0f - EPS;
        TestLineSegIntersect( p_Capsule, expectedResult, lineStart, lineEnd, shouldHit, mtx, FAT );
    }

    // Test 23 - line with start point on capsule +endcap
    shouldHit = true;
    // Thin Capsule
    {
        lineStart = Vector3( 0.0f , 0.0f , HH + RADIUS - EPS );
        lineEnd = Vector3( 0.0f , 0.0f, HH + RADIUS - EPS + 1.0f);
        expectedResult.position = lineStart;
        expectedResult.normal = Vector3( 0.0f, 0.0f, 1.0f );
        expectedResult.lineParam = 0.0f;
        TestLineSegIntersect( p_Capsule, expectedResult, lineStart, lineEnd, shouldHit, mtx, THIN );
    }
    // Fat Capsule
    {
        lineStart = Vector3( 0.0f , 0.0f , HH + RADIUS + FAT - EPS );
        lineEnd = Vector3( 0.0f , 0.0f, HH + RADIUS + FAT - EPS + 1.0f);
        expectedResult.position = Vector3( 0.0f , 0.0f , HH + RADIUS - EPS );
        expectedResult.normal = Vector3( 0.0f, 0.0f, 1.0f );
        expectedResult.lineParam = 0.0f;
        TestLineSegIntersect( p_Capsule, expectedResult, lineStart, lineEnd, shouldHit, mtx, FAT );
    }

    // Test 24 - line with start point on capsule -endcap
    shouldHit = true;
    // Thin Capsule
    {
        lineStart = Vector3( 0.0f , 0.0f , -( HH + RADIUS - EPS ) );
        lineEnd = Vector3( 0.0f , 0.0f, -( HH + RADIUS - EPS + 1.0f ) );
        expectedResult.position = lineStart;
        expectedResult.normal = Vector3( 0.0f, 0.0f, -1.0f );
        expectedResult.lineParam = 0.0f;
        TestLineSegIntersect( p_Capsule, expectedResult, lineStart, lineEnd, shouldHit, mtx, THIN );
    }
    // Fat Capsule
    {
        lineStart = Vector3( 0.0f , 0.0f , -( HH + RADIUS + FAT - EPS ) );
        lineEnd = Vector3( 0.0f , 0.0f,  -( HH + RADIUS + FAT - EPS + 1.0f ) );
        expectedResult.position = Vector3( 0.0f , 0.0f , -( HH + RADIUS - EPS ) );
        expectedResult.normal = Vector3( 0.0f, 0.0f, -1.0f );
        expectedResult.lineParam = 0.0f;
        TestLineSegIntersect( p_Capsule, expectedResult, lineStart, lineEnd, shouldHit, mtx, FAT );
    }

    // Line starting in cylinder region and hitting endcap (BUG#29535)
    {
        const float angle = rwpmath::PI / 8.0f;
        shouldHit = true;
        expectedResult.position = Vector3( RADIUS * rw::math::fpu::Cos(angle), 0.0f, HH + RADIUS * rw::math::fpu::Sin(angle));
        lineStart = Vector3( HH + RADIUS , 0.0f , 0.0f );
        lineEnd = lineStart + 1.25f * (expectedResult.position - lineStart);
        expectedResult.normal = Vector3( rw::math::fpu::Cos(angle), 0.0f, rw::math::fpu::Sin(angle) );
        expectedResult.lineParam = 0.8f;
        TestLineSegIntersect( p_Capsule, expectedResult, lineStart, lineEnd, shouldHit, mtx, THIN );
    }

    // Line starting in endcap region and hitting cylinder region  (BUG#29535)
    {
        shouldHit = true;
        expectedResult.position = Vector3( RADIUS, 0.0f, RADIUS);
        expectedResult.normal = Vector3( 1.0f, 0.0f, 0.0f );
        lineStart = Vector3( HH + RADIUS , 0.0f , HH + RADIUS );
        lineEnd = Vector3( 0.0f, 0.0f, 0.0f);
        expectedResult.lineParam = HH / (HH + RADIUS);
        TestLineSegIntersect( p_Capsule, expectedResult, lineStart, lineEnd, shouldHit, mtx, THIN );
    }

}



// Testing the LineSegIntersect method - called by TestLineSegIntersections, this method holds a number
// of matricies which are applied to the volume and line before intersection detection is carried out
void TestCapsuleLineQuery::TestLineSegIntersectionsWithMatricies( CapsuleVolume* p_Capsule ){

    // Transformation matrix
    Matrix44Affine tm;

    // NULL matrix
    LineSegIntersectionScenarios( p_Capsule, 0 );

    //X Translate
    tm = Matrix44AffineFromTranslation(Vector3(10.0f, 0.0f, 0.0f));
    LineSegIntersectionScenarios( p_Capsule, &tm );

    //Y Translate
    tm = Matrix44AffineFromTranslation(Vector3(0.0f, 1.0f, 0.0f));
    LineSegIntersectionScenarios( p_Capsule, &tm );

    //Z Translate
    tm = Matrix44AffineFromTranslation(Vector3(0.0f, 0.0f, 10.0f));
    LineSegIntersectionScenarios( p_Capsule, &tm );

    //90 X Axis rotation
    tm = Matrix44AffineFromXRotationAngle(rwpmath::PI/2.0f);
    LineSegIntersectionScenarios( p_Capsule, &tm );

    //-90 X Axis rotation
    tm = Matrix44AffineFromXRotationAngle(-rwpmath::PI/2.0f);
    LineSegIntersectionScenarios( p_Capsule, &tm );

    //180 X Axis rotation
    tm = Matrix44AffineFromXRotationAngle(rwpmath::PI);
    LineSegIntersectionScenarios( p_Capsule, &tm );

    //90 Y Axis rotation
    tm = Matrix44AffineFromYRotationAngle(rwpmath::PI/2.0f);
    LineSegIntersectionScenarios( p_Capsule, &tm );

    //-90 Y Axis rotation
    tm = Matrix44AffineFromYRotationAngle(-rwpmath::PI/2.0f);
    LineSegIntersectionScenarios( p_Capsule, &tm );

    //180 Y Axis rotation
    tm = Matrix44AffineFromXRotationAngle(rwpmath::PI);
    LineSegIntersectionScenarios( p_Capsule, &tm );

    //90 Z Axis rotation
    tm = Matrix44AffineFromZRotationAngle(rwpmath::PI/2.0f);
    LineSegIntersectionScenarios( p_Capsule, &tm );

    //-90 Z Axis rotation
    tm = Matrix44AffineFromZRotationAngle(-rwpmath::PI/2.0f);
    LineSegIntersectionScenarios( p_Capsule, &tm );

    //180 X Axis rotation
    tm = Matrix44AffineFromXRotationAngle(rwpmath::PI);
    LineSegIntersectionScenarios( p_Capsule, &tm );   
}


// Testing the LineSegIntersect method - this method creates a number of different Capsule volumes which are
// used to test the LineSegIntersect method in various scenarios and with various transformation matrices
void TestCapsuleLineQuery::TestLineSegIntersections(){

    // Capsule with 5.0 halfheight and 1.0 radius
    const float HH = 5.0f;
    const float RADIUS = 1.0f;
    CapsuleVolume * p_Capsule = EA::Physics::UnitFramework::Creator<CapsuleVolume>().New( RADIUS, HH );
    TestLineSegIntersectionsWithMatricies(p_Capsule);

#if !defined(DISABLE_KNOWN_FAILING_UNITTESTS)
    // Capsule with 5.0 halfheight and 0 radius
    p_Capsule = EA::Physics::UnitFramework::Creator<CapsuleVolume>().New( 0.0f, HH );
    TestLineSegIntersectionsWithMatricies(p_Capsule);

    // Capsule with 0.0 halfheight and 1.0 radius
    p_Capsule = EA::Physics::UnitFramework::Creator<CapsuleVolume>().New( RADIUS, 0.0f );
    TestLineSegIntersectionsWithMatricies(p_Capsule));

    // Capsule with 0.0 halfheight and 0.0 radius
    p_Capsule = EA::Physics::UnitFramework::Creator<CapsuleVolume>().New( 0.0f, 0.0f );
    TestLineSegIntersectionsWithMatricies(p_Capsule);
#endif

}

void TestCapsuleLineQuery::TestrwcCylinderLineSegIntersect(){

    // parametric distance along line segment to point of entry or exit
    Fraction dist;
    // line coordinates
    Vector3 lineStart, lineEnd;
    // center of cylinder
    Vector3 center( 0.0f, 0.0f, 0.0f );
    // axis along which cylinder is created
    Vector3 axis( 0.0f, 0.0f, 1.0f );
    // cylinder axis length squared
    float halfHeight = 2;
    float axisLengthSq = (halfHeight*2) * (halfHeight*2); // halfheight = 2, length = 4, lengthSq = 16
    // cylinder radius
    float radius = 2;


    // Test 1 - line which lies inside cylinder
    lineStart = Vector3( 0.0f, 0.0f, 0.0f );
    lineEnd = Vector3( 0.0f, 0.0f, 1.0f );

    EATESTAssert( 1 == rwcCylinderLineSegIntersect( &dist, lineStart, lineEnd, center, axis, axisLengthSq, radius, false, false),
        "Testing rwcCylinderLineSegIntersect - line which lies inside cylinder return value" );
    EATESTAssert( IsSimilar( static_cast<float>(dist.num / dist.den), 0.0f )  , "Testing rwcCylinderLineSegIntersect - line which lies tangential to dist" );

    // Test 2 - line which lies tangential to cylinder
    lineStart = Vector3( radius, radius, 0.0f );
    lineEnd = Vector3(  radius, -radius, 0.0f );
    EATESTAssert( 1 == rwcCylinderLineSegIntersect( &dist, lineStart, lineEnd, center, axis, axisLengthSq, radius, false, false),
        "Testing rwcCylinderLineSegIntersect - line which lies tangential to cylinder return value" );
    EATESTAssert( IsSimilar( static_cast<float>(dist.num / dist.den), 0.5f )  , "Testing rwcCylinderLineSegIntersect - line which lies tangential to dist" );

    // Test 3 - line which lies outside of capsule and points away from the cylinder
    lineStart = Vector3( radius, radius, 0.0f );
    lineEnd = Vector3(  radius + 1.0f , radius + 1.0f, 0.0f );
    EATESTAssert( 0 == rwcCylinderLineSegIntersect( &dist, lineStart, lineEnd, center, axis, axisLengthSq, radius, false, false),
        "Testing rwcCylinderLineSegIntersect - line which lies outside of capsule and points away from the cylinder return value" );

    // Test 4 - line which lies outside of capsule and points towards the cylinder
    lineStart = Vector3( radius + 1.0f, radius + 1.0f, 0.0f );
    lineEnd = Vector3(  radius, radius, 0.0f );
    EATESTAssert( 0 == rwcCylinderLineSegIntersect( &dist, lineStart, lineEnd, center, axis, axisLengthSq, radius, false, false),
        "Testing rwcCylinderLineSegIntersect - line which lies outside of capsule and points towards the cylinder return value" );

    // Test 5 - line with end point on cylinder
    lineStart = Vector3( radius + 1.0f, 0.0f, 0.0f );
    lineEnd = Vector3(  radius, 0.0f, 0.0f );
    EATESTAssert( 1 == rwcCylinderLineSegIntersect( &dist, lineStart, lineEnd, center, axis, axisLengthSq, radius, false, false),
        "Testing rwcCylinderLineSegIntersect - line with end point on cylinder return value" );
    EATESTAssert( IsSimilar( static_cast<float>(dist.num / dist.den) , 1.0f ) , "Testing rwcCylinderLineSegIntersect - line with end point on cylinder dist" );


    // Test 6 - line with start point on cylinder
    lineStart = Vector3( radius, 0.0f, 0.0f );
    lineEnd = Vector3(  radius + 1.0f, 0.0f, 0.0f );
    EATESTAssert( 1 == rwcCylinderLineSegIntersect( &dist, lineStart, lineEnd, center, axis, axisLengthSq, radius, false, false),
        "Testing rwcCylinderLineSegIntersect - line with start point on cylinder return value" );
    EATESTAssert( IsSimilar( static_cast<float>(dist.num / dist.den) , 0.0f ) , "Testing rwcCylinderLineSegIntersect - line with end point on cylinder dist" );

    // Test 7 - line which lies on cylinder body
    lineStart = Vector3( radius, 0.0f, -halfHeight );
    lineEnd = Vector3(  radius + 1.0f, 0.0f, halfHeight );
    EATESTAssert( 1 == rwcCylinderLineSegIntersect( &dist, lineStart, lineEnd, center, axis, axisLengthSq, radius, false, false),
        "Testing rwcCylinderLineSegIntersect - line with start point on cylinder return value" );
    EATESTAssert( IsSimilar( static_cast<float>(dist.num / dist.den) , 0.0f ) , "Testing rwcCylinderLineSegIntersect - line with end point on cylinder dist" );
}
