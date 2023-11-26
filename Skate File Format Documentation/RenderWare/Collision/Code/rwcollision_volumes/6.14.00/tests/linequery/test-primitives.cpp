// (c) Electronic Arts. All Rights Reserved.

#include <unit/unit.h>

#include <EABase/eabase.h>

#include <eaphysics/base.h>

#include <rw/collision/libcore.h>
#include <rw/collision/kdtreebuilder.h>
#include <rw/collision/kdtree.h>

#include <eaphysics/unitframework/allocator.h> // For ResetAllocator
#include <eaphysics/unitframework/creator.h> // For Creator

#include "testsuitebase.h" // For TestSuiteBase

using namespace rw::collision;
using namespace rwpmath;

// ***********************************************************************************************************
// Test suite


class TestLineQueryPrimitives : public tests::TestSuiteBase
{
public:
    virtual void Initialize()
    {
        SuiteName("TestLineQueryPrimitives");

        // Test individual Methods
        EATEST_REGISTER("SphereTest", "SphereTest", TestLineQueryPrimitives, SphereTest);   
        EATEST_REGISTER("CapsuleTest", "CapsuleTest", TestLineQueryPrimitives, CapsuleTest);    
        EATEST_REGISTER("TriangleOneSidedTest", "TriangleOneSidedTest", TestLineQueryPrimitives, TriangleOneSidedTest);  
        EATEST_REGISTER("TriangleTwoSidedTest", "TriangleTwoSidedTest", TestLineQueryPrimitives, TriangleTwoSidedTest); 
        EATEST_REGISTER("BoxTest", "BoxTest", TestLineQueryPrimitives, BoxTest);    
        EATEST_REGISTER("CylinderTest", "CylinderTest", TestLineQueryPrimitives, CylinderTest);    
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
        Volume::ReleaseVTable();
        tests::TestSuiteBase::TeardownSuite();
    }

private:
    // Test individual Methods
    void SphereTest();
    void CapsuleTest();
    void TriangleOneSidedTest();
    void TriangleTwoSidedTest();
    void BoxTest();
    void CylinderTest();
    
} TestLineQueryPrimitivesSingleton;

void TestLineQueryPrimitives::SphereTest()
{
    //Create SphereVolume of radius 5 units
    SphereVolume* vol = EA::Physics::UnitFramework::Creator<SphereVolume>().New(5.0f);

    //Set transform position.
    Vector3 position = Vector3(10.0f, -10.0f, 0.0f);
    Matrix44Affine mtx(vol->GetLocalTransform());
    mtx.SetW(position);
    vol->SetLocalTransform(mtx);

    //Test Line hits
    {
        VolumeLineSegIntersectResult lineResult;
        RwpBool hit;
        Vector3 lineStart = Vector3(10.0f, 0.0f, 0.0f);
        Vector3 lineUnitDirection = Normalize(position - lineStart);
        hit = vol->LineSegIntersect(lineStart, position, NULL, lineResult);
        EATESTAssert(hit, "Expected line sphere intersection");
        EATESTAssert(IsSimilar(lineResult.lineParam, 0.5f), "Expected LineParam of 0.5f");
        EATESTAssert(IsSimilar(lineResult.normal, -lineUnitDirection), "Expected 'Up' normal");
        EATESTAssert(IsSimilar(lineResult.position, lineStart + 0.5f * lineUnitDirection * Magnitude(position - lineStart)), "Unexpected Position of intersection");
        EATESTAssert(IsSimilar(lineResult.volParam, GetVector3_Zero()), "Expected Zero volParam");
        EATESTAssert(lineResult.v == vol, "Expected volume pointer");
    }

    //Test line misses
    {
        VolumeLineSegIntersectResult lineResult;
        RwpBool hit;
        hit = vol->LineSegIntersect(Vector3(10.0f, 0.0f, 0.0f), Vector3(10.0f, 10.0f, 0.0f), NULL, lineResult);
        EATESTAssert(hit ? false : true, "Expected line sphere miss");
        EATESTAssert(IsSimilar(lineResult.lineParam, 0.0f), "Expected LineParam of 0");
        EATESTAssert(IsSimilar(lineResult.normal, GetVector3_Zero()), "Expected Zero normal");
        EATESTAssert(IsSimilar(lineResult.position, GetVector3_Zero()), "Expected Zero position");
        EATESTAssert(IsSimilar(lineResult.volParam, GetVector3_Zero()), "Expected Zero volParam");
        EATESTAssert(lineResult.v == vol, "Expected volume pointer");
    }

    //Test line starts inside
    {
        VolumeLineSegIntersectResult lineResult;
        Vector3 lineStart = position + Vector3(0.0f, 0.1f, 0.0f);
        RwpBool hit = vol->LineSegIntersect(lineStart, Vector3(10.0f, 0.0f, 0.0f), NULL, lineResult);
        EATESTAssert(hit, "Expected line volume hit");
        EATESTAssert(IsSimilar(lineResult.lineParam, 0.0f), "Expected LineParam of 0");
        EATESTAssert(IsSimilar(lineResult.normal, Vector3(0.0f, 1.0f, 0.0f)), "Unexpected normal");
        EATESTAssert(IsSimilar(lineResult.position, lineStart), "Expected position equal to lineStart");
        EATESTAssert(IsSimilar(lineResult.volParam, GetVector3_Zero()), "Expected Zero volParam");
        EATESTAssert(lineResult.v == vol, "Expected volume pointer");
    }
}

void TestLineQueryPrimitives::CapsuleTest()
{
    //Create CapsuleVolume of radius 5 units and halflength 5 units
    CapsuleVolume* vol = EA::Physics::UnitFramework::Creator<CapsuleVolume>().New(5.0f, 5.0f);

    //Set transform position.
    Vector3 position = Vector3(10.0f, -10.0f, 0.0f);
    Matrix44Affine mtx(vol->GetLocalTransform());
    mtx.SetW(position);
    vol->SetLocalTransform(mtx);

    //Test Line hits
    {
        VolumeLineSegIntersectResult lineResult;
        RwpBool hit;
        Vector3 lineStart = Vector3(10.0f, 0.0f, 0.0f);
        Vector3 lineUnitDirection = Normalize(position - lineStart);
        hit = vol->LineSegIntersect(lineStart, position, NULL, lineResult);
        EATESTAssert(hit, "Expected line volume intersection");
        EATESTAssert(IsSimilar(lineResult.lineParam, 0.5f), "Expected LineParam of 0.5f");
        EATESTAssert(IsSimilar(lineResult.normal, -lineUnitDirection), "Expected 'Up' normal");
        EATESTAssert(IsSimilar(lineResult.position, lineStart + 0.5f * lineUnitDirection * Magnitude(position - lineStart)), "Unexpected Position of intersection");
        EATESTAssert(IsSimilar(lineResult.volParam, GetVector3_Zero()), "Expected Zero volParam");
        EATESTAssert(lineResult.v == vol, "Expected volume pointer");
    }

    //Test line misses
    {
        VolumeLineSegIntersectResult lineResult;
        RwpBool hit = vol->LineSegIntersect(Vector3(10.0f, 0.0f, 0.0f), Vector3(10.0f, 10.0f, 0.0f), NULL, lineResult);
        EATESTAssert(hit ? false : true, "Expected line volume miss");
        EATESTAssert(IsSimilar(lineResult.lineParam, 0.0f), "Expected LineParam of 0");
        EATESTAssert(IsSimilar(lineResult.normal, GetVector3_Zero()), "Expected Zero normal");
        EATESTAssert(IsSimilar(lineResult.position, GetVector3_Zero()), "Expected Zero position");
        EATESTAssert(IsSimilar(lineResult.volParam, GetVector3_Zero()), "Expected Zero volParam");
        EATESTAssert(lineResult.v == vol, "Expected volume pointer");
    }

    //Test line starts inside
    {
        VolumeLineSegIntersectResult lineResult;
        Vector3 lineStart = position + Vector3(0.0f, 0.1f, 0.0f);
        RwpBool hit = vol->LineSegIntersect(lineStart, Vector3(10.0f, 0.0f, 0.0f), NULL, lineResult);
        EATESTAssert(hit, "Expected line volume hit");
        EATESTAssert(IsSimilar(lineResult.lineParam, 0.0f), "Expected LineParam of 0");
        EATESTAssert(IsSimilar(lineResult.normal, Vector3(0.0f, 1.0f, 0.0f)), "Unexpected normal");
        EATESTAssert(IsSimilar(lineResult.position, lineStart), "Expected position equal to lineStart");
        EATESTAssert(IsSimilar(lineResult.volParam, GetVector3_Zero()), "Expected Zero volParam");
        EATESTAssert(lineResult.v == vol, "Expected volume pointer");
    }
}

void TestLineQueryPrimitives::TriangleOneSidedTest()
{
    //Create TriangleVolume of radius 5 units.
    TriangleVolume* vol = EA::Physics::UnitFramework::Creator<TriangleVolume>().New( Vector3(-5.0f, 0.0f, -5.0f), Vector3(0.0f, 0.0f, 5.0f), Vector3(5.0f, 0.0f, -5.0f), 5.0f);
    //Set transform position.
    Vector3 position = Vector3(0.0f, 0.0f, 0.0f);
    //Set onesided triangle
    vol->SetFlags(vol->GetFlags() | VOLUMEFLAG_TRIANGLEONESIDED);

    //Test Line hits
    {
        VolumeLineSegIntersectResult lineResult;
        RwpBool hit;
        Vector3 lineStart = Vector3(0.0f, 10.0f, 0.0f);
        Vector3 lineUnitDirection = Normalize(position - lineStart);
        hit = vol->LineSegIntersect(lineStart, position, NULL, lineResult);
        EATESTAssert(hit, "Expected line volume intersection");
        EATESTAssert(IsSimilar(lineResult.lineParam, 0.5f), "Expected LineParam of 0.5f");
        EATESTAssert(IsSimilar(lineResult.normal, Vector3(0.0f, 1.0f, 0.0f)), "Expected 'Up' normal");
        EATESTAssert(IsSimilar(lineResult.position, lineStart + 0.5f * lineUnitDirection * Magnitude(position - lineStart)), "Unexpected Position of intersection");
        EATESTAssert(IsSimilar(lineResult.volParam, Vector3(0.5f, 0.25f, 0.0f)), "Expected Zero volParam");
        EATESTAssert(lineResult.v == vol, "Expected volume pointer");
    }

    //Test line misses
    {
        VolumeLineSegIntersectResult lineResult;
        RwpBool hit;
        hit = vol->LineSegIntersect(Vector3(0.0f, 10.0f, 0.0f), Vector3(0.0f, 20.0f, 0.0f), NULL, lineResult);
        EATESTAssert(hit ? false : true, "Expected line volume miss");
        EATESTAssert(IsSimilar(lineResult.lineParam, 0.0f), "Expected LineParam of 0");
        EATESTAssert(IsSimilar(lineResult.normal, Vector3(0.0f, 1.0f, 0.0f)), "Expected Zero normal");
        EATESTAssert(IsSimilar(lineResult.position, GetVector3_Zero()), "Expected Zero position");
        EATESTAssert(IsSimilar(lineResult.volParam, GetVector3_Zero()), "Unexpected volParam");
        EATESTAssert(lineResult.v == vol, "Expected volume pointer");
    }

    //Test line starts inside
    {
        VolumeLineSegIntersectResult lineResult;
        RwpBool hit;
        hit = vol->LineSegIntersect(Vector3(0.0f, 0.1f, 0.0f), Vector3(0.0f, -10.0f, 0.0f), NULL, lineResult);
        EATESTAssert(hit, "Expected line volume hit");
        EATESTAssert(IsSimilar(lineResult.lineParam, 0.0f), "Expected LineParam of 0");
        EATESTAssert(IsSimilar(lineResult.normal, Vector3(0.0f, 1.0f, 0.0f)), "Expected Up normal");
        EATESTAssert(IsSimilar(lineResult.position, Vector3(0.0f, 0.1f, 0.0f)), "Expected position equal to lineStart");
        EATESTAssert(IsSimilar(lineResult.volParam, Vector3(0.5f, 0.25f, 24.01f)), "Expected Zero volParam");
        EATESTAssert(lineResult.v == vol, "Expected volume pointer");
    }
}

void TestLineQueryPrimitives::TriangleTwoSidedTest()
{
    //Create TriangleVolume of radius 5 units.
    TriangleVolume* vol = EA::Physics::UnitFramework::Creator<TriangleVolume>().New(Vector3(0.0f, 0.0f, 5.0f), Vector3(-5.0f, 0.0f, -5.0f), Vector3(5.0f, 0.0f, -5.0f), 5.0f);
    //Set transform position.
    Vector3 position = Vector3(0.0f, 0.0f, 0.0f);
    //Set two sided triangle
    vol->SetFlags(vol->GetFlags() & ~VOLUMEFLAG_TRIANGLEONESIDED);

    //Test Line hits
    {
        VolumeLineSegIntersectResult lineResult;
        RwpBool hit;
        Vector3 lineStart = Vector3(0.0f, 10.0f, 0.0f);
        Vector3 lineUnitDirection = Normalize(position - lineStart);
        hit = vol->LineSegIntersect(lineStart, position, NULL, lineResult);
        EATESTAssert(hit, "Expected line volume intersection");
        EATESTAssert(IsSimilar(lineResult.lineParam, 0.5f), "Expected LineParam of 0.5f");
        EATESTAssert(IsSimilar(lineResult.normal, Vector3(0.0f, 1.0f, 0.0f)), "Expected 'Up' normal");
        EATESTAssert(IsSimilar(lineResult.position, lineStart + 0.5f * lineUnitDirection * Magnitude(position - lineStart)), "Unexpected Position of intersection");
        EATESTAssert(IsSimilar(lineResult.volParam, Vector3(0.25f, 0.25f, 0.0f)), "Unexpected volParam");
        EATESTAssert(lineResult.v == vol, "Expected volume pointer");
    }

    //Test line misses
    {
        VolumeLineSegIntersectResult lineResult;
        RwpBool hit;
        hit = vol->LineSegIntersect(Vector3(0.0f, 10.0f, 0.0f), Vector3(0.0f, 20.0f, 0.0f), NULL, lineResult);
        EATESTAssert(hit ? false : true, "Expected line volume miss");
        EATESTAssert(IsSimilar(lineResult.lineParam, 0.0f), "Expected LineParam of 0");
        EATESTAssert(IsSimilar(lineResult.normal, Vector3(0.0f, -1.0f, 0.0f)), "Unexpected normal");
        EATESTAssert(IsSimilar(lineResult.position, GetVector3_Zero()), "Expected Zero position");
        EATESTAssert(IsSimilar(lineResult.volParam, GetVector3_Zero()), "Unexpected volParam");
        EATESTAssert(lineResult.v == vol, "Expected volume pointer");
    }

    //Test line starts inside
    {
        VolumeLineSegIntersectResult lineResult;
        RwpBool hit;
        hit = vol->LineSegIntersect(Vector3(0.0f, 0.1f, 0.0f), Vector3(0.0f, 10.0f, 0.0f), NULL, lineResult);
        EATESTAssert(hit, "Expected line volume hit");
        EATESTAssert(IsSimilar(lineResult.lineParam, 0.0f), "Expected LineParam of 0");
        EATESTAssert(IsSimilar(lineResult.normal, Vector3(0.0f, 1.0f, 0.0f)), "Expected Up normal");
        EATESTAssert(IsSimilar(lineResult.position, Vector3(0.0f, 0.1f, 0.0f)), "Expected position equal to lineStart");
        EATESTAssert(IsSimilar(lineResult.volParam, Vector3(0.25f, 0.25f, 24.01f)), "Expected Zero volParam");
        EATESTAssert(lineResult.v == vol, "Expected volume pointer");
    }
}


void TestLineQueryPrimitives::BoxTest()
{
    //Create CapsuleVolume of radius 5 units and halflength 5 units
    BoxVolume* vol = EA::Physics::UnitFramework::Creator<BoxVolume>().New(Vector3(5.0f, 5.0f, 5.0f), 0.0f);


    //Set transform position.
    Vector3 position = Vector3(10.0f, -10.0f, 0.0f);
    Matrix44Affine mtx(vol->GetLocalTransform());
    mtx.SetW(position);
    vol->SetLocalTransform(mtx);

    //Test Line hits
    {
        VolumeLineSegIntersectResult lineResult;
        RwpBool hit;
        Vector3 lineStart = Vector3(10.0f, 0.0f, 0.0f);
        Vector3 lineUnitDirection = Normalize(position - lineStart);
        hit = vol->LineSegIntersect(lineStart, position, NULL, lineResult);
        EATESTAssert(hit, "Expected line volume intersection");
        EATESTAssert(IsSimilar(lineResult.lineParam, 0.5f), "Expected LineParam of 0.5f");
        EATESTAssert(IsSimilar(lineResult.normal, -lineUnitDirection), "Expected 'Up' normal");
        EATESTAssert(IsSimilar(lineResult.position, lineStart + 0.5f * lineUnitDirection * Magnitude(position - lineStart)), "Unexpected Position of intersection");
        EATESTAssert(IsSimilar(lineResult.volParam, Vector3(0.0f, 1.0f, 0.0f)), "Unexpected volParam");
        EATESTAssert(lineResult.v == vol, "Expected volume pointer");
    }

    //Test line misses
    {
        VolumeLineSegIntersectResult lineResult;
        RwpBool hit;
        hit = vol->LineSegIntersect(Vector3(10.0f, 0.0f, 0.0f), Vector3(10.0f, 10.0f, 0.0f), NULL, lineResult);
        EATESTAssert(hit ? false : true, "Expected line volume miss");
        EATESTAssert(IsSimilar(lineResult.lineParam, 0.0f), "Expected LineParam of 0");
        EATESTAssert(IsSimilar(lineResult.normal, GetVector3_Zero()), "Expected Zero normal");
        EATESTAssert(IsSimilar(lineResult.position, GetVector3_Zero()), "Expected Zero position");
        EATESTAssert(IsSimilar(lineResult.volParam, GetVector3_Zero()), "Expected Zero volParam");
        EATESTAssert(lineResult.v == vol, "Expected volume pointer");
    }

    //Test line starts inside
    {
        VolumeLineSegIntersectResult lineResult;
        RwpBool hit;
        Vector3 lineStart = position + Vector3(0.0f, 0.1f, 0.0f);
        hit = vol->LineSegIntersect(lineStart, Vector3(10.0f, 0.0f, 0.0f), NULL, lineResult);
        EATESTAssert(hit, "Expected line volume hit");
        EATESTAssert(IsSimilar(lineResult.lineParam, 0.0f), "Expected LineParam of 0");
        EATESTAssert(IsSimilar(lineResult.normal, Vector3(0.0f, 1.0f, 0.0f)), "Unexpected normal");
        EATESTAssert(IsSimilar(lineResult.position, lineStart), "Expected position equal to lineStart");
        EATESTAssert(IsSimilar(lineResult.volParam, GetVector3_Zero()), "Expected Zero volParam");
        EATESTAssert(lineResult.v == vol, "Expected volume pointer");
    }
}

void TestLineQueryPrimitives::CylinderTest()
{
    //Create CapsuleVolume of radius 5 units and halflength 5 units
    CylinderVolume * vol = EA::Physics::UnitFramework::Creator<CylinderVolume>().New(5.0f, 5.0f);

    //Set transform position.
    Vector3 position = Vector3(10.0f, -10.0f, 0.0f);
    Matrix44Affine mtx(vol->GetLocalTransform());
    mtx.SetW(position);
    vol->SetLocalTransform(mtx);

    //Test Line hits
    {
        VolumeLineSegIntersectResult lineResult;
        RwpBool hit;
        Vector3 lineStart = Vector3(10.0f, 0.0f, 0.0f);
        Vector3 lineUnitDirection = Normalize(position - lineStart);
        hit = vol->LineSegIntersect(lineStart, position, NULL, lineResult);
        EATESTAssert(hit, "Expected line volume intersection");
        EATESTAssert(IsSimilar(lineResult.lineParam, 0.5f), "Expected LineParam of 0.5f");
        EATESTAssert(IsSimilar(lineResult.normal, -lineUnitDirection), "Expected 'Up' normal");
        EATESTAssert(IsSimilar(lineResult.position, lineStart + 0.5f * lineUnitDirection * Magnitude(position - lineStart)), "Unexpected Position of intersection");
        EATESTAssert(IsSimilar(lineResult.volParam, Vector3(0.0f, 0.0f, 0.0f)), "Unexpected volParam");
        EATESTAssert(lineResult.v == vol, "Expected volume pointer");
    }

    //Test line misses
    {
        VolumeLineSegIntersectResult lineResult;
        RwpBool hit;
        hit = vol->LineSegIntersect(Vector3(10.0f, 0.0f, 0.0f), Vector3(10.0f, 10.0f, 0.0f), NULL, lineResult);
        EATESTAssert(hit ? false : true, "Expected line volume miss");
        EATESTAssert(IsSimilar(lineResult.lineParam, 0.0f), "Expected LineParam of 0");
        EATESTAssert(IsSimilar(lineResult.normal, GetVector3_Zero()), "Expected Zero normal");
        EATESTAssert(IsSimilar(lineResult.position, GetVector3_Zero()), "Expected Zero position");
        EATESTAssert(IsSimilar(lineResult.volParam, GetVector3_Zero()), "Expected Zero volParam");
        EATESTAssert(lineResult.v == vol, "Expected volume pointer");
    }

    //Test line starts inside
    {
        VolumeLineSegIntersectResult lineResult;
        RwpBool hit;
        Vector3 lineStart = position + Vector3(0.0f, 0.1f, 0.0f);
        hit = vol->LineSegIntersect(lineStart, Vector3(10.0f, 0.0f, 0.0f), NULL, lineResult);
        EATESTAssert(hit, "Expected line volume hit");
        EATESTAssert(IsSimilar(lineResult.lineParam, 0.0f), "Expected LineParam of 0");
        EATESTAssert(IsSimilar(lineResult.normal, Vector3(0.0f, 1.0f, 0.0f)), "Unexpected normal");
        EATESTAssert(IsSimilar(lineResult.position, lineStart), "Expected position equal to lineStart");
        EATESTAssert(IsSimilar(lineResult.volParam, GetVector3_Zero()), "Expected Zero volParam");
        EATESTAssert(lineResult.v == vol, "Expected volume pointer");
    }
}

