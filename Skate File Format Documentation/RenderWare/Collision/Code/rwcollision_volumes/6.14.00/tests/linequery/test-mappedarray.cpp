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


class TestLineQueryMappedArray : public tests::TestSuiteBase
{
public:
    virtual void Initialize()
    {
        SuiteName("TestLineQueryMappedArray");

        // Test individual Methods
        EATEST_REGISTER("SMASphereTest", "SMASphereTest", TestLineQueryMappedArray, SMASphereTest);
        EATEST_REGISTER("KDTreeMASphereTest", "KDTreeMASphereTest", TestLineQueryMappedArray, KDTreeMASphereTest);
        EATEST_REGISTER("SMABBoxCullingTest", "SMABBoxCullingTest", TestLineQueryMappedArray, SMABBoxCullingTest);
        EATEST_REGISTER("KDTreeMABBoxCullingTest", "KDTreeMABBoxCullingTest", TestLineQueryMappedArray, KDTreeMABBoxCullingTest);
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
    void SMASphereTest();
    void SMABBoxCullingTest();
    void KDTreeMABBoxCullingTest();
    void KDTreeMASphereTest();

    void AggregateSphereTest(const Volume& vol);
    void AggregateCullingTest(const Volume& vol);

    SimpleMappedArray * CreateSMA(const Volume& vol);
    KDTreeMappedArray * CreateKDTreeMA(const Volume& vol);

} TestLineQueryMappedArraySingleton;

SimpleMappedArray * TestLineQueryMappedArray::CreateSMA(const Volume& vol)
{
    SimpleMappedArray * sma = EA::Physics::UnitFramework::Creator<SimpleMappedArray>().New(3u);
    Volume* vols = sma->GetVolumeArray();
    vols[0] = vol;
    vols[1] = vol;
    rwpmath::Matrix44Affine trans(rwpmath::Matrix44AffineFromYRotationAngle(1.0f));
    trans.SetW(3.0f * rwpmath::GetVector3_YAxis());
    vols[1].SetLocalTransform(trans);
    vols[2] = vol;
    trans = rwpmath::Matrix44AffineFromZRotationAngle(1.0f);
    trans.SetW(3.0f * rwpmath::GetVector3_XAxis());
    vols[2].SetLocalTransform(trans);
    sma->Update();
    return sma;
}

KDTreeMappedArray * TestLineQueryMappedArray::CreateKDTreeMA(const Volume& vol)
{
    const uint32_t numVols = 3;
    Volume vols[numVols];
    vols[0] = vol;
    vols[1] = vol;
    rwpmath::Matrix44Affine trans(rwpmath::Matrix44AffineFromYRotationAngle(1.0f));
    trans.SetW(3.0f * rwpmath::GetVector3_YAxis());
    vols[1].SetLocalTransform(trans);
    vols[2] = vol;
    trans = rwpmath::Matrix44AffineFromZRotationAngle(1.0f);
    trans.SetW(3.0f * rwpmath::GetVector3_XAxis());
    vols[2].SetLocalTransform(trans);

    // Build a set of bboxes to generate our kdtree
    AABBoxU kdtreeBBoxes[numVols];
    for (uint32_t i = 0; i < numVols; ++i)
    {
        rw::collision::AABBox bbox;
        vols[i].GetBBox(0, TRUE, bbox);
        kdtreeBBoxes[i].Set(rw::math::fpu::Vector3(bbox.Min()), rw::math::fpu::Vector3(bbox.Max()));
    }

    KDTreeBuilder kdtreeBuilder(*EA::Allocator::ICoreAllocator::GetDefaultAllocator());
    kdtreeBuilder.BuildTree(numVols, kdtreeBBoxes, numVols);

    KDTreeMappedArray * kdtreeMappedArray = EA::Physics::UnitFramework::Creator<KDTreeMappedArray>().New(numVols, kdtreeBuilder.GetNumBranchNodes(), kdtreeBuilder.GetRootBBox());

    // Initialize the volumes in the simple mapped array
    for (uint32_t i = 0; i < kdtreeMappedArray->GetVolumeCount(); ++i)
    {
        kdtreeMappedArray->GetVolumeArray()[i] = vols[i];
    }

    kdtreeMappedArray->Update();

    return kdtreeMappedArray;
}

Volume::VTable fakedSphereVTableLineQuery = 
{
    rw::collision::VOLUMETYPESPHERE,
    (Volume::GetBBoxFn)(&SphereVolume::GetBBox),
    (Volume::GetBBoxDiagFn)(&SphereVolume::GetBBoxDiag),
    0,      // formerly GetInterval
    0,      // formerly GetMaximumFeature
    (Volume::CreateGPInstanceFn)(&SphereVolume::CreateGPInstance),
    (Volume::LineSegIntersectFn)(&SphereVolume::LineSegIntersect),
    (Volume::ReleaseFn)(&SphereVolume::Release),
    "SphereVolume",
    0,
    0,
    0,
};

class FakeSphereVolume : public Volume
{
public:

    static FakeSphereVolume* Initialize(const EA::Physics::MemoryPtr& resource, float radius);

    RwpBool
    FakeLineSegIntersect(rwpmath::Vector3::InParam pt1,
    rwpmath::Vector3::InParam pt2,
    const rwpmath::Matrix44Affine *tm,
    VolumeLineSegIntersectResult &result,
    const float fatness=0.0f) const
    {
        ++s_numQueryCalls;
        return LineSegIntersect(pt1, pt2, tm, result, fatness);
    }

    static uint32_t s_numQueryCalls;

protected:
    FakeSphereVolume(float rad)
        : Volume(rw::collision::VOLUMETYPECUSTOM, rad)
    {

    }
};

uint32_t FakeSphereVolume::s_numQueryCalls = 0;



FakeSphereVolume* FakeSphereVolume::Initialize(const EA::Physics::MemoryPtr & resource, float radius)
{
    vTableArray[VOLUMETYPECUSTOM] = &fakedSphereVTableLineQuery;
    FakeSphereVolume * fakeVol = new (resource.GetMemory()) FakeSphereVolume(radius);    
    return fakeVol;
}

void
TestLineQueryMappedArray::SMASphereTest()
{
    Volume sphere;
    FakeSphereVolume::Initialize(&sphere, 1.0f);

    SimpleMappedArray * ma = CreateSMA(sphere);
    Volume agg;
    AggregateVolume::Initialize(&agg, &*ma);
    AggregateSphereTest(agg);
}

void
TestLineQueryMappedArray::KDTreeMASphereTest()
{
    Volume sphere;
    FakeSphereVolume::Initialize(&sphere, 1.0f);

    KDTreeMappedArray * ma = CreateKDTreeMA(sphere);
    Volume agg;
    AggregateVolume::Initialize(&agg, &*ma);
    AggregateSphereTest(agg);
}

void
TestLineQueryMappedArray::AggregateSphereTest(const Volume& agg)
{
    VolumeLineQuery * vlq = EA::Physics::UnitFramework::Creator<VolumeLineQuery>().New(128u, 128u);
    const Volume* aggPtr = &agg;

    // Hit the sphere at the origin from above - thin
    Vector3 lineStart(0,1.5f,0); // = rwpmath::GetVector3_ZAxis();     // TODO - this is a bug!!
    Vector3 lineEnd = rwpmath::GetVector3_Zero();

    vlq->InitQuery(&aggPtr, 0, 1, lineStart, lineEnd);
    VolumeLineSegIntersectResult *result = vlq->GetNearestIntersection();

    EATESTAssert(result != 0, "Line should intersect a sphere");

    // Hit the sphere at the origin from above - fat
    lineStart = Vector3(0,1.5f,0); // = rwpmath::GetVector3_ZAxis();     // TODO - this is a bug!!_ZAxis();
    lineEnd = rwpmath::GetVector3_Zero();

    vlq->InitQuery(&aggPtr, 0, 1, lineStart, lineEnd, 0.1f);
    result = vlq->GetNearestIntersection();

    EATESTAssert(result != 0, "Line should intersect a sphere");

    // Hit the sphere at the origin from above - very fat and translated
    lineStart = rwpmath::Vector3(-1.05f, 0.0f, 1.0f);
    lineEnd = rwpmath::Vector3(-1.05f, -0.0f, 0.0f);

    vlq->InitQuery(&aggPtr, 0, 1, lineStart, lineEnd, 0.1f);
    result = vlq->GetNearestIntersection();

    EATESTAssert(result != 0, "Line should intersect a sphere");

    // Hit the sphere at the origin from above - very fat
    lineStart = rwpmath::GetVector3_ZAxis();
    lineEnd = rwpmath::GetVector3_Zero();

    vlq->InitQuery(&aggPtr, 0, 1, lineStart, lineEnd, 10.0f);
    result = vlq->GetNearestIntersection();

    EATESTAssert(result != 0, "Line should intersect a sphere");

    // Hit the sphere at the origin from above - very fat and translated
    lineStart = rwpmath::Vector3(-5.0f, -5.0f, 1.0f);
    lineEnd = rwpmath::Vector3(-5.0f, -5.0f, 0.0f);

    vlq->InitQuery(&aggPtr, 0, 1, lineStart, lineEnd, 10.0f);
    result = vlq->GetNearestIntersection();

    EATESTAssert(result != 0, "Line should intersect a sphere");

    // Miss all the spheres - thin
    lineStart = rwpmath::Vector3(-1.05f, 0.0f, 1.0f);
    lineEnd = rwpmath::Vector3(-1.05f, 0.0f, 0.0f);

    vlq->InitQuery(&aggPtr, 0, 1, lineStart, lineEnd);
    result = vlq->GetNearestIntersection();

    EATESTAssert(result == 0, "Line should not intersect a sphere");
    
    // Miss all the spheres - fat
    lineStart = rwpmath::Vector3(-1.15f, 0.0f, 1.0f);
    lineEnd = rwpmath::Vector3(-1.15f, 0.0f, 0.0f);

    vlq->InitQuery(&aggPtr, 0, 1, lineStart, lineEnd, 0.1f);
    result = vlq->GetNearestIntersection();

    EATESTAssert(result == 0, "Line should not intersect a sphere");

    // Miss all the spheres - very fat
    lineStart = rwpmath::Vector3(-12.0f, 0.0f, 1.0f);
    lineEnd = rwpmath::Vector3(-12.0f, 0.0f, 0.0f);

    vlq->InitQuery(&aggPtr, 0, 1, lineStart, lineEnd, 10.0f);
    result = vlq->GetNearestIntersection();

    EATESTAssert(result == 0, "Line should not intersect a sphere");

    // Hit all the spheres - very fat
    FakeSphereVolume::s_numQueryCalls = 0;
    lineStart = rwpmath::Vector3(0.0f, 0.0f, 20.0f);
    lineEnd = rwpmath::Vector3(0.0f, 0.0f, 0.0f);

    vlq->InitQuery(&aggPtr, 0, 1, lineStart, lineEnd, 10.0f);
    uint32_t numHits = vlq->GetAllIntersections();

    EATESTAssert(numHits == 3, "Line should not intersect a sphere");

}

void
TestLineQueryMappedArray::SMABBoxCullingTest()
{
    Volume sphere;
    FakeSphereVolume::Initialize(&sphere, 1.0f);

    SimpleMappedArray * ma = CreateSMA(sphere);
    Volume agg;
    AggregateVolume::Initialize(&agg, &*ma);
    AggregateCullingTest(agg);
}

void
TestLineQueryMappedArray::KDTreeMABBoxCullingTest()
{
    Volume sphere;
    FakeSphereVolume::Initialize(&sphere, 1.0f);

    KDTreeMappedArray * ma = CreateKDTreeMA(sphere);
    Volume agg;
    AggregateVolume::Initialize(&agg, &*ma);
    AggregateCullingTest(agg);
}

void
TestLineQueryMappedArray::AggregateCullingTest(const Volume& agg)
{
    VolumeLineQuery * vlq =EA::Physics::UnitFramework::Creator<VolumeLineQuery>().New(128u, 128u);
    const Volume* aggPtr = &agg;

    // Miss all the spheres - thin
    FakeSphereVolume::s_numQueryCalls = 0;
    Vector3 lineStart = rwpmath::Vector3(1.05f, 0.0f, 1.0f);
    Vector3 lineEnd = rwpmath::Vector3(1.05f, 0.0f, 0.0f);

    vlq->InitQuery(&aggPtr, 0, 1, lineStart, lineEnd);
    VolumeLineSegIntersectResult *result = vlq->GetNearestIntersection();

    EATESTAssert(result == 0, "Line should not intersect a sphere");
    EATESTAssert(FakeSphereVolume::s_numQueryCalls == 0, "No queries on spheres should be performed");

    // Miss all the spheres - fat
    FakeSphereVolume::s_numQueryCalls = 0;
    lineStart = rwpmath::Vector3(1.15f, 0.0f, 1.0f);
    lineEnd = rwpmath::Vector3(1.15f, 0.0f, 0.0f);

    vlq->InitQuery(&aggPtr, 0, 1, lineStart, lineEnd, 0.1f);
    result = vlq->GetNearestIntersection();

    EATESTAssert(result == 0, "Line should not intersect a sphere");
    EATESTAssert(FakeSphereVolume::s_numQueryCalls == 0, "No queries on spheres should be performed");

    // Miss all the spheres - very fat
    FakeSphereVolume::s_numQueryCalls = 0;
    lineStart = rwpmath::Vector3(-12.0f, 0.0f, 1.0f);
    lineEnd = rwpmath::Vector3(-12.0f, 0.0f, 0.0f);

    vlq->InitQuery(&aggPtr, 0, 1, lineStart, lineEnd, 10.0f);
    result = vlq->GetNearestIntersection();

    EATESTAssert(result == 0, "Line should not intersect a sphere");
    EATESTAssert(FakeSphereVolume::s_numQueryCalls == 0, "No queries on spheres should be performed");

}
