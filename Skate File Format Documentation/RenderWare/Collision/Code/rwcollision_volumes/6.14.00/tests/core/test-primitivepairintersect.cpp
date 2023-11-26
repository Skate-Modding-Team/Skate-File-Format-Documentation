// (c) Electronic Arts. All Rights Reserved.

#include <unit/unit.h>

#include <EABase/eabase.h>
#include <eaphysics/base.h>

#include <rw/collision/libcore.h>
#include <eacollision_features/version.h>

#include "testsuitebase.h" // For TestSuiteBase

#include "stdio.h"          // for sprintf()

using namespace rwpmath;
using namespace rw::collision;


class TestPrimitivePairIntersect : public tests::TestSuiteBase
{
public:
    virtual void Initialize()
    {
        SuiteName("TestPrimitivePairIntersect");

        EATEST_REGISTER("TestBatch1xN", "Test PrimitivePair intersect for 1xN Batches",
                        TestPrimitivePairIntersect, TestBatch1xN);
        EATEST_REGISTER("TestBatchNxM", "Test PrimitivePair intersect for NxM Batches",
                        TestPrimitivePairIntersect, TestBatchNxM);
        EATEST_REGISTER("TestCapsuleEndCaps", "Check contacts with disabled capsule vertices are discarded", 
                        TestPrimitivePairIntersect, TestCapsuleEndCaps);
    }

    void SetupSuite()
    {
        tests::TestSuiteBase::SetupSuite();

        // Initialise the collision system
        Volume::InitializeVTable();
    }

    void TeardownSuite()
    {
        tests::TestSuiteBase::TeardownSuite();
    }

    void TestExpectedPair(const rw::collision::Volume & volumeA, const rw::collision::Volume & volumeB, float padding, uint32_t numExpectedContacts);

    void TestBatch1xN();
    void TestBatchNxM();
    void TestCapsuleEndCaps();

} TestPrimitivePairIntersectSingleton;


void TestPrimitivePairIntersect::TestBatch1xN()
{
    Volume vols1[1];
    SphereVolume::Initialize(&vols1[0], 0.5f);
    Matrix44Affine transforms1[] =
    {
        Matrix44Affine(GetVector3_XAxis(), GetVector3_YAxis(), GetVector3_ZAxis(), Vector3(1.0f, 0.0f, 0.0f)),
    };

    Volume vols2[3];
    SphereVolume::Initialize(&vols2[0], 1.0f);
    SphereVolume::Initialize(&vols2[1], 0.1f);
    SphereVolume::Initialize(&vols2[2], 2.0f);
    Matrix44Affine transforms2[] =
    {
        Matrix44Affine(GetVector3_XAxis(), GetVector3_YAxis(), GetVector3_ZAxis(), Vector3(1.3f, 0.0f, 0.0f)),
        Matrix44Affine(GetVector3_XAxis(), GetVector3_YAxis(), GetVector3_ZAxis(), Vector3(2.0f, 0.0f, 0.0f)),
        Matrix44Affine(GetVector3_XAxis(), GetVector3_YAxis(), GetVector3_ZAxis(), Vector3(2.3f, 0.0f, 0.0f)),
    };

    PrimitivePairIntersectResult results[10];
    GPInstance instancingSPR[10];

    int32_t numResults = rw::collision::detail::PrimitiveBatchIntersect1xN(results, 10, instancingSPR, vols1, transforms1, vols2, transforms2, 3);
    EATESTAssert(numResults == 2, "Number of results incorrect");

    // alternative calling parameters
    const Volume* v2[] = { &vols2[0], &vols2[1], &vols2[2] };
    const Matrix44Affine* t2[] = { &transforms2[0], &transforms2[1], &transforms2[2] };
    int numResults2 = rw::collision::detail::PrimitiveBatchIntersect1xN(results, 10, instancingSPR, vols1, transforms1, v2, t2, 3);
    EATESTAssert(numResults2 == 2, "Number of results incorrect");
}


void TestPrimitivePairIntersect::TestBatchNxM()
{
    Volume vols1[2];
    SphereVolume::Initialize(&vols1[0], 0.5f);
    SphereVolume::Initialize(&vols1[1], 1.0f);
    Matrix44Affine transforms1[] =
    {
        Matrix44Affine(GetVector3_XAxis(), GetVector3_YAxis(), GetVector3_ZAxis(), Vector3(1.0f, 0.0f, 0.0f)),
        Matrix44Affine(GetVector3_XAxis(), GetVector3_YAxis(), GetVector3_ZAxis(), Vector3(5.0f, 0.0f, 0.0f)),
    };
    const Volume* v1[] = { &vols1[0], &vols1[1] };
    const Matrix44Affine* t1[] = { &transforms1[0], &transforms1[1] };

    Volume vols2[3];
    SphereVolume::Initialize(&vols2[0], 1.0f);
    SphereVolume::Initialize(&vols2[1], 0.1f);
    SphereVolume::Initialize(&vols2[2], 2.0f);
    Matrix44Affine transforms2[] =
    {
        Matrix44Affine(GetVector3_XAxis(), GetVector3_YAxis(), GetVector3_ZAxis(), Vector3(1.3f, 0.0f, 0.0f)),
        Matrix44Affine(GetVector3_XAxis(), GetVector3_YAxis(), GetVector3_ZAxis(), Vector3(2.0f, 0.0f, 0.0f)),
        Matrix44Affine(GetVector3_XAxis(), GetVector3_YAxis(), GetVector3_ZAxis(), Vector3(2.3f, 0.0f, 0.0f)),
    };
    const Volume* v2[] = { &vols2[0], &vols2[1], &vols2[2] };
    const Matrix44Affine* t2[] = { &transforms2[0], &transforms2[1], &transforms2[2] };

    PrimitivePairIntersectResult results[10];
    GPInstance instancingSPR[10];

    int32_t numResults = rw::collision::detail::PrimitiveBatchIntersectNxM(results, 10, instancingSPR, v1, t1, 2, v2, t2, 3);
    EATESTAssert(numResults == 3, "Number of results incorrect");
}

void TestPrimitivePairIntersect::TestExpectedPair(const rw::collision::Volume & volumeA, const rw::collision::Volume & volumeB, float padding, uint32_t numExpectedContacts)
{
    PrimitivePairIntersectResult result;
    result.numPoints = 0;
    Matrix44Affine identity = rwpmath::GetMatrix44Affine_Identity();
    if (rw::collision::detail::PrimitivePairIntersect(result, &volumeA, &identity, &volumeB, &identity, padding))
    {
        char msg[256];
        sprintf(msg, "Expected %d contacts between %d and %d but found %d", numExpectedContacts, volumeA.GetType(), volumeB.GetType(), result.numPoints);
        EATESTAssert(result.numPoints == numExpectedContacts, msg);
    }
    else
    {
        char msg[256];
        sprintf(msg, "Didn't hit, but expected contacts %d contacts between %d and %d", numExpectedContacts, volumeA.GetType(), volumeB.GetType());
        EATESTAssert(0 == numExpectedContacts, msg);
    }
}

static void SetCenterAndZAxis(rw::collision::Volume & volume, rwpmath::Vector3::InParam center, rwpmath::Vector3::InParam zaxis = rwpmath::GetVector3_ZAxis())
{
    rwpmath::Matrix44Affine tm = volume.GetLocalTransform();
    tm.SetW(center);
    tm.SetZ(zaxis);
    volume.SetLocalTransform(tm);
}

void TestPrimitivePairIntersect::TestCapsuleEndCaps()
{
    // Create pairs between a capsule in 3 orientations against each prim type
    static const uint32_t NUMPRIMS = 5;
    rw::collision::Volume capsules[NUMPRIMS][3];
    rw::collision::Volume others[NUMPRIMS][3];

    float r = 0.5f;             // capsule radius
    float hh = 0.8f;            // capsule half height
    float spacing = 10.0f;      // between each pair
    float gap = 2.0f;           // between the volumes in the pair
    float padding = 3.0f*gap;   // to ensure we have a hit

    rwpmath::Vector3 yaxis = rwpmath::GetVector3_YAxis();
    rwpmath::Vector3 zaxis = rwpmath::GetVector3_ZAxis();

    for (uint32_t i = 0; i < NUMPRIMS; ++i)
    {
        float x = float(NUMPRIMS/2) - float(i);

        rwpmath::Vector3 centers[3];
        centers[0] = rwpmath::Vector3(x*spacing, -spacing, -gap);
        centers[1] = rwpmath::Vector3(x*spacing, spacing, -gap);
        centers[2] = rwpmath::Vector3(x*spacing, 0.0f, -gap);

        // 3 test capsules oriented to produce one contact point with other prim
        for (int j = 0; j < 3; ++j)
        {
            rw::collision::CapsuleVolume::Initialize(EA::Physics::MemoryPtr(&capsules[i][j]), r, hh);
        }
        SetCenterAndZAxis(capsules[i][0], centers[0], -zaxis);
        SetCenterAndZAxis(capsules[i][1], centers[1], zaxis);
        SetCenterAndZAxis(capsules[i][2], centers[2], yaxis);

        // 3 primitives to test against these, oriented to generate a single contact
        for (int j = 0; j < 3; ++j)
        {
            rwpmath::Vector3 centerB = centers[j] + zaxis*2.0f*gap;
            switch (i)
            {
            case 0:
                rw::collision::SphereVolume::Initialize(EA::Physics::MemoryPtr(&others[i][j]), r);
                SetCenterAndZAxis(others[i][j], centerB);
                break;
            case 1:
                rw::collision::CapsuleVolume::Initialize(EA::Physics::MemoryPtr(&others[i][j]), r, 1.0f);
                SetCenterAndZAxis(others[i][j], centerB, zaxis);
                break;
            case 2:
                rw::collision::CylinderVolume::Initialize(EA::Physics::MemoryPtr(&others[i][j]), r, 0.6f*r);
                SetCenterAndZAxis(others[i][j], centerB, rwpmath::Normalize(zaxis + yaxis));
                break;
            case 3:
                rw::collision::TriangleVolume::Initialize(EA::Physics::MemoryPtr(&others[i][j]), centerB, centerB+zaxis, centerB+yaxis+zaxis, r);
                break;
            case 4:
                {
                    // rotate so we only generate one contact
                    rwpmath::Matrix44Affine tm = rwpmath::Matrix44AffineFromEulerXYZ(rwpmath::Vector3(rwpmath::PI/4.0f, rwpmath::PI/4.0f, 0.0f));
                    tm.SetW(centerB);
                    rw::collision::BoxVolume::Initialize(EA::Physics::MemoryPtr(&others[i][j]), rwpmath::Vector3(r*1.5f, r*1.3f, r*1.7f), r);
                    others[i][j].SetLocalTransform(tm);
                }
                break;
            }
        }
    }

    // Try all combination of end cap removals possibly on both capsules in pair
    bool disabled[] = { false, false, false };
    const uint32_t arrayCount(static_cast<uint32_t>(EAArrayCount(disabled)));
    for (uint32_t k = 0; k < 1<<arrayCount; ++k)
    {
        for (uint32_t j = 0; j < arrayCount; ++j)
        {
            disabled[j] = ((k & (1<<j)) != 0);
        }
        for (uint32_t i = 0; i < NUMPRIMS; ++i)
        {
            for (int j = 0; j < 3; ++j)
            {
                rw::collision::CapsuleVolume * cap = static_cast<rw::collision::CapsuleVolume *>(&capsules[i][j]);
                cap->SetEndCap0Disabled(disabled[0]);
                cap->SetEndCap1Disabled(disabled[1]);
                if (rw::collision::VOLUMETYPECAPSULE == others[i][j].GetType())
                {
                    rw::collision::CapsuleVolume * other = static_cast<rw::collision::CapsuleVolume *>(&others[i][j]);
                    other->SetEndCap0Disabled(disabled[2]);
                    other->SetEndCap1Disabled(disabled[2]);
                }
            }
        }

        for (uint32_t i = 0; i < NUMPRIMS; ++i)
        {
#if EACOLLISION_FEATURES_VERSION >= EACOLLISION_FEATURES_CREATE_VERSION_NUMBER(1,7,00)
            bool alldisabled = (rw::collision::VOLUMETYPECAPSULE == others[i][0].GetType()) && disabled[2];
            uint32_t expected0 = disabled[0] || alldisabled ? 0u : 1u;
            uint32_t expected1 = disabled[1] || alldisabled ? 0u : 1u;
#else
            bool alldisabled = false;
            uint32_t expected0 = 1u;
            uint32_t expected1 = 1u;
#endif

            TestExpectedPair(capsules[i][0], others[i][0], padding, expected0);
            TestExpectedPair(capsules[i][1], others[i][1], padding, expected1);
            TestExpectedPair(capsules[i][2], others[i][2], padding, alldisabled ? 0u : 1u);
        }
    }
}



