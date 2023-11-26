// (c) Electronic Arts. All Rights Reserved.

#include <unit/unit.h>

#include <EABase/eabase.h>

#include <eaphysics/base.h>

#include <rw/collision/libcore.h>

#include "testsuitebase.h" // For TestSuiteBase

using namespace rwpmath;
using namespace rw::collision;

namespace rw
{
namespace collision
{
namespace test
{

    class VTableTestVolume : public rw::collision::Volume
    {
    private:

        static rw::collision::Volume::VTable s_vTableTestVolumeVTable;

        // Note the VTable member functions are private to stop you from being able to call them directly.

        RwpBool GetBBox(const rwpmath::Matrix44Affine *, RwpBool, AABBox &) const
        {
            m_getBBox = true;
            return TRUE;
        }

        rwpmath::Vector3 GetBBoxDiag() const
        {
            m_getBBoxDiag = true;
            return rwpmath::Vector3(rwpmath::PI, rwpmath::PI, rwpmath::PI);
        }

        RwpBool LineSegIntersect(rwpmath::Vector3::InParam,
                                  rwpmath::Vector3::InParam,
                                  const rwpmath::Matrix44Affine *,
                                  VolumeLineSegIntersectResult &,
                                  const float) const
        {
            m_lineSegIntersect = true;
            return TRUE;
        }

        void Release()
        {
            m_released = true;
        }

        RwpBool GetMoments(rwpmath::Matrix44 &) const
        {
            m_getMoments = true;
            return TRUE;
        }

        void GetAsTriangles(void *, TriangleCallback) const
        {
            m_getAsTriangles = true;
        }

        void ClearAllProcessedFlags()
        {
            m_clearAllProcessedFlags = true;
        }

        void ApplyUniformScale(float, bool)
        {
            m_applyUniformScale = true;
        }

    public:

        VTableTestVolume()
            : Volume(rw::collision::VOLUMETYPECUSTOM)
            , m_getBBox(false)
            , m_getBBoxDiag(false)
            , m_lineSegIntersect(false)
            , m_released(false)
            , m_getMoments(false)
            , m_getAsTriangles(false)
            , m_clearAllProcessedFlags(false)
            , m_applyUniformScale(false)
        {
        }

        static EA::Physics::SizeAndAlignment GetResourceDescriptor()
        {
            EA::Physics::SizeAndAlignment al(sizeof(VTableTestVolume), EA_ALIGN_OF(VTableTestVolume));
            return al;
        }

        static VTableTestVolume * Initialize(const EA::Physics::MemoryPtr &resource)
        {
            rwcASSERTALIGN(resource.GetMemory(), rwcVOLUMEALIGNMENT);
            return new (resource.GetMemory()) VTableTestVolume();
        }

        static void InitializeVTableToNull()
        {
            s_vTableTestVolumeVTable.typeID = s_vTableTestVolumeType;
            s_vTableTestVolumeVTable.getBBox = 0;
            s_vTableTestVolumeVTable.getBBoxDiag = 0;
            s_vTableTestVolumeVTable.getInterval = 0;
            s_vTableTestVolumeVTable.getMaximumFeature = 0;
            s_vTableTestVolumeVTable.createGPInstance = 0;
            s_vTableTestVolumeVTable.lineSegIntersect = 0;
            s_vTableTestVolumeVTable.release = 0;
            s_vTableTestVolumeVTable.name = s_vTableTestVolumeTypeName,
            s_vTableTestVolumeVTable.flags = 0;
            s_vTableTestVolumeVTable.getMoments = 0;
            s_vTableTestVolumeVTable.getAsTriangles = 0;
            s_vTableTestVolumeVTable.clearAllProcessedFlags = 0;
            s_vTableTestVolumeVTable.applyUniformScale = 0;

            rw::collision::Volume::vTableArray[rw::collision::VOLUMETYPECUSTOM] = &s_vTableTestVolumeVTable;
        }

        static void InitializeVTable()
        {
            s_vTableTestVolumeVTable.typeID = s_vTableTestVolumeType;
            s_vTableTestVolumeVTable.getBBox = (Volume::GetBBoxFn)(&rw::collision::test::VTableTestVolume::GetBBox);
            s_vTableTestVolumeVTable.getBBoxDiag = (Volume::GetBBoxDiagFn)(&rw::collision::test::VTableTestVolume::GetBBoxDiag);
            s_vTableTestVolumeVTable.getInterval = 0;
            s_vTableTestVolumeVTable.getMaximumFeature = 0;
            s_vTableTestVolumeVTable.createGPInstance = 0;
            s_vTableTestVolumeVTable.lineSegIntersect = (Volume::LineSegIntersectFn)(&rw::collision::test::VTableTestVolume::LineSegIntersect);
            s_vTableTestVolumeVTable.release = (Volume::ReleaseFn)(&rw::collision::test::VTableTestVolume::Release);
            s_vTableTestVolumeVTable.name = s_vTableTestVolumeTypeName,
            s_vTableTestVolumeVTable.flags = 0;
            s_vTableTestVolumeVTable.getMoments = (Volume::GetMomentsFn)(&rw::collision::test::VTableTestVolume::GetMoments);
            s_vTableTestVolumeVTable.getAsTriangles = (Volume::GetAsTrianglesFn)(&rw::collision::test::VTableTestVolume::GetAsTriangles);
            s_vTableTestVolumeVTable.clearAllProcessedFlags = (Volume::ClearAllProcessedFlagsFn)(&rw::collision::test::VTableTestVolume::ClearAllProcessedFlags);
            s_vTableTestVolumeVTable.applyUniformScale = (Volume::ApplyUniformScaleFn)(&rw::collision::test::VTableTestVolume::ApplyUniformScale);

            rw::collision::Volume::vTableArray[rw::collision::VOLUMETYPECUSTOM] = &s_vTableTestVolumeVTable;
        }

    public:

        mutable bool m_getBBox;
        mutable bool m_getBBoxDiag;
        mutable bool m_lineSegIntersect;
        bool m_released;
        mutable bool m_getMoments;
        mutable bool m_getAsTriangles;
        bool m_clearAllProcessedFlags;
        bool m_applyUniformScale;

        static const char * s_vTableTestVolumeTypeName;
        static const rw::collision::VolumeType s_vTableTestVolumeType;
    };

    rw::collision::Volume::VTable VTableTestVolume::s_vTableTestVolumeVTable;

    const char * VTableTestVolume::s_vTableTestVolumeTypeName = "VTableTestVolume";
    const rw::collision::VolumeType VTableTestVolume::s_vTableTestVolumeType = rw::collision::VOLUMETYPECUSTOM;

}
}
}

class TestVolume: public tests::TestSuiteBase
{
public:
    virtual void Initialize()
    {
        SuiteName("TestVolume");

        EATEST_REGISTER("TestProcessingFlags", "Test API for setting/clearing processing flags",
                         TestVolume, TestProcessingFlags);
        EATEST_REGISTER("TestNullVTableEntries", "Test API functions that cope with null VTable entries",
            TestVolume, TestNullVTableEntries);
        EATEST_REGISTER("TestVTable", "Test API function that are in the VTable",
            TestVolume, TestVTable);

        EATEST_REGISTER_SPU("TestVolume", "SPU Volume tests", "test-volume.elf");
    }

    void SetupSuite()
    {
        tests::TestSuiteBase::SetupSuite();
    }

    void TeardownSuite()
    {
        tests::TestSuiteBase::TeardownSuite();
    }

private:

    void TestProcessingFlags();
    void TestNullVTableEntries();
    void TestVTable();

} TestVolumeSingleton;


void TestVolume::TestProcessingFlags()
{
    rw::collision::Volume volume;
    rw::collision::Volume *volumePtr = &volume;

    // These call generic volume functions that are applicable to all primitive types
    volumePtr->SetFlags(!VOLUMEFLAG_ISPROCESSED);
    EATESTAssert(!(volumePtr->GetFlags() & VOLUMEFLAG_ISPROCESSED), "VOLUMEFLAG_ISPROCESSED should not be set");
    volumePtr->SetProcessedFlag();
    EATESTAssert(volumePtr->GetFlags() & VOLUMEFLAG_ISPROCESSED, "SetProcessedFlag() failed");

    volumePtr->ClearProcessedFlag();
    EATESTAssert(!(volumePtr->GetFlags() & VOLUMEFLAG_ISPROCESSED), "ClearProcessedFlag() failed");
}


// Test calling VTable functions that cope with NULL entries in the VTable.
void TestVolume::TestNullVTableEntries()
{
    // Initialize the test volumes VTable to have null entries and then create a test volume.
    test::VTableTestVolume::InitializeVTableToNull();
    rw::collision::test::VTableTestVolume vTableTestVolume;

    // Assign vTableTestVolume to volume so that the class member functons will get called from the base class via the vTable.
    // Note the member functions in VTableTestVolume are private to stop you from being able to call the directly.
    rw::collision::Volume * volume = &vTableTestVolume;

    rwpmath::Matrix44 moments(rwpmath::GetMatrix44_Identity());
    EATESTAssert(false == vTableTestVolume.m_getMoments, "The m_getMoments member value should initially be false");
    EATESTAssert(FALSE == volume->GetMoments(moments), "Calling GetMoments should return FALSE");
    EATESTAssert(false == vTableTestVolume.m_getMoments, "Calling GetMoments should not set the member m_getMoments to true");

    EATESTAssert(false == vTableTestVolume.m_getAsTriangles, "The m_getAsTriangles member value should initially be false");
    volume->GetAsTriangles(0, 0);
    EATESTAssert(false == vTableTestVolume.m_getAsTriangles, "Calling GetAsTriangles should not set the member m_getAsTriangles to true");

    EATESTAssert(false == vTableTestVolume.m_clearAllProcessedFlags, "The m_clearAllProcessedFlags member value should initially be false");
    volume->ClearAllProcessedFlags();
    EATESTAssert(false == vTableTestVolume.m_clearAllProcessedFlags, "Calling ClearAllProcessedFlags should not set the member m_clearAllProcessedFlags to true");

    EATESTAssert(false == vTableTestVolume.m_applyUniformScale, "The m_applyUniformScale member value should initially be false");
    volume->ApplyUniformScale(1.0f, false);
    EATESTAssert(false == vTableTestVolume.m_applyUniformScale, "Calling ApplyUniformScale should not set the member m_applyUniformScale to true");

    EATESTAssert(0 == strcmp(rw::collision::test::VTableTestVolume::s_vTableTestVolumeTypeName, volume->GetTypeName()), "Incorrect type name returned from GetTypeName");
    EATESTAssert(rw::collision::test::VTableTestVolume::s_vTableTestVolumeType == volume->GetType(), "Incorrect type return from GetType");

    rw::collision::Volume::vTableArray[rw::collision::VOLUMETYPECUSTOM] = 0;
}


// Test calling all supported VTable functions
void TestVolume::TestVTable()
{
    // Initialize the test volumes VTable to have null entries and then create a test volume.
    test::VTableTestVolume::InitializeVTable();
    rw::collision::test::VTableTestVolume vTableTestVolume;

    // Assign vTableTestVolume to volume so that the class member functons will get called from the base class via the vTable.
    // Note the member functions in VTableTestVolume are private to stop you from being able to call the directly.
    rw::collision::Volume * volume = &vTableTestVolume;

    rw::collision::AABBox aabbox(rwpmath::GetVector3_Zero(), rwpmath::GetVector3_One());
    EATESTAssert(false == vTableTestVolume.m_getBBox, "The m_getBBox member value should initially be false");
    EATESTAssert(RwpBool(TRUE) == volume->GetBBox(0, FALSE, aabbox), "Calling GetBBox should return TRUE");
    EATESTAssert(true == vTableTestVolume.m_getBBox, "Calling GetBBox should set the m_getBBox member value to true");

    EATESTAssert(false == vTableTestVolume.m_getBBoxDiag, "The m_getBBoxDiag member value should initially be false");
    EATESTAssert(rwpmath::IsSimilar(rwpmath::Vector3(rwpmath::PI, rwpmath::PI, rwpmath::PI), volume->GetBBoxDiag()), "Calling GetBBoxDiag should return a vector of [0.5f, 0.5f, 0.5f]");
    EATESTAssert(true == vTableTestVolume.m_getBBoxDiag, "Calling GetBBoxDiag should set the m_getBBoxDiag member value to true");

    rwpmath::Vector3 pt0(rwpmath::GetVector3_Zero());
    rwpmath::Vector3 pt1(rwpmath::GetVector3_Zero());
    VolumeLineSegIntersectResult lineSegResult;
    EATESTAssert(false == vTableTestVolume.m_lineSegIntersect, "The m_lineSegIntersect member value should initially be false");
    EATESTAssert(RwpBool(TRUE) == volume->LineSegIntersect(pt0, pt1, 0,lineSegResult, 1.0f), "Calling LineSegIntersect should return TRUE");
    EATESTAssert(true == vTableTestVolume.m_lineSegIntersect, "Calling LineSegIntersect should set the m_lineSegIntersect member value to true");

    EATESTAssert(false == vTableTestVolume.m_released, "The m_released member value should initially be false");
    volume->Release();
    EATESTAssert(true == vTableTestVolume.m_released, "Calling Release should set the m_released member value to true");

    rwpmath::Matrix44 moments(rwpmath::GetMatrix44_Identity());
    EATESTAssert(false == vTableTestVolume.m_getMoments, "The m_getMoments member value should initially be false");
    EATESTAssert(RwpBool(TRUE) == volume->GetMoments(moments), "Calling GetMoments should return TRUE");
    EATESTAssert(true == vTableTestVolume.m_getMoments, "Calling GetAsTriangles should set the m_getMoments member value to true");

    EATESTAssert(false == vTableTestVolume.m_getAsTriangles, "The m_getAsTriangles member value should initially be false");
    volume->GetAsTriangles(0, 0);
    EATESTAssert(true == vTableTestVolume.m_getAsTriangles, "Calling GetAsTriangles should set the m_getAsTriangles member value to true");

    EATESTAssert(false == vTableTestVolume.m_clearAllProcessedFlags, "The m_clearAllProcessedFlags member value should initially be false");
    volume->ClearAllProcessedFlags();
    EATESTAssert(true == vTableTestVolume.m_clearAllProcessedFlags, "Calling ClearAllProcessedFlags should set the m_clearAllProcessedFlags member value to true");

    EATESTAssert(false == vTableTestVolume.m_applyUniformScale, "The m_applyUniformScale member value should initially be false");
    volume->ApplyUniformScale(1.0f, false);
    EATESTAssert(true == vTableTestVolume.m_applyUniformScale, "Calling ApplyUniformScale should set the m_applyUniformScale member value to true");

    EATESTAssert(0 == strcmp(rw::collision::test::VTableTestVolume::s_vTableTestVolumeTypeName, volume->GetTypeName()), "Incorrect type name returned from GetTypeName");
    EATESTAssert(rw::collision::test::VTableTestVolume::s_vTableTestVolumeType == volume->GetType(), "Incorrect type return from GetType");

    rw::collision::Volume::vTableArray[rw::collision::VOLUMETYPECUSTOM] = 0;
}
