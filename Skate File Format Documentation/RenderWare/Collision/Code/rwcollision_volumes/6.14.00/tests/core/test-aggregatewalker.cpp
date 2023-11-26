// (c) Electronic Arts. All Rights Reserved.

#include <unit/unit.h>

#include <EABase/eabase.h>
#include <eaphysics/base.h>
#include <rw/collision/libcore.h>

#include <eaphysics/unitframework/allocator.h> // For ResetAllocator
#include <eaphysics/unitframework/creator.h> // For Creator

#include "testsuitebase.h" // For TestSuiteBase

using namespace rw::collision;


class TestAggregateWalker: public tests::TestSuiteBase
{
public:

    virtual void Initialize()
    {
        SuiteName("TestAggregateWalker");
        EATEST_REGISTER("Construction",     "Tests Construction",        TestAggregateWalker, Construction);
        EATEST_REGISTER("Validity",         "Tests Validity",            TestAggregateWalker, Validity);
        EATEST_REGISTER("Finished",         "Tests Finished State",      TestAggregateWalker, Finished);
        EATEST_REGISTER("ChildIndex",       "Tests Child Index",         TestAggregateWalker, ChildIndex);
        EATEST_REGISTER("VolumePointer",    "Tests VolumePointer",       TestAggregateWalker, VolumePointer);
        EATEST_REGISTER("UserData",         "Tests UserData",            TestAggregateWalker, UserData);
        EATEST_REGISTER("VolumeInstance",   "Tests VolumeInstance",      TestAggregateWalker, VolumeInstance);
        EATEST_REGISTER("IsVolumeInstance", "Tests IsVolumeInstance",    TestAggregateWalker, IsVolumeInstance);
        EATEST_REGISTER("Simple",           "Tests Simple Walker",       TestAggregateWalker, Simple);
        EATEST_REGISTER("Recursive",        "Tests Recursive Walker",    TestAggregateWalker, Recursive);
        EATEST_REGISTER("Reuse",            "Tests Reuse of Walker",     TestAggregateWalker, Reuse);
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

    typedef void (TestAggregateWalker::*VolumeCallbackFn)(void *data, const Volume *volume);

    void CountVolumeCallback(void *data, const Volume *volume);
    void IterateAggregateVolumes(const Aggregate &aggregate, void *data, TestAggregateWalker::VolumeCallbackFn volumeCallback);

private:

    void Construction();
    void Validity();
    void Finished();
    void ChildIndex();
    void VolumePointer();
    void UserData();
    void VolumeInstance();
    void IsVolumeInstance();
    void Simple();
    void Recursive();
    void Reuse();

} TestAggregateWalkerSingleton;



class TestAggregate : public Aggregate
{
private:

    SimpleMappedArray * m_simpleMappedArray;

    struct VolumeWalkerData
    {
        uint32_t m_nextVolumeIndex;
    };

public:

    TestAggregate(SimpleMappedArray * simpleMappedArray, uint32_t numVolumes, VTable *vTable)
        : Aggregate(numVolumes, vTable),
          m_simpleMappedArray(simpleMappedArray)
    {
    }

    RwpBool GetNextVolume(VolumeWalker & volumeWalker) const
    {
        VolumeWalkerData *walker(reinterpret_cast<VolumeWalkerData *>(volumeWalker.GetUserData()));

        if (!volumeWalker.IsValid())
        {
            walker->m_nextVolumeIndex = 0;
            volumeWalker.SetValid();
        }

        if (walker->m_nextVolumeIndex < m_simpleMappedArray->GetVolumeCount())
        {
            volumeWalker.SetVolumePointer(&m_simpleMappedArray->GetVolumeArray()[walker->m_nextVolumeIndex]);
            volumeWalker.SetChildIndex(walker->m_nextVolumeIndex);
            ++walker->m_nextVolumeIndex;
            return TRUE;
        }

        volumeWalker.SetFinished();
        return FALSE;
    }

};


static Aggregate::VTable s_TestAggregateVTable =
{
    RWCOBJECTTYPE_NA,                                                           //rw::collision::ObjectType m_type;
    0,                                                                          //GetSizeFn                 m_GetSize;
    rwcSIMPLEMAPPEDARRAYALIGNMENT,                                              //uint32_t                  m_alignment;
    FALSE,                                                                      //RwpBool                   m_isProcedural;
    0,                                                                          //UpdateFn                  m_Update;
    0,                                                                          //LineIntersectionQueryFn   m_LineIntersectionQuery;
    0,                                                                          //BBoxOverlapQueryFn        m_BBoxOverlapQuery;
    static_cast<Aggregate::GetNextVolumeFn>(&TestAggregate::GetNextVolume),     //GetNextVolumeFn           m_GetNextVolume;
};


void TestAggregateWalker::Construction()
{
    Aggregate::VolumeWalker volumeWalker;
    EATESTAssert(!volumeWalker.IsValid(), "VolumeWalker is valid but should be invalid");
}


void TestAggregateWalker::Validity()
{
    Aggregate::VolumeWalker volumeWalker;
    EATESTAssert(!volumeWalker.IsValid(), "VolumeWalker is valid but should be invalid");
    volumeWalker.SetValid();
    EATESTAssert(volumeWalker.IsValid(), "VolumeWalker is invalid but should invalid");
    volumeWalker.SetInvalid();
    EATESTAssert(!volumeWalker.IsValid(), "VolumeWalker is valid but should be invalid");
}


void TestAggregateWalker::Finished()
{
    Aggregate::VolumeWalker volumeWalker;
    // Need to set the volume walker as valid to call Finished
    volumeWalker.SetValid();
    EATESTAssert(!volumeWalker.Finished(), "VolumeWalker is finished but shouldn't be");
    volumeWalker.SetFinished();
    EATESTAssert(volumeWalker.Finished(), "VolumeWalker is not finished but should be");
}


void TestAggregateWalker::ChildIndex()
{
    Aggregate::VolumeWalker volumeWalker;
    EATESTAssert(!volumeWalker.IsValid(), "VolumeWalker is valid but should be invalid");
    volumeWalker.SetChildIndex(0);
    EATESTAssert(0 == volumeWalker.GetChildIndex(), "Child index should be set to 0 but isn't");
    volumeWalker.SetChildIndex(0x1234);
    EATESTAssert(0x1234 == volumeWalker.GetChildIndex(), "Child index should be set to 0x1234 but isn't");
}


void TestAggregateWalker::VolumePointer()
{
    Aggregate::VolumeWalker volumeWalker;
    EATESTAssert(!volumeWalker.IsValid(), "VolumeWalker is valid but should be invalid");

    Volume sphereVolume;
    SphereVolume::Initialize(&sphereVolume, 1.0f);

    volumeWalker.SetVolumePointer(0);
    volumeWalker.SetVolumePointer(&sphereVolume);
    volumeWalker.SetChildIndex(0);

    EATESTAssert(&*volumeWalker == &sphereVolume, "VolumeWalker volume pointer is not correct");
    EATESTAssert(VOLUMETYPESPHERE == volumeWalker->GetType(), "Volume is not of type VOLUMETYPESPHERE");
    EATESTAssert(1.0f == volumeWalker->GetRadius(), "Volumes radius is incorrect");
    EATESTAssert(volumeWalker->GetLocalTransform() == (*volumeWalker).GetLocalTransform(), "Volume relative transform does not match");
}


void TestAggregateWalker::UserData()
{
    Aggregate::VolumeWalker volumeWalker;
    EATESTAssert(!volumeWalker.IsValid(), "VolumeWalker is valid but should be invalid");

    EATESTAssert(volumeWalker.GetUserData(), "User data is null");

    const uint32_t magicNumber(0xAAAAAAAA);
    uint32_t *userData(reinterpret_cast<uint32_t *>(volumeWalker.GetUserData()));
    *userData = magicNumber;

    EATESTAssert(magicNumber == *reinterpret_cast<uint32_t *>(volumeWalker.GetUserData()), "User data is incorrect");
}


void TestAggregateWalker::VolumeInstance()
{
    Aggregate::VolumeWalker volumeWalker;
    EATESTAssert(!volumeWalker.IsValid(), "VolumeWalker is valid but should be invalid");

    SphereVolume::Initialize(volumeWalker.GetVolumeInstance(), 1.0f);
    EATESTAssert(VOLUMETYPESPHERE == volumeWalker.GetVolumeInstance()->GetType(), "Volume is not of type VOLUMETYPESPHERE");
    EATESTAssert(1.0f == volumeWalker.GetVolumeInstance()->GetRadius(), "Volume radius is incorrect");

    const rwpmath::Vector3 boxDimensions(2.0f, 3.0f, 4.0f);
    BoxVolume::Initialize(volumeWalker.GetVolumeInstance(), boxDimensions, 0.5f);
    EATESTAssert(VOLUMETYPEBOX == volumeWalker.GetVolumeInstance()->GetType(), "Volume is not of type VOLUMETYPESPHERE");
    BoxVolume *boxVolume(static_cast<BoxVolume*>(volumeWalker.GetVolumeInstance()));
    EATESTAssert(IsSimilar(boxDimensions ,boxVolume->GetDimensions()), "Volume dimensions are incorrect");
    EATESTAssert(0.5f == boxVolume->GetRadius(), "Volume radius is incorrect");
}


void TestAggregateWalker::IsVolumeInstance()
{
    Aggregate::VolumeWalker volumeWalker;
    EATESTAssert(!volumeWalker.IsValid(), "VolumeWalker is valid but should be invalid");

    // Set the walker to be valid so the * and -> operators don't assert.
    volumeWalker.SetValid();

    // Set a valid child index
    volumeWalker.SetChildIndex(0);
    SphereVolume::Initialize(volumeWalker.GetVolumeInstance(), 1.0f);
    volumeWalker.SetVolumePointer(volumeWalker.GetVolumeInstance());
    EATESTAssert(volumeWalker.IsVolumeInstanced(), "Volume is not instanced but should be");

    const SphereVolume * sphereVolume(EA::Physics::UnitFramework::Creator<SphereVolume>().New(1.0f));
    volumeWalker.SetVolumePointer(&*sphereVolume);
    EATESTAssert(!volumeWalker.IsVolumeInstanced(), "Volume is instanced but should not be");
}


void TestAggregateWalker::Simple()
{
    // Create a simple mapped array
    const uint32_t numVolumes(16);
    SimpleMappedArray * simpleMappedArray(EA::Physics::UnitFramework::Creator<SimpleMappedArray>().New(numVolumes));

    EATESTAssert(numVolumes == simpleMappedArray->GetVolumeCount(), "Volume counts do not match");

    // Initialzie the volumes in the simple mapped array
    for (uint32_t i(0); i < simpleMappedArray->GetVolumeCount(); ++i)
    {
        SphereVolume::Initialize(&simpleMappedArray->GetVolumeArray()[i], static_cast<float>(i));
    }

    // Create a TestAggregate
    TestAggregate testAggregate(&*simpleMappedArray, 0, &s_TestAggregateVTable);

    uint32_t countedVolumes(0);
    Aggregate::VolumeWalker volumeWalker(&testAggregate);
    while (!volumeWalker.Finished())
    {
        EATESTAssert(volumeWalker.IsValid(), "VolumeWalker is not valid");
        EATESTAssert(VOLUMETYPESPHERE == volumeWalker->GetType(), "Volume is not of type VOLUMETYPESPHERE");
        EATESTAssert(static_cast<float>(countedVolumes) == volumeWalker->GetRadius(), "Volumes radius is incorrect");

        ++countedVolumes;
        ++volumeWalker;
    }

    EATESTAssert(numVolumes == countedVolumes, "Volume counts do not match");
    EATESTAssert(volumeWalker.IsValid(), "VolumeWalker is invalid but should be valid");
    EATESTAssert(volumeWalker.Finished(), "VolumeWalker is not finished but should be");
}


void TestAggregateWalker::CountVolumeCallback(void *data, const Volume * /*volume*/)
{
    ++(*reinterpret_cast<uint32_t *>(data));
}


void TestAggregateWalker::IterateAggregateVolumes(const Aggregate &aggregate, void *data, TestAggregateWalker::VolumeCallbackFn volumeCallback)
{
    Aggregate::VolumeWalker volumeWalker(&aggregate);

    while (!volumeWalker.Finished())
    {
        const Volume &nextVolume(*volumeWalker);

        if (VOLUMETYPEAGGREGATE == nextVolume.GetType())
        {
            const Aggregate &nextAggregate(*static_cast<const AggregateVolume *>(&nextVolume)->GetAggregate());
            IterateAggregateVolumes(nextAggregate, data, volumeCallback);
        }
        else
        {
            (this->*(volumeCallback))(data, &nextVolume);
        }

        ++volumeWalker;
    }
}


void TestAggregateWalker::Recursive()
{
    // Create a simple mapped array
    const uint32_t numVolumes(8);
    SimpleMappedArray * simpleMappedArray(EA::Physics::UnitFramework::Creator<SimpleMappedArray>().New(numVolumes));
    SimpleMappedArray * childSimpleMappedArrays[numVolumes];

    EATESTAssert(numVolumes == simpleMappedArray->GetVolumeCount(), "Volume counts do not match");

    // Initialize the volumes in the simple mapped array
    for (uint32_t i(0); i < simpleMappedArray->GetVolumeCount(); ++i)
    {
        // Create the child simple mapped array
        childSimpleMappedArrays[i] = EA::Physics::UnitFramework::Creator<SimpleMappedArray>().New(numVolumes);

        // Initialzie the volumes in the child simple mapped array
        for (uint32_t j(0); j < childSimpleMappedArrays[i]->GetVolumeCount(); ++j)
        {
            SphereVolume::Initialize(&childSimpleMappedArrays[i]->GetVolumeArray()[j], 1.0f);
        }

        // Initialize the simple mapped array entry as an AggregateVolume refering to a simple mapped array.
        AggregateVolume::Initialize(&simpleMappedArray->GetVolumeArray()[i], &*childSimpleMappedArrays[i]);
    }

    uint32_t countedVolumes(0);
    IterateAggregateVolumes(*simpleMappedArray, &countedVolumes, &TestAggregateWalker::CountVolumeCallback);
    EATESTAssert(numVolumes * numVolumes == countedVolumes, "Volume counts do not match");
}


void TestAggregateWalker::Reuse()
{
    // Create a simple mapped array
    const uint32_t numVolumes(16);
    SimpleMappedArray * simpleMappedArray(EA::Physics::UnitFramework::Creator<SimpleMappedArray>().New(numVolumes));

    EATESTAssert(numVolumes == simpleMappedArray->GetVolumeCount(), "Volume counts do not match");

    // Initialzie the volumes in the simple mapped array
    for (uint32_t i(0); i < simpleMappedArray->GetVolumeCount(); ++i)
    {
        SphereVolume::Initialize(&simpleMappedArray->GetVolumeArray()[i], static_cast<float>(i));
    }

    // Create a TestAggregate
    TestAggregate testAggregate(&*simpleMappedArray, 0, &s_TestAggregateVTable);

    uint32_t countedVolumes(0);
    Aggregate::VolumeWalker volumeWalker(&testAggregate);
    while (!volumeWalker.Finished())
    {
        EATESTAssert(volumeWalker.IsValid(), "VolumeWalker is not valid");
        EATESTAssert(VOLUMETYPESPHERE == volumeWalker->GetType(), "Volume is not of type VOLUMETYPESPHERE");
        EATESTAssert(static_cast<float>(countedVolumes) == volumeWalker->GetRadius(), "Volumes radius is incorrect");

        ++countedVolumes;
        ++volumeWalker;
    }

    EATESTAssert(numVolumes == countedVolumes, "Volume counts do not match");
    EATESTAssert(volumeWalker.IsValid(), "VolumeWalker is invalid but should be valid");
    EATESTAssert(volumeWalker.Finished(), "VolumeWalker is not finished but should be");

    // Reuse
    countedVolumes = 0;
    volumeWalker.Initialize(&testAggregate);
    while (!volumeWalker.Finished())
    {
        EATESTAssert(volumeWalker.IsValid(), "VolumeWalker is not valid");
        EATESTAssert(VOLUMETYPESPHERE == volumeWalker->GetType(), "Volume is not of type VOLUMETYPESPHERE");
        EATESTAssert(static_cast<float>(countedVolumes) == volumeWalker->GetRadius(), "Volumes radius is incorrect");

        ++countedVolumes;
        ++volumeWalker;
    }

    EATESTAssert(numVolumes == countedVolumes, "Volume counts do not match");
    EATESTAssert(volumeWalker.IsValid(), "VolumeWalker is invalid but should be valid");
    EATESTAssert(volumeWalker.Finished(), "VolumeWalker is not finished but should be");
}

