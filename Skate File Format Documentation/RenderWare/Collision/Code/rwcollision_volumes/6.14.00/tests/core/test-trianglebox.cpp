// (c) Electronic Arts. All Rights Reserved.
#ifdef _MSC_VER
#pragma warning(disable: 4700)
#endif

#include <new>

#include <EABase/eabase.h>
#include <rw/math/math.h>
#include <rw/collision/libcore.h>

#include <unit/unit.h>

#include "testsuitebase.h" // For TestSuiteBase

using namespace rwpmath;
using namespace rw::collision;


// ***********************************************************************************************************
// Test suite

class TestTriangleBox : public tests::TestSuiteBase
{
public:
    virtual void Initialize()
    {
        SuiteName("TestTriangleBox");

        EATEST_REGISTER("TestChrisStottsScenario", "TestChrisStottsScenario", TestTriangleBox, TestChrisStottsScenario);
    }

private:
    void TestChrisStottsScenario();

} TestTriangleBoxSingleton;



void TestTriangleBox::TestChrisStottsScenario()
{

#if 0  // DEBUGGING!!!

    // why is this not using GPInstance::Initialize
    GPInstance::ContactPoints contactPoints;
    GPInstance partInstance;
    partInstance.mPos=rwpmath::Vector3(-1.6998540163f,0.3184999824f,-8.3706521988f); // [(bfd994d1),(3ea3126e),(c105ee31)]
    partInstance.mFaceNormals[0]=rwpmath::Vector3(1.0000000000f,-0.0000000005f,-0.0000000442f); // [(3f800000),(b0043c1c),(b33ddc54)]
    partInstance.mFaceNormals[1]=rwpmath::Vector3(0.0000000005f,1.0000000000f,0.0000000001f); // [(30043bc0),(3f800000),(2f08ec00)]
    partInstance.mFaceNormals[2]=rwpmath::Vector3(0.0000000442f,-0.0000000001f,1.0000000000f); // [(333ddc52),(af08ea00),(3f800000)]
    partInstance.mEdgeDirections[0]=rwpmath::Vector3(1.0000000000f,-0.0000000005f,-0.0000000442f); // [(3f800000),(b0043c1c),(b33ddc54)]
    partInstance.mEdgeDirections[1]=rwpmath::Vector3(0.0000000005f,1.0000000000f,0.0000000001f); // [(30043bc0),(3f800000),(2f08ec00)]
    partInstance.mEdgeDirections[2]=rwpmath::Vector3(0.0000000442f,-0.0000000001f,1.0000000000f); // [(333ddc52),(af08ea00),(3f800000)]
    partInstance.mDimensions=rwpmath::Vector3(0.2784999907f,0.3184999824f,0.2890000045f); // [(3e8e978d),(3ea3126e),(3e93f7cf)]
    partInstance.mFatness=0.0000000000f; // (00000000)
    partInstance.mVolumeTag=1159007888; // (45150e90)
    partInstance.mUserTag=0; // (00000000)
    partInstance.mNumFaceNormals=3; // (03)
    partInstance.mNumEdgeDirections=3; // (03)
    partInstance.mVolumeType=(GPInstance::VolumeType)4; // (00000004)
    partInstance.mFlags=1; // (00000001)
    partInstance.mEdgeData[0]=0.0000000000f; // (00000000)
    partInstance.mEdgeData[1]=560.2390747070f; // (440c0f4d)
    partInstance.mEdgeData[2]=0.0000000000f; // (00000000)
#if defined(RW_COLLISION_GP_INSTANCE_EMBED_FUNCTION_PTRS)
    partInstance.mVolumeMethods=GPInstance::sVolumeMethods[GPInstance::BOX];
#endif // defined(RW_COLLISION_GP_INSTANCE_EMBED_FUNCTION_PTRS)

    // why is this not using GPTriangle::Initialize
    GPTriangle triangleInstance;
    triangleInstance.mPos=rwpmath::Vector3(0.0000000000f,0.0000000000f,0.0000000000f); // [(00000000),(00000000),(00000000)]
    triangleInstance.mFaceNormals[0]=rwpmath::Vector3(-0.0000000000f,1.0000001192f,0.0000000000f); // [(80000000),(3f800001),(00000000)]
    triangleInstance.mFaceNormals[1]=rwpmath::Vector3(0.0000000000f,0.0000000000f,-58.9620018005f); // [(00000000),(00000000),(c26bd917)]
    triangleInstance.mFaceNormals[2]=rwpmath::Vector3(-58.9620018005f,0.0000000000f,-58.9620018005f); // [(c26bd917),(00000000),(c26bd917)]
    triangleInstance.mEdgeDirections[0]=rwpmath::Vector3(-0.7071068287f,0.0000000000f,-0.7071068287f); // [(bf3504f4),(00000000),(bf3504f4)]
    triangleInstance.mEdgeDirections[1]=rwpmath::Vector3(1.0000000000f,0.0000000000f,0.0000000000f); // [(3f800000),(00000000),(00000000)]
    triangleInstance.mEdgeDirections[2]=rwpmath::Vector3(0.0000000000f,0.0000000000f,1.0000000000f); // [(00000000),(00000000),(3f800000)]
    triangleInstance.mDimensions=rwpmath::Vector3(83.3848571777f,58.9620018005f,58.9620018005f); // [(42a6c50c),(426bd917),(426bd917)]
    triangleInstance.mFatness=0.0000000000f; // (00000000)
    triangleInstance.mVolumeTag=0; // (00000000)
    triangleInstance.mUserTag=17039360; // (01040000)
    triangleInstance.mNumFaceNormals=1; // (01)
    triangleInstance.mNumEdgeDirections=3; // (03)
    triangleInstance.mVolumeType=(GPInstance::VolumeType)3; // (00000003)
    triangleInstance.mFlags=3857; // (00000f11)
    triangleInstance.mEdgeData[0]=1.0000000000f; // (3f800000)
    triangleInstance.mEdgeData[1]=1.0000000000f; // (3f800000)
    triangleInstance.mEdgeData[2]=1.0000000000f; // (3f800000)
#if defined(RW_COLLISION_GP_INSTANCE_EMBED_FUNCTION_PTRS)
    triangleInstance.mVolumeMethods=GPInstance::sVolumeMethods[GPInstance::TRIANGLE];
#endif // defined(RW_COLLISION_GP_INSTANCE_EMBED_FUNCTION_PTRS)

    float partPadding=0.0299999993f; // (3cf5c28f)
    float edgeCosBendNormalThreshold=0.9990000129f; // (3f7fbe77)
    float convexityEpsilon=0.0099999998f; // (3c23d70a)
    uint32_t numContacts = ComputeContactPoints(partInstance,triangleInstance,partPadding,contactPoints,edgeCosBendNormalThreshold,convexityEpsilon);

    EATESTAssert(numContacts == 4, "Wrong number of contacts");

#endif

}


