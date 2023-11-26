// (c) Electronic Arts. All Rights Reserved.

#include <unit/unit.h>

#include <EABase/eabase.h>

#include <rw/collision/libcore.h>
#include <rw/collision/detail/fpu/aabbox.h>

#include <serialization/serialization.h>
#include <serialization/binary_stream_oarchive.h>
#include <serialization/binary_stream_iarchive.h>

#include "testsuitebase.h" // For TestSuiteBase

#include "SimpleStream.hpp"


using namespace rwpmath;
using namespace rw::collision;



class TestAABBox : public tests::TestSuiteBase
{
public:
    virtual void Initialize()
    {
        SuiteName("TestAABBox");
        EATEST_REGISTER("TestVectorConstructor", "TestVectorConstructor", TestAABBox, TestVectorConstructor);
        EATEST_REGISTER("TestFloatConstructor", "TestFloatConstructor", TestAABBox, TestFloatConstructor);
        EATEST_REGISTER("TestValid", "TestValid", TestAABBox, TestValid);
        EATEST_REGISTER("TestScale", "TestScale", TestAABBox, TestScale);
        EATEST_REGISTER("TestOverlaps", "TestOverlaps", TestAABBox, TestOverlaps);
        EATEST_REGISTER("TestDistance", "TestDistance", TestAABBox, TestDistance);
        EATEST_REGISTER("TestContains", "TestContains", TestAABBox, TestContains);
        EATEST_REGISTER("TestTransform", "TestTransform", TestAABBox, TestTransform);
        EATEST_REGISTER("TestUnion", "TestUnion", TestAABBox, TestUnion);
        EATEST_REGISTER("TestPad", "Test ComputePadded()", TestAABBox, TestPad);
        EATEST_REGISTER("TestSerialization", "Serialization of rw::collision::AABBox", TestAABBox, TestSerialization);
#if !defined(RWP_NO_VPU_MATH)
        EATEST_REGISTER("TestSerializationOfFpuLayout", "Serialization of fpu layout rw::collision::AABBox", TestAABBox, TestSerializationOfFpuLayout);
#endif // !defined(RWP_NO_VPU_MATH)
    }

    static bool CompareAABBoxes(const rw::collision::AABBox& original, const rw::collision::AABBox& copied)
    {
        return ((original.Min() == copied.Min()) && (original.Max() == copied.Max()));
    }


private:
    void TestVectorConstructor();
    void TestFloatConstructor();
    void TestValid();
    void TestScale();
    void TestOverlaps();
    void TestDistance();
    void TestContains();
    void TestTransform();
    void TestUnion();
    void TestPad();
    void TestSerialization();
#if !defined(RWP_NO_VPU_MATH)
    void TestSerializationOfFpuLayout();
#endif // !defined(RWP_NO_VPU_MATH)

} TestAABBoxSingleton;


void TestAABBox::TestVectorConstructor()
{
    rw::collision::AABBox basicPositive(Vector3(0.0f, 0.0f, 0.0f), Vector3(1.0f, 1.0f, 1.0f));
    EATESTAssert(basicPositive.Min() == Vector3(0.0f, 0.0f, 0.0f), "Basic Vector Positive Min");
    EATESTAssert(basicPositive.Max() == Vector3(1.0f, 1.0f, 1.0f), "Basic Vector Positive Max");

    rw::collision::AABBox basicNegative(Vector3(-1.0f, -1.0f, -1.0f), Vector3(0.0f, 0.0f, 0.0f));
    EATESTAssert(basicNegative.Min() == Vector3(-1.0f, -1.0f, -1.0f), "Basic Vector Negative Min");
    EATESTAssert(basicNegative.Max() == Vector3(0.0f, 0.0f, 0.0f), "Basic Vector Negative Max");
}

void TestAABBox::TestFloatConstructor()
{
    rw::collision::AABBox basicPositive(0.0f, 0.0f, 0.0f, 1.0f, 1.0f, 1.0f);
    EATESTAssert(basicPositive.Min() == Vector3(0.0f, 0.0f, 0.0f), "Basic Float Positive Min");
    EATESTAssert(basicPositive.Max() == Vector3(1.0f, 1.0f, 1.0f), "Basic Float Positive Max");

    rw::collision::AABBox basicNegative(-1.0f, -1.0f, -1.0f, 0.0f, 0.0f, 0.0f);
    EATESTAssert(basicNegative.Min() == Vector3(-1.0f, -1.0f, -1.0f), "Basic Float Negative Min");
    EATESTAssert(basicNegative.Max() == Vector3(0.0f, 0.0f, 0.0f), "Basic Float Negative Max");
}


void TestAABBox::TestValid()
{
    rw::collision::AABBox validBox(Vector3(0.0f, 0.0f, 0.0f), Vector3(3.0f, 3.0f, 3.0f));
    EATESTAssert(validBox.IsValid(), "Valid Box");
    rw::collision::AABBox invalidBox(Vector3(3.0f, 3.0f, 3.0f), Vector3(0.0f, 0.0f, 0.0f));
    EATESTAssert(!invalidBox.IsValid(), "Invalid Box");
}

void TestAABBox::TestScale()
{
    rw::collision::AABBox preScale(Vector3(-10.0f, -10.0f, -10.0f), Vector3(10.0f, 10.0f, 10.0f));
    rw::collision::AABBox postScale = preScale.Scale(0.5);
    EATESTAssert(postScale.Min() == Vector3(-5.0f, -5.0f, -5.0f), "Scale Min");
    EATESTAssert(postScale.Max() == Vector3(5.0f, 5.0f, 5.0f), "Scale Max");
}

void TestAABBox::TestOverlaps()
{
    rw::collision::AABBox bbox1(Vector3(5.0f, 5.0f, 5.0f), Vector3(10.0f, 10.0f, 10.0f));
    rw::collision::AABBox bbox2(Vector3(7.0f, 7.0f, 7.0f), Vector3(12.0f, 12.0f, 12.0f));
    rw::collision::AABBox bbox3(Vector3(0.0f, 0.0f, 0.0f), Vector3(5.0f, 5.0f, 5.0f));
    EATESTAssert(bbox1.Overlaps(bbox2), "Overlapping");
    EATESTAssert(!bbox2.Overlaps(bbox3), "Non-Overlapping");
    EATESTAssert(bbox1.Overlaps(bbox3), "Touching");
}

void TestAABBox::TestDistance()
{
    rw::collision::AABBox bbox1(Vector3(0.0f, 0.0f, 0.0f), Vector3(5.0f, 5.0f, 5.0f));
    rw::collision::AABBox bbox2(Vector3(5.0f, 5.0f, 6.0f), Vector3(5.0f, 5.0f, 12.0f));
    rw::collision::AABBox bbox3(Vector3(4.0f, 4.0f, 3.0f), Vector3(7.0f, 7.0f,7.0f));

    EATESTAssert(bbox1.Distance(bbox2) == 1.0f, "Distance = 1");
    EATESTAssert(bbox2.Distance(bbox1) == 1.0f, "Reverse Distance = 1");
    EATESTAssert(bbox1.Distance(bbox3) == -1.0f, "Penetration");
}

void TestAABBox::TestContains()
{
    rw::collision::AABBox bbox1(Vector3(0.0f, 0.0f, 0.0f), Vector3(5.0f, 5.0f, 5.0f));
    rw::collision::AABBox bbox2(Vector3(2.0f, 2.0f, 2.0f), Vector3(4.0f, 4.0f, 4.0f));
    rw::collision::AABBox bbox3(Vector3(7.0f, 7.0f, 7.0f), Vector3(10.0f, 10.0f, 10.0f));

    EATESTAssert(bbox1.Contains(bbox2), "Contains");
    EATESTAssert(!bbox2.Contains(bbox1), "Doesn't Contain");
    EATESTAssert(!bbox1.Contains(bbox3), "Doesn't Contain");

    Vector3 pt1(3.0f, 3.0f, 4.5f);
    EATESTAssert(bbox1.Contains(pt1), "Contains");
    EATESTAssert(!bbox2.Contains(pt1), "Doesn't Contain");

}

void TestAABBox::TestTransform()
{
    rw::collision::AABBox bbox1(Vector3(0.0f, 0.0f, 0.0f), Vector3(5.0f, 5.0f, 5.0f));
    rw::collision::AABBox bbox2(Vector3(0.0f, 0.0f, 0.0f), Vector3(5.0f, 1.0f, 1.0f));

    Matrix44Affine translate = rwpmath::Matrix44AffineFromTranslation(Vector3(1.0f, 2.0f, 3.0f));
    Matrix44Affine scale     = rwpmath::Matrix44AffineFromScale(Vector3(1.0f, 2.0f, 3.0f));
    Matrix44Affine rotate    = rwpmath::Matrix44AffineFromZRotationAngle(PI/2.0f);

    rw::collision::AABBox translated = bbox1.Transform(&translate);
    rw::collision::AABBox scaled     = bbox1.Transform(&scale);
    rw::collision::AABBox rotated    = bbox2.Transform(&rotate);

    EATESTAssert(translated.Min() == Vector3(1.0f, 2.0f, 3.0f) && translated.Max() == Vector3(6.0f, 7.0f, 8.0f), "Translate");
    EATESTAssert(scaled.Min() == Vector3(0.0f, 0.0f, 0.0f) && scaled.Max() == Vector3(5.0f, 10.0f, 15.0f), "Scale");
    EATESTAssert(IsSimilar(rotated.Min(), Vector3(-1.0f, 0.0f, 0.0f)) && IsSimilar(rotated.Max(), Vector3(0.0f, 5.0f, 1.0f)), "Rotate");
}

void TestAABBox::TestUnion()
{
    rw::collision::AABBox bbox1(Vector3(0.0f, 0.0f, 0.0f), Vector3(5.0f, 5.0f, 5.0f));
    rw::collision::AABBox bbox2(Vector3(5.0f, 5.0f, 5.0f), Vector3(10.0f, 10.0f, 10.0f));


    rw::collision::AABBox unionBox = Union(bbox1, bbox2);

    EATESTAssert(unionBox.Min() == Vector3(0.0f, 0.0f, 0.0f) && unionBox.Max() == Vector3(10.0f, 10.0f, 10.0f), "Union");

    unionBox = bbox2;
    unionBox.Union(bbox1);
    EATESTAssert(unionBox.Min() == Vector3(0.0f, 0.0f, 0.0f) && unionBox.Max() == Vector3(10.0f, 10.0f, 10.0f), "Union");

    unionBox = bbox1;
    unionBox.Union(Vector3(-2.0f, 8.0f, 9.0f));
    EATESTAssert(unionBox.Min() == Vector3(-2.0f, 0.0f, 0.0f) && unionBox.Max() == Vector3(5.0f, 8.0f, 9.0f), "Union");

    unionBox = Union(bbox1, Vector3(-2.0f, 8.0f, 9.0f));
    EATESTAssert(unionBox.Min() == Vector3(-2.0f, 0.0f, 0.0f) && unionBox.Max() == Vector3(5.0f, 8.0f, 9.0f), "Union");

}

void TestAABBox::TestPad()
{
    rw::collision::AABBox bbox(Vector3(0.0f, 0.0f, 0.0f), Vector3(5.0f, 5.0f, 5.0f));
    
    rwpmath::VecFloat pad(0.3f);
    rwpmath::Vector3 expectedMin = bbox.Min() - rwpmath::Vector3(pad, pad, pad);
    rwpmath::Vector3 expectedMax = bbox.Max() + rwpmath::Vector3(pad, pad, pad);

    rw::collision::AABBox paddedBox = ComputePadded(bbox, pad);
    EATESTAssert(paddedBox.Min() == expectedMin, "Min padded as expected");
    EATESTAssert(paddedBox.Max() == expectedMax, "Max padded as expected");
}

void TestAABBox::TestSerialization()
{
    const uint32_t bufferSize = 256u;
#if RWPMATH_IS_VPU
    uint8_t EA_PREFIX_ALIGN(16) buffer[bufferSize] EA_POSTFIX_ALIGN(16);
#else
    uint8_t EA_PREFIX_ALIGN(4) buffer[bufferSize] EA_POSTFIX_ALIGN(4);
#endif

    rw::collision::AABBox original(Vector3(-1.0f, -2.0f, -3.0f), Vector3(4.0f, 5.0f, 6.0f));
    {
        SimpleStream strm(buffer, bufferSize);
        EA::Serialization::basic_binary_stream_oarchive<SimpleStream, EA::Serialization::Endian::LittleEndianConverter> oArchive(strm);
        oArchive & EA_SERIALIZATION_NAMED_VALUE(original);
        EATESTAssert(oArchive.Close(), "Failure during serialization of rw::collision::AABBox.");
    }

    rw::collision::AABBox copied;
    {
        SimpleStream strm(buffer, bufferSize);
        EA::Serialization::basic_binary_stream_iarchive<SimpleStream, EA::Serialization::Endian::LittleEndianConverter> iArchive(strm);
        iArchive & EA_SERIALIZATION_NAMED_VALUE(copied);
        EATESTAssert(iArchive.Close(), "Failure during deserialization of rw::collision::AABBox.");
    }

    EATESTAssert(CompareAABBoxes(original, copied), "Original and serialized copies do not match.");
}


#if !defined(RWP_NO_VPU_MATH)

void TestAABBox::TestSerializationOfFpuLayout()
{
    const uint32_t bufferSize = 256u;
    uint8_t EA_PREFIX_ALIGN(16) buffer[bufferSize] EA_POSTFIX_ALIGN(16);

    rw::collision::AABBox original(Vector3(-1.0f, -2.0f, -3.0f), Vector3(4.0f, 5.0f, 6.0f));
    {
        // serialize into buffer
        SimpleStream strm(buffer, bufferSize);
        EA::Serialization::basic_binary_stream_oarchive<SimpleStream, EA::Serialization::Endian::LittleEndianConverter> oArchive(strm);
        oArchive & EA_SERIALIZATION_NAMED_VALUE(original);
        EATESTAssert(oArchive.Close(), "Failure during serialization of rw::collision::AABBox.");
    }

    rw::collision::detail::fpu::AABBox fpuCopy;
    {
        // serialize out of buffer into fpu layout version
        SimpleStream strm(buffer, bufferSize);
        EA::Serialization::basic_binary_stream_iarchive<SimpleStream, EA::Serialization::Endian::LittleEndianConverter> iArchive(strm);
        iArchive & EA_SERIALIZATION_NAMED_VALUE(fpuCopy);
        EATESTAssert(iArchive.Close(), "Failure during deserialization of fpu copy of rw::collision::AABBox.");
    }

    {
        // serialize into buffer
        SimpleStream strm(buffer, bufferSize);
        EA::Serialization::basic_binary_stream_oarchive<SimpleStream, EA::Serialization::Endian::LittleEndianConverter> oArchive(strm);
        oArchive & EA_SERIALIZATION_NAMED_VALUE(fpuCopy);
        EATESTAssert(oArchive.Close(), "Failure during serialization of fpu copy of rw::collision::AABBox.");
    }

    rw::collision::AABBox copied;
    {
        // serialize out of buffer back into vpu layout version
        SimpleStream strm(buffer, bufferSize);
        EA::Serialization::basic_binary_stream_iarchive<SimpleStream, EA::Serialization::Endian::LittleEndianConverter> iArchive(strm);
        iArchive & EA_SERIALIZATION_NAMED_VALUE(copied);
        EATESTAssert(iArchive.Close(), "Failure during deserialization of fpu copy of rw::collision::AABBox.");
    }

    EATESTAssert(CompareAABBoxes(original, copied), "Original and serialized copies do not match.");
}

#endif // !defined(RWP_NO_VPU_MATH)
