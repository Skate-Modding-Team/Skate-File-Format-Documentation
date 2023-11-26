// (c) Electronic Arts. All Rights Reserved.

#include <unit/unit.h>

#include <EABase/eabase.h>
#include <rw/collision/libcore.h>

#include "testsuitebase.h" // For TestSuiteBase

using namespace rw::collision;


class TestAggregateTag: public tests::TestSuiteBase
{
public:

    virtual void Initialize()
    {
        SuiteName("TestAggregateTag");
        EATEST_REGISTER("NumTagBits",       "Tests Tags",                TestAggregateTag, NumTagBits);
        EATEST_REGISTER("TagExtration",     "Tests Tags",                TestAggregateTag, TagExtration);
        EATEST_REGISTER("TagBuilding",      "Tests Tags",                TestAggregateTag, TagBuilding);
    }

    void SetupSuite()
    {
        tests::TestSuiteBase::SetupSuite();
        // Initialise the collision system
        Volume::InitializeVTable();
    }

private:

    void CheckTag(uint32_t aggregateSize, uint32_t index, uint32_t parentTag);
    void CheckTag2(uint32_t aggregateSize1, uint32_t index1, 
        uint32_t aggregateSize2, uint32_t index2, 
        uint32_t aggregateSize3, uint32_t index3);

    void TagExtration();
    void NumTagBits();
    void TagBuilding();

} TestAggregateTagSingleton;


class TestAggregate : public Aggregate
{
public:
    TestAggregate(uint32_t numVols)
    : Aggregate(numVols, 0)
    {

    }

    uint32_t GetNumTagBits()
    {
        return m_numTagBits;
    }

};

void TestAggregateTag::CheckTag(uint32_t aggregateSize, uint32_t index, uint32_t childTag)
{
    TestAggregate agg(aggregateSize);

    uint32_t tag = agg.GetTagFromChildIndexAndChildTag(index, childTag);

    EATESTAssert(tag != 0, "Tags should not be 0");
    
    EATESTAssert(agg.GetChildIndexFromTag(tag) == index, "Output index should be the same as input index");
    EATESTAssert(agg.GetChildTagFromTag(tag) == childTag, "Output child tag should be the same as input child tag");
}


void TestAggregateTag::CheckTag2(uint32_t aggregateSize1, uint32_t index1, 
                                 uint32_t aggregateSize2, uint32_t index2, 
                                 uint32_t aggregateSize3, uint32_t index3)
{
    TestAggregate agg1(aggregateSize1);
    TestAggregate agg2(aggregateSize2);
    TestAggregate agg3(aggregateSize3);

    uint32_t tag = 0;
    uint32_t numBitsUsed = 0;

    agg1.UpdateTagWithChildIndex(tag, numBitsUsed, index1);
    EATESTAssert(tag != 0, "Tags should not be 0");
    EATESTAssert(numBitsUsed != 0, "numBitsUsed should not be 0");
    agg2.UpdateTagWithChildIndex(tag, numBitsUsed, index2);
    EATESTAssert(tag != 0, "Tags should not be 0");
    EATESTAssert(numBitsUsed != 0, "numBitsUsed should not be 0");
    agg3.UpdateTagWithChildIndex(tag, numBitsUsed, index3);
    EATESTAssert(tag != 0, "Tags should not be 0");
    EATESTAssert(numBitsUsed != 0, "numBitsUsed should not be 0");

    uint32_t test = agg1.GetChildIndexFromTag(tag);
    EATESTAssert(test == index1, "Input index should be the same as the output index");
    tag = agg1.GetChildTagFromTag(tag);
    EATESTAssert(tag != 0, "Tags should not be 0");

    test = agg2.GetChildIndexFromTag(tag);
    EATESTAssert(test == index2, "Input index should be the same as the output index");
    tag = agg2.GetChildTagFromTag(tag);
    EATESTAssert(tag != 0, "Tags should not be 0");

    test = agg3.GetChildIndexFromTag(tag);
    EATESTAssert(test == index3, "Input index should be the same as the output index");
    tag = agg3.GetChildTagFromTag(tag);
    EATESTAssert(tag == 0, "Tag should be 0 at the end");
}


void TestAggregateTag::TagExtration()
{
    CheckTag(1, 0, 0);
    CheckTag(1, 0, 1);
    CheckTag(1, 0, 0x7FFFFFFF);

    CheckTag(2, 0, 0);
    CheckTag(2, 0, 1);
    CheckTag(2, 0, 0x3FFFFFFF);

    CheckTag(2, 1, 0);
    CheckTag(2, 1, 1);
    CheckTag(2, 1, 0x3FFFFFFF);

    CheckTag(3, 0, 0);
    CheckTag(3, 0, 1);
    CheckTag(3, 0, 0x3FFFFFFF);

    CheckTag(3, 1, 0);
    CheckTag(3, 1, 1);
    CheckTag(3, 1, 0x3FFFFFFF);

    CheckTag(3, 2, 0);
    CheckTag(3, 2, 1);
    CheckTag(3, 2, 0x3FFFFFFF);

    CheckTag(0xFF, 0, 0);
    CheckTag(0xFF, 0, 1);
    CheckTag(0xFF, 0, 0xFFFFFF);

    CheckTag(0xFF, 0xFE, 0);
    CheckTag(0xFF, 0xFE, 1);
    CheckTag(0xFF, 0xFE, 0xFFFFFF);

    CheckTag(0x0100, 0, 0);
    CheckTag(0x0100, 0, 1);
    CheckTag(0x0100, 0, 0x7FFFFF);

    CheckTag(0x0100, 0xFF, 0);
    CheckTag(0x0100, 0xFF, 1);
    CheckTag(0x0100, 0xFF, 0x7FFFFF);

    CheckTag(0xFFFF, 0, 0);
    CheckTag(0xFFFF, 0, 1);
    CheckTag(0xFFFF, 0, 0xFFFF);

    CheckTag(0xFFFF, 0xFFFE, 0);
    CheckTag(0xFFFF, 0xFFFE, 1);
    CheckTag(0xFFFF, 0xFFFE, 0xFFFF);

    CheckTag(0x010000, 0, 0);
    CheckTag(0x010000, 0, 1);
    CheckTag(0x010000, 0, 0x7FFF);

    CheckTag(0x010000, 0xFFFF, 0);
    CheckTag(0x010000, 0xFFFF, 1);
    CheckTag(0x010000, 0xFFFF, 0x7FFF);
}

void TestAggregateTag::TagBuilding()
{
    CheckTag2(1, 0, 1, 0, 1, 0);

    CheckTag2(2, 0, 1, 0, 1, 0);
    CheckTag2(1, 0, 2, 0, 1, 0);
    CheckTag2(1, 0, 1, 0, 2, 0);

    CheckTag2(2, 1, 1, 0, 1, 0);
    CheckTag2(1, 0, 2, 1, 1, 0);
    CheckTag2(1, 0, 1, 0, 2, 1);

    CheckTag2(2, 0, 2, 0, 1, 0);
    CheckTag2(1, 0, 2, 0, 2, 0);
    CheckTag2(2, 0, 1, 0, 2, 0);

    CheckTag2(2, 1, 2, 0, 1, 0);
    CheckTag2(1, 0, 2, 1, 2, 0);
    CheckTag2(2, 1, 1, 0, 2, 0);

    CheckTag2(2, 0, 2, 1, 1, 0);
    CheckTag2(1, 0, 2, 0, 2, 1);
    CheckTag2(2, 0, 1, 0, 2, 1);

    CheckTag2(2, 1, 2, 1, 1, 0);
    CheckTag2(1, 0, 2, 1, 2, 1);
    CheckTag2(2, 1, 1, 0, 2, 1);

    CheckTag2(3, 0, 1, 0, 1, 0);
    CheckTag2(1, 0, 3, 0, 1, 0);
    CheckTag2(1, 0, 1, 0, 3, 0);

    CheckTag2(3, 2, 1, 0, 1, 0);
    CheckTag2(1, 0, 3, 2, 1, 0);
    CheckTag2(1, 0, 1, 0, 3, 2);

    CheckTag2(3, 0, 2, 0, 1, 0);
    CheckTag2(3, 0, 2, 1, 1, 0);
    CheckTag2(3, 2, 2, 0, 1, 0);
    CheckTag2(3, 2, 2, 1, 1, 0);

    CheckTag2(1, 0, 2, 0, 3, 0);
    CheckTag2(1, 0, 2, 1, 3, 0);
    CheckTag2(1, 0, 2, 0, 3, 2);
    CheckTag2(1, 0, 2, 1, 3, 2);

    CheckTag2(7, 0, 8, 0, 16, 0);
    CheckTag2(7, 6, 8, 7, 16, 15);
    CheckTag2(16, 0, 8, 0, 7, 0);
    CheckTag2(16, 15, 8, 7, 7, 6);
}

void TestAggregateTag::NumTagBits()
{
    {
        TestAggregate agg(1);
        EATESTAssert(agg.GetNumTagBits() == 1, "Only need 1 bit for a 1 volume aggregate");
    }

    {
        TestAggregate agg(2);
        EATESTAssert(agg.GetNumTagBits() == 2, "Need 2 bits for a 2 volume aggregate");
    }

    {
        TestAggregate agg(3);
        EATESTAssert(agg.GetNumTagBits() == 2, "Need 2 bits for a 3 volume aggregate");
    }

    {
        TestAggregate agg(4);
        EATESTAssert(agg.GetNumTagBits() == 3, "Need 3 bits for a 4 volume aggregate");
    }

    {
        TestAggregate agg(7);
        EATESTAssert(agg.GetNumTagBits() == 3, "Need 3 bits for a 7 volume aggregate");
    }

    {
        TestAggregate agg(8);
        EATESTAssert(agg.GetNumTagBits() == 4, "Need 4 bits for a 8 volume aggregate");
    }

    {
        TestAggregate agg(15);
        EATESTAssert(agg.GetNumTagBits() == 4, "Need 4 bits for a 15 volume aggregate");
    }

    {
        TestAggregate agg(16);
        EATESTAssert(agg.GetNumTagBits() == 5, "Need 5 bits for a 16 volume aggregate");
    }

    {
        TestAggregate agg(31);
        EATESTAssert(agg.GetNumTagBits() == 5, "Need 5 bits for a 31 volume aggregate");
    }

    {
        TestAggregate agg(32);
        EATESTAssert(agg.GetNumTagBits() == 6, "Need 6 bits for a 32 volume aggregate");
    }
}
