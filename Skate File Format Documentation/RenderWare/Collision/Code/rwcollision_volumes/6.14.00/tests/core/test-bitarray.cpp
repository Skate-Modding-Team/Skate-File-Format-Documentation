// (c) Electronic Arts. All Rights Reserved.

#include <unit/unit.h>
#include <EABase/eabase.h>
#include <rw/collision/bitarray.h>
#include <benchmarkenvironment/timer.h>
#include <benchmarkenvironment/sample.h>

#include "testsuitebase.h" // For TestSuiteBase

#include "random.hpp"

class TestBitArray : public rw::collision::tests::TestSuiteBase
{
public:
    virtual void Initialize()
    {
        SuiteName("TestBitArray");

        EATEST_REGISTER( "TestFindFirstZero",  "Test that bit array can find the first zero.",  TestBitArray, TestFindFirstZero );
        EATEST_REGISTER( "TestFindFirstOne",  "Test that bit array can find the first one.",  TestBitArray, TestFindFirstOne );
        EATEST_REGISTER( "TestIterateZeros",  "Test that bit array can find all the zeros.",  TestBitArray, TestIterateZeros );
        EATEST_REGISTER( "TestIterateOnes",  "Test that bit array can find all the ones.",  TestBitArray, TestIterateOnes );
        EATEST_REGISTER( "Benchmark",  "Benchmark the bit array iterators",  TestBitArray, Benchmark );
    }

private:
    void TestFindFirstZero();
    void TestFindFirstOne();

    void TestIterateZeros();
    void TestIterateOnes();

    void Benchmark();
    
    uint32_t ComputeBitArraySize(uint32_t numBits);
    void RandomizeBoolArray(bool* array, uint32_t numItems);
    void CopyToBitArray(bool* referenceArray, uint32_t numItems, rw::collision::BitArray & bitArray);

    class IteratorTester
    {
    public:
        IteratorTester(bool * seen, uint32_t maxEntries) : m_seen(seen){
            for(uint32_t i=0; i<maxEntries; i++)
            {
                m_seen[i] = false;
            }
        };

        void Process(uint32_t index)
        {
            m_seen[index]=true;
        };
    private:
        bool * m_seen;
    };

    class NullHandler
    {
    public:
        NullHandler(){};

        void Process(uint32_t /*index*/)
        {
        };
    };
} TestBitArraySingleton;

#define BITARRAY_ROUNDUP(N, R) (((N) + ((R) - 1)) & ~((R) - 1))
#define GET_SIZE_OF_BIT_ARRAY(numBits) (sizeof(rw::collision::BitArray::WordType) * (BITARRAY_ROUNDUP(numBits, 32) >> 5))

void 
TestBitArray::RandomizeBoolArray(bool* array, uint32_t numItems)
{
    for(uint32_t i=0; i<numItems; i++)
    {
        array[i] = Random01() > 0.5f;
    }
}

void 
TestBitArray::CopyToBitArray(bool* referenceArray, uint32_t numItems, rw::collision::BitArray & bitArray)
{
    for(uint32_t i=0; i<numItems; i++)
    {
        if(referenceArray[i])
        {
            bitArray.Set(i);
        }
        else
        {
            bitArray.Unset(i);
        }
    }
}

void 
TestBitArray::TestFindFirstZero()
{
    static const uint32_t MAX_BITS = 1024;
    static const uint32_t MAX_DATA = GET_SIZE_OF_BIT_ARRAY(MAX_BITS);
    char data[MAX_DATA];
    rw::collision::BitArray::WordType * arrayData = reinterpret_cast<rw::collision::BitArray::WordType *>(data);

    for(uint32_t sizeInBits=1; sizeInBits<=MAX_BITS; sizeInBits++)
    {
        rw::collision::BitArray testArray;
        testArray.Initialize(arrayData, sizeInBits);
        testArray.SetAll();

        for(uint32_t i=0; i<sizeInBits; i++)
        {
            testArray.Unset(i);
            uint32_t firstZeroIndex=0;
            EATESTAssert(testArray.FirstZero(firstZeroIndex), "Failed to find a zero");
            EATESTAssert(firstZeroIndex == i, "Failed to find correct index");
            testArray.Set(i);
        }
    }
}

void 
TestBitArray::TestFindFirstOne()
{
    static const uint32_t MAX_BITS = 1024;
    static const uint32_t MAX_DATA = GET_SIZE_OF_BIT_ARRAY(MAX_BITS);
    char data[MAX_DATA];
    rw::collision::BitArray::WordType * arrayData = reinterpret_cast<rw::collision::BitArray::WordType *>(data);

    for(uint32_t sizeInBits=1; sizeInBits<=MAX_BITS; sizeInBits++)
    {
        rw::collision::BitArray testArray;
        testArray.Initialize(arrayData, sizeInBits);
        testArray.UnsetAll();

        for(uint32_t i=0; i<sizeInBits; i++)
        {
            testArray.Set(i);
            uint32_t firstOneIndex=0;
            EATESTAssert(testArray.FirstOne(firstOneIndex), "Failed to find a zero");
            EATESTAssert(firstOneIndex == i, "Failed to find correct index");
            testArray.Unset(i);
        }
    }
}

void 
TestBitArray::TestIterateZeros()
{
    static const uint32_t MAX_BITS = 1024;
    static const uint32_t MAX_DATA = GET_SIZE_OF_BIT_ARRAY(MAX_BITS);
    char data[MAX_DATA];
    bool referenceData[MAX_BITS];
    bool resultData[MAX_BITS];
    rw::collision::BitArray::WordType * arrayData = reinterpret_cast<rw::collision::BitArray::WordType *>(data);

    for(uint32_t sizeInBits=1; sizeInBits<=MAX_BITS; sizeInBits++)
    {
        rw::collision::BitArray testArray;
        testArray.Initialize(arrayData, sizeInBits);
        RandomizeBoolArray(referenceData, sizeInBits);
        CopyToBitArray(referenceData, sizeInBits, testArray);

        IteratorTester tester(resultData, sizeInBits);
        testArray.IterateZeros(tester);

        for(uint32_t i=0; i<sizeInBits; i++)
        {
            EATESTAssert(referenceData[i] == !resultData[i], "Result mismatch when finding zeros");
        }

    }
}

void 
TestBitArray::TestIterateOnes()
{
    static const uint32_t MAX_BITS = 1024;
    static const uint32_t MAX_DATA = GET_SIZE_OF_BIT_ARRAY(MAX_BITS);
    char data[MAX_DATA];
    bool referenceData[MAX_BITS];
    bool resultData[MAX_BITS];
    rw::collision::BitArray::WordType * arrayData = reinterpret_cast<rw::collision::BitArray::WordType *>(data);

    for(uint32_t sizeInBits=1; sizeInBits<=MAX_BITS; sizeInBits++)
    {
        rw::collision::BitArray testArray;
        testArray.Initialize(arrayData, sizeInBits);
        RandomizeBoolArray(referenceData, sizeInBits);
        CopyToBitArray(referenceData, sizeInBits, testArray);

        IteratorTester tester(resultData, sizeInBits);
        testArray.IterateOnes(tester);

        for(uint32_t i=0; i<sizeInBits; i++)
        {
            EATESTAssert(referenceData[i] == resultData[i], "Result mismatch when finding zeros");
        }

    }
}

void 
TestBitArray::Benchmark()
{
    rw::math::SeedRandom(0x123456);    
    
    static const uint32_t NUM_SAMPLES = 100;
    static const uint32_t MAX_BITS = 1000;
    static const uint32_t MAX_DATA = GET_SIZE_OF_BIT_ARRAY(MAX_BITS);
    char data[MAX_DATA];
    bool referenceData[MAX_BITS];
    bool resultData[MAX_BITS];
    rw::collision::BitArray::WordType * arrayData = reinterpret_cast<rw::collision::BitArray::WordType *>(data);

    rw::collision::BitArray testArray;
    testArray.Initialize(arrayData, MAX_BITS);
    RandomizeBoolArray(referenceData, MAX_BITS);
    CopyToBitArray(referenceData, MAX_BITS, testArray);

    {
        benchmarkenvironment::Timer queryTimer;
        benchmarkenvironment::NoiselessSample sample;
        for(uint32_t i=0; i<NUM_SAMPLES; i++)
        {
            queryTimer.Start();
            IteratorTester tester(resultData, MAX_BITS);
            testArray.IterateOnes(tester);
            queryTimer.Stop();
            sample.AddElement(queryTimer.AsSeconds());
        }

        EATESTSendBenchmark("BitArray::IterateOnes", sample.GetMean(), sample.GetMin(), sample.GetMax());
    }

    for(uint32_t i=0; i<MAX_BITS; i++)
    {
        if(testArray[i])
        {
            testArray.Unset(i);
        }
        else
        {
            testArray.Unset(i);
        }
    }

    {
        benchmarkenvironment::Timer queryTimer;
        benchmarkenvironment::NoiselessSample sample;
        for(uint32_t i=0; i<NUM_SAMPLES; i++)
        {
            queryTimer.Start();
            IteratorTester tester(resultData, MAX_BITS);
            testArray.IterateZeros(tester);
            queryTimer.Stop();
            sample.AddElement(queryTimer.AsSeconds());
        }

        EATESTSendBenchmark("BitArray::IterateZeros", sample.GetMean(), sample.GetMin(), sample.GetMax());
    }
    
}
