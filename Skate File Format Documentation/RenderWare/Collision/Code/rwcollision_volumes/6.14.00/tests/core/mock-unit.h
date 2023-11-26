// (c) Electronic Arts. All Rights Reserved.

/*************************************************************************************************************

File: mock-unit.h

Purpose: Mocked up ClusteredMesh Unit for testing with

*/

#ifndef MOCK_UNIT_H
#define MOCK_UNIT_H

#include "EABase/eabase.h"
#include "rw/collision/common.h"
#include "rw/collision/clusteredmeshcluster.h"

/// Mock Unit class for testing. Just records what is done to it.
class MockUnit
{
public: // Expected Unit API

    MockUnit(const rw::collision::ClusteredMeshCluster & cluster,
        const rw::collision::ClusterParams & clusterParams,
        uint32_t offset = 0u,
        uint32_t size = DEFAULT_SIZE)
        : mCluster(&cluster)
        , mClusterParams(&clusterParams)
        , mOffset(offset)
        , mData(NULL)
        , mValid(true)
        , mSize(size)
    {
        Reset(offset);
    }

    void Advance()
    {
        mData += mSize;
    }

    void Reset(uint32_t offset)
    {
        mOffset = offset;
        mData = mCluster->UnitData() + offset;
    }

    uint32_t GetSize() const
    {
        return mSize;
    }

    bool IsValid() const
    {
        return mValid;
    }

    const rw::collision::ClusteredMeshCluster & GetCluster() const
    {
        return *mCluster;
    }

public: // Control over behavior for unit testing

    void SetValid(bool valid) const
    {
        mValid = valid;
    }
    void SetSize(uint32_t size) const
    {
        mSize = size;
    }

public: // Data

    static const uint32_t DEFAULT_SIZE = 4;
    const rw::collision::ClusteredMeshCluster * mCluster;
    const rw::collision::ClusterParams * mClusterParams;
    uint32_t mOffset;
    const uint8_t * mData;
    mutable bool mValid;
    mutable uint32_t mSize;
};

/// Add additional API required for ClusteredMeshTriangleIterator.
/// Use derived class to ensure ClusteredMeshUnitIterator doesn't use this API.
class MockUnit2 : public MockUnit
{
public: // Expected Unit API for Triangle iteration

    MockUnit2(const rw::collision::ClusteredMeshCluster & cluster,
        const rw::collision::ClusterParams & clusterParams,
        uint32_t offset = 0u,
        uint32_t size = MockUnit::DEFAULT_SIZE) : MockUnit(cluster, clusterParams, offset, size)
    {
        mVertexBase = 0;
        mNumTriangles = 1;
        mID = 0x12345678u;
    }

    uint32_t GetTriCount() const
    {
        return mNumTriangles;
    }

    void GetTriVertices(rwpmath::Vector3 & v0, rwpmath::Vector3 & v1, rwpmath::Vector3 & v2, uint32_t tri = 0) const
    {
        // mock up to return vertex base + 2*i from cluster.
        if (tri == 0)
        {
            v0 = mCluster->GetVertex((uint8_t) (mVertexBase + 0), mClusterParams->mVertexCompressionGranularity);
            v1 = mCluster->GetVertex((uint8_t) (mVertexBase + 1), mClusterParams->mVertexCompressionGranularity);
            v2 = mCluster->GetVertex((uint8_t) (mVertexBase + 2), mClusterParams->mVertexCompressionGranularity);
        }
        else if (tri == 1)
        {
            v0 = mCluster->GetVertex((uint8_t) (mVertexBase + 3), mClusterParams->mVertexCompressionGranularity);
            v1 = mCluster->GetVertex((uint8_t) (mVertexBase + 2), mClusterParams->mVertexCompressionGranularity);
            v2 = mCluster->GetVertex((uint8_t) (mVertexBase + 1), mClusterParams->mVertexCompressionGranularity);
        }
        else
        {
            v0 = v1 = v2 = rwpmath::GetVector3_Zero();
        }
    }

    void GetTriVertexIndices(uint8_t &v0, uint8_t &v1, uint8_t &v2, uint32_t tri = 0) const
    {
        if (tri == 0)
        {
            v0 = mData[1];
            v1 = mData[2];
            v2 = mData[3];
        }
        else
        {
            v0 = mData[4];
            v1 = mData[3];
            v2 = mData[2];
        }
    }

    uint8_t GetEdgeData(uint8_t i) const
    {
        return (uint8_t) (255u - i - mVertexBase);
    }

    uint32_t GetEdgeCosinesAndFlags(rwpmath::Vector3 & edgeCosines, uint32_t tri = 0) const
    {
        edgeCosines = rwpmath::Vector3(mVertexBase + tri - 2.0f, mVertexBase + tri - 4.0f, mVertexBase + tri - 6.0f);
        return FLAGS + tri;
    }

    uint32_t GetID() const
    {
        return mID;
    }

    uint32_t GetGroupID() const
    {
        return mID & 0x0000FFFF;
    }

    uint32_t GetSurfaceID() const
    {
        return mID >> 16;
    }

    uint32_t GetOffset() const
    {
        uint32_t offset = static_cast<uint32_t>(mData - mCluster->UnitData());
        return offset;
    }

public: // Control over behavior for unit testing

    void SetTriCount(uint32_t numTriangles) const
    {
        mNumTriangles = numTriangles;
    }
    void SetVertexBase(uint8_t base) const
    {
        mVertexBase = base;
    }
    void SetID(uint32_t id) const
    {
        mID = id;
    }

public: // Data

    static const uint32_t FLAGS = 0xabcd1234u;

    mutable uint8_t mVertexBase;
    mutable uint32_t mNumTriangles;
    mutable uint32_t mID;

};

/// Variant on MockUnit that starts with a quad
class MockUnitQuad : public MockUnit2
{
public:

    MockUnitQuad(const rw::collision::ClusteredMeshCluster & cluster,
        const rw::collision::ClusterParams & clusterParams,
        uint32_t offset = 0u) : MockUnit2(cluster, clusterParams, offset, DEFAULT_SIZE)
    {
        mNumTriangles = 2;
    }

    static const uint32_t DEFAULT_SIZE = 5;
};

#endif // !MOCK_UNIT_H
