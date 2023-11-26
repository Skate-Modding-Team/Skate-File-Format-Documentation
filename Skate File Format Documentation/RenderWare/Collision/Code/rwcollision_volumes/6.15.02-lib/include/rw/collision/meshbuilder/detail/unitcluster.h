// (c) Electronic Arts. All Rights Reserved.

#ifndef PUBLIC_RW_COLLISION_MESHBUILDER_DETAIL_UNITCLUSTER_H
#define PUBLIC_RW_COLLISION_MESHBUILDER_DETAIL_UNITCLUSTER_H


/*************************************************************************************************************

File: unitcluster.h

Purpose: UnitCluster struct.

A structure representing a UnitCluster, containing the list of clusters vertices,
a list of unitIds, a compressionMode indicator and a buffer for the final unit data.
*/


#include <rw/collision/common.h>

#if !defined EA_PLATFORM_PS3_SPU


#include <EASTL/sort.h>
#include <rw/collision/clusteredmeshcluster.h> // needed for rw::collision::ClusteredMeshCluster::Vertex32


namespace rw
{
namespace collision
{
namespace meshbuilder
{
namespace detail
{


struct UnitCluster
{
    /*
    This is used to store vertex ids in the cluster, and later to look them up efficiently.
    The key is the vertex id within the whole mesh, while the data (one byte) is the code within the cluster.
    The codes are assigned during finalize by sweeping the set.
    */
    typedef uint32_t VertexSet[ClusteredMeshCluster::MAX_VERTEX_COUNT];
    typedef uint32_t UnitID;

    struct VertexSetCompare
    {
        bool operator()(const uint32_t &left, const uint32_t &right) const
        {
            // sorts on left < right
            return (left < right);
        }
    };

    UnitCluster()
        : clusterID(0)
        , clusterOffset()
        , unitIDs(NULL)
        , numUnits(0)
        , numVertices(0)
        , compressionMode(rw::collision::ClusteredMeshCluster::VERTICES_UNCOMPRESSED)
    {
        clusterOffset.x = 0;
        clusterOffset.y = 0;
        clusterOffset.z = 0;
    }

    // Reset method used to set the cluster to an initial state
    void Reset(uint32_t id, uint32_t *IdList)
    {
        clusterID = id;

        clusterOffset.x = 0;
        clusterOffset.y = 0;
        clusterOffset.z = 0;

        numVertices = 0;

        numUnits = 0;
        unitIDs = IdList;
    }


    // Sorts and compresses the vertex set
    static void SortAndCompressVertexSet(VertexSet & vertexSet, uint32_t & vertexSetCount)
    {
        // Sort the vertex set
        VertexSetCompare compare;
        eastl::sort<uint32_t*, VertexSetCompare>(&vertexSet[0],
                                                 &vertexSet[vertexSetCount],
                                                 compare);

        // Compress the vertex set, removing duplicates
        uint32_t currentIndex = 0;
        uint32_t headIndex = currentIndex + 1;
        while (headIndex < vertexSetCount)
        {
            if (vertexSet[headIndex] == vertexSet[currentIndex])
            {
                ++headIndex;
            }
            else
            {
                vertexSet[++currentIndex] = vertexSet[headIndex++];
            }
        }

        // Set the new size of the compressed vertex set
        vertexSetCount = currentIndex + 1;
    }

    // Given a global vertex index, returns the cluster vertex index
    uint8_t GetVertexCode(const uint32_t vertexIndex) const
    {
        uint8_t start = 0;
        uint8_t end = static_cast<uint8_t>(numVertices - 1u);
        uint8_t ret = static_cast<uint8_t>((end - start) / 2u);

        // Binary search through the items in the vector
        while (start <= end)
        {
            if (vertexIndex == vertexIDs[ret])
            {
                return ret;
            }
            else if (vertexIndex > vertexIDs[ret])
            {
                start = static_cast<uint8_t>(ret + 1u);
            }
            else
            {
                end = static_cast<uint8_t>(ret - 1u);
            }

            ret = static_cast<uint8_t>(start + ((end - start) / 2u));
        }

        // This case should never occur, assert with a non const expression which will always fail
        EA_ASSERT_MSG(start > end, ("Vertex not found in cluster."));

        // return with FF 
        return 0xFF;
    }

    // UnitCluster ID
    uint32_t clusterID;

    // Used in 16-bit compression mode only
    rw::collision::ClusteredMeshCluster::Vertex32 clusterOffset; 

    // UnitId collection
    UnitID * unitIDs;
    // Count of 
    uint32_t numUnits;

    // Vertex collection
    VertexSet vertexIDs;
    // Count of entries in vertex set
    uint32_t numVertices;

    // Compression mode
    uint8_t compressionMode;

    // Used to ensure the struct is padded to with a 4 byte alignment
    uint8_t m_padding[3];
};


} // namespace detail
} // namespace meshbuilder
} // namespace collision
} // namespace rw


#endif // !defined EA_PLATFORM_PS3_SPU

#endif // #define PUBLIC_RW_COLLISION_MESHBUILDER_DETAIL_UNITCLUSTER_H
