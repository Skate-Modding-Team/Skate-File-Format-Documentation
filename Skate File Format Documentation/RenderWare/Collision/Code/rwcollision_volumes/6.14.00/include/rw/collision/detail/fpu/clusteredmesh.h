// (c) Electronic Arts. All Rights Reserved.
#ifndef PUBLIC_RW_COLLISION_DETAIL_FPU_CLUSTEREDMESH_H
#define PUBLIC_RW_COLLISION_DETAIL_FPU_CLUSTEREDMESH_H

/*************************************************************************************************************

 File: clusteredmesh.h

 Purpose:
*/

#include "rw/collision/common.h"

#if !defined(EA_PLATFORM_PS3_SPU)

#include "rw/collision/clusteredmesh.h"
#include "aabbox.h"
#include "procedural.h"
#include "kdtreewithsubtrees.h"
#include "kdtree.h"
#include "rw/collision/detail/fpu/clusteredmeshcluster.h"

namespace rw
{
namespace collision
{
namespace detail
{
namespace fpu
{
    // forward declare ClusteredMesh in the rw::collision::detail::fpu namespace so that we can
    // use it in the EA_SERIALIZATION_CLASS_VERSION macro
    class ClusteredMesh;
} // namespace fpu
} // namespace detail
} // namespace collision
} // namespace rw

// We need to specify the class serialization version prior to the class definition
// due to a problem with ps2 gcc.
EA_SERIALIZATION_CLASS_VERSION(rw::collision::detail::fpu::ClusteredMesh, 5)


namespace rw
{
namespace collision
{
namespace detail
{
namespace fpu
{



/** \brief This class mimics the layout of rw::collision::ClusterParams when built using fpu
 * rwmath.
 */
struct ClusterParams
{
    float   mVertexCompressionGranularity;
    uint16_t  mFlags;
    uint8_t   mGroupIdSize;
    uint8_t   mSurfaceIdSize;
};

class AllocationHelper
{
public:
    explicit
    AllocationHelper(EA::Physics::MemoryPtr res)
    {
        mem = reinterpret_cast<uintptr_t>(res.GetMemory());
    }

    template <typename T>
    void SubAlloc(T *&result, uint32_t size, uint32_t alignment)
    {
        mem = EA::Physics::SizeAlign<uintptr_t>(mem, alignment);
        result = reinterpret_cast<T*>(mem);
        mem += size;
    }

    void *SubAlloc(uint32_t size, uint32_t alignment = 1)
    {
        void *p;
        SubAlloc(p, size, alignment);
        return p;
    }

    uintptr_t mem;
};


/** \brief This class mimics the layout of rw::collision::ClusteredMesh when built using fpu
 * rwmath.
 *
 * This class can be used for creating memory imaged fpu versions of rw::collision::ClusteredMesh
 * which can be deserialized using the LLSerializable framework for loading on platforms
 * using fpu rwmath.
 *
 * As the serialization function matches that of rw::collision::ClusteredMesh it is possible to
 * convert between the two using the Serialization framework. As this class also implements the
 * ObjectDescriptor/EA::Physics::SizeAndAlignment framework so HLSerializable can also be used.
 *
 * Changes to data members in rw::collision::ClusteredMesh or its serialization function should be
 * mirrored in this class.
 */
class ClusteredMesh : public Procedural
{
public:

    template <class Archive>
    void Serialize(Archive &ar, uint32_t version);

    // these two functions are required for determining the cluster data size during serialization
    ClusteredMeshCluster& GetCluster(uint32_t index) const;
    uint32_t GetClusterSize(ClusteredMeshCluster& cluster) const;

    static EA::Physics::SizeAndAlignment
    GetResourceDescriptor(uint32_t maxClusters,
                          uint32_t clusterDataSize,
                          uint32_t numBranchNodes,
                          uint32_t maxUnits,
                          const AABBox &bbox,
                          float vertexCompressionGranularity = 0.01f,
                          uint32_t classSize = sizeof(ClusteredMesh),
                          RwpBool includeKDSubTrees = false);
    static ClusteredMesh*
    Initialize(const EA::Physics::MemoryPtr& resource,
               uint32_t maxClusters,
               uint32_t clusterDataSize,
               uint32_t numBranchNodes,
               uint32_t maxUnits,
               const AABBox &bbox,
               float vertexCompressionGranularity = 0.01f,
               uint32_t classSize = sizeof(ClusteredMesh),
               RwpBool includeKDSubTrees = false);

    struct ObjectDescriptor;
    const ObjectDescriptor GetObjectDescriptor();
    static EA::Physics::SizeAndAlignment GetResourceDescriptor(const ObjectDescriptor & objDesc);
    static ClusteredMesh* Initialize(const EA::Physics::MemoryPtr& resource, const ObjectDescriptor& objDesc);

    void Release();

    void UpdateNumTagBits();

    KDTreeWithSubTrees * mKDTree;
    uint32_t * mCluster;

    ClusterParams mClusterParams;
    uint32_t mNumClusters;
    uint32_t mMaxClusters;

    uint32_t mNumUnits;
    uint32_t mMaxUnits;

    uint32_t mSizeOfThis;

    uint16_t mDefaultGroupId;
    uint16_t mDefaultSurfaceId;
    uint8_t mDefaultEdgeAngle;

    uint32_t mNumClusterTagBits;
};



inline ClusteredMeshCluster&
ClusteredMesh::GetCluster(uint32_t index) const
{
    return *reinterpret_cast<ClusteredMeshCluster *>(reinterpret_cast<uintptr_t>(mCluster) + mCluster[index]);
}


inline uint32_t
ClusteredMesh::GetClusterSize(ClusteredMeshCluster& cluster) const
{
    uint32_t bytes = 16; // the cluster header is 16 bytes in size
    if (cluster.compressionMode == rw::collision::ClusteredMeshCluster::VERTICES_16BIT_COMPRESSED)
    {
        bytes += 3 * sizeof(uint32_t);
        bytes += sizeof(rw::collision::ClusteredMeshCluster::Vertex16) * cluster.vertexCount;
    }
    else if (cluster.compressionMode == rw::collision::ClusteredMeshCluster::VERTICES_32BIT_COMPRESSED)
    {
        bytes += sizeof(rw::collision::ClusteredMeshCluster::Vertex32) * cluster.vertexCount;
    }
    else
    {
        bytes += rwcCLUSTEREDMESHCLUSTER_VERTEXDATA_ALIGNMENT * cluster.vertexCount;
    }
    bytes = EA::Physics::SizeAlign<uint32_t>(bytes, rwcCLUSTEREDMESHCLUSTER_VERTEXDATA_ALIGNMENT);
    EA_ASSERT(static_cast<uint32_t>(cluster.normalStart * rwcCLUSTEREDMESHCLUSTER_VERTEXDATA_ALIGNMENT) == static_cast<uint32_t>(bytes - 16));
    bytes += rwcCLUSTEREDMESHCLUSTER_VERTEXDATA_ALIGNMENT * cluster.normalCount;
    EA_ASSERT(static_cast<uint32_t>(cluster.unitDataStart * rwcCLUSTEREDMESHCLUSTER_VERTEXDATA_ALIGNMENT) == static_cast<uint32_t>(bytes - 16));
    bytes += cluster.unitDataSize;
    return bytes;
}


inline EA::Physics::SizeAndAlignment
ClusteredMesh::GetResourceDescriptor(uint32_t maxClusters,
                                     uint32_t clusterDataSize,
                                     uint32_t numBranchNodes,
                                     uint32_t /*maxUnits*/,
                                     const AABBox & /*bbox*/,
                                     float /*vertexCompressionGranularity*/,
                                     uint32_t classSize,
                                     RwpBool includeKDSubTrees)
{
    uint32_t numSubTrees = includeKDSubTrees ? maxClusters : 0u;
    KDTreeWithSubTrees::ObjectDescriptor kdtreeParams(numBranchNodes, numSubTrees);
    EA::Physics::SizeAndAlignment kdtree = KDTreeWithSubTrees::GetResourceDescriptor(kdtreeParams);
    EA_ASSERT_MSG(kdtree.GetAlignment() <= rwcCLUSTEREDMESH_ALIGNMENT,
        ("KDtree alignment is not expected to be more strict that clustered mesh."));

    uint32_t size = EA::Physics::SizeAlign<uint32_t>(classSize, rwcCLUSTEREDMESH_ALIGNMENT);
    size += EA::Physics::SizeAlign<uint32_t>(kdtree.GetSize(), rwcCLUSTEREDMESH_ALIGNMENT);
    size += EA::Physics::SizeAlign<uint32_t>(maxClusters*sizeof(ClusteredMeshCluster*), rwcCLUSTEREDMESH_ALIGNMENT);
    size += clusterDataSize;

    // TODO : The following code suggests we may be counting the space required for KDSubTrees twice.
    //        This is not duplicated in the rwpmath version. It needs to be investigated.
    if (includeKDSubTrees)
    {
        size = EA::Physics::SizeAlign<uint32_t>(size, rwcCLUSTEREDMESH_ALIGNMENT);
        size += EA::Physics::SizeAlign<uint32_t>(maxClusters*sizeof(KDSubTree), rwcCLUSTEREDMESH_ALIGNMENT);
    }
    return EA::Physics::SizeAndAlignment(size, rwcCLUSTEREDMESH_ALIGNMENT);
}


inline ClusteredMesh*
ClusteredMesh::Initialize(const EA::Physics::MemoryPtr& resource,
                          uint32_t maxClusters,
                          uint32_t clusterDataSize,
                          uint32_t numBranchNodes,
                          uint32_t maxUnits,
                          const AABBox &bbox,
                          float vertexCompressionGranularity,
                          uint32_t classSize,
                          RwpBool includeKDSubTrees)
{
    AllocationHelper heap(resource);

    // allocate mesh
    ClusteredMesh *mesh = reinterpret_cast<ClusteredMesh *>(heap.SubAlloc(classSize));

    // allocate kdtree
    uint32_t numSubTrees = includeKDSubTrees ? maxClusters : 0u;
    KDTreeWithSubTrees::ObjectDescriptor kdtreeParams(numBranchNodes, numSubTrees);
    EA::Physics::SizeAndAlignment rd = KDTreeWithSubTrees::GetResourceDescriptor(kdtreeParams);
    mesh->mKDTree = KDTreeWithSubTrees::Initialize(EA::Physics::MemoryPtr(heap.SubAlloc(rd.GetSize(), rd.GetAlignment())), kdtreeParams);

    // round up the heap pointer to correct alignment
    heap.SubAlloc(0, rwcCLUSTEREDMESH_ALIGNMENT);

    // allocate cluster pointers
    heap.SubAlloc(mesh->mCluster, maxClusters*sizeof(uint32_t), rwcCLUSTEREDMESH_ALIGNMENT);

    // round up the heap pointer to correct alignment
    heap.SubAlloc(0, rwcCLUSTEREDMESH_ALIGNMENT);

    // reserve space for all clusters
    heap.SubAlloc(clusterDataSize, rwcCLUSTEREDMESH_ALIGNMENT);

    // set size of this for the GetSizeThis method
    mesh->mSizeOfThis = ClusteredMesh::GetResourceDescriptor(maxClusters, clusterDataSize, numBranchNodes,
                                                             maxUnits, bbox,
                                                             vertexCompressionGranularity,
                                                             classSize, includeKDSubTrees).GetSize();

    // assert that the memory allocated from the heap is not greater than the size allowed.
    EA_ASSERT(mesh->mSizeOfThis >= heap.mem - reinterpret_cast<uintptr_t>(mesh));

    return mesh;
}

}   // namespace fpu
}   // namespace detail
}   // namespace collision
}   // namespace rw

// We need to specify the class serialization version prior to the class definition
// due to a problem with ps2 gcc.
// Version 2 added m_includeKDSubTrees
EA_SERIALIZATION_CLASS_VERSION(rw::collision::detail::fpu::ClusteredMesh::ObjectDescriptor, 2)

namespace rw {
namespace collision {
namespace detail {
namespace fpu {

struct ClusteredMesh::ObjectDescriptor
{
    ObjectDescriptor(uint32_t maxClusters,
                     uint32_t clusterDataSize,
                     uint32_t numBranchNodes,
                     uint32_t maxUnits,
                     const AABBox& bbox,
                     RwpBool includeKDSubTrees = false)
    {
        m_maxClusters = maxClusters;
        m_clusterDataSize = clusterDataSize;
        m_numBranchNodes = numBranchNodes;
        m_maxUnits = maxUnits;
        m_bbox = bbox;
        m_includeKDSubTrees = includeKDSubTrees;
    }

    ObjectDescriptor()
    {
        m_maxClusters = 0;
        m_clusterDataSize = 0;
        m_numBranchNodes = 0;
        m_maxUnits = 0;
        m_bbox = AABBox();
        m_includeKDSubTrees = false;
    }

    template <class Archive>
        void Serialize(Archive &ar, uint32_t version)
    {
        ar & EA_SERIALIZATION_NAMED_VALUE(m_maxClusters);
        ar & EA_SERIALIZATION_NAMED_VALUE(m_clusterDataSize);
        ar & EA_SERIALIZATION_NAMED_VALUE(m_numBranchNodes);
        ar & EA_SERIALIZATION_NAMED_VALUE(m_maxUnits);
        ar & EA_SERIALIZATION_NAMED_VALUE(m_bbox);
        if (version > 1)
        {
            ar & EA_SERIALIZATION_NAMED_VALUE(m_includeKDSubTrees);
        }
        else
        {
            m_includeKDSubTrees = false;
        }
    }

    uint32_t m_maxClusters;
    uint32_t m_clusterDataSize;
    uint32_t m_numBranchNodes;
    uint32_t m_maxUnits;
    AABBox m_bbox;
    RwpBool m_includeKDSubTrees;
};


const inline ClusteredMesh::ObjectDescriptor
ClusteredMesh::GetObjectDescriptor()
{
    // Compute new total cluster data size for fpu mesh
    uint32_t i, clusterdatasize = 0;
    for (i = 0; i < mNumClusters; ++i)
    {
        ClusteredMeshCluster &cluster = GetCluster(i);
        clusterdatasize += GetClusterSize(cluster);
        clusterdatasize = EA::Physics::SizeAlign<uint32_t>(clusterdatasize, rwcCLUSTEREDMESHCLUSTER_ALIGNMENT);
    }

    return ObjectDescriptor(mMaxClusters, clusterdatasize,  mKDTree->m_numBranchNodes, mMaxUnits, m_AABB,
        (RwpBool) (mKDTree->GetNumKDSubTrees() > 0));
}


inline EA::Physics::SizeAndAlignment
ClusteredMesh::GetResourceDescriptor(const ClusteredMesh::ObjectDescriptor & objDesc)
{
    return GetResourceDescriptor(objDesc.m_maxClusters, 
                                 objDesc.m_clusterDataSize, objDesc.m_numBranchNodes,
                                 objDesc.m_maxUnits, objDesc.m_bbox,
                                 0.01f, sizeof(ClusteredMesh), objDesc.m_includeKDSubTrees);
}


inline ClusteredMesh*
ClusteredMesh::Initialize(const EA::Physics::MemoryPtr& resource, const ClusteredMesh::ObjectDescriptor& objDesc)
{
    return Initialize(resource, objDesc.m_maxClusters,
                      objDesc.m_clusterDataSize, objDesc.m_numBranchNodes,
                      objDesc.m_maxUnits, objDesc.m_bbox,
                      0.01f, sizeof(ClusteredMesh), objDesc.m_includeKDSubTrees);
}


inline void
ClusteredMesh::Release()
{
}


inline void
ClusteredMesh::UpdateNumTagBits()
{
    // Set the aggregate tag details
    mNumClusterTagBits = 1u + static_cast<uint32_t>(rw::math::fpu::Log(static_cast<float>(mNumClusters)) / rw::math::fpu::Log(2.0f));

    // Determine the maximum unit stream length
    uint32_t maxUnitStreamLength = 0u;
    for (uint32_t clusterIndex = 0 ; clusterIndex < mNumClusters ; ++clusterIndex)
    {
        ClusteredMeshCluster &cluster = GetCluster(clusterIndex);
        maxUnitStreamLength = (maxUnitStreamLength < cluster.unitDataSize ? cluster.unitDataSize : maxUnitStreamLength);
    }

    const uint32_t numUnitTagBits = 1u + static_cast<uint32_t>(rw::math::fpu::Log(static_cast<float>(maxUnitStreamLength)) / rw::math::fpu::Log(2.0f));

    // The complete number of tag bits is the sum of the cluster tag bits,
    // unit tag bits and one byte to indicate the unit triangle index
    m_numTagBits = mNumClusterTagBits + numUnitTagBits + 1;
}


template <class Archive>
void ClusteredMesh::Serialize(Archive &ar, uint32_t version)
{
    // Chain serialize down to base class. This is done via the '&' operator
    // rather than calling Serialize directly so that the version number of
    // the Procedural class is correct.
    ar & EA::Serialization::MakeNamedValue(*static_cast<Procedural*>(this), "Procedural");

    ar & EA_SERIALIZATION_NAMED_VALUE(mNumClusters);
    ar & EA_SERIALIZATION_NAMED_VALUE(mMaxClusters);
    ar & EA_SERIALIZATION_NAMED_VALUE(mNumUnits);
    ar & EA_SERIALIZATION_NAMED_VALUE(mMaxUnits);
    ar & EA_SERIALIZATION_NAMED_VALUE(mClusterParams.mVertexCompressionGranularity);
    if (version < 5)
    {
        ar & EA_SERIALIZATION_NAMED_VALUE(mSizeOfThis);
    }
    ar & EA_SERIALIZATION_NAMED_VALUE(mClusterParams.mFlags);
    ar & EA_SERIALIZATION_NAMED_VALUE(mDefaultGroupId);
    ar & EA_SERIALIZATION_NAMED_VALUE(mDefaultSurfaceId);
    ar & EA_SERIALIZATION_NAMED_VALUE(mDefaultEdgeAngle);
    ar & EA_SERIALIZATION_NAMED_VALUE(mClusterParams.mGroupIdSize);
    ar & EA_SERIALIZATION_NAMED_VALUE(mClusterParams.mSurfaceIdSize);

    ar.TrackInternalPointer(mKDTree);
    if (version > 2)
    {
        ar & EA_SERIALIZATION_NAMED_VALUE(*mKDTree);
        ar.TrackInternalPointer(mCluster);
        ar & EA_SERIALIZATION_NAMED_STATIC_ARRAY(mCluster, mNumClusters);
    }
    else
    {
        EA_ASSERT(ar.IsLoading());  // Should only try to load older versions
        if (ar.IsLoading())
        {
            // Read in KDTtreeWithSubTrees as a vanilla KDTree (this is OK since it effectively is since all
            // data is inherited from the common KDTreeBase class)...
            ar & EA::Serialization::MakeNamedValue(*reinterpret_cast<KDTree *>(mKDTree), "mKDTree");
            // TODO: The ensure remaining data is initialized
            mKDTree->SetKDSubTrees(0, 0);

            // Update so that mCluster[i] stores offset to cluster relative to mCluster, not this
            uint32_t newClusterOffset = (uint32_t) mCluster[0];
            ar.TrackInternalPointer(mCluster);
            ar & EA_SERIALIZATION_NAMED_STATIC_ARRAY(mCluster, mNumClusters);
            uint32_t deltaBytes = newClusterOffset - mCluster[0];
            for (uint32_t i = 0; i < mNumClusters; ++i)
            {
                mCluster[i] += deltaBytes;
            }
        }
    }

    uint32_t i;
    for (i = 0; i < mNumClusters; ++i)
    {
        ClusteredMeshCluster &cluster = GetCluster(i);
        ar & EA_SERIALIZATION_NAMED_VALUE(cluster.unitCount);
        ar & EA_SERIALIZATION_NAMED_VALUE(cluster.unitDataSize);
        ar & EA_SERIALIZATION_NAMED_VALUE(cluster.unitDataStart);
        ar & EA_SERIALIZATION_NAMED_VALUE(cluster.normalStart);
        ar & EA_SERIALIZATION_NAMED_VALUE(cluster.totalSize);
        ar & EA_SERIALIZATION_NAMED_VALUE(cluster.compressionMode);
        ar & EA_SERIALIZATION_NAMED_VALUE(cluster.vertexCount);
        ar & EA_SERIALIZATION_NAMED_VALUE(cluster.normalCount);

        if ( cluster.compressionMode == rw::collision::ClusteredMeshCluster::VERTICES_16BIT_COMPRESSED )
        {
            uint32_t* vertexArrayHeader = reinterpret_cast<uint32_t*>(cluster.vertexArray);
            ar & EA_SERIALIZATION_NAMED_STATIC_ARRAY(vertexArrayHeader, 3u);

            uint16_t* vertexArray = reinterpret_cast<uint16_t*>(cluster.vertexArray) + 3 * 2;
            ar & EA_SERIALIZATION_NAMED_STATIC_ARRAY(vertexArray, cluster.vertexCount * 3u);
        }
        else if ( cluster.compressionMode == rw::collision::ClusteredMeshCluster::VERTICES_32BIT_COMPRESSED )
        {
            int32_t* vertexArray = reinterpret_cast<int32_t*>(cluster.vertexArray);
            ar & EA_SERIALIZATION_NAMED_STATIC_ARRAY(vertexArray, cluster.vertexCount * 3u);
        }
        else
        {
            if (version == 1)
            {
                // Using fpu math it is not valid to serialize the normals with the vertices
                // as there will be padding between the two arrays.
                uint32_t size = uint32_t(cluster.vertexCount + cluster.normalCount);
                rw::math::fpu::Vector3* vertexAndNormalArray = cluster.vertexArray + size;
                ar & EA_SERIALIZATION_NAMED_STATIC_ARRAY(vertexAndNormalArray, size);
            }
            else
            {
                rw::math::fpu::Vector3* vertexArray = cluster.vertexArray;
                ar & EA_SERIALIZATION_NAMED_STATIC_ARRAY(vertexArray, cluster.vertexCount);
            }
        }

        if (version > 1)
        {
            // normalStart is a quad-word offset
            rw::math::fpu::Vector3* normalArray =
                reinterpret_cast<rw::math::fpu::Vector3*>(reinterpret_cast<uintptr_t>(cluster.vertexArray) + cluster.normalStart * 16);
            ar & EA_SERIALIZATION_NAMED_STATIC_ARRAY(normalArray, cluster.normalCount);
        }

        // unitDataStart is a quad-word offset
        uint8_t* unitData = reinterpret_cast<uint8_t*>(cluster.vertexArray) + cluster.unitDataStart * 16;
        ar & EA_SERIALIZATION_NAMED_STATIC_ARRAY(unitData, cluster.unitDataSize);
    }

    if (version > 3)
    {
        ar & EA_SERIALIZATION_NAMED_VALUE(mNumClusterTagBits);
    }
    else
    {
        EA_ASSERT(ar.IsLoading());  // Should only try to load older versions
        if (ar.IsLoading())
        {
            UpdateNumTagBits();
        }
    }

    if (ar.IsLoading())
    {
        // Initialize the mSizeOfThis member
        const ObjectDescriptor objectDescriptor(GetObjectDescriptor());
        const EA::Physics::SizeAndAlignment sizeAndAlignment(GetResourceDescriptor(objectDescriptor));
        mSizeOfThis = sizeAndAlignment.GetSize();
    }
}

} // namespace fpu
} // namespace detail
} // namespace collision
} // namespace rw

#endif // !defined(EA_PLATFORM_PS3_SPU)

#endif //PUBLIC_RW_COLLISION_DETAIL_FPU_CLUSTEREDMESH_H
