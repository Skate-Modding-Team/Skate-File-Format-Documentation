// (c) Electronic Arts. All Rights Reserved.
#ifndef PUBLIC_RW_COLLISION_CLUSTEREDMESH_BASE_H
#define PUBLIC_RW_COLLISION_CLUSTEREDMESH_BASE_H

/*************************************************************************************************************

 File: rwcclusteredmesh.hpp

 Purpose: Compressed aggregate of triangles and quads with KDTree spatial map.
 */


#include "rw/collision/common.h"
#include "rw/collision/aabbox.h"
#include "rw/collision/procedural.h"
#include "rw/collision/volume.h"
#include "rw/collision/clusteredmeshcluster.h"
#include "rw/collision/clusteredmeshunit.h"
#include "rw/collision/kdtree.h"
#include "rw/collision/kdtreewithsubtrees.h"

// Alignment must be 16 to support loading legacy data
#define rwcCLUSTEREDMESH_ALIGNMENT 16

namespace rw
{
namespace collision
{
    // Forward declare ClusteredMesh in the rw::collision namespace so
    // that we can use it in the EA_SERIALIZATION_CLASS_* macros
    class ClusteredMesh;

    // Forward declare TriangleVolume in the rw::collision namespace so
    // that it can be used in method arguments
    class TriangleVolume;

} // namespace collision
} // namespace rw

// We need to specify the class serialization version prior to the class definition due to a problem with ps2 gcc.
// This version MUST be updated if the Serialize function is modified.
// Version 2 fixed arithmetic for vertex array address.
// Version 3 changed mKDTree to be a KDTreeWithSubTrees pointer and the cluster offsets to be 
// relative to mCluster array rather than the ClusteredMesh.
EA_SERIALIZATION_CLASS_VERSION(rw::collision::ClusteredMesh, 5)
// These macro provide the type name used in text-based archives' serialization.
EA_SERIALIZATION_CLASS_NAME(rw::collision::ClusteredMesh, "rw::collision::ClusteredMesh")

namespace rw
{
namespace collision
{

/**
\brief
The ClusteredMesh is a procedural aggregate consisting of a compressed vertex and shape data indexed
by a  rw::collision::KDTree.   The triangle and quad volume types are stored in a clustered mesh.

This provides an efficient data structure for a collision mesh. It can be created in a conditioning pipeline
using  rw::collision::conditioning::NodeClusteredMeshCreate.

\importlib rwccore
*/
class ClusteredMesh : public Procedural
{
private:
    /**
    \internal
    Constructor is private and NOT implemented.  You must use Initialize instead.
    */
    ClusteredMesh();

public:

    // *****************************************************************************************************
    //  Lifecycle API

    static EA::Physics::SizeAndAlignment
    GetResourceDescriptor(uint32_t maxClusters,
                          uint32_t clusterDataSize,
                          uint32_t numBranchNodes,
                          uint32_t maxUnits,
                          const rw::collision::AABBox &bbox,
                          float vertexCompressionGranularity = 0.01f,
                          uint32_t classSize = sizeof(ClusteredMesh),
                          RwpBool includeKDSubTrees = FALSE);

    static ClusteredMesh *
    Initialize(const EA::Physics::MemoryPtr & memoryResource,
               uint32_t maxClusters,
               uint32_t clusterDataSize,
               uint32_t numBranchNodes,
               uint32_t maxUnits,
               const rw::collision::AABBox &bbox,
               float vertexCompressionGranularity = 0.01f,
               uint32_t classSize = sizeof(ClusteredMesh),
               RwpBool includeKDSubTrees = FALSE);

    void
    Release();

    // *****************************************************************************************************
    //  Population API

    ClusteredMeshCluster *
    AllocateNextCluster(uint32_t sizeInBytes, uint32_t numUnits);

    ClusteredMeshCluster *
    AllocateNextCluster(const ClusterConstructionParameters & parameters);

    // *****************************************************************************************************
    //  Get/Set properties

    KDTree *
    GetKDTree() const;

    KDTreeBase *
    GetKDTreeBase() const;

    const KDSubTree *
    GetClusterKDTree(uint32_t clusterIndex) const;

    uint32_t
    GetNumCluster() const;

    uint32_t
    GetMaxCluster() const;

    uint16_t
    GetFlags() const { return mClusterParams.mFlags; }

    /// \brief Return pointer to table of cluster offsets for calculating cluster addresses without accessing 
    /// the whole mesh (for example, from SPU).
    /// Use GetClusterFromClusterTable() to convert from these offsets to the address of the cluster.
    /// Use GetCluster() in preference to these methods if you have access to the ClusteredMesh.
    uint32_t*
    GetClusterTableAddress() const { return mCluster; }

    static uintptr_t GetClusterFromClusterTable(uintptr_t mesh, const uint32_t * clusterTable, uint32_t clusterIndex);

    ClusteredMeshCluster &
    GetCluster(uint32_t index) const;

    void
    GetClusterIndexAndUnitFromNode(uint32_t node, uint32_t & index, uint32_t & unit);

    uint32_t
    GetNumUnits() const;

    uint32_t
    GetNumUnitInCluster(uint32_t clusterId) const;

    uint32_t
    GetMaxUnits() const;

    uint32_t
    GetUnitVolumes(uint32_t index, uint32_t offset, Volume *triList, uint32_t &triCount) const;

    uint32_t
    UnitGetOverlappingGPInstances(uint32_t index, uint32_t offset, const AABBox &bbox, const rwpmath::Matrix44Affine *transform,
                                    GPTriangle *instances, uint32_t &numPrimitivesInUnit) const;

    uint16_t
    GetDefaultGroupId() const;

    void
    SetDefaultGroupId(uint16_t defaultGroupId);

    uint16_t
    GetDefaultSurfaceId() const;

    void
    SetDefaultSurfaceId(uint16_t defaultSurfaceId);

    uint8_t
    GetDefaultEdgeAngle() const;

    void
    SetDefaultEdgeAngle(uint8_t defaultEdgeAngle);

    uint8_t
    GetGroupIdSize() const;

    void
    SetGroupIdSize(uint8_t GroupId);

    uint8_t
    GetSurfaceIdSize() const;

    void
    SetSurfaceIdSize(uint8_t SurfaceId);

    RwpBool
    IsOneSided() const;

    void
    SetOneSided(RwpBool onesided);

    const float &
    GetVertexCompressionGranularity() const;

    ClusterParams
    GetClusterParams() const;

    uint32_t
    GetUnitSize(uint32_t index, uint32_t offset) const;

    uint32_t
    NumVolumesInUnit(uint32_t index, uint32_t offset) const;

    uint32_t
    GetUnitType(uint32_t index, uint32_t offset) const;

    RwpBool
    IsValid() const;

    void SetClusterKDTrees(KDSubTree * trees);

    void CreateClusterKDTrees(const EA::Physics::MemoryPtr &workspaceRes);

    void
    GetVolumeFromChildIndex(rw::collision::TriangleVolume & volume, uint32_t childIndex) const;

    uint32_t
    GetClusterIndexFromChildIndex(uint32_t childIndex) const;

    uint32_t
    GetUnitOffsetFromChildIndex(uint32_t childIndex) const;

    uint32_t
    GetTriangleIndexWithinUnitFromChildIndex(uint32_t childIndex) const;

    // *****************************************************************************************************
    // Virtual functions required by the Aggregate interface

    uint32_t
    GetSizeThis();

    void
    UpdateThis();

    RwpBool
    LineIntersectionQueryThis(VolumeLineQuery *lineQuery,
                              const rwpmath::Matrix44Affine *tm);

    RwpBool
    BBoxOverlapQueryThis(VolumeBBoxQuery *bboxQuery,
                         const rwpmath::Matrix44Affine *tm);

    uint32_t
    GetUnitVolume(uint32_t index, uint32_t offset, uint32_t subindex, Volume *vol) const;

    struct ObjectDescriptor;

    static ClusteredMesh * Initialize(const EA::Physics::MemoryPtr& memoryResource, const ObjectDescriptor & objDesc);
    static EA::Physics::SizeAndAlignment GetResourceDescriptor(const ObjectDescriptor & objDesc);

    // Return the information needed to allocate this object when deserializing
    const ObjectDescriptor GetObjectDescriptor() const;

    uint32_t GetClusterSize(const ClusteredMeshCluster& cluster) const
    {
        uint32_t bytes = 16; // the cluster header is 16 bytes in size
        if (cluster.compressionMode == rw::collision::ClusteredMeshCluster::VERTICES_16BIT_COMPRESSED)
        {
            bytes += 3 * sizeof(int32_t);
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

    template <class Archive>
    void Serialize(Archive &ar, uint32_t version);

private:

    // *****************************************************************************************************
    //  Mesh private methods (inline in the cpp)

    inline rwpmath::Vector3
    GetVertex(uint32_t index, uint8_t vertid) const;

    inline AABBox
    GetUnitBBox(uint32_t index, uint32_t offset) const;

    inline uint32_t
    GetChildIndex(
        const uint32_t unitOffset,
        const uint32_t unitTriangleIndex,
        const uint32_t clusterIndex) const;

    inline uint32_t
    GetNumClusterTagBits() const;

    inline uint32_t
    GetNumUnitTagBits() const;

    void
    UpdateNumTagBits();

protected:

    // *****************************************************************************************************
    //  Mesh private data

    KDTreeWithSubTrees * mKDTree;         ///<A pointer to the KDTree that indexes the cluster
    uint32_t * mCluster;                  ///<A pointer to the first cluster (which follows this class)

    ClusterParams mClusterParams;         ///<Some mesh-wide parameters
    uint32_t mNumClusters;                ///<The total number of clusters in the mesh
    uint32_t mMaxClusters;                ///<The maximum allowable number of clusters in the mesh

    uint32_t mNumUnits;                   ///<The total number of units in the mesh
    uint32_t mMaxUnits;                   ///<The maximum allowable number of units in the mesh

    uint32_t mSizeOfThis;                 ///<The total size of the clustered mesh, including vertices, normals, units, etc

    uint16_t mDefaultGroupId;             ///<The default value for the user-defined group ID
    uint16_t mDefaultSurfaceId;           ///<The default value for the user-defined surface ID
    uint8_t mDefaultEdgeAngle;            ///<The default value for the edge angle

    uint32_t mNumClusterTagBits;          ///<The number of bits required to store the cluster tags

    static VTable sm_vTable;

    /* inherited from Aggregate

    AABBox        m_AABB;
    RWPTR(VTable) m_vTable;
    uint32_t      m_numTagBits;
    uint32_t      m_numVolumes;
    */
private:

};



// ***********************************************************************************************************
// Inline Methods

/**
\brief Get pointer to the spatial map that is used for this ClusteredMesh.
\deprecated This is deprecated functionality - use GetKDTreeBase() instead

Each unit is inserted in the spatial map and the index is the cluster index (16 bit) and the unit
index (16 bit).  Unless the FLAG_20BITCLUSTERINDEX flag is set, in which case the split is 20/12.
\return The KDTree used by the ClusteredMesh.
*/
inline KDTree *
ClusteredMesh::GetKDTree() const
{
    rwcDEPRECATED("ClusteredMesh::GetKDTree() is deprecated. ClusteredMesh::GetKDTreeBase() should be used instead.");
    // This cast is valid, since KDTree has the same data layout as KDTreeBase and mKDTree points
    // to a KDTreeWithSubTrees which is a KDTreeBase.
    // Ideally, we'd return a KDTreeBase from this function, but that will break customer code.
    return static_cast<KDTree *>(static_cast<KDTreeBase *>(mKDTree));
}

/**
\brief Get pointer to the spatial map that is used for this ClusteredMesh.

Each unit is inserted in the spatial map and the index is the cluster index (16 bit) and the unit
index (16 bit).  Unless the FLAG_20BITCLUSTERINDEX flag is set, in which case the split is 20/12.
\return The KDTree used by the ClusteredMesh.
*/
inline KDTreeBase *
ClusteredMesh::GetKDTreeBase() const
{
    return mKDTree;
}

/**
\brief Gets the number of cluster currently in the mesh
\return the numCluster
*/
inline uint32_t
ClusteredMesh::GetNumCluster() const
{
    return mNumClusters;
}


/**
\brief Gets the maximum number of cluster this mesh can have.
\return the maxCluster
*/
inline uint32_t
ClusteredMesh::GetMaxCluster() const
{
    return mMaxClusters;
}

/**
\brief Get address of ClusteredMeshCluster from ClusterTable.
This does not access the mesh, just the clusterTable.
\param clusterTableAddress Host address of table, as returned from mesh->GetClusterTableAddress().
\param clusterTable Local copy of table of offsets returned from mesh->GetClusterTableAddress().
\param clusterIndex Index of cluster from mesh/table to return address of.
\return Host address of ClusteredMeshCluster (within the mesh).
*/
inline uintptr_t
ClusteredMesh::GetClusterFromClusterTable(uintptr_t clusterTableAddress, 
                                          const uint32_t * clusterTable, 
                                          uint32_t clusterIndex)
{
    // Cluster data stored at given offset relative to mCluster array.
    // Changed from being relative to "this" in version 3.
    return clusterTableAddress + clusterTable[clusterIndex];
}

/**
\brief Get reference to a cluster.
\param clusterIndex cluster index
\return reference to a cluster.
*/
inline ClusteredMeshCluster &
ClusteredMesh::GetCluster(uint32_t clusterIndex) const
{
    EA_ASSERT(clusterIndex < mNumClusters);
    // Cluster data stored at given offset relative to mCluster array.
    // Changed from being relative to "this" in version 3.
    char *baseAddress = reinterpret_cast<char *>(mCluster);
    return *reinterpret_cast<ClusteredMeshCluster *>(EA::Physics::MemAddOffset(baseAddress, mCluster[clusterIndex]));
}

/**
\brief Decodes the index and unit from the node returned from a kd tree
\param node The id to decode
\param index  The index of the cluster
\param unit   The unit within the cluster
*/
inline void
ClusteredMesh::GetClusterIndexAndUnitFromNode(uint32_t node, uint32_t & index, uint32_t & unit)
{
    uint32_t shift = 16u + (mClusterParams.mFlags & CMFLAG_20BITCLUSTERINDEX);
    uint32_t mask = ((1u << shift) - 1);

    uint32_t clusterIndex = node;
    unit = node & mask;
    index = clusterIndex >> shift;
}


/**
\brief Gets the total number of units that are in the clustered mesh.
\return the numUnits
*/
inline uint32_t
ClusteredMesh::GetNumUnits() const
{
    return mNumUnits;
}


/**
\brief Gets the maximum number of units that this clustered map can hold.

The max units is constrained vaguely by the number of clusters, and more precisely by the KDTree.
\return the maxUnits
*/
inline uint32_t
ClusteredMesh::GetMaxUnits() const
{
    return mMaxUnits;
}


/**
\brief Gets the default group Id.
\return  the default group Id.
*/
inline uint16_t
ClusteredMesh::GetDefaultGroupId() const
{
    return mDefaultGroupId;
}


/**
\brief Sets the default group id.
\param defaultGroupId the new default groupId
*/
inline void
ClusteredMesh::SetDefaultGroupId(uint16_t defaultGroupId)
{
    mDefaultGroupId = defaultGroupId;
}


/**
\brief Gets the default SurfaceId.
\return the default SurfaceId.
*/
inline uint16_t
ClusteredMesh::GetDefaultSurfaceId() const
{
    return mDefaultSurfaceId;
}


/**
\brief Sets the default SurfaceId.
\param defaultSurfaceId the new default SurfaceId.
*/
inline void
ClusteredMesh::SetDefaultSurfaceId(uint16_t defaultSurfaceId)
{
    mDefaultSurfaceId = defaultSurfaceId;
}


/**
\brief Gets the default EdgeAngle.
\return defaultEdgeAngle
*/
inline uint8_t
ClusteredMesh::GetDefaultEdgeAngle() const
{
    return mDefaultEdgeAngle;
}


/**
\brief Sets the default edge angle.

The edge angle is the interior angle between the two trangles that form the edge, for example when
the triangles are coplanar, the angle is PI.  The edge angle is encoded in one byte, thus 128=PI.
You should round up rather than down, to avoid loss of contact as a ball rolls over an edge.
\param defaultEdgeAngle the default edge angle.
*/
inline void
ClusteredMesh::SetDefaultEdgeAngle(uint8_t defaultEdgeAngle)
{
    mDefaultEdgeAngle = defaultEdgeAngle;
}


/**
\brief Gets the size of the groupId in bytes for each mesh unit that has a group id
\return the groupId size in bytes.
*/
inline uint8_t
ClusteredMesh::GetGroupIdSize() const
{
    return mClusterParams.mGroupIdSize;
}


/**
\brief Sets the GroupIdSize.
\param GroupIdSize the new groupIdSize.
*/
inline void
ClusteredMesh::SetGroupIdSize(uint8_t groupIdSize)
{
    mClusterParams.mGroupIdSize = groupIdSize;
}


/**
\brief Gets the size of the surfaceId in bytes for each mesh unit that has a surfaceId
\return the surfaceId size in bytes
*/
inline uint8_t
ClusteredMesh::GetSurfaceIdSize() const
{
    return mClusterParams.mSurfaceIdSize;
}


/**
\brief Sets the SurfaceIdSize.
\param SurfaceIdSize the new surfaceIdSize.
*/
inline void
ClusteredMesh::SetSurfaceIdSize(uint8_t surfaceIdSize)
{
    mClusterParams.mSurfaceIdSize = surfaceIdSize;
}


/**
\brief Tests onesided flag.
\see SetOneSided
*/
inline RwpBool
ClusteredMesh::IsOneSided() const
{
    return static_cast<RwpBool>((mClusterParams.mFlags & CMFLAG_ONESIDED)!=0);
}


/**
\brief Get the granularity that is used for vertex compression.
\return the factor that is multiplied by the integer representation of the compressed vertex data.
*/
inline const float &
ClusteredMesh::GetVertexCompressionGranularity() const
{
    return mClusterParams.mVertexCompressionGranularity;
}


/**
\brief Gets a copy of the cluster param
\return The cluster params
*/
inline ClusterParams
ClusteredMesh::GetClusterParams() const
{
    return mClusterParams;
}


/**
\brief Sets the onesided flag.

The mesh is onesided by default.  When the mesh is onesided all collisions with reflex edges and the back
face of the triangle are ignored.
\param onesided new setting of the flag
*/
inline void
ClusteredMesh::SetOneSided(RwpBool onesided)
{
    if (onesided)
    {
        mClusterParams.mFlags = static_cast<uint16_t>(mClusterParams.mFlags | CMFLAG_ONESIDED);
    }
    else
    {
        mClusterParams.mFlags = static_cast<uint16_t>(mClusterParams.mFlags & ~CMFLAG_ONESIDED);
    }
}

}   // namespace collision
}   // namespace rw

// We need to specify the class serialization version prior to the class definition
// due to a problem with ps2 gcc.
// Version 2 added m_includeKDSubTrees
EA_SERIALIZATION_CLASS_VERSION(rw::collision::ClusteredMesh::ObjectDescriptor, 2)

namespace rw {
namespace collision {

struct ClusteredMesh::ObjectDescriptor
{
    ObjectDescriptor(uint32_t maxClusters,
        uint32_t clusterDataSize,
        uint32_t numBranchNodes,
        uint32_t maxUnits,
        const rw::collision::AABBox& bbox,
        RwpBool includeKDSubTrees = FALSE)
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
        m_bbox = AABBox(rwpmath::GetVector3_Zero(), rwpmath::GetVector3_Zero());
        m_includeKDSubTrees = FALSE;
    }

    uint32_t m_maxClusters;
    uint32_t m_clusterDataSize;
    uint32_t m_numBranchNodes;
    uint32_t m_maxUnits;
    rw::collision::AABBox m_bbox;
    RwpBool m_includeKDSubTrees;

    // NOTE: If any changes to this object affecting its LL-Serialization, you'll also need to
    // make identical changes to its FPU version here: ".\include\cmn\rw\collision\detail\fpu\"
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
};


inline ClusteredMesh *
ClusteredMesh::Initialize(const EA::Physics::MemoryPtr& resource, const ObjectDescriptor & objDesc)
{
    return (Initialize(resource, objDesc.m_maxClusters,
                       objDesc.m_clusterDataSize, objDesc.m_numBranchNodes,
                       objDesc.m_maxUnits, objDesc.m_bbox,
                       0.01f, sizeof(ClusteredMesh), objDesc.m_includeKDSubTrees));
}


inline EA::Physics::SizeAndAlignment
ClusteredMesh::GetResourceDescriptor(const ObjectDescriptor & objDesc)
{
    return (GetResourceDescriptor(objDesc.m_maxClusters,
                                  objDesc.m_clusterDataSize, objDesc.m_numBranchNodes,
                                  objDesc.m_maxUnits, objDesc.m_bbox,
                                  0.01f, sizeof(ClusteredMesh), objDesc.m_includeKDSubTrees));
}


inline const ClusteredMesh::ObjectDescriptor ClusteredMesh::GetObjectDescriptor() const
{
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

/**
Releases any resources that were acquired by the Initialize method.
\note Call this method just prior to freeing the memory of the mesh.
*/
inline void
ClusteredMesh::Release()
{
    // The release sets mNumClusters to 0 in order to make the mesh unusable.
    mNumClusters = 0;
}

//KDSubTree Utility functions
EA::Physics::SizeAndAlignment
GetKDSubTreeWorkSpaceResourceDescriptor(rw::collision::ClusteredMesh &clusteredMesh);

void
CreateKDSubTreeArray(rw::collision::KDSubTree *kdSubTreeArray, const EA::Physics::MemoryPtr &workspaceRes, rw::collision::ClusteredMesh &clusteredMesh);

inline void ClusteredMesh::CreateClusterKDTrees(const EA::Physics::MemoryPtr &workspaceRes)
{
    if (mKDTree->GetNumKDSubTrees())
    {
        CreateKDSubTreeArray(mKDTree->GetKDSubTrees(), workspaceRes, *this);
    }
}

inline void ClusteredMesh::SetClusterKDTrees(KDSubTree * subtrees)
{
    EA_ASSERT(subtrees);
    mKDTree->SetKDSubTrees(subtrees, GetNumCluster());
}

inline const KDSubTree *
ClusteredMesh::GetClusterKDTree(uint32_t clusterIndex) const
{
    EA_ASSERT(clusterIndex < GetNumCluster());
    if (clusterIndex < mKDTree->GetNumKDSubTrees())
    {
        return & mKDTree->GetKDSubTree(clusterIndex);
    }
    else
    {
        return NULL;
    }
}

// *****************************************************************************************************
// Serialization
/*
\autodoc
*/
// NOTE: If any changes to this object affecting its LL-Serialization, you'll also need to
// make identical changes to its FPU version here: ".\include\cmn\rw\collision\detail\fpu\"
template <class Archive>
inline void ClusteredMesh::Serialize(Archive &ar, uint32_t version)
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
        // Due to the use of pointer members the value of mSizeOfThis differs between
        // 32 and 64 bit platforms. To ensure archives are platform independent we
        // avoid serializing this value and instead calculate it on load only. This
        // call to serialize the value is purely for backwards compatibility.
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
            ar & EA::Serialization::MakeNamedValue(*reinterpret_cast<KDTree *>(mKDTree), "*mKDTree");
            // Ensure remaining data is initialized
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

    for (uint32_t i = 0; i < mNumClusters; ++i)
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

        if (cluster.compressionMode == ClusteredMeshCluster::VERTICES_16BIT_COMPRESSED)
        {
            rw::collision::ClusteredMeshCluster::CompressedVertexDataUnion vdUnion;
            vdUnion.m_as_rwpmathVector3Ptr = cluster.vertexArray;

            uint32_t* vertexArrayHeader = const_cast<uint32_t *>(reinterpret_cast<const uint32_t *>(vdUnion.m_asInt32Ptr));
            ar & EA_SERIALIZATION_NAMED_STATIC_ARRAY(vertexArrayHeader, 3u);

            uint16_t* vertexArray = const_cast<uint16_t *>(&vdUnion.m_asVertex16Ptr[0].x) + 3 * 2;
            ar & EA_SERIALIZATION_NAMED_STATIC_ARRAY(vertexArray, cluster.vertexCount * 3u);
        }
        else if (cluster.compressionMode == ClusteredMeshCluster::VERTICES_32BIT_COMPRESSED)
        {
            rw::collision::ClusteredMeshCluster::CompressedVertexDataUnion vdUnion;
            vdUnion.m_as_rwpmathVector3Ptr = cluster.vertexArray;
            int32_t* vertexArray = const_cast<int32_t *>(vdUnion.m_asInt32Ptr);
            ar & EA_SERIALIZATION_NAMED_STATIC_ARRAY(vertexArray, cluster.vertexCount * 3u);
        }
        else
        {
            if (version == 1)
            {
                // Using fpu math it is not valid to serialize the normals with the vertices
                // as there will be padding between the two arrays.
                uint32_t size = uint32_t(cluster.vertexCount + cluster.normalCount);
                rwpmath::Vector3* vertexAndNormalArray = cluster.vertexArray;
                ar & EA_SERIALIZATION_NAMED_STATIC_ARRAY(vertexAndNormalArray, size);
            }
            else
            {
                rwpmath::Vector3* vertexArray = cluster.vertexArray;
                ar & EA_SERIALIZATION_NAMED_STATIC_ARRAY(vertexArray, cluster.vertexCount);
            }
        }

        if ((cluster.compressionMode != ClusteredMeshCluster::VERTICES_UNCOMPRESSED) || (version > 1))
        {
            // normalStart is a quad-word offset
            rwpmath::Vector3* normalArray =
                reinterpret_cast<rwpmath::Vector3*>(reinterpret_cast<uintptr_t>(cluster.vertexArray) + cluster.normalStart * 16);
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

    if(ar.IsLoading())
    {
        // Initialize the Aggregate v-table
        m_vTable = &sm_vTable;
        EA_ASSERT(m_vTable != NULL);

        // Initialize the mSizeOfThis member
        const ObjectDescriptor objectDescriptor(GetObjectDescriptor());
        const EA::Physics::SizeAndAlignment sizeAndAlignment(GetResourceDescriptor(objectDescriptor));
        mSizeOfThis = sizeAndAlignment.GetSize();
    }

    EA_ASSERT(IsValid());

}


} // namespace collision
} // namespace rw



#endif // PUBLIC_RW_COLLISION_CLUSTEREDMESH_BASE_H
