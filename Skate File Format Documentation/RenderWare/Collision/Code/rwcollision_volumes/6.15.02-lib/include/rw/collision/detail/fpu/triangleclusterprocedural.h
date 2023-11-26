// (c) Electronic Arts. All Rights Reserved.
#ifndef PUBLIC_RW_COLLISION_DETAIL_FPU_TRIANGLECLUSTERPROCEDURAL_H
#define PUBLIC_RW_COLLISION_DETAIL_FPU_TRIANGLECLUSTERPROCEDURAL_H

/*************************************************************************************************************

File: triangleclusterprocedural.hpp

Purpose: Procedural aggregate containing a single clusteredmeshcluster.
*/


#include "rw/collision/common.h"
#include "rw/collision/detail/fpu/procedural.h"
#include "rw/collision/triangleclusterprocedural.h"
#include "rw/collision/clusteredmeshcluster.h"
#include "rw/collision/detail/fpu/clusteredmeshcluster.h"


namespace rw
{
namespace collision
{
namespace detail
{
namespace fpu
{


/**
\brief A procedural aggregate wrapping a single ClusteredMeshCluster.

A TriangleClusterProcedural is a procedural aggregate wrapper around a single cluster.
It accepts line queries and bounding box queries, and enables a standalone cluster to
be used as an aggregate volume for collision.

The intention is that building of individual clusters can be done rapidly at runtime,
providing a solution for teams that want to build meshes dynamically (for example,
from procedurally generated terrain).

\importlib rwccore
*/
class TriangleClusterProcedural : public Procedural
{
public:

    static EA::Physics::SizeAndAlignment GetResourceDescriptor(
        const uint32_t clusterSize);

    static TriangleClusterProcedural *Initialize(
        const EA::Physics::MemoryPtr &resource,
        const uint32_t clusterSize);

    template <class Archive>
        void Serialize(Archive &ar, uint32_t /*version*/)
    {
        // Serialize base class
        ar & EA::Serialization::MakeNamedValue(*static_cast<Procedural*>(this), "Procedural");

        // Serialize the parameters struct
        ar & EA_SERIALIZATION_NAMED_VALUE(mClusterParams.mVertexCompressionGranularity);
        ar & EA_SERIALIZATION_NAMED_VALUE(mClusterParams.mFlags);
        ar & EA_SERIALIZATION_NAMED_VALUE(mClusterParams.mGroupIdSize);
        ar & EA_SERIALIZATION_NAMED_VALUE(mClusterParams.mSurfaceIdSize);

        // Serialize the Cluster
        ar.TrackInternalPointer(mCluster);
        ar & EA_SERIALIZATION_NAMED_VALUE(*mCluster);

        if (ar.IsLoading())
        {
            mSizeOfThis = 0u;   // Not needed for fpu version
        }
    }

    struct ObjectDescriptor;

    static TriangleClusterProcedural * Initialize(const EA::Physics::MemoryPtr& resource, const ObjectDescriptor & objDesc);

    static EA::Physics::SizeAndAlignment GetResourceDescriptor(const ObjectDescriptor & objDesc);

    const ObjectDescriptor GetObjectDescriptor() const;

    uint32_t GetClusterSize(ClusteredMeshCluster& cluster) const;

private:

    TriangleClusterProcedural()
        : mCluster(NULL)
    {
        mClusterParams.mFlags = CMFLAG_ONESIDED;
        mClusterParams.mGroupIdSize = 0;
        mClusterParams.mSurfaceIdSize = 0;
        mClusterParams.mVertexCompressionGranularity = 0.0f;
    }

    rw::collision::ClusterParams mClusterParams;        ///< Cluster wide parameters
    rw::collision::detail::fpu::ClusteredMeshCluster * mCluster;     ///< The ClusteredMeshCluster

    uint32_t mSizeOfThis;                               ///< The total size of the mesh cluster aggregate
};

inline uint32_t
TriangleClusterProcedural::GetClusterSize(ClusteredMeshCluster& cluster) const
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
        bytes += RWMATH_VECTOR3_ALIGNMENT * cluster.vertexCount;
    }
    bytes = EA::Physics::SizeAlign<uint32_t>(bytes, RWMATH_VECTOR3_ALIGNMENT);
    EA_ASSERT(static_cast<uint32_t>(cluster.normalStart * RWMATH_VECTOR3_ALIGNMENT) == static_cast<uint32_t>(bytes - 16));
    bytes += RWMATH_VECTOR3_ALIGNMENT * cluster.normalCount;
    EA_ASSERT(static_cast<uint32_t>(cluster.unitDataStart * RWMATH_VECTOR3_ALIGNMENT) == static_cast<uint32_t>(bytes - 16));
    bytes += cluster.unitDataSize;
    return bytes;
}


inline EA::Physics::SizeAndAlignment
TriangleClusterProcedural::GetResourceDescriptor(
    const uint32_t clusterSize)
{
    uint32_t size = 0;
    size = EA::Physics::SizeAlign<uint32_t>(sizeof(TriangleClusterProcedural), rwcTRIANGLECLUSTERPROCEDURAL_ALIGNMENT);
    size += clusterSize;
    return EA::Physics::SizeAndAlignment(size, rwcTRIANGLECLUSTERPROCEDURAL_ALIGNMENT);
}


inline TriangleClusterProcedural *
TriangleClusterProcedural::Initialize(const EA::Physics::MemoryPtr & resource,
                                      const uint32_t /*clusterSize*/)
{
    // Check the alignment of the resource
    rwcASSERTALIGN(resource.GetMemory(), rwcTRIANGLECLUSTERPROCEDURAL_ALIGNMENT);

    uintptr_t res = reinterpret_cast<uintptr_t>(resource.GetMemory());
    res = EA::Physics::SizeAlign<uintptr_t>(res, rwcTRIANGLECLUSTERPROCEDURAL_ALIGNMENT);

    // Construct the object
    TriangleClusterProcedural *procedural = new (reinterpret_cast<void*>(res)) TriangleClusterProcedural();

    // Advance the resource pointer
    res += sizeof(TriangleClusterProcedural);

    // Allocate the ClusteredMeshCluster
    res = EA::Physics::SizeAlign<uintptr_t>(res, rwcTRIANGLECLUSTERPROCEDURAL_ALIGNMENT);
    procedural->mCluster = reinterpret_cast<rw::collision::detail::fpu::ClusteredMeshCluster*>(res);

    // Return the object
    return procedural;
}

struct TriangleClusterProcedural::ObjectDescriptor
{
    ObjectDescriptor(
        const uint32_t clusterSize)
        : mClusterSize(clusterSize)
    {
    }

    ObjectDescriptor()
    {
    }

    uint32_t mClusterSize;

    template <class Archive>
    void Serialize(Archive &ar, uint32_t /*version*/)
    {
        ar & EA_SERIALIZATION_NAMED_VALUE(mClusterSize);
    }

};

inline TriangleClusterProcedural *
TriangleClusterProcedural::Initialize(const EA::Physics::MemoryPtr& resource, const ObjectDescriptor & objDesc)
{
    return Initialize(resource, objDesc.mClusterSize);
}


inline EA::Physics::SizeAndAlignment
TriangleClusterProcedural::GetResourceDescriptor(const TriangleClusterProcedural::ObjectDescriptor & objDesc)
{
    return GetResourceDescriptor(objDesc.mClusterSize);
}

inline const TriangleClusterProcedural::ObjectDescriptor
TriangleClusterProcedural::GetObjectDescriptor() const
{
    return ObjectDescriptor(GetClusterSize(reinterpret_cast<ClusteredMeshCluster&>(*mCluster)));
}

} // namespace fpu
} // namespace detail
} // namespace collision
} // namespace rw

#endif // PUBLIC_RW_COLLISION_DETAIL_FPU_TRIANGLECLUSTERPROCEDURAL_H
