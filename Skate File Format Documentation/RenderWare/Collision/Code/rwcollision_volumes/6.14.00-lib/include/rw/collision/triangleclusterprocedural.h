// (c) Electronic Arts. All Rights Reserved.
#ifndef PUBLIC_RW_COLLISION_TRIANGLECLUSTERPROCEDURAL_H
#define PUBLIC_RW_COLLISION_TRIANGLECLUSTERPROCEDURAL_H

/*************************************************************************************************************

File: triangleclusterprocedural.hpp

Purpose: Procedural aggregate containing a single clusteredmeshcluster.
*/


#include "rw/collision/common.h"
#include "rw/collision/procedural.h"
#include "rw/collision/clusteredmeshcluster.h"
#include "rw/collision/triangle.h"

#define rwcTRIANGLECLUSTERPROCEDURAL_ALIGNMENT RWMATH_VECTOR3_ALIGNMENT


namespace rw
{
namespace collision
{
    class VolumeBBoxQuery;
    class VolumeLineQuery;
    class TriangleVolume;
}
}


namespace rw
{
namespace collision
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

    /**
    \brief Gets a resource descriptor describing the memory allocation requirements of a TriangleClusterProcedural.

    \param parameters ClusterConstructionParameters describing the wrapped ClusteredMeshCluster.

    \return a resource descriptor for the memory requirements of the proposed TriangleClusterProcedural object.
    */
    static EA::Physics::SizeAndAlignment GetResourceDescriptor(
        const rw::collision::ClusterConstructionParameters &parameters);

    /**
    \brief Allocates and initializes a TriangleClusterProcedural within a provided memory resource.

    \note The parameters you pass to this method must match the ones passed to the GetResourceDescriptor call
    from which the resource was allocated.

    \param resource The resource used to allocate the object.
    \param parameters The ClusterConstructionParameters used to describe the ClusteredMeshCluster
    \return A TriangleClusterProcedural object.
    */
    static TriangleClusterProcedural *Initialize(
        const EA::Physics::MemoryPtr &resource,
        const rw::collision::ClusterConstructionParameters &parameters);

    /**
    \brief Releases any resources that were acquired by the Initialize method.

    Causes the TriangleClusterProcedural to release all internally allocated memory prior to destruction.
    */
    void Release();

    /**
    \brief Gets a reference to the cluster wrapped by the TriangleClusterProcedural.
    */
    rw::collision::ClusteredMeshCluster &GetCluster();

    /**
    \brief Gets a const reference to the cluster wrapped by the TriangleClusterProcedural.
    */
    const rw::collision::ClusteredMeshCluster &GetCluster() const;

    /**
    \brief Gets a const reference to the parameters describing the wrapped cluster.
    */
    const rw::collision::ClusterParams &GetClusterParams() const;

    /**
    \internal Implementation of rw::collision::Aggregate interface GetSizeThis virtual method.

    \return size of this object
    */
    uint32_t GetSizeThis();

    /**
    \brief Updates the object after changes to the wrapped cluster.  Uses the provided AABBox,
	instead of recalculating.
    */
    void UpdateWithBBox(const AABBox & bbox);


    /**
    \brief Updates the object after changes to the wrapped cluster.  Recomputes the AABBox
	from the mesh.
    
    Implementation of rw::collision::Aggregate interface UpdateThis virtual method.
    */
    void UpdateThis();

    /**
    \brief Performs a line intersection query with the cluster aggregate volume.

    Implementation of rw::collision::Aggregate interface LineIntersectionQueryThis virtual method.

    \param lineQuery Initialized line query structure.
    \param tm The transform of the aggregate in the query frame.

    \return TRUE if the query finished, FALSE if the results buffer overflowed and
    LineIntersectionQuery needs to be called again.

    \see rw::collision::Aggregate::LineIntersectionQuery.
    */
    RwpBool LineIntersectionQueryThis(
        rw::collision::VolumeLineQuery *lineQuery,
        const rwpmath::Matrix44Affine *tm);

    /**
    \brief Performs a bounding box overlap query with the cluster aggregate volume.

    Implementation of rw::collision::Aggregate interface BBoxOverlapQueryThis virtual method.

    \param bboxQuery Initialized bounding box query structure.
    \param tm The transform of the aggregate in the query frame.

    \return TRUE if the query finished, FALSE if the results buffer could not hold
    all the results and BBoxOverlapQuery needs to be called again.
    */
    RwpBool BBoxOverlapQueryThis(
        rw::collision::VolumeBBoxQuery *bboxQuery,
        const rwpmath::Matrix44Affine *tm);

    /**
    \brief Fills out a triangle volume with the triangle details referred to by a child index

    \param triangleVolume the triangle volume.
    \param childIndex the child index referring to the source triangle.
    */
    void GetVolumeFromChildIndex(rw::collision::TriangleVolume &volume, const uint32_t childIndex) const;

    /**
    \brief Fills out the triangle indices of the triangle referred to by a child index

    \param v0, v1, v2 the vertex indices.
    \param childIndex the child index referring to the source triangle.
    */
    void GetVertexIndicesFromChildIndex(uint8_t &v0, uint8_t &v1, uint8_t &v2, const uint32_t childIndex) const;

    /**
    \brief Gets the offset of the contained cluster unit identified by the given child index.

    \param childIndex the child index.
    \return the unit offset from the child index.
    */
    uint32_t GetUnitOffsetFromChildIndex(const uint32_t childIndex) const;

    /**
    \brief Gets the index, within its containing unit, of the contained triangle identified by the given child index.

    \param childIndex the child index.
    \return the triangle index from the child index.
    */
    uint32_t GetTriangleIndexWithinUnitFromChildIndex(const uint32_t childIndex) const;

    /**
    \brief Gets the size, in bytes, of the group ID fields of the contained units.
    \return the groupId size in bytes.
    */
    uint8_t GetGroupIdSize() const;

    /**
    \brief Sets the size, in bytes, of the group ID fields of the contained units.

    This dictates the number of bytes to be used to store each unit's group ID,
    and should be called only once.

    \param GroupIdSize the new groupIdSize.
    */
    void SetGroupIdSize(const uint8_t groupIdSize);

    /**
    Gets the size, in bytes, of the surface ID fields of the contained units.
    \return the surfaceId size in bytes
    */
    uint8_t GetSurfaceIdSize() const;

    /**
    Sets the size, in bytes, of the surface ID fields of the contained units.

    This dictates the number of bytes to be used to store each unit's surface ID,
    and should be called only once.

    \param SurfaceIdSize the new surfaceIdSize.
    */
    void SetSurfaceIdSize(const uint8_t surfaceIdSize);

    /**
    \brief Sets the granularity to be used for vertex compression within the owned cluster.

    This method should be called only once.

    \param vertexCompressionGranularity the vertex compression granularity.
    */
    void SetVertexCompressionGranularity(const float vertexCompressionGranularity);

    /**
    \brief Returns true if the contained triangles are one-sided for the purpose of collision.
    \see SetOneSided
    */
    bool IsOneSided() const;

    /**
    \brief Sets whether the contained triangles are one-sided for the purpose of collision.

    The cluster mesh is one-sided by default. When the mesh is one-sided all collisions
    with reflex edges and the back faces of triangles are ignored.

    \param onesided new setting of the flag

    \see IsOneSided
    */
    void SetOneSided(const bool onesided);

    // TriangleClusterProcedural ObjectDescriptor 
    struct ObjectDescriptor
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

    /**
    \brief Serializes the TriangleClusterProcedural.

    \param ar serialization archive being written to or read from

    \param version version number of TriangleClusterProcedural
    */
    template <class Archive>
        void Serialize(Archive &ar, uint32_t /*version*/)
    {
        // Serialize base class
        ar & EA::Serialization::MakeNamedValue(*static_cast<Procedural*>(this), "Procedural");

        // Serialize the parametes struct
        ar & EA_SERIALIZATION_NAMED_VALUE(mClusterParams.mVertexCompressionGranularity);
        ar & EA_SERIALIZATION_NAMED_VALUE(mClusterParams.mFlags);
        ar & EA_SERIALIZATION_NAMED_VALUE(mClusterParams.mGroupIdSize);
        ar & EA_SERIALIZATION_NAMED_VALUE(mClusterParams.mSurfaceIdSize);

        // Serialize the Cluster
        ar.TrackInternalPointer(mCluster);
        ar & EA_SERIALIZATION_NAMED_VALUE(*mCluster);

        // Setup the virtual table
        if(ar.IsLoading())
        {
            m_vTable = &sm_vTable;
            EA_ASSERT(m_vTable != NULL);
        }

        // Setup the mSizeOf
        if (ar.IsLoading())
        {
            // This will slow down low level serialization. It may be possible to remove this
            // member and Calculate each time the GetSizeThis method is called.
            mSizeOfThis = GetResourceDescriptor(ObjectDescriptor(mCluster->totalSize)).GetSize();
        }
    }

    /**
    \brief Initializes a TriangleClusterProcedural given a resource and ObjectDescriptor.

    \param resource memory resource in which to initialize the TriangleClusterProcedural

    \param objDesc TriangleClusterProcedural ObjectDescriptor

    \return Initialized TriangleClusterProcedural
    */
    static TriangleClusterProcedural * Initialize(const EA::Physics::MemoryPtr& resource, const ObjectDescriptor & objDesc);

    /**
    \brief Create a SizeAndAlignment object from a given ObjectDescriptor

    \param objDesc TriangleClusterProcedural ObjectDescriptor

    \return SizeAndAlignment object
    */
    static EA::Physics::SizeAndAlignment GetResourceDescriptor(const ObjectDescriptor & objDesc);

    /**
    \brief Create an ObjectDescriptor describing this TriangleClusterProcedural

    \return ObjectDescriptor object
    */
    // Return the information needed to allocate this object when deserializing
    const ObjectDescriptor GetObjectDescriptor() const;

private:

    uint32_t GetClusterSize(ClusteredMeshCluster& cluster) const;

    /**
    \internal Constructs a TriangleClusterProcedural object

    \param parameters The ClusterConstructionParameters used to describe the ClusteredMeshCluster
    \param vTable The TriangleClusterProcedural vTable
    */
    TriangleClusterProcedural(
        const rw::collision::ClusterConstructionParameters &parameters,
        rw::collision::Procedural::VTable *vTable);

    /**
    \internal Constructs a TriangleClusterProcedural object

    \param cluster pointer to the ClusteredMeshCluster
    \param vTable The TriangleClusterProcedural vTable
    */
    TriangleClusterProcedural(
        rw::collision::ClusteredMeshCluster * cluster,
        rw::collision::Procedural::VTable *vTable);

    /**
    \internal Calculates the child index of a triangle referred to by a unit offset and triangle index.

    The triangle index parameter is used to specify a triangle within the indicated unit. The range of
    values are as follows:
    Unit is a single triangle - 0.
    Unit is a triangle pair - 0 refers to first triangle / 1 refers to the second triangle.

    \param unitOffset offset of the unit.
    \param unitTriangleIndex index of the triangle in the unit.

    \return the ChildIndex of the triangle
    */
    uint32_t GetChildIndex(
        const uint32_t unitOffset,
        const uint32_t unitTriangleIndex) const;

    /**
    \internal Gets the number of bits required to store the unit tag.

    \return the number of unit tag bits.
    */
    uint32_t GetNumUnitTagBits() const;

    rw::collision::ClusterParams mClusterParams;        ///< Cluster wide parameters
    rw::collision::ClusteredMeshCluster * mCluster;     ///< The ClusteredMeshCluster

    uint32_t mSizeOfThis;                               ///< The total size of the mesh cluster aggregate

    static VTable sm_vTable;                            ///< The virtual funtion table of this procedural
};


inline rw::collision::ClusteredMeshCluster &
TriangleClusterProcedural::GetCluster()
{
    return *mCluster;
}


inline const rw::collision::ClusteredMeshCluster &
TriangleClusterProcedural::GetCluster() const
{
    return *mCluster;
}


inline const rw::collision::ClusterParams &
TriangleClusterProcedural::GetClusterParams() const
{
    return mClusterParams;
}


inline void
TriangleClusterProcedural::Release()
{
}


inline uint32_t
TriangleClusterProcedural::GetSizeThis()
{
    return mSizeOfThis;
}


inline uint32_t
TriangleClusterProcedural::GetNumUnitTagBits() const
{
    return m_numTagBits - 1;
}


inline uint32_t
TriangleClusterProcedural::GetChildIndex(
    const uint32_t unitOffset,
    const uint32_t unitTriangleIndex) const
{
    return (unitTriangleIndex << GetNumUnitTagBits()) + unitOffset;
}


inline uint32_t
TriangleClusterProcedural::GetUnitOffsetFromChildIndex(const uint32_t childIndex) const
{
    return (~(0xffffffff << GetNumUnitTagBits()) & childIndex);
}


inline uint32_t
TriangleClusterProcedural::GetTriangleIndexWithinUnitFromChildIndex(const uint32_t childIndex) const
{
    return childIndex >> GetNumUnitTagBits();
}


inline uint8_t
TriangleClusterProcedural::GetGroupIdSize() const
{
    return mClusterParams.mGroupIdSize;
}


inline void
TriangleClusterProcedural::SetGroupIdSize(const uint8_t groupIdSize)
{
    mClusterParams.mGroupIdSize = groupIdSize;
}


inline uint8_t
TriangleClusterProcedural::GetSurfaceIdSize() const
{
    return mClusterParams.mSurfaceIdSize;
}


inline void
TriangleClusterProcedural::SetSurfaceIdSize(const uint8_t surfaceIdSize)
{
    mClusterParams.mSurfaceIdSize = surfaceIdSize;
}


inline void
TriangleClusterProcedural::SetVertexCompressionGranularity(const float vertexCompressionGranularity)
{
    mClusterParams.mVertexCompressionGranularity = vertexCompressionGranularity;
}


inline bool
TriangleClusterProcedural::IsOneSided() const
{
    return (mClusterParams.mFlags & CMFLAG_ONESIDED)!= 0;
}

inline void
TriangleClusterProcedural::SetOneSided(const bool onesided)
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

inline TriangleClusterProcedural *
TriangleClusterProcedural::Initialize(const EA::Physics::MemoryPtr& resource, const ObjectDescriptor & /*objDesc*/)
{
    //return Initialize(resource, objDesc.mClusterConstructionParameters);

    // Check the alignment of the resource
    rwcASSERTALIGN(resource.GetMemory(), rwcTRIANGLECLUSTERPROCEDURAL_ALIGNMENT);

    uintptr_t res = reinterpret_cast<uintptr_t>(resource.GetMemory());
    res = EA::Physics::SizeAlign<uintptr_t>(res, rwcTRIANGLECLUSTERPROCEDURAL_ALIGNMENT);

    uintptr_t clusterRes = res;
    clusterRes += sizeof(TriangleClusterProcedural);

    // Construct the object
    TriangleClusterProcedural *agg =  new (reinterpret_cast<void*>(res)) TriangleClusterProcedural(
        reinterpret_cast<rw::collision::ClusteredMeshCluster*>(clusterRes),
        &sm_vTable);

    // Return the object
    return agg;
}

inline uint32_t
TriangleClusterProcedural::GetClusterSize(ClusteredMeshCluster& cluster) const
{
    return cluster.totalSize;
}

inline EA::Physics::SizeAndAlignment
TriangleClusterProcedural::GetResourceDescriptor(const TriangleClusterProcedural::ObjectDescriptor & objDesc)
{
    uint32_t size = 0;
    size = EA::Physics::SizeAlign<uint32_t>(sizeof(TriangleClusterProcedural), rwcCLUSTEREDMESHCLUSTER_ALIGNMENT);
    size += objDesc.mClusterSize;

    return EA::Physics::SizeAndAlignment(
        size,
        (rwcTRIANGLECLUSTERPROCEDURAL_ALIGNMENT > rwcCLUSTEREDMESHCLUSTER_ALIGNMENT ?
            rwcTRIANGLECLUSTERPROCEDURAL_ALIGNMENT : rwcCLUSTEREDMESHCLUSTER_ALIGNMENT));
}

} // namespace collision
} // namespace rw


#endif // PUBLIC_RW_COLLISION_TRIANGLECLUSTERPROCEDURAL_H
