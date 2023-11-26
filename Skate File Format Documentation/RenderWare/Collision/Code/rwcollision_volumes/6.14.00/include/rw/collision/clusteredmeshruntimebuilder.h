// (c) Electronic Arts. All Rights Reserved.

#ifndef PUBLIC_RW_COLLISION_CLUSTEREDMESHRUNTIMEBUILDER_H
#define PUBLIC_RW_COLLISION_CLUSTEREDMESHRUNTIMEBUILDER_H


#include <rw/collision/common.h>

#if !defined EA_PLATFORM_PS3_SPU

#include <rw/collision/meshbuilder/detail/clusteredmeshbuilder.h>
#include <rw/collision/meshbuilder/detail/linearallocator.h>
#include <rw/collision/meshbuilder/detail/generalallocator.h>


// Forward declarations
namespace rw
{
    namespace collision
    {
        class ClusteredMesh;
    }
}


namespace rw
{
namespace collision
{


/**
\class ClusteredMeshRuntimeBuilder
\brief Helper class for building a clustered mesh at runtime.

The helper takes a block of memory, on construction, which is used to efficiently deal with memory allocation
throughout the ClusteredMesh build process. The helper also takes an allocator, on construction, which is used
to allocate the final ClusteredMesh.
*/
class ClusteredMeshRuntimeBuilder
{
public:

    typedef meshbuilder::detail::ClusteredMeshBuilder::Parameters Parameters;

    ClusteredMeshRuntimeBuilder(uint32_t           numPrim,
                                uint32_t           numVert,
                                uint32_t           numMergePlanes,
                                Parameters         &builderParams,
                                uint8_t            *builderBuffer,
                                uint32_t           builderBufferSize,
                                EA::Allocator::ICoreAllocator *clusteredMeshAllocator);

    ~ClusteredMeshRuntimeBuilder();

    void SetTriangle(uint32_t triangleIndex,
                     uint32_t vertex0Index,
                     uint32_t vertex1Index,
                     uint32_t vertex2Index,
                     uint32_t groupid = 0,
                     uint32_t surfid = 0);

    void SetVertex(uint32_t vertexIndex,
                   const rw::math::fpu::Vector3U_32& pos);

    void SetMergePlane(const uint32_t planeIndex,
                       rwpmath::Vector3::InParam planeNormal,
                       const rwpmath::VecFloat & planeDistance);

    bool IsBuilderValid();

    rw::collision::ClusteredMesh * BuildClusteredMesh();

    void Release();

private:

    /// The ClusteredMeshBuilder.
    meshbuilder::detail::ClusteredMeshBuilder *m_clusteredMeshBuilder;
    /// The ParamBlock used to control the build process.
    Parameters                 m_buildParams;
    /// The base Allocator used to deal with the ClusteredMeshBuilder memory requirements during the build process.
    meshbuilder::detail::LinearAllocator m_allocator;
    /// The Allocator used to deal with the final ClusteredMesh memory requirements.
    meshbuilder::detail::GeneralAllocator m_clusteredMeshAllocator;

    /// Valid flag
    bool                       m_isValid;

    /// Merge Plane Count
    uint32_t                   m_mergePlaneCount;
    /// Merge Plane Normals
    rwpmath::Vector3*          m_mergePlaneNormals;
    /// Merge Plane Distances
    rwpmath::VecFloat*         m_mergePlaneDistances;

    /// Private copy constructor.
    ClusteredMeshRuntimeBuilder(const ClusteredMeshRuntimeBuilder & other);
    /// Private assignment constructor.
    ClusteredMeshRuntimeBuilder & operator = (const ClusteredMeshRuntimeBuilder & other);
};

} // namespace collision
} // namespace 

#endif // !defined EA_PLATFORM_PS3_SPU

#endif // PUBLIC_RW_COLLISION_CLUSTEREDMESHRUNTIMEBUILDER_H
