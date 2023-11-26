// (c) Electronic Arts. All Rights Reserved.

#ifndef PUBLIC_RW_COLLISION_CLUSTEREDMESHOFFLINEBUILDER_H
#define PUBLIC_RW_COLLISION_CLUSTEREDMESHOFFLINEBUILDER_H


#include <rw/collision/common.h>

#if !defined EA_PLATFORM_PS3_SPU

#include <coreallocator/icoreallocator_interface.h>

#include <rw/collision/meshbuilder/detail/clusteredmeshbuilder.h>
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
\brief
Class for building a clustered mesh offline.

The helper uses the rw::DefaultAllocator to deal with the memory requirements of the ClusteredMeshBuilder,
and the final ClusteredMesh and has not been optimized for runtime use. See ClusteredMeshRuntimeBuilder for
runtime use.
*/
class ClusteredMeshOfflineBuilder
{
public:

    typedef meshbuilder::detail::ClusteredMeshBuilder::Parameters Parameters;

    ClusteredMeshOfflineBuilder(uint32_t           numPrim,
                                uint32_t           numVert,
                                uint32_t           numMergePlanes,
                                Parameters         &builderParams,
                                EA::Allocator::ICoreAllocator *generalAllocator);


    ~ClusteredMeshOfflineBuilder();

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
					   const rwpmath::VecFloat &  planeDistance);

    bool IsBuilderValid();

    rw::collision::ClusteredMesh * BuildClusteredMesh();

    void Release();

private:

    /// The ClusteredMeshBuilder.
    meshbuilder::detail::ClusteredMeshBuilder *m_clusteredMeshBuilder;
    /// The ParamBlock used to control the build process.
    Parameters                 m_buildParams;
    /// Wrapper around a provided allocator, used to allocate both internal working storage and the final mesh.
    meshbuilder::detail::GeneralAllocator m_allocator;

    /// Valid flag
    bool                       m_isValid;

    /// Merge Plane Count
    uint32_t                   m_mergePlaneCount;
    /// Merge Plane Normals
    rwpmath::Vector3*          m_mergePlaneNormals;
    /// Merge Plane Distances
    rwpmath::VecFloat*         m_mergePlaneDistances;

    /// Private copy constructor.
    ClusteredMeshOfflineBuilder(const ClusteredMeshOfflineBuilder & other);
    /// Private assignment constructor.
    ClusteredMeshOfflineBuilder & operator = (const ClusteredMeshOfflineBuilder & other);
};

} // namespace collision
} // namespace 



#endif // !defined EA_PLATFORM_PS3_SPU

#endif // PUBLIC_RW_COLLISION_CLUSTEREDMESHOFFLINEBUILDER_H
