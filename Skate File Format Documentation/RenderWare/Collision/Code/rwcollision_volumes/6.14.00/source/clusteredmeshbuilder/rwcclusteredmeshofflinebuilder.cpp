// (c) Electronic Arts. All Rights Reserved.


#include "rw/collision/clusteredmeshofflinebuilder.h"
#include "rw/collision/meshbuilder/detail/clusteredmeshbuilder.h"

#include "rw/collision/clusteredmesh.h"


namespace rw
{
namespace collision
{


/**
\brief Constructor

\param buffer buffer used by ClusteredMeshBuilder
\param sizeBuffer size of buffer used by ClusteredMeshBuilder
\param clusteredMeshAllocator allocator used to allocate memory for ClusteredMesh
*/
ClusteredMeshOfflineBuilder::ClusteredMeshOfflineBuilder(uint32_t           numPrim,
                                                         uint32_t           numVert,
                                                         uint32_t           numMergePlanes,
                                                         Parameters         &builderParams,
                                                         EA::Allocator::ICoreAllocator *allocator)
    : m_clusteredMeshBuilder(NULL)
    , m_allocator(allocator)
    , m_isValid(true)
    , m_mergePlaneCount(numMergePlanes)
    , m_mergePlaneNormals(NULL)
    , m_mergePlaneDistances(NULL)
{
    m_buildParams = builderParams;

    // As old triangles are no longer supported
    EA_ASSERT_MSG(m_buildParams.oldTriangles_Enable == false, ("Old triangles are no longer supported"));
    m_buildParams.oldTriangles_Enable = false;

    // As group and surface ID defaults have to be zero
    EA_ASSERT_MSG(m_buildParams.groupId_Default == 0u, ("GroupID Default is now always set to zero"));
    m_buildParams.groupId_Default = 0u;
    EA_ASSERT_MSG(m_buildParams.surfaceId_Default == 0u, ("SurfaceID Default is now always set to zero"));
    m_buildParams.surfaceId_Default = 0u;

    if (m_mergePlaneCount > 0)
    {
        // Allocate space for the plane normals
        m_mergePlaneNormals = reinterpret_cast<rwpmath::Vector3*>(m_allocator.Alloc(sizeof(rwpmath::Vector3) * m_mergePlaneCount, NULL, 0, RW_MATH_VECTOR3_ALIGNMENT));

        if (NULL == m_mergePlaneDistances)
        {
            m_isValid = false;
            return;
        }

        // Allocate space for the plane distances
        m_mergePlaneDistances = reinterpret_cast<rwpmath::VecFloat*>(m_allocator.Alloc(sizeof(rwpmath::VecFloat) * m_mergePlaneCount, NULL, 0, RW_MATH_VECTOR3_ALIGNMENT));

        if (NULL == m_mergePlaneNormals)
        {
            Release();
            m_isValid = false;
            return;
        }
    }

    // Allocate and create the ClusteredMeshBuilder
    m_clusteredMeshBuilder = reinterpret_cast<meshbuilder::detail::ClusteredMeshBuilder*>(m_allocator.Alloc(sizeof(meshbuilder::detail::ClusteredMeshBuilder), NULL, 0, 4u));

    if (NULL == m_clusteredMeshBuilder)
    {
        Release();
        m_isValid = false;
        return;
    }

    m_clusteredMeshBuilder = new (m_clusteredMeshBuilder) meshbuilder::detail::ClusteredMeshBuilder(
        numPrim,
        numVert,
        m_buildParams.vertexMerge_DistanceTolerance,
        0.0f,
        &m_allocator);

    if (!m_clusteredMeshBuilder->IsBuilderValid())
    {
        Release();
        m_isValid = false;
        return;
    }
}


ClusteredMeshOfflineBuilder::~ClusteredMeshOfflineBuilder()
{
    Release();
}


/**
\brief Sets the ith triangle with the given vertex indices, and group and surface IDs.

This method, along with SetVertex, should be used to set the builder input data, and should
be called before BuildClusteredMesh.

\param triangleIndex Index of triangle
\param vertex0Index Index of 1st triangle vertex.
\param vertex1Index Index of 2nd triangle vertex.
\param vertex2Index Index of 3rd triangle vertex.
\param groupid Triangle group ID.
\param surfid Triangle surface ID.
*/
void
ClusteredMeshOfflineBuilder::SetTriangle(uint32_t triangleIndex,
                                         uint32_t vertex0Index,
                                         uint32_t vertex1Index,
                                         uint32_t vertex2Index,
                                         uint32_t groupid,
                                         uint32_t surfid)
{
    EAPHYSICS_WARNING(true == m_isValid, "ClusteredMeshRuntimeBuilder is not in valid state");
    if (m_isValid)
    {
        m_clusteredMeshBuilder->SetTriangle(triangleIndex,
                                            vertex0Index,
                                            vertex1Index,
                                            vertex2Index,
                                            groupid,
                                            surfid);
    }
}


/**
\brief Sets the ith vertex with the given position.

This method, along with SetTriangle, should be used to set the builder input data, and should
be called before BuildClusteredMesh.

\param vertexIndex Index of vertex.
\param pos Position of vertex.
*/
void
ClusteredMeshOfflineBuilder::SetVertex(uint32_t vertexIndex,
                                       const rw::math::fpu::Vector3U_32& pos)
{
    EAPHYSICS_WARNING(true == m_isValid, "ClusteredMeshRuntimeBuilder is not in valid state");
    if (m_isValid)
    {
        m_clusteredMeshBuilder->SetVertex(vertexIndex,
                                          pos);
    }
}


/**
\brief Adds another merge plane to the collection of merge planes.

\param mergePlaneNormal Index of vertex.
\param mergePlaneDistance Position of vertex.
*/
void
ClusteredMeshOfflineBuilder::SetMergePlane(const uint32_t planeIndex,
                                           rwpmath::Vector3::InParam planeNormal,
                                           const rwpmath::VecFloat & planeDistance)
{
    EAPHYSICS_WARNING(true == m_isValid, "ClusteredMeshRuntimeBuilder is not in valid state");
    EAPHYSICS_WARNING(planeIndex < m_mergePlaneCount, "merge plane index is out of range");

    if (true == m_isValid  && planeIndex < m_mergePlaneCount)
    {
        m_mergePlaneNormals[planeIndex] = planeNormal;
        m_mergePlaneDistances[planeIndex] = planeDistance;
    }
}


/**
\brief Indicates whether or not the builder is in a valid state.

\return True if the builder is valid, false otherwise
*/
bool
ClusteredMeshOfflineBuilder::IsBuilderValid()
{
    if (!m_isValid || NULL == m_clusteredMeshBuilder || !m_clusteredMeshBuilder->IsBuilderValid())
        return false;

    return true;
}


/**
\brief Releases all resources.
*/
void
ClusteredMeshOfflineBuilder::Release()
{
    if (0 != m_clusteredMeshBuilder)
    {
        m_clusteredMeshBuilder->Release();
        m_allocator.Free(m_clusteredMeshBuilder);
        m_clusteredMeshBuilder = 0;
    }

    if (0 != m_mergePlaneNormals)
    {
        m_allocator.Free(m_mergePlaneNormals);
        m_mergePlaneNormals = 0;
    }

    if (0 != m_mergePlaneDistances)
    {
        m_allocator.Free(m_mergePlaneDistances);
        m_mergePlaneDistances = 0;
    }
}


/**
\brief Builds a Clustered Mesh

This method should be called after all input triangle and vertices have been fed into the
builder.

\return A valid ClusteredMesh pointer if the build process is successful.
*/
rw::collision::ClusteredMesh *
ClusteredMeshOfflineBuilder::BuildClusteredMesh()
{
    EA_ASSERT_MSG(NULL != m_clusteredMeshBuilder, ("clusteredMeshBuilder should not be NULL"));

    // TODO: Pointless wrapper method
    return m_clusteredMeshBuilder->BuildClusteredMesh(
        m_buildParams,
        m_mergePlaneCount,
        m_mergePlaneNormals,
        m_mergePlaneDistances,
        &m_allocator);
}


} // namespace collision
} // namespace rw
