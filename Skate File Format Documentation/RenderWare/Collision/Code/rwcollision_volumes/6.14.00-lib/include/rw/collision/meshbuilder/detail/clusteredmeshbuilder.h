// (c) Electronic Arts. All Rights Reserved.

#ifndef PUBLIC_RW_COLLISION_MESHBUILDER_DETAIL_CLUSTEREDMESHBUILDER_H
#define PUBLIC_RW_COLLISION_MESHBUILDER_DETAIL_CLUSTEREDMESHBUILDER_H


#include <rw/collision/common.h>


#if !defined EA_PLATFORM_PS3_SPU

#include <rw/collision/aabbox.h>
#include <rw/collision/clusteredmesh.h>
#include <rw/collision/clusteredmeshcluster.h>
#include <rw/collision/kdtreebuilder.h>

#include <rw/collision/meshbuilder/common.h>

#include <rw/collision/meshbuilder/detail/types.h>
#include <rw/collision/meshbuilder/detail/containers.h>
#include <rw/collision/meshbuilder/detail/unitcluster.h>
#include <rw/collision/meshbuilder/detail/unitclusterstack.h>
#include <rw/collision/meshbuilder/detail/iallocator.h>
#include <rw/collision/meshbuilder/detail/generalallocator.h>


namespace rw
{
namespace collision
{
namespace meshbuilder
{
namespace detail
{

/**
\class ClusteredMeshBuilder

\brief A utility class for building a clustered mesh.

See ClusteredMeshCreator::Create for an example how to use this class. It's pretty complicated right now
and the intention is to simplify the process eventually.

The builders efficient memory allocation pattern makes it suitable for offline and runtime use.

This class currently uses a number of EASTL container classes internally.

See http://globaltechdocs.ea.com/RWPhysics:ClusteredMesh/ClusteredMeshBuilder for further details.
*/
class ClusteredMeshBuilder
{
public:

    /**
    \typedef The vector type used internally throughout the build process.
    */
    typedef meshbuilder::VectorType VectorType;

    /**
    \typedef The AABBox type used during KDTreeGeneration Process
    */
    typedef meshbuilder::AABBoxType AABBoxType;

    /**
    The leaf map is used to store the branchId of the leaf in the xKDTree for each unit that is the first unit
    added to the leaf. We need this because initially the xKDTree is constructed from bboxes and the units
    are reference in the KDTree simply by sequential numbers 0...numUnit-1. However, it is not efficient to
    find a unit by it's number at runtime, so we need the leafMap in order to change the "start index" of
    each leaf to be a clusterId + byte offset.
    */
    typedef detail::LeafMap LeafMap;

    /**
    List of IDs as uint32_t
    */
    typedef detail::IDList IDList;

    /**
    List of VectorType
    */
    typedef detail::VertexList VertexList;

    /**
    List of Units
    */
    typedef detail::UnitList UnitList;

    /**
    List of Triangle structs
    */
    typedef detail::TriangleList TriangleList;

    /**
    List of TriangleEdgeCodes
    */
    typedef detail::TriangleEdgeCodesList TriangleEdgeCodesList;

    /**
    List of TriangleSurfaceID
    */
    typedef detail::TriangleSurfaceIDList TriangleSurfaceIDList;

    /**
    List of TriangleGroupID
    */
    typedef detail::TriangleGroupIDList TriangleGroupIDList;

    /**
    List of TriangleEdgeCosines
    */
    typedef detail::TriangleEdgeCosinesList TriangleEdgeCosinesList;

    /**
    List of TriangleNeighbors
    */
    typedef detail::TriangleNeighborsList TriangleNeighborsList;

    /**
    List of TriangleDataFlag
    */
    typedef detail::TriangleFlagsList TriangleFlagsList;

    /**
    A Unit is a triangle or quad (two joined triangles). However, the design may be
    altered to expand the scope of the unit to include fans, strips, lists etc.
    */
    typedef detail::Unit Unit;

    /**
    A struct containing three vertex indices representing a triangle.
    */
    typedef detail::Triangle Triangle;

    /**
    A struct containing data associated with a triangle.
    */
    typedef detail::TriangleEdgeCodes TriangleEdgeCodes;

    /**
    Integer ID identifying the surface of which a triangle is a component.
    */
    typedef detail::TriangleSurfaceID TriangleSurfaceID;

    /**
    Integer ID identifying the face group of which a triangle is an element.
    */
    typedef detail::TriangleGroupID TriangleGroupID;

    /**
    A class which stores triangle edge cosine information.
    */
    typedef detail::TriangleEdgeCosines TriangleEdgeCosines;

    /**
    A class which stores triangle neighboring information.
    */
    typedef detail::TriangleNeighbors TriangleNeighbors;

    /**
    A class for flag data associated with a triangle.
    */
    typedef detail::TriangleFlags TriangleFlags;

    /**
    \struct Parameters
    \brief A group of build parameters used to control various stages of the build process.
    */
    struct Parameters
    {
        /**
        \brief Constructor
        Initializes all parameters to their default values.
        */
        Parameters()
            : vertexCompression_Enable(false)
            , vertexCompression_Granularity(0.001f)
            , oldTriangles_Enable(false)
            , edgeAngles_Enable(true)
            , quads_Enable(false)
            , kdTreeBuilder_SplitThreshold(8)
            , kdTreeBuilder_LargeItemThreshold(rwcKDTREEBUILDER_DEFAULTLARGEITEMTHRESHOLD)
            , kdTreeBuilder_MinChildEntriesThreshold(rwcKDTREEBUILDER_DEFAULTMINPROPORTIONNODEENTRIES)
            , kdTreeBuilder_MaxEntriesPerNode(rwcKDTREEBUILER_DEFAULTMAXENTRIESPERNODE)
            , kdTreeBuilder_MinSimilarAreaThreshold(rwcKDTREEBUILDER_DEFAULTMINSIMILARSIZETHRESHOLD)
            , groupId_NumBytes(0)
            , groupId_Default(0)
            , surfaceId_NumBytes(0)
            , surfaceId_Default(0)
            , vertexMerge_Enable(true)
            , vertexMerge_DistanceTolerance(0.1f)
            , vertexMerge_ScaleTolerance(true)
            , internalTriangleRemoval_Enabled(false)
            , edgeCosineCorrection_Enabled(false)
            , vertexSmoothing_Enabled(false)
        {
        }

        /// Enables/Disables vertex compression. Set to true to enable.
        bool    vertexCompression_Enable;
        /// Specifies the requested vertex compression granularity.
        float vertexCompression_Granularity;
        /// Determines whether or not the ClusteredMesh will consist of "old triangles".
        bool    oldTriangles_Enable;
        /// Determines whether or not edge cosine data will be encoded in the ClusteredMesh.
        bool    edgeAngles_Enable;
        /// Determines whether or not triangles will be merged to form quads in the ClusteredMesh.
        bool    quads_Enable;
        /// The split threshold used by the KDTreeBuilder.
        uint32_t  kdTreeBuilder_SplitThreshold;
        /// The large item threshold used by the KDTreeBuilder.
        float kdTreeBuilder_LargeItemThreshold;
        /// The minimum number of entries in a child node when doing a forced split used by the KDTreeBuilder.
        float kdTreeBuilder_MinChildEntriesThreshold;
        /// The maximum entries per leaf node used by the KDTreeBuilder.
        uint32_t kdTreeBuilder_MaxEntriesPerNode;
        /// The value at which objects above this size are considered to be similar used by the KDTreeBuilder.
        float kdTreeBuilder_MinSimilarAreaThreshold;

        /**
        \brief The number of bytes each Unit will use to store the group ID.
        Range 0 - 2 : 0 = excludes group ID.
        1 - 2 = number of bytes used to store the group ID with each Unit.
        */
        uint32_t  groupId_NumBytes;
        /// The Default group ID is no longer user controlled. It is always set to zero.
        uint32_t  groupId_Default;
        /**
        \brief The number of bytes each Unit will use to store the surface ID.
        Range 0 - 2 : 0 = excludes surface ID.
        1 - 2 = number of bytes used to store the surface ID with each Unit.
        */
        uint32_t  surfaceId_NumBytes;
        /// The Default surface ID  is no longer user controlled. It is always set to zero.
        uint32_t  surfaceId_Default;
        /// Enables/Disables vertex merging.
        bool    vertexMerge_Enable;
        /**
        \brief Determines the vertex merging distance tolerance. See ClusteredMeshBuilder::m_vertexMergeDistanceTolerance
        for more information.
        */
        float vertexMerge_DistanceTolerance;
        /**
        \brief Enabled/Disables scaling of the vertex merging distance tolerance. Set to True to enable. See
        ClusteredMeshBuilder::AdjustToleranceUsingEdgeScale for more information.
        */
        bool    vertexMerge_ScaleTolerance;
        /// Enables/Disables removal of internal triangles
        bool    internalTriangleRemoval_Enabled;
        /// Enables/Disables edge cosine correction of unmatched edges
        bool    edgeCosineCorrection_Enabled;
        /// Enables/Disables vertex smoothing
        bool    vertexSmoothing_Enabled;
    };

    // Constructor
    ClusteredMeshBuilder(
        uint32_t numPrim,
        uint32_t numVert,
        float vertexMerge_DistanceTolerance,
        float angleTol,
        IAllocator *const allocator = new meshbuilder::detail::GeneralAllocator(EA::Allocator::ICoreAllocator::GetDefaultAllocator()));

    // Destructor
    ~ClusteredMeshBuilder();

    // Resource release
    void Release();

    /**
    \brief Prints internal memory allocation details.
    A debugging utility method.
    */
    void PrintInternalMemoryUse()
    {
        /*
        rwMESSAGE(("vert: %d\n", m_vertices.get_allocator().GetLimit()));
        rwMESSAGE(("vertexGroups: %d\n", m_vertexGroups.get_allocator().GetLimit()));
        rwMESSAGE(("triangles: %d\n", m_triangles.get_allocator().GetLimit()));
        rwMESSAGE(("triangleEdgeCodes: %d\n", m_triangleEdgeCodes.get_allocator().GetLimit()));
        rwMESSAGE(("triangleEdgeCosines: %d\n", m_triangleEdgeCosines.get_allocator().GetLimit()));
        rwMESSAGE(("triangleNeighbors: %d\n", m_triangleNeighbors.get_allocator().GetLimit()));
        rwMESSAGE(("triDataFlags: %d\n", m_triangleFlags.get_allocator().GetLimit()));
        rwMESSAGE(("triangleSurfaceIDs: %d\n", m_triangleSurfaceIDs.get_allocator().GetLimit()));
        rwMESSAGE(("triangleGroupIDs: %d\n", m_triangleGroupIDs.get_allocator().GetLimit()));
        rwMESSAGE(("vertMap: %d\n", m_vertMap.GetMemUsed()));
        rwMESSAGE(("unitList: %d\n", m_unitList.get_allocator().GetLimit()));
        rwMESSAGE(("unitBBoxList: %d\n", (m_unitAABBoxList ? (m_numTriangles * sizeof(AABBoxType)) : 0)));
        rwMESSAGE(("clusterListUsed: %d\n", m_UnitClusterStack.GetMemUsed()));
        */
    }

    rw::collision::ClusteredMesh *BuildClusteredMesh(
        Parameters buildParams,
        const uint32_t mergePlaneCount,
        const rwpmath::Vector3 *const mergePlaneNormals,
        const rwpmath::VecFloat *const mergePlaneDistances,
        IAllocator *const clusteredMeshAllocator);

    void SetTriangle(
        uint32_t i,
        uint32_t v0,
        uint32_t v1,
        uint32_t v2,
        uint32_t groupid = 0,
        uint32_t surfid = 0);

    void SetVertex(
        uint32_t i,
        const VectorType & pos);

    void SetEdgeCosConcaveAngleTolerance(float edgecosAngle);

    void SetClusterOptions(
        const bool vertexCompression_Enable,
        const float vertexCompression_Granularity,
        const uint16_t flagsDefault,
        const uint8_t groupId_NumBytes,
        const uint8_t surfaceId_NumBytes);

    void AdjustVertexMergeDistanceToleranceUsingEdgeScale();

    void MergeVertexGroups(bool mergeVertices = true);

    bool FixUnmatchedEdges(const uint32_t maxInputLimit);

    void DetermineClusterCompressionMode(
        bool vertexCompressionOn,
        UnitCluster &unitCluster) const;

    float CalculateMinimumGranularityForCluster(
        const UnitCluster & unitCluster) const;

    uint32_t BuildUnitList(
        bool findQuads);

    AABBoxType * GetAllUnitBBoxes() const;

    uint32_t CreateClustersUsingKDTree(
        rw::collision::KDTreeBuilder& kdtreeBuilder);

    void InitializeCluster(
        rw::collision::ClusteredMeshCluster *cluster,
        const UnitCluster & unitCluster) const;

    void InitializeClusterConstructionParameters(ClusterConstructionParameters & parameters,
                                                 const UnitCluster &unitCluster);

    UnitClusterStack& GetUnitClusterStack();

    uint16_t GetClusterSize(
        const UnitCluster & clusterid) const;

    uint32_t GetClusterTotalSize() const;

    uint16_t GetClusterUnitCount(
        uint32_t clusterid) const;

    bool IsBuilderValid();

protected:

    // Accessors

    uint32_t GetTriangleCount();

    VectorType GetVertex(const uint32_t vertexIndex) const;

    uint32_t * GetTriangleVertexIndices(uint32_t triangleIndex);

    float * GetTriangleEdgeCosines(uint32_t triangleIndex);

    uint32_t * GetTriangleNeighborIndices(uint32_t triangleIndex);

    bool GetTriangleDisabledFlag(uint32_t triangleIndex);

    void SetTriangleDisabledFlag(uint32_t triangleIndex, bool disabled);

    uint32_t GetTriangleGroupId(uint32_t triangleIndex);

    uint32_t GetTriangleSurfaceId(uint32_t triangleIndex);

protected:

    // Internal containers

    /// Input vertex container.
    VertexList              *m_vertices;
    /// Group of vertex IDs, used to translate merged vertex input vertex ID to merged vertex IDs.
    IDList                  *m_vertexGroups;
    /// Internal triangle container.
    TriangleList            *m_triangles;
    /// Internal triangle edge cosine codes container.
    TriangleEdgeCodesList    *m_triangleEdgeCodes;
    /// Internal triangle surface ID container.
    TriangleSurfaceIDList   *m_triangleSurfaceIDs;
    /// Internal triangle group ID container.
    TriangleGroupIDList     *m_triangleGroupIDs;
    /// Internal triangle edge cosine data container.
    TriangleEdgeCosinesList  *m_triangleEdgeCosines;
    /// Internal triangle neighboring index data container.
    TriangleNeighborsList    *m_triangleNeighbors;
    /// Internal triangle flag container.
    TriangleFlagsList       *m_triangleFlags;
    /// UnitClusterStack. See UnitClusterStack for more information.
    UnitClusterStack        m_unitClusterStack;
    /// Unit collection. See Unit for more details.
    UnitList                *m_unitList;
    /// Unit AABBox array.
    AABBoxType              *m_unitAABBoxList;
    /// Unit AABBox array buffer.
    uint8_t                 *m_unitAABBoxListBuffer;

    // Internal counts, flags and tolerances.

    /**
    \brief Count of valid input triangles. This is not necessarily the number of input triangles and is
    evaluated during the triangle data building step.
    */
    uint32_t                m_numTriangles;
    /// Vertex collection AABBox
    AABBoxType              m_vertAABBox;
    /// Tolerance distance used to determine when two vertices should be merged during vertex merging.
    float               m_vertexMergeDistanceTolerance;
    /// Flag to determine whether or not vertices should be compressed.
    bool                    m_compressVerts;
    /// Tolerance angle used to determine whether a concave edge should be disabled.
    float               m_edgeCosConcaveAngleTolerance;
    /// Tolerance cosine angle used to determine whether or not two triangle can be considered coplanar
    float               m_coplanarCosineTolerance;
    /// Tolerance height used to determine whether the plane distance of two triangles can be considered to be the same
    float               m_coplanarHeightTolerance;
    /// Tolerance maximum edge cosine value below which used to determine whether or not a triangle need to be merged either
    /// during FixUnmatchedEdges or MergeWithPlanes stages.
    float               m_maximumEdgeCosineMergeTolerance;
    /// Tolerance used to determine when a vertex is sitting in a concave region of the mesh.
    float               m_concaveCosineTolerance;
    /// Tolerance used while determining when, given 2 edges originating from a vertex hub, a 3rd edge lies between those
    /// two edges.
    float               m_cosineTolerance;
    /// The default unit parameters of all units.
    UnitParameters          m_unitParameters;
    /// The vertex compression granularity, used during compression
    float                   m_vertexCompressionGranularity;
    /// Builder Validity flag. Used to determine whether or not the builder is in a valid state.
    bool                    m_isBuilderValid;

    /// Main allocator. Used to deal with long term memory allocation.
    IAllocator *m_allocator;
};


/**
\brief Indicates whether or not the builder is in a valid state.

\return bool Indicating whether or not the builder is in a valid state.
*/
EA_FORCE_INLINE
bool
ClusteredMeshBuilder::IsBuilderValid()
{
    return m_isBuilderValid;
}

/**
\brief Returns the Position of a vertex.

\param vertexIndex Index of vertex.
\return Position of vertex.
*/
EA_FORCE_INLINE
ClusteredMeshBuilder::VectorType
ClusteredMeshBuilder::GetVertex(const uint32_t vertexIndex) const
{
    EA_ASSERT_MSG(m_vertices->size() > vertexIndex,"VertexIndex should be less than the vertex count");
    return (*m_vertices)[vertexIndex];
}


/**
\brief Returns a pointer to the first element of a triangle vertex index array.

\param triangleIndex Index of triangle.
\return Pointer to first element of the triangle vertex index array.
*/
EA_FORCE_INLINE
uint32_t *
ClusteredMeshBuilder::GetTriangleVertexIndices(uint32_t triangleIndex)
{
    EA_ASSERT_MSG(m_triangles->size() > triangleIndex, "Triangle Index should be less than triangle count");
    return (*m_triangles)[triangleIndex].vertices;
}


/**
\brief Returns a pointer to the first element of a triangle edge cosine array.

\param triangleIndex Index of triangle.
\return Pointer to the first element of a triangle edge cosine array.
*/
EA_FORCE_INLINE
float *
ClusteredMeshBuilder::GetTriangleEdgeCosines(uint32_t triangleIndex)
{
    EA_ASSERT_MSG(m_triangleEdgeCosines->size() > triangleIndex, "Triangle Index should be less than triangle count");
    return (*m_triangleEdgeCosines)[triangleIndex].edgeCos;
}


/**
\brief Returns a pointer to the first element of a triangle neighboring triangle index array.

\param triangleIndex Index of triangle.
\return Pointer to the first element of a triangle neighboring triangle index array.
*/
EA_FORCE_INLINE
uint32_t *
ClusteredMeshBuilder::GetTriangleNeighborIndices(uint32_t triangleIndex)
{
    EA_ASSERT_MSG(m_triangleNeighbors->size() > triangleIndex, "Triangle Index should be less than triangle count");
    return (*m_triangleNeighbors)[triangleIndex].neighbor;
}


/**
\brief Returns the state of the disabled flag of a given triangle.

\param triangleIndex Index of triangle.
\return Flag indicating the state of the triangles disabled flag.
*/
EA_FORCE_INLINE
bool
ClusteredMeshBuilder::GetTriangleDisabledFlag(uint32_t triangleIndex)
{
    // Note that the API is negative ("disabled") but the flags are stored positive ("enabled")
    EA_ASSERT_MSG(m_triangleFlags->size() > triangleIndex, "Triangle Index should be less than triangle count");
    return ((*m_triangleFlags)[triangleIndex].enabled == false);
}


/**
\brief Sets the state of the disabled flag of a given triangle.

\param triangleIndex Index of triangle.
*/
EA_FORCE_INLINE
void
ClusteredMeshBuilder::SetTriangleDisabledFlag(uint32_t triangleIndex, bool disabled)
{
    // Note that the API is negative ("disabled") but the flags are stored positive ("enabled")
    EA_ASSERT_MSG(m_triangleFlags->size() > triangleIndex, "Triangle Index should be less than triangle count");

    if (disabled)
    {
        (*m_triangleFlags)[triangleIndex].enabled = false;
    }
    else
    {
        (*m_triangleFlags)[triangleIndex].enabled = true;
    }
}


/**
\brief Returns the group ID of a given triangle.

\param triangleIndex Index of triangle.
\return group ID of triangle.
*/
EA_FORCE_INLINE
uint32_t
ClusteredMeshBuilder::GetTriangleGroupId(uint32_t triangleIndex)
{
	EA_ASSERT_MSG(m_triangleGroupIDs->size() > triangleIndex, "Triangle Index should be less than triangle count");
	return (*m_triangleGroupIDs)[triangleIndex];
}


/**
\brief Returns the group ID of a given triangle.

\param triangleIndex Index of triangle.
\return group ID of triangle.
*/
EA_FORCE_INLINE
uint32_t
ClusteredMeshBuilder::GetTriangleSurfaceId(uint32_t triangleIndex)
{
	EA_ASSERT_MSG(m_triangleSurfaceIDs->size() > triangleIndex, "Triangle Index should be less than triangle count");
	return (*m_triangleSurfaceIDs)[triangleIndex];
}


/**
\brief Returns the triangle count.

\return number of triangles.
*/
EA_FORCE_INLINE
uint32_t
ClusteredMeshBuilder::GetTriangleCount()
{
    return m_triangles->size();
}


} // namespace detail
} // namespace meshbuilder
} // namespace collision
} // namespace


#endif // !defined EA_PLATFORM_PS3_SPU

#endif // PUBLIC_RW_COLLISION_MESHBUILDER_DETAIL_CLUSTEREDMESHBUILDER_H
