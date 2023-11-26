// (c) Electronic Arts. All Rights Reserved.


#include <stdio.h>

#include <rw/collision/meshbuilder/detail/clusteredmeshbuilder.h>

#include <rw/collision/meshbuilder/vertexmerger.h>
#include <rw/collision/meshbuilder/edgecodegenerator.h>
#include <rw/collision/meshbuilder/unitlistbuilder.h>
#include <rw/collision/meshbuilder/vertexcompression.h>

#include <rw/collision/meshbuilder/detail/clusterdatabuilder.h>
#include <rw/collision/meshbuilder/detail/clusterparametersbuilder.h>
#include <rw/collision/meshbuilder/detail/gridspatialmap.h>
#include <rw/collision/meshbuilder/detail/clusteredmeshbuildermethods.h>
#include <rw/collision/meshbuilder/detail/vertextrianglemap.h>
#include <rw/collision/meshbuilder/detail/triangleneighborfinder.h>


namespace rw
{
namespace collision
{
namespace meshbuilder
{
namespace detail
{


/**
\internal
These are used internally.
*/
#define CLUSTEREDMESHBUILDER_NOGROUP 0xfffffffe  // default box-group id
#define CLUSTEREDMESHBUILDER_TRIMMED 0xfffffffd  // flag to remove a triangle from a box-group


/**
\brief Constructor.

\param numTri Maximum input triangle count.
\param numVert Maximum input vertex count.
\param vertexMerge_DistanceTolerance Linear vertex merging tolerance factor.
\param angleTol Unused parameter.
\param mainAlloc Allocator used to allocate buffers for persistent data.
\param scratchAlloc Allocator used to allocate buffers for transient data.
*/
ClusteredMeshBuilder::ClusteredMeshBuilder(
    uint32_t numTri,
    uint32_t numVert,
    float vertexMerge_DistanceTolerance,
    float /*angleTol*/,
    IAllocator *const allocator) :
  m_vertices(0),
  m_vertexGroups(0),
  m_triangles(0),
  m_triangleEdgeCodes(0),
  m_triangleSurfaceIDs(0),
  m_triangleGroupIDs(0),
  m_triangleEdgeCosines(0),
  m_triangleNeighbors(0),
  m_triangleFlags(0),
  m_unitList(0),
  m_unitAABBoxList(NULL),
  m_unitAABBoxListBuffer(NULL),
  m_numTriangles(numTri),
  m_vertAABBox(
      0.0f, 0.0f, 0.0f,
      0.0f, 0.0f, 0.0f),
  m_vertexMergeDistanceTolerance(vertexMerge_DistanceTolerance),
  m_edgeCosConcaveAngleTolerance(-1.0f),
  m_coplanarCosineTolerance(0.01f),
  m_coplanarHeightTolerance(0.05f),
  m_maximumEdgeCosineMergeTolerance(0.1f),
  m_concaveCosineTolerance(0.15f),
  m_cosineTolerance(0.05f),
  m_isBuilderValid(true),
  m_allocator(allocator)
{
    // Initialize the vertex bounding box to an inverted box
    const rwpmath::VecFloat max = rwpmath::GetVecFloat_MaxValue();
    m_vertAABBox.m_min = AABBoxType::Vector3Type(max, max, max);
    m_vertAABBox.m_max = AABBoxType::Vector3Type(-max, -max, -max);

    // Mark permanent heap and allocate long-term vertex and triangle containers
    // These are allocated for the lifetime of the class and are freed by Release()
    m_allocator->Mark(EA::Allocator::MEM_PERM);

    m_vertices = VertexList::Allocate(m_allocator, numVert, EA::Allocator::MEM_PERM);
    if (!m_vertices)
    {
        m_isBuilderValid = FALSE;
        return;
    }

    m_triangles = TriangleList::Allocate(m_allocator, m_numTriangles, EA::Allocator::MEM_PERM);
    if (!m_triangles)
    {
        m_isBuilderValid = FALSE;
        return;
    }

    m_triangleSurfaceIDs = TriangleSurfaceIDList::Allocate(m_allocator, m_numTriangles, EA::Allocator::MEM_PERM);
    if (!m_triangleSurfaceIDs)
    {
        m_isBuilderValid = FALSE;
        return;
    }

    m_triangleGroupIDs = TriangleGroupIDList::Allocate(m_allocator, m_numTriangles, EA::Allocator::MEM_PERM);
    if (!m_triangleGroupIDs)
    {
        m_isBuilderValid = FALSE;
        return;
    }

    m_triangleEdgeCodes = TriangleEdgeCodesList::Allocate(m_allocator, m_numTriangles, EA::Allocator::MEM_PERM);
    if (!m_triangleEdgeCodes)
    {
        m_isBuilderValid = FALSE;
        return;
    }

    m_vertices->resize(numVert);
    m_triangles->resize(m_numTriangles);
    m_triangleSurfaceIDs->resize(m_numTriangles);
    m_triangleGroupIDs->resize(m_numTriangles);
    m_triangleEdgeCodes->resize(m_numTriangles);
}


/**
\brief Destructor.
Deallocates remaining memory.
*/
ClusteredMeshBuilder::~ClusteredMeshBuilder()
{
    Release();
}


/**
\brief Releases remaining assets
*/
void
ClusteredMeshBuilder::Release()
{
    // Free long-term vertex and triangle containers
    TriangleEdgeCodesList::Free(m_allocator, m_triangleEdgeCodes);
    TriangleGroupIDList::Free(m_allocator, m_triangleGroupIDs);
    TriangleSurfaceIDList::Free(m_allocator, m_triangleSurfaceIDs);
    TriangleList::Free(m_allocator, m_triangles);
    VertexList::Free(m_allocator, m_vertices);

    // Release the permanent heap; done with long-term vertex and triangle containers
    m_allocator->Release(EA::Allocator::MEM_PERM);
}


rw::collision::ClusteredMesh *
ClusteredMeshBuilder::BuildClusteredMesh(
    Parameters buildParams,
    const uint32_t mergePlaneCount,
    const rwpmath::Vector3 *const mergePlaneNormals,
    const rwpmath::VecFloat *const mergePlaneDistances,
    IAllocator *const clusteredMeshAllocator)
{
    rw::collision::ClusteredMesh *clusteredMesh = NULL;

    // Set Cluster Options

    // Set unit flags
    uint32_t unitFlags = 0;

    // Default flags
    unitFlags |= buildParams.edgeAngles_Enable ? UNITFLAG_EDGEANGLE : 0;
    unitFlags |= buildParams.groupId_NumBytes ? UNITFLAG_GROUPID : 0;
    unitFlags |= buildParams.surfaceId_NumBytes ? UNITFLAG_SURFACEID : 0;

    // This flag overrides all others
    unitFlags = buildParams.oldTriangles_Enable ? UNITFLAG_USEOLDTRI : unitFlags;

    SetClusterOptions(
        buildParams.vertexCompression_Enable,
        buildParams.vertexCompression_Granularity,
        static_cast<uint16_t>(unitFlags),
        static_cast<uint8_t>(buildParams.groupId_NumBytes),
        static_cast<uint8_t>(buildParams.surfaceId_NumBytes));

    // Adjust Edge Length Tolerance
    if (buildParams.vertexMerge_ScaleTolerance)
    {
        AdjustVertexMergeDistanceToleranceUsingEdgeScale();

        if (!IsBuilderValid())
            return clusteredMesh;
    }

    // Merge Vertices
    MergeVertexGroups(buildParams.vertexMerge_Enable);

    if (!IsBuilderValid())
        return clusteredMesh;

    // Mark permanent heap and allocate triangle adjacency data containers
    m_allocator->Mark(EA::Allocator::MEM_PERM);

    m_triangleFlags = TriangleFlagsList::Allocate(m_allocator, m_numTriangles, EA::Allocator::MEM_PERM);
    if (!m_triangleFlags)
    {
        m_isBuilderValid = FALSE;
        return clusteredMesh;
    }

    m_triangleEdgeCosines = TriangleEdgeCosinesList::Allocate(m_allocator, m_numTriangles, EA::Allocator::MEM_PERM);
    if (!m_triangleEdgeCosines)
    {
        m_isBuilderValid = FALSE;
        return clusteredMesh;
    }
    
    m_triangleNeighbors = TriangleNeighborsList::Allocate(m_allocator, m_numTriangles, EA::Allocator::MEM_PERM);
    if (!m_triangleNeighbors)
    {
        m_isBuilderValid = FALSE;
        return clusteredMesh;
    }

    m_triangleFlags->resize(m_numTriangles);
    m_triangleEdgeCosines->resize(m_numTriangles);
    m_triangleNeighbors->resize(m_numTriangles);

    // Validate all triangles
    const uint32_t numValidTriangles = ClusteredMeshBuilderMethods::ValidateTriangles(
                                           *m_triangleFlags,
                                           *m_triangles,
                                           *m_vertices);

    // Check that not all triangles have been removed.
    if (0u == numValidTriangles)
    {
        m_isBuilderValid = FALSE;
        return clusteredMesh;
    }

    detail::TriangleNeighborFinder::InitializeTriangleEdgeCosines(*m_triangleEdgeCosines);
    detail::TriangleNeighborFinder::InitializeTriangleNeighbors(*m_triangleNeighbors);

    // Mark temporary heap before allocation of the vertex triangle map
    m_allocator->Mark(EA::Allocator::MEM_TEMP);

    // Create the vertex map, implicitly using MEM_TEMP, the temporary heap
    // A container used to associate vertex indices with the indices of triangle which own that vertex.
    VertexTriangleMap vertexTriangleMap;
    vertexTriangleMap.Initialize(m_numTriangles, m_allocator);

    if (vertexTriangleMap.IsValid())
    {
        detail::TriangleNeighborFinder::InitializeVertexTriangleMap(vertexTriangleMap, *m_triangles);
    }
    else
    {
        m_isBuilderValid = FALSE;
    }

    if (!IsBuilderValid())
        return clusteredMesh;

    // Removing Internal Triangles
    if (buildParams.internalTriangleRemoval_Enabled)
    {
        ClusteredMeshBuilderMethods::DisableInternalTriangles(
            *m_triangleFlags,
            *m_triangles,
            *m_triangleGroupIDs,
            *m_vertices,
            vertexTriangleMap);
    }

    // Determine triangle connectivity, finding neighboring triangles and edgecosines
    detail::TriangleNeighborFinder::FindTriangleNeighbors(
        *m_triangles,
        *m_triangleEdgeCosines,
        *m_triangleNeighbors,
        *m_triangleFlags,
        *m_vertices,
        vertexTriangleMap);

    if (!IsBuilderValid())
        return clusteredMesh;

    // Merge triangles with planes
    ClusteredMeshBuilderMethods::MergeWithPlanes(
        *m_triangleEdgeCosines,
        *m_triangleNeighbors,
        *m_triangles,
        *m_triangleFlags,
        *m_vertices,
        mergePlaneNormals,
        mergePlaneDistances,
        mergePlaneCount,
        m_coplanarCosineTolerance,
        m_coplanarHeightTolerance,
        m_maximumEdgeCosineMergeTolerance);

    // Fix unmatched edges, correcting edge cosine values
    if (buildParams.edgeCosineCorrection_Enabled)
    {
        // TODO: This value is arbitary, needs to be replaced with an estimate or user set value.
        const uint32_t maxInputLimit(3000u);
        FixUnmatchedEdges(maxInputLimit);
    }

    if (!IsBuilderValid())
        return clusteredMesh;

    // Encode the triangle data
    EdgeCodeGenerator::GenerateTriangleEdgeCodes(
        *m_triangleEdgeCodes,
        *m_triangleEdgeCosines,
        *m_triangleNeighbors,
        m_edgeCosConcaveAngleTolerance);

    if (buildParams.vertexSmoothing_Enabled)
    {
        ClusteredMeshBuilderMethods::SmoothVertices(
            vertexTriangleMap,
            *m_triangles,
            *m_triangleEdgeCodes,
            *m_triangleFlags,
            *m_vertices,
            m_coplanarCosineTolerance,
            m_cosineTolerance,
            m_concaveCosineTolerance);
    }

    vertexTriangleMap.Release();

    // Release temporary heap after freeing of the vertex triangle map
    m_allocator->Release(EA::Allocator::MEM_TEMP);

    // Allocate the unit list on the temporary heap
    m_unitList = UnitList::Allocate(m_allocator, m_numTriangles, EA::Allocator::MEM_TEMP);
    if (!m_unitList)
    {
        m_isBuilderValid = FALSE;
        return clusteredMesh;
    }

    // Create lists of units on which the clusters will be based
    const uint32_t numUnits = BuildUnitList(buildParams.quads_Enable);

    // Free triangle adjacency data containers
    TriangleNeighborsList::Free(m_allocator, m_triangleNeighbors);
    TriangleEdgeCosinesList::Free(m_allocator, m_triangleEdgeCosines);
    TriangleFlagsList::Free(m_allocator, m_triangleFlags);

    // Release the permanent heap; done with triangle adjacency data containers
    m_allocator->Release(EA::Allocator::MEM_PERM);

    if (!IsBuilderValid())
        return clusteredMesh;

    // Build the KDTree, implicitly using the temporary heap
    rw::collision::AABBoxU *unitAABBoxes = GetAllUnitBBoxes();
    KDTreeBuilder kdTreeBuilder(*m_allocator);

    kdTreeBuilder.BuildTree(
        numUnits,
        unitAABBoxes,
        buildParams.kdTreeBuilder_SplitThreshold,
        buildParams.kdTreeBuilder_LargeItemThreshold,
        buildParams.kdTreeBuilder_MinChildEntriesThreshold,
        buildParams.kdTreeBuilder_MaxEntriesPerNode,
        buildParams.kdTreeBuilder_MinSimilarAreaThreshold);

    if (!kdTreeBuilder.SuccessfulBuild())
        return clusteredMesh;

    // Create the Clusters using the KDTree
    uint32_t numBranchNodes = kdTreeBuilder.GetNumBranchNodes();
    rw::collision::AABBox rootBBox = kdTreeBuilder.GetRootBBox();
    uint32_t numClusters = CreateClustersUsingKDTree(kdTreeBuilder);

    if (!IsBuilderValid())
        return clusteredMesh;

    UnitClusterStack & unitClusterStack = GetUnitClusterStack();
    UnitClusterStack::ClusterIterator it = unitClusterStack.Begin();
    const UnitClusterStack::ClusterIterator itEnd = unitClusterStack.End();

    // Determine the Cluster Compression modes
    while (it != itEnd)
    {
        UnitCluster * unitCluster = *it;
        DetermineClusterCompressionMode(buildParams.vertexCompression_Enable, *unitCluster);
        ++it;
    }

    // Determine required granularity
    float granularityNeeded = 0.0f;
    if (buildParams.vertexCompression_Enable)
    {
        it = unitClusterStack.Begin();

        while (it != itEnd)
        {
            UnitCluster * unitCluster = *it;

            float minGranularityForThis = CalculateMinimumGranularityForCluster(*unitCluster);
            if ( unitCluster->clusterID == 0 || ( minGranularityForThis > granularityNeeded ) )
            {
                granularityNeeded = minGranularityForThis;
            }
            ++it;
        }

        granularityNeeded *= 2.0f;
        EAPHYSICS_MESSAGE( "Granularity needed to fit all clusters in 16 bits: %f", granularityNeeded );

        if (granularityNeeded > buildParams.vertexCompression_Granularity)
        {
            EAPHYSICS_MESSAGE("Vertex compression granularity exceeded: needed %f, allowed %f",
                       granularityNeeded,
                       buildParams.vertexCompression_Granularity);
        }

        granularityNeeded = buildParams.vertexCompression_Granularity;
    }

    // Allocate and Create the runtime ClusteredMesh
    EA::Physics::SizeAndAlignment resDesc = ClusteredMesh::GetResourceDescriptor(numClusters,
                                                                                 GetClusterTotalSize(),
                                                                                 numBranchNodes,
                                                                                 numUnits,
                                                                                 rootBBox,
                                                                                 granularityNeeded,
                                                                                 sizeof(ClusteredMesh),
                                                                                 TRUE);

    // Allocate the resource
    void * resource = clusteredMeshAllocator->Alloc(resDesc.GetSize(), NULL, 0, resDesc.GetAlignment());
    EA::Physics::MemoryPtr rwResource(resource);

    // Initialize the ClusteredMesh
    clusteredMesh = ClusteredMesh::Initialize(rwResource,
                                              numClusters,
                                              GetClusterTotalSize(),
                                              numBranchNodes,
                                              numUnits,
                                              rootBBox,
                                              granularityNeeded,
                                              sizeof(ClusteredMesh),
                                              TRUE);

    // Check that the allocation and initialization were successful
    if (!clusteredMesh)
    {
        clusteredMeshAllocator->Free(resource);
        return clusteredMesh;
    }

    clusteredMesh->SetGroupIdSize(static_cast<uint8_t>(buildParams.groupId_NumBytes));
    clusteredMesh->SetSurfaceIdSize(static_cast<uint8_t>(buildParams.surfaceId_NumBytes));

    // Populate each Cluster of the runtime ClusteredMesh
    it = unitClusterStack.Begin();

    while (it != itEnd)
    {
        UnitCluster * unitCluster = *it;

        ClusterConstructionParameters parameters;

        InitializeClusterConstructionParameters(parameters, *unitCluster);

        ClusteredMeshCluster *newcluster = clusteredMesh->AllocateNextCluster(parameters);

        InitializeCluster(newcluster, *unitCluster);

        ++it;
    }

    // Initialize the runtime KDTree
    kdTreeBuilder.InitializeRuntimeKDTree(clusteredMesh->GetKDTree());

    // Update the ClusteredMesh after having populated each cluster and initalizing the KDTree
    clusteredMesh->Update();

    // Create the workspace for KDSubTree array creation
    m_allocator->Mark(EA::Allocator::MEM_TEMP);
    EA::Physics::SizeAndAlignment workspaceDesc(rw::collision::GetKDSubTreeWorkSpaceResourceDescriptor(*clusteredMesh));
    void *workspace = m_allocator->Alloc(workspaceDesc.size, NULL, EA::Allocator::MEM_TEMP, workspaceDesc.alignment);
    if (NULL == workspace)
    {
        EAPHYSICS_MESSAGE("While generating the KDSubTree the builder allocator ran out of memory.");
        m_isBuilderValid = FALSE;
        return clusteredMesh;
    }

    EA::Physics::MemoryPtr workspaceRes(workspace);
     
    // Create KDSubTree array from ClusteredMesh
    KDTreeWithSubTrees *kdSubTreeArray = static_cast<KDTreeWithSubTrees *>(static_cast<KDTreeBase *>(clusteredMesh->GetKDTree()));
    rw::collision::CreateKDSubTreeArray(kdSubTreeArray->GetKDSubTrees(), workspaceRes, *clusteredMesh);

    // Free KDSubTree workspace as it is no longer needed
    m_allocator->Free(workspace);
    m_allocator->Release(EA::Allocator::MEM_TEMP);

    // Deallocate workspace data
    UnitList::Free(m_allocator, m_unitList);
    m_unitClusterStack.Release();

    // Return the ClusteredMesh
    return clusteredMesh;
}


/**
\brief Set options for clusters, what is stored, and how it is stored.

\param vertexCompression_Enable Compress vertices using base/offset.
\param compress_Normals Unused parameter.
\param flags_Default Default unit flags, see ClusteredMesh::UnitTypeAndFlags.
\param group_Size 1, or 2 bytes, size of group id of each triangle.
\param surf_Size 1, or 2 bytes, size of surface id of each triangle.
*/
void
ClusteredMeshBuilder::SetClusterOptions(const bool vertexCompression_Enable,
                                        const float vertexCompression_Granularity,
                                        const uint16_t flagsDefault,
                                        const uint8_t groupId_NumBytes,
                                        const uint8_t surfaceId_NumBytes)
{
    EA_ASSERT_MSG(m_isBuilderValid, "Builder is in an invalid state - memory allocation has failed before this point");
    EA_ASSERT_MSG(!((flagsDefault != rw::collision::UNITFLAG_USEOLDTRI) && (flagsDefault & rw::collision::UNITFLAG_NORMAL)),
               "Unit normals are no longer supported by the ClusteredMeshBuilder");
    EA_ASSERT_MSG(flagsDefault != rw::collision::UNITFLAG_USEOLDTRI, ("Old triangle format is no longer supported by the ClusteredMeshBuilder"));

    m_compressVerts = vertexCompression_Enable;
    m_vertexCompressionGranularity = vertexCompression_Granularity;

    // If unitFlagsdefault is set to UNITFLAG_USEOLDTRI then all flags are unset. Here UNITFLAGS_OLDTRIANGLE corresponds to 0.
    // If unitFlagsdefault is not set the UNITFLAG_USEOLDTRI then the UNITFLAG_NORMAL flag is unset.
    m_unitParameters.unitFlagsDefault = static_cast<uint8_t>((flagsDefault == UNITFLAG_USEOLDTRI) ? UNITTYPE_OLDTRIANGLE : flagsDefault & ~rw::collision::UNITFLAG_NORMAL);
    m_unitParameters.groupIDSize = static_cast<uint8_t>(groupId_NumBytes);
    m_unitParameters.surfaceIDSize = static_cast<uint8_t>(surfaceId_NumBytes);
}


/**
\brief Sets the ith triangle with the given vertex indices, and group and surface IDs.

\param i Index of the triangle.
\param v0 Index of the 1st triangle vertex.
\param v1 Index of the 2nd triangle vertex.
\param v2 Index of the 3rd triangle vertex.
\param groupid Triangle group ID.
\param surfid Triangle surface ID.
*/
void
ClusteredMeshBuilder::SetTriangle(uint32_t i, uint32_t v0, uint32_t v1, uint32_t v2, 
                                  uint32_t groupid, uint32_t surfid)
{
    EA_ASSERT_MSG(m_isBuilderValid, "ClusteredMeshBuilder::SetTriangle: builder is not valid.\n");
    EA_ASSERT_MSG(i <= m_triangles->size(), "ClusteredMeshBuilder::SetTriangle: triangle index out of range.\n");

    (*m_triangles)[i].vertices[0] = v0;
    (*m_triangles)[i].vertices[1] = v1;
    (*m_triangles)[i].vertices[2] = v2;

    (*m_triangleGroupIDs)[i] = groupid;
    (*m_triangleSurfaceIDs)[i] = surfid;
}


/**
\brief Sets the ith vertex with the given position.

\param i Index of the vertex.
\param pos Position of the vertex.
*/
void
ClusteredMeshBuilder::SetVertex(uint32_t i, const rw::math::fpu::Vector3U_32& pos)
{
    EA_ASSERT_MSG(m_isBuilderValid, "Builder is in an invalid state - memory allocation has failed before this point");
    EA_ASSERT_MSG(i <= m_vertices->size(), "ClusteredMeshBuilder::SetTriangle: triangle index out of range.\n");

    m_vertAABBox.Set(Min(m_vertAABBox.Min(), AABBoxType::Vector3Type(pos)),
                   Max(m_vertAABBox.Max(), AABBoxType::Vector3Type(pos)));

    (*m_vertices)[i] = pos;
}


/**
\brief edgecos angle tolerance used to control disabling of concave edges.
The edgecos tolerance describes the upper limit of the edgecos below which
an edge will will be disabled. The edgecos is the cosine of the angle between
the normals of the two related triangles.
\param edgecosAngle the edgecos angle tolerance. Valid values are between 1 and -1.
*/
void
ClusteredMeshBuilder::SetEdgeCosConcaveAngleTolerance(float edgecosAngle)
{
    EA_ASSERT_MSG(edgecosAngle <= 1 && edgecosAngle >= -1, "edgecosAngle should be within range of 1 to -1");

    if (edgecosAngle > 1)
    {
        edgecosAngle = 1;
    }
    else if (edgecosAngle < -1)
    {
        edgecosAngle = -1;
    }

    m_edgeCosConcaveAngleTolerance = edgecosAngle;
}


/**
\brief Adjusts the tolerance used to control vertex merging.
The complete collection of input triangle edges are measured and an average and minimum edge length is
calculated. The adjustment factor is determined by the larger of the minimum edge length and 100th of
the average edge length. The vertex merge distance tolerance is then multiplied by the adjustment factor.
*/
void
ClusteredMeshBuilder::AdjustVertexMergeDistanceToleranceUsingEdgeScale()
{
    EA_ASSERT_MSG(m_isBuilderValid, "Builder is in an invalid state - memory allocation has failed before this point");
    EA_ASSERT_MSG(m_triangles->size() != 0, "Input triangle count should not be zero");
    EA_ASSERT_MSG(m_vertices->size() != 0, "Input vertex count should not be zero");

    rwpmath::VecFloat averageEdgeLength = rwpmath::GetVecFloat_Zero();
    rwpmath::VecFloat minimumEdgeLength = rwpmath::MAX_FLOAT;

    ClusteredMeshBuilderMethods::CalculateAverageAndMinimumEdgeLength(
        averageEdgeLength,
        minimumEdgeLength,
        *m_triangles,
        *m_vertices);

    rwpmath::VecFloat tolerance = m_vertexMergeDistanceTolerance;

    ClusteredMeshBuilderMethods::AdjustVertexMergeDistanceTolerance(
        tolerance,
        averageEdgeLength,
        minimumEdgeLength);

    m_vertexMergeDistanceTolerance = tolerance;
}


/**
\brief Merges vertices which are within a separation tolerance of each other.

\param mergeVertices Flag used to control whether or not merging takes place.
*/
void
ClusteredMeshBuilder::MergeVertexGroups(bool mergeVertices)
{
    EA_ASSERT_MSG(m_isBuilderValid, "Builder is in an invalid state - memory allocation has failed before this point");
    EA_ASSERT_MSG(m_triangles->size() != 0, "Input triangle count should not be zero");
    EA_ASSERT_MSG(m_vertices->size() != 0, "Input vertex count should not be zero");

    if (mergeVertices)
    {
        const uint32_t numVertices(m_vertices->size());

        m_allocator->Mark(EA::Allocator::MEM_TEMP);

        m_vertexGroups = IDList::Allocate(m_allocator, numVertices, EA::Allocator::MEM_TEMP);
        if (!m_vertexGroups)
        {
            m_isBuilderValid = FALSE;
            return;
        }

        m_vertexGroups->resize(numVertices);

        // Initialize the vertex groups
        for (uint32_t i = 0; i < numVertices; ++i)
        {
            (*m_vertexGroups)[i] = i;
        }

        // Merge the vertices
        // This uses only MEM_TEMP internally
        if (!VertexMerger::MergeVertexGroups(
            *m_vertexGroups,
            *m_allocator,
            m_vertAABBox,
            m_vertexMergeDistanceTolerance,
            *m_vertices))
        {
            m_isBuilderValid = false;
            return;
        }

        // Merge the triangle vertex indices.
        VertexMerger::UpdateTriangleVertexIndices(
            *m_triangles,
            *m_vertexGroups);

        IDList::Free(m_allocator, m_vertexGroups);

        m_allocator->Release(EA::Allocator::MEM_TEMP);
    }
}


/**
\brief Fixes edge cosines of edges which have not been neighbored correctly

An edge can be considered to not have neighbored properly if it intersects a
triangle which is not its neighbor, and the edge lies in the plane of the intersected triangle.
This method corrects the edge cosines of edges which lie horizontally and are shared between
a downward facing and upward facing triangle.

A GridSpatialMap is used to reduce the complexity of the issue. The current
implementation of this spatial map makes it difficult to predict the amount of
memory it requires without having added all of the entries.

The current pattern of use consists of setting a maximum number of entries.
The map is filled to this limit and then used. After use the map is filled again
with the remaining entries and used again. This repeats until all entries have
been dealt with.

\return true if successful, false if memory allocation fails.
*/
bool
ClusteredMeshBuilder::FixUnmatchedEdges(const uint32_t maxInputLimit)
{
    EA_ASSERT_MSG(m_isBuilderValid, "Builder is in an invalid state - memory allocation has failed before this point");
    EA_ASSERT_MSG(m_triangles->size() != 0, "m_triangles count should not be zero");
    EA_ASSERT_MSG(m_triangleEdgeCodes->size() != 0, "m_triangleEdgeCodes count should not be zero");
    EA_ASSERT_MSG(m_triangleEdgeCosines->size() != 0, "m_triangleEdgeCosines count should not be zero");
    EA_ASSERT_MSG(m_triangleNeighbors->size() != 0, "m_triangleNeighbors count should not be zero");
    EA_ASSERT_MSG(m_triangleFlags->size() != 0, "m_triangleFlags count should not be zero");
    EA_ASSERT_MSG(m_vertices->size() != 0, "vert count should not be zero");

    // Calculate how big a grid spatial map we can allocate
    const uint32_t maxBufferSize = m_allocator->LargestAllocatableSize(EA::Allocator::MEM_TEMP, 4);
    uint32_t maxInputs = GridSpatialMap::MaxNumInputs(maxBufferSize, 16u);

    // Limit the actual size to a reasonable maximum, so we don't allocate all available memory!
    if (maxInputs > maxInputLimit)
    {
        maxInputs = maxInputLimit;
    }

    // The resolution of the spatial map.
    uint32_t gridResolution = 16u;

    // Mark temporary heap before allocation of the grid spatial map
    m_allocator->Mark(EA::Allocator::MEM_TEMP);

    {
        // Create the GridSpatialMap
        GridSpatialMap spatialMap(m_allocator);

        // Attempt to initialize the spatial map.
        if (!spatialMap.Initialize(
            GridSpatialMap::VectorType(m_vertAABBox.Min()),
            GridSpatialMap::VectorType(m_vertAABBox.Max()),
            gridResolution,
            maxInputs))
        {
            // Memory allocation failed. Return with failure.
            EAPHYSICS_MESSAGE("FixUnmatchedEdges: Memory requirements not met for GridSpatialMap with entry count of %d.", maxInputs);
            m_isBuilderValid = false;
            return false;
        }

        ClusteredMeshBuilderMethods::FixUnmatchedEdges(
            spatialMap,
            *m_triangleGroupIDs,
            *m_triangleEdgeCosines,
            *m_triangleNeighbors,
            *m_vertices,
            *m_triangles,
            *m_triangleFlags,
            m_coplanarCosineTolerance,
            m_coplanarHeightTolerance,
            m_maximumEdgeCosineMergeTolerance);
    }

    // Release temporary heap after freeing of the grid spatial map
    m_allocator->Release(EA::Allocator::MEM_TEMP);

    return true;
}


/**
\brief Constructs the internal collection of units.
This has to be called before a call to getting all unit bounding boxes.

\param findQuads Flag indicating whether or not triangles should be merged into quads.
\param mergeQuads Unused.
\return Number of units constructed.
*/
uint32_t
ClusteredMeshBuilder::BuildUnitList(bool findQuads)
{
    EA_ASSERT_MSG(m_isBuilderValid, "Builder is in an invalid state - memory allocation has failed before this point");
    EA_ASSERT_MSG(m_vertices->size() != 0, "vert count should not be zero");
    EA_ASSERT_MSG(m_triangles->size() != 0, "m_triangles count should not be zero");
    EA_ASSERT_MSG(m_triangleEdgeCodes->size() != 0, "m_triangleEdgeCodes count should not be zero");
    EA_ASSERT_MSG(m_triangleEdgeCosines->size() != 0, "m_triangleEdgeCosines count should not be zero");
    EA_ASSERT_MSG(m_triangleNeighbors->size() != 0, "m_triangleNeighbors count should not be zero");

    m_unitList->reserve(m_numTriangles);

    if (findQuads)
    {
        // The compressedUnitIndex list is used to map triangle indices to unit indices.
        // For example: if triangles with indices A and B are merged to form a quad then
        // compressedUnitIndex[A] = compressedUnitIndex[B] = A (the indices merge to the
        // lower of the two)

        // Mark temporary heap before allocation of the ID list
        m_allocator->Mark(EA::Allocator::MEM_TEMP);

        // Allocate and initialize compressedUnitIndex list - using the scratch allocator
        IDList *compressedUnitIndex = IDList::Allocate(m_allocator, m_numTriangles, EA::Allocator::MEM_TEMP);
        if (!compressedUnitIndex)
        {
            m_isBuilderValid = FALSE;
            return 0;
        }

        compressedUnitIndex->resize(m_numTriangles);

        UnitListBuilder::BuildUnitListWithQuads(
            *m_unitList,
            *compressedUnitIndex,
            *m_triangles,
            *m_triangleSurfaceIDs,
            *m_triangleGroupIDs,
            *m_triangleNeighbors,
            *m_triangleFlags,
            *m_vertices,
            m_unitParameters.surfaceIDSize,
            m_unitParameters.groupIDSize);

        // The compressedUnitIndex list is now redundant so release any allocated memory
        IDList::Free(m_allocator, compressedUnitIndex);

        // Release temporary heap after freeing of ID list
        m_allocator->Release(EA::Allocator::MEM_TEMP);
    }
    else
    {
        UnitListBuilder::BuildUnitListWithTriangles(
            *m_unitList,
            *m_triangles,
            *m_triangleFlags);
    }

    // Set allocation for unitBBoxList - using the scratch allocator
    uint32_t sizeUnitAABBoxListBuffer = (m_unitList->size() * sizeof(AABBoxType)) + sizeof(rwpmath::Vector3);
    m_unitAABBoxListBuffer = reinterpret_cast<uint8_t*>(m_allocator->Alloc(sizeUnitAABBoxListBuffer, NULL, EA::Allocator::MEM_TEMP, 4u));
    if (NULL != m_unitAABBoxListBuffer)
    {
        m_unitAABBoxList = reinterpret_cast<AABBoxType*>(m_unitAABBoxListBuffer);
    }
    else
    {
        m_isBuilderValid = FALSE;
        return 0;
    }

    ClusteredMeshBuilderMethods::BuildUnitAABBoxesList(
        m_unitAABBoxList,
        *m_unitList,
        *m_triangles,
        *m_vertices);

    return m_unitList->size();
}


/**
\brief Returns a pointer to the internal collection of unit AABBoxes.

\return a pointer to the array of unit AABBoxes.
*/
ClusteredMeshBuilder::AABBoxType *
ClusteredMeshBuilder::GetAllUnitBBoxes() const
{
    return m_unitAABBoxList;
}


/**
\brief Walks the kdtree and creates clusters

For each branch node in the kdtree such that the branch contains between 1 and 256 unique vertices,
and the parent of the branch contains more than 256 unique vertices: create a cluster corresponding to
that branch.

\param kdtreeBuilder a kdtree builder object. The start index might be altered.
\return The number of clusters found, or zero on failure.
*/
uint32_t
ClusteredMeshBuilder::CreateClustersUsingKDTree(rw::collision::KDTreeBuilder& kdtreeBuilder)
{
    EA_ASSERT_MSG(m_isBuilderValid, "Builder is in an invalid state - memory allocation has failed before this point");
    EA_ASSERT_MSG(m_triangles->size() != 0, "m_triangles count should not be zero");
    EA_ASSERT_MSG(m_triangleEdgeCodes->size() != 0, "m_triangleEdgeCodes count should not be zero");
    EA_ASSERT_MSG(m_unitList->size(), "m_unitList count should not be zero");

    // m_unitAABBoxList is now redundant so release the m_unitAABBoxList buffer
    m_allocator->Free(m_unitAABBoxListBuffer);
    m_unitAABBoxList = NULL;
    m_unitAABBoxListBuffer = NULL;

    // Allocate the leafMap, implicitly  using the temporary heap
    const uint32_t numLeafNodes = kdtreeBuilder.GetNumNodes() - kdtreeBuilder.GetNumBranchNodes();

    LeafMap leafMap;
    if (!leafMap.get_allocator().Initialize(
        numLeafNodes,
        sizeof(LeafMap::node_type),
        m_allocator))
    {
        // Indicate a failure has occurred
        m_isBuilderValid = FALSE;
        return 0;
    }

    // Initialize the UnitClusterStack, implicitly using the temporary heap
    m_unitClusterStack.Initialize(m_allocator, m_numTriangles);
    if (m_unitClusterStack.IsValid())
    {
        uint32_t mergedVertices[ClusteredMeshCluster::MAX_VERTEX_COUNT];

        uint32_t failureFlags = ClusteredMeshBuilderMethods::CLUSTER_GENERATION_FAILURE_NO_FAILURES;

        // Initialize the UnitClusters
        ClusteredMeshBuilderMethods::InitializeUnitClustersUsingKDTree(
            leafMap,
            m_unitClusterStack,
            *m_triangles,
            mergedVertices,
            failureFlags,
            *m_unitList,
            *m_vertices,
            kdtreeBuilder);

        // If no failures occurred during cluster generation
        if (ClusteredMeshBuilderMethods::CLUSTER_GENERATION_FAILURE_NO_FAILURES == failureFlags)
        {
            // Finalize the Unit Clusters.
            const uint32_t unitClusterCount = m_unitClusterStack.Size();
            const uint32_t unitClusterIDShift = (unitClusterCount > 65536) ? 20U : 16U;

            UnitClusterStack::ClusterIterator it = m_unitClusterStack.Begin();
            const UnitClusterStack::ClusterIterator itEnd = m_unitClusterStack.End();

            while (it != itEnd)
            {
                UnitCluster * unitCluster = *it;

                ClusteredMeshBuilderMethods::AdjustKDTreeNodeEntriesForCluster(
                    *unitCluster,
                    leafMap,
                    *m_unitList,
                    *m_triangleSurfaceIDs,
                    *m_triangleGroupIDs,
                    m_unitParameters,
                    unitCluster->clusterID,
                    unitClusterIDShift);

                ++it;
            }
        }
        else // failures occurred during the cluster generation process
        {
            // Indicate a failure has occurred
            m_isBuilderValid = FALSE;

            // Indicate the failing case
            if (ClusteredMeshBuilderMethods::CLUSTER_GENERATION_FAILURE_OUT_OF_MEMORY & failureFlags)
            {
                EAPHYSICS_MESSAGE("While generating clusters the UnitClusterStack ran out of memory resource.");
            }
            if (ClusteredMeshBuilderMethods::CLUSTER_GENERATION_FAILURE_MULTI_LEAF_CLUSTER & failureFlags)
            {
                EAPHYSICS_MESSAGE("A KDTree leaf node encompasses more vertices than a single cluster can contain.");
            }
        }
    }
    else // UnitClusterStack has failed initialization
    {
        // Indicate a failure has occurred
        m_isBuilderValid = FALSE;
    }

    leafMap.clear();
    // The leaf map is no longer required so we can deallocate its resources.
    leafMap.get_allocator().Release();

    if (TRUE == m_isBuilderValid)
    {
        return m_unitClusterStack.Size();
    }
    else
    {
        return 0;
    }
}


/**
\brief Check if this cluster's vertices fit into 16 bits given the granularity, and mark the cluster
appropriately.

\param vertexCompressionOn Flag indicating whether or not vertex compression can take place.
\param clusterid ID of cluster.
*/
void
ClusteredMeshBuilder::DetermineClusterCompressionMode(bool vertexCompressionOn,
                                                      UnitCluster &unitCluster) const
{
    EA_ASSERT_MSG(m_isBuilderValid, "Builder is in an invalid state - memory allocation has failed before this point");

    if ( vertexCompressionOn )
    {
        int32_t x32min = 0, y32min = 0, z32min = 0, x32max = 0, y32max = 0, z32max = 0;

        // let's find the cluster's extents when converted into integer space
        bool firstRun = true;
        for (uint32_t i = 0 ; i < unitCluster.numVertices ; ++i)
        {
            const VectorType &v = (*m_vertices)[unitCluster.vertexIDs[i]];
            int32_t x32 = (int32_t)( v.GetX() / m_vertexCompressionGranularity );
            int32_t y32 = (int32_t)( v.GetY() / m_vertexCompressionGranularity );
            int32_t z32 = (int32_t)( v.GetZ() / m_vertexCompressionGranularity );

            if ( firstRun || ( x32 < x32min ) ) x32min = x32;
            if ( firstRun || ( x32 > x32max ) ) x32max = x32;

            if ( firstRun || ( y32 < y32min ) ) y32min = y32;
            if ( firstRun || ( y32 > y32max ) ) y32max = y32;

            if ( firstRun || ( z32 < z32min ) ) z32min = z32;
            if ( firstRun || ( z32 > z32max ) ) z32max = z32;

            firstRun = false;
        }

        VertexCompression::DetermineCompressionModeAndOffsetForRange(
            unitCluster.compressionMode,
            unitCluster.clusterOffset,
            x32min, x32max,
            y32min, y32max,
            z32min, z32max);
    }
    else
    {
        unitCluster.compressionMode = rw::collision::ClusteredMeshCluster::VERTICES_UNCOMPRESSED;
    }
}


/**
\brief Determine the granularity needed for this cluster's vertices to fit into 16 bits.

\param clusterid ID of cluster.
\return Granularity needed to store cluster vertices in 16bits.
*/
float
ClusteredMeshBuilder::CalculateMinimumGranularityForCluster( const UnitCluster & unitCluster ) const
{
    EA_ASSERT_MSG(m_isBuilderValid, "Builder is in an invalid state - memory allocation has failed before this point");

    float x32min = 0, y32min = 0, z32min = 0, x32max = 0, y32max = 0, z32max = 0;

    // let's find the cluster's extents when converted into integer space
    bool firstRun = true;
    for (uint32_t i = 0 ; i < unitCluster.numVertices; ++i)
    {
        const VectorType &v = (*m_vertices)[unitCluster.vertexIDs[i]];
        float x32 = v.GetX();
        float y32 = v.GetY();
        float z32 = v.GetZ();

        if ( firstRun || ( x32 < x32min ) ) x32min = x32;
        if ( firstRun || ( x32 > x32max ) ) x32max = x32;

        if ( firstRun || ( y32 < y32min ) ) y32min = y32;
        if ( firstRun || ( y32 > y32max ) ) y32max = y32;

        if ( firstRun || ( z32 < z32min ) ) z32min = z32;
        if ( firstRun || ( z32 > z32max ) ) z32max = z32;

        firstRun = false;
    }

    return VertexCompression::CalculateMinimum16BitGranularityForRange(
        x32min, x32max,
        y32min, y32max,
        z32min, z32max);
}


/**
\brief Returns the number of bytes in the specified cluster, including vertices, etc.

\param clusterid ID of cluster.

\return The number of bytes in the specified cluster, including vertices, etc.
*/
uint16_t
ClusteredMeshBuilder::GetClusterSize(const UnitCluster & unitCluster) const
{
    EA_ASSERT_MSG(m_isBuilderValid, "Builder is in an invalid state - memory allocation has failed before this point");

    ClusterConstructionParameters parameters;
    detail::ClusterParametersBuilder::InitializeClusterParameters(
        parameters,
        unitCluster,
        *m_triangleSurfaceIDs,
        *m_triangleGroupIDs,
        *m_unitList,
        m_unitParameters);

    return rw::collision::ClusteredMeshCluster::GetSize(parameters);
}


UnitClusterStack&
ClusteredMeshBuilder::GetUnitClusterStack()
{
    EA_ASSERT_MSG(m_isBuilderValid, "Builder is in an invalid state - memory allocation has failed before this point");
    return m_unitClusterStack;
}


/**
\brief Determine the total storage size of all the clusters.

This is merely the sum of GetClusterSize for all clusters.

\return The total size in bytes of all cluster data including vertices, etc.
*/
uint32_t
ClusteredMeshBuilder::GetClusterTotalSize() const
{
    EA_ASSERT_MSG(m_isBuilderValid, "Builder is in an invalid state - memory allocation has failed before this point");

    uint32_t size = 0;

    UnitClusterStack::ClusterIterator it = m_unitClusterStack.Begin();
    const UnitClusterStack::ClusterIterator itEnd = m_unitClusterStack.End();

    while (it != itEnd)
    {
        // Get the UnitCluster
        const UnitCluster * const unitCluster = *it;

        size += GetClusterSize(*unitCluster);

        size = EA::Physics::SizeAlign<uint32_t>(size, rwcCLUSTEREDMESHCLUSTER_ALIGNMENT);

        ++it;
    }

    EAPHYSICS_MESSAGE("Total memory for clusters %d", size);

    return size;
}


/**
\brief Creates a ClusteredMeshCluster.

The size of the memory must match GetClusterSize, and the alignment must be rwcCLUSTEREDMESHCLUSTER_ALIGNMENT.

\param cluster A memory block into which the cluster is created
\param clusterid ID of cluster
\return The cluster data for specified cluster.
*/
void
ClusteredMeshBuilder::InitializeCluster(rw::collision::ClusteredMeshCluster *cluster,
                                        const UnitCluster & unitCluster) const
{
    EA_ASSERT_MSG(m_isBuilderValid, "Builder is in an invalid state - memory allocation has failed before this point");
    EA_ASSERT_MSG(m_vertices->size() != 0, "vert count should not be zero");
    EA_ASSERT_MSG(NULL != cluster, "cluster should not be null");
    rwcASSERTALIGN(cluster, rwcCLUSTEREDMESHCLUSTER_ALIGNMENT);

    EA_ASSERT(unitCluster.numVertices <= ClusteredMeshCluster::MAX_VERTEX_COUNT);
    EA_ASSERT(unitCluster.numUnits < 65535);

    detail::ClusterDataBuilder::Build(
        *cluster,
        unitCluster,
        *m_vertices,
        *m_triangles,
        *m_triangleEdgeCodes,
        *m_triangleSurfaceIDs,
        *m_triangleGroupIDs,
        *m_unitList,
        m_unitParameters,
        m_vertexCompressionGranularity);
}


/**
\brief Initialize a ClusteredMeshCluster construction parameters structure.

\param parameters ClusteredMeshCluster construction parameters.
\param unitCluster UnitCluster used to initialize parameters.
*/
void
ClusteredMeshBuilder::InitializeClusterConstructionParameters(
    ClusterConstructionParameters & parameters,
    const UnitCluster &unitCluster)
{
    detail::ClusterParametersBuilder::InitializeClusterParameters(
        parameters,
        unitCluster,
        *m_triangleSurfaceIDs,
        *m_triangleGroupIDs,
        *m_unitList,
        m_unitParameters);
}


} // namespace detail
} // namespace meshbuilder
} // namespace collision
} // namespace rw
