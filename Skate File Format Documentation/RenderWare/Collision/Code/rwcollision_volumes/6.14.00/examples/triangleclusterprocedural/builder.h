// (c) Electronic Arts. All Rights Reserved.
#ifndef RWCOLLISION_VOLUMES_EXAMPLES_TRIANGLECLUSTERPROCEDURAL_EXAMPLEBUILDER_H
#define RWCOLLISION_VOLUMES_EXAMPLES_TRIANGLECLUSTERPROCEDURAL_EXAMPLEBUILDER_H


#include <rw/collision/common.h>


#if !defined EA_PLATFORM_PS3_SPU

#include <coreallocator/icoreallocator_interface.h>

#include <rw/collision/common.h>
#include <rw/collision/clusteredmeshcluster.h>
#include <rw/collision/triangleclusterprocedural.h>

#include <rw/collision/meshbuilder/common.h>
#include <rw/collision/meshbuilder/edgecodegenerator.h>
#include <rw/collision/meshbuilder/unitlistbuilder.h>
#include <rw/collision/meshbuilder/triangleclusterproceduralbuilder.h>


/**
This example builder uses a subset of ClusteredMeshBuilder functionality to build a
TriangleClusterProcedural. Given a collection of vertices, triangles and connectivity
information the builder generates a TriangleClusterProcedural.

Internally the builder is based on the simpler TriangleClusterProceduralBuilder tool
provided by the public API of this package. It extends that "backend" builder with
"frontend" functionality to build unit data from a set of triangle input data.

The input vertices are simply a list of vertices.
The input triangle information consists of vertex indices, which index into the vertex collection,
and enhanced edge cosine values for each triangle edge.
The input connectivity information is formed of a collection of neighboring triangle
indices for each of the input triangles.

In this example the builder is provided with triangle connectivity information directly,
on the assumption that it can be generated procedurally or read from some user data
structure owned by the caller. If the input is a triangle soup and triangle connectivity
needs to be computed automatically then the builder could easily be extended with this
step - see the rw::collision::meshbuilder::TriangleConnector utility for more info.

The steps implemented by this builder are as follows:

Step A - Initialize the triangle input information
Step B - Create a list of Units
Step C - Use the TriangleClusterProceduralBuilder to build a TriangleClusterProcedural

Internally the TriangleClusterProceduralBuilder takes the following actions to build
the TriangleClusterProcedural:

Step C - Create a UnitCluster
Step D - Determine the vertex compression mode
Step E - Initialize the ClusterConstructionParameters
Step F - Initialize the TriangleClusterProcedural using the ClusterConstructionParameters
Step G - Get the owned ClusteredMeshCluster from the TriangleClusterProcedural and fill it with data from the unit cluster.

Through the use of a BuilderParameters struct accepted by the builder, the user can
control the following features:

Quad creation -
    The builder can attempt to convert adjacent triangles into quads. Quads are pairs
    of adjacent triangles which serve to reduce the memory requirements of the
    TriangleClusterProcedural. This functionality is is achieved through the use of the
    UnitListBuilder::BuildUnitListWithQuads method.

Vertex Compression -
    The builder can attempt to compress the vertices, given a compression granularity.
    This, as with the quad creation, serves to reduce the memory requirements of the
    TriangleClusterProcedural. It is possible to determine the minimum compression
    granularity required to compress the vertices through use of the
    VertexCompression::CalculateMinimum16BitGranularityForRange method.

The following list highlights features of the ClusteredMeshBuilder which could be added
to the builder:

Vertex Merging - 
    Merging, and sharing, vertices which lie within a distance tolerance of each other

Triangle Removal -
    Flagging triangles for removal from the final collection. The standard use of this
    functionality can be found in the ClusteredMeshBuilderMethods::InternalTriangleRemoval.

Merging Triangle with Planes -
    Adjusting the edge cosines of triangles which lay in a plane allowing smooth rolling transition
    from the plan to the triangles.

Fixing unshared edge cosines -
    Adjusting the edge cosines of triangle edges which intersect other triangles. As with plane
    merging, this allows smooth rolling transition between the two triangles.

Vertex Smoothing -
    Determines which vertices are not 'feature vertices' and disables them accordingly.
*/
class Builder
{

public:

    typedef rw::collision::meshbuilder::VectorType VectorType;

    /*
    \brief Parameters used to control the build process.
    **/
    struct BuildParameters
    {
        BuildParameters() :
          buildQuads(true),
          compressVertices(true),
          vertexCompressionGranularity(1.0f),
          edgeCosineConcaveAngleTolerance(0.0f)
        {
            unitParameters.unitFlagsDefault = 0u;
            unitParameters.groupIDSize = 0u;
            unitParameters.surfaceIDSize = 0u;
        }

        /// ClusteredMeshCluster parameters
        rw::collision::UnitParameters unitParameters;
        /// Flag controlling quad generation
        bool buildQuads;
        /// Flag controlling vertex compression
        bool compressVertices;
        /// Vertex compression granularity
        float vertexCompressionGranularity;
        /// Tolerance controlling the range of angles which can be considered concave
        rwpmath::VecFloat edgeCosineConcaveAngleTolerance;
    };

    /**
    \brief Constructs a builder with the given expected data sizes and build parameters.

    \param vertexCount                          The expected number of unique vertices in the mesh data.
    \param triangleCount                        The expected number of triangles in the mesh data.
    \param buildParams                          Build parameters controlling the build process.
    \param triangleClusterProceduralAllocator   An allocator used to allocate the TriangleClusterProcedural.
    \param workspaceAllocator                   An allocator used for all internal workspace allocations required during the build process.
    */
    Builder(
        const uint32_t vertexCount,
        const uint32_t triangleCount,
        const BuildParameters & buildParams,
        EA::Allocator::ICoreAllocator *triangleClusterProceduralAllocator,
        EA::Allocator::ICoreAllocator *workspaceAllocator);

    /**
    \brief Class Destructor.
    */
    ~Builder();

    /**
    \brief Adds a mesh vertex to the builder.
    */
    void AddVertex(const VectorType & vertex);

    /**
    \brief Adds a mesh triangle to the builder, indexing into the list of added vertices.

    \note Pass CLUSTEREDMESHBUILDER_TRIANGLENEIGHBORINDEX_NOMATCH for a neighbor
    triangle index to imply "no neighbor" on that edge.

    \param v0           Index of first triangle vertex (base zero)
    \param v1           Index of second triangle vertex (base zero)
    \param v2           Index of third triangle vertex (base zero)
    \param edgeCosine0  Edge cosine of triangle edge v0v1
    \param edgeCosine1  Edge cosine of triangle edge v1v2
    \param edgeCosine2  Edge cosine of triangle edge v2v0
    \param neighbor0    Index of neighboring triangle on edge v0v1
    \param neighbor1    Index of neighboring triangle on edge v1v2
    \param neighbor2    Index of neighboring triangle on edge v2v0
    \param groupID      Optional integer group ID of this triangle, identifying submeshes.
    \param surfaceID    Optional integer surface ID of this triangle, identifying collision surfaces.
    */
    void AddTriangle(
        uint32_t v0,
        uint32_t v1,
        uint32_t v2,
        float edgeCosine0,
        float edgeCosine1,
        float edgeCosine2,
        const uint32_t neighbor0,
        const uint32_t neighbor1,
        const uint32_t neighbor2,
        const bool edge0Matched,
        const bool edge1Matched,
        const bool edge2Matched,
        uint32_t groupID = 0u,
        uint32_t surfaceID = 0u);


    /**
    \brief Adds a mesh quad to the builder, indexing into the list of added vertices.

    \note Pass CLUSTEREDMESHBUILDER_TRIANGLENEIGHBORINDEX_NOMATCH for a neighbor
    triangle index to imply "no neighbor" on that edge.

    \param v0           Index of first quad vertex (base zero)
    \param v1           Index of second quad vertex (base zero)
    \param v2           Index of third quad vertex (base zero)
    \param v3           Index of fourth quad vertex (base zero)
    \param edgeCosine0  Edge cosine of quad edge v0v1
    \param edgeCosine1  Edge cosine of quad edge v1v2
    \param edgeCosine2  Edge cosine of quad edge v2v3
    \param edgeCosine3  Edge cosine of quad egde v3v0
    \param neighbor0    Index of neighboring triangle on edge v0v1
    \param neighbor1    Index of neighboring triangle on edge v1v2
    \param neighbor2    Index of neighboring triangle on edge v2v3
    \param neighbor3    Index of neighboring triangle on edge v3v0
    \param groupID      Optional integer group ID of this triangle, identifying submeshes.
    \param surfaceID    Optional integer surface ID of this triangle, identifying collision surfaces.
    */
    void AddQuad(
        uint32_t v0,
        uint32_t v1,
        uint32_t v2,
        uint32_t v3,
        float edgeCosine0,
        float edgeCosine1,
        float edgeCosine2,
        float edgeCosine3,
        const uint32_t neighbor0,
        const uint32_t neighbor1,
        const uint32_t neighbor2,
        const uint32_t neighbor3,
        const bool edge0Matched,
        const bool edge1Matched,
        const bool edge2Matched,
        const bool edge3Matched,
        uint32_t groupID = 0u,
        uint32_t surfaceID = 0u);

    /**
    \brief Builds a TriangleClusterProcedural from data input earlier via \ref AddVertex and \ref AddTriangle.
    */
    rw::collision::TriangleClusterProcedural *Build();

private:

    typedef rw::collision::meshbuilder::TriangleClusterProceduralBuilder::VertexList VertexList;
    typedef rw::collision::meshbuilder::TriangleClusterProceduralBuilder::TriangleList TriangleList;
    typedef rw::collision::meshbuilder::TriangleClusterProceduralBuilder::TriangleSurfaceIDList TriangleSurfaceIDList;
    typedef rw::collision::meshbuilder::TriangleClusterProceduralBuilder::TriangleGroupIDList TriangleGroupIDList;
    typedef rw::collision::meshbuilder::TriangleClusterProceduralBuilder::TriangleEdgeCodesList TriangleEdgeCodesList;
    typedef rw::collision::meshbuilder::TriangleClusterProceduralBuilder::UnitList UnitList;

    typedef rw::collision::meshbuilder::UnitListBuilder::TriangleFlagsList TriangleFlagsList;
    typedef rw::collision::meshbuilder::UnitListBuilder::TriangleNeighborsList TriangleNeighborsList;

    /**
    \internal Creates a collection units, required for UnitCluster generation
    later in the build process.
    */
    void CreateUnits();

    /**
    \internal Deallocates all internal permanent resources.
    Called automatically on destruction.
    */
    void Release();

    BuildParameters m_buildParameters;

    VertexList *m_vertices;
    TriangleList *m_triangles;
    TriangleSurfaceIDList *m_triangleSurfaceIDs;
    TriangleGroupIDList *m_triangleGroupIDs;
    TriangleEdgeCodesList *m_triangleEdgeCodes;
    TriangleNeighborsList *m_triangleNeighbors;
    UnitList *m_units;

    EA::Allocator::ICoreAllocator * m_triangleClusterProceduralAllocator;
    EA::Allocator::ICoreAllocator * m_workspaceAllocator;
};


inline Builder::Builder(
    const uint32_t vertexCount,
    const uint32_t triangleCount,
    const BuildParameters & buildParams,
    EA::Allocator::ICoreAllocator *triangleClusterProceduralAllocator,
    EA::Allocator::ICoreAllocator *workspaceAllocator)
    : m_buildParameters(buildParams)
    , m_vertices(NULL)
    , m_triangles(NULL)
    , m_triangleSurfaceIDs(NULL)
    , m_triangleGroupIDs(NULL)
    , m_triangleEdgeCodes(NULL)
    , m_triangleNeighbors(NULL)
    , m_units(NULL)
    , m_triangleClusterProceduralAllocator(triangleClusterProceduralAllocator)
    , m_workspaceAllocator(workspaceAllocator)
{
    // Create the permanent containers
    m_vertices = VertexList::Allocate(m_workspaceAllocator, vertexCount, EA::Allocator::MEM_TEMP);
    m_triangles = TriangleList::Allocate(m_workspaceAllocator, triangleCount, EA::Allocator::MEM_TEMP);
    m_triangleSurfaceIDs = TriangleSurfaceIDList::Allocate(m_workspaceAllocator, triangleCount, EA::Allocator::MEM_TEMP);
    m_triangleGroupIDs = TriangleGroupIDList::Allocate(m_workspaceAllocator, triangleCount, EA::Allocator::MEM_TEMP);
    m_triangleEdgeCodes = TriangleEdgeCodesList::Allocate(m_workspaceAllocator, triangleCount, EA::Allocator::MEM_TEMP);
    m_triangleNeighbors = TriangleNeighborsList::Allocate(m_workspaceAllocator, triangleCount, EA::Allocator::MEM_TEMP);
    m_units = UnitList::Allocate(m_workspaceAllocator, triangleCount, EA::Allocator::MEM_TEMP);

    m_vertices->reserve(vertexCount);
    m_triangles->reserve(triangleCount);
    m_triangleSurfaceIDs->reserve(triangleCount);
    m_triangleGroupIDs->reserve(triangleCount);
    m_triangleEdgeCodes->reserve(triangleCount);
    m_triangleNeighbors->reserve(triangleCount);
    m_units->reserve(triangleCount);
}


inline Builder::~Builder()
{
    Release();
}


inline void
Builder::Release()
{
    UnitList::Free(m_workspaceAllocator, m_units);
    TriangleNeighborsList::Free(m_workspaceAllocator, m_triangleNeighbors);
    TriangleEdgeCodesList::Free(m_workspaceAllocator, m_triangleEdgeCodes);
    TriangleGroupIDList::Free(m_workspaceAllocator, m_triangleGroupIDs);
    TriangleSurfaceIDList::Free(m_workspaceAllocator, m_triangleSurfaceIDs);
    TriangleList::Free(m_workspaceAllocator, m_triangles);
    VertexList::Free(m_workspaceAllocator, m_vertices);
}


inline void
Builder::AddVertex(const VectorType & vertex)
{
    m_vertices->push_back(vertex);
}


inline void
Builder::AddTriangle(
    uint32_t v0,
    uint32_t v1,
    uint32_t v2,
    float edgeCosine0,
    float edgeCosine1,
    float edgeCosine2,
    const uint32_t neighbor0,
    const uint32_t neighbor1,
    const uint32_t neighbor2,
    const bool edge0Matched,
    const bool edge1Matched,
    const bool edge2Matched,
    uint32_t groupID /* = 0u */,
    uint32_t surfaceID /* = 0u */)
{
    // Push the triangle vertex indices
    TriangleList::value_type triangle;
    triangle.vertices[0] = v0;
    triangle.vertices[1] = v1;
    triangle.vertices[2] = v2;
    m_triangles->push_back(triangle);

    // Push the triangle neighbor indices
    TriangleNeighborsList::value_type neighbors;
    neighbors.neighbor[0] = neighbor0;
    neighbors.neighbor[1] = neighbor1;
    neighbors.neighbor[2] = neighbor2;
    m_triangleNeighbors->push_back(neighbors);

    // Encode and push the edge cosines
    TriangleEdgeCodesList::value_type edgeCodes;

    edgeCodes.encodedEdgeCos[0] = rw::collision::meshbuilder::EdgeCodeGenerator::GenerateEdgeCode(
                                     edgeCosine0,
                                     m_buildParameters.edgeCosineConcaveAngleTolerance,
                                     edge0Matched);

    edgeCodes.encodedEdgeCos[1] = rw::collision::meshbuilder::EdgeCodeGenerator::GenerateEdgeCode(
                                     edgeCosine1,
                                     m_buildParameters.edgeCosineConcaveAngleTolerance,
                                     edge1Matched);

    edgeCodes.encodedEdgeCos[2] = rw::collision::meshbuilder::EdgeCodeGenerator::GenerateEdgeCode(
                                     edgeCosine2,
                                     m_buildParameters.edgeCosineConcaveAngleTolerance,
                                     edge2Matched);

    m_triangleEdgeCodes->push_back(edgeCodes);

    // Push the triangle surface and group IDs
    m_triangleSurfaceIDs->push_back(surfaceID);
    m_triangleGroupIDs->push_back(groupID);
}


inline void
Builder::AddQuad(
    uint32_t v0,
    uint32_t v1,
    uint32_t v2,
    uint32_t v3,
    float edgeCosine0,
    float edgeCosine1,
    float edgeCosine2,
    float edgeCosine3,
    const uint32_t neighbor0,
    const uint32_t neighbor1,
    const uint32_t neighbor2,
    const uint32_t neighbor3,
    const bool edge0Matched,
    const bool edge1Matched,
    const bool edge2Matched,
    const bool edge3Matched,
    uint32_t groupID /* = 0u */,
    uint32_t surfaceID /* = 0u */)
{
    const uint32_t triangleCount = m_triangles->size();

    // As the quad is fed to the builder as two triangles we need to specify
    // an edge cosine value for the shared edge. However, as the triangles are
    // later merged into quads this value is discarded and therefore its value
    // is unimportant.
    const float shared_edge_cosine = 0.0f;

    // Push the first triangle of the quad
    AddTriangle(
        v0,
        v1,
        v3,
        edgeCosine0,
        shared_edge_cosine,
        edgeCosine3,
        neighbor0,
        triangleCount + 1,
        neighbor3,
        edge0Matched,
        true,
        edge3Matched,
        groupID,
        surfaceID);

    // Push the second triangle of the quad
    AddTriangle(
        v3,
        v1,
        v2,
        shared_edge_cosine,
        edgeCosine1,
        edgeCosine2,
        triangleCount,
        neighbor1,
        neighbor2,
        true,
        edge1Matched,
        edge2Matched,
        groupID,
        surfaceID);
}


inline rw::collision::TriangleClusterProcedural *
Builder::Build()
{
    // Create the units
    CreateUnits();

    rw::collision::meshbuilder::TriangleClusterProceduralBuilder::BuildParameters params;
    params.compressVertices = m_buildParameters.compressVertices;
    params.unitParameters = m_buildParameters.unitParameters;
    params.vertexCompressionGranularity = m_buildParameters.vertexCompressionGranularity;

    // Use the "back-end" builder to build the TriangleClusterProcedural with the given input.
    return rw::collision::meshbuilder::TriangleClusterProceduralBuilder::Build(
        *m_triangleClusterProceduralAllocator,
        *m_workspaceAllocator,
        params,
        *m_vertices,
        *m_triangles,
        *m_units,
        *m_triangleEdgeCodes,
        *m_triangleSurfaceIDs,
        *m_triangleGroupIDs);
}


inline void
Builder::CreateUnits()
{
    // Allocate triangle flag resources using the temp allocator
    TriangleFlagsList *triangleFlags = TriangleFlagsList::Allocate(m_workspaceAllocator, m_triangles->size(), EA::Allocator::MEM_TEMP);
    triangleFlags->resize(m_triangles->size());

    if (m_buildParameters.buildQuads)
    {
        typedef rw::collision::meshbuilder::UnitListBuilder::IDList IDList;

        // Create an ID collection which is used during quad unit generation
        IDList * compressedUnitIndex = IDList::Allocate(m_workspaceAllocator, m_triangles->size(), EA::Allocator::MEM_TEMP);
        compressedUnitIndex->resize(m_triangles->size());

        rw::collision::meshbuilder::UnitListBuilder::BuildUnitListWithQuads(
            *m_units,
            *compressedUnitIndex,
            *m_triangles,
            *m_triangleSurfaceIDs,
            *m_triangleGroupIDs,
            *m_triangleNeighbors,
            *triangleFlags,
            *m_vertices,
            m_buildParameters.unitParameters.surfaceIDSize,
            m_buildParameters.unitParameters.groupIDSize);

        IDList::Free(m_workspaceAllocator, compressedUnitIndex);
    }
    else
    {
        rw::collision::meshbuilder::UnitListBuilder::BuildUnitListWithTriangles(
            *m_units,
            *m_triangles,
            *triangleFlags);
    }

    // Release the triangle flags collection resource
    TriangleFlagsList::Free(m_workspaceAllocator, triangleFlags);
}


#endif // #if !defined EA_PLATFORM_PS3_SPU


#endif // RWCOLLISION_VOLUMES_EXAMPLES_TRIANGLECLUSTERPROCEDURAL_EXAMPLEBUILDER_H
