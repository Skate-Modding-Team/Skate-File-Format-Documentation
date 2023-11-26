// (c) Electronic Arts. All Rights Reserved.


#include <rw/collision/common.h>


#if !defined EA_PLATFORM_PS3_SPU

#include <stdio.h>

#include <rw/collision/triangleclusterprocedural.h>

#include <rw/collision/meshbuilder/common.h>

#include "heightfield.h"
#include "builder.h"

#include <common/common.h>

#include "EAMain/EAEntryPointMain.inl" // For EAMain


/*
This code example demonstrates how to write and use a simple builder to build a
TriangleClusterProcedural aggregate volume.

The example builder is provided in builder.h. It is built around the
rw::collision::meshbuilder::TriangleClusterProceduralBuilder tool provided as part of the
public API of this package. The TriangleClusterProceduralBuilder allows an aggregate
volume wrapping a single cluster to be built from intermediatory triangle data built
using other API functions. The custom builder in this example adds frontend functionality
implemented using those API functions to produce an easier-to-use builder tailored
for building meshes with particular requirements.

The example builder class wraps a subset of the ClusteredMeshBuilder functionality
to implement a simple builder. The builder contains the minimum number of steps required
to build a TriangleClusterProcedural, namely building triangle data, computing edge
cosines, and building unit data from the triangles. It could easily be expanded to
include further ClusteredMeshBuilder functionality such as vertex merging, triangle
connecitivity matching, vertex compression, plane merging, etc.

The remainder of the code in this example consists of using the custom builder
to produce a TriangleClusterProcedural aggregate volume. This code includes the following
steps:

Step A - Create the builder.
Step B - Add the input triangle and vertex data to the builder.
Step C - Call the 'build' method.

The builder process is broken down into the following steps:

Step A - Initialize the triangle input information
Step B - Create a list of Units
Step C - Use the TriangleClusterProceduralBuilder to build a TriangleClusterProcedural
*/


/**
Adds a single HeightField vertex to a builder.
*/
static void AddHeightFieldVertexToBuilder(
    Builder &builder,
    const HeightField & heightfield,
    const uint32_t xIndex,
    const uint32_t zIndex)
{
    rwpmath::Vector3 vec = rwpmath::GetVector3_Zero();

    heightfield.GetVertex(
        vec,
        xIndex,
        zIndex);

    const meshbuilder::VectorType v(
        static_cast<float>(vec.GetX()),
        static_cast<float>(vec.GetY()),
        static_cast<float>(vec.GetZ()));

    builder.AddVertex(v);
}


/**
Adds a single HeightField triangle to a builder.
*/
static void AddHeightFieldTriangleToBuilder(
    Builder &builder,
    const HeightField & heightfield,
    const uint32_t xIndex,
    const uint32_t zIndex,
    const uint32_t trianglePairIndex)
{
    uint32_t v0 = 0;
    uint32_t v1 = 0;
    uint32_t v2 = 0;

    heightfield.GetTriangleVertexIndices(
        v0,
        v1,
        v2,
        xIndex,
        zIndex,
        trianglePairIndex);

    uint32_t neighbor0 = 0;
    uint32_t neighbor1 = 0;
    uint32_t neighbor2 = 0;

    heightfield.GetTriangleNeighborIndices(
        neighbor0,
        neighbor1,
        neighbor2,
        xIndex,
        zIndex,
        trianglePairIndex);

    rwpmath::VecFloat extendedEdgeCos0;
    rwpmath::VecFloat extendedEdgeCos1;
    rwpmath::VecFloat extendedEdgeCos2;

    heightfield.GetTriangleExtendedEdgeCosines(
        extendedEdgeCos0,
        extendedEdgeCos1,
        extendedEdgeCos2,
        xIndex,
        zIndex,
        trianglePairIndex);

    const bool edge0Matched = (neighbor0 == HeightField::NO_TRIANGLE_NEIGHBOR);
    const bool edge1Matched = (neighbor1 == HeightField::NO_TRIANGLE_NEIGHBOR);
    const bool edge2Matched = (neighbor2 == HeightField::NO_TRIANGLE_NEIGHBOR);

    builder.AddTriangle(
        v0,
        v1,
        v2,
        extendedEdgeCos0,
        extendedEdgeCos1,
        extendedEdgeCos2,
        neighbor0,
        neighbor1,
        neighbor2,
        edge0Matched,
        edge1Matched,
        edge2Matched);
}


/**
Adds a single HeightField quad to a builder.
*/
static void AddHeighFieldQuadToBuilder(
    Builder &builder,
    const HeightField & heightfield,
    const uint32_t xIndex,
    const uint32_t zIndex)
{
    uint32_t v0 = 0;
    uint32_t v1 = 0;
    uint32_t v2 = 0;
    uint32_t v3 = 0;

    heightfield.GetQuadVertexIndices(
        v0,
        v1,
        v2,
        v3,
        xIndex,
        zIndex);

    uint32_t neighbor0 = 0;
    uint32_t neighbor1 = 0;
    uint32_t neighbor2 = 0;
    uint32_t neighbor3 = 0;

    heightfield.GetQuadNeighborIndices(
        neighbor0,
        neighbor1,
        neighbor2,
        neighbor3,
        xIndex,
        zIndex);

    rwpmath::VecFloat extendedEdgeCos0;
    rwpmath::VecFloat extendedEdgeCos1;
    rwpmath::VecFloat extendedEdgeCos2;
    rwpmath::VecFloat extendedEdgeCos3;

    heightfield.GetQuadExtendedEdgeCosines(
        extendedEdgeCos0,
        extendedEdgeCos1,
        extendedEdgeCos2,
        extendedEdgeCos3,
        xIndex,
        zIndex);

    const bool edge0Matched = (neighbor0 == HeightField::NO_TRIANGLE_NEIGHBOR);
    const bool edge1Matched = (neighbor1 == HeightField::NO_TRIANGLE_NEIGHBOR);
    const bool edge2Matched = (neighbor2 == HeightField::NO_TRIANGLE_NEIGHBOR);
    const bool edge3Matched = (neighbor2 == HeightField::NO_TRIANGLE_NEIGHBOR);

    builder.AddQuad(
        v0,
        v1,
        v2,
        v3,
        extendedEdgeCos0,
        extendedEdgeCos1,
        extendedEdgeCos2,
        extendedEdgeCos3,
        neighbor0,
        neighbor1,
        neighbor2,
        neighbor3,
        edge0Matched,
        edge1Matched,
        edge2Matched,
        edge3Matched);
};

/**
Creates a triangle soup procedurally while adding the data to the builder.
The input vertices are shared.
*/
static void FeedBuilder(
    Builder &builder,
    const HeightField &heightfield,
    bool inputQuads)
{
    typedef rw::collision::meshbuilder::VectorType VectorType;

    // Extract the vertices from the heightfield and add them to the builder
    for (uint32_t zIndex = 0 ; zIndex < heightfield.GetZCount() ; ++zIndex)
    {
        for (uint32_t xIndex = 0 ; xIndex < heightfield.GetXCount() ; ++xIndex)
        {
            AddHeightFieldVertexToBuilder(
                builder,
                heightfield,
                xIndex,
                zIndex);
        }
    }


    if (inputQuads)
    {
        // Extract the triangles from the heightfield and add them to the builder
        for (uint32_t zIndex = 0 ; zIndex < heightfield.GetZCount() - 1 ; ++zIndex)
        {
            for (uint32_t xIndex = 0 ; xIndex < heightfield.GetXCount() - 1; ++xIndex)
            {
                AddHeighFieldQuadToBuilder(
                    builder,
                    heightfield,
                    xIndex,
                    zIndex);
            }
        }
    }
    else
    {
        // Extract the triangles from the heightfield and add them to the builder
        for (uint32_t zIndex = 0 ; zIndex < heightfield.GetZCount() - 1 ; ++zIndex)
        {
            for (uint32_t xIndex = 0 ; xIndex < heightfield.GetXCount() - 1; ++xIndex)
            {
                AddHeightFieldTriangleToBuilder(
                    builder,
                    heightfield,
                    xIndex,
                    zIndex,
                    0);

                AddHeightFieldTriangleToBuilder(
                    builder,
                    heightfield,
                    xIndex,
                    zIndex,
                    1);
            }
        }
    }
}


int EAMain(int /*argc*/, char ** /*argv*/)
{
    // Triangle and Vertex counts
    const uint32_t xCount = 2;
    const uint32_t zCount = 2;

    // Create a heightfield
    HeightField heightField(xCount, zCount);

    // Create and set the build parameters
    Builder::BuildParameters buildParameters;

    // This flag indicated that edge cosine data will be stored with each triangle
    buildParameters.unitParameters.unitFlagsDefault = rw::collision::UNITFLAG_EDGEANGLE;

    // These values describe the number of bytes used to store the Group and Surface ID.
    // The range is 0 - 2, with the value of 0 indicating that no ID will be stored.
    buildParameters.unitParameters.groupIDSize = 0;
    buildParameters.unitParameters.surfaceIDSize = 0;

    // This value is the granularity used during vertex compression.
    buildParameters.vertexCompressionGranularity = 0.5f;

    // This flag indicates that quads should be generated, if possible.
    buildParameters.buildQuads = true;

    // This flag indicates that vertex compression will be attempted.
    buildParameters.compressVertices = true;

    // This flag indicates that edge cosine angles of PI/2 or less will
    // cause the corresponding edges to be disabled.
    buildParameters.edgeCosineConcaveAngleTolerance = 0.0f;

    // This is the allocator used to allocate the final TriangleClusterProcedural
    EA::Allocator::ICoreAllocator * triangleClusterProceduralAllocator = EA::Allocator::ICoreAllocator::GetDefaultAllocator();

    // This is the allocator used to allocate the data required by the builder during the build process.
    EA::Allocator::ICoreAllocator * workspaceAllocator = EA::Allocator::ICoreAllocator::GetDefaultAllocator();

    // Construct the builder
    Builder builder(
        heightField.GetVertexCount(),
        heightField.GetTriangleCount(),
        buildParameters,
        triangleClusterProceduralAllocator,
        workspaceAllocator);

    // This flag determines whether or not the data from the heightfield is fed into
    // the builder as triangles or quads.
    const bool InputQuadsIntoTheBuilder = false;

    // Set the builders input data
    FeedBuilder(
        builder,
        heightField,
        InputQuadsIntoTheBuilder);

    // Build the TriangleClusterProcedural
    rw::collision::TriangleClusterProcedural * triangleClusterProcedural = builder.Build();

    common::DescribeTriangleClusterProcedural(*triangleClusterProcedural);

    // Release resources
    if (NULL != triangleClusterProcedural)
        triangleClusterProceduralAllocator->Free(triangleClusterProcedural);

    return 0;
}


#endif // #if !defined EA_PLATFORM_PS3_SPU
