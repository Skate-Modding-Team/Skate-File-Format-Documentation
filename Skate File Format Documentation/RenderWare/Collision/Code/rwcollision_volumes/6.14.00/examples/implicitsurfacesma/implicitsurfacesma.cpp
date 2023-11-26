// (c) Electronic Arts. All Rights Reserved.


#include <rw/collision/common.h>


#if !defined EA_PLATFORM_PS3_SPU

#include <stdio.h>

#include <rw/collision/simplemappedarray.h>
#include <rw/collision/triangle.h>

#include <rw/collision/meshbuilder/common.h>
#include <rw/collision/meshbuilder/edgecosines.h>

#include <common/common.h>

#include "heightfield.h"

#include "EAMain/EAEntryPointMain.inl" // For EAMain


/*
This code example demonstrates how to use the ClusteredMeshBuilder utilities to create
a SimpleMappedArray of TriangleVolumes with edge cosines. The input in this example takes
the form of an implicit surface, more specifically a height-field. 

Since the connectivity is already implied by the height-field the process of creating an
SMA consists of (per triangle):

Step A) Extracting a triangle from the height field.

Step B) Calculating the triangle edge cosine and flags

Step C) Copying the triangle data into the SMA.
*/


/// Struct which wraps an edge cosine and convex flag
struct EdgeCosineAndFlags
{
    rwpmath::VecFloat edgeCosine;
    rwpmath::MaskScalar convex;
};


/**
Calculates the edge cosine for and convexity flag for an unmatched edge
*/
static void CalculateUnmatchedEdgeCosineAndFlags(EdgeCosineAndFlags& edge)
{
    // Set the edge cosine to the unshared value
    edge.edgeCosine = CLUSTEREDMESHBUILDER_EDGECOS_OF_UNMATCHED_EDGE;

    // Set the edge flag to the unshared value
    edge.convex.SetTrue();
}


/**
Calculates edge cosine and convexity flag for a matched edge
*/
static void CalculateMatchedEdgeCosineAndFlags(EdgeCosineAndFlags & edge,
                                               rwpmath::Vector3::InParam edgeVector,
                                               rwpmath::Vector3::InParam triangleNormal,
                                               rwpmath::Vector3::InParam neighborTriangleNormal)
{
    // Calculate the extended edge cosine
    rwpmath::VecFloat extendedEdgeCosine = rw::collision::meshbuilder::EdgeCosines::ComputeExtendedEdgeCosine(
        triangleNormal,
        neighborTriangleNormal,
        edgeVector);

    // Convert the extended edge cosine into a standard edge cosine and set the edge flags
    rw::collision::meshbuilder::EdgeCosines::DecodeExtendedEdgeCosine(
        edge.edgeCosine,
        edge.convex,
        extendedEdgeCosine);
}


/**
Sets edge cosines and convexity flags of the edges of a triangle volume
*/
static void InitializeTriangle(rw::collision::TriangleVolume & triangle,
                               rwpmath::Vector3::InParam v0,
                               rwpmath::Vector3::InParam v1,
                               rwpmath::Vector3::InParam v2,
                               const EdgeCosineAndFlags & edge0,
                               const EdgeCosineAndFlags & edge1,
                               const EdgeCosineAndFlags & edge2)
{
    // Create the triangle flags
    uint32_t triangleFlags = rw::collision::VOLUMEFLAG_TRIANGLEDEFAULT;
    if (!edge0.convex.GetBool())
    {
        triangleFlags &= ~uint32_t(rw::collision::VOLUMEFLAG_TRIANGLEEDGE0CONVEX);
    }

    if (!edge1.convex.GetBool())
    {
        triangleFlags &= ~uint32_t(rw::collision::VOLUMEFLAG_TRIANGLEEDGE1CONVEX);
    }

    if (!edge2.convex.GetBool())
    {
        triangleFlags &= ~uint32_t(rw::collision::VOLUMEFLAG_TRIANGLEEDGE2CONVEX);
    }

    // Set the triangle flags
    triangle.SetFlags(triangleFlags);

    // Set the triangle vertices
    triangle.SetPoints(v0,
        v1,
        v2);

    // Set the triangle edge cosines
    triangle.SetEdgeCos(edge0.edgeCosine,
                        edge1.edgeCosine,
                        edge2.edgeCosine);
}


static rwpmath::Vector3 ComputeTriangleNormalFast(
    rwpmath::Vector3::InParam p0,
    rwpmath::Vector3::InParam p1,
    rwpmath::Vector3::InParam p2)
{
    const rwpmath::Vector3 p0p1(rwpmath::NormalizeFast(p1 - p0));
    const rwpmath::Vector3 p0p2(rwpmath::NormalizeFast(p2 - p0));

    return NormalizeFast(Cross(p0p1, p0p2));
}


/**
Calculates edge cosine and convexity flag for an edge
*/
static void CalculateEdgeCosineAndFlags(EdgeCosineAndFlags & edge,
                                        rwpmath::Vector3::InParam edgeStart,
                                        rwpmath::Vector3::InParam edgeEnd,
                                        rwpmath::Vector3::InParam triangleNormal,
                                        const rwpmath::Vector3 * neighboringTriangleVertex)
{
    // If the triangle has a neighbor along this edge, i.e whether 
    // the edge is shared between two triangles.
    if (NULL != neighboringTriangleVertex)
    {
        // Calculate the normal of the neighboring triangle
        rwpmath::Vector3 neighborTriangleNormal = ComputeTriangleNormalFast(
            edgeEnd,
            edgeStart,
            *neighboringTriangleVertex);

        // Calculate the edge direction
        rwpmath::Vector3 edgeVector = edgeEnd - edgeStart;

        // Calculate the edge cosine and flags value
        CalculateMatchedEdgeCosineAndFlags(edge,
                                           edgeVector,
                                           triangleNormal,
                                           neighborTriangleNormal);
    }
    else
    {
        // Calculate the edge cosine and flags of the edge which
        // is not shared between two triangle.
        CalculateUnmatchedEdgeCosineAndFlags(edge);
    }
}


/**
Calculates edge cosines and convexity flags of the edges of a triangle
*/
static void CalculateTriangleEdgeCosinesAndEdgeFlags(EdgeCosineAndFlags &edge0,
                                                     EdgeCosineAndFlags &edge1,
                                                     EdgeCosineAndFlags &edge2,
                                                     rwpmath::Vector3::InParam triangleVertex0,
                                                     rwpmath::Vector3::InParam triangleVertex1,
                                                     rwpmath::Vector3::InParam triangleVertex2,
                                                     const rwpmath::Vector3 * extraVertex0,
                                                     const rwpmath::Vector3 * extraVertex1,
                                                     const rwpmath::Vector3 * extraVertex2)
{
    // Calculate the triangle normal of the triangle
    const rwpmath::Vector3 triangleNormal = ComputeTriangleNormalFast(
        triangleVertex0,
        triangleVertex1,
        triangleVertex2);

    // Edge 0
    {
        CalculateEdgeCosineAndFlags(
            edge0,
            triangleVertex0,
            triangleVertex1,
            triangleNormal,
            extraVertex0);
    }

    // Edge 1
    {
        CalculateEdgeCosineAndFlags(
            edge1,
            triangleVertex1,
            triangleVertex2,
            triangleNormal,
            extraVertex1);
    }

    // Edge 2
    {
        CalculateEdgeCosineAndFlags(
            edge2,
            triangleVertex2,
            triangleVertex0,
            triangleNormal,
            extraVertex2);
    }
}


/**
Creates a triangle volume from a height field triangle
*/
static void CreateTriangleVolumeFromHeightFieldTriangle(rw::collision::TriangleVolume * triangle,
                                                        rwpmath::Vector3 & v0,
                                                        rwpmath::Vector3 & v1,
                                                        rwpmath::Vector3 & v2,
                                                        rwpmath::Vector3 * ev0,
                                                        rwpmath::Vector3 * ev1,
                                                        rwpmath::Vector3 * ev2)
{
    // The edge cosine and flags of each edge of the triangle
    EdgeCosineAndFlags edge0;
    EdgeCosineAndFlags edge1;
    EdgeCosineAndFlags edge2;

    // Calculate the edge cosine values and flags of the triangle
    CalculateTriangleEdgeCosinesAndEdgeFlags(
        edge0,
        edge1,
        edge2,
        v0,
        v1,
        v2,
        ev0,
        ev1,
        ev2);

    // Set the values in the Triangle Volume
    InitializeTriangle(
        *triangle,
        v0,
        v1,
        v2,
        edge0,
        edge1,
        edge2);
}


/**
Creates two triangle volume from a height field quad
*/
static void CreateTrianglesVolumeFromHeightFieldQuad(rw::collision::TriangleVolume * triangleA,
                                                     rw::collision::TriangleVolume * triangleB,
                                                     HeightField & heightField,
                                                     const uint32_t xIndex,
                                                     const uint32_t zIndex)
{
    // The Quad vertices
    rwpmath::Vector3 v0;
    rwpmath::Vector3 v1;
    rwpmath::Vector3 v2;
    rwpmath::Vector3 v3;

    // Get the vertices of the current height field quad
    heightField.GetQuadVertices(v0,
                                v1,
                                v2,
                                v3,
                                xIndex,
                                zIndex);

    // The vertices surrounding the Quad
    rwpmath::Vector3 ev0;
    rwpmath::Vector3 ev1;
    rwpmath::Vector3 ev2;
    rwpmath::Vector3 ev3;
    rwpmath::Vector3 * p_ev0 = &ev0;
    rwpmath::Vector3 * p_ev1 = &ev1;
    rwpmath::Vector3 * p_ev2 = &ev2;
    rwpmath::Vector3 * p_ev3 = &ev3;

    // Get the vertices of the triangles surrounding the current quad
    heightField.GetSurroundingTriangleVertices(p_ev0,
                                               p_ev1,
                                               p_ev2,
                                               p_ev3,
                                               xIndex,
                                               zIndex);

    // Create the first triangle in the quad
    CreateTriangleVolumeFromHeightFieldTriangle(triangleA,
                                                v0,
                                                v1,
                                                v2,
                                                p_ev0,
                                                p_ev1,
                                                &v3);

    // Create the second triangle in the quad
    CreateTriangleVolumeFromHeightFieldTriangle(triangleB,
                                                v2,
                                                v1,
                                                v3,
                                                &v0,
                                                p_ev2,
                                                p_ev3);
}


int EAMain(int /*argc*/, char ** /*argv*/)
{
    // The width and length of the height field
    const uint32_t xCount = 4u;
    const uint32_t zCount = 4u;

    // The number of triangles
    const uint32_t triangleCount =(xCount - 1u) * (zCount - 1u) * 2u;

    // Create the HeightField
    HeightField heightField(xCount, zCount);

    // Create the SMA
    EA::Physics::SizeAndAlignment sal = rw::collision::SimpleMappedArray::GetResourceDescriptor(triangleCount);
    void * mem = EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Alloc(sal.GetSize(), 0, 0, sal.GetAlignment());
    rw::collision::SimpleMappedArray * sma = rw::collision::SimpleMappedArray::Initialize(mem, triangleCount);

    // Iterate through each Quad on the height field
    uint16_t triangleIndex = 0u;
    for (uint32_t xIndex = 0 ; xIndex < xCount - 1u ; ++xIndex)
    {
        for (uint32_t zIndex = 0 ; zIndex < zCount - 1u ; ++zIndex)
        {
            // Get the two quad triangles from the SMA
            rw::collision::TriangleVolume * triangleA = static_cast<rw::collision::TriangleVolume*>(sma->GetVolume(triangleIndex++));
            rw::collision::TriangleVolume * triangleB = static_cast<rw::collision::TriangleVolume*>(sma->GetVolume(triangleIndex++));

            // Create two triangle volumes from the HeightField Quad
            CreateTrianglesVolumeFromHeightFieldQuad(triangleA,
                                                     triangleB,
                                                     heightField,
                                                     xIndex,
                                                     zIndex);
        }
    }

    // Describe the SMA
    common::DescribeSMA(sma);

    // Free the SMA memory
    EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Free(mem);

    return 0;
}


#endif // #if !defined EA_PLATFORM_PS3_SPU

