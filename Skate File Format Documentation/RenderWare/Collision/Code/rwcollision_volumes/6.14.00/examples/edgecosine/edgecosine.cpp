// (c) Electronic Arts. All Rights Reserved.


#include <rw/collision/common.h>


#if !defined EA_PLATFORM_PS3_SPU

#include <stdio.h>

#include <rw/collision/meshbuilder/edgecosines.h>

#include "EAMain/EAEntryPointMain.inl" // For EAMain

/*
This code example demonstrates how to use the ClusteredMeshBuilder utilities to compute
edge cosines for edges shared by two adjacent triangles.

Computing edge cosines directly allows the resulting data to be fed into collision
objects other than full-blown clustered meshes, for example single clusters,
SimpleMappedArrays of triangle volumes, or even completely custom user classes.

Utilities are provided for computing edge cosines, convexity flags, and "extended" edge
cosines (with extended range [-1, +3], used to encode convexity and concavity). In
this example we show the computation of a simple edge cosine.
*/


int EAMain(int /*argc*/, char ** /*argv*/)
{
    // First triangle vertices
    const rwpmath::Vector3 p0(0.0f, 0.0f, 0.0f);
    const rwpmath::Vector3 p1(0.0f, 0.0f, 1.0f);
    const rwpmath::Vector3 p2(1.0f, 0.0f, 0.0f);
    
    // Second triangle vertices
    const rwpmath::Vector3 p3(0.0f, 0.0f, 0.0f);
    const rwpmath::Vector3 p4(1.0f, 0.0f, 0.0f);
    const rwpmath::Vector3 p5(0.0f, 0.0f, -1.0f);

    // Edge directions
    const rwpmath::Vector3 edge01(rwpmath::NormalizeFast(p1 - p0));
    const rwpmath::Vector3 edge20(rwpmath::NormalizeFast(p0 - p2));

    const rwpmath::Vector3 edge34(rwpmath::NormalizeFast(p4 - p3));
    const rwpmath::Vector3 edge45(rwpmath::NormalizeFast(p5 - p4));

    // Triangle normals
    const rwpmath::Vector3 triangleOneDirection(rwpmath::Cross(edge20, edge01));
    const rwpmath::Vector3 triangleTwoDirection(rwpmath::Cross(edge34, edge45));
    const rwpmath::Vector3 triangleOneNormal(rwpmath::NormalizeFast(triangleOneDirection));
    const rwpmath::Vector3 triangleTwoNormal(rwpmath::NormalizeFast(triangleTwoDirection));

    rwpmath::VecFloat edgeCosine(rwpmath::GetVecFloat_Zero());
    rwpmath::MaskScalar convex(rwpmath::GetMaskScalar_False());

    // The edge cosine method takes the normalized direction vectors of the two triangles and
    // the normalized direction of the shared edge, oriented with respect to the first triangle.
    rw::collision::meshbuilder::EdgeCosines::ComputeEdgeCosine(
        edgeCosine,
        convex,
        triangleOneNormal,
        triangleTwoNormal,
        edge20);

    const float edgeCosineFloat(edgeCosine);
    const bool convexBool(convex.GetBool());

    printf("Calculated edge cosine value is %f\n", edgeCosineFloat);
    printf("Edge is considered %s\n", convexBool ? "convex" : "concave");

    return 0;
}


#endif // #if !defined EA_PLATFORM_PS3_SPU

