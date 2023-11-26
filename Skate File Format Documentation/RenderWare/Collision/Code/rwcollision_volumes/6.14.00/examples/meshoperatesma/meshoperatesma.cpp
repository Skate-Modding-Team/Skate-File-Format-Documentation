// (c) Electronic Arts. All Rights Reserved.


#include <rw/collision/common.h>


#if !defined EA_PLATFORM_PS3_SPU

#include <stdio.h>

#include <rw/collision/clusteredmeshcluster.h>
#include <rw/collision/simplemappedarray.h>
#include <rw/collision/triangle.h>

#include <rw/collision/meshbuilder/common.h>
#include <rw/collision/meshbuilder/edgecosines.h>

#include <meshoperate/implementations/halfedgemesh/offlinemesh.h>
#include <meshoperate/shapes/diamond.h>

#include <common/common.h>

#include "EAMain/EAEntryPointMain.inl" // For EAMain


/*
This code example demonstrates how to use the ClusteredMeshBuilder Utilities to create
a SimpleMappedArray of TriangleVolumes with edge cosines. The input in this example takes
the form of a MeshOperate mesh.

MeshOperate is a powerful mesh processing toolkit built around a generic API.
Various mesh types are available, each of which supports a well-defined subset of the API.
The mesh type used in this example is required to support the meshoperate::IHalfEdges
API component, and the mesh data is expected to be 2-manifold (with at most two faces
incident to each edge).

The process is broken down into a number of steps. Part of the process of building the
triangle volumes is the computation of edge cosines and convexity flags, encoding
the geometric relationships between triangles and their neighbors on their three
respective edges. The clustered mesh builder edge cosine utilities are used to compute
the edge cosines. The inputs to these utility methods consist of the normalized direction
vectors (normals) of the edge and the two triangles incident to it. The example shows
how to compute these normals using the MeshOperate API.
*/


template <class MeshType>
static typename MeshType::VectorType CalculateTriangleNormal(const MeshType &mesh, const typename MeshType::FaceHandle &faceHandle)
{
    typedef typename MeshType::VertexHandle VertexHandle;
    typedef typename MeshType::VectorType VectorType;
    typedef typename MeshType::FaceVertexCirculator FaceVertexCirculator;
    typedef typename MeshType::MathType MathType;

    // Circulate triangle edges and accumulate vertex points and edge cosine values
    FaceVertexCirculator faceVertices(mesh.FaceVerticesBegin(faceHandle));
    const VertexHandle v0(mesh.FaceVertexCirculatorToHandle(faceVertices));
    ++faceVertices;
    const VertexHandle v1(mesh.FaceVertexCirculatorToHandle(faceVertices));
    ++faceVertices;
    const VertexHandle v2(mesh.FaceVertexCirculatorToHandle(faceVertices));

    const VectorType p0(mesh.GetVertexPosition(v0));
    const VectorType p1(mesh.GetVertexPosition(v1));
    const VectorType p2(mesh.GetVertexPosition(v2));

    const VectorType p0p1(p1 - p0);
    const VectorType p1p2(p2 - p1);

    const VectorType faceDirection(MathType::Cross(p0p1, p1p2));
    const VectorType faceNormal(MathType::NormalizeFast(faceDirection));

    return faceNormal;
}


template <class MeshType>
static bool CalculateEdgeNormals(
    const MeshType &mesh,
    const typename MeshType::HalfEdgeHandle &halfEdgeHandle,
    rwpmath::Vector3 &faceNormalOne,
    rwpmath::Vector3 &faceNormalTwo,
    rwpmath::Vector3 &edgeDirectionOne)
{
    typedef typename MeshType::VertexHandle VertexHandle;
    typedef typename MeshType::FaceHandle FaceHandle;
    typedef typename MeshType::HalfEdgeHandle HalfEdgeHandle;
    typedef typename MeshType::VectorType VectorType;
    typedef typename MeshType::MathType MathType;

    // Handles of the two faces incident to this edge
    const FaceHandle firstFaceHandle(mesh.GetFaceIncidentToHalfEdge(halfEdgeHandle));
    const HalfEdgeHandle otherHalfEdgeHandle(mesh.GetOppositeHalfEdge(halfEdgeHandle));
    const FaceHandle otherFaceHandle(mesh.GetFaceIncidentToHalfEdge(otherHalfEdgeHandle));

    // We assume that the mesh is at least 2-manifold, but it may not also be closed
    if (firstFaceHandle == mesh.GetInvalidFaceHandle())
    {
        return false;
    }

    if (otherFaceHandle == mesh.GetInvalidFaceHandle())
    {
        return false;
    }

    // Normals of the two faces on the edge
    const VectorType firstFaceNormal(CalculateTriangleNormal(mesh, firstFaceHandle));
    const VectorType otherFaceNormal(CalculateTriangleNormal(mesh, otherFaceHandle));

    // Direction of the shared edge, oriented with respect to the first triangle
    const VertexHandle v0(mesh.GetHalfEdgeStartVertex(halfEdgeHandle));
    const VertexHandle v1(mesh.GetHalfEdgeEndVertex(halfEdgeHandle));
    const VectorType p0(mesh.GetVertexPosition(v0));
    const VectorType p1(mesh.GetVertexPosition(v1));
    const VectorType edgeDirection(MathType::NormalizeFast(p1 - p0));

    faceNormalOne = rwpmath::Vector3(firstFaceNormal);
    faceNormalTwo = rwpmath::Vector3(otherFaceNormal);
    edgeDirectionOne = rwpmath::Vector3(edgeDirection);

    return true;
}


template <class MeshType>
static void CopyTriangle(const MeshType &mesh,
                         const typename MeshType::FaceHandle &faceHandle,
                         rw::collision::TriangleVolume *const triangle)
{
    typedef typename MeshType::VertexHandle VertexHandle;
    typedef typename MeshType::HalfEdgeHandle HalfEdgeHandle;
    typedef typename MeshType::VectorType VectorType;
    typedef typename MeshType::FaceHalfEdgeCirculator FaceHalfEdgeCirculator;

    rwpmath::Vector3 trianglePoints[3] = { rwpmath::GetVector3_Zero() };
    rwpmath::VecFloat edgeCosines[3] = { rwpmath::GetVecFloat_Zero() };
    rwpmath::MaskScalar edgeConvexFlags[3] = { rwpmath::GetMaskScalar_False() };

    // Circulate triangle edges and accumulate vertex points and edge cosine values
    FaceHalfEdgeCirculator faceHalfEdges(mesh.FaceHalfEdgesBegin(faceHandle));
    uint32_t edgeIndex(0);

    while (faceHalfEdges && edgeIndex < 3)
    {
        const HalfEdgeHandle halfEdgeHandle(mesh.FaceHalfEdgeCirculatorToHandle(faceHalfEdges));
        
        // Read triangle vertex on this edge
        const VertexHandle startVertexHandle(mesh.GetHalfEdgeStartVertex(halfEdgeHandle));
        const VectorType vertexPoint(mesh.GetVertexPosition(startVertexHandle));
        trianglePoints[edgeIndex] = rwpmath::Vector3(vertexPoint);

        // Calculate normalized directions of the edge and the two incident faces
        rwpmath::Vector3 faceNormalOne(rwpmath::GetVector3_Zero());
        rwpmath::Vector3 faceNormalTwo(rwpmath::GetVector3_Zero());
        rwpmath::Vector3 edgeDirectionOne(rwpmath::GetVector3_Zero());

        if (CalculateEdgeNormals(
            mesh,
            halfEdgeHandle,
            faceNormalOne,
            faceNormalTwo,
            edgeDirectionOne))
        {
            // Calculate edge cosine and convex flag
            rw::collision::meshbuilder::EdgeCosines::ComputeEdgeCosine(
                edgeCosines[edgeIndex],
                edgeConvexFlags[edgeIndex],
                faceNormalOne,
                faceNormalTwo,
                edgeDirectionOne);
        }
        else
        {
            // The edge has only one incident face
            // By convention this is convex with an edge cosine value of minus one
            edgeCosines[edgeIndex] = -rwpmath::GetVecFloat_One();
            edgeConvexFlags[edgeIndex] = rwpmath::GetMaskScalar_True();
        }

        ++edgeIndex;
        ++faceHalfEdges;
    }

    // Check that the face doesn't have more than three edges
    EA_ASSERT(!faceHalfEdges);

    // Set up triangle points and edge cosines
    triangle->SetPoints(trianglePoints[0], trianglePoints[1], trianglePoints[2]);
    triangle->SetEdgeCos(edgeCosines[0], edgeCosines[1], edgeCosines[2]);

    // Set up triangle flags. Triangle edges are marked convex by default
    uint32_t triangleFlags = rw::collision::VOLUMEFLAG_TRIANGLEDEFAULT;
    if (!edgeConvexFlags[0].GetBool())
    {
        triangleFlags &= ~uint32_t(rw::collision::VOLUMEFLAG_TRIANGLEEDGE0CONVEX);
    }

    if (!edgeConvexFlags[1].GetBool())
    {
        triangleFlags &= ~uint32_t(rw::collision::VOLUMEFLAG_TRIANGLEEDGE1CONVEX);
    }

    if (!edgeConvexFlags[2].GetBool())
    {
        triangleFlags &= ~uint32_t(rw::collision::VOLUMEFLAG_TRIANGLEEDGE2CONVEX);
    }

    triangle->SetFlags(triangleFlags);
}


template <class MeshType>
static void CopyMeshTriangles(MeshType &mesh, rw::collision::SimpleMappedArray *const sma)
{
    typedef typename MeshType::FaceIterator FaceIterator;
    typedef typename MeshType::FaceHandle FaceHandle;

    // Copy the faces, which must be triangular
    FaceIterator meshFaces(mesh.FacesBegin());
    const FaceIterator meshFacesEnd(mesh.FacesEnd());
    uint16_t triangleIndex(0);

    while (meshFaces != meshFacesEnd)
    {
        const FaceHandle faceHandle(mesh.FaceIteratorToHandle(meshFaces));

        rw::collision::Volume *const volume(sma->GetVolume(triangleIndex));
        rw::collision::TriangleVolume *const triangle(static_cast<rw::collision::TriangleVolume *>(volume));

        CopyTriangle(mesh, faceHandle, triangle);

        ++meshFaces;
        ++triangleIndex;
    }
}


int EAMain(int /*argc*/, char ** /*argv*/)
{
    // Build a simple MeshOperate mesh for example data
    // We need a simple mesh with triangular faces, the diamond will do
    typedef meshoperate::halfedge::offline::Mesh HalfEdgeMesh;
    typedef meshoperate::shapes::Diamond<HalfEdgeMesh> DiamondShape;

    HalfEdgeMesh mesh;
    DiamondShape diamond(mesh);
    const DiamondShape::Parameters diamondParams;
    diamond.Generate(diamondParams);

    //
    // Initialize the SMA
    //

    const uint32_t triangleCount(mesh.GetNumFaces());
    EA::Physics::SizeAndAlignment sal = rw::collision::SimpleMappedArray::GetResourceDescriptor(triangleCount);
    void * mem = EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Alloc(sal.GetSize(), 0, 0, sal.GetAlignment());
    rw::collision::SimpleMappedArray *sma = rw::collision::SimpleMappedArray::Initialize(mem, triangleCount);

    //
    // Fill in the SMA with triangle data from the mesh plus edge cosines
    //

    CopyMeshTriangles(mesh, sma);

    common::DescribeSMA(sma);

    EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Free(mem);

    return 0;
}


#endif // #if !defined EA_PLATFORM_PS3_SPU
