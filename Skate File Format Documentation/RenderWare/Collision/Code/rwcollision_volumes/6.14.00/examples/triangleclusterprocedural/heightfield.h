// (c) Electronic Arts. All Rights Reserved.
#ifndef RWCOLLISION_VOLUMES_EXAMPLES_HEIGHTFIELD_H
#define RWCOLLISION_VOLUMES_EXAMPLES_HEIGHTFIELD_H


#include <rw/collision/common.h>

#if !defined EA_PLATFORM_PS3_SPU


#include <rw/collision/meshbuilder/common.h>
#include <rw/collision/meshbuilder/edgecosines.h>


// Utility methods
static inline void AlignPointer(uint8_t *&ptr, const uint8_t alignment)
{
    ptr = (uint8_t *)(((uintptr_t)ptr + ((uintptr_t)alignment - 1)) & ~((uintptr_t)alignment - 1));
}


// Using directives
using namespace rw::collision;


/*
HeightField Class

The purpose of this class is to simply provide triangle and vertex input data
to the example class TriangleClusterProceduralBuilder. Is is, by no means, an
example of how a height field should be implemented.

Internally a collection of vertices laid out in a grid fashion, which can be
indexed by a x and z coordinate.

The methods of interest are 

HeightField() - The constructor takes 2 parameters which describe the dimensions
of the height field grid.

GetTriangle/QuadVertexIndices() - Gets the vertices of a triangle/quad.

GetTriangle/QuadNeighborIndices() - Gets the indices of triangles neighboring a triangle/quad.

GetTriangle/Quad ExtendedEdgeCosines() - Gets the extended edge cosines of a triangle/quad.
**/
class HeightField
{
public:

    // Flag used to indicate an unshared triangle edge, i.e. the triangle has no neighboring triangle
    // sharing that edge.
    static const uint32_t NO_TRIANGLE_NEIGHBOR = CLUSTEREDMESHBUILDER_TRIANGLENEIGHBORINDEX_NOMATCH;


    /*
    Triangle information
    **/
    struct Triangle
    {
        /// The indices of the triangles vertices. These values index into the HeighFields collection of vertices.
        uint32_t m_vertices[3];
        /// The Extended edge cosine values of each edge.
        float extendedEdgeCosines[3];
        /// The indices of the neighboring triangles.
        uint32_t neighborTriangleIndices[3];
    };

    /*
    Constructor

    Allocates internal data structures and initializes the height values.
    **/
    HeightField(const uint32_t x,
                const uint32_t z)
        : xCount(x)
        , zCount(z)
        , m_vertexBuffer(NULL)
        , m_vertices(NULL)
    {
        // Create the vertex buffer
        m_vertexBuffer = reinterpret_cast<uint8_t *>(new rwpmath::Vector3[sizeof(rwpmath::Vector3) * (xCount * zCount + 1)]);

        // Align vertices
        uint8_t * ptr = static_cast<uint8_t*>(m_vertexBuffer);
        AlignPointer(ptr, RW_MATH_VECTOR3_ALIGNMENT);
        m_vertices = reinterpret_cast<rwpmath::Vector3*>(ptr);

        uint32_t vertexIndex = 0u;
        // Initialize the vertices
        for (uint32_t xIndex = 0 ; xIndex < xCount ; ++xIndex)
        {
            for (uint32_t zIndex = 0 ; zIndex < zCount ; ++zIndex)
            {
                uint32_t height = rw::math::Random() % 5;
                m_vertices[vertexIndex++] = rwpmath::Vector3(
                    static_cast<float>(xIndex),
                    static_cast<float>(height),
                    static_cast<float>(zIndex));
            }
        }
    }

    /*
    Destructor

    Deallocates internal data structures.
    **/
    ~HeightField()
    {
        delete [] m_vertexBuffer;
    }

    /*
    Gets the triangle vertex indices of a specified triangle.

    \param v0 the index of the first vertex.
    \param v1 the index of the second vertex.
    \param v2 the index of the third vertex.
    \param xIndex the x index, of the triangle, into the HeighField.
    \param yIndex the y index, of the triangle, into the HeighField.
    \param trianglePairIndex the index of the triangle into the trianglePair,
                             0 == first triangle in pair,
                             1 == second triangle in pair.
    **/
    void
    GetTriangleVertexIndices(
        uint32_t & v0,
        uint32_t & v1,
        uint32_t & v2,
        const uint32_t xIndex,
        const uint32_t zIndex,
        const uint32_t trianglePairIndex) const
    {
        if (0 == trianglePairIndex)
        {
            v0 = (zIndex * xCount) + xIndex;
            v1 = ((zIndex + 1u) * xCount) + xIndex;
            v2 = (zIndex * xCount) + xIndex + 1u;
        }
        else
        {
            v0 = ((zIndex + 1u) * xCount) + xIndex;
            v1 = ((zIndex + 1u) * xCount) + xIndex + 1u;
            v2 = (zIndex * xCount) + xIndex + 1u;
        }
    }

    /*
    Gets the quad vertex indices of a specified quad.

    \param v0 the index of the first vertex.
    \param v1 the index of the second vertex.
    \param v2 the index of the third vertex.
    \param v3 the index of the fourth vertex.
    \param xIndex the x index, of the quad, into the HeighField.
    \param yIndex the y index, of the quad, into the HeighField.
    **/
    void
    GetQuadVertexIndices(
        uint32_t & v0,
        uint32_t & v1,
        uint32_t & v2,
        uint32_t & v3,
        const uint32_t xIndex,
        const uint32_t zIndex) const
    {
        v0 = (zIndex * xCount) + xIndex;
        v1 = ((zIndex + 1u) * xCount) + xIndex;
        v2 = ((zIndex + 1u) * xCount) + xIndex + 1u;
        v3 = (zIndex * xCount) + xIndex + 1u;
    }



    /*
    Gets the indices of the neighboring triangles of a specified triangle.

    \param neighbor0 the index of the first neighboring triangle.
    \param neighbor1 the index of the second neighboring triangle.
    \param neighbor2 the index of the third neighboring triangle
    \param xIndex the x index, of the triangle, into the HeighField.
    \param yIndex the y index, of the triangle, into the HeighField.
    \param trianglePairIndex the index of the triangle into the trianglePair,
                             0 == first triangle in pair,
                             1 == second triangle in pair.
    **/
    void
    GetTriangleNeighborIndices(
        uint32_t & neighbor0,
        uint32_t & neighbor1,
        uint32_t & neighbor2,
        const uint32_t xIndex,
        const uint32_t zIndex,
        const uint32_t trianglePairIndex) const
    {
        if (0 == trianglePairIndex)
        {
            if (0 == xIndex)
            {
                neighbor0 = NO_TRIANGLE_NEIGHBOR;
            }
            else
            {
                neighbor0 = GetTriangleIndex(
                    xIndex - 1u,
                    zIndex,
                    1);
            }

            neighbor1 = GetTriangleIndex(
                            xIndex,
                            zIndex,
                            1);

            if (0 == zIndex)
            {
                neighbor2 = NO_TRIANGLE_NEIGHBOR;
            }
            else
            {
                neighbor2 = GetTriangleIndex(
                                xIndex,
                                zIndex - 1,
                                1);
            }
        }
        else // if ( 1 == trianglePairIndex )
        {
            if (zCount - 2u == zIndex)
            {
                neighbor0 = NO_TRIANGLE_NEIGHBOR;
            }
            else
            {
                neighbor0 = GetTriangleIndex(
                                xIndex,
                                zIndex + 1u,
                                0);
            }

            if (xCount - 2u == xIndex)
            {
                neighbor1 = NO_TRIANGLE_NEIGHBOR;
            }
            else
            {
                neighbor1 = GetTriangleIndex(
                                xIndex + 1u,
                                zIndex,
                                0);
            }

            neighbor2 = GetTriangleIndex(
                            xIndex,
                            zIndex,
                            0);
        }
    }

    /*
    Gets the indices of the neighboring triangles of a specified quad.

    \param neighbor0 the index of the first neighboring triangle.
    \param neighbor1 the index of the second neighboring triangle.
    \param neighbor2 the index of the third neighboring triangle.
    \param neighbor2 the index of the fourth neighboring triangle.
    \param xIndex the x index, of the quad, into the HeighField.
    \param yIndex the y index, of the quad, into the HeighField.
    **/
    void
    GetQuadNeighborIndices(
        uint32_t & neighbor0,
        uint32_t & neighbor1,
        uint32_t & neighbor2,
        uint32_t & neighbor3,
        const uint32_t xIndex,
        const uint32_t zIndex) const
    {
        // Neighbor 0
        if (0 == xIndex)
        {
            neighbor0 = NO_TRIANGLE_NEIGHBOR;
        }
        else
        {
            neighbor0 = GetTriangleIndex(
                xIndex - 1u,
                zIndex,
                1);
        }

        // Neighbor 1
        if (zCount - 2u == zIndex)
        {
            neighbor1 = NO_TRIANGLE_NEIGHBOR;
        }
        else
        {
            neighbor1 = GetTriangleIndex(
                xIndex,
                zIndex + 1u,
                0);
        }

        // Neighbor 2
        if (xCount - 2u == xIndex)
        {
            neighbor2 = NO_TRIANGLE_NEIGHBOR;
        }
        else
        {
            neighbor2 = GetTriangleIndex(
                xIndex + 1u,
                zIndex,
                0);
        }

        // Neighbor 3
        if (0 == zIndex)
        {
            neighbor3 = NO_TRIANGLE_NEIGHBOR;
        }
        else
        {
            neighbor3 = GetTriangleIndex(
                            xIndex,
                            zIndex - 1,
                            1);
        }
     }


    /*
    Gets the extended edge cosines of a specified quad.

    \param edgeCos0 the extended edge cosine of the first quad edge.
    \param edgeCos1 the extended edge cosine of the second quad edge.
    \param edgeCos2 the extended edge cosine of the third quad edge.
    \param edgeCos3 the extended edge cosine of the third quad edge.
    \param xIndex the x index, of the triangle, into the HeighField.
    \param yIndex the y index, of the triangle, into the HeighField.
    **/
    void
    GetQuadExtendedEdgeCosines(
        rwpmath::VecFloat & edgeCos0,
        rwpmath::VecFloat & edgeCos1,
        rwpmath::VecFloat & edgeCos2,
        rwpmath::VecFloat & edgeCos3,
        const uint32_t xIndex,
        const uint32_t zIndex) const
    {
        rwpmath::Vector3 v0, v1, v2, v3;

        // Get the vertices of the triangle
        GetQuadVertices(
            v0,
            v1,
            v2,
            v3,
            xIndex,
            zIndex);

        rwpmath::Vector3 * adjacentV0 = NULL;
        rwpmath::Vector3 * adjacentV1 = NULL;
        rwpmath::Vector3 * adjacentV2 = NULL;
        rwpmath::Vector3 * adjacentV3 = NULL;

        // Get the vertices of the neighbor triangles
        GetAdjacentVertices(
            &adjacentV0,
            &adjacentV1,
            &adjacentV2,
            &adjacentV3,
            xIndex,
            zIndex);

        // Get the extended edge cosines
        GetExtendedEdgeCosines(
            edgeCos0,
            edgeCos1,
            edgeCos2,
            edgeCos3,
            v0,
            v1,
            v2,
            v3,
            adjacentV0,
            adjacentV1,
            adjacentV2,
            adjacentV3);
    }

        /*
    Gets the extended edge cosines of a specified triangle.

    \param edgeCos0 the extended edge cosine of the first triangle edge.
    \param edgeCos1 the extended edge cosine of the second triangle edge.
    \param edgeCos2 the extended edge cosine of the third triangle edge.
    \param xIndex the x index, of the triangle, into the HeighField.
    \param yIndex the y index, of the triangle, into the HeighField.
    \param trianglePairIndex the index of the triangle into the trianglePair,
                             0 == first triangle in pair,
                             1 == second triangle in pair.
    **/
    void
    GetTriangleExtendedEdgeCosines(
        rwpmath::VecFloat & edgeCos0,
        rwpmath::VecFloat & edgeCos1,
        rwpmath::VecFloat & edgeCos2,
        const uint32_t xIndex,
        const uint32_t zIndex,
        const uint32_t triangleIndex) const
    {
        rwpmath::Vector3 v0, v1, v2;

        // Get the vertices of the triangle
        GetTriangleVertices(
            v0,
            v1,
            v2,
            xIndex,
            zIndex,
            triangleIndex);

        rwpmath::Vector3 * adjacentV0 = NULL;
        rwpmath::Vector3 * adjacentV1 = NULL;
        rwpmath::Vector3 * adjacentV2 = NULL;

        // Get the vertices of the neighbor triangles
        GetAdjacentVertices(
            &adjacentV0,
            &adjacentV1,
            &adjacentV2,
            xIndex,
            zIndex,
            triangleIndex);

        // Get the extended edge cosines
        GetExtendedEdgeCosines(
            edgeCos0,
            edgeCos1,
            edgeCos2,
            v0,
            v1,
            v2,
            adjacentV0,
            adjacentV1,
            adjacentV2);
    }

    /*
    Gets the x dimension of the HeightField.

    \return the x dimension of the HeightField.
    **/
    uint32_t GetXCount() const
    {
        return xCount;
    }

    /*
    Gets the x dimension of the HeightField.

    \return the x dimension of the HeightField.
    **/
    uint32_t GetZCount() const
    {
        return zCount;
    }

    /*
    Gets the vertex count of the HeightField.

    \return the vertex count of the HeightField.
    **/
    uint32_t GetVertexCount() const
    {
        return xCount * zCount;
    }

    /*
    Gets the triangle count of the HeightField.

    \return the triangle count of the HeightField.
    **/
    uint32_t GetTriangleCount() const
    {
        return ((xCount - 1) * (zCount - 1) * 2);
    }

    /*
    Gets the vertex indicated by the xIndex and zIndex.

    \param vertex the vertex
    \param xIndex the x index into the height field grid.
    \param zIndex the z index into the height field grid.
    **/
    void GetVertex(rwpmath::Vector3 & vertex,
                   const uint32_t xIndex,
                   const uint32_t zIndex) const
    {
        vertex = m_vertices[(zIndex * xCount) + xIndex];
    }

private:

    /*
    Gets the index of a specific triangle.

    \param xIndex the x index of the triangle.
    \param yIndex the y index of the triangle.
    \param trianglePairIndex the triangle pair index of the triangle.
    **/
    uint32_t GetTriangleIndex(
        const uint32_t xIndex,
        const uint32_t zIndex,
        const uint32_t trianglePairIndex) const
    {
        return (zIndex * ((xCount - 1u) * 2u)) + (xIndex * 2u) + trianglePairIndex;
    }

    /*
    Gets the vertices of a specific triangle.

    \param v0 the first vertex of the triangle.
    \param v1 the second vertex of the triangle.
    \param v2 the third vertex of the triangle.
    \param xIndex the x index of the triangle.
    \param yIndex the y index of the triangle.
    \param trianglePairIndex the triangle pair index of the triangle.
    **/
    void GetTriangleVertices(
        rwpmath::Vector3 & v0,
        rwpmath::Vector3 & v1,
        rwpmath::Vector3 & v2,
        const uint32_t xIndex,
        const uint32_t zIndex,
        const uint32_t triangleIndex) const
    {
        uint32_t v0Index, v1Index, v2Index;
         GetTriangleVertexIndices(
             v0Index,
             v1Index,
             v2Index,
             xIndex,
             zIndex,
             triangleIndex);

         v0 = m_vertices[v0Index];
         v1 = m_vertices[v1Index];
         v2 = m_vertices[v2Index];
    }

    /*
    Gets the vertices of a specific quad.

    \param v0 the first vertex of the quad.
    \param v1 the second vertex of the quad.
    \param v2 the third vertex of the quad.
    \param v3 the fourth vertex of the quad.
    \param xIndex the x index of the quad.
    \param yIndex the y index of the quad.
    **/
    void GetQuadVertices(
        rwpmath::Vector3 & v0,
        rwpmath::Vector3 & v1,
        rwpmath::Vector3 & v2,
        rwpmath::Vector3 & v3,
        const uint32_t xIndex,
        const uint32_t zIndex) const
    {
        uint32_t v0Index, v1Index, v2Index, v3Index;

        GetQuadVertexIndices(
            v0Index,
            v1Index,
            v2Index,
            v3Index,
            xIndex,
            zIndex);

         v0 = m_vertices[v0Index];
         v1 = m_vertices[v1Index];
         v2 = m_vertices[v2Index];
         v3 = m_vertices[v3Index];
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


    /*
    Calculates the extended edge cosines of the edges of a triangle.

    \param edgeCos0 extended edge cosine of the first edge.
    \param edgeCos1 extended edge cosine of the second edge.
    \param edgeCos2 extended edge cosine of the third edge.
    \param v0 the first vertex of the triangle.
    \param v1 the second vertex of the triangle.
    \param v2 the third vertex of the triangle.
    \param aV0 the vertex of the triangle neighboring edge one.
    \param aV1 the vertex of the triangle neighboring edge two.
    \param aV2 the vertex of the triangle neighboring edge three.
    **/
    void GetExtendedEdgeCosines(
        rwpmath::VecFloat & edgeCos0,
        rwpmath::VecFloat & edgeCos1,
        rwpmath::VecFloat & edgeCos2,
        rwpmath::Vector3::InParam v0,
        rwpmath::Vector3::InParam v1,
        rwpmath::Vector3::InParam v2,
        rwpmath::Vector3 * aV0,
        rwpmath::Vector3 * aV1,
        rwpmath::Vector3 * aV2) const
    {
        // Calculate the triangle normal of the triangle
        rwpmath::Vector3 triangleNormal = ComputeTriangleNormalFast(
            v0,
            v1,
            v2);

        // Edge 0
        {
            CalculateExtendedEdgeCosine(
                edgeCos0,
                v0,
                v1,
                triangleNormal,
                aV0);
        }

        // Edge 1
        {
            CalculateExtendedEdgeCosine(
                edgeCos1,
                v1,
                v2,
                triangleNormal,
                aV1);
        }

        // Edge 2
        {
            CalculateExtendedEdgeCosine(
                edgeCos2,
                v2,
                v0,
                triangleNormal,
                aV2);
        }
    }

    /*
    Calculates the extended edge cosines of the edges of a quad.

    \param edgeCos0 extended edge cosine of the first edge.
    \param edgeCos1 extended edge cosine of the second edge.
    \param edgeCos2 extended edge cosine of the third edge.
    \param edgeCos3 extended edge cosine of the fourth edge.
    \param v0 the first vertex of the quad.
    \param v1 the second vertex of the quad.
    \param v2 the third vertex of the quad.
    \param v3 the fourth vertex of the quad.
    \param aV0 the vertex of the triangle neighboring edge one.
    \param aV1 the vertex of the triangle neighboring edge two.
    \param aV2 the vertex of the triangle neighboring edge three.
    \param aV2 the vertex of the triangle neighboring edge four.
    **/
    void GetExtendedEdgeCosines(
        rwpmath::VecFloat & edgeCos0,
        rwpmath::VecFloat & edgeCos1,
        rwpmath::VecFloat & edgeCos2,
        rwpmath::VecFloat & edgeCos3,
        rwpmath::Vector3::InParam v0,
        rwpmath::Vector3::InParam v1,
        rwpmath::Vector3::InParam v2,
        rwpmath::Vector3::InParam v3,
        rwpmath::Vector3 * aV0,
        rwpmath::Vector3 * aV1,
        rwpmath::Vector3 * aV2,
        rwpmath::Vector3 * aV3) const
    {
        // Calculate the triangle normal of the first triangle/half of the quad
        rwpmath::Vector3 triangleNormal = ComputeTriangleNormalFast(
            v0,
            v1,
            v3);

        // Edge 0
        {
            CalculateExtendedEdgeCosine(
                edgeCos0,
                v0,
                v1,
                triangleNormal,
                aV0);
        }

        // Edge 3
        {
            CalculateExtendedEdgeCosine(
                edgeCos3,
                v3,
                v0,
                triangleNormal,
                aV3);
        }

        // Calculate the triangle normal of the second triangle/half of the quad
        triangleNormal = ComputeTriangleNormalFast(
            v3,
            v1,
            v2);

        // Edge 1
        {
            CalculateExtendedEdgeCosine(
                edgeCos1,
                v1,
                v2,
                triangleNormal,
                aV1);
        }

        // Edge 2
        {
            CalculateExtendedEdgeCosine(
                edgeCos2,
                v2,
                v3,
                triangleNormal,
                aV2);
        }
    }

    /*
    Calculates the extended edge cosine of an edge.

    \param edgeCosine the extended edge cosine
    \param edgeStart the start of the edge.
    \param edgeEnd the end of the edge.
    \param triangleNormal the normal of the triangle owning the edge.
    \param neighboringTriangleVertex the vertex of the neighboring triangle
           which shares the edge
    **/
    void CalculateExtendedEdgeCosine(
        rwpmath::VecFloat & edgeCosine,
        rwpmath::Vector3::InParam edgeStart,
        rwpmath::Vector3::InParam edgeEnd,
        rwpmath::Vector3::InParam triangleNormal,
        const rwpmath::Vector3 * neighboringTriangleVertex) const
    {
        // If the triangle has a neighbor along this edge, i.e whether 
        // the edge is shared between two triangles.
        if (NULL != neighboringTriangleVertex)
        {
            // Calculate the normal of the neighboring triangle
            const rwpmath::Vector3 neighborTriangleNormal = ComputeTriangleNormalFast(
                edgeEnd,
                edgeStart,
                *neighboringTriangleVertex);

            // Calculate the edge direction
            rwpmath::Vector3 edgeVector = edgeEnd - edgeStart;

            // Calculate the edge cosine
            edgeCosine = rw::collision::meshbuilder::EdgeCosines::ComputeExtendedEdgeCosine(
                triangleNormal,
                neighborTriangleNormal,
                edgeVector);
        }
        else
        {
            // Calculate the edge cosine and flags of the edge which
            // is not shared between two triangle.
            edgeCosine = CLUSTEREDMESHBUILDER_EDGECOS_OF_UNMATCHED_EDGE;
        }
    }

    /*
    Gets the vertices of the triangles adjacent to a triangle.

    \param adjacentV0 the vertex of the triangle neighboring on the first edge.
    \param adjacentV1 the vertex of the triangle neighboring on the second edge.
    \param adjacentV2 the vertex of the triangle neighboring on the third edge.
    \param xIndex the x index of the triangle.
    \param yIndex the y index of the triangle.
    \param trianglePairIndex the triangle pair index of the triangle.
    **/
    void GetAdjacentVertices(
        rwpmath::Vector3 ** adjacentV0,
        rwpmath::Vector3 ** adjacentV1,
        rwpmath::Vector3 ** adjacentV2,
        const uint32_t xIndex,
        const uint32_t zIndex,
        const uint32_t triangleIndex) const
    {
        if (0 == triangleIndex)
        {
            if (0 == xIndex)
            {
                *adjacentV0 = NULL;
            }
            else
            {
                *adjacentV0 = &m_vertices[((zIndex + 1) * xCount) + xIndex - 1];
            }

            *adjacentV1 = &m_vertices[((zIndex + 1) * xCount) + xIndex + 1];

            if (0 == zIndex)
            {
                *adjacentV2 = NULL;
            }
            else
            {
                *adjacentV2 = &m_vertices[((zIndex - 1) * xCount) + xIndex + 1];
            }
        }
        else
        {
            if (zCount - 2 == zIndex)
            {
                *adjacentV0 = NULL;
            }
            else
            {
                *adjacentV0 = &m_vertices[((zIndex + 2) * xCount) + xIndex];
            }

            if (xCount - 2 == xIndex)
            {
                *adjacentV1 = NULL;
            }
            else
            {
                *adjacentV1 = &m_vertices[((zIndex) * xCount) + xIndex + 2];
            }

            *adjacentV2 = &m_vertices[((zIndex) * xCount) + xIndex];
        }
    }

    /*
    Gets the vertices of the triangles adjacent to a quad.

    \param adjacentV0 the vertex of the triangle neighboring on the first edge.
    \param adjacentV1 the vertex of the triangle neighboring on the second edge.
    \param adjacentV2 the vertex of the triangle neighboring on the third edge.
    \param adjacentV2 the vertex of the triangle neighboring on the fourth edge.
    \param xIndex the x index of the triangle.
    \param yIndex the y index of the triangle.
    **/
    void GetAdjacentVertices(
        rwpmath::Vector3 ** adjacentV0,
        rwpmath::Vector3 ** adjacentV1,
        rwpmath::Vector3 ** adjacentV2,
        rwpmath::Vector3 ** adjacentV3,
        const uint32_t xIndex,
        const uint32_t zIndex) const
    {
        // Adjacent vertex 0
        if (0 == xIndex)
        {
            *adjacentV0 = NULL;
        }
        else
        {
            *adjacentV0 = &m_vertices[((zIndex + 1) * xCount) + xIndex - 1];
        }

        // Adjacent vertex 1
        if (zCount - 2 == zIndex)
        {
            *adjacentV1 = NULL;
        }
        else
        {
            *adjacentV1 = &m_vertices[((zIndex + 2) * xCount) + xIndex];
        }

        // Adjacent vertex 2
        if (xCount - 2 == xIndex)
        {
            *adjacentV2 = NULL;
        }
        else
        {
            *adjacentV2 = &m_vertices[((zIndex) * xCount) + xIndex + 2];
        }

        // Adjacent vertex 3
        if (0 == zIndex)
        {
            *adjacentV3 = NULL;
        }
        else
        {
            *adjacentV3 = &m_vertices[((zIndex - 1) * xCount) + xIndex + 1];
        }
    }

private:

    /// Size of the grid in the x direction
    uint32_t xCount;
    /// Size of the grid in the z direction
    uint32_t zCount;

    /// Internal vertex buffer
    uint8_t* m_vertexBuffer;
    /// Collection of vertices
    rwpmath::Vector3* m_vertices;
};


#endif // #if !defined EA_PLATFORM_PS3_SPU


#endif // #define RWCOLLISION_VOLUMES_EXAMPLES_HEIGHTFIELD_H
