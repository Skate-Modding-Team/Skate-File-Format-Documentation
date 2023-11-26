// (c) Electronic Arts. All Rights Reserved.
#ifndef RWCOLLISION_VOLUMES_EXAMPLES_QUADHEIGHTFIELD_H
#define RWCOLLISION_VOLUMES_EXAMPLES_QUADHEIGHTFIELD_H


#include <rw/collision/common.h>

#if !defined EA_PLATFORM_PS3_SPU


#include <rw/collision/meshbuilder/common.h>
#include <rw/collision/meshbuilder/edgecosines.h>
#include <eaphysics/sizeandalignment.h>


// Using directives
using namespace rw::collision;


/*
QuadHeightField Class

The purpose of this class is to simply represent a collection of quad and vertexdata

Internally a collection of vertices laid out in a grid fashion, which can be
indexed by a x and z coordinate.

The methods of interest are 

HeightField() - The constructor takes 2 parameters which describe the dimensions
of the height field grid.

GetQuadVertexIndices() - Gets the vertex indices of a quad.

GetQuadVertices() - Gets the vertices of a quad.

GetQuadNeighborIndices() - Gets the indices of triangles neighboring a quad.

**/
class QuadHeightField
{
public:

    /*
    Constructor

    Allocates internal data structures and initializes the height values.
    **/
    QuadHeightField(const uint32_t x,
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
        ptr = EA::Physics::MemAlign<uint8_t>(ptr, RW_MATH_VECTOR3_ALIGNMENT);
        m_vertices = reinterpret_cast<rwpmath::Vector3*>(ptr);

        uint32_t vertexIndex = 0u;
        // Initialize the vertices
        for (uint32_t zIndex = 0 ; zIndex < zCount ; ++zIndex)
        {
            for (uint32_t xIndex = 0 ; xIndex < xCount ; ++xIndex)
            {
                uint32_t height = rw::math::Random() % 50;
                m_vertices[vertexIndex++] = rwpmath::Vector3(
                    static_cast<float>(xIndex),
                    static_cast<float>(height) / 50,
                    static_cast<float>(zIndex));
            }
        }
    }

    /*
    Destructor

    Deallocates internal data structures.
    **/
    ~QuadHeightField()
    {
        delete [] m_vertexBuffer;
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
        v0 = (zIndex * xCount) + xIndex ;
        v1 = ((zIndex + 1u) * xCount) + xIndex;
        v2 = (zIndex * xCount) + xIndex + 1u;
        v3 = ((zIndex + 1u) * xCount) + xIndex + 1u;
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

    /*
    Gets the vertices of the triangles adjacent to a quad.

    \param adjacentV0 the vertex of the triangle neighboring on the first edge.
    \param adjacentV1 the vertex of the triangle neighboring on the second edge.
    \param adjacentV2 the vertex of the triangle neighboring on the third edge.
    \param adjacentV2 the vertex of the triangle neighboring on the fourth edge.
    \param xIndex the x index of the quad.
    \param yIndex the y index of the quad.
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
        if (0 == zIndex)
        {
            *adjacentV2 = NULL;
        }
        else
        {
            *adjacentV2 = &m_vertices[((zIndex - 1) * xCount) + xIndex + 1];
        }

        // Adjacent vertex 3
        if (xCount - 2 == xIndex)
        {
            *adjacentV3 = NULL;
        }
        else
        {
            *adjacentV3 = &m_vertices[((zIndex) * xCount) + xIndex + 2];
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


#endif // #define RWCOLLISION_VOLUMES_EXAMPLES_QUADHEIGHTFIELD_H
