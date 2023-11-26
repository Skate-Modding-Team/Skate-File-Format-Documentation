// (c) Electronic Arts. All Rights Reserved.

#ifndef RWCOLLISION_VOLUMES_EXAMPLES_HEIGHTFIELD_H
#define RWCOLLISION_VOLUMES_EXAMPLES_HEIGHTFIELD_H


#include <rw/collision/common.h>

#if !defined EA_PLATFORM_PS3_SPU



// Utility methods
static inline void AlignPointer(uint8_t *&ptr, const uint8_t alignment)
{
    ptr = (uint8_t *)(((uintptr_t)ptr + ((uintptr_t)alignment - 1)) & ~((uintptr_t)alignment - 1));
}


// HeightField class
class HeightField
{
public:

    /*
    Constructor

    Allocates internal data structures and initializes the height values.
    **/
    HeightField(const uint32_t x,
                const uint32_t z)
        : xCount(x)
        , zCount(z)
        , width(rwpmath::GetVecFloat_One())
        , m_pointBuffer(0)
        , m_pointBufferSize(0)
        , m_points(0)
    {
        // Create the point buffer
        m_pointBufferSize = sizeof(rwpmath::VecFloat) * ((xCount * zCount) + 1);
        m_pointBuffer = new uint8_t[sizeof(rwpmath::VecFloat) * ((xCount * zCount) + 1)];

        // Align points
        uint8_t * ptr = m_pointBuffer;
        AlignPointer(ptr, RW_MATH_VECTOR3_ALIGNMENT);
        m_points = reinterpret_cast<rwpmath::VecFloat*>(ptr);

        // Initialize the points
        for (uint32_t pointIndex = 0 ; pointIndex < xCount * zCount ; ++pointIndex)
        {
            uint32_t num = rw::math::Random() % 5;
            m_points[pointIndex] = (rwpmath::VecFloat(static_cast<float>(num)));
        }
    }

    /*
    Destructor

    Deallocates internal data structures.
    **/
    ~HeightField()
    {
        delete [] m_pointBuffer;
    }


    /*
    Gets the vertices of the quad indicated by the xIndex and zIndex.

    param v0 vertex 0
    param v1 vertex 1
    param v2 vertex 2
    param v2 vertex 3
    param xIndex the x index into the height field grid. This value should be no more than
    the xCount - 2.
    param zIndex the z index into the height field gird. This value should be no more than
    the zCount - 2.
    **/
    void GetQuadVertices(rwpmath::Vector3 & v0,
                         rwpmath::Vector3 & v1,
                         rwpmath::Vector3 & v2,
                         rwpmath::Vector3 & v3,
                         const uint32_t xIndex,
                         const uint32_t zIndex)
    {
        GetVertex(v0, xIndex, zIndex);
        GetVertex(v1, xIndex, zIndex + 1);
        GetVertex(v2, xIndex + 1, zIndex);
        GetVertex(v3, xIndex + 1, zIndex + 1);
    }

    /*
    Gets the vertices of the triangles surround the quad indicated by the xIndex and zIndex.

    If the surrounding vertices do not exist (the quad is on a boundary) then the vertex pointer
    is set to null.

    param v0 vertex 0
    param v1 vertex 1
    param v2 vertex 2
    param v2 vertex 3
    param xIndex the x index into the height field grid. This value should be no more than
    the xCount - 2.
    param zIndex the z index into the height field gird. This value should be no more than
    the zCount - 2.
    **/
    void GetSurroundingTriangleVertices(rwpmath::Vector3 * v0,
                                        rwpmath::Vector3 * v1,
                                        rwpmath::Vector3 * v2,
                                        rwpmath::Vector3 * v3,
                                        const uint32_t xIndex,
                                        const uint32_t zIndex)
    {
        if (0 == zIndex)
        {
            v0 = 0;
        }
        else
        {
            GetVertex(*v0, xIndex + 1, zIndex - 1);
        }

        if (0 == xIndex)
        {
            v1 = 0;
        }
        else
        {
            GetVertex(*v1, xIndex - 1, zIndex + 1);
        }

        if (zCount - 1 <= zCount)
        {
            v2 = 0;
        }
        else
        {
            GetVertex(*v2, xIndex, zIndex + 2);
        }

        if (xCount - 1 <= xCount)
        {
            v3 = 0;
        }
        else
        {
            GetVertex(*v3, xIndex + 2, zCount);
        }
    }


private:

    // Private methods

    /*
    Gets the height field point indicated by the xIndex and zIndex.

    param xIndex the x index into the height field grid. This value should be no more than
    the xCount - 1.
    param zIndex the z index into the height field gird. This value should be no more than
    the zCount - 1.
    **/
    rwpmath::VecFloat* GetPoint(uint32_t xIndex,
                                uint32_t zIndex)
    {
        return &m_points[(zIndex * xCount) + xIndex];
    }

    /*
    Gets the height field vertex indicated by the xIndex and zIndex.

    param xIndex the x index into the height field grid. This value should be no more than
    the xCount - 1.
    param zIndex the z index into the height field gird. This value should be no more than
    the zCount - 1.
    **/
    void GetVertex(rwpmath::Vector3 & vertex,
                   const uint32_t xIndex,
                   const uint32_t zIndex)
    {
        vertex = rwpmath::Vector3(rwpmath::VecFloat(static_cast<float>(xIndex)) * width,
            *GetPoint(xIndex, zIndex),
            rwpmath::VecFloat(static_cast<float>(zIndex)) * width);
    }


private:

    // Private members

    // Size of the grid in the x direction
    uint32_t xCount;
    // Size of the grid in the z direction
    uint32_t zCount;
    // Stride of each point
    rwpmath::VecFloat width;

    // Internal point buffer
    uint8_t *m_pointBuffer;
    // Point buffer size
    uint32_t m_pointBufferSize;
    // Collection of points
    rwpmath::VecFloat *m_points;
};


#endif // #if !defined EA_PLATFORM_PS3_SPU


#endif // #define RWCOLLISION_VOLUMES_EXAMPLES_HEIGHTFIELD_H
