// (c) Electronic Arts. All Rights Reserved.

#ifndef PUBLIC_RW_COLLISION_MESHBUILDER_VERTEXCOMPRESSION_H
#define PUBLIC_RW_COLLISION_MESHBUILDER_VERTEXCOMPRESSION_H


#include <rw/collision/common.h>


#if !defined EA_PLATFORM_PS3_SPU

#include <rw/collision/clusteredmeshcluster.h>


namespace rw
{
namespace collision
{
namespace meshbuilder
{


/**
\brief Provides utility methods for deciding compression mode and granularity for cluster vertices.
*/
class VertexCompression
{

public:

    /**
    \brief Determines the granularity needed to represent a collection of vertices
    spanning the given geometric range using 16-bit integer compressed values.

    The geometric range of the vertices is specified as the coordinates of a tight axis-aligned
    bounding box containing all vertices.

    The returned granulatity represents the quantization size of the compression.

    \param xMin Minimum extent of x range.
    \param xMax Maximum extent of x range.
    \param yMin Minimum extent of y range.
    \param yMax Maximum extent of y range.
    \param zMin Minimum extent of z range.
    \param zMax Maximum extent of z range.

    \return Granularity needed to store vertices in the given range in 16bits.
    */
    static rwpmath::VecFloat CalculateMinimum16BitGranularityForRange(
        const rwpmath::VecFloat &xMin,
        const rwpmath::VecFloat &xMax,
        const rwpmath::VecFloat &yMin,
        const rwpmath::VecFloat &yMax,
        const rwpmath::VecFloat &zMin,
        const rwpmath::VecFloat &zMax);

    /**
    \brief Determines the compression mode of a cluster and the geometric offset used if applicable.

    \param compressionMode          Returned compression mode indicated for use with the range.
    \param offset                   Returned base offset translation to be subtracted from the vertex positions.
    \param xMin                     Minimum integer extent of x range.
    \param xMax                     Maximum integer extent of x range.
    \param yMin                     Minimum integer extent of y range.
    \param yMax                     Maximum integer extent of y range.
    \param zMin                     Minimum integer extent of z range.
    \param zMax                     Maximum integer extent of z range.
    */
    static void DetermineCompressionModeAndOffsetForRange(
        uint8_t &compressionMode,
        rw::collision::ClusteredMeshCluster::Vertex32 &offset,
        const int32_t xMin,
        const int32_t xMax,
        const int32_t yMin,
        const int32_t yMax,
        const int32_t zMin,
        const int32_t zMax);
};


} // namespace meshbuilder
} // namespace collision
} // namespace rw


#endif // !defined EA_PLATFORM_PS3_SPU

#endif // defined PUBLIC_RW_COLLISION_MESHBUILDER_VERTEXCOMPRESSION_H
