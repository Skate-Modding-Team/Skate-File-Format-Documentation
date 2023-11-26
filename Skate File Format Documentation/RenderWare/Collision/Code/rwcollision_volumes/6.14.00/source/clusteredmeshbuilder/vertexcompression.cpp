// (c) Electronic Arts. All Rights Reserved.


#include <rw/collision/common.h>


#if !defined EA_PLATFORM_PS3_SPU

#include <rw/collision/meshbuilder/vertexcompression.h>


namespace rw
{
namespace collision
{
namespace meshbuilder
{


rwpmath::VecFloat VertexCompression::CalculateMinimum16BitGranularityForRange(
    const rwpmath::VecFloat & xMin,
    const rwpmath::VecFloat & xMax,
    const rwpmath::VecFloat & yMin,
    const rwpmath::VecFloat & yMax,
    const rwpmath::VecFloat & zMin,
    const rwpmath::VecFloat & zMax)
{
    const rwpmath::VecFloat granularityExtent(65535.0f);

    rwpmath::VecFloat minimumGranulatiry = ( xMax - xMin ) / granularityExtent;

    if ( ( yMax - yMin ) / granularityExtent > minimumGranulatiry )
    {
        minimumGranulatiry = ( yMax - yMin ) / granularityExtent;
    }

    if ( ( zMax - zMin ) / granularityExtent > minimumGranulatiry )
    {
        minimumGranulatiry = ( zMax - zMin ) / granularityExtent;
    }

    return  minimumGranulatiry ;
}


void VertexCompression::DetermineCompressionModeAndOffsetForRange(
    uint8_t &compressionMode,
    rw::collision::ClusteredMeshCluster::Vertex32 &offset,
    const int32_t xMin,
    const int32_t xMax,
    const int32_t yMin,
    const int32_t yMax,
    const int32_t zMin,
    const int32_t zMax)
{
    const int32_t granularityTolerance = 65534;

    // validate that cluster fits into 16 bits given our vertexCompressionGranularity
    // Allow a tolerance of one unit at either end for floating point errors - the calculation above
    // can compile up to be slightly different to the later phase of actual compression.
    if (( xMax - xMin < granularityTolerance) && (yMax - yMin < granularityTolerance) && (zMax - zMin < granularityTolerance))
    {
        compressionMode = rw::collision::ClusteredMeshCluster::VERTICES_16BIT_COMPRESSED;
        offset.x = xMin - 1;
        offset.y = yMin - 1;
        offset.z = zMin - 1;
    }
    else
    {
        compressionMode = rw::collision::ClusteredMeshCluster::VERTICES_32BIT_COMPRESSED;
        offset.x = 0;
        offset.y = 0;
        offset.z = 0;
    }
}


} // namespace meshbuilder
} // collision
} // namespace rw


#endif // !defined EA_PLATFORM_PS3_SPU

