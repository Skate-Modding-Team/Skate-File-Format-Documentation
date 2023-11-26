// (c) Electronic Arts. All Rights Reserved.
#ifndef PUBLIC_RW_COLLISION_DETAIL_FPU_CLUSTEREDMESHCLUSTER_H
#define PUBLIC_RW_COLLISION_DETAIL_FPU_CLUSTEREDMESHCLUSTER_H

/*************************************************************************************************************

File: rwcclusteredmesh.hpp

Purpose: Compressed aggregate of triangles and quads with KDTree spatial map.

*/

#include "rw/collision/clusteredmeshcluster.h"


namespace rw
{
namespace collision
{
namespace detail
{
namespace fpu
{
    // Forward declare ClusteredMesh in the rw::collision namespace so
    // that we can use it in the EA_SERIALIZATION_CLASS_* macros
    class ClusteredMeshCluster;

} // namespace fpu
} // namespace detail
} // namespace collision
} // namespace rw

// We need to specify the class serialization version prior to the class definition
// due to a problem with ps2 gcc.
EA_SERIALIZATION_CLASS_VERSION(rw::collision::detail::fpu::ClusteredMeshCluster, 5)
// These macro provide the type name used in text-based archives' serialization.
EA_SERIALIZATION_CLASS_NAME(rw::collision::detail::fpu::ClusteredMeshCluster, "rw::collision::ClusteredMeshCluster")

namespace rw
{
namespace collision
{
namespace detail
{
namespace fpu
{

/**
\brief This class mimics the layout of rw::collision::ClusteredMeshCluster when built using fpu
rwmath.

This class contains a serialize method and is not intended to be serialized alone. It it used by
higher level classes, such as the TriangleClusterProcedural.

Changes to data members in rw::collision::ClusteredMeshCluster or its serialization function
should be mirrored in this class.
*/
class ClusteredMeshCluster
{
public:

    ClusteredMeshCluster()
    {
    };

    template <class Archive>
    void Serialize(Archive &ar, uint32_t version)
    {
        // Serialize the counts, sizes and starts
        ar & EA_SERIALIZATION_NAMED_VALUE(unitCount);
        ar & EA_SERIALIZATION_NAMED_VALUE(unitDataSize);
        ar & EA_SERIALIZATION_NAMED_VALUE(unitDataStart);
        ar & EA_SERIALIZATION_NAMED_VALUE(normalStart);
        ar & EA_SERIALIZATION_NAMED_VALUE(totalSize);
        ar & EA_SERIALIZATION_NAMED_VALUE(compressionMode);
        ar & EA_SERIALIZATION_NAMED_VALUE(vertexCount);
        ar & EA_SERIALIZATION_NAMED_VALUE(normalCount);

        // Serialize the vertices
        if ( compressionMode == rw::collision::ClusteredMeshCluster::VERTICES_16BIT_COMPRESSED )
        {
            // Access the vertex array as a collection of rwpmath::Vector3
            uint32_t* vertexArrayHeader = reinterpret_cast<uint32_t*>(vertexArray);
            // Serialize the vertex array header
            ar & EA_SERIALIZATION_NAMED_STATIC_ARRAY(vertexArrayHeader, 3u);

            // Access the vertex array as a collection of uint16_t and offset the pointer to the end of the header
            uint16_t* vertexArrayPointer = reinterpret_cast<uint16_t*>(vertexArray) + 3 * 2;
            // Serialize the vertices
            ar & EA_SERIALIZATION_NAMED_STATIC_ARRAY(vertexArrayPointer, vertexCount * 3u);
        }
        else if ( compressionMode == rw::collision::ClusteredMeshCluster::VERTICES_32BIT_COMPRESSED )
        {
            // Access the vertex array as a collection of int32_t
            int32_t* vertexArrayPointer = reinterpret_cast<int32_t*>(vertexArray);
            // Serialize the vertices
            ar & EA_SERIALIZATION_NAMED_STATIC_ARRAY(vertexArrayPointer, vertexCount * 3u);
        }
        else
        {
            // If the vertices are uncompressed
            if (version == 1)
            {
                // Using fpu math it is not valid to serialize the normals with the vertices
                // as there will be padding between the two arrays.
                uint32_t size = uint32_t(vertexCount + normalCount);
                rw::math::fpu::Vector3* vertexAndNormalArray = reinterpret_cast<rw::math::fpu::Vector3*>(vertexArray);
                // Serialize the vertices and normals
                ar & EA_SERIALIZATION_NAMED_STATIC_ARRAY(vertexAndNormalArray, size);
            }
            else
            {
                // Access the vertex array as a collection of rw::math::fpu::Vector3
                rw::math::fpu::Vector3* vertexArrayPointer = reinterpret_cast<rw::math::fpu::Vector3*>(vertexArray);
                // Serialize the vertices
                ar & EA_SERIALIZATION_NAMED_STATIC_ARRAY(vertexArrayPointer, vertexCount);
            }
        }

        // Serialize the normals
        if ((compressionMode != rw::collision::ClusteredMeshCluster::VERTICES_UNCOMPRESSED) || (version > 1))
        {
            // normalStart is a quad-word offset
            rw::math::fpu::Vector3* normalArray =
                reinterpret_cast<rw::math::fpu::Vector3*>(reinterpret_cast<uintptr_t>(vertexArray) + normalStart * 16);
            ar & EA_SERIALIZATION_NAMED_STATIC_ARRAY(normalArray, normalCount);
        }

        // Serialize Unit data
        // unitDataStart is a quad-word offset
        uint8_t* unitData = reinterpret_cast<uint8_t*>(vertexArray) + unitDataStart * 16;
        ar & EA_SERIALIZATION_NAMED_STATIC_ARRAY(unitData, unitDataSize);
    }

public:

    //  All the data in this class is public so that the MeshWorkingData class can access it.
    //  (Maybe it should just be a friend?)

    uint16_t unitCount;                         ///< The number of units in this cluster
    uint16_t unitDataSize;                      ///< The size of the unit data
    uint16_t unitDataStart;                     ///< The quad word offset to the beginning of the unit data
    uint16_t normalStart;                       ///< The quad word offset to the beginning of the normal data
    uint16_t totalSize;                         ///< The total size of the entire cluster
    uint8_t vertexCount;                        ///< The total number of vertices
    uint8_t normalCount;                        ///< The total number of normal vectors
    uint8_t compressionMode;                    ///< The compression mode
    uint8_t padding[3];                         ///< force 16 byte alignment for the vertex array
    rw::math::fpu::Vector3 vertexArray[1];      ///< The first of the array of vertices, the rest are immediately after this class
};


} // namespace fpu
} // namespace detail
} // namespace collision
} // namespace rw

#endif //PUBLIC_RW_COLLISION_DETAIL_FPU_CLUSTEREDMESHCLUSTER_H
